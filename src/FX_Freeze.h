#ifndef FX_FREEZE_H
#define FX_FREEZE_H

#include "DSPEngine.h"

class FX_Freeze {
public:
    static inline void process(bool active, float inSample, float att, float rel, float srScale, float apfParam, float invFreqLen, int activeLen, int freezeLen, const float* hannLUT, int16_t* freezeBuffer, float* apf1Buffer, float* apf2Buffer, int& writeIdx, int& playCounter, int startIdx, volatile float& frzRamp, float& fzOutNode, int& apf1Idx, int& apf2Idx) __attribute__((always_inline)) {
        int safeFreezeLen = (freezeLen > 0) ? freezeLen : 96000;
        
        if(!active) {
            freezeBuffer[writeIdx] = (int16_t)(__builtin_fmaxf(-1.0f, __builtin_fminf(inSample, 1.0f)) * 32767.0f);
            if(++writeIdx >= safeFreezeLen) writeIdx = 0;
        }
        if(frzRamp > 0.0f || active) {
            frzRamp = active ? __builtin_fminf(1.0f, __builtin_fmaf(att, srScale, frzRamp)) : __builtin_fmaxf(0.0f, __builtin_fmaf(-rel, srScale, frzRamp));
        }
        if(frzRamp > 0.0f) {
            float phaseRead = (float)playCounter * invFreqLen;
            float phase2 = phaseRead + 0.5f; if(phase2 >= 1.0f) phase2 -= 1.0f;
            
            // Bounds-safe modulo indexing across dynamic sample rate shifts
            int idx1 = (startIdx + playCounter) % safeFreezeLen;
            int aLen = (activeLen >= 64 && activeLen <= safeFreezeLen) ? activeLen : safeFreezeLen;
            int c2 = (playCounter + (aLen / 2)) % aLen;
            int idx2 = (startIdx + c2) % safeFreezeLen;
            
            int lut1 = (int)(phaseRead * 4095.0f) & 4095;
            int lut2 = (int)(phase2 * 4095.0f) & 4095;
            
            float rFrz = __builtin_fmaf((float)freezeBuffer[idx1] * 3.0517578125e-5f, hannLUT[lut1], (float)freezeBuffer[idx2] * 3.0517578125e-5f * hannLUT[lut2]);
            
            float d1 = apf1Buffer[apf1Idx];
            float n1 = __builtin_fmaf(apfParam, d1, rFrz + 1e-9f);
            float a1 = __builtin_fmaf(-apfParam, rFrz, d1);
            apf1Buffer[apf1Idx] = n1; if(++apf1Idx >= 1009) apf1Idx = 0;
            
            float d2 = apf2Buffer[apf2Idx];
            float n2 = __builtin_fmaf(apfParam, d2, a1 + 1e-9f);
            float a2 = __builtin_fmaf(-apfParam, a1, d2);
            apf2Buffer[apf2Idx] = n2; if(++apf2Idx >= 863) apf2Idx = 0;
            
            fzOutNode = a2 * frzRamp;
            if(++playCounter >= aLen) playCounter = 0;
        } else {
            fzOutNode = 0.0f;
        }
    }
};

#endif