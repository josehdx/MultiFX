#ifndef FX_FEEDBACK_H
#define FX_FEEDBACK_H

#include "DSPEngine.h"

class FX_Feedback {
public:
    static inline void process(bool active, float srScale, float drv, float fbHpfCoeff, float fbLpfCoeff, float fbLpfRetain, float fbRatio, float currentPitch, const float* lfoLUT, volatile float& localFbPhase, float phaseInc, uint32_t& wowRng, float& wowState, float& spd4, float& spd5, float w4, float w5, float env, volatile float& fbRamp, float frzOut, bool frzActive, float localFrzRamp, float& fbHpfState, float& feedbackFilter, int16_t* fbDelayBuffer, int delaySamples, int& fbDelayWriteIdx, float& fbOutNode) __attribute__((always_inline)) {
        if(active || fbRamp > 0.0f) {
            localFbPhase += phaseInc; if(localFbPhase >= 1024.0f) localFbPhase -= 1024.0f;
            float lfoVal = DSPEngine::getLfoInterpolated(localFbPhase, lfoLUT);
            spd4 = lfoVal; spd5 = currentPitch * fbRatio * lfoVal;
            
            wowRng = wowRng * 1664525U + 1013904223U; 
            float rawNoise = ((float)(wowRng & 0xFFFF) * 0.0000305185f) - 1.0f; 
            wowState = DSPEngine::AntiDenormal(__builtin_fmaf(rawNoise - wowState, 0.0005f * srScale, wowState)); 
            float wowMod = 1.0f + (wowState * 0.0015f); 
            spd4 *= wowMod; spd5 *= wowMod;
            
            if(active) fbRamp = (env > 0.005f) ? __builtin_fminf(1.0f, __builtin_fmaf(0.000011f, srScale, fbRamp)) : __builtin_fmaxf(0.0f, __builtin_fmaf(-0.0005f, srScale, fbRamp)); 
            else fbRamp = __builtin_fmaxf(0.0f, __builtin_fmaf(-0.0001f, srScale, fbRamp));
            
            float mixV = __builtin_fmaxf(0.0f, __builtin_fminf((fbRamp - 0.1f) * 2.0f, 1.0f));
            float feedInput = (frzActive && localFrzRamp > 0.0f) ? frzOut : __builtin_fmaf(w4, (1.0f - mixV), __builtin_fmaf(w5, mixV, fbOutNode * 0.95f)); 
            
            fbHpfState = DSPEngine::AntiDenormal(__builtin_fmaf(fbHpfCoeff, (feedInput - fbHpfState), 1e-9f)); 
            float rawDrive = (feedInput - fbHpfState) * drv;
            float boundedDrive = __builtin_fmaxf(-1.5f, __builtin_fminf(rawDrive, 1.5f));
            float gainDrive = boundedDrive * (1.0f - (0.15f * boundedDrive * boundedDrive)); 
            feedbackFilter = DSPEngine::AntiDenormal(__builtin_fmaf(gainDrive, fbLpfCoeff, __builtin_fmaf(feedbackFilter, fbLpfRetain, 1e-9f))); 
            float satFb = feedbackFilter * (fbRamp * fbRamp * fbRamp) * 0.85f; 
            
            fbDelayBuffer[fbDelayWriteIdx] = (int16_t)(__builtin_fmaxf(-1.0f, __builtin_fminf(satFb, 1.0f)) * 32767.0f);
            int fbReadIdx = (fbDelayWriteIdx - delaySamples + 8192) & 0x1FFF; 
            fbOutNode = DSPEngine::AntiDenormal((float)fbDelayBuffer[fbReadIdx] * 3.0517578125e-5f); 
            fbDelayWriteIdx = (fbDelayWriteIdx + 1) & 0x1FFF;
        } else {
            spd4 = 1.0f; spd5 = 1.0f;
            fbDelayBuffer[fbDelayWriteIdx] = 0; 
            fbOutNode = 0.0f; 
            fbDelayWriteIdx = (fbDelayWriteIdx + 1) & 0x1FFF;
        }
    }
};

#endif