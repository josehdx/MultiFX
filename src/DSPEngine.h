#ifndef DSP_ENGINE_H
#define DSP_ENGINE_H

#include <math.h>
#include <stdint.h>
#include "dsps_mul.h"

#define SRAM_PITCH_BUF_SIZE 8192
#define SRAM_PITCH_BUF_MASK 0x1FFF

class DSPEngine {
public:
    static inline float __attribute__((always_inline)) __attribute__((optimize("O3"))) AntiDenormal(float value) {
        union { float f; uint32_t i; } u = { .f = value };
        if (__builtin_expect((u.i & 0x7F800000) == 0, 0)) return 0.0f;
        return value;
    }

    static inline float __attribute__((always_inline)) getLfoInterpolated(float phase, const float* lfoLUT) {
        int idx = (int)phase & 1023;
        float frac = phase - (float)idx;
        float v1 = lfoLUT[idx];
        float v2 = lfoLUT[(idx + 1) & 1023];
        return __builtin_fmaf(v2 - v1, frac, v1);
    }

    static inline float __attribute__((hot)) __attribute__((always_inline)) __attribute__((optimize("O3"))) processSincTap(uint32_t tapPhase, const int16_t* sramBuffer, int currentSramWriteIdx, uint32_t windowMask, uint32_t hannIntMult, const float* hannLUT) {
        int T = (tapPhase >> 16) & windowMask; 
        float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
        int effTap = T + 3;
        int idx_m1 = (currentSramWriteIdx - effTap + 2 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        int idx_0  = (currentSramWriteIdx - effTap + 3 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        int idx_1  = (currentSramWriteIdx - effTap + 4 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        int idx_2  = (currentSramWriteIdx - effTap + 5 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        float s_m1 = (float)sramBuffer[idx_m1] * 3.0517578125e-5f;
        float s_0  = (float)sramBuffer[idx_0]  * 3.0517578125e-5f;
        float s_1  = (float)sramBuffer[idx_1]  * 3.0517578125e-5f;
        float s_2  = (float)sramBuffer[idx_2]  * 3.0517578125e-5f;
        float fm1 = frac + 1.0f; float f0  = frac; float f1  = 1.0f - frac; float f2  = 2.0f - frac;
        float w_m1 = __builtin_fmaf(-0.16666667f * f0, f1 * f2, 0.0f);
        float w_0  = __builtin_fmaf(0.5f * fm1, f1 * f2, 0.0f);
        float w_1  = __builtin_fmaf(0.5f * fm1, f0 * f2, 0.0f);
        float w_2  = __builtin_fmaf(-0.16666667f * fm1, f0 * f1, 0.0f);
        float interpSample = __builtin_fmaf(s_m1, w_m1, __builtin_fmaf(s_0, w_0, __builtin_fmaf(s_1, w_1, s_2 * w_2)));
        int lutIdx = ((uint32_t)(T * hannIntMult) >> 16) & 4095; 
        return AntiDenormal(interpSample * hannLUT[lutIdx]);
    }

    static inline float __attribute__((hot)) __attribute__((always_inline)) __attribute__((optimize("O3"))) processHermiteTap(uint32_t tapPhase, const int16_t* sramBuffer, int currentSramWriteIdx, uint32_t windowMask, uint32_t hannIntMult, const float* hannLUT) {
        int T = (tapPhase >> 16) & windowMask; 
        float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
        int idx_m1 = (currentSramWriteIdx - T + 1 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        int idx_0  = (currentSramWriteIdx - T + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        int idx_1  = (currentSramWriteIdx - T - 1 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        int idx_2  = (currentSramWriteIdx - T - 2 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        float y_m1 = (float)sramBuffer[idx_m1] * 3.0517578125e-5f;
        float y0   = (float)sramBuffer[idx_0]  * 3.0517578125e-5f;
        float y1   = (float)sramBuffer[idx_1]  * 3.0517578125e-5f;
        float y2   = (float)sramBuffer[idx_2]  * 3.0517578125e-5f;
        float c0 = y0; float c1 = 0.5f * (y1 - y_m1);
        float c2 = y_m1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
        float c3 = 0.5f * (y2 - y_m1) + 1.5f * (y0 - y1);
        float interpSample = ((c3 * frac + c2) * frac + c1) * frac + c0;
        int lutIdx = ((uint32_t)(T * hannIntMult) >> 16) & 4095; 
        return AntiDenormal(interpSample * hannLUT[lutIdx]);
    }

    static void __attribute__((optimize("O3"))) processInput(
        int framesRead, int32_t* i2s_in_block, float normFactor, float dc_alpha, 
        float envRetain, float envAttack, float p_sw_thr, float p_sw_att, 
        float p_sw_rel, float srScale, bool swellActive, float localVolGain, 
        float vol_alpha, float& input_dc_offset, float& inputEnvelope, 
        float& localSwellGain, float& smoothedVolGain, float& currentPitch, 
        float targetPitch, float* envBuf, float* masterGainBuf, float* inBuf, float* fzOutBuf) {
        
        if (__builtin_expect(isnan(input_dc_offset) || isinf(input_dc_offset), 0)) input_dc_offset = 0.0f;
        if (__builtin_expect(isnan(inputEnvelope) || isinf(inputEnvelope), 0)) inputEnvelope = 0.0f;
        float pitchInc = 0.0f;
        if (__builtin_expect(fabsf(targetPitch - currentPitch) < 1e-6f, 1)) { currentPitch = targetPitch; } 
        else { pitchInc = (targetPitch - currentPitch) * (1.0f / (float)framesRead); }
        float localPitch = currentPitch, localDC = input_dc_offset, localEnv = inputEnvelope;
        float localSwell = localSwellGain, localSmVol = smoothedVolGain;
        
        #pragma GCC unroll 4
        for(int i = 0; i < framesRead; i++) {
            localPitch += pitchInc;
            float raw_in = ((float)(i2s_in_block[i * 2] & 0xFFFFFF00) * normFactor);
            localDC = AntiDenormal(__builtin_fmaf(raw_in, dc_alpha, localDC * (1.0f - dc_alpha)));
            float inSample = raw_in - localDC;
            localEnv = AntiDenormal(__builtin_fmaf(fabsf(inSample), envAttack, __builtin_fmaf(localEnv, envRetain, 1e-9f)));
            if(__builtin_expect(swellActive, 0)) {
                localSwell = (localEnv > p_sw_thr) ? __builtin_fminf(1.0f, __builtin_fmaf(p_sw_att, srScale, localSwell)) : __builtin_fmaxf(0.0f, __builtin_fmaf(-p_sw_rel, srScale, localSwell));
            } else { localSwell = __builtin_fminf(1.0f, __builtin_fmaf(0.005f, srScale, localSwell)); }
            localSmVol = __builtin_fmaf(localVolGain, vol_alpha, __builtin_fmaf(localSmVol, (1.0f - vol_alpha), 1e-9f));
            envBuf[i] = localEnv; masterGainBuf[i] = localSwell * localSmVol;
            inBuf[i] = inSample; fzOutBuf[i] = 0.0f;
        }
        currentPitch = localPitch; input_dc_offset = localDC; inputEnvelope = localEnv;
        localSwellGain = localSwell; smoothedVolGain = localSmVol;
    }

    static void __attribute__((optimize("O3"))) mixdownAndOutput(
        int framesRead, bool activeGroup, float localFrzRamp, float localFbRamp,
        float g_whammy, float g_dry, float g_base, float g_w2, float g_w3,
        float g_pad, float g_frz, float g_fb, float* padFilterBuf, float* dryBuf, 
        float* w1Buf, float* w2Buf, float* w3Buf, float* fzOutBuf, float* fbOutBuf, 
        float* sMixBuf, float* masterGainBuf, float* inBuf, int32_t* i2s_out_block,
        float& peakInputVal, float& peakOutputVal) {
        
        #pragma GCC ivdep
        #pragma GCC unroll 4
        for(int i = 0; i < framesRead; i++) {
            float sMix = 1e-9f;
            if(__builtin_expect(!activeGroup && localFrzRamp <= 0.0f && localFbRamp <= 0.0f && padFilterBuf[i] <= 0.001f, 0)) {
                 sMix = dryBuf[i];
             } else {
                 sMix = __builtin_fmaf(__builtin_fmaf(w1Buf[i], g_whammy, dryBuf[i] * g_dry), g_base, sMix);
                 sMix = __builtin_fmaf(w2Buf[i], g_w2, sMix);
                 sMix = __builtin_fmaf(w3Buf[i], g_w3, sMix);
                 sMix = __builtin_fmaf(padFilterBuf[i], g_pad, sMix);
                 sMix = __builtin_fmaf(fzOutBuf[i], g_frz, sMix);
                 sMix = __builtin_fmaf(fbOutBuf[i], g_fb, sMix);
                 float x = __builtin_fmaxf(-1.15f, __builtin_fminf(sMix * 0.85f, 1.15f)); 
                 sMix = __builtin_fmaf(1.5f, x, -0.5f * x * x * x);
             }
            sMixBuf[i] = sMix;
        }
        dsps_mul_f32(sMixBuf, masterGainBuf, sMixBuf, framesRead, 1, 1, 1);
        
        float localPeakIn = peakInputVal, localPeakOut = peakOutputVal;
        #pragma GCC unroll 4
        for(int i = 0; i < framesRead; i++) {
            float abs_in = fabsf(inBuf[i]), abs_out = fabsf(sMixBuf[i]);
            if(__builtin_expect(abs_in > localPeakIn, 0)) localPeakIn = abs_in;
            if(__builtin_expect(abs_out > localPeakOut, 0)) localPeakOut = abs_out;
            int32_t finalOut = (int32_t)(__builtin_fmaxf(-0.999f, __builtin_fminf(sMixBuf[i], 0.999f)) * 2147483520.0f) & 0xFFFFFF00;
            i2s_out_block[i * 2] = finalOut; i2s_out_block[i * 2 + 1] = finalOut;
        }
        peakInputVal = localPeakIn; peakOutputVal = localPeakOut;
    }
};

#endif