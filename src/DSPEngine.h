#ifndef DSP_ENGINE_H
#define DSP_ENGINE_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_biquad.h"
#include "FX_Swell.h" // Injected Decoupled Component

#define SRAM_PITCH_BUF_SIZE 8192
#define SRAM_PITCH_BUF_MASK 0x1FFF

struct __attribute__((aligned(64))) VectorBiquadS3 {
    __attribute__((aligned(64))) float coeffs[5] = {0.0f}; 
    __attribute__((aligned(64))) float delay_state[2] = {0.0f, 0.0f};

    void setLPF(float cutoffHz, float sampleRate, float Q = 0.7071f) {
        float w0 = 2.0f * M_PI * cutoffHz / sampleRate;
        float alpha = sinf(w0) / (2.0f * Q);
        float cosw0 = cosf(w0);
        float a0 = 1.0f + alpha;

        coeffs[0] = ((1.0f - cosw0) / 2.0f) / a0; 
        coeffs[1] = (1.0f - cosw0) / a0;          
        coeffs[2] = ((1.0f - cosw0) / 2.0f) / a0; 
        coeffs[3] = (-2.0f * cosw0) / a0;         
        coeffs[4] = (1.0f - alpha) / a0;          
    }

    void inline process(const float* input, float* output, int len) {
        dsps_biquad_f32_aes3(input, output, len, coeffs, delay_state);
    }

    void reset() { delay_state[0] = 0.0f; delay_state[1] = 0.0f; }
};

class DSPEngine {
public:
    static inline float __attribute__((always_inline)) __attribute__((optimize("Ofast"))) AntiDenormal(float value) {
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

    static inline float __attribute__((hot)) __attribute__((always_inline)) __attribute__((optimize("Ofast"))) processPitchTap(
        uint32_t tapPhase, const int16_t* sramBuffer, int currentSramWriteIdx, 
        uint32_t windowMask, uint32_t hannIntMult, const float* hannLUT, const float* pitchSincLUT) {
        
        int T = (tapPhase >> 16) & windowMask; 
        uint32_t fracInt = tapPhase & 0xFFFF;
        int lutIdxFrac = fracInt >> 6; 
        int startIdx = (currentSramWriteIdx - T - 2 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
        float y_m1, y0, y1, y2;

        if (__builtin_expect(startIdx <= SRAM_PITCH_BUF_MASK - 3, 1)) {
            y2   = (float)sramBuffer[startIdx]     * 3.0517578125e-5f;
            y1   = (float)sramBuffer[startIdx + 1] * 3.0517578125e-5f;
            y0   = (float)sramBuffer[startIdx + 2] * 3.0517578125e-5f;
            y_m1 = (float)sramBuffer[startIdx + 3] * 3.0517578125e-5f;
        } else {
            y2   = (float)sramBuffer[(startIdx + 0) & SRAM_PITCH_BUF_MASK] * 3.0517578125e-5f;
            y1   = (float)sramBuffer[(startIdx + 1) & SRAM_PITCH_BUF_MASK] * 3.0517578125e-5f;
            y0   = (float)sramBuffer[(startIdx + 2) & SRAM_PITCH_BUF_MASK] * 3.0517578125e-5f;
            y_m1 = (float)sramBuffer[(startIdx + 3) & SRAM_PITCH_BUF_MASK] * 3.0517578125e-5f;
        }

        int baseLut = lutIdxFrac << 2; 
        float w_m1 = pitchSincLUT[baseLut];
        float w_0  = pitchSincLUT[baseLut + 1];
        float w_1  = pitchSincLUT[baseLut + 2];
        float w_2  = pitchSincLUT[baseLut + 3];

        float interpSample = __builtin_fmaf(y_m1, w_m1, __builtin_fmaf(y0, w_0, __builtin_fmaf(y1, w_1, y2 * w_2)));
        int lutIdx = ((uint32_t)(T * hannIntMult) >> 16) & 4095; 
        return AntiDenormal(interpSample * hannLUT[lutIdx]);
    }

    static void __attribute__((optimize("Ofast"))) processInput(
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
            
            FX_Swell::process(swellActive, localEnv, p_sw_thr, p_sw_att, p_sw_rel, srScale, localSwell);
            
            localSmVol = __builtin_fmaf(localVolGain, vol_alpha, __builtin_fmaf(localSmVol, (1.0f - vol_alpha), 1e-9f));
            envBuf[i] = localEnv; masterGainBuf[i] = localSwell * localSmVol;
            inBuf[i] = inSample; fzOutBuf[i] = 0.0f;
        }
        currentPitch = localPitch; input_dc_offset = localDC; inputEnvelope = localEnv;
        localSwellGain = localSwell; smoothedVolGain = localSmVol;
    }

    static void __attribute__((optimize("Ofast"))) mixdownAndOutput(
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