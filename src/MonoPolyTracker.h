#pragma once 
#include "SystemState.h" 
#include "LUTManager.h" 
#include <dsps_fft2r.h> 
#include <atomic> 
#include <cmath> 

extern std::atomic<uint32_t> currentSampleRate; 

class MonoPolyTracker { 
private:     
    const int FADE_OUT_SAMPLES = 480;        
    const int CONFIDENCE_WAIT = 2400;        
    const int FADE_IN_SAMPLES = 1440;             
    
    enum State { UNVOICED, WAITING, FADING_IN, LOCKED, FADING_OUT };     
    State currentState = UNVOICED;          
    int timerCount = 0;     
    float confidenceMix = 0.0f;           
    
    float analysisBuffer[2048] __attribute__((aligned(16))) = {0.0f};     
    int writeIdx = 0;          
    float detectedT0 = 0.0f;     
    int sweepCounter = 0;     
    bool lastVoicedState = false;     
    
    // --- Algo 0: TD-PSOLA State ---     
    struct PSOLAGrain {         
        bool active = false;         
        int anchorIdx = 0;         
        float currentPhase = 0.0f;          
        float phaseInc = 0.0f;         
        float winSpan = 0.0f;      
    };     
    PSOLAGrain grains[8];      
    float synthesisPhase = 0.0f;     
    
    // --- Algo 1: YIN Synth State ---     
    float synthOscPhase = 0.0f;     
    float smoothedFreq = 0.0f;     
    
    // --- Algo 2: ECKF State ---     
    float kf_freq = 0.0f;        
    float kf_p = 1.0f;           
    float eckfPhase = 0.0f;      
    
    // --- Algo 3: Variable Delay Line State (Pointers for PSRAM) ---     
    float* varDelayBuffer = nullptr;     
    int varWriteIdx = 0;     
    float vdPhase = 0.0f;     
    float vdEnv = 0.0f;     
    float vdTransientMix = 0.0f;     
    
    // --- Algo 4: Phase Vocoder State (Pointers for PSRAM) ---     
    bool pvInitialized = false;     
    float* pvInput = nullptr;     
    float* pvOutput = nullptr;     
    float* pvWorkspace = nullptr; 
    float* pvMagWorkspace = nullptr;     
    float* pvPhaseWorkspace = nullptr;     
    float* pvLastPhase = nullptr;     
    float* pvSumPhase = nullptr;     
    int pvWriteIdx = 0;     
    int pvReadIdx = 0;     
    int pvHopCounter = 0;     
    float pvEnv = 0.0f;     
    float pvTransientMix = 0.0f; 

public:     
    void allocateBuffers() {
        // Dynamic allocation in PSRAM to protect internal DMA SRAM
        varDelayBuffer   = (float*)heap_caps_aligned_alloc(64, 8192 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvInput          = (float*)heap_caps_aligned_alloc(64, 512 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvOutput         = (float*)heap_caps_aligned_alloc(64, 512 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvWorkspace      = (float*)heap_caps_aligned_alloc(64, 1024 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvMagWorkspace   = (float*)heap_caps_aligned_alloc(64, 257 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvPhaseWorkspace = (float*)heap_caps_aligned_alloc(64, 257 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvLastPhase      = (float*)heap_caps_aligned_alloc(64, 257 * sizeof(float), MALLOC_CAP_SPIRAM);
        pvSumPhase       = (float*)heap_caps_aligned_alloc(64, 257 * sizeof(float), MALLOC_CAP_SPIRAM);

        if (varDelayBuffer) memset(varDelayBuffer, 0, 8192 * sizeof(float));
        if (pvInput) memset(pvInput, 0, 512 * sizeof(float));
        if (pvOutput) memset(pvOutput, 0, 512 * sizeof(float));
        if (pvWorkspace) memset(pvWorkspace, 0, 1024 * sizeof(float));
        if (pvMagWorkspace) memset(pvMagWorkspace, 0, 257 * sizeof(float));
        if (pvPhaseWorkspace) memset(pvPhaseWorkspace, 0, 257 * sizeof(float));
        if (pvLastPhase) memset(pvLastPhase, 0, 257 * sizeof(float));
        if (pvSumPhase) memset(pvSumPhase, 0, 257 * sizeof(float));
    }

    inline void process(float inSample, float targetPitch, float& outSample, float& drySample,                          
                         float p1, float p2, float p3, float p4, float p5) __attribute__((always_inline)) {                  
        analysisBuffer[writeIdx] = inSample;         
        writeIdx = (writeIdx + 1) & 2047;         
        int algo = constrain((int)(p1 * 4.99f), 0, 4);         
        
        if (++sweepCounter >= 256) {             
            sweepCounter = 0;             
            int minFreq = (algo == 0) ? (70 + (int)(p2 * 230.0f)) : 70;             
            lastVoicedState = trackPitch(currentSampleRate.load(std::memory_order_relaxed), minFreq);         
        }         
        
        switch(currentState) {             
            case UNVOICED:                 
                confidenceMix = 0.0f;                 
                if (lastVoicedState) { currentState = WAITING; timerCount = 0; }                 
                break;             
            case WAITING:                 
                confidenceMix = 0.0f;                 
                if (!lastVoicedState) { currentState = UNVOICED; }                 
                else if (++timerCount >= CONFIDENCE_WAIT) { currentState = FADING_IN; timerCount = 0; }                 
                break;             
            case FADING_IN:                 
                if (!lastVoicedState) { currentState = FADING_OUT; timerCount = (int)(confidenceMix * FADE_OUT_SAMPLES); }                 
                else {                     
                    confidenceMix += (1.0f / FADE_IN_SAMPLES);                     
                    if (confidenceMix >= 1.0f) { confidenceMix = 1.0f; currentState = LOCKED; }                 
                }                 
                break;             
            case LOCKED:                 
                confidenceMix = 1.0f;                 
                if (!lastVoicedState) { currentState = FADING_OUT; timerCount = FADE_OUT_SAMPLES; }                 
                break;             
            case FADING_OUT:                 
                confidenceMix -= (1.0f / FADE_OUT_SAMPLES);                 
                if (confidenceMix <= 0.0f || --timerCount <= 0) { confidenceMix = 0.0f; currentState = UNVOICED; }                 
                break;         
        }         
        
        float shiftedSample = 0.0f;         
        float finalAppliedMix = confidenceMix * p5;          
        
        if (finalAppliedMix > 0.001f) {             
            switch(algo) {                 
                case 0: shiftedSample = executeTDPSOLA(inSample, targetPitch, detectedT0, p3, p4); break;                 
                case 1: shiftedSample = executeYinSynth(targetPitch, detectedT0, p2, p3, p4, currentSampleRate.load(std::memory_order_relaxed)); break;                 
                case 2: shiftedSample = executeECKF(targetPitch, detectedT0, p2, p3, p4, currentSampleRate.load(std::memory_order_relaxed)); break;                 
                case 3: if (varDelayBuffer) shiftedSample = executeVarDelay(inSample, targetPitch, detectedT0, p2, p3, p4, currentSampleRate.load(std::memory_order_relaxed)); break;                 
                case 4: if (pvInput) shiftedSample = executePhaseVocoder(inSample, targetPitch, p2, p3, p4, currentSampleRate.load(std::memory_order_relaxed)); break;             
            }         
        }                  
        outSample = __builtin_fmaf(shiftedSample, finalAppliedMix, drySample * (1.0f - finalAppliedMix));     
    }

private:
    inline bool trackPitch(uint32_t sr, int minFreq) __attribute__((always_inline)) {         
        int lagMin = sr / 1200;          
        int lagMax = sr / minFreq;         
        int window = 512;                         
        float r0 = 0.0f;          
        
        for (int i = 0; i < window; i++) {             
            int idx = (writeIdx - window + i + 2048) & 2047;             
            float val = analysisBuffer[idx];             
            r0 = __builtin_fmaf(val, val, r0);         
        }         
        if (r0 < 0.01f) return false;         
        
        float maxR = 0.0f;         
        int bestLag = lagMin;         
        for (int lag = lagMin; lag <= lagMax; lag++) {             
            float rLag = 0.0f;             
            for (int i = 0; i < window; i++) {                 
                int idx1 = (writeIdx - window - lag + i + 2048) & 2047;                 
                int idx2 = (writeIdx - window + i + 2048) & 2047;                 
                rLag = __builtin_fmaf(analysisBuffer[idx1], analysisBuffer[idx2], rLag);             
            }             
            if (rLag > maxR) { maxR = rLag; bestLag = lag; }         
        }         
        if ((maxR / r0) > 0.85f) { detectedT0 = (float)bestLag; return true; }         
        return false;      
    }     

    // --- ALGO 0: TD-PSOLA ---     
    inline float executeTDPSOLA(float sample, float targetPitch, float T0, float p_frm, float p_tol) __attribute__((always_inline)) {         
        float T1 = T0 / targetPitch;          
        synthesisPhase += 1.0f;                  
        
        if (synthesisPhase >= T1) {             
            synthesisPhase -= T1;              
            int searchStart = (writeIdx - (int)(2.5f * T0) + 4096) & 2047;             
            int searchLen = (int)(T0 * (0.1f + p_tol * 0.9f));             
            float maxVal = -1.0f; int epochIdx = searchStart;             
            for (int i = 0; i < searchLen; i++) {                 
                int idx = (searchStart + i) & 2047;                 
                float absVal = fabsf(analysisBuffer[idx]);                 
                if (absVal > maxVal) { maxVal = absVal; epochIdx = idx; }             
            }             
            float windowMultiplier = 1.5f + (p_frm * 1.5f);             
            for (int i = 0; i < 8; i++) {                 
                if (!grains[i].active) {                     
                    grains[i].active = true; grains[i].winSpan = windowMultiplier * T0;                     
                    grains[i].anchorIdx = (epochIdx - (int)(grains[i].winSpan * 0.5f) + 4096) & 2047;                     
                    grains[i].currentPhase = 0.0f; grains[i].phaseInc = 1.0f / grains[i].winSpan; break;                 
                }             
            }         
        }         
        float outSample = 0.0f;         
        for (int i = 0; i < 8; i++) {             
            if (grains[i].active) {                 
                int readOffset = (int)(grains[i].currentPhase * grains[i].winSpan);                 
                int readIdx = (grains[i].anchorIdx + readOffset) & 2047;                 
                float rawSample = analysisBuffer[readIdx];                 
                int lutIdx = ((int)(grains[i].currentPhase * 4095.0f)) & 4095;                 
                outSample = __builtin_fmaf(rawSample, LUTManager::hannLUT[lutIdx], outSample);                 
                grains[i].currentPhase += grains[i].phaseInc;                 
                if (grains[i].currentPhase >= 1.0f) grains[i].active = false;             
            }         
        }         
        return outSample;     
    }     

    // --- ALGO 1: YIN SYNTH ---     
    inline float executeYinSynth(float targetPitch, float T0, float p_wav, float p_gld, float p_oct, float sr) __attribute__((always_inline)) {         
        if (T0 <= 0.1f) return 0.0f;         
        float F0 = sr / T0;          
        if (p_oct > 0.5f) targetPitch = exp2f(roundf(log2f(targetPitch)));         
        float F1 = F0 * targetPitch;         
        if (currentState == UNVOICED || currentState == WAITING) smoothedFreq = F1;         
        float alpha = (p_gld > 0.01f) ? (1.0f / (p_gld * sr * 0.5f)) : 1.0f;         
        smoothedFreq += (F1 - smoothedFreq) * alpha;         
        synthOscPhase += smoothedFreq / sr;         
        if (synthOscPhase >= 1.0f) synthOscPhase -= 1.0f;         
        float sine = sinf(synthOscPhase * 6.2831853f);         
        float tri = 2.0f * fabsf(2.0f * synthOscPhase - 1.0f) - 1.0f;         
        float saw = 2.0f * synthOscPhase - 1.0f;                  
        if (p_wav < 0.5f) return __builtin_fmaf(tri - sine, p_wav * 2.0f, sine);         
        return __builtin_fmaf(saw - tri, (p_wav - 0.5f) * 2.0f, tri);     
    }     

    // --- ALGO 2: ECKF ---     
    inline float executeECKF(float targetPitch, float T0, float p_Q, float p_R, float p_spd, float sr) __attribute__((always_inline)) {         
        if (T0 <= 0.1f) return 0.0f;         
        float F0_meas = sr / T0;         
        if (currentState == UNVOICED) { kf_freq = F0_meas; kf_p = 1.0f; }         
        float Q = 0.0001f + (p_Q * 0.05f);          
        float R = 0.01f + (p_R * 10.0f);         
        float p_pred = kf_p + Q;         
        float K = __builtin_fminf(p_pred / (p_pred + R), 0.1f + (p_spd * 0.9f));         
        kf_freq = kf_freq + K * (F0_meas - kf_freq);         
        kf_p = (1.0f - K) * p_pred;         
        eckfPhase += (kf_freq * targetPitch) / sr;         
        if (eckfPhase >= 1.0f) eckfPhase -= 1.0f;         
        return sinf(eckfPhase * 6.2831853f);     
    }     

    // --- ALGO 3: VARIABLE DELAY LINE ---     
    inline float executeVarDelay(float sample, float targetPitch, float T0, float p_win, float p_zc, float p_trn, float sr) __attribute__((always_inline)) {         
        varDelayBuffer[varWriteIdx] = sample;                  
        float absSample = fabsf(sample);         
        vdEnv = __builtin_fmaf(absSample - vdEnv, (absSample > vdEnv) ? 0.2f : 0.001f, vdEnv);         
        float transientSpike = absSample / (vdEnv + 0.001f);                   
        if (p_trn > 0.05f && transientSpike > (10.0f - p_trn * 8.0f)) vdTransientMix = 1.0f;          
        vdTransientMix *= 0.995f;          
        float currentShift = (p_trn > 0.05f) ? __builtin_fmaf(1.0f - targetPitch, vdTransientMix, targetPitch) : targetPitch;         
        float baseWin = sr * (0.01f + p_win * 0.04f);                   
        if (p_zc > 0.5f && T0 > 10.0f) {             
            float cycles = __builtin_fmaxf(roundf(baseWin / T0), 1.0f);             
            baseWin = cycles * T0;         
        }         
        baseWin = __builtin_fminf(baseWin, 8000.0f);          
        vdPhase += (1.0f - currentShift) / baseWin;         
        if (vdPhase >= 1.0f) vdPhase -= 1.0f;         
        else if (vdPhase < 0.0f) vdPhase += 1.0f;         
        float tap1Phase = vdPhase;         
        float tap2Phase = fmodf(vdPhase + 0.5f, 1.0f);          
        int delay1 = (int)(tap1Phase * baseWin);         
        int delay2 = (int)(tap2Phase * baseWin);         
        float s1 = varDelayBuffer[(varWriteIdx - delay1 + 8192) & 8191];         
        float s2 = varDelayBuffer[(varWriteIdx - delay2 + 8192) & 8191];         
        float w1 = 1.0f - fabsf(tap1Phase * 2.0f - 1.0f);         
        float w2 = 1.0f - fabsf(tap2Phase * 2.0f - 1.0f);         
        varWriteIdx = (varWriteIdx + 1) & 8191;         
        return __builtin_fmaf(s1, w1, s2 * w2);     
    }     

    // --- ALGO 4: MONO PHASE VOCODER ---     
    inline float executePhaseVocoder(float sample, float targetPitch, float p_blr, float p_phs, float p_trn, float sr) __attribute__((always_inline)) {         
        float absSample = fabsf(sample);         
        pvEnv = __builtin_fmaf(absSample - pvEnv, (absSample > pvEnv) ? 0.2f : 0.001f, pvEnv);         
        float transientSpike = absSample / (pvEnv + 0.001f);         
        if (p_trn > 0.05f && transientSpike > (10.0f - p_trn * 8.0f)) pvTransientMix = 1.0f;         
        pvTransientMix *= 0.995f;         
        float currentShift = (p_trn > 0.05f) ? __builtin_fmaf(1.0f - targetPitch, pvTransientMix, targetPitch) : targetPitch;         
        
        pvInput[pvWriteIdx] = sample;         
        pvWriteIdx = (pvWriteIdx + 1) & 511;         
        int hopSize = 128; 
        
        float outSample = pvOutput[pvReadIdx];         
        pvOutput[pvReadIdx] = 0.0f;          
        pvReadIdx = (pvReadIdx + 1) & 511;         
        
        if (++pvHopCounter >= hopSize) {             
            pvHopCounter = 0;                          
            if (!pvInitialized) {                 
                dsps_fft2r_init_fc32(NULL, 512);                 
                pvInitialized = true;             
            }             
            for (int i = 0; i < 512; i++) {                 
                int idx = (pvWriteIdx - 512 + i + 512) & 511;                 
                pvWorkspace[i*2] = pvInput[idx] * LUTManager::hannLUT[(i * 8) & 4095];                  
                pvWorkspace[i*2+1] = 0.0f;                 
                pvMagWorkspace[i] = 0.0f;                 
                pvPhaseWorkspace[i] = 0.0f;             
            }             
            dsps_fft2r_fc32(pvWorkspace, 512);             
            dsps_bit_rev_fc32(pvWorkspace, 512);             
            for (int k = 0; k <= 256; k++) {                 
                float re = pvWorkspace[k*2];                 
                float im = pvWorkspace[k*2+1];                                  
                float mag = sqrtf(re*re + im*im);                 
                float phase = atan2f(im, re);                 
                float deltaPhase = phase - pvLastPhase[k];                 
                pvLastPhase[k] = phase;                 
                deltaPhase -= (float)k * 6.2831853f * (float)hopSize / 512.0f;                                  
                deltaPhase = deltaPhase - 6.2831853f * roundf(deltaPhase * 0.1591549f);                 
                float trueFreq = ((float)k * 6.2831853f / 512.0f) + (deltaPhase / (float)hopSize);                 
                int newBin = (int)((float)k * currentShift);                                  
                if (newBin <= 256) {                     
                    float blur = 1.0f - (p_blr * 0.8f);                      
                    pvMagWorkspace[newBin] += mag * blur;                      
                    float synthPhase = pvSumPhase[k] + trueFreq * (float)hopSize * currentShift;                     
                    pvPhaseWorkspace[newBin] = __builtin_fmaf(phase - synthPhase, p_phs, synthPhase);                     
                    pvSumPhase[newBin] = pvPhaseWorkspace[newBin];                 
                }             
            }             
            for (int k = 0; k <= 256; k++) {                 
                float mag = pvMagWorkspace[k];                 
                float phase = pvPhaseWorkspace[k];                 
                pvWorkspace[k*2] = mag * cosf(phase);                 
                pvWorkspace[k*2+1] = mag * sinf(phase);                                  
                if (k > 0 && k < 256) {                      
                    pvWorkspace[(512-k)*2] = pvWorkspace[k*2];                     
                    pvWorkspace[(512-k)*2+1] = -pvWorkspace[k*2+1];                 
                }             
            }             
            dsps_fft2r_fc32(pvWorkspace, 512);             
            dsps_bit_rev_fc32(pvWorkspace, 512);              
            for (int i = 0; i < 512; i++) {                 
                int idx = (pvReadIdx + i) & 511;                 
                float val = pvWorkspace[i*2] * 0.001953125f * LUTManager::hannLUT[(i * 8) & 4095];                 
                pvOutput[idx] += val;             
            }         
        }         
        return outSample;     
    } 
}; 

extern MonoPolyTracker monoPolyEngine;