#ifndef LUT_MANAGER_H
#define LUT_MANAGER_H

#include <Arduino.h>
#include <atomic>

class LUTManager {
public:
    inline static float *hannLUT = nullptr;
    inline static float *lfoLUT = nullptr;
    inline static float *synthLUT = nullptr;
    inline static float *pitchSincLUT = nullptr;
    inline static float *apf1Buffer = nullptr;
    inline static float *apf2Buffer = nullptr;
    inline static std::atomic<float*> pitchShiftLUT{nullptr}; 
    inline static float *pitchShiftLUT_temp = nullptr;

    inline static std::atomic<float> globalHarmRatio{1.0f};
    inline static std::atomic<float> globalChorusRatio{1.0f};
    inline static std::atomic<float> globalFbRatio{1.0f};
    inline static std::atomic<float> globalVibratoPhaseInc{0.0f};

    static void allocateAll() {
        hannLUT = (float*)heap_caps_aligned_alloc(64, 4096 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        lfoLUT = (float*)heap_caps_aligned_alloc(64, 1024 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        synthLUT = (float*)heap_caps_aligned_alloc(64, 2048 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        pitchSincLUT = (float*)heap_caps_aligned_alloc(64, 4096 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        apf1Buffer = (float*)heap_caps_aligned_alloc(64, 1009 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        apf2Buffer = (float*)heap_caps_aligned_alloc(64, 863 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        
        float* initialLUT = (float*)heap_caps_aligned_alloc(64, 16384*sizeof(float), MALLOC_CAP_SPIRAM); 
        pitchShiftLUT.store(initialLUT, std::memory_order_relaxed); 
        pitchShiftLUT_temp = (float*)heap_caps_aligned_alloc(64, 16384*sizeof(float), MALLOC_CAP_SPIRAM);
    }

    static void generateStaticTables() {
        memset(apf1Buffer, 0, 1009 * sizeof(float));
        memset(apf2Buffer, 0, 863 * sizeof(float));

        for(int i=0; i<4096; i++) hannLUT[i] = 0.5f*(1.0f-cosf(TWO_PI*((float)i/4095.0f))); 
        for(int i=0; i<1024; i++) lfoLUT[i] = powf(2.0f,(15.0f*sinf(TWO_PI*((float)i/1024.0f)))/1200.0f); 
        for(int i=0; i<2048; i++) synthLUT[i] = sinf((((float)i-1024.0f)/1024.0f)*45.0f);

        for(int i=0; i<1024; i++) {
            float frac = (float)i / 1023.0f;
            float x[4] = { 1.0f + frac, frac, 1.0f - frac, 2.0f - frac };
            for(int j=0; j<4; j++) {
                float val = (fabsf(x[j]) < 1e-5f) ? 1.0f : sinf(M_PI * x[j]) / (M_PI * x[j]);
                val *= 0.5f * (1.0f + cosf(M_PI * x[j] / 2.0f)); 
                pitchSincLUT[i*4 + j] = val;
            }
        }
    }

    static void __attribute__((optimize("Ofast"))) updateDynamicLUT(bool isCapo, bool isWhammy, int activeMode, volatile float* fxMem, int fbIdx, uint32_t currentSampleRate) {
        static std::atomic<bool> lutBusy{false};
        if (lutBusy.exchange(true, std::memory_order_acquire)) return; 

        if (!pitchShiftLUT_temp) { lutBusy.store(false, std::memory_order_release); return; }

        float basePitch = (isCapo || (activeMode==4 && isWhammy)) ? fxMem[4] : 0.0f;
        float toeBend = fxMem[0], heelBend = fxMem[1], harmRatioMem = fxMem[3], chorusRatioMem = fxMem[7], vibHzMem = fxMem[9];
        
        // Removed vTaskDelay(1) to execute table generation atomically and prevent pedal event lockups
        for(int i=0; i<16384; i++) {
            float normalizedThrow = (i>=8192) ? ((float)(i-8192)/8191.0f) : ((float)(i-8192)/8192.0f); 
            pitchShiftLUT_temp[i] = powf(2.0f,(basePitch+((normalizedThrow>=0.0f)?(toeBend*normalizedThrow):(heelBend*fabsf(normalizedThrow))))/12.0f); 
        }

        float* tempPtr = pitchShiftLUT.load(std::memory_order_relaxed);
        pitchShiftLUT.store(pitchShiftLUT_temp, std::memory_order_release); 
        pitchShiftLUT_temp = tempPtr;

        globalHarmRatio.store(powf(2.0f, harmRatioMem/12.0f), std::memory_order_release); 
        globalChorusRatio.store(powf(2.0f, chorusRatioMem/12.0f), std::memory_order_release); 
        float fbIntervals[5] = {0.0f, 12.0f, 19.0f, 24.0f, 28.0f}; 
        globalFbRatio.store(powf(2.0f, fbIntervals[constrain(fbIdx,0,4)]/12.0f), std::memory_order_release);
        if (currentSampleRate > 0) globalVibratoPhaseInc.store((((vibHzMem!=0.0f)?fabsf(vibHzMem):2.0f)*1024.0f)/(float)currentSampleRate, std::memory_order_release);
        
        lutBusy.store(false, std::memory_order_release);
    }
};

#endif