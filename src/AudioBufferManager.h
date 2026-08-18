#pragma once
#include "SystemState.h"

class AudioBufferManager {
public:
    static void allocate() {
        delayBuffer=(int16_t*)heap_caps_aligned_alloc(64, MAX_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM); 
        sramPitchLow=(int16_t*)heap_caps_aligned_alloc(64, SRAM_PITCH_BUF_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        sramPitchHigh=(int16_t*)heap_caps_aligned_alloc(64, SRAM_PITCH_BUF_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        fbDelayBuffer=(int16_t*)heap_caps_aligned_alloc(64, FB_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        freezeBuffer=(int16_t*)heap_caps_aligned_alloc(64, FREEZE_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM); 
        diffuserBuf = (float*)heap_caps_aligned_alloc(64, 1024 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        i2s_in_block = (int32_t*)heap_caps_aligned_alloc(64, HOP_SIZE * 2 * sizeof(int32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        i2s_out_block = (int32_t*)heap_caps_aligned_alloc(64, HOP_SIZE * 2 * sizeof(int32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        inBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        envBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        fzOutBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        masterGainBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        w1Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        w2Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        w3Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        padFilterBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        dryBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        fbOutBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        sMixBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); 
        
        pitchLutBufferA = (float*)heap_caps_aligned_alloc(64, 16384 * sizeof(float), MALLOC_CAP_SPIRAM);
        pitchLutBufferB = (float*)heap_caps_aligned_alloc(64, 16384 * sizeof(float), MALLOC_CAP_SPIRAM);
        activePitchLUT.store(pitchLutBufferA, std::memory_order_release); // <-- FIXED TYPO
        
        if (!delayBuffer || !sramPitchLow || !sramPitchHigh || !fbDelayBuffer || 
            !freezeBuffer || !diffuserBuf || !i2s_in_block || !i2s_out_block || 
            !inBuf || !envBuf || !fzOutBuf || !masterGainBuf || !w1Buf || 
            !w2Buf || !w3Buf || !padFilterBuf || !dryBuf || !fbOutBuf || 
            !sMixBuf || !pitchLutBufferA || !pitchLutBufferB) {
            Serial.println("CRITICAL: SPIRAM/SRAM Allocation Failed!");
            while(1) { vTaskDelay(pdMS_TO_TICKS(100)); } 
        }

        memset(i2s_in_block, 0, HOP_SIZE * 2 * sizeof(int32_t)); memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t)); memset(inBuf, 0, HOP_SIZE * sizeof(float)); memset(envBuf, 0, HOP_SIZE * sizeof(float)); memset(fzOutBuf, 0, HOP_SIZE * sizeof(float)); memset(masterGainBuf, 0, HOP_SIZE * sizeof(float)); memset(w1Buf, 0, HOP_SIZE * sizeof(float)); memset(w2Buf, 0, HOP_SIZE * sizeof(float)); memset(w3Buf, 0, HOP_SIZE * sizeof(float)); memset(padFilterBuf, 0, HOP_SIZE * sizeof(float)); memset(dryBuf, 0, HOP_SIZE * sizeof(float)); memset(fbOutBuf, 0, HOP_SIZE * sizeof(float)); memset(sMixBuf, 0, HOP_SIZE * sizeof(float)); 
        memset(delayBuffer, 0, MAX_BUFFER_SIZE*sizeof(int16_t)); 
        memset(sramPitchLow, 0, SRAM_PITCH_BUF_SIZE*sizeof(int16_t)); 
        memset(sramPitchHigh, 0, SRAM_PITCH_BUF_SIZE*sizeof(int16_t)); 
        memset(fbDelayBuffer, 0, FB_BUFFER_SIZE*sizeof(int16_t)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE*sizeof(int16_t)); memset(diffuserBuf, 0, 1024*sizeof(float));
    }
};