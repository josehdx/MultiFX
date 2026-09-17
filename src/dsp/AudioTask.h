#pragma once

#include "SystemState.h"
#include "LUTManager.h"
#include "AudioPipeline.h"

static void IRAM_ATTR __attribute__((optimize("Ofast"))) AudioDSPTask(void * pvParameters) {
    DSPCoreState* lastAckedDSP = nullptr;

    while (LUTManager::hannLUT == nullptr || LUTManager::lfoLUT == nullptr || LUTManager::synthLUT == nullptr || LUTManager::apf1Buffer == nullptr || LUTManager::apf2Buffer == nullptr || LUTManager::pitchSincLUT == nullptr) { 
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }

    for(;;) {
#ifdef ENABLE_ADVANCED_TELEMETRY
        // Record remaining Stack RAM in Bytes for Core 0 Audio Task
        dsp_stack_watermark.store((uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t), std::memory_order_relaxed);
#endif

        if(__builtin_expect(dsp_is_paused.load(std::memory_order_acquire), 0)) {
            dsp_ack_parked.store(true, std::memory_order_release);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            dsp_ack_parked.store(false, std::memory_order_release);
        }

        size_t bytesRead = 0; 
        i2s_channel_read((i2s_chan_handle_t)I2SManager::rx_chan, i2s_in_block, HOP_SIZE*2*sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(10));

        if(__builtin_expect(bytesRead > 0, 1)) {
            int framesRead = bytesRead / 8;
            if(__builtin_expect(framesRead == HOP_SIZE, 1)) {
                
                // --- 1. HANDLE SYSTEM RESET EVENTS ---
                if(__builtin_expect(panicResetRequested.load(std::memory_order_acquire), 0)) {
                    AudioPipeline::handlePanicReset();
                }

                if(__builtin_expect(globalAudioResetRequested.load(std::memory_order_acquire), 0)) {
                    AudioPipeline::handleGlobalReset();
                }

                int currentMute = hardwareSyncMuteFrames.load(std::memory_order_acquire); bool isMuted = false;
                if(__builtin_expect(currentMute > 0, 0)) { hardwareSyncMuteFrames.store(currentMute - 1, std::memory_order_release); isMuted = true; }
                
                uint32_t start_cycles=xthal_get_ccount(); 
                
                // Fetch latest control state safely
                DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                
                // --- 2. EXECUTE THE INLINED DSP MATH PIPELINE ---
                AudioPipeline::processBlock(framesRead, activeDSP);

                uint32_t end_timer=xthal_get_ccount(); 
                float max_cycles = (currentSampleRate.load(std::memory_order_relaxed) == 96000) ? (2500.0f * (float)framesRead) : (5000.0f * (float)framesRead);
                core0_dsp_load.store(__builtin_fmaf(core0_dsp_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, (((float)(end_timer - start_cycles) / max_cycles) * 100.0f)) * 0.05f), std::memory_order_relaxed);

                // --- 3. HARDWARE MUTE & I2S WRITE ---
                if(__builtin_expect(isMuted, 0)) memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                size_t bytesWrittenCount;
                i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20));
                
                if (__builtin_expect(activeDSP != lastAckedDSP, 0)) {
                    lastAckedDSP = activeDSP;
                    dspAckCommit.store(true, std::memory_order_release);
                }

            } else {
#ifdef ENABLE_ADVANCED_TELEMETRY
                audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
#endif
                memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
                size_t dummyBytes;
                i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, HOP_SIZE*8, &dummyBytes, 0);
                vTaskDelay(pdMS_TO_TICKS(1)); // Prevents tight CPU 0 loop when framesRead != HOP_SIZE
            }
        } else {
#ifdef ENABLE_ADVANCED_TELEMETRY
            audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
#endif
            memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
            size_t dummyBytes;
            i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, HOP_SIZE*8, &dummyBytes, 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // Prevents Task WDT trigger during I2S clock reconfig
        }
    }
}