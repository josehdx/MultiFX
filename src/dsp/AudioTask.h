#pragma once
#include "SystemState.h"
#include "LUTManager.h"
#include "AudioPipeline.h"

static void IRAM_ATTR __attribute__((optimize("Ofast"))) AudioDSPTask(void * pvParameters) {
    DSPCoreState* lastAckedDSP = nullptr;

    while (LUTManager::hannLUT == nullptr || LUTManager::lfoLUT == nullptr || LUTManager::synthLUT == nullptr || LUTManager::apf1Buffer == nullptr || LUTManager::apf2Buffer == nullptr || LUTManager::pitchSincLUT == nullptr) {
          vTaskDelay(pdMS_TO_TICKS(10));
      }

    // CRITICAL FIX: Signal that AudioDSPTask is ready to consume I2S frames.
    // SystemBootloader will NOT enable I2S DMA until this flag is set, preventing
    // ring buffer overflow and stereo phase misalignment at 96kHz boot.
    dsp_ready_for_i2s.store(true, std::memory_order_release);
    
    while (I2SManager::rx_chan == NULL || I2SManager::tx_chan == NULL) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for(;;) {
#ifdef ENABLE_ADVANCED_TELEMETRY
        // Record remaining Stack RAM in Bytes for Core 0 Audio Task
        dsp_stack_watermark.store((uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t), std::memory_order_relaxed);
#endif

        if(__builtin_expect(dsp_is_paused.load(std::memory_order_acquire), 0)) {
            while (dsp_is_paused.load(std::memory_order_acquire)) {
                dsp_ack_parked.store(true, std::memory_order_release);
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            }
            dsp_ack_parked.store(false, std::memory_order_release);
            
            memset(i2s_in_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
            memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
        }

        size_t bytesRead = 0;
        esp_err_t rx_err = ESP_FAIL;
        
        if (I2SManager::rx_chan != NULL && I2SManager::tx_chan != NULL) {
            rx_err = i2s_channel_read((i2s_chan_handle_t)I2SManager::rx_chan, i2s_in_block, HOP_SIZE*2*sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(10));
            
            // CRITICAL FIX: DMA Phase-Lock Protection.
            // If channels are disabled, destroyed, or rebuilding, reading returns ESP_ERR_INVALID_STATE.
            // We MUST skip the entire DSP and TX write phase. Attempting to write a dummy block 
            // to a disabled TX channel permanently breaks the TX/RX pointer sync.
            if (rx_err != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue; 
            }
        } else {
            // Channels are currently NULL (destroyed for mode swap or save)
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

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
                
                DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                
                // --- 2. EXECUTE THE INLINED DSP MATH PIPELINE ---
                AudioPipeline::processBlock(framesRead, activeDSP);

                uint32_t end_timer=xthal_get_ccount();
                
                float max_cycles = (currentSampleRate.load(std::memory_order_relaxed) == 96000) ? (2500.0f * (float)framesRead) : (5000.0f * (float)framesRead);
                core0_dsp_load.store(__builtin_fmaf(core0_dsp_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, (((float)(end_timer - start_cycles) / max_cycles) * 100.0f)) * 0.05f), std::memory_order_relaxed);

                // --- 3. HARDWARE MUTE & I2S WRITE ---
                if(__builtin_expect(isMuted, 0)) memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                
                size_t bytesWrittenCount = 0;
                if (I2SManager::tx_chan != NULL) {
                    i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20));
                }
                
                if (__builtin_expect(activeDSP != lastAckedDSP, 0)) {
                    lastAckedDSP = activeDSP;
                    dspAckCommit.store(true, std::memory_order_release);
                }
                taskYIELD();
            } else {
#ifdef ENABLE_ADVANCED_TELEMETRY
                audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
#endif
                memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
                size_t dummyBytes = 0;
                if (I2SManager::tx_chan != NULL) {
                    i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, HOP_SIZE * 8, &dummyBytes, 0);
                }
                taskYIELD();
            }
        } else {
            taskYIELD();
        }
    }
}