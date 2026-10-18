#pragma once
#include "SystemState.h"
#include "LUTManager.h"
#include "AudioPipeline.h"

static void IRAM_ATTR __attribute__((optimize("Ofast"))) AudioDSPTask(void * pvParameters) {
    DSPCoreState* lastAckedDSP = nullptr;

    while (LUTManager::hannLUT == nullptr || LUTManager::lfoLUT == nullptr || LUTManager::synthLUT == nullptr || LUTManager::apf1Buffer == nullptr || LUTManager::apf2Buffer == nullptr || LUTManager::pitchSincLUT == nullptr) {
          vTaskDelay(pdMS_TO_TICKS(10));
      }

    dsp_ready_for_i2s.store(true, std::memory_order_release);
    
    while (I2SManager::rx_chan == NULL || I2SManager::tx_chan == NULL) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for(;;) {
#ifdef ENABLE_ADVANCED_TELEMETRY
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
            uint32_t t0 = xthal_get_ccount();
            rx_err = i2s_channel_read((i2s_chan_handle_t)I2SManager::rx_chan, i2s_in_block, HOP_SIZE*8, &bytesRead, portMAX_DELAY);
            uint32_t rx_c = xthal_get_ccount() - t0;
            
            // Log Peak CPU Cycles for the hardware read
            if (rx_c > dbg_peak_rx_cyc.load(std::memory_order_relaxed)) dbg_peak_rx_cyc.store(rx_c, std::memory_order_relaxed);
            
            if (rx_err != ESP_OK) dbg_rx_errors.fetch_add(1, std::memory_order_relaxed);
            
            // A partial read means the DMA lost phase alignment. This is fatal to audio.
            if (bytesRead > 0 && bytesRead != HOP_SIZE*8) dbg_partial_rx.fetch_add(1, std::memory_order_relaxed);
            // If bytes are not a multiple of 8, L/R stereo bytes are physically inverted.
            if (bytesRead % 8 != 0) dbg_phase_tears.fetch_add(1, std::memory_order_relaxed);

            if (rx_err != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue; 
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if(__builtin_expect(bytesRead > 0, 1)) {
            int framesRead = bytesRead / 8;
            if(__builtin_expect(framesRead == HOP_SIZE, 1)) {
                
                if(__builtin_expect(panicResetRequested.load(std::memory_order_acquire), 0)) {
                    AudioPipeline::handlePanicReset();
                }
                if(__builtin_expect(globalAudioResetRequested.load(std::memory_order_acquire), 0)) {
                    AudioPipeline::handleGlobalReset();
                }

                int currentMute = hardwareSyncMuteFrames.load(std::memory_order_acquire); bool isMuted = false;
                if(__builtin_expect(currentMute > 0, 0)) { hardwareSyncMuteFrames.store(currentMute - 1, std::memory_order_release); isMuted = true; }
                
                uint32_t start_cycles = xthal_get_ccount();
                
                DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                
                AudioPipeline::processBlock(framesRead, activeDSP);

                uint32_t end_timer = xthal_get_ccount();
                uint32_t dsp_c = end_timer - start_cycles;
                
                // Log Peak CPU Cycles for the DSP loop. At 240MHz, > 160000 means we missed the 96kHz deadline.
                if (dsp_c > dbg_peak_dsp_cyc.load(std::memory_order_relaxed)) dbg_peak_dsp_cyc.store(dsp_c, std::memory_order_relaxed);
                
                float max_cycles = (currentSampleRate.load(std::memory_order_relaxed) == 96000) ? (2500.0f * (float)framesRead) : (5000.0f * (float)framesRead);
                core0_dsp_load.store(__builtin_fmaf(core0_dsp_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, (((float)dsp_c / max_cycles) * 100.0f)) * 0.05f), std::memory_order_relaxed);

                if(__builtin_expect(isMuted, 0)) memset(i2s_out_block, 0, framesRead * 8);
                
                size_t bytesWrittenCount = 0;
                if (I2SManager::tx_chan != NULL) {
                    uint32_t t4 = xthal_get_ccount();
                    esp_err_t tx_err = i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, portMAX_DELAY);
                    uint32_t tx_c = xthal_get_ccount() - t4;
                    
                    // Log Peak CPU Cycles for the hardware write buffer wait
                    if (tx_c > dbg_peak_tx_cyc.load(std::memory_order_relaxed)) dbg_peak_tx_cyc.store(tx_c, std::memory_order_relaxed);
                    if (tx_err != ESP_OK) dbg_tx_errors.fetch_add(1, std::memory_order_relaxed);
                }
                
                if (__builtin_expect(activeDSP != lastAckedDSP, 0)) {
                    lastAckedDSP = activeDSP;
                    dspAckCommit.store(true, std::memory_order_release);
                }
            } else {
#ifdef ENABLE_ADVANCED_TELEMETRY
                audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
#endif
                memset(i2s_out_block, 0, HOP_SIZE * 8);
                size_t dummyBytes = 0;
                if (I2SManager::tx_chan != NULL) {
                    i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, HOP_SIZE * 8, &dummyBytes, portMAX_DELAY);
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}