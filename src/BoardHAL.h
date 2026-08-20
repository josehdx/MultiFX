#pragma once

#include <Arduino.h>
#include "SystemState.h"
#include "PowerManager.h"
#include "esp_adc/adc_continuous.h"
#include "driver/i2s_std.h"

#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
extern DisplayManager displayManager;
#endif

// External state flags managed by the main system loop
extern bool showSavingScreen;
extern bool showKnobModeScreen;
extern bool showBleWarning;

class BoardHAL {
public:
    static void init() {
        // Initialize Hardware UART0 on GPIO 43 (TX) / GPIO 44 (RX)
        Serial0.begin(115200, SERIAL_8N1, 44, 43);
    }

    static i2s_std_config_t getI2SConfig(uint32_t sampleRate) {
        i2s_std_config_t std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)9,
                .ws   = (gpio_num_t)45,
                .dout = (gpio_num_t)8,
                .din  = (gpio_num_t)10,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv   = false,
                },
            },
        };
        return std_cfg;
    }

    // Fully updated signature to match global volatile int and std::atomic types
    static void fetchADC(adc_continuous_handle_t handle, volatile bool& isPaused, volatile int& pb1, volatile int& pb2, volatile int& pb3, volatile int& par1, std::atomic<int>& bat) {
        if (isPaused) return;
        
        uint32_t ret_num = 0;
        uint8_t result[256] = {0};
        
        esp_err_t ret = adc_continuous_read(handle, result, 256, &ret_num, 0);
        if (ret == ESP_OK) {
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = p->type2.channel;
                uint32_t data = p->type2.data;
                
                switch(chan_num) {
                    case ADC_CHANNEL_0: pb1 = data; break;
                    case ADC_CHANNEL_1: pb2 = data; break;
                    case ADC_CHANNEL_9: pb3 = data; break;
                    case ADC_CHANNEL_3: par1 = data; break;
                    case ADC_CHANNEL_2: bat.store(data, std::memory_order_relaxed); break;
                }
            }
        }
    }

    static void updateExtraControls(int activeMode, volatile float* effectMemory, volatile float fxParams[10][5], volatile bool& lutNeedsUpdate, volatile bool& dspNeedsCommit, std::atomic<int>& feedbackIntervalIdx, bool isKnobEditMode, uint16_t latestPar1) {
        // Implement any specific hardware button debouncing or 
        // external knob overriding logic for effectMemory/fxParams here
        // (Standard MIDI/Bluetooth logic is handled upstream)
    }

    static void updateUI(DSPCoreState* activeDSP, uint16_t pb1, uint16_t pb2, uint16_t pb3, uint16_t cc11, float inMeter, float outMeter, float dspLoad, float ctrlLoad, uint32_t sampleRate, uint32_t peakLatency, uint32_t underflows, uint32_t stackWatermark, bool bleConnected, float batVal, bool isMonoPolyActiveFlag) {
#if defined(TARGET_LILYGO)         
        static unsigned long lastDisplayUpdate = 0;             
        if (millis() - lastDisplayUpdate >= 33 || showSavingScreen || showKnobModeScreen) {             
            lastDisplayUpdate = millis();             
            DisplayData dData;             
            dData.batVoltage = PowerManager::getBatteryVoltage(batVal);             
            dData.batPercent = PowerManager::getBatteryPercentage(dData.batVoltage);             
            dData.bleConnected = bleConnected;             
            dData.activeMode = activeDSP->activeMode;             
            
            // Map active effects states
            for(int i = 0; i < 10; i++) {
                dData.fxStates[i] = (&activeDSP->w)[i];
            }
             
            dData.pb1 = pb1; 
            dData.pb2 = pb2; 
            dData.pb3 = pb3; 
            dData.cc11 = cc11;             
            
            // Map parameter values 
            for(int i = 0; i < 5; i++) {
                dData.paramVals[i] = activeDSP->params[activeDSP->activeMode][i];             
            }
            
            // Default names (overridden dynamically in DisplayManager if MonoPoly is active)
            dData.paramNames[0] = "P1"; 
            dData.paramNames[1] = "P2"; 
            dData.paramNames[2] = "P3"; 
            dData.paramNames[3] = "P4"; 
            dData.paramNames[4] = "P5";             
            
            dData.inMeter = inMeter; 
            dData.outMeter = outMeter;             
            dData.dspCoreLoad = dspLoad; 
            dData.ctrlCoreLoad = ctrlLoad;             
            dData.freeSRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);             
            dData.freePSRAM = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);             
            dData.sampleRate = sampleRate; 
            dData.peakLatency = peakLatency;             
            dData.underflows = underflows; 
            dData.dmaCount = 0; 
            dData.stackWatermark = stackWatermark;                          
            
            dData.isKnobEditMode = isKnobEditMode;             
            dData.showBleWarning = showBleWarning;             
            dData.showSavingScreen = showSavingScreen;             
            dData.showKnobModeScreen = showKnobModeScreen;             
            
            // Propagate MonoPoly active state flag
            dData.isMonoPolyActive = isMonoPolyActiveFlag; 

            // Map Whammy limits for the MonoPoly UI background header
            dData.whammyToe = activeDSP->fxMem[0];
            dData.whammyHeel = activeDSP->fxMem[1];

            displayManager.render(dData);         
        } 
#endif
    }
};