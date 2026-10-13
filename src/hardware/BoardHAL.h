#pragma once
#include <Arduino.h>
#include "SystemState.h"
#include "PowerManager.h"
#include "driver/i2s_std.h"

// We no longer need the continuous DMA header
// #include "esp_adc/adc_continuous.h"

#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
extern DisplayManager displayManager;
#endif

extern bool showSavingScreen;
extern bool showKnobModeScreen;
extern bool showBleWarning;

class BoardHAL {
public:
    static void init() {
#if defined(TARGET_LILYGO)
        // Disconnect Serial0 from pins 43 and 44 to prevent colliding with the I2S Audio DAC
        Serial0.begin(115200, SERIAL_8N1, -1, -1);
#else
        // Initialize Hardware UART0 on GPIO 43 (TX) / GPIO 44 (RX)
        Serial0.begin(115200, SERIAL_8N1, 44, 43);
#endif
    }

    static i2s_std_config_t getI2SConfig(uint32_t sampleRate) {
        i2s_std_config_t std_cfg = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
#if defined(TARGET_LILYGO)
                .mclk = (gpio_num_t)43,
                .bclk = (gpio_num_t)44,
                .ws   = (gpio_num_t)18,
                .dout = (gpio_num_t)21,
                .din  = (gpio_num_t)17,
#else
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)9,
                .ws   = (gpio_num_t)45,
                .dout = (gpio_num_t)8,
                .din  = (gpio_num_t)10,
#endif
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv   = false,
                },
            },
        };
        std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
        return std_cfg;
    }

    // Notice we keep the original signature so we don't break main.cpp, but 'handle' is ignored
    static void fetchADC(adc_continuous_handle_t handle, volatile bool& isPaused, volatile int& pb1, volatile int& pb2, volatile int& pb3, volatile int& par1, std::atomic<int>& bat) {
        if (isPaused) return;
        
        // Fast, non-blocking single-shot reads bypass the ADC2 DMA silicon bugs
        pb1 = analogRead(1);
        pb2 = analogRead(2);
        
#if defined(TARGET_LILYGO)
        par1 = analogRead(3);
        pb3 = analogRead(10);
        latestPar2 = analogRead(11);
        latestPar3 = analogRead(12);
        latestPar4 = analogRead(13);
        latestPar5 = analogRead(16);
        bat.store(analogRead(4), std::memory_order_relaxed);
#else
        par1 = analogRead(3);
        latestPar2 = analogRead(4);
        pb3 = analogRead(5);
        latestPar3 = analogRead(6);
        latestPar4 = analogRead(7);
        latestPar5 = analogRead(11);
#endif
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
            
            for(int i = 0; i < 10; i++) { dData.fxStates[i] = (&activeDSP->w)[i]; }
            
            dData.pb1 = pb1; 
            dData.pb2 = pb2; 
            dData.pb3 = pb3; 
            dData.cc11 = cc11;
            
            for(int i = 0; i < 5; i++) { dData.paramVals[i] = activeDSP->params[activeDSP->activeMode][i]; }
            
            dData.paramNames[0] = "P1"; dData.paramNames[1] = "P2"; dData.paramNames[2] = "P3"; 
            dData.paramNames[3] = "P4"; dData.paramNames[4] = "P5";
            
            dData.inMeter = inMeter; dData.outMeter = outMeter;
            dData.dspCoreLoad = dspLoad; dData.ctrlCoreLoad = ctrlLoad;
            
            dData.freeSRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            dData.freePSRAM = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            dData.sampleRate = sampleRate; dData.peakLatency = peakLatency;
            dData.underflows = underflows; dData.dmaCount = 0; 
            dData.stackWatermark = stackWatermark;
            
            dData.isKnobEditMode = isKnobEditMode;
            dData.showBleWarning = showBleWarning;
            dData.showSavingScreen = showSavingScreen;
            dData.showKnobModeScreen = showKnobModeScreen;
            dData.isMonoPolyActive = isMonoPolyActiveFlag;
            dData.whammyToe = activeDSP->fxMem[0];
            dData.whammyHeel = activeDSP->fxMem[1];

            displayManager.render(dData);
        }
#endif
    }
};