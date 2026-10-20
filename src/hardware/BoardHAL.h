#pragma once

#include <Arduino.h>
#include "SystemState.h"
#include "PowerManager.h"
#include "driver/i2s_std.h"
#include "BluetoothManager.h"

#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
extern DisplayManager displayManager;
#endif

extern bool showSavingScreen;
extern bool showKnobModeScreen;
extern bool showBleWarning;

// =========================================================================
// HARDWARE MODE TOGGLE
// 1 = UART Debugging on GPIO 17 & 18 (I2S Disabled on those pins)
// 0 = Full I2S Audio Mode (GPIO 43=MCLK, 44=BCLK, 18=WS, 17=DIN)
//     NOTE: Serial0 MUST be detached in Mode 0 to prevent seizing GPIO 43/44!
// =========================================================================
#define ENABLE_SERIAL_DEBUG 0

class BoardHAL {
public:
    static void init() {
#if defined(TARGET_LILYGO)
    #if ENABLE_SERIAL_DEBUG
        Serial0.begin(115200, SERIAL_8N1, 18, 17);
    #else
        Serial0.end();
        pinMode(43, OUTPUT);
        pinMode(44, OUTPUT);
    #endif
#else
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
    #if ENABLE_SERIAL_DEBUG
                .ws   = (gpio_num_t)-1, 
                .dout = (gpio_num_t)21,
                .din  = (gpio_num_t)-1,              
    #else
                .ws   = (gpio_num_t)18,
                .dout = (gpio_num_t)21,
                .din  = (gpio_num_t)17,
    #endif
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

    static void fetchADC(adc_continuous_handle_t handle, volatile bool& isPaused, volatile int& pb1, volatile int& pb2, volatile int& pb3, volatile int& par1, std::atomic<int>& bat) {
        if (isPaused) return;
        
        if (adcLock.test_and_set(std::memory_order_acquire)) return;
        
        pb1 = analogRead(1); 
        pb2 = analogRead(2); 

#if defined(TARGET_LILYGO)
        par1 = analogRead(3);
        pb3  = analogRead(10);
        bat.store(analogRead(4), std::memory_order_relaxed);
#else
        par1 = analogRead(3);
        latestPar2 = analogRead(4);
        pb3 = analogRead(5);
        latestPar3 = analogRead(6);
        latestPar4 = analogRead(7);
        latestPar5 = analogRead(11);
#endif
        adcLock.clear(std::memory_order_release);
    }

    static void updateUI(DSPCoreState* activeDSP, uint16_t pb1, uint16_t pb2, uint16_t pb3, uint16_t cc11, float inMeter, float outMeter, float dspLoad, float ctrlLoad, uint32_t sampleRate, uint32_t peakLatency, uint32_t underflows, uint32_t stackWatermark, bool bleConnected, float batVal, bool isMonoPolyActiveFlag, const char* dbgMsg) {
#if defined(TARGET_LILYGO)
        static unsigned long lastDisplayUpdate = 0;
        
        static float smoothBatRaw = -1.0f;
        if (smoothBatRaw < 0) {
            smoothBatRaw = batVal;
        } else if (batVal < smoothBatRaw - 100.0f) {
            // Drop instantly by bypassing the moving average if the USB cable is unplugged (~0.1V drop)
            smoothBatRaw = batVal; 
        } else {
            // Apply heavy filtering to strictly stabilize resting battery level jitter
            smoothBatRaw = (smoothBatRaw * 0.95f) + (batVal * 0.05f);
        }
        
        if (millis() - lastDisplayUpdate >= 250 || showSavingScreen || showKnobModeScreen) {
            lastDisplayUpdate = millis();
            DisplayData dData;
            
            dData.batVoltage = PowerManager::getBatteryVoltage(smoothBatRaw);
            dData.batPercent = PowerManager::getBatteryPercentage(dData.batVoltage);
            
            // Ditch the stuck USB boolean. Use a strict physical voltage threshold.
            // Active chargers push the line above 4.15V. Resting batteries sit below 4.15V.
            dData.isCharging = (dData.batVoltage >= 4.15f); 
            
            dData.bleConnected = bleConnected;
            dData.bleHostConnected = BluetoothManager::isHostConnected;
            dData.blePedalConnected = BluetoothManager::isMvaveConnected;
            dData.isNuxConnected = BluetoothManager::isNuxConnected;
            
            dData.activeMode = activeDSP->activeMode;
            for(int i = 0; i < 10; i++) { dData.fxStates[i] = (&activeDSP->w)[i]; }
            dData.pb1 = pb1;  dData.pb2 = pb2;  dData.pb3 = pb3;  dData.cc11 = cc11;
            
            for(int i = 0; i < 5; i++) { 
                dData.paramVals[i] = activeDSP->params[activeDSP->activeMode % 10][i]; 
            }
            
            if (dData.activeMode == 0) {
                dData.paramVals[2] = activeDSP->fxMem[0];
                dData.paramVals[3] = activeDSP->fxMem[1];
            }

            dData.paramNames[0] = "P1"; dData.paramNames[1] = "P2"; dData.paramNames[2] = "P3"; 
            dData.paramNames[3] = "P4"; dData.paramNames[4] = "P5";

            dData.inMeter = inMeter; dData.outMeter = outMeter;
            dData.dspCoreLoad = dspLoad; dData.ctrlCoreLoad = ctrlLoad;
            dData.freeSRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            dData.freePSRAM = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            dData.sampleRate = sampleRate; dData.peakLatency = peakLatency;
            dData.underflows = underflows; dData.dmaCount = 0; 
            dData.stackWatermark = stackWatermark;
            
            dData.latencyMode = latencyMode.load(std::memory_order_relaxed);
            dData.isKnobEditMode = isKnobEditMode;
            dData.showBleWarning = showBleWarning;
            dData.showSavingScreen = showSavingScreen;
            dData.showKnobModeScreen = showKnobModeScreen;
            
            dData.isMonoPolyActive = isMonoPolyActiveFlag;
            dData.isVolumeMode = isVolumeMode;
            
            dData.whammyToe = activeDSP->fxMem[0];
            dData.whammyHeel = activeDSP->fxMem[1];
            
            strlcpy(dData.debugMsg, dbgMsg, sizeof(dData.debugMsg));

            dData.peakRxCyc = dbg_peak_rx_cyc.load(std::memory_order_relaxed);
            dData.peakDspCyc = dbg_peak_dsp_cyc.load(std::memory_order_relaxed);
            dData.peakTxCyc = dbg_peak_tx_cyc.load(std::memory_order_relaxed);
            dData.partialRx = dbg_partial_rx.load(std::memory_order_relaxed);
            dData.phaseTears = dbg_phase_tears.load(std::memory_order_relaxed);
            dData.rxErrors = dbg_rx_errors.load(std::memory_order_relaxed);
            dData.txErrors = dbg_tx_errors.load(std::memory_order_relaxed);
            
            displayManager.render(dData);
        }
#endif
    }
};