#pragma once
#include <Arduino.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_continuous.h"

#if defined(TARGET_LILYGO)
    #include "PowerManager.h"
    #include "DisplayManager.h"
    #include <Control_Surface.h>
    #include "MidiRouter.h"
    #include "SpinlockGuard.h"
#elif defined(TARGET_BANANA)
    #include "BananaHardware.h"
    #include "SerialMonitor.h"
#endif

// Expose the global state variables from main.cpp
extern bool isKnobEditMode;
extern bool showBleWarning;

class BoardHAL {
private:
#if defined(TARGET_LILYGO)
    inline static DisplayManager displayManager;
    inline static FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar1{3};
    inline static FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar2{11};
    inline static FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar3{12};
    inline static FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar4{13};
    inline static FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar5{16};
    inline static int lastCcOut[5] = {-1, -1, -1, -1, -1};
    inline static unsigned long lastDisplayUpdate = 0;
#elif defined(TARGET_BANANA)
    inline static SerialMonitor serialMonitor;
    inline static unsigned long lastTelemetryPrint = 0;
#endif

public:
    static void init() {
#if defined(TARGET_LILYGO)
        FilteredAnalog<>::setupADC();
        displayManager.begin();
#elif defined(TARGET_BANANA)
        BananaHardware::initUART();
#endif
    }

    static i2s_std_config_t getI2SConfig(uint32_t sampleRate) {
#if defined(TARGET_LILYGO)
        return PowerManager::getI2SConfig(sampleRate);
#else
        return BananaHardware::getI2SConfig(sampleRate);
#endif
    }

    static void fetchADC(adc_continuous_handle_t handle, volatile bool& isPaused, volatile int& pb1, volatile int& pb2, volatile int& pb3, std::atomic<int>& bat) {
#if defined(TARGET_LILYGO)
        PowerManager::fetchADCDMA(handle, isPaused, pb1, pb2, pb3, bat);
#else
        BananaHardware::fetchADCDMA(handle, isPaused, pb1, pb2, pb3, bat);
#endif
    }

    static void updateExtraControls(int activeMode, volatile float* effectMemory, float fxParams[10][5], volatile bool& lutNeedsUpdate, volatile bool& dspNeedsCommit, std::atomic<int>& feedbackIntervalIdx, bool isKnobEditModeFlag) {
#if defined(TARGET_LILYGO)
        auto processKnob = [&](FilteredAnalog<12, 4, uint32_t, uint32_t>& knob, int ccNum, int idx) {
            if (knob.update()) {
                int ccVal = map(knob.getValue(), 0, 4095, 0, 127);
                if (ccVal != lastCcOut[idx]) {
                    Control_Surface.sendControlChange({(uint8_t)ccNum, Channel_1}, ccVal);
                    {
                        CriticalSectionGuard lock(MidiRouter::paramMux);
                        MidiRouter::updateParameter(ccNum, ccVal, activeMode, effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx);
                    }
                    lastCcOut[idx] = ccVal;
                }
            }
        };

        // PAR 1 (GPIO 3) lives on ADC1. It is immune to the BLE lock and always safe to poll.
        processKnob(filterPar1, 24, 0); 

        // PAR 2-5 live on ADC2. We strictly block them unless BLE is completely offline.
        if (isKnobEditModeFlag) {
            processKnob(filterPar2, 25, 1);
            processKnob(filterPar3, 26, 2); 
            processKnob(filterPar4, 27, 3);
            processKnob(filterPar5, 28, 4);
        }
#endif
    }

    template<typename DSPStateStruct>
    static void updateUI(DSPStateStruct* activeDSP, uint16_t pb1, uint16_t pb2, uint16_t pb3, uint16_t cc11,
                         float inMeter, float outMeter, float dspLoad, float ctrlLoad,
                         uint32_t sampleRate, uint32_t peakLatency, uint32_t underflows,
                         uint32_t stackWatermark, bool bleConnected, std::atomic<int>& latestBat) {
#if defined(TARGET_LILYGO)
        if (millis() - lastDisplayUpdate >= 33) {
            lastDisplayUpdate = millis();
            DisplayData dData;
            dData.batVoltage = PowerManager::getBatteryVoltage(latestBat.load(std::memory_order_relaxed));
            dData.batPercent = PowerManager::getBatteryPercentage(dData.batVoltage);
            dData.bleConnected = bleConnected;
            dData.activeMode = activeDSP->activeMode;
            for(int i=0; i<10; i++) dData.fxStates[i] = (&activeDSP->w)[i];
            dData.pb1 = pb1; dData.pb2 = pb2; dData.pb3 = pb3; dData.cc11 = cc11;
            for(int i=0; i<5; i++) dData.paramVals[i] = activeDSP->params[activeDSP->activeMode][i];
            dData.paramNames[0] = "P1"; dData.paramNames[1] = "P2"; dData.paramNames[2] = "P3"; dData.paramNames[3] = "P4"; dData.paramNames[4] = "P5";
            dData.inMeter = inMeter; dData.outMeter = outMeter;
            dData.dspCoreLoad = dspLoad; dData.ctrlCoreLoad = ctrlLoad;
            dData.freeSRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            dData.freePSRAM = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            dData.sampleRate = sampleRate; dData.peakLatency = peakLatency;
            dData.underflows = underflows; dData.dmaCount = 0; dData.stackWatermark = stackWatermark;
            
            // Pass warning flags to UI renderer
            dData.isKnobEditMode = isKnobEditMode;
            dData.showBleWarning = showBleWarning;

            displayManager.render(dData);
        }
#elif defined(TARGET_BANANA)
        if (millis() - lastTelemetryPrint >= 1000) {
            lastTelemetryPrint = millis();
            TelemetryData tData;
            tData.dspCoreLoad = dspLoad; tData.ctrlCoreLoad = ctrlLoad;
            tData.sampleRate = sampleRate; tData.latencyMode = activeDSP->latMode;
            tData.batVoltage = 4.00f; tData.batPercent = 100; tData.isCharging = false;
            tData.bleConnected = bleConnected; tData.activeMode = activeDSP->activeMode;
            for(int i=0; i<10; i++) tData.fxStates[i] = (&activeDSP->w)[i];
            tData.pb1 = pb1; tData.pb2 = pb2; tData.pb3 = pb3; tData.cc11 = cc11;
            tData.inMeter = inMeter; tData.outMeter = outMeter;
            tData.audioUnderflows = underflows; tData.dmaTransfers = 0;
            tData.dspStackWatermark = stackWatermark; tData.peakLoopLatency = peakLatency;
            serialMonitor.printMetrics(tData);
        }
#endif
    }
};