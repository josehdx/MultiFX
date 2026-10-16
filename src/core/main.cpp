#include <Arduino.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_log.h"
#define FW_VERSION "v5.00H"

// Core System Includes
#include "SystemState.h"
#include "BoardHAL.h"
#include "USBMscManager.h"
#include "SystemBootloader.h"
#include "TelemetryManager.h"
#include "InputManager.h"
#include "MidiHandler.h"

#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
DisplayManager displayManager;
#endif

#include "FirmwareManager.h"

void dispatchBleMidi(uint8_t header, uint8_t data1, uint8_t data2) {
    ChannelMessage cm(MIDIMessage(header, data1, data2, Cable_1));
    // Wrap the dispatch in the parameter mux to prevent RTOS collisions
    CriticalSectionGuard lock(MidiRouter::paramMux);
    channelMessageCallback(cm);
}

void setup() {
    esp_log_level_set("task_wdt", ESP_LOG_NONE);

#if defined(TARGET_LILYGO)
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);
    // Initialize display early so update logs appear on screen
    displayManager.begin();
#endif

    Serial.begin(115200);
    BoardHAL::init();
    
    // Check and apply OTA update with live TFT on-screen feedback
    FirmwareManager::checkAndApplyUpdate();
    
    USBMscManager::init();
    
    SystemBootloader::run();
}

void loop() {
    Control_Surface.loop();

#if !defined(FW_MODE_KNOBS_ONLY)
    BluetoothManager::process();
#endif

    if(millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT) goToLightSleep();

#if !ENABLE_STRESS_TESTER
    BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat);
#endif

    bool btConnected = false;
#if !defined(FW_MODE_KNOBS_ONLY)
    btConnected = BluetoothManager::isMvaveConnected || BluetoothManager::isHostConnected;
#endif

    InputManager::processInputs(btConnected);
    InputManager::processPedals();

    if (dspNeedsCommit) {
        if (commitDSPState()) {
            dspNeedsCommit = false;
        }
    }

    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    
    if (activeDSP) {
        uint32_t watermarkVal = 0;
#ifdef ENABLE_ADVANCED_TELEMETRY
        watermarkVal = lut_stack_watermark.load(std::memory_order_relaxed);
#endif

#if defined(TARGET_LILYGO)
        BoardHAL::updateUI(activeDSP, currentPB1, currentPB2, currentPB3, currentCC11,
                            ui_audio_level.load(std::memory_order_acquire),
                            ui_output_level.load(std::memory_order_acquire),
                            core0_dsp_load.load(std::memory_order_relaxed),
                            core1_ctrl_load.load(std::memory_order_relaxed),
                            currentSampleRate.load(std::memory_order_acquire),
                            max_loop_latency_ms.exchange(0, std::memory_order_relaxed),
                            audio_underflow_count.load(std::memory_order_relaxed),
                            watermarkVal, btConnected, latestBat,
                            isMonoPolyActive.load(std::memory_order_acquire));
#endif
        TelemetryManager::report(activeDSP);
    }

    if (settingsNeedSaving && (millis() - lastParameterChangeTime > 2000)) {
        dsp_is_paused.store(true, std::memory_order_release);
        
        // Explicitly wait for BOTH AudioTask and LUTUpdateTask to acknowledge park state
        while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) { 
             vTaskDelay(pdMS_TO_TICKS(1)); 
         }
        
        AppSettings currentSettings;
        memset(&currentSettings, 0, sizeof(AppSettings));
        uint16_t currentFxStates = 0;
        
        for(int i = 0; i < 10; i++) {
            currentSettings.fxMem[i] = effectMemory[i];
            for(int p = 0; p < 5; p++) {
                currentSettings.params[i][p] = fxParams[i][p];
            }
            if ((&activeDSP->w)[i]) currentFxStates |= (1 << i);
        }
        
        // Pass all global states into SettingsManager so Sample Rate persists
        settingsMgr.save(preferences, 
                         activeDSP->activeMode, 
                         activeDSP->latMode,
                         activeDSP->fbIdx, 
                         isPB2WiperMode, 
                         isVolumeMode, 
                         currentFxStates, 
                         currentSampleRate.load(std::memory_order_acquire),
                         &currentSettings, 
                         sizeof(AppSettings));
                         
        settingsNeedSaving = false;
        
        dsp_is_paused.store(false, std::memory_order_release);
        if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
        if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
        
        // Ensure tasks have safely unparked before re-entering main system loop
        while(dsp_ack_parked.load(std::memory_order_acquire) || lut_ack_parked.load(std::memory_order_acquire)) { 
             vTaskDelay(pdMS_TO_TICKS(1)); 
         }
    }

    if (panicResetRequested.load(std::memory_order_acquire)) {
        panicResetRequested.store(false, std::memory_order_release);
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
}