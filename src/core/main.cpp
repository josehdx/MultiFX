#include <Arduino.h>
#include <Preferences.h>
#include <Update.h>
#include "esp_log.h"

#define FW_VERSION "v5.00"

// Core System Includes
#include "SystemState.h"
#include "BoardHAL.h"
#include "FirmwareManager.h"
#include "USBMscManager.h"
#include "SystemBootloader.h"
#include "TelemetryManager.h"
#include "InputManager.h"

#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
DisplayManager displayManager;
#endif

void dispatchBleMidi(uint8_t header, uint8_t data1, uint8_t data2) {
    ChannelMessage cm(MIDIMessage(header, data1, data2, Cable_1));
    channelMessageCallback(cm);
}

void setup() {
    esp_log_level_set("task_wdt", ESP_LOG_NONE);
    
#if defined(TARGET_LILYGO)
    // Critical: Power on LilyGO peripheral & LCD rail immediately for battery boot
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);
#endif

    Serial.begin(115200);
    Serial0.begin(115200);
    BoardHAL::init();
    
    FirmwareManager::checkAndApplyUpdate();

    SystemBootloader::run();

#if defined(TARGET_LILYGO)
    displayManager.begin();
#endif

    USBMscManager::init();
}

void loop() {
    Control_Surface.loop();

#if !defined(FW_MODE_KNOBS_ONLY)
    BluetoothManager::process();
#endif

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
        while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
        
        AppSettings currentSettings;
        memset(&currentSettings, 0, sizeof(AppSettings));
        
        for(int i = 0; i < 10; i++) {
            currentSettings.fxMem[i] = effectMemory[i];
            for(int p = 0; p < 5; p++) {
                currentSettings.params[i][p] = fxParams[i][p];
            }
        }
        
        preferences.putBytes("dspData", &currentSettings, sizeof(AppSettings));
        settingsNeedSaving = false;
        
        dsp_is_paused.store(false, std::memory_order_release);
        if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
        if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
    }

    if (panicResetRequested.load(std::memory_order_acquire)) {
        panicResetRequested.store(false, std::memory_order_release);
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
}