#include <Arduino.h>
#include "esp_log.h"

#define FW_VERSION "v4.92"

// Core System Includes
#include "SystemState.h"
#include "BoardHAL.h"
#include "FirmwareManager.h"
#include "USBMscManager.h"
#include "SystemBootloader.h"
#include "TelemetryManager.h"
#include "InputManager.h"

// Instantiate DisplayManager when building for LilyGO
#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
DisplayManager displayManager;
#endif

void setup() {
    // Silence harmless task_wdt reset warnings during high-load stress testing
    esp_log_level_set("task_wdt", ESP_LOG_NONE);

    Serial.begin(115200); 
    Serial0.begin(115200);
    
    BoardHAL::init();             
    
    Serial.println("\n--- HARDWARE SYSTEM INITIALIZED ---");
    Serial.print("Firmware Version: ");
    Serial.println(FW_VERSION);

    Serial0.println("\n--- HARDWARE UART0 INITIALIZED ---");
    Serial0.print("Firmware Version: ");
    Serial0.println(FW_VERSION);

    // 1. Run OTA check BEFORE USB MSC mounts
    FirmwareManager::checkAndApplyUpdate();

    // 2. Initialize USB Storage
    USBMscManager::init();

    // 3. Complete system boot sequence
    SystemBootloader::run();
}

void loop() {
    Control_Surface.loop();

    // 1. Fetch raw ADC values from hardware DMA (when stress tester is disabled)
#if !ENABLE_STRESS_TESTER
    BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat);
#endif

    // 2. Map raw ADC / Stress-test values through PedalManager calibration & process GPIO buttons
    bool btConnected = false;
#if !defined(FW_MODE_KNOBS_ONLY)
    if (btmidi) btConnected = btmidi->isConnected();
#endif
    InputManager::processInputs(btConnected);
    InputManager::processPedals();

    // 3. Commit pending DSP state changes (syncs MonoPoly/Whammy state overrides)
    if (dspNeedsCommit) {
        if (commitDSPState()) {
            dspNeedsCommit = false;
        }
    }

    // 4. Update LilyGO TFT display if enabled
    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    if (activeDSP) {
        uint32_t watermarkVal = 0;
#ifdef ENABLE_ADVANCED_TELEMETRY
        watermarkVal = lut_stack_watermark.load(std::memory_order_relaxed);
#endif
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

        // 5. Output periodic serial telemetry
        TelemetryManager::report(activeDSP);
    }

    // 6. Auto-save settings to NVS
    if (settingsNeedSaving && (millis() - lastParameterChangeTime > 2000)) {
        AppSettings currentSettings;
        for(int i = 0; i < 10; i++) { 
            currentSettings.fxMem[i] = effectMemory[i]; 
            for(int p = 0; p < 5; p++) {
                currentSettings.params[i][p] = fxParams[i][p]; 
            }
        }
        preferences.putBytes("dspData", &currentSettings, sizeof(AppSettings));
        settingsNeedSaving = false;
        Serial0.println("[SYSTEM] Settings auto-saved to NVS.");
    }

    // 7. Panic reset handling
    if (panicResetRequested.load(std::memory_order_acquire)) {
        Serial0.println("[SYSTEM] Executing Panic Reset...");
        panicResetRequested.store(false, std::memory_order_release);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}