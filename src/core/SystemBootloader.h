#pragma once

#include <Arduino.h>
#include "esp_task_wdt.h"
#include "SystemState.h"
#include "SettingsManager.h"
#include "BananaHardware.h"
#include "LUTManager.h"
#include "AudioBufferManager.h"
#include "I2SManager.h"
#include "BluetoothManager.h"
#include "SystemActions.h"
#include "MidiHandler.h"
#include "StressTester.h"
#include "AudioTask.h"
#include "LUTTask.h"

class SystemBootloader {
public:
    static void run() {
        Serial0.println("[INIT 1/8] Initializing NVS Flash...");
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
        settingsMgr.init(preferences);
        
        isKnobEditMode = preferences.getBool("knobEditMode", false);
        
        // --- REMOVED THE HARDCODED "FORCE BLE ON" OVERRIDE HERE ---

        Serial0.println("[INIT 2/8] Setting up Control Surface MIDI & Bluetooth...");
        if (!isKnobEditMode) {
            Control_Surface >> pipes >> usbmidi;
            usbmidi >> pipes >> Control_Surface;
            Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr);
            Control_Surface.begin();
            
            BluetoothManager::initDualRole();
        } else {
            Control_Surface >> pipes >> usbmidi;
            usbmidi >> pipes >> Control_Surface;
            Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr);
            Control_Surface.begin();
        }

        Serial0.println("[INIT 3/8] Allocating System Tables, Audio Buffers & Task Stacks...");
        dspTaskStack = (StackType_t*)heap_caps_aligned_alloc(16, 8192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        dspTaskTCB = (StaticTask_t*)heap_caps_aligned_alloc(16, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        lutTaskStack = (StackType_t*)heap_caps_aligned_alloc(16, 3072, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        lutTaskTCB   = (StaticTask_t*)heap_caps_aligned_alloc(16, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        LUTManager::allocateAll();
        LUTManager::generateStaticTables();
        AudioBufferManager::allocate();
        monoPolyEngine.allocateBuffers();
        padVectorFilter.setLPF(1200.0f, (float)currentSampleRate.load(std::memory_order_acquire));

        I2SManager::initChannels(BoardHAL::getI2SConfig(currentSampleRate.load(std::memory_order_acquire)), HOP_SIZE);

        Serial0.println("[INIT 4/8] Setting up Analog Inputs...");
        analogReadResolution(12);

        Serial0.println("[INIT 5/8] Configuring Power Lock...");
        esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "DSP_Max_CPU", &dsp_cpu_lock);
        if(dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);

        activeEffectMode.store(0, std::memory_order_release);
        latencyMode.store(constrain(preferences.getInt("latMode", 0), 0, 3), std::memory_order_release);
        isPB2WiperMode = preferences.getBool("pb2Wiper", false);
        isVolumeMode = false;
        currentSampleRate.store(96000, std::memory_order_release);
        freezeLength = 96000;
        feedbackIntervalIdx.store(constrain(preferences.getInt("fbIdx", 0), 0, 4), std::memory_order_release);

        AppSettings savedSettings;
        size_t len = preferences.getBytes("dspData", &savedSettings, sizeof(AppSettings));
        if(len == sizeof(AppSettings)) {
            for(int i = 0; i < 10; i++) {
                effectMemory[i] = savedSettings.fxMem[i];
                for(int p = 0; p < 5; p++) fxParams[i][p] = savedSettings.params[i][p];
            }
        }

        commitDSPState();

        pedals.resetToCenter();
        pinMode(BOOT_SENSE_PIN, INPUT_PULLUP);
        pinMode(BLE_TOGGLE_PIN, INPUT_PULLUP);
        lastActivityTime = millis();

        Serial0.println("[INIT 6/8] Calibrating Expression Pedals...");
        calibratePBs();

        DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
        LUTManager::updateDynamicLUT(activeDSP->cp, activeDSP->w, activeDSP->activeMode, effectMemory, activeDSP->fbIdx, currentSampleRate.load(std::memory_order_acquire));
        float* currLut = LUTManager::pitchShiftLUT.load(std::memory_order_acquire);
        if (currLut) pitchShiftFactor.store(currLut[8192], std::memory_order_release);

        Serial0.println("[INIT 7/8] Spawning Core 0 (Audio) and Core 1 (LUT) FreeRTOS Tasks...");
        I2SManager::enableChannels();

        if (dspTaskStack != nullptr && dspTaskTCB != nullptr) {
            audioTaskHandle = xTaskCreateStaticPinnedToCore(AudioDSPTask, "DSP", 8192, NULL, 20, dspTaskStack, dspTaskTCB, 0);
        }
        if (lutTaskStack != nullptr && lutTaskTCB != nullptr) {
            lutTaskHandle = xTaskCreateStaticPinnedToCore(LUTUpdateTask, "LUT_Task", 3072, NULL, 1, lutTaskStack, lutTaskTCB, 1);
        }

#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        esp_task_wdt_config_t twdt_config = {
            .timeout_ms = 30000,
            .idle_core_mask = (1 << 0) | (1 << 1),
            .trigger_panic = false,
        };
        esp_task_wdt_reconfigure(&twdt_config);
#else
        esp_task_wdt_init(30, false);
#endif
        Serial0.println("--- SETUP COMPLETE: ENTERING MAIN SYSTEM LOOP ---");
    }
};