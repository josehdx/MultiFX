#define FW_VERSION "v4.32"
#include <Arduino.h>

// Master State Context (must be included before specific modules)
#include "SystemState.h"

// Decoupled Logic Files
#include "SystemActions.h"
#include "MidiHandler.h"
#include "LUTTask.h"
#include "AudioTask.h"
#include "AudioBufferManager.h"
#include "InputManager.h"
#include "StressTester.h"
#include "BluetoothManager.h"

// Hardware / Framework Includes
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_private/brownout.h"
#include "esp_pm.h"
#include "esp_task_wdt.h" 

#if defined(TARGET_BANANA)
    #define ENABLE_STRESS_TESTER true
#else
    #define ENABLE_STRESS_TESTER false 
#endif

void setup() {
    Serial.begin(115200);      
    BoardHAL::init(); 

#if defined(TARGET_BANANA)
    Serial.println("--- Native USB Serial Initialized ---");
#endif
    Serial.print("Firmware Version: ");
    Serial.println(FW_VERSION);

    esp_brownout_init(); 
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    esp_err_t err = nvs_flash_init(); 
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { 
        ESP_ERROR_CHECK(nvs_flash_erase()); err = nvs_flash_init(); 
    } 
    ESP_ERROR_CHECK(err);

    settingsMgr.init(preferences);
    isKnobEditMode = preferences.getBool("knobEditMode", false);

#if defined(FW_MODE_KNOBS_ONLY)
    isKnobEditMode = true;
#endif

#if !defined(FW_MODE_KNOBS_ONLY)
    if (!isKnobEditMode) {
        BluetoothManager::initHCI();
        btmidi = new BluetoothMIDI_Interface(); 
        btmidi->setName("Whammy_S3"); 
        Control_Surface >> pipes >> *btmidi; Control_Surface >> pipes >> usbmidi; usbmidi >> pipes >> Control_Surface; *btmidi >> pipes >> Control_Surface;
        Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr);
        Control_Surface.begin(); 
        BluetoothManager::configurePowerAndMac();
    } else {
        Control_Surface >> pipes >> usbmidi; usbmidi >> pipes >> Control_Surface;
        Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr);
        Control_Surface.begin();
    }
#else
    Control_Surface >> pipes >> usbmidi; usbmidi >> pipes >> Control_Surface;
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr);
    Control_Surface.begin();
#endif
    
#if defined(CS_USE_NIMBLE) && !defined(FW_MODE_KNOBS_ONLY)
    if (!isKnobEditMode) NimBLEDevice::setCustomGapHandler(nullptr); 
#endif

    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        esp_task_wdt_config_t wdt_cfg = { .timeout_ms = 30000, .idle_core_mask = (1 << 1), .trigger_panic = true }; esp_task_wdt_reconfigure(&wdt_cfg);
    #else
        esp_task_wdt_init(30, true);
    #endif

    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "DSP_Max_CPU", &dsp_cpu_lock); 
    if(dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);
    
    adc_continuous_handle_cfg_t adc_config={}; adc_config.max_store_buf_size=16384; adc_config.conv_frame_size=128; ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &multifx_adc_handle));
    adc_continuous_config_t dig_cfg={}; dig_cfg.sample_freq_hz = 2 * 1000; dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1; dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    adc_digi_pattern_config_t adc_pattern[5]={ {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_0,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_1,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_9,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_3,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_2,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH} };
    dig_cfg.pattern_num=5; dig_cfg.adc_pattern=adc_pattern; 
    ESP_ERROR_CHECK(adc_continuous_config(multifx_adc_handle, &dig_cfg)); ESP_ERROR_CHECK(adc_continuous_start(multifx_adc_handle));

    activeEffectMode.store(0, std::memory_order_release); latencyMode.store(constrain(preferences.getInt("latMode", 0), 0, 3), std::memory_order_release); isPB2WiperMode=preferences.getBool("pb2Wiper", false); isVolumeMode=false; currentSampleRate.store(96000, std::memory_order_release); freezeLength = 96000; feedbackIntervalIdx.store(constrain(preferences.getInt("fbIdx", 0), 0, 4), std::memory_order_release);
    
    AppSettings savedSettings; size_t len=preferences.getBytes("dspData", &savedSettings, sizeof(AppSettings));
    if(len==sizeof(AppSettings)) { for(int i=0; i<10; i++) { effectMemory[i]=savedSettings.fxMem[i]; for(int p=0; p<5; p++) fxParams[i][p]=savedSettings.params[i][p]; } }
    
    commitDSPState(); pedals.resetToCenter(); 
    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP); pinMode(BLE_TOGGLE_PIN, INPUT_PULLUP); lastActivityTime=millis();

    LUTManager::allocateAll(); LUTManager::generateStaticTables();
    AudioBufferManager::allocate();
    calibratePBs();
    
    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    LUTManager::updateDynamicLUT(activeDSP->cp, activeDSP->w, activeDSP->activeMode, effectMemory, activeDSP->fbIdx, currentSampleRate.load(std::memory_order_acquire));
    
    float* currLut = LUTManager::pitchShiftLUT.load(std::memory_order_acquire); 
    if (currLut) { memcpy(pitchLutBufferA, currLut, 16384 * sizeof(float)); pitchShiftFactor.store(pitchLutBufferA[8192], std::memory_order_release); }
    
    padVectorFilter.setLPF(1200.0f, (float)currentSampleRate.load(std::memory_order_acquire));
    I2SManager::initChannels(BoardHAL::getI2SConfig(currentSampleRate.load(std::memory_order_acquire)), HOP_SIZE);
    
    dspTaskStack = (StackType_t*)heap_caps_aligned_alloc(16, 8192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); dspTaskTCB = (StaticTask_t*)heap_caps_aligned_alloc(16, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (dspTaskStack != nullptr && dspTaskTCB != nullptr) { audioTaskHandle = xTaskCreateStaticPinnedToCore(AudioDSPTask, "DSP", 8192, NULL, configMAX_PRIORITIES - 1, dspTaskStack, dspTaskTCB, 0); } else { while(1) vTaskDelay(100); }
    
    lutTaskStack = (StackType_t*)heap_caps_aligned_alloc(16, 3072, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); lutTaskTCB = (StaticTask_t*)heap_caps_aligned_alloc(16, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (lutTaskStack != nullptr && lutTaskTCB != nullptr) { lutTaskHandle = xTaskCreateStaticPinnedToCore(LUTUpdateTask, "LUT_Task", 3072, NULL, 1, lutTaskStack, lutTaskTCB, 1); } else { while(1) vTaskDelay(100); }
    
    I2SManager::enableChannels();
    
    if (ENABLE_STRESS_TESTER) { static StressParams stressParams = { &latestPB1, &latestPB2, &latestPB3, switchEffectMode, triggerPanicReset, &sampleRateToggleRequested }; xTaskCreatePinnedToCore(StressTester::StressTask, "StressTask", 4096, &stressParams, 1, NULL, 1); }
}

void loop() {
    unsigned long loop_start_time = micros();
    static unsigned long lastLoopMicro = micros();
    static bool lastBtState=false;
    
    if (bleEnabled.load(std::memory_order_relaxed)) { Control_Surface.loop(); } else { Control_Surface.updateMidiInput(); }
    
    bool currentBtState = false;
#if !defined(FW_MODE_KNOBS_ONLY)
    if (!isKnobEditMode && btmidi != nullptr) { currentBtState = btmidi->isConnected(); }
#endif
    if(currentBtState!=lastBtState) { lastBtState=currentBtState; }

    InputManager::processInputs(currentBtState);

    if (!ENABLE_STRESS_TESTER) BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat);
    BoardHAL::updateExtraControls(activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx, isKnobEditMode, latestPar1);
    
    InputManager::processPedals();
    
    static unsigned long lastLutUpdate = 0;
    if(lutNeedsUpdate && (millis() - lastLutUpdate > (ENABLE_STRESS_TESTER ? 250 : 40))) { 
        lutNeedsUpdate = false; asyncLutUpdateRequested.store(true, std::memory_order_release); lastLutUpdate = millis(); 
    }
    
    if(sampleRateToggleRequested) { sampleRateToggleRequested=false; toggleSampleRate(); } if(pb2ToggleRequested) { pb2ToggleRequested=false; calibratePBs(); }
    if (dspNeedsCommit) { if (commitDSPState()) dspNeedsCommit = false; }
    
    unsigned long loopBusyTime = micros() - loop_start_time;
    unsigned long totalLoopTime = micros() - lastLoopMicro;
    lastLoopMicro = micros();
    if (totalLoopTime > 0) core1_ctrl_load.store(__builtin_fmaf(core1_ctrl_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, (((float)loopBusyTime / (float)totalLoopTime) * 100.0f)) * 0.05f), std::memory_order_relaxed);
    
    uint32_t iter_latency = (micros() - loop_start_time) / 1000; 
    if (iter_latency > max_loop_latency_ms.load(std::memory_order_relaxed)) max_loop_latency_ms.store(iter_latency, std::memory_order_relaxed);
    
#ifdef ENABLE_ADVANCED_TELEMETRY
    if (audioTaskHandle != NULL) dsp_stack_watermark.store(uxTaskGetStackHighWaterMark(audioTaskHandle) * sizeof(StackType_t), std::memory_order_relaxed);
    if (lutTaskHandle != NULL) lut_stack_watermark.store(uxTaskGetStackHighWaterMark(lutTaskHandle) * sizeof(StackType_t), std::memory_order_relaxed);
#endif

    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    uint32_t watermarkVal = 0;
#ifdef ENABLE_ADVANCED_TELEMETRY
    watermarkVal = lut_stack_watermark.load(std::memory_order_relaxed);
#endif

    BoardHAL::updateUI(activeDSP, currentPB1, currentPB2, currentPB3, currentCC11, ui_audio_level.load(std::memory_order_acquire), ui_output_level.load(std::memory_order_acquire), core0_dsp_load.load(std::memory_order_relaxed), core1_ctrl_load.load(std::memory_order_relaxed), currentSampleRate.load(std::memory_order_acquire), max_loop_latency_ms.exchange(0, std::memory_order_relaxed), audio_underflow_count.load(std::memory_order_relaxed), watermarkVal, currentBtState, latestBat);
    
    vTaskDelay(pdMS_TO_TICKS(5));
}