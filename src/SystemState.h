#pragma once

// Make sure all modules agree that telemetry is enabled
#define ENABLE_ADVANCED_TELEMETRY 

#include <Arduino.h>

#if defined(TARGET_BANANA)
    #define ENABLE_STRESS_TESTER true
#else
    #define ENABLE_STRESS_TESTER false 
#endif

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_continuous.h"
#include "esp_pm.h" // Fixes esp_pm_lock_handle_t undefined

#include <Control_Surface.h>
#include <NimBLEDevice.h>
#include "PedalManager.h"
#include "SettingsManager.h"
#include "DSPEngine.h"

#include "MonoPolyTracker.h"

#define HOP_SIZE 64
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 131072

struct __attribute__((aligned(64))) DSPCoreState {
    float fxMem[10]; float params[10][5]; int activeMode; int latMode; int fbIdx;
    bool w, fz, fb, hr, cp, sy, pd, ch, sw, vb; float vg; uint8_t _padding[52]; 
};

struct AppSettings {
    float fxMem[10];
    float params[10][5];
};

extern std::atomic<bool> isMonoPolyActive;
extern std::atomic<int> monoPolyAlgo; // 0 = TD-PSOLA, 1 = YIN Synth, etc.

extern VectorBiquadS3 padVectorFilter;
extern Preferences preferences;
extern SettingsManager settingsMgr;
extern volatile bool settingsNeedSaving;
extern volatile unsigned long lastParameterChangeTime;
extern float fxParams[10][5];
extern const bool INVERT_PB3;

extern std::atomic<bool> dsp_is_paused, dsp_ack_parked, lut_ack_parked, ui_clear_meters_requested, globalAudioResetRequested, panicResetRequested, bleEnabled;
extern std::atomic<float> pitchShiftFactor;
extern std::atomic<uint32_t> currentSampleRate;

extern bool isKnobEditMode, showBleWarning, showSavingScreen, showKnobModeScreen;
extern unsigned long warningTimer;
extern const float LATENCY_WINDOWS[];

extern int32_t *i2s_in_block, *i2s_out_block;
extern float *inBuf, *envBuf, *fzOutBuf, *masterGainBuf, *w1Buf, *w2Buf, *w3Buf, *padFilterBuf, *dryBuf, *fbOutBuf, *sMixBuf;
extern esp_pm_lock_handle_t dsp_cpu_lock;

extern int16_t dmaPingBuffer[HOP_SIZE];
extern int16_t dmaPongBuffer[HOP_SIZE];
extern int16_t* activeDmaReadBuf;
extern int16_t* activeDmaWriteBuf;
extern int16_t sramDryBlock[HOP_SIZE];

extern DSPCoreState dspStates[2];
extern std::atomic<DSPCoreState*> dspActiveState;
extern int dspWriteIndex;
extern volatile bool dspNeedsCommit;
extern std::atomic<bool> dspAckCommit;

extern volatile uint16_t lastActivePedal;
extern volatile float effectMemory[10];

extern volatile bool isWhammyActive, isFrozen, isFeedbackActive, isHarmonizerMode, isSynthMode, isPadMode, isCapoMode, isChorusMode, isSwellMode, isVibratoMode, isVolumeMode, isPB2WiperMode;
extern volatile float volumePedalGain;
extern std::atomic<int> latencyMode, activeEffectMode, feedbackIntervalIdx;

extern int16_t *sramPitchLow, *sramPitchHigh, *delayBuffer, *fbDelayBuffer, *freezeBuffer;
extern float *diffuserBuf;
extern int diffuserIdx;

extern TaskHandle_t audioTaskHandle, lutTaskHandle;
extern StackType_t *dspTaskStack, *lutTaskStack;
extern StaticTask_t *dspTaskTCB, *lutTaskTCB;

extern int writeIndex, fbDelayWriteIdx, sramWriteIdx;
extern std::atomic<bool> asyncLutUpdateRequested;
extern volatile bool lutNeedsUpdate;
extern float *pitchLutBufferA, *pitchLutBufferB;
extern std::atomic<float*> activePitchLUT;

extern uint32_t tap_w1_lo_1, tap_w1_lo_2, tap_w1_hi_1, tap_w1_hi_2;
extern uint32_t tap_w2_lo_1, tap_w2_lo_2, tap_w2_hi_1, tap_w2_hi_2;
extern uint32_t tap_w3_1, tap_w3_2, tap_w4_1, tap_w4_2, tap_w5_1, tap_w5_2;
extern float currentWindowSize;
extern int freezeLength;
extern bool wasFrozen;
extern volatile bool apfNeedsClear;
extern volatile float freezeRamp, chorusLfoPhase, feedbackLfoPhase, vibratoLfoPhase, swellGain, feedbackRamp;
extern float fbHpfState, feedbackFilter; // Removed from volatile group
extern std::atomic<int> hardwareSyncMuteFrames;
extern volatile bool sampleRateToggleRequested, pb2ToggleRequested;
extern unsigned long lastActivityTime;

extern std::atomic<float> core0_dsp_load, core1_ctrl_load;
extern std::atomic<uint32_t> max_loop_latency_ms;
extern std::atomic<float> ui_audio_level, ui_output_level;

#ifdef ENABLE_ADVANCED_TELEMETRY
    extern std::atomic<uint32_t> audio_underflow_count;
    extern std::atomic<uint32_t> dsp_stack_watermark;
    extern std::atomic<uint32_t> lut_stack_watermark;
#endif

extern volatile bool isAdcPaused;
extern adc_continuous_handle_t multifx_adc_handle;
extern volatile int latestPB1, latestPB2, latestPB3, latestPar1;
extern std::atomic<int> latestBat;

extern const int BOOT_SENSE_PIN, BLE_TOGGLE_PIN;
extern volatile uint16_t currentPB1, currentPB2, currentPB3, currentCC11;

#if !defined(FW_MODE_KNOBS_ONLY)
extern BluetoothMIDI_Interface* btmidi;
#endif
extern USBMIDI_Interface usbmidi;
extern MIDI_PipeFactory<4> pipes;
extern PedalManager pedals;