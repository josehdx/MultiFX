#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <driver/i2s_std.h> 
#include "driver/gpio.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_biquad.h"
#include "driver/rtc_io.h"
#include "esp_bt.h"
#include <math.h>
#include <Preferences.h>

#include "PedalManager.h" // 🔒 The new locked hardware class

// --- HARDWARE CONFIGURATION TOGGLES ---
#define ENABLE_PAR_KNOBS false  

// --- MEMORY PREFERENCES ---
struct AppSettings {
    float fxMem[10];
    float params[10][5];
};

Preferences preferences;
volatile bool settingsNeedSaving = false;
volatile unsigned long lastParameterChangeTime = 0; 

// --- DYNAMIC FX PARAMETERS MATRIX ---
volatile float fxParams[10][5] = {
    {0.0f, 1.0f, 0.0f, 0.0f, 0.0f},            // 0: Whammy (Dry Mix, Wet Mix)
    {0.6f, 0.0002f, 0.00005f, 0.0f, 0.0f},     // 1: Freeze (APF Coeff, Attack, Release)
    {5120.0f, 30.0f, 0.02f, 0.0f, 0.0f},       // 2: Feedback (LFO Speed, Drive, DelayOffset)
    {0.5f, 0.0f, 0.0f, 0.0f, 0.0f},            // 3: Harmony (Mix)
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},            // 4: Capo (N/A)
    {0.1f, 0.005f, 0.3f, 0.1f, 0.0f},          // 5: Synth (Env Attack, Env Rel, Filter Base, Out Mix)
    {0.95f, 1.5f, 0.0f, 0.0f, 0.0f},           // 6: Pad (Smoothing, Mix)
    {1536.0f, 0.4f, 0.0f, 0.0f, 0.0f},         // 7: Chorus (LFO Speed, Mix)
    {0.015f, 0.00002f, 0.00005f, 0.0f, 0.0f},  // 8: Swell (Threshold, Attack, Release)
    {1.0f, 0.0f, 0.0f, 0.0f, 0.0f}             // 9: Vibrato (Depth Scalar)
};

const bool INVERT_PB3 = false; 

void __attribute__((constructor)) pre_boot_kill_switch() {
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_38, 0); 
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_15, 0);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_5, 0);
}

void updateLUT();
void DisplayTask(void * pvParameters);
void MidiTask(void * pvParameters);
void AudioDSPTask(void * pvParameters);
bool channelMessageCallback(ChannelMessage cm);

i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;
SemaphoreHandle_t audioBufferMutex = NULL;

volatile uint32_t currentSampleRate = 48000; 
#define HOP_SIZE 64            
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF 
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 131072

float* delayBuffer = nullptr;    
float* fbDelayBuffer = nullptr;  
float* freezeBuffer = nullptr;   

float* volatile pitchShiftLUT = nullptr; 
float* pitchShiftLUT_temp = nullptr; 

int writeIndex = 0; int fbDelayWriteIdx = 0;

#define HANN_LUT_SIZE 1024
#define LFO_LUT_SIZE 1024
#define WAVE_LUT_SIZE 2048
DRAM_ATTR float hannLUT[HANN_LUT_SIZE];
DRAM_ATTR float lfoLUT[LFO_LUT_SIZE];
DRAM_ATTR float synthLUT[WAVE_LUT_SIZE];

volatile float globalHarmRatio = 1.0f; volatile float globalChorusRatio = 1.0f;
volatile float globalFbRatio = 1.0f; volatile float globalVibratoPhaseInc = 0.0f;

uint32_t tap_w1_1 = 0; uint32_t tap_w1_2 = 256 << 16; 
uint32_t tap_w2_1 = 0; uint32_t tap_w2_2 = 256 << 16; 
uint32_t tap_w3_1 = 0; uint32_t tap_w3_2 = 256 << 16; 
uint32_t tap_w4_1 = 0; uint32_t tap_w4_2 = 256 << 16; 
uint32_t tap_w5_1 = 0; uint32_t tap_w5_2 = 256 << 16; 
float currentWindowSize = 512.0f; 

int freezeLength = 48000; bool wasFrozen = false;
volatile bool apfNeedsClear = false; volatile float freezeRamp = 0.0f;
float apf1Buffer[1009] = { 0.0f }; int apf1Idx = 0;
float apf2Buffer[863] = { 0.0f };  int apf2Idx = 0;

volatile int feedbackIntervalIdx = 0; 
volatile bool lutNeedsUpdate = false;
volatile uint16_t lastActivePedal = 8192; 

TaskHandle_t audioTaskHandle = NULL; 

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft); 
TFT_eSprite meterSpr = TFT_eSprite(&tft); 
volatile bool forceUIUpdate = true; 

volatile int activeEffectMode = 0; 
volatile float effectMemory[10] = { 12.0f, -12.0f, 0.0f, 5.0f, -2.0f, -12.0f, -12.0f, 12.0f, 0.0f, 0.0f };
volatile float pitchShiftFactor = 1.0f;

volatile bool isWhammyActive = true;  volatile bool isFrozen = false; volatile bool isFeedbackActive = false;
volatile bool isHarmonizerMode = false; volatile bool isSynthMode = false; volatile bool isPadMode = false;
volatile bool isCapoMode = false; volatile bool isChorusMode = false; volatile bool isSwellMode = false; 
volatile bool isVibratoMode = false; volatile bool isVolumeMode = false; volatile bool isPB2WiperMode = false; 

volatile float chorusLfoPhase = 0.0f; volatile float feedbackLfoPhase = 0.0f; volatile float vibratoLfoPhase = 0.0f;
volatile float swellGain = 0.0f; volatile float volumePedalGain = 1.0f; 
volatile float feedbackRamp = 0.0f; float fbHpfState = 0.0f; float feedbackFilter = 0.0f;
volatile int latencyMode = 0; const float LATENCY_WINDOWS[] = {512.0f, 1024.0f, 2048.0f, 4096.0f};

volatile bool globalAudioResetRequested = false; volatile bool clearBuffersRequested = false; volatile int hardwareSyncMuteFrames = 0; 

unsigned long lastActivityTime = 0; unsigned long lastScreenActivityTime = 0;

const unsigned long LIGHT_SLEEP_TIMEOUT = 25000; const unsigned long SCREEN_OFF_TIMEOUT = 15000;  

bool isScreenOff = false; volatile bool wakeupPending = false; volatile float core1_load = 0.0f; 
volatile bool sleepRequested = false; volatile bool isSleeping = false;
const int BATTERY_PIN = 4; 
volatile int currentBatteryPercent = 100; 
volatile float currentBatteryVoltage = 4.00f; 
volatile bool isBatteryCharging = false;

pin_t pinPB = 1; pin_t pinPB2 = 2; pin_t pinPB3 = 10;    
const int BOOT_SENSE_PIN = 0; 

pin_t pinPar1 = 3; pin_t pinPar2 = 11; pin_t pinPar3 = 12; pin_t pinPar4 = 13; pin_t pinPar5 = 14; 

uint16_t lastMidiSent = 8192; volatile uint16_t currentPB1 = 8192; volatile uint16_t currentPB2 = 8192;
volatile uint16_t currentPB3 = 8192; volatile uint16_t currentCC11 = 0;
volatile float ui_audio_level = 0.0f; volatile float ui_output_level = 0.0f;

FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB2 = pinPB2;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB3 = pinPB3;

FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar1 = pinPar1;
FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar2 = pinPar2;
FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar3 = pinPar3;
FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar4 = pinPar4;
FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar5 = pinPar5;

BluetoothMIDI_Interface btmidi; USBMIDI_Interface usbmidi; MIDI_PipeFactory<4> pipes;

// Global Pedal Manager Instance
PedalManager pedals;

void switchEffectMode(int newMode) {
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    activeEffectMode = (newMode % 10 + 10) % 10;
    chorusLfoPhase = 0.0f; 
    feedbackLfoPhase = 0.0f; 
    vibratoLfoPhase = 0.0f; 
    
    isWhammyActive = true; 
    isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false;
    isSynthMode = false; isPadMode = false; isCapoMode = false;
    isChorusMode = false; isSwellMode = false; isVibratoMode = false;

    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    lutNeedsUpdate = true; 
    forceUIUpdate = true; 
    settingsNeedSaving = true; 
    lastParameterChangeTime = millis();
}

void saveSettings() {
    AppSettings currentSettings;
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    for(int i = 0; i < 10; i++) {
        currentSettings.fxMem[i] = effectMemory[i];
        for (int p = 0; p < 5; p++) {
            currentSettings.params[i][p] = fxParams[i][p];
        }
    }
    
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    preferences.begin("whammy_cfg", false); 
    preferences.putInt("activeMode", activeEffectMode); 
    preferences.putInt("latMode", latencyMode);
    preferences.putBool("pb2Wiper", isPB2WiperMode); 
    preferences.putUInt("sampleRate", currentSampleRate);
    preferences.putInt("fbIdx", constrain((int)feedbackIntervalIdx, 0, 4)); 
    preferences.putBytes("dspData", &currentSettings, sizeof(AppSettings));
    preferences.end();
}

int getBatteryPercentage(float voltage) {
    float clampedVolts = fmaxf(3.30f, fminf(4.15f, voltage));
    if (clampedVolts >= 4.15f) return 100;
    if (clampedVolts <= 3.30f) return 0;
    if (clampedVolts >= 4.00f) return 90 + (int)((clampedVolts - 4.00f) / 0.15f * 10.0f);
    if (clampedVolts >= 3.90f) return 80 + (int)((clampedVolts - 3.90f) / 0.10f * 10.0f);
    if (clampedVolts >= 3.80f) return 70 + (int)((clampedVolts - 3.80f) / 0.10f * 10.0f); 
    if (clampedVolts >= 3.75f) return 60 + (int)((clampedVolts - 3.75f) / 0.05f * 10.0f);
    if (clampedVolts >= 3.70f) return 50 + (int)((clampedVolts - 3.70f) / 0.05f * 10.0f);
    if (clampedVolts >= 3.65f) return 40 + (int)((clampedVolts - 3.65f) / 0.05f * 10.0f);
    if (clampedVolts >= 3.60f) return 30 + (int)((clampedVolts - 3.60f) / 0.05f * 10.0f);
    if (clampedVolts >= 3.55f) return 20 + (int)((clampedVolts - 3.55f) / 0.05f * 10.0f);
    if (clampedVolts >= 3.50f) return 10 + (int)((clampedVolts - 3.50f) / 0.05f * 10.0f);
    return (int)((clampedVolts - 3.30f) / 0.20f * 10.0f);
}

void calibratePBs() {
    for (int i = 0; i < 50; i++) { filterPB.update(); filterPB2.update(); filterPB3.update(); delay(1); }
    long sum1 = 0; long sum2 = 0; long sum3 = 0;
    for (int i = 1; i <= 250; i++) { 
        filterPB.update(); filterPB2.update(); filterPB3.update(); 
        sum1 += filterPB.getValue(); sum2 += filterPB2.getValue(); sum3 += filterPB3.getValue();
        delay(1); 
    }
    
    pedals.setCenters(sum1 / 250, sum2 / 250, sum3 / 250);
}

void toggleSampleRate() {
    sleepRequested = true; globalAudioResetRequested = true; int timeoutCounter = 0;
    while (!isSleeping && timeoutCounter < 40) { vTaskDelay(pdMS_TO_TICKS(5)); timeoutCounter++; }

    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);

    i2s_channel_disable(tx_chan); i2s_channel_disable(rx_chan); 
    vTaskDelay(pdMS_TO_TICKS(50)); 
    
    i2s_del_channel(tx_chan); 
    i2s_del_channel(rx_chan);
    
    if (currentSampleRate == 96000) currentSampleRate = 48000; else currentSampleRate = 96000;
    
    i2s_chan_config_t i2sConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); 
    i2sConfig.dma_desc_num = 8; i2sConfig.dma_frame_num = HOP_SIZE; i2sConfig.auto_clear = true;
    i2s_new_channel(&i2sConfig, &tx_chan, &rx_chan);
    
    i2s_std_config_t stdConfig = { 
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate), 
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg = { .mclk = GPIO_NUM_43, .bclk = GPIO_NUM_44, .ws = GPIO_NUM_18, .dout = GPIO_NUM_16, .din = GPIO_NUM_17 } 
    };
    
    i2s_channel_init_std_mode(tx_chan, &stdConfig); 
    i2s_channel_init_std_mode(rx_chan, &stdConfig); 
    
    freezeLength = currentSampleRate; lutNeedsUpdate = true;
    
    i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); 
    
    size_t dummyBytes; static int32_t dummyBuf[HOP_SIZE * 2];
    for(int k = 0; k < 10; k++) {
        i2s_channel_read(rx_chan, dummyBuf, sizeof(dummyBuf), &dummyBytes, pdMS_TO_TICKS(5));
    }

    hardwareSyncMuteFrames = (currentSampleRate / HOP_SIZE) * 0.15f;

    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    pedals.triggerSystemRecovery();

    sleepRequested = false; forceUIUpdate = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
}

void turnScreenOff() { if (!isScreenOff) { digitalWrite(38, LOW); digitalWrite(15, LOW); isScreenOff = true; } }
void turnScreenOn() { if (isScreenOff && !wakeupPending) wakeupPending = true; }

void goToLightSleep() {
    turnScreenOff(); sleepRequested = true; int timeoutCounter = 0;
    while (!isSleeping && timeoutCounter < 10) { vTaskDelay(pdMS_TO_TICKS(10)); timeoutCounter++; }

    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    i2s_channel_disable(tx_chan); i2s_channel_disable(rx_chan);
    i2s_del_channel(tx_chan); 
    i2s_del_channel(rx_chan);
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    int initA = filterPB.getValue(); int initB = filterPB2.getValue(); int initC = filterPB3.getValue();
    while (digitalRead(BOOT_SENSE_PIN) == HIGH) {
        vTaskDelay(pdMS_TO_TICKS(50));
        filterPB.update(); filterPB2.update(); filterPB3.update();
        if (abs((int)filterPB.getValue() - initA) > 150) break;
        if (abs((int)filterPB2.getValue() - initB) > 150) break;
        if (abs((int)filterPB3.getValue() - initC) > 150) break;
    }

    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    
    i2s_chan_config_t i2sConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); 
    i2sConfig.dma_desc_num = 8; i2sConfig.dma_frame_num = HOP_SIZE; i2sConfig.auto_clear = true;
    i2s_new_channel(&i2sConfig, &tx_chan, &rx_chan);
    
    i2s_std_config_t stdConfig = { 
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate), 
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg = { .mclk = GPIO_NUM_43, .bclk = GPIO_NUM_44, .ws = GPIO_NUM_18, .dout = GPIO_NUM_16, .din = GPIO_NUM_17 } 
    };
    
    i2s_channel_init_std_mode(tx_chan, &stdConfig); 
    i2s_channel_init_std_mode(rx_chan, &stdConfig); 
    
    i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); 
    
    size_t dummyBytes; static int32_t dummyBuf[HOP_SIZE * 2];
    for(int k = 0; k < 10; k++) {
        i2s_channel_read(rx_chan, dummyBuf, sizeof(dummyBuf), &dummyBytes, pdMS_TO_TICKS(5));
    }

    hardwareSyncMuteFrames = (currentSampleRate / HOP_SIZE) * 0.15f;
    globalAudioResetRequested = true;
    
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    sleepRequested = false; timeoutCounter = 0;
    while (isSleeping && timeoutCounter < 10) { vTaskDelay(pdMS_TO_TICKS(10)); timeoutCounter++; }
    vTaskDelay(pdMS_TO_TICKS(200)); 
    
    if (isScreenOff) turnScreenOn(); 
    
    pedals.triggerSystemRecovery(); // Reset tracking boundaries after power surge

    lastActivityTime = millis();
    lastScreenActivityTime = millis(); 
}

inline float IRAM_ATTR processTap(uint32_t tapPhase, const float* buffer, int currentWriteIdx, uint32_t windowMask, uint32_t hannIntMult) {
    int T = (tapPhase >> 16) & windowMask; float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
    int effTap = T + 2; 
    int idx1 = (currentWriteIdx - effTap + MAX_BUFFER_SIZE) & BUFFER_MASK; int idx0 = (idx1 + 1) & BUFFER_MASK; 
    int idx2 = (idx1 - 1 + MAX_BUFFER_SIZE) & BUFFER_MASK; int idx3 = (idx1 - 2 + MAX_BUFFER_SIZE) & BUFFER_MASK;
    float y0 = buffer[idx0]; float y1 = buffer[idx1]; float y2 = buffer[idx2]; float y3 = buffer[idx3];
    float c0 = y1; float c1 = 0.5f * (y2 - y0); float c3 = 1.5f * (y1 - y2) + 0.5f * (y3 - y0); float c2 = y0 - y1 + c1 - c3;
    float sample = ((c3 * frac + c2) * frac + c1) * frac + c0; int lutIdx = (T * hannIntMult) >> 16;
    return sample * hannLUT[lutIdx];
}

void updateLUT() {
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    float basePitch = 0.0f; if (isCapoMode || (activeEffectMode == 4 && isWhammyActive)) basePitch += effectMemory[4]; 
    float toeBend = effectMemory[0]; 
    float heelBend = effectMemory[1]; 
    float harmRatioMem = effectMemory[3];
    float chorusRatioMem = effectMemory[7];
    float vibHzMem = effectMemory[9];
    int fbIntervalIdxLocal = feedbackIntervalIdx;
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    for (int i = 0; i < 16384; i++) {
        float normalizedThrow = (i >= 8192) ? ((float)(i - 8192) / 8191.0f) : ((float)(i - 8192) / 8192.0f);
        float dynamicBend = (normalizedThrow >= 0.0f) ? (toeBend * normalizedThrow) : (heelBend * fabsf(normalizedThrow));
        pitchShiftLUT_temp[i] = powf(2.0f, (basePitch + dynamicBend) / 12.0f);
        if (i % 2048 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
    }
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    float* tempPtr = pitchShiftLUT;
    pitchShiftLUT = pitchShiftLUT_temp;
    pitchShiftLUT_temp = tempPtr;

    globalHarmRatio = powf(2.0f, harmRatioMem / 12.0f); 
    globalChorusRatio = powf(2.0f, chorusRatioMem / 12.0f); 
    float fbIntervals[5] = {0.0f, 12.0f, 19.0f, 24.0f, 28.0f}; 
    globalFbRatio = powf(2.0f, fbIntervals[constrain(fbIntervalIdxLocal, 0, 4)] / 12.0f);
    
    float vibHz = (vibHzMem != 0.0f) ? fabsf(vibHzMem) : 2.0f; 
    globalVibratoPhaseInc = (vibHz * LFO_LUT_SIZE) / (float)currentSampleRate;

    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
}

void updateMeters() {
    int barHeight = 98;
    int inFillHeight = constrain((int)(ui_audio_level * barHeight), 0, barHeight); 
    meterSpr.fillSprite(TFT_BLACK); meterSpr.fillRect(0, barHeight - inFillHeight, 6, inFillHeight, (ui_audio_level > 0.90f) ? TFT_RED : TFT_GREEN); meterSpr.pushSprite(11, 31);
    int outFillHeight = constrain((int)(ui_output_level * barHeight), 0, barHeight); 
    meterSpr.fillSprite(TFT_BLACK); meterSpr.fillRect(0, barHeight - outFillHeight, 6, outFillHeight, (ui_output_level > 0.90f) ? TFT_RED : TFT_GREEN); meterSpr.pushSprite(spr.width() - 17, 31);
}

// --- UI HELPER: CIRCULAR GAUGES ---
void drawCircularGauge(TFT_eSprite& sprite, int x, int y, int radius, float value, const char* label, const char* valStr, uint32_t color, int type) {
    int thickness = 3;
    sprite.drawArc(x, y, radius, radius - thickness, 210, 150, TFT_DARKGREY, TFT_BLACK, true);
    
    if (type == 1) { 
        float clamped = fmaxf(-1.0f, fminf(1.0f, value));
        int sweep = (int)(clamped * 150.0f);
        if (sweep > 0) sprite.drawArc(x, y, radius, radius - thickness, 0, sweep, color, TFT_BLACK, true);
        else if (sweep < 0) sprite.drawArc(x, y, radius, radius - thickness, 360 + sweep, 360, color, TFT_BLACK, true);
    } else if (type == 2) { 
        float clamped = fmaxf(0.0f, fminf(1.0f, value));
        int sweep = (int)(clamped * 300.0f);
        if (sweep > 0) {
            int startAngle = (150 - sweep + 360) % 360;
            sprite.drawArc(x, y, radius, radius - thickness, startAngle, 150, color, TFT_BLACK, true);
        }
    } else { 
        float clamped = fmaxf(0.0f, fminf(1.0f, value));
        int sweep = (int)(clamped * 300.0f);
        if (sweep > 0) {
            int endAngle = (210 + sweep) % 360;
            sprite.drawArc(x, y, radius, radius - thickness, 210, endAngle, color, TFT_BLACK, true);
        }
    }
    sprite.setTextDatum(MC_DATUM); sprite.setTextColor(TFT_WHITE, TFT_BLACK); sprite.setTextSize(1);
    sprite.drawString(valStr, x, y - 4);
    sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK); sprite.drawString(label, x, y + 14);
}

struct GaugeDef {
    int type; 
    float normVal;
    const char* label;
    char valStr[16];
};

void updateDisplay() {
    spr.fillSprite(TFT_BLACK); 
    
    char batStr[20];
    if (isBatteryCharging) { 
        sprintf(batStr, "CHG %.2fV %d%%", currentBatteryVoltage, currentBatteryPercent); 
        spr.setTextColor(TFT_GREEN, TFT_BLACK); 
    } else { 
        sprintf(batStr, "%.2fV %d%%", currentBatteryVoltage, currentBatteryPercent); 
        spr.setTextColor((currentBatteryPercent > 20) ? TFT_GREEN : TFT_RED, TFT_BLACK); 
    }
    spr.setTextDatum(TL_DATUM); spr.setTextSize(1); spr.drawString(batStr, 5, 2); 

    spr.setTextDatum(TR_DATUM); 
    if (btmidi.isConnected()) { spr.setTextColor(TFT_GREEN, TFT_BLACK); spr.drawString("BT: CONN", 315, 2); } 
    else { spr.setTextColor(TFT_YELLOW, TFT_BLACK); spr.drawString("BT: WAIT", 315, 2); }

    const char* effectTitleNames[] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};
    uint32_t effectTitleColors[] = {TFT_ORANGE, TFT_CYAN, TFT_RED, TFT_MAGENTA, TFT_GREEN, TFT_YELLOW, TFT_PINK, TFT_SKYBLUE, TFT_WHITE, TFT_PURPLE};
    spr.setTextDatum(MC_DATUM); spr.setTextSize(2); spr.setTextColor(effectTitleColors[activeEffectMode], TFT_BLACK); 
    spr.drawString(effectTitleNames[activeEffectMode], 160, 15);
    
    bool effectIsActive = false;
    if (activeEffectMode == 0) effectIsActive = isWhammyActive; else if (activeEffectMode == 1) effectIsActive = (isWhammyActive || isFrozen);
    else if (activeEffectMode == 2) effectIsActive = (isWhammyActive || isFeedbackActive); else if (activeEffectMode == 3) effectIsActive = (isWhammyActive || isHarmonizerMode);
    else if (activeEffectMode == 4) effectIsActive = (isWhammyActive || isCapoMode); else if (activeEffectMode == 5) effectIsActive = (isWhammyActive || isSynthMode);
    else if (activeEffectMode == 6) effectIsActive = (isWhammyActive || isPadMode); else if (activeEffectMode == 7) effectIsActive = (isWhammyActive || isChorusMode);
    else if (activeEffectMode == 8) effectIsActive = (isWhammyActive || isSwellMode); else if (activeEffectMode == 9) effectIsActive = (isWhammyActive || isVibratoMode);
    spr.fillCircle(240, 15, 6, effectIsActive ? TFT_GREEN : TFT_RED); spr.drawCircle(240, 15, 6, TFT_WHITE);

    spr.drawRect(10, 30, 8, 100, TFT_DARKGREY); spr.setTextColor(TFT_WHITE, TFT_BLACK); spr.setTextSize(1); spr.drawString("IN", 14, 140);
    spr.drawRect(spr.width() - 18, 30, 8, 100, TFT_DARKGREY); spr.drawString("OUT", spr.width() - 14, 140);

    int topY = 55; int topR = 17; char valBuf[16];
    
    float pb1Val = (currentPB1 - 8192) / 8192.0f; sprintf(valBuf, "%d%%", (int)(pb1Val * 100));
    drawCircularGauge(spr, 65, topY, topR, pb1Val, "PB1", valBuf, TFT_CYAN, 1);

    float pb2Val = (currentPB2 - 8192) / 8192.0f; sprintf(valBuf, "%d%%", (int)(pb2Val * 100));
    drawCircularGauge(spr, 125, topY, topR, pb2Val, isPB2WiperMode ? "PB2 W" : "PB2 H", valBuf, TFT_MAGENTA, 1);

    float pb3Val = currentPB3 / 16383.0f; sprintf(valBuf, "%d%%", (int)(pb3Val * 100));
    drawCircularGauge(spr, 185, topY, topR, pb3Val, isVolumeMode ? "VOL" : "PB3", valBuf, TFT_YELLOW, 0);

    float cc11Val = currentCC11 / 16383.0f; sprintf(valBuf, "%d%%", (int)(cc11Val * 100));
    drawCircularGauge(spr, 245, topY, topR, cc11Val, "CC11", valBuf, TFT_GREEN, 0);

    GaugeDef bGauges[6]; int numG = 0;
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);

    if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
        bGauges[numG++] = {1, effectMemory[1]/24.0f, "HEEL", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", effectMemory[1]);
        bGauges[numG++] = {1, effectMemory[0]/24.0f, "TOE", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", effectMemory[0]);
    } else if (activeEffectMode == 2) {
        float fbi[] = {0.0f, 12.0f, 19.0f, 24.0f, 28.0f}; 
        float val = fbi[constrain((int)feedbackIntervalIdx, 0, 4)];
        bGauges[numG++] = {2, val/28.0f, "OVT", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", val);
    } else if (activeEffectMode == 4) {
        int semi = (int)roundf(effectMemory[4]); 
        int cents = (int)roundf((effectMemory[4] - (float)semi) * 100.0f);
        bGauges[numG++] = {1, semi/24.0f, "SEMI", ""}; sprintf(bGauges[numG-1].valStr, "%+d", semi);
        bGauges[numG++] = {1, cents/50.0f, "CENT", ""}; sprintf(bGauges[numG-1].valStr, "%+d", cents);
        bGauges[numG++] = {1, effectMemory[1]/24.0f, "HEEL", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", effectMemory[1]);
        bGauges[numG++] = {1, effectMemory[0]/24.0f, "TOE", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", effectMemory[0]);
    } else {
        float val = effectMemory[activeEffectMode];
        if (activeEffectMode == 3) {
            bGauges[numG++] = {1, val/24.0f, "INT", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", val);
        } else if (activeEffectMode == 5) {
            bGauges[numG++] = {1, val/24.0f, "OSC", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", val);
        } else if (activeEffectMode == 6) {
            bGauges[numG++] = {1, val/24.0f, "SHFT", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", val);
        } else if (activeEffectMode == 7) {
            bGauges[numG++] = {1, val/24.0f, "SHFT", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", val);
        } else if (activeEffectMode == 9) {
            bGauges[numG++] = {1, val/24.0f, "BASE", ""}; sprintf(bGauges[numG-1].valStr, "%+.1f", val);
        }
    }
    
    if (activeEffectMode == 0) {
        bGauges[numG++] = {2, fxParams[0][0], "D.MIX", ""}; sprintf(bGauges[numG-1].valStr, "%.1f", fxParams[0][0]);
        bGauges[numG++] = {2, fxParams[0][1], "W.MIX", ""}; sprintf(bGauges[numG-1].valStr, "%.1f", fxParams[0][1]);
    } else if (activeEffectMode == 1) {
        bGauges[numG++] = {2, fxParams[1][0]/0.95f, "RVB", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[1][0]);
        bGauges[numG++] = {2, (fxParams[1][1]-0.00001f)/0.001f, "ATK", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[1][1]*1000.0f);
        bGauges[numG++] = {2, (fxParams[1][2]-0.00001f)/0.0005f, "REL", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[1][2]*1000.0f);
    } else if (activeEffectMode == 2) {
        bGauges[numG++] = {2, (fxParams[2][0]-1000.0f)/10000.0f, "SPD", ""}; sprintf(bGauges[numG-1].valStr, "%.0f", fxParams[2][0]);
        bGauges[numG++] = {2, (fxParams[2][1]-1.0f)/100.0f, "DRV", ""}; sprintf(bGauges[numG-1].valStr, "%.1f", fxParams[2][1]);
        bGauges[numG++] = {2, (fxParams[2][2]-0.005f)/0.045f, "DLY", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[2][2]);
    } else if (activeEffectMode == 3) {
        bGauges[numG++] = {2, fxParams[3][0], "MIX", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[3][0]);
    } else if (activeEffectMode == 5) {
        bGauges[numG++] = {2, (fxParams[5][0]-0.01f)/0.5f, "ATK", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[5][0]);
        bGauges[numG++] = {2, (fxParams[5][1]-0.001f)/0.05f, "REL", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[5][1]);
        bGauges[numG++] = {2, (fxParams[5][2]-0.1f)/0.8f, "FLT", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[5][2]);
        bGauges[numG++] = {2, fxParams[5][3], "MIX", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[5][3]);
    } else if (activeEffectMode == 6) {
        bGauges[numG++] = {2, (fxParams[6][0]-0.8f)/0.199f, "TON", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[6][0]);
        bGauges[numG++] = {2, fxParams[6][1]/3.0f, "MIX", ""}; sprintf(bGauges[numG-1].valStr, "%.1f", fxParams[6][1]);
    } else if (activeEffectMode == 7) {
        bGauges[numG++] = {2, (fxParams[7][0]-500.0f)/4500.0f, "SPD", ""}; sprintf(bGauges[numG-1].valStr, "%.0f", fxParams[7][0]);
        bGauges[numG++] = {2, fxParams[7][1], "MIX", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[7][1]);
    } else if (activeEffectMode == 8) {
        bGauges[numG++] = {2, (fxParams[8][0]-0.001f)/0.05f, "THR", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[8][0]);
        bGauges[numG++] = {2, (fxParams[8][1]-0.00001f)/0.0005f, "ATK", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[8][1]*1000.0f);
        bGauges[numG++] = {2, (fxParams[8][2]-0.00001f)/0.0005f, "REL", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[8][2]*1000.0f);
    } else if (activeEffectMode == 9) {
        bGauges[numG++] = {2, fxParams[9][0]/2.0f, "DEP", ""}; sprintf(bGauges[numG-1].valStr, "%.2f", fxParams[9][0]);
    }
    
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    int bY = 115; int bR = 17; uint32_t actCol = effectTitleColors[activeEffectMode];
    for (int i = 0; i < numG; i++) {
        int xPos = 160 - ((numG - 1) * 26) + (i * 52); 
        drawCircularGauge(spr, xPos, bY, bR, bGauges[i].normVal, bGauges[i].label, bGauges[i].valStr, actCol, bGauges[i].type);
    }

    spr.setTextDatum(ML_DATUM); spr.setTextSize(1);
    int bannerX = 25;
    auto drawBanner = [&](const char* lbl, uint32_t col) { spr.setTextColor(col, TFT_BLACK); spr.drawString(lbl, bannerX, 150); bannerX += 28; };
    if (isFrozen && activeEffectMode != 1) drawBanner("FRZ", TFT_CYAN); 
    if (isFeedbackActive && activeEffectMode != 2) drawBanner("SCM", TFT_RED);
    if (isHarmonizerMode && activeEffectMode != 3) drawBanner("HRM", TFT_MAGENTA); 
    if (isCapoMode && activeEffectMode != 4) drawBanner("CAP", TFT_GREEN);
    if (isSynthMode && activeEffectMode != 5) drawBanner("SYN", TFT_YELLOW); 
    if (isPadMode && activeEffectMode != 6) drawBanner("PAD", TFT_PINK);
    if (isChorusMode && activeEffectMode != 7) drawBanner("CHO", TFT_SKYBLUE); 
    if (isSwellMode && activeEffectMode != 8) drawBanner("SWL", TFT_WHITE);
    if (isVibratoMode && activeEffectMode != 9) drawBanner("VIB", TFT_PURPLE); 
    if (isVolumeMode) drawBanner("VOL", TFT_DARKGREY);

    int statsRowY = 162; spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK); spr.setTextDatum(ML_DATUM); 
    char cpuUsageBuffer[16]; sprintf(cpuUsageBuffer, "CPU:%2d%%", (int)core1_load); spr.drawString(cpuUsageBuffer, 25, statsRowY);
    char internalSramBuffer[16]; sprintf(internalSramBuffer, "SRM:%dK", (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024)); spr.drawString(internalSramBuffer, 85, statsRowY);
    char psramBuffer[16]; sprintf(psramBuffer, "PSR:%dK", (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024)); spr.drawString(psramBuffer, 150, statsRowY);

    spr.setTextDatum(MC_DATUM); spr.setTextColor(TFT_WHITE); 
    spr.drawRect(210, statsRowY - 7, 40, 14, TFT_DARKGREY);
    spr.drawString((currentSampleRate == 96000) ? "96k" : "48k", 230, statsRowY);
    
    spr.drawRect(255, statsRowY - 7, 40, 14, TFT_DARKGREY);
    const char* latencyLabelStrings[] = {"U.Low", "Low", "Mid", "High"}; 
    spr.drawString(latencyLabelStrings[latencyMode], 275, statsRowY);

    spr.pushSprite(0, 0); updateMeters();
}

void DisplayTask(void * pvParameters) {
    bool metersNeedClear = false;
    for (;;) {
        if (wakeupPending) { 
            pinMode(15, OUTPUT); digitalWrite(15, HIGH); tft.init(); 
            vTaskDelay(pdMS_TO_TICKS(120)); pinMode(38, OUTPUT); digitalWrite(38, HIGH); 
            isScreenOff = false; wakeupPending = false; forceUIUpdate = true; 
        }
        if (forceUIUpdate) { forceUIUpdate = false; updateDisplay(); metersNeedClear = true; } 
        else if (!isScreenOff) {
            if (ui_audio_level > 0.02f || ui_output_level > 0.02f) { updateMeters(); metersNeedClear = true; } 
            else if (metersNeedClear) { ui_audio_level = 0.0f; ui_output_level = 0.0f; updateMeters(); metersNeedClear = false; }
        }
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

void IRAM_ATTR AudioDSPTask(void * pvParameters) {
    static int32_t i2s_in_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_out_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    
    static float dc_block[HOP_SIZE] __attribute__((aligned(16)));
    static float mix_block[HOP_SIZE] __attribute__((aligned(16)));
    
    static float w1_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w2_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w3_block[HOP_SIZE] __attribute__((aligned(16)));
    static float dry_block[HOP_SIZE] __attribute__((aligned(16)));
    static float fz_block[HOP_SIZE] __attribute__((aligned(16)));
    static float pad_block[HOP_SIZE] __attribute__((aligned(16)));
    static float fbOut_block[HOP_SIZE] __attribute__((aligned(16)));
    
    static float input_dc_offset = 0.0f;
    
    static float synthEnv = 0.0f; static float synthFilter = 0.0f;
    static float padFilter = 0.0f; static float padEnv = 0.0f;
    static float inputEnvelope = 0.0f; static float feedbackFilterVar = 0.0f;
    static float smoothedVolGain = 1.0f;
    static float currentPitch = 1.0f; 
    
    static float fbOutNode = 0.0f;
    static float smoothed_delay_samples = 0.0f;
    
    static bool wasFeedbackActive = false;
    
    static int freezeWriteIdxVar = 0; static int freezePlayCounterVar = 0; 
    static int freezeStartIdxVar = 0; static int activeFreezeLength = 48000;
    
    const float normFactor = 1.0f / 2147483648.0f; 
    const float DC_OFFSET = 1e-9f;
    
    for (;;) {
        if (sleepRequested) { isSleeping = true; vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        isSleeping = false; 
        size_t bytesRead; 
        
        i2s_channel_read(rx_chan, i2s_in_block, sizeof(i2s_in_block), &bytesRead, pdMS_TO_TICKS(20));
        
        if (bytesRead > 0) {
            int framesRead = bytesRead / 8; 
            framesRead &= ~3; 
            
            if (framesRead > 0) {
                if (globalAudioResetRequested) {
                    synthEnv = 0.0f; synthFilter = 0.0f; padFilter = 0.0f; padEnv = 0.0f;
                    inputEnvelope = 0.0f; feedbackFilterVar = 0.0f; smoothedVolGain = volumePedalGain; currentPitch = 1.0f;
                    freezeWriteIdxVar = 0; freezePlayCounterVar = 0; freezeStartIdxVar = 0; activeFreezeLength = currentSampleRate;
                    fbDelayWriteIdx = 0; writeIndex = 0; apfNeedsClear = true;
                    input_dc_offset = 0.0f; 
                    ui_audio_level = 0.0f; ui_output_level = 0.0f; 
                    clearBuffersRequested = true; globalAudioResetRequested = false;
                    
                    smoothed_delay_samples = 0.0f; 
                    
                    if (hardwareSyncMuteFrames < 10) hardwareSyncMuteFrames = (currentSampleRate / HOP_SIZE) * 0.15f; 
                }

                if (hardwareSyncMuteFrames > 0) {
                    hardwareSyncMuteFrames = hardwareSyncMuteFrames - 1;
                    memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                    ui_audio_level = 0.0f; ui_output_level = 0.0f;
                    
                    for (int i = 0; i < framesRead; i++) { 
                        int32_t clean_sample = i2s_in_block[i * 2] & 0xFFFFFF00;
                        float raw_in = ((float)clean_sample * normFactor); 
                        input_dc_offset = (input_dc_offset * 0.95f) + (raw_in * 0.05f); 
                    }
                    
                    size_t bytesWrittenCount; i2s_channel_write(tx_chan, i2s_out_block, framesRead * 8, &bytesWrittenCount, portMAX_DELAY);
                    continue; 
                }

                bool blockIsMuted = clearBuffersRequested;
                uint32_t start_cycles = xthal_get_ccount(); 
                
                float srScale = 48000.0f / (float)currentSampleRate;

                if (blockIsMuted) {
                    memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                    ui_audio_level = 0.0f; ui_output_level = 0.0f;
                } else {
                    if (audioBufferMutex != NULL && xSemaphoreTake(audioBufferMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
                        float targetWindow = LATENCY_WINDOWS[latencyMode];
                        if (currentWindowSize != targetWindow) { 
                            currentWindowSize = targetWindow; 
                            uint32_t halfWindowFixed = ((uint32_t)targetWindow / 2) << 16;
                            tap_w1_1 = 0; tap_w1_2 = halfWindowFixed; tap_w2_1 = 0; tap_w2_2 = halfWindowFixed;
                            tap_w3_1 = 0; tap_w3_2 = halfWindowFixed; tap_w4_1 = 0; tap_w4_2 = halfWindowFixed; 
                            tap_w5_1 = 0; tap_w5_2 = halfWindowFixed;
                        }
                        uint32_t hannIntMult = (1024U << 16) / (uint32_t)currentWindowSize; 
                        uint32_t windowMask = (uint32_t)currentWindowSize - 1;
                        
                        float p_w_dry = fxParams[0][0]; float p_w_wet = fxParams[0][1];
                        float p_fz_apf = fxParams[1][0]; float p_fz_att = fxParams[1][1]; float p_fz_rel = fxParams[1][2];
                        float p_fb_spd = fxParams[2][0]; float p_fb_drv = fxParams[2][1]; float p_fb_off = fxParams[2][2];
                        float p_hr_mix = fxParams[3][0];
                        float p_sy_att = fxParams[5][0]; float p_sy_rel = fxParams[5][1]; float p_sy_flt = fxParams[5][2]; float p_sy_mix = fxParams[5][3];
                        float p_pd_sm = fxParams[6][0]; float p_pd_mix = fxParams[6][1];
                        float p_ch_spd = fxParams[7][0]; float p_ch_mix = fxParams[7][1];
                        float p_sw_thr = fxParams[8][0]; float p_sw_att = fxParams[8][1]; float p_sw_rel = fxParams[8][2];
                        float p_vb_dep = fxParams[9][0];

                        float chorusPhaseIncr = p_ch_spd / (float)currentSampleRate; 
                        float feedbackPhaseIncr = p_fb_spd / (float)currentSampleRate;
                        
                        float targetPitch = pitchShiftFactor;
                        float pitchInc = (targetPitch - currentPitch) / (float)framesRead;
                        
                        bool frzActive = ((activeEffectMode == 1 && isWhammyActive) || isFrozen); 
                        if (frzActive && !wasFrozen) { 
                            freezePlayCounterVar = 0; 
                            int bestStart = freezeWriteIdxVar;
                            int tempIdx = freezeWriteIdxVar;
                            for (int s = 0; s < 4000; s++) {
                                int prev = tempIdx - 1;
                                if (prev < 0) prev += freezeLength;
                                if (freezeBuffer[tempIdx] >= 0.0f && freezeBuffer[prev] < 0.0f) { bestStart = tempIdx; break; }
                                tempIdx = prev;
                            }
                            freezeStartIdxVar = bestStart;
                            activeFreezeLength = freezeLength;
                            
                            int searchEnd = bestStart - 1;
                            if (searchEnd < 0) searchEnd += freezeLength;
                            tempIdx = searchEnd;
                            for (int s = 0; s < 4000; s++) {
                                int prev = tempIdx - 1;
                                if (prev < 0) prev += freezeLength;
                                if (freezeBuffer[tempIdx] >= 0.0f && freezeBuffer[prev] < 0.0f) { 
                                    activeFreezeLength = s; 
                                    break; 
                                }
                                tempIdx = prev;
                            }
                            if (activeFreezeLength < 64) activeFreezeLength = freezeLength;
                        } 
                        if (!frzActive && wasFrozen) apfNeedsClear = true;
                        wasFrozen = frzActive;
                        float activeInvFreqLength = 1.0f / (float)activeFreezeLength;
                        
                        bool synthActive = ((activeEffectMode == 5 && isWhammyActive) || isSynthMode);
                        bool padActive = ((activeEffectMode == 6 && isWhammyActive) || isPadMode);
                        bool harmActive = ((activeEffectMode == 3 && isWhammyActive) || isHarmonizerMode);
                        bool swellActive = ((activeEffectMode == 8 && isWhammyActive) || isSwellMode);
                        bool chorusActive = ((activeEffectMode == 7 && isWhammyActive) || isChorusMode);
                        
                        bool feedbackActive = ((activeEffectMode == 2 && isWhammyActive) || isFeedbackActive);
                        if (feedbackActive && !wasFeedbackActive) {
                            fbOutNode = 0.0f;
                            fbHpfState = 0.0f;
                            feedbackFilterVar = 0.0f;
                        }
                        wasFeedbackActive = feedbackActive;
                        
                        bool vibratoActive = ((activeEffectMode == 9 && isWhammyActive) || isVibratoMode);
                        bool capoActive = ((activeEffectMode == 4 && isWhammyActive) || isCapoMode);
                        
                        float peakInputVal = 0.0f; float peakOutputVal = 0.0f;
                        float localSwellGain = swellGain; float localVolGain = volumePedalGain;
                        float localFrzRamp = freezeRamp; float localFbRamp = feedbackRamp;

                        float pdSmCoeff = powf(p_pd_sm, srScale);
                        
                        float target_delay = constrain((float)(currentSampleRate * p_fb_off), 0.0f, (float)(FB_BUFFER_SIZE - 1));
                        
                        // FIX 3: Appended DC_OFFSET to prevent FPU subnormal mathematical lockup when idle
                        smoothed_delay_samples += (target_delay - smoothed_delay_samples) * 0.01f * srScale + DC_OFFSET;
                        int delaySamples = (int)smoothed_delay_samples;
                        
                        float fbHpfCoeff = (currentSampleRate == 96000) ? 0.025f : 0.05f;
                        float fbLpfCoeff = (currentSampleRate == 96000) ? 0.05f : 0.1f;
                        float fbLpfRetain = 1.0f - fbLpfCoeff;

                        float dc_alpha = (currentSampleRate == 96000) ? 0.0005f : 0.001f;
                        
                        int halfWindow = (int)currentWindowSize / 2;

                        #pragma GCC ivdep
                        for (int i = 0; i < framesRead; i++) { 
                            int32_t clean_sample = i2s_in_block[i * 2] & 0xFFFFFF00;
                            float raw_in = ((float)clean_sample * normFactor); 
                            
                            input_dc_offset = (input_dc_offset * (1.0f - dc_alpha)) + (raw_in * dc_alpha);
                            dc_block[i] = raw_in - input_dc_offset; 
                        }
                        
                        for (int i = 0; i < framesRead; i++) {
                            currentPitch += pitchInc;
                            float harmPitch = currentPitch * globalHarmRatio;
                            float choPitch  = currentPitch * globalChorusRatio;
                            float fbPitch   = currentPitch * globalFbRatio;
                            
                            float inSample = dc_block[i]; 
                            inputEnvelope = inputEnvelope * 0.99f + fabsf(inSample) * 0.01f + DC_OFFSET;
                            
                            if (swellActive) {
                                if (inputEnvelope > p_sw_thr) { localSwellGain = fminf(1.0f, localSwellGain + (p_sw_att * srScale)); } 
                                else { localSwellGain = fmaxf(0.0f, localSwellGain - (p_sw_rel * srScale)); }
                            } else { 
                                if (localSwellGain < 1.0f) localSwellGain = fminf(1.0f, localSwellGain + (0.005f * srScale)); 
                                else localSwellGain = 1.0f; 
                            }
                            
                            float procSample = inSample;
                            
                            if (synthActive) { 
                                if (inputEnvelope > 0.005f) { synthEnv = fminf(1.0f, synthEnv + (p_sy_att * srScale)); } 
                                else { synthEnv = fmaxf(0.0f, synthEnv - (p_sy_rel * srScale)); }
                                
                                if (isnan(procSample)) procSample = 0.0f;
                                int waveIdx = constrain((int)((procSample + 1.0f) * 1023.5f), 0, WAVE_LUT_SIZE - 1);
                                procSample = synthLUT[waveIdx]; 
                                float fltCoeff = fmaxf(0.001f, fminf(0.99f, (p_sy_flt + 0.6f * synthEnv) * srScale));
                                synthFilter = synthFilter + fltCoeff * (procSample - synthFilter) + DC_OFFSET; 
                                procSample = synthFilter * p_sy_mix;
                            } 
                            
                            if (padActive) { 
                                if (inputEnvelope > 0.005f) { padEnv = fminf(1.0f, padEnv + (0.00002f * srScale)); } 
                                else { padEnv = fmaxf(0.0f, padEnv - (0.000005f * srScale)); }
                                procSample *= padEnv; 
                            }
                            
                            if (!frzActive) { 
                                freezeBuffer[freezeWriteIdxVar] = procSample; freezeWriteIdxVar++;
                                if (freezeWriteIdxVar >= freezeLength) freezeWriteIdxVar = 0;
                            }
                            
                            if (localFrzRamp > 0.0f || frzActive) {
                                if (frzActive) { localFrzRamp = fminf(1.0f, localFrzRamp + (p_fz_att * srScale)); } 
                                else { localFrzRamp = fmaxf(0.0f, localFrzRamp - (p_fz_rel * srScale)); }
                            }
                            
                            float fzOut = 0.0f;
                            if (localFrzRamp > 0.0f) { 
                                float phaseRead = (float)freezePlayCounterVar * activeInvFreqLength; 
                                float phase2 = (phaseRead + 0.5f); if (phase2 >= 1.0f) { phase2 -= 1.0f; }
                                
                                int sum1 = freezeStartIdxVar + freezePlayCounterVar;
                                int idx1 = sum1;
                                if (idx1 >= freezeLength) idx1 -= freezeLength;
                                
                                int counter2 = freezePlayCounterVar + (activeFreezeLength / 2);
                                if (counter2 >= activeFreezeLength) counter2 -= activeFreezeLength;
                                int sum2 = freezeStartIdxVar + counter2;
                                int idx2 = sum2;
                                if (idx2 >= freezeLength) idx2 -= freezeLength;
                                
                                float rFrz = (freezeBuffer[idx1] * hannLUT[(int)(phaseRead * 1023.0f)]) + (freezeBuffer[idx2] * hannLUT[(int)(phase2 * 1023.0f)]);
                                
                                float d1 = apf1Buffer[apf1Idx]; 
                                float next_apf1 = rFrz + p_fz_apf * d1 + DC_OFFSET; 
                                float a1 = -p_fz_apf * rFrz + d1; 
                                apf1Buffer[apf1Idx] = next_apf1; apf1Idx++; if (apf1Idx >= 1009) { apf1Idx = 0; }
                                
                                float d2 = apf2Buffer[apf2Idx]; 
                                float next_apf2 = a1 + p_fz_apf * d2 + DC_OFFSET; 
                                float a2 = -p_fz_apf * a1 + d2; 
                                apf2Buffer[apf2Idx] = next_apf2; apf2Idx++; if (apf2Idx >= 863) { apf2Idx = 0; }
                                
                                fzOut = a2 * localFrzRamp; 
                                freezePlayCounterVar++; if (freezePlayCounterVar >= activeFreezeLength) { freezePlayCounterVar = 0; }
                            } else if (apfNeedsClear) {
                                memset(apf1Buffer, 0, sizeof(apf1Buffer)); memset(apf2Buffer, 0, sizeof(apf2Buffer));
                                apf1Idx = 0; apf2Idx = 0; 
                                apfNeedsClear = false;
                            }
                            
                            float delayIn = (frzActive && localFrzRamp > 0.0f) ? fzOut : procSample; 
                            float boundedDelayIn = constrain(delayIn, -1.0f, 1.0f);
                            int localWriteIdx = writeIndex; delayBuffer[localWriteIdx] = boundedDelayIn;
                            
                            float spd1 = currentPitch;
                            if (vibratoActive) {
                                vibratoLfoPhase += globalVibratoPhaseInc; 
                                if (vibratoLfoPhase >= LFO_LUT_SIZE) { vibratoLfoPhase -= LFO_LUT_SIZE; }
                                spd1 *= 1.0f + ((lfoLUT[(int)vibratoLfoPhase & 1023] - 1.0f) * p_vb_dep);
                            }
                            
                            float spd2 = harmPitch; float spd3 = choPitch;
                            if (chorusActive) { 
                                chorusLfoPhase += chorusPhaseIncr; 
                                if (chorusLfoPhase >= LFO_LUT_SIZE) { chorusLfoPhase -= LFO_LUT_SIZE; }
                                spd3 *= lfoLUT[(int)chorusLfoPhase & 1023]; 
                            }
                            
                            float spd4 = 1.0f; float spd5 = 1.0f;
                            
                            if (feedbackActive || localFbRamp > 0.0f) { 
                                feedbackLfoPhase += feedbackPhaseIncr; 
                                if (feedbackLfoPhase >= LFO_LUT_SIZE) { feedbackLfoPhase -= LFO_LUT_SIZE; }
                                float lfoVal = lfoLUT[(int)feedbackLfoPhase & 1023]; spd4 = lfoVal; spd5 = fbPitch * lfoVal; 
                                
                                float w4 = processTap(tap_w4_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + processTap(tap_w4_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                                float w5 = processTap(tap_w5_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + processTap(tap_w5_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                                    
                                if (feedbackActive) {
                                    if (inputEnvelope > 0.005f) { localFbRamp = fminf(1.0f, localFbRamp + (0.000011f * srScale)); } 
                                    else { localFbRamp = fmaxf(0.0f, localFbRamp - (0.005f * srScale)); }
                                } else { localFbRamp = fmaxf(0.0f, localFbRamp - (0.0001f * srScale)); }
                                
                                float mixV = fmaxf(0.0f, fminf((localFbRamp - 0.1f) * 2.0f, 1.0f));
                                
                                float feedInput = (frzActive && localFrzRamp > 0.0f) ? fzOut : (w4 * (1.0f - mixV)) + (w5 * mixV) + (fbOutNode * 0.95f);
                                
                                fbHpfState += fbHpfCoeff * (feedInput - fbHpfState) + DC_OFFSET; 
                                
                                float rawDrive = (feedInput - fbHpfState) * p_fb_drv;
                                float boundedDrive = fmaxf(-1.5f, fminf(rawDrive, 1.5f));
                                float gainDrive = boundedDrive * (1.0f - (0.15f * boundedDrive * boundedDrive));
                                
                                feedbackFilterVar = feedbackFilterVar * fbLpfRetain + gainDrive * fbLpfCoeff + DC_OFFSET;
                                float satFb = feedbackFilterVar * (localFbRamp * localFbRamp * localFbRamp) * 0.85f; 
                                fbDelayBuffer[fbDelayWriteIdx] = satFb;
                                
                                int fbReadIdx = (fbDelayWriteIdx - delaySamples + FB_BUFFER_SIZE) & FB_BUFFER_MASK;
                                fbOutNode = fbDelayBuffer[fbReadIdx];
                                fbDelayWriteIdx = (fbDelayWriteIdx + 1) & FB_BUFFER_MASK;
                            }
                            
                            float w1 = processTap(tap_w1_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + processTap(tap_w1_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                            w1_block[i] = w1; 
                            
                            float w2 = 0.0f;
                            if (harmActive) w2 = processTap(tap_w2_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + processTap(tap_w2_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                            w2_block[i] = w2; 
                            
                            float w3 = 0.0f;
                            if (chorusActive) w3 = processTap(tap_w3_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + processTap(tap_w3_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                            w3_block[i] = w3; 
                            
                            int32_t step1 = (int32_t)((1.0f - spd1) * 65536.0f); tap_w1_1 += step1; tap_w1_2 += step1; 
                            int32_t step2 = (int32_t)((1.0f - spd2) * 65536.0f); tap_w2_1 += step2; tap_w2_2 += step2; 
                            int32_t step3 = (int32_t)((1.0f - spd3) * 65536.0f); tap_w3_1 += step3; tap_w3_2 += step3; 
                            int32_t step4 = (int32_t)((1.0f - spd4) * 65536.0f); tap_w4_1 += step4; tap_w4_2 += step4; 
                            int32_t step5 = (int32_t)((1.0f - spd5) * 65536.0f); tap_w5_1 += step5; tap_w5_2 += step5; 
                            
                            if (padActive) { padFilter = padFilter * pdSmCoeff + w1 * (1.0f - pdSmCoeff) + DC_OFFSET; } 
                            else { padFilter = padFilter * pdSmCoeff + DC_OFFSET; }
                            
                            int dryIdx = (localWriteIdx - halfWindow + MAX_BUFFER_SIZE) & BUFFER_MASK;
                            dry_block[i] = delayBuffer[dryIdx]; 
                            
                            fz_block[i] = fzOut; pad_block[i] = padFilter; fbOut_block[i] = fbOutNode;
                            
                            writeIndex = (writeIndex + 1) & BUFFER_MASK;
                        }
                        
                        swellGain = localSwellGain; freezeRamp = localFrzRamp; feedbackRamp = localFbRamp;
                        
                        bool activeGroup = isWhammyActive || harmActive || chorusActive || feedbackActive || synthActive || padActive || frzActive || vibratoActive || capoActive;
                        bool dryGroup = chorusActive || padActive || frzActive || feedbackActive || (localFrzRamp > 0.0f) || (localFbRamp > 0.0f);
                        bool repeatGroup = capoActive || synthActive || vibratoActive || padActive || harmActive;
                        bool padIsAudible = padActive || (padFilter > 0.001f);
                        
                        if (!activeGroup && localFrzRamp <= 0.0f && localFbRamp <= 0.0f && !padIsAudible) {
                            memcpy(mix_block, dry_block, framesRead * sizeof(float));
                        } else {
                            float g_base = 0.0f;
                            if (dryGroup) { if (!repeatGroup) g_base = 0.4f; } 
                            else if (harmActive) { g_base = 0.5f; } 
                            else { g_base = 1.0f; }
                            
                            float g_w2 = harmActive ? p_hr_mix : 0.0f;
                            float g_w3 = chorusActive ? p_ch_mix : 0.0f;
                            float g_pad = padIsAudible ? p_pd_mix : 0.0f;
                            float g_frz = (!frzActive && localFrzRamp > 0.0f) ? 0.5f : 0.0f;
                            float g_fb = (feedbackActive || localFbRamp > 0.0f) ? 0.6f : 0.0f;
                            
                            float g_whammy = isWhammyActive ? p_w_wet : 0.0f;
                            float g_dry = isWhammyActive ? p_w_dry : 1.0f;
                            
                            float* __restrict p_mix = mix_block; const float* __restrict p_dry = dry_block; const float* __restrict p_w1 = w1_block;
                            const float* __restrict p_w2 = w2_block; const float* __restrict p_w3 = w3_block; const float* __restrict p_pad = pad_block;
                            const float* __restrict p_fz = fz_block; const float* __restrict p_fb = fbOut_block;
                            
                            #pragma GCC ivdep
                            for (int i = 0; i < framesRead; i++) {
                                float baseSignal = (p_w1[i] * g_whammy) + (p_dry[i] * g_dry);
                                float sMix = DC_OFFSET;
                                sMix += baseSignal * g_base; sMix += p_w2[i] * g_w2; sMix += p_w3[i] * g_w3;
                                sMix += p_pad[i] * g_pad; sMix += p_fz[i] * g_frz; sMix += p_fb[i] * g_fb;
                                sMix = fmaxf(-1.8f, fminf(sMix, 1.8f));
                                p_mix[i] = sMix * (1.0f - (0.1f * sMix * sMix)) * 0.82f;
                            }
                        }
                        
                        float vol_alpha = 0.01f * srScale;
                        float meter_decay = (currentSampleRate == 96000) ? 0.999f : 0.998f;

                        #pragma GCC ivdep
                        for (int i = 0; i < framesRead; i++) {
                            smoothedVolGain = smoothedVolGain * (1.0f - vol_alpha) + localVolGain * vol_alpha;
                            float rawOut = mix_block[i] * localSwellGain * smoothedVolGain; 
                            
                            if (fabsf(dc_block[i]) > peakInputVal) peakInputVal = fabsf(dc_block[i]); 
                            if (fabsf(rawOut) > peakOutputVal) peakOutputVal = fabsf(rawOut); 
                            
                            float currentMasterScale = 2147483520.0f * localSwellGain * smoothedVolGain;
                            float scaledOut = mix_block[i] * currentMasterScale;
                            int32_t finalOut = (int32_t)fmaxf(-2147483520.0f, fminf(scaledOut, 2147483520.0f));
                            
                            finalOut &= 0xFFFFFF00;
                            i2s_out_block[i * 2] = finalOut; i2s_out_block[i * 2 + 1] = finalOut;
                        }

                        if (peakInputVal > ui_audio_level) ui_audio_level = peakInputVal; else { ui_audio_level *= meter_decay; if (ui_audio_level < 1e-5f) ui_audio_level = 0.0f; }
                        if (peakOutputVal > ui_output_level) ui_output_level = peakOutputVal; else { ui_output_level *= meter_decay; if (ui_output_level < 1e-5f) ui_output_level = 0.0f; }
                        
                        xSemaphoreGive(audioBufferMutex);
                    } else {
                        memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                        ui_audio_level = 0.0f; ui_output_level = 0.0f;
                    }
                } 
                
                uint32_t end_timer = xthal_get_ccount(); 
                float max_cycles = (240000000.0f / (float)currentSampleRate) * (float)framesRead;
                float currentLoadPercentage = ((float)(end_timer - start_cycles) / max_cycles) * 100.0f;
                core1_load = core1_load * 0.95f + fminf(100.0f, currentLoadPercentage) * 0.05f; 
                
                size_t bytesWrittenCount; 
                i2s_channel_write(tx_chan, i2s_out_block, framesRead * 8, &bytesWrittenCount, portMAX_DELAY);
            } else {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void updateParameterFromCC(uint8_t cc, uint8_t val) {
    float norm = (float)val / 127.0f; 
    int pIdx = cc - 24; 
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);

    if (activeEffectMode == 0) { 
        if (pIdx == 0) { effectMemory[1] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) { effectMemory[0] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 2) fxParams[0][0] = norm; 
        if (pIdx == 3) fxParams[0][1] = norm; 
    } 
    else if (activeEffectMode == 1) { 
        if (pIdx == 0) { effectMemory[1] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) { effectMemory[0] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 2) fxParams[1][0] = 0.0f + (norm * 0.95f);               
        if (pIdx == 3) fxParams[1][1] = 0.00001f + (norm * 0.001f);          
        if (pIdx == 4) fxParams[1][2] = 0.00001f + (norm * 0.0005f);         
    }
    else if (activeEffectMode == 2) { 
        if (pIdx == 0) { 
            int newIdx = constrain((int)roundf(norm * 4.0f), 0, 4);
            if (newIdx != feedbackIntervalIdx) {
                feedbackIntervalIdx = newIdx;
                lutNeedsUpdate = true;
                forceUIUpdate = true;
            }
        }
        if (pIdx == 1) fxParams[2][0] = 1000.0f + (norm * 10000.0f);         
        if (pIdx == 2) fxParams[2][1] = 1.0f + (norm * 100.0f);             
        if (pIdx == 3) fxParams[2][2] = 0.005f + (norm * 0.045f);            
    }
    else if (activeEffectMode == 3) { 
        if (pIdx == 0) { effectMemory[3] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) fxParams[3][0] = norm;                                
    }
    else if (activeEffectMode == 4) { 
        if (pIdx == 0) { 
            int cents = (int)roundf((effectMemory[4] - (float)roundf(effectMemory[4])) * 100.0f); 
            effectMemory[4] = constrain(roundf((norm * 48.0f) - 24.0f) + ((float)cents / 100.0f), -24.0f, 24.0f);
            lutNeedsUpdate = true; forceUIUpdate = true; 
        }
        if (pIdx == 1) { 
            int semi = (int)roundf(effectMemory[4]); 
            float c = roundf((norm * 100.0f) - 50.0f) / 100.0f; 
            effectMemory[4] = constrain((float)semi + c, -24.0f, 24.0f);
            lutNeedsUpdate = true; forceUIUpdate = true; 
        }
    }
    else if (activeEffectMode == 5) { 
        if (pIdx == 0) { effectMemory[5] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) fxParams[5][0] = 0.01f + (norm * 0.5f);               
        if (pIdx == 2) fxParams[5][1] = 0.001f + (norm * 0.05f);             
        if (pIdx == 3) fxParams[5][2] = 0.1f + (norm * 0.8f);               
        if (pIdx == 4) fxParams[5][3] = norm;                                
    }
    else if (activeEffectMode == 6) { 
        if (pIdx == 0) { effectMemory[6] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) fxParams[6][0] = 0.8f + (norm * 0.199f);              
        if (pIdx == 2) fxParams[6][1] = norm * 3.0f;                         
    }
    else if (activeEffectMode == 7) { 
        if (pIdx == 0) { effectMemory[7] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) fxParams[7][0] = 500.0f + (norm * 4500.0f);           
        if (pIdx == 2) fxParams[7][1] = norm;                                
    }
    else if (activeEffectMode == 8) { 
        if (pIdx == 0) { effectMemory[1] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) { effectMemory[0] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 2) fxParams[8][0] = 0.001f + (norm * 0.05f);             
        if (pIdx == 3) fxParams[8][1] = 0.00001f + (norm * 0.001f);        
        if (pIdx == 4) fxParams[8][2] = 0.00001f + (norm * 0.0005f);        
    }
    else if (activeEffectMode == 9) { 
        if (pIdx == 0) { effectMemory[9] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; forceUIUpdate = true; } 
        if (pIdx == 1) fxParams[9][0] = norm * 2.0f;                         
    }

    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    settingsNeedSaving = true; 
    lastParameterChangeTime = millis();
}

void MidiTask(void * pvParameters) {
    pedals.resetToCenter(); 

    static bool lastBtState = false; static uint8_t lastVolumeCC = 127;
    #if ENABLE_PAR_KNOBS
        static int lastCcOut[5] = {-1, -1, -1, -1, -1};
    #endif
    
    static unsigned long lastBatteryTime = 0;
    
    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP);
    
    for (;;) {
        Control_Surface.loop(); 
        
        bool currentBtState = btmidi.isConnected();
        if (currentBtState != lastBtState) { 
            lastBtState = currentBtState; forceUIUpdate = true; 
            if (isScreenOff) turnScreenOn(); 
            lastActivityTime = millis(); 
            lastScreenActivityTime = millis(); 
        }

        if (!currentBtState && (millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT)) goToLightSleep(); 
        if (!isScreenOff && (millis() - lastScreenActivityTime > SCREEN_OFF_TIMEOUT)) turnScreenOff(); 
        
        #if ENABLE_PAR_KNOBS
            if (filterPar1.update()) {
                int cc1 = map(filterPar1.getValue(), 0, 4095, 0, 127);
                if (cc1 != lastCcOut[0]) {
                    Control_Surface.sendControlChange({24, Channel_1}, cc1);
                    updateParameterFromCC(24, cc1); 
                    lastCcOut[0] = cc1;
                }
            }
            
            if (filterPar2.update()) {
                int cc2 = map(filterPar2.getValue(), 0, 4095, 0, 127);
                if (cc2 != lastCcOut[1]) {
                    Control_Surface.sendControlChange({25, Channel_1}, cc2);
                    updateParameterFromCC(25, cc2); 
                    lastCcOut[1] = cc2;
                }
            }
            
            if (filterPar3.update()) {
                int cc3 = map(filterPar3.getValue(), 0, 4095, 0, 127);
                if (cc3 != lastCcOut[2]) {
                    Control_Surface.sendControlChange({26, Channel_1}, cc3);
                    updateParameterFromCC(26, cc3); 
                    lastCcOut[2] = cc3;
                }
            }
            
            if (filterPar4.update()) {
                int cc4 = map(filterPar4.getValue(), 0, 4095, 0, 127);
                if (cc4 != lastCcOut[3]) {
                    Control_Surface.sendControlChange({27, Channel_1}, cc4);
                    updateParameterFromCC(27, cc4); 
                    lastCcOut[3] = cc4;
                }
            }
            
            if (filterPar5.update()) {
                int cc5 = map(filterPar5.getValue(), 0, 4095, 0, 127);
                if (cc5 != lastCcOut[4]) {
                    Control_Surface.sendControlChange({28, Channel_1}, cc5);
                    updateParameterFromCC(28, cc5); 
                    lastCcOut[4] = cc5;
                }
            }
        #endif
        
        analogRead(pinPB); 
        analogRead(pinPB2); 
        analogRead(pinPB3);

        filterPB.update(); filterPB2.update(); filterPB3.update();
        
        if (digitalRead(BOOT_SENSE_PIN) == HIGH) {
            
            pedals.process(filterPB.getValue(), filterPB2.getValue(), filterPB3.getValue(), isVolumeMode, INVERT_PB3);
            
            int calA = pedals.getCalA();
            int calB = pedals.getCalB();
            int calC = pedals.getCalC();
            
            bool moveA = pedals.hasMovedA();
            bool moveB = pedals.hasMovedB();
            bool moveC = pedals.hasMovedC();
            
            if (moveA || moveB || moveC) {
                if (isScreenOff) turnScreenOn();
                lastScreenActivityTime = millis();
                lastActivityTime = millis();
                
                if (moveA) { Control_Surface.sendPitchBend(Channel_1, calA); pedals.updateLastMidiA(); currentPB1 = calA; }
                if (moveB) { Control_Surface.sendPitchBend(Channel_2, calB); pedals.updateLastMidiB(); currentPB2 = calB; }
                if (moveC) {
                    if (!isVolumeMode) Control_Surface.sendPitchBend(Channel_3, calC);
                    pedals.updateLastMidiC(); currentPB3 = calC; 
                }

                bool pitchChanged = false;
                if (moveA) { lastActivePedal = calA; pitchChanged = true; }
                if (moveB) { lastActivePedal = calB; pitchChanged = true; }
                if (moveC && !isVolumeMode) { lastActivePedal = calC; pitchChanged = true; }

                if (pitchChanged) {
                    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
                    pitchShiftFactor = pitchShiftLUT[constrain(lastActivePedal, 0, 16383)];
                    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
                }

                if (moveC && isVolumeMode) {
                    uint8_t vCC = map(calC, 0, 16383, 0, 127); 
                    if (vCC != lastVolumeCC) { Control_Surface.sendControlChange({19, Channel_1}, vCC); lastVolumeCC = vCC; } 
                    
                    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
                    volumePedalGain = (float)calC / 16383.0f; 
                    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
                }
                
                forceUIUpdate = true;
            }
        } else {
            pedals.resetToCenter();
            lastActivePedal = 8192;
            
            if (!lutNeedsUpdate) {
                if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
                pitchShiftFactor = pitchShiftLUT[8192]; 
                if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            }
            
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            if (isVolumeMode) volumePedalGain = 8192.0f / 16383.0f; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            
            forceUIUpdate = true;
            
            vTaskDelay(pdMS_TO_TICKS(5)); 
        }

        if (millis() - lastBatteryTime > 1000) {
            lastBatteryTime = millis();
            
            uint32_t rawSum = 0;
            for (int i = 0; i < 32; i++) {
                rawSum += analogRead(BATTERY_PIN);
            }
            
            float rawAvg = (float)rawSum / 32.0f;
            const float CALIBRATION_MULTIPLIER = 1.04571f; 
            float instantVoltage = (rawAvg / 4095.0f) * 3.3f * 2.0f * CALIBRATION_MULTIPLIER;
            
            if (instantVoltage > 2.0f) {
                bool charging = (instantVoltage > 4.20f);
                static float smoothedVoltage = -1.0f;
                
                if (smoothedVoltage < 0.1f) {
                    smoothedVoltage = instantVoltage; 
                } else {
                    smoothedVoltage = (smoothedVoltage * 0.9f) + (instantVoltage * 0.1f);
                }
                
                int newPercent = getBatteryPercentage(smoothedVoltage);
                bool stateChanged = (newPercent != currentBatteryPercent) || (charging != isBatteryCharging);
                
                currentBatteryVoltage = smoothedVoltage;
                currentBatteryPercent = newPercent;
                isBatteryCharging = charging;
                
                if (stateChanged) forceUIUpdate = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool channelMessageCallback(ChannelMessage cm) {
    if (isScreenOff) turnScreenOn();
    lastActivityTime = millis();
    lastScreenActivityTime = millis();

    if (cm.header == 0xB0) {
        
        if (cm.data1 == 11) { 
            uint16_t mappedCC = map(cm.data2, 0, 127, 0, 16383); currentCC11 = mappedCC; currentPB3 = mappedCC; 
            if (isVolumeMode) { 
                if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
                volumePedalGain = (float)mappedCC / 16383.0f; 
                if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
                Control_Surface.sendControlChange({19, Channel_1}, cm.data2); 
            } 
            else { 
                if (!lutNeedsUpdate) {
                    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
                    pitchShiftFactor = pitchShiftLUT[mappedCC]; 
                    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
                }
            }
            forceUIUpdate = true; 
            return false; 
        }

        if (cm.data1 >= 24 && cm.data1 <= 28) {
            updateParameterFromCC(cm.data1, cm.data2);
            return false;
        }

        if (cm.data1 == 5 && cm.data2 >= 64) {
            isPB2WiperMode = !isPB2WiperMode; 
            forceUIUpdate = true; 
            settingsNeedSaving = true; 
            lastParameterChangeTime = millis();
        }
        else if (cm.data1 == 6 && cm.data2 >= 64) { 
            bool sendCenterMidi = false;
            
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isVolumeMode = !isVolumeMode; 
            if (!isVolumeMode) {
                volumePedalGain = 1.0f; 
                pedals.lockPB3Whammy();
                sendCenterMidi = true;
                currentPB3 = 8192;
                lastActivePedal = 8192;
                if (!lutNeedsUpdate && pitchShiftLUT != nullptr) pitchShiftFactor = pitchShiftLUT[8192];
            } else {
                pedals.lockPB3Volume();
                lastActivePedal = 8192;
                volumePedalGain = (float)currentPB3 / 16383.0f;
                if (!lutNeedsUpdate && pitchShiftLUT != nullptr) pitchShiftFactor = pitchShiftLUT[8192];
            }
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            
            if (sendCenterMidi) Control_Surface.sendPitchBend(Channel_3, 8192);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 0 && cm.data2 >= 64) { switchEffectMode(activeEffectMode - 1); }
        else if (cm.data1 == 1 && cm.data2 >= 64) { switchEffectMode(activeEffectMode + 1); }
        else if (cm.data1 == 2 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            latencyMode = (latencyMode + 1) % 4; 
            globalAudioResetRequested = true; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; settingsNeedSaving = true; lastParameterChangeTime = millis(); 
        }
        else if (cm.data1 == 3 && cm.data2 >= 64) {
            bool sendCenterMidi = false;
            
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            globalAudioResetRequested = true; 
            bool anyEffectOn = isWhammyActive || isFrozen || isFeedbackActive || isHarmonizerMode || isCapoMode || isSynthMode || isPadMode || isChorusMode || isSwellMode || isVibratoMode || isVolumeMode;
            if (anyEffectOn) {
                isWhammyActive = false; isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false;
                isCapoMode = false; isSynthMode = false; isPadMode = false; isChorusMode = false; isSwellMode = false; isVibratoMode = false; 
                if (isVolumeMode) {
                    isVolumeMode = false;
                    pedals.lockPB3Whammy();
                    sendCenterMidi = true;
                    currentPB3 = 8192;
                }
                volumePedalGain = 1.0f; activeEffectMode = 0; 
            } else { isWhammyActive = true; }
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            
            if (sendCenterMidi) Control_Surface.sendPitchBend(Channel_3, 8192);
            lutNeedsUpdate = true; forceUIUpdate = true;
        }
        else if (cm.data1 == 4 && cm.data2 >= 64) { toggleSampleRate(); }
        else if (cm.data1 == 8 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isFrozen = !isFrozen; if (activeEffectMode == 1) isWhammyActive = isFrozen; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 9 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isFeedbackActive = !isFeedbackActive; if (activeEffectMode == 2) isWhammyActive = isFeedbackActive; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 10 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isHarmonizerMode = !isHarmonizerMode; if (activeEffectMode == 3) isWhammyActive = isHarmonizerMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 12 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isCapoMode = !isCapoMode; if (activeEffectMode == 4) isWhammyActive = isCapoMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            lutNeedsUpdate = true; forceUIUpdate = true; 
        }
        else if (cm.data1 == 13 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isSynthMode = !isSynthMode; if (activeEffectMode == 5) isWhammyActive = isSynthMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 14 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isPadMode = !isPadMode; if (activeEffectMode == 6) isWhammyActive = isPadMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 15 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isChorusMode = !isChorusMode; if (activeEffectMode == 7) isWhammyActive = isChorusMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 16 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isSwellMode = !isSwellMode; if (activeEffectMode == 8) isWhammyActive = isSwellMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 7 && cm.data2 >= 64) { 
            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            isVibratoMode = !isVibratoMode; if (activeEffectMode == 9) isWhammyActive = isVibratoMode; 
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 17 || cm.data1 == 18) {
            float step = (cm.data2 >= 64) ? -1.0f : 1.0f; 

            if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
            if (cm.data1 == 17) {
                if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
                    effectMemory[1] = constrain(effectMemory[1] + step, -24.0f, 24.0f); 
                } else if (activeEffectMode == 4) {
                    effectMemory[4] = constrain(effectMemory[4] + step, -24.0f, 24.0f); 
                } else if (activeEffectMode == 2) { 
                    if (step > 0) feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5; 
                    else feedbackIntervalIdx = (feedbackIntervalIdx + 4) % 5;           
                } else { 
                    effectMemory[activeEffectMode] = constrain(effectMemory[activeEffectMode] + step, -24.0f, 24.0f); 
                }
            } 
            else if (cm.data1 == 18) {
                if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
                    effectMemory[0] = constrain(effectMemory[0] + step, -24.0f, 24.0f); 
                } else if (activeEffectMode == 4) {
                    float centStep = step * 0.01f;
                    effectMemory[4] = constrain(effectMemory[4] + centStep, -24.0f, 24.0f); 
                }
            }
            if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
            
            lutNeedsUpdate = true; forceUIUpdate = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
        }
    }
    return false;
}

void setup() {
    audioBufferMutex = xSemaphoreCreateMutex(); WiFi.mode(WIFI_OFF);
    preferences.begin("whammy_cfg", true); 
    activeEffectMode = constrain(preferences.getInt("activeMode", 0), 0, 9); 
    latencyMode = constrain(preferences.getInt("latMode", 0), 0, 3);
    isPB2WiperMode = preferences.getBool("pb2Wiper", false); 
    currentSampleRate = preferences.getUInt("sampleRate", 48000);
    feedbackIntervalIdx = constrain(preferences.getInt("fbIdx", 0), 0, 4); 
    
    AppSettings savedSettings;
    size_t len = preferences.getBytes("dspData", &savedSettings, sizeof(AppSettings));
    if (len == sizeof(AppSettings)) {
        for(int i = 0; i < 10; i++) {
            effectMemory[i] = savedSettings.fxMem[i]; 
            for (int p = 0; p < 5; p++) { 
                fxParams[i][p] = savedSettings.params[i][p]; 
            }
        }
    }
    preferences.end();

    pinMode(BATTERY_PIN, INPUT);
    pinMode(38, OUTPUT); digitalWrite(38, LOW); pinMode(15, OUTPUT); digitalWrite(15, HIGH);
    
    Serial.begin(115200); tft.init(); tft.setRotation(1); 
    spr.createSprite(tft.width(), tft.height()); meterSpr.createSprite(6, 98);
    tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM); tft.setTextSize(3); tft.setTextColor(TFT_WHITE, TFT_BLACK); 
    tft.drawString("BOOTING...", 160, 85);
    
    delay(120); digitalWrite(38, HIGH); btmidi.setName("Whammy_S3"); 
    pinMode(pinPB, INPUT_PULLUP); pinMode(pinPB2, INPUT_PULLUP); pinMode(pinPB3, INPUT_PULLUP); 
    
    // FIX 1: Reverted to PSRAM. 96KB of internal SRAM exhausted the ESP32 baseband allocation, 
    // starving the BLE stack and I2S DMA, resulting in silent hardware failure.
    delayBuffer = (float*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    fbDelayBuffer = (float*)heap_caps_aligned_alloc(16, FB_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    freezeBuffer = (float*)heap_caps_aligned_alloc(16, FREEZE_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    pitchShiftLUT = (float*)heap_caps_aligned_alloc(16, 16384 * sizeof(float), MALLOC_CAP_SPIRAM);
    pitchShiftLUT_temp = (float*)heap_caps_aligned_alloc(16, 16384 * sizeof(float), MALLOC_CAP_SPIRAM);
    
    if (delayBuffer == nullptr || fbDelayBuffer == nullptr || freezeBuffer == nullptr || pitchShiftLUT == nullptr || pitchShiftLUT_temp == nullptr) {
        tft.fillScreen(TFT_RED); tft.setTextColor(TFT_WHITE, TFT_RED); tft.drawString("MEMORY ERROR", 160, 85); while(1) { delay(100); }
    }
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float)); memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(float)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(float));
    memset(pitchShiftLUT, 0, 16384 * sizeof(float)); memset(pitchShiftLUT_temp, 0, 16384 * sizeof(float)); memset(hannLUT, 0, 1024 * sizeof(float));           
    
    for (int i = 0; i < 1024; i++) { 
        hannLUT[i] = 0.5f * (1.0f - cosf(TWO_PI * ((float)i / 1023.0f))); 
        lfoLUT[i] = powf(2.0f, (15.0f * sinf(TWO_PI * ((float)i / 1024.0f))) / 1200.0f); 
    }
    for (int i = 0; i < 2048; i++) { synthLUT[i] = sinf((((float)i - 1024.0f) / 1024.0f) * 45.0f); }
    
    FilteredAnalog<>::setupADC(); delay(500); calibratePBs(); 
    
    updateLUT();
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    pitchShiftFactor = pitchShiftLUT[8192];
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    Control_Surface >> pipes >> btmidi; Control_Surface >> pipes >> usbmidi; 
    usbmidi >> pipes >> Control_Surface; btmidi >> pipes >> Control_Surface;
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr); 
    Control_Surface.begin();

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
    
    i2s_chan_config_t i2sConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); 
    i2sConfig.dma_desc_num = 8; i2sConfig.dma_frame_num = HOP_SIZE; i2sConfig.auto_clear = true;
    i2s_new_channel(&i2sConfig, &tx_chan, &rx_chan);
    
    i2s_std_config_t stdConfig = { 
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate), 
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg = { .mclk = GPIO_NUM_43, .bclk = GPIO_NUM_44, .ws = GPIO_NUM_18, .dout = GPIO_NUM_16, .din = GPIO_NUM_17 } 
    };
    
    i2s_channel_init_std_mode(tx_chan, &stdConfig); i2s_channel_init_std_mode(rx_chan, &stdConfig); 
    i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan);
    
    xTaskCreatePinnedToCore(DisplayTask, "UI", 8192, NULL, 1, NULL, 0); 
    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);
}

void loop() {
    static unsigned long lastLutUpdate = 0;
    if (lutNeedsUpdate && (millis() - lastLutUpdate > 40)) { 
        // FIX 2: Clear the flag BEFORE processing so rapid MIDI interrupts aren't discarded
        lutNeedsUpdate = false; 
        
        updateLUT(); 
        if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
        pitchShiftFactor = pitchShiftLUT[constrain(lastActivePedal, 0, 16383)]; 
        if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
        
        lastLutUpdate = millis();
    }

    if (clearBuffersRequested) {
        if (audioBufferMutex != NULL && xSemaphoreTake(audioBufferMutex, portMAX_DELAY) == pdTRUE) {
            memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float)); memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(float)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(float));
            clearBuffersRequested = false; xSemaphoreGive(audioBufferMutex);
        }
    }
    
    if (settingsNeedSaving && (millis() - lastParameterChangeTime > 2000) && (millis() - lastActivityTime > 2000)) { 
        saveSettings(); 
        settingsNeedSaving = false; 
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
}