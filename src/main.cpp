#pragma GCC optimize ("O3, tree-vectorize")
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

// --- MEMORY PREFERENCES ---
Preferences preferences;
volatile bool settingsNeedSaving = false;
unsigned long lastParameterChangeTime = 0;

// --- PEDAL CONFIGURATION ---
// Set to 'false' to map the expression pedal linearly (Heel = 0, Toe = 16383)
const bool INVERT_PB3 = false; 

// --- BARE-METAL PRE-BOOT ASSASSIN ---
void __attribute__((constructor)) pre_boot_kill_switch() {
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_38, 0); 
    
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_15, 0);
    
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 0);
}

/// --- GLOBALS & I2S HANDLES ---
i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;

// --- FREE-RTOS MUTEX ---
SemaphoreHandle_t audioBufferMutex = NULL;

volatile uint32_t currentSampleRate = 48000; 
#define HOP_SIZE 64            

// --- TIME-DOMAIN DSP BUFFERS ---
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF 
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 131072

float* delayBuffer = nullptr;    
float* fbDelayBuffer = nullptr;  
float* freezeBuffer = nullptr;   
float* pitchShiftLUT = nullptr; 
float* pitchShiftLUT_temp = nullptr; 

int writeIndex = 0;
int fbDelayWriteIdx = 0;

// --- DSP LOOK-UP TABLES ---
#define HANN_LUT_SIZE 1024
#define LFO_LUT_SIZE 1024
#define WAVE_LUT_SIZE 2048

DRAM_ATTR float hannLUT[HANN_LUT_SIZE];
DRAM_ATTR float lfoLUT[LFO_LUT_SIZE];
DRAM_ATTR float synthLUT[WAVE_LUT_SIZE];

// --- DSP PRE-CALCULATED RATIOS ---
volatile float globalHarmRatio = 1.0f;
volatile float globalChorusRatio = 1.0f;
volatile float globalFbRatio = 1.0f;
volatile float globalVibratoPhaseInc = 0.0f;

// --- DEDICATED INDEPENDENT TAP STATES ---
uint32_t tap_w1_1 = 0; 
uint32_t tap_w1_2 = 256 << 16; 

uint32_t tap_w2_1 = 0; 
uint32_t tap_w2_2 = 256 << 16; 

uint32_t tap_w3_1 = 0; 
uint32_t tap_w3_2 = 256 << 16; 

uint32_t tap_w4_1 = 0; 
uint32_t tap_w4_2 = 256 << 16; 

uint32_t tap_w5_1 = 0; 
uint32_t tap_w5_2 = 256 << 16; 

float currentWindowSize = 512.0f; 

// --- FREEZE STATE & ALL-PASS FILTERS ---
int freezeLength = 48000; 
bool wasFrozen = false;
volatile bool apfNeedsClear = false;
volatile float freezeRamp = 0.0f;

float apf1Buffer[1009] = { 0.0f }; 
int apf1Idx = 0;

float apf2Buffer[863] = { 0.0f };  
int apf2Idx = 0;

const float intervalList[] = {-12.0f, -7.0f, -5.0f, -2.0f, 0.0f, 2.0f, 5.0f, 7.0f, 12.0f};
int currentIntervalIdx = 8; 
volatile int feedbackIntervalIdx = 0; 

volatile bool lutNeedsUpdate = false;
volatile uint16_t lastActivePedal = 8192; 
void updateLUT();
TaskHandle_t audioTaskHandle = NULL; 

// --- TFT DISPLAY ---
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft); 
TFT_eSprite meterSpr = TFT_eSprite(&tft); 
volatile bool forceUIUpdate = true; 

// --- EFFECT STATE ---
volatile int activeEffectMode = 0; 
volatile float effectMemory[10] = { 12.0f, 12.0f, 12.0f, 5.0f, -2.0f, -12.0f, -12.0f, 12.0f, 0.0f, 0.0f };
volatile float pitchShiftFactor = 1.0f;

volatile bool isWhammyActive = true;  
volatile bool isFrozen = false;
volatile bool isFeedbackActive = false;
volatile bool isHarmonizerMode = false;
volatile bool isSynthMode = false;
volatile bool isPadMode = false;
volatile bool isCapoMode = false; 
volatile bool isChorusMode = false; 
volatile bool isSwellMode = false; 
volatile bool isVibratoMode = false; 
volatile bool isVolumeMode = false; 
volatile bool isPB2LinearMode = false; // False = Hall/Spring, True = Wiper

volatile float chorusLfoPhase = 0.0f;
volatile float feedbackLfoPhase = 0.0f;
volatile float vibratoLfoPhase = 0.0f;
volatile float swellGain = 0.0f; 
volatile float volumePedalGain = 1.0f; 

volatile float feedbackRamp = 0.0f;
float fbHpfState = 0.0f;
float feedbackFilter = 0.0f;
volatile int latencyMode = 0; 
const float LATENCY_WINDOWS[] = {512.0f, 1024.0f, 2048.0f, 4096.0f};

// --- GLOBAL BUFFER WIPE FLAG ---
volatile bool globalAudioResetRequested = false;
volatile bool clearBuffersRequested = false;
volatile int hardwareSyncMuteFrames = 0; 

// --- POWER SAVING, BATTERY & UI GLOBALS ---
unsigned long lastActivityTime = 0;       
unsigned long lastScreenActivityTime = 0;
const unsigned long LIGHT_SLEEP_TIMEOUT = 600000;
const unsigned long SCREEN_OFF_TIMEOUT = 1200000;  
bool isScreenOff = false;
volatile bool wakeupPending = false; 
volatile float core1_load = 0.0f; 

volatile bool sleepRequested = false;
volatile bool isSleeping = false;

const int BATTERY_PIN = 4;
volatile int currentBatteryPercent = 100;
volatile bool isBatteryCharging = false;

// --- PIN ASSIGNMENTS ---
pin_t pinPB = 1;     
pin_t pinPB2 = 2;
pin_t pinPB3 = 10;    
const int BOOT_SENSE_PIN = 0; 
const int CAROUSEL_BUTTON_PIN = 14; 

uint16_t lastMidiSent = 8192;
volatile uint16_t currentPB1 = 8192;
volatile uint16_t currentPB2 = 8192;
volatile uint16_t currentPB3 = 8192;
volatile uint16_t currentCC11 = 0;
volatile float ui_audio_level = 0.0f; 
volatile float ui_output_level = 0.0f;

// --- CONTINUOUS AUTO-CALIBRATION BOUNDS ---
uint16_t PB1_raw_min = 1000;
uint16_t PB1_raw_max = 3000;
uint16_t PB1_raw_center = 2048;

uint16_t PB2_raw_min = 1000;
uint16_t PB2_raw_max = 3000;
uint16_t PB2_raw_center = 2048;

uint16_t PB3_raw_min = 1000;
uint16_t PB3_raw_max = 3000;

int deadzone_size = 400; 

FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB2 = pinPB2;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB3 = pinPB3;
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;

// --- FLASH MEMORY SAVER FUNCTION ---
void saveSettings() {
    preferences.begin("whammy_cfg", false); 
    
    preferences.putInt("activeMode", activeEffectMode);
    preferences.putInt("latMode", latencyMode);
    preferences.putBool("pb2Linear", isPB2LinearMode);
    preferences.putUInt("sampleRate", currentSampleRate);
    
    for(int i = 0; i < 10; i++) {
        char key[8];
        sprintf(key, "fxMem%d", i);
        preferences.putFloat(key, effectMemory[i]);
    }
    
    preferences.end();
}

// --- NON-LINEAR LIPO BATTERY CURVE CALCULATION ---
int getBatteryPercentage(float voltage) {
    if (voltage >= 4.15f) return 100;
    if (voltage >= 4.00f) return 90;
    if (voltage >= 3.90f) return 80;
    if (voltage >= 3.80f) return 70;
    if (voltage >= 3.75f) return 60;
    if (voltage >= 3.70f) return 50;
    if (voltage >= 3.65f) return 40;
    if (voltage >= 3.60f) return 30;
    if (voltage >= 3.55f) return 20;
    if (voltage >= 3.50f) return 10;
    if (voltage <= 3.30f) return 0;
    return (int)((voltage - 3.3f) / (4.15f - 3.3f) * 100.0f);
}

// --- DYNAMIC RAW DEADZONE MAPPING (PB1 & PB2) ---
analog_t map_raw_deadzone(int raw, uint16_t center, uint16_t rMin, uint16_t rMax, int dZone) {
    int outerDeadzone = 350; 
    
    int deadLower = center - dZone;
    int deadUpper = center + dZone;
    
    int effMin = rMin + outerDeadzone;
    int effMax = rMax - outerDeadzone;
    
    if (effMin >= deadLower) effMin = rMin + 50; 
    if (effMax <= deadUpper) effMax = rMax - 50;
    
    if (raw <= effMin) return 0;
    if (raw >= effMax) return 16383;
    
    if (raw >= deadLower && raw <= deadUpper) return 8192;
    
    long mappedValue;
    if (raw < deadLower) {
        mappedValue = map(raw, effMin, deadLower, 0, 8191);
    } else {
        mappedValue = map(raw, deadUpper, effMax, 8193, 16383);
    }
    
    return constrain(mappedValue, 0, 16383);
}

// --- DYNAMIC RAW EXPRESSION MAPPING (PB3) ---
analog_t map_raw_expression(int raw, uint16_t rMin, uint16_t rMax, bool invert) {
    int heelLockZone = 350; 
    int toeLockZone = 300;   
    
    int lowerLimit;
    int upperLimit;

    if (!invert) {
        lowerLimit = rMin + heelLockZone;
        upperLimit = rMax - toeLockZone;
        
        if (lowerLimit >= upperLimit) { 
            lowerLimit = rMin; 
            upperLimit = rMax; 
        }
        
        if (raw <= lowerLimit) {
            return 0; 
        }
        if (raw >= upperLimit) {
            return 16383; 
        }
        
        return map(raw, lowerLimit, upperLimit, 0, 16383);
        
    } else {
        lowerLimit = rMin + toeLockZone;
        upperLimit = rMax - heelLockZone;
        
        if (lowerLimit >= upperLimit) { 
            lowerLimit = rMin; 
            upperLimit = rMax; 
        }
        
        if (raw <= lowerLimit) {
            return 16383; 
        }
        if (raw >= upperLimit) {
            return 0; 
        }
        
        return map(raw, lowerLimit, upperLimit, 16383, 0);
    }
}

void calibratePBs() {
    for (int i = 0; i < 50; i++) { 
        filterPB.update(); 
        filterPB2.update(); 
        filterPB3.update(); 
        delay(1); 
    }
    
    long sum1 = 0;
    long sum2 = 0;
    
    for (int i = 1; i <= 250; i++) {
        filterPB.update(); 
        filterPB2.update(); 
        filterPB3.update();
        
        sum1 += filterPB.getValue(); 
        sum2 += filterPB2.getValue();
        delay(1);
    }
    
    PB1_raw_center = sum1 / 250;
    PB2_raw_center = sum2 / 250;
    
    if (PB1_raw_center > 4000 || PB1_raw_center < 100) {
        PB1_raw_center = 2048;
    }
    
    if (PB2_raw_center > 4000 || PB2_raw_center < 100) {
        PB2_raw_center = 2048;
    }
    
    PB1_raw_min = PB1_raw_center - 200; 
    PB1_raw_max = PB1_raw_center + 200;
    
    PB2_raw_min = PB2_raw_center - 200; 
    PB2_raw_max = PB2_raw_center + 200;
    
    PB3_raw_min = 1000; 
    PB3_raw_max = 3000; 
}

// --- SAMPLE RATE HARDWARE TOGGLE ---
void toggleSampleRate() {
    sleepRequested = true; 
    globalAudioResetRequested = true; 
    
    int timeoutCounter = 0;
    while (!isSleeping && timeoutCounter < 40) { 
        vTaskDelay(pdMS_TO_TICKS(5)); 
        timeoutCounter++;
    }
    
    i2s_channel_disable(tx_chan);
    i2s_channel_disable(rx_chan);
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    if (currentSampleRate == 96000) {
        currentSampleRate = 48000;
    } else {
        currentSampleRate = 96000;
    }
    
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate);
    i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
    i2s_channel_reconfig_std_clock(rx_chan, &clk_cfg);
    
    freezeLength = currentSampleRate;
    lutNeedsUpdate = true;
    
    i2s_channel_enable(tx_chan);
    i2s_channel_enable(rx_chan);
    
    hardwareSyncMuteFrames = (currentSampleRate / HOP_SIZE) * 0.15f;
    
    sleepRequested = false;
    forceUIUpdate = true;
    
    settingsNeedSaving = true;
    lastParameterChangeTime = millis();
}

// --- LCD & SLEEP CONTROL ---
void turnScreenOff() { 
    if (!isScreenOff) { 
        digitalWrite(38, LOW); 
        digitalWrite(15, LOW); 
        isScreenOff = true; 
    } 
}

void turnScreenOn() { 
    if (isScreenOff && !wakeupPending) { 
        wakeupPending = true; 
    } 
}

void goToLightSleep() {
    turnScreenOff(); 
    sleepRequested = true; 
    int timeoutCounter = 0;
    
    while (!isSleeping && timeoutCounter < 10) { 
        vTaskDelay(pdMS_TO_TICKS(10)); 
        timeoutCounter++; 
    }
    
    i2s_channel_disable(tx_chan); 
    i2s_channel_disable(rx_chan);      
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    
    rtc_gpio_init(GPIO_NUM_14); 
    rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(GPIO_NUM_14); 
    esp_sleep_enable_ext1_wakeup(1ULL << 14, ESP_EXT1_WAKEUP_ANY_LOW);
    
    delay(50); 
    esp_light_sleep_start();
    
    rtc_gpio_deinit(GPIO_NUM_14); 
    pinMode(CAROUSEL_BUTTON_PIN, INPUT_PULLUP);
    
    i2s_channel_enable(tx_chan); 
    i2s_channel_enable(rx_chan);
    
    sleepRequested = false; 
    timeoutCounter = 0;
    
    while (isSleeping && timeoutCounter < 10) { 
        vTaskDelay(pdMS_TO_TICKS(10)); 
        timeoutCounter++; 
    }
    
    vTaskDelay(pdMS_TO_TICKS(200)); 
    turnScreenOn(); 
}

// --- OPTIMIZED FIXED-POINT LINEAR HERMITE INTERPOLATOR ---
inline float IRAM_ATTR processTap(uint32_t tapPhase, const float* buffer, int currentWriteIdx, uint32_t windowMask, uint32_t hannIntMult) {
    int T = (tapPhase >> 16) & windowMask;
    float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
    
    int effTap = T + 2; 
    
    int idx1 = (currentWriteIdx - effTap + MAX_BUFFER_SIZE) & BUFFER_MASK;
    int idx0 = (idx1 + 1) & BUFFER_MASK; 
    int idx2 = (idx1 - 1 + MAX_BUFFER_SIZE) & BUFFER_MASK; 
    int idx3 = (idx1 - 2 + MAX_BUFFER_SIZE) & BUFFER_MASK;
    
    float y0 = buffer[idx0]; 
    float y1 = buffer[idx1];
    float y2 = buffer[idx2]; 
    float y3 = buffer[idx3];
    
    float c0 = y1; 
    float c1 = 0.5f * (y2 - y0); 
    float c3 = 1.5f * (y1 - y2) + 0.5f * (y3 - y0); 
    float c2 = y0 - y1 + c1 - c3;
    
    float sample = ((c3 * frac + c2) * frac + c1) * frac + c0;
    
    int lutIdx = (T * hannIntMult) >> 16;
    return sample * hannLUT[lutIdx];
}

void updateLUT() {
    float basePitch = 0.0f; 
    
    if (isCapoMode || (activeEffectMode == 4 && isWhammyActive)) { 
        basePitch += effectMemory[4]; 
    }
    
    float toeBend = effectMemory[0]; 
    float heelBend = effectMemory[5];
    
    for (int i = 0; i < 16384; i++) {
        float normalizedThrow = (i >= 8192) ? ((float)(i - 8192) / 8191.0f) : ((float)(i - 8192) / 8192.0f);
        float dynamicBend = (normalizedThrow >= 0.0f) ? (toeBend * normalizedThrow) : (heelBend * fabsf(normalizedThrow));
        pitchShiftLUT_temp[i] = powf(2.0f, (basePitch + dynamicBend) / 12.0f);
        
        if (i % 2048 == 0) { vTaskDelay(pdMS_TO_TICKS(1)); }
    }
    
    memcpy(pitchShiftLUT, pitchShiftLUT_temp, 16384 * sizeof(float));
    
    globalHarmRatio = powf(2.0f, effectMemory[3] / 12.0f);
    globalChorusRatio = powf(2.0f, effectMemory[8] / 12.0f);
    
    float fbIntervals[5] = {0.0f, 12.0f, 19.0f, 24.0f, 28.0f}; 
    globalFbRatio = powf(2.0f, fbIntervals[feedbackIntervalIdx % 5] / 12.0f);
    
    float vibHz = (effectMemory[9] != 0.0f) ? fabsf(effectMemory[9]) : 2.0f;
    globalVibratoPhaseInc = (vibHz * LFO_LUT_SIZE) / (float)currentSampleRate;
}

void updateMeters() {
    int barHeight = 98;
    int inFillHeight = constrain((int)(ui_audio_level * barHeight), 0, barHeight); 
    
    meterSpr.fillSprite(TFT_BLACK); 
    meterSpr.fillRect(0, barHeight - inFillHeight, 6, inFillHeight, (ui_audio_level > 0.90f) ? TFT_RED : TFT_GREEN); 
    meterSpr.pushSprite(11, 31);
    
    int outFillHeight = constrain((int)(ui_output_level * barHeight), 0, barHeight); 
    
    meterSpr.fillSprite(TFT_BLACK); 
    meterSpr.fillRect(0, barHeight - outFillHeight, 6, outFillHeight, (ui_output_level > 0.90f) ? TFT_RED : TFT_GREEN); 
    meterSpr.pushSprite(spr.width() - 17, 31);
}

void updateDisplay() {
    spr.fillSprite(TFT_BLACK); 
    
    // UI - Battery Indicator (Top Left)
    char batStr[16];
    if (isBatteryCharging) {
        sprintf(batStr, "CHG %d%%", currentBatteryPercent);
        spr.setTextColor(TFT_GREEN, TFT_BLACK); 
    } else {
        sprintf(batStr, "BAT %d%%", currentBatteryPercent);
        if (currentBatteryPercent > 20) {
            spr.setTextColor(TFT_GREEN, TFT_BLACK); 
        } else {
            spr.setTextColor(TFT_RED, TFT_BLACK); 
        }
    }
    spr.setTextDatum(TL_DATUM); 
    spr.drawString(batStr, 5, 5); 

    spr.setTextDatum(MC_DATUM); 
    spr.setTextSize(1);
    
    if (btmidi.isConnected() == true) { 
        spr.setTextColor(TFT_GREEN, TFT_BLACK); 
        spr.drawString("BT: Connected", spr.width() / 2, 10); 
    } else { 
        spr.setTextColor(TFT_YELLOW, TFT_BLACK); 
        spr.drawString("BT: Waiting", spr.width() / 2, 10); 
    }

    spr.drawRect(10, 30, 8, 100, TFT_DARKGREY); 
    spr.setTextColor(TFT_WHITE, TFT_BLACK); 
    spr.drawString("IN", 14, 140);
    
    spr.drawRect(spr.width() - 18, 30, 8, 100, TFT_DARKGREY); 
    spr.drawString("OUT", spr.width() - 14, 140);

    bool effectIsActive = false;
    if (activeEffectMode == 0) {
        effectIsActive = isWhammyActive;
    } else if (activeEffectMode == 1) {
        effectIsActive = (isWhammyActive || isFrozen);
    } else if (activeEffectMode == 2) {
        effectIsActive = (isWhammyActive || isFeedbackActive);
    } else if (activeEffectMode == 3) {
        effectIsActive = (isWhammyActive || isHarmonizerMode);
    } else if (activeEffectMode == 4) {
        effectIsActive = (isWhammyActive || isCapoMode);
    } else if (activeEffectMode == 5) {
        effectIsActive = (isWhammyActive || isSynthMode);
    } else if (activeEffectMode == 6) {
        effectIsActive = (isWhammyActive || isPadMode);
    } else if (activeEffectMode == 7) {
        effectIsActive = (isWhammyActive || isChorusMode);
    } else if (activeEffectMode == 8) {
        effectIsActive = (isWhammyActive || isSwellMode);
    } else if (activeEffectMode == 9) {
        effectIsActive = (isWhammyActive || isVibratoMode);
    }
    
    spr.fillCircle(spr.width() - 12, 12, 6, (effectIsActive == true) ? TFT_GREEN : TFT_RED); 
    spr.drawCircle(spr.width() - 12, 12, 6, TFT_WHITE);

    spr.setTextSize(3); 
    int titleXPosition = 215;
    const char* effectTitleNames[] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};
    uint32_t effectTitleColors[] = {TFT_ORANGE, TFT_CYAN, TFT_RED, TFT_MAGENTA, TFT_GREEN, TFT_YELLOW, TFT_PINK, TFT_SKYBLUE, TFT_WHITE, TFT_PURPLE};
    
    spr.setTextColor(effectTitleColors[activeEffectMode], TFT_BLACK); 
    spr.drawString(effectTitleNames[activeEffectMode], titleXPosition, 30);

    spr.setTextColor(TFT_WHITE);
    if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
        char intervalTop[16]; 
        char intervalBottom[16]; 
        spr.setTextSize(3); 
        sprintf(intervalTop, "%+.1f", effectMemory[0]); 
        sprintf(intervalBottom, "%+.1f", effectMemory[5]); 
        spr.drawString(intervalTop, titleXPosition, 60); 
        spr.drawString(intervalBottom, titleXPosition, 85);
    } else if (activeEffectMode == 4) {
        char intervalCapo[16]; 
        spr.setTextSize(4); 
        sprintf(intervalCapo, "%+.2f", effectMemory[4]); 
        spr.drawString(intervalCapo, titleXPosition, 75);
    } else {
        char intervalSingle[16]; 
        spr.setTextSize(4); 
        float displayedValue = effectMemory[activeEffectMode];
        if (activeEffectMode == 2) { 
            float feedbackIntervals[] = {0.0f, 12.0f, 19.0f, 24.0f, 28.0f}; 
            displayedValue = feedbackIntervals[feedbackIntervalIdx % 5]; 
        }
        sprintf(intervalSingle, "%+.1f", displayedValue); 
        spr.drawString(intervalSingle, titleXPosition, 75);
    }

    int gaugeTopY = 30; 
    int gaugeBottomY = 125;
    int xPB1 = 40; 
    int xPB2 = 75; 
    int xPB3 = 110; 
    int xCC11 = 145;
    
    spr.setTextSize(1); 
    spr.drawString("PB1", xPB1, gaugeBottomY + 15); 
    
    // Dynamic PB2 Hardware Label
    if (isPB2LinearMode) {
        spr.drawString("PB2 W", xPB2, gaugeBottomY + 15); 
    } else {
        spr.drawString("PB2 H", xPB2, gaugeBottomY + 15); 
    }
    
    // Dynamic PB3 Label based on Volume Mode status
    if (isVolumeMode) {
        spr.drawString("Vol", xPB3, gaugeBottomY + 15);
    } else {
        spr.drawString("PB3", xPB3, gaugeBottomY + 15); 
    }
    
    spr.drawString("CC11", xCC11, gaugeBottomY + 15);
    
    for (int yStep = gaugeTopY; yStep <= gaugeBottomY; yStep += 5) { 
        spr.drawFastVLine(xPB1, yStep, 2, TFT_DARKGREY); 
        spr.drawFastVLine(xPB2, yStep, 2, TFT_DARKGREY); 
        spr.drawFastVLine(xPB3, yStep, 2, TFT_DARKGREY); 
        spr.drawFastVLine(xCC11, yStep, 2, TFT_DARKGREY); 
    }
    
    spr.fillCircle(xPB1, map(currentPB1, 0, 16383, gaugeBottomY, gaugeTopY), 4, TFT_CYAN); 
    spr.fillCircle(xPB2, map(currentPB2, 0, 16383, gaugeBottomY, gaugeTopY), 4, TFT_MAGENTA);
    spr.fillCircle(xPB3, map(currentPB3, 0, 16383, gaugeBottomY, gaugeTopY), 4, TFT_YELLOW); 
    spr.fillCircle(xCC11, map(currentCC11, 0, 16383, gaugeBottomY, gaugeTopY), 4, TFT_GREEN);

    int statsRowY = 162; 
    spr.setTextSize(1); 
    spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK); 
    spr.setTextDatum(ML_DATUM); 
    
    char cpuUsageBuffer[16]; 
    sprintf(cpuUsageBuffer, "CPU:%2d%%", (int)core1_load); 
    spr.drawString(cpuUsageBuffer, 2, statsRowY);
    
    char internalSramBuffer[16]; 
    sprintf(internalSramBuffer, "SRM:%dK", (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024)); 
    spr.drawString(internalSramBuffer, 52, statsRowY);
    
    char psramBuffer[16]; 
    sprintf(psramBuffer, "PSR:%dK", (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024)); 
    spr.drawString(psramBuffer, 107, statsRowY);

    spr.setTextDatum(MC_DATUM); 
    spr.setTextColor(TFT_WHITE); 
    
    // UI - Sample Rate Box
    spr.drawRect(165, statsRowY - 7, 50, 14, TFT_DARKGREY);
    if (currentSampleRate == 96000) {
        spr.drawString("96kHz", 190, statsRowY);
    } else {
        spr.drawString("48kHz", 190, statsRowY);
    }
    
    // UI - Latency Box
    spr.drawRect(225, statsRowY - 7, 85, 14, TFT_DARKGREY);
    const char* latencyLabelStrings[] = {"U.Low Lat", "Low Lat", "Mid Lat", "High Lat"}; 
    spr.drawString(latencyLabelStrings[latencyMode], 267, statsRowY);

    spr.setTextSize(2); 
    spr.setTextDatum(MC_DATUM); 
    int bannerCount = 0;
    int gridStartX = 175; 
    int gridWidthX = 110; 
    int gridStepX = gridWidthX / 2; 
    int gridStartY = 110; 
    int gridStepY = 18;  

    auto drawActiveEffectBanner = [&](const char* shortLabel, uint32_t effectColor) {
        int currentColumn = bannerCount % 3; 
        int currentRow = bannerCount / 3;
        int drawX = gridStartX + (currentColumn * gridStepX); 
        int drawY = gridStartY + (currentRow * gridStepY);
        spr.setTextColor(effectColor, TFT_BLACK); 
        spr.drawString(shortLabel, drawX, drawY); 
        bannerCount++;
    };
    
    if (isFrozen == true && activeEffectMode != 1) { drawActiveEffectBanner("FRZ", TFT_CYAN); } 
    if (isFeedbackActive == true && activeEffectMode != 2) { drawActiveEffectBanner("SCM", TFT_RED); }
    if (isHarmonizerMode == true && activeEffectMode != 3) { drawActiveEffectBanner("HRM", TFT_MAGENTA); } 
    if (isCapoMode == true && activeEffectMode != 4) { drawActiveEffectBanner("CAP", TFT_GREEN); }
    if (isSynthMode == true && activeEffectMode != 5) { drawActiveEffectBanner("SYN", TFT_YELLOW); } 
    if (isPadMode == true && activeEffectMode != 6) { drawActiveEffectBanner("PAD", TFT_PINK); }
    if (isChorusMode == true && activeEffectMode != 7) { drawActiveEffectBanner("CHO", TFT_SKYBLUE); } 
    if (isSwellMode == true && activeEffectMode != 8) { drawActiveEffectBanner("SWL", TFT_WHITE); }
    if (isVibratoMode == true && activeEffectMode != 9) { drawActiveEffectBanner("VIB", TFT_PURPLE); } 
    if (isVolumeMode == true) { drawActiveEffectBanner("VOL", TFT_DARKGREY); }

    spr.pushSprite(0, 0); 
    updateMeters();
}

struct DebouncedButton {
    uint8_t pin; 
    bool state; 
    bool lastReading; 
    bool isActive; 
    unsigned long lastDebounceTime; 
    unsigned long pressedTime;
    
    DebouncedButton(uint8_t p) { 
        pin = p; 
        state = HIGH; 
        lastReading = HIGH; 
        lastDebounceTime = 0; 
        pressedTime = 0; 
        isActive = false; 
    }
    
    bool update(unsigned long delay = 50) {
        bool current = digitalRead(pin); 
        bool changed = false;
        
        if (current != lastReading) { 
            lastDebounceTime = millis(); 
        }
        
        if ((millis() - lastDebounceTime) > delay) { 
            if (current != state) { 
                state = current; 
                changed = true; 
            } 
        }
        
        lastReading = current; 
        return changed;
    }
};

void DisplayTask(void * pvParameters) {
    bool metersNeedClear = false;
    for (;;) {
        if (wakeupPending) { 
            pinMode(15, OUTPUT); 
            digitalWrite(15, HIGH); 
            tft.init(); 
            vTaskDelay(pdMS_TO_TICKS(120)); 
            
            pinMode(38, OUTPUT); 
            digitalWrite(38, HIGH); 
            isScreenOff = false; 
            wakeupPending = false; 
            forceUIUpdate = true; 
        }
        
        if (forceUIUpdate) { 
            forceUIUpdate = false; 
            updateDisplay(); 
            metersNeedClear = true;
        } else if (!isScreenOff) {
            if (ui_audio_level > 0.02f || ui_output_level > 0.02f) { 
                updateMeters(); 
                metersNeedClear = true;
            } else if (metersNeedClear) {
                // Instantly zero the meters to draw a blank frame and stop CPU burn
                ui_audio_level = 0.0f;
                ui_output_level = 0.0f;
                updateMeters();
                metersNeedClear = false;
            }
        }
        
        // RELAXED DISPLAY REFRESH RATE
        // Yields 33ms (~30 FPS) to drastically reduce Core 0 SPI strain and protect Bluetooth stability
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

// --- HARDWARE ACCELERATED BLOCK PROCESSING AUDIO TASK ---
void IRAM_ATTR AudioDSPTask(void * pvParameters) {
    static int32_t i2s_in_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_out_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    
    static float input_block[HOP_SIZE] __attribute__((aligned(16)));
    static float dc_block[HOP_SIZE] __attribute__((aligned(16)));
    static float mix_block[HOP_SIZE] __attribute__((aligned(16)));
    
    // Memory aligned arrays perfectly formatted for the ESP32 SIMD Instruction set
    static float w1_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w2_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w3_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w4_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w5_block[HOP_SIZE] __attribute__((aligned(16)));
    static float dry_block[HOP_SIZE] __attribute__((aligned(16)));
    static float fz_block[HOP_SIZE] __attribute__((aligned(16)));
    static float pad_block[HOP_SIZE] __attribute__((aligned(16)));
    static float fbOut_block[HOP_SIZE] __attribute__((aligned(16)));
    
    static float dc_coeffs[5] = {1.0f, -1.0f, 0.0f, -0.995f, 0.0f}; 
    static float dc_state[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    static float synthEnv = 0.0f; 
    static float synthFilter = 0.0f;
    static float padFilter = 0.0f; 
    static float padEnv = 0.0f;
    static float inputEnvelope = 0.0f; 
    static float feedbackFilterVar = 0.0f;
    
    // ADDED: Static variable for LPF per-sample volume smoothing
    static float smoothedVolGain = 1.0f;
    
    static int freezeWriteIdxVar = 0; 
    static int freezePlayCounterVar = 0; 
    static int freezeStartIdxVar = 0;
    static int activeFreezeLength = 48000;
    
    const float normFactor = 1.0f / 2147483648.0f; 
    const float DC_OFFSET = 1e-9f;
    
    for (;;) {
        if (sleepRequested) { 
            isSleeping = true; 
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue; 
        }
        
        isSleeping = false; 
        size_t bytesRead; 
        i2s_channel_read(rx_chan, i2s_in_block, sizeof(i2s_in_block), &bytesRead, portMAX_DELAY);
        
        if (bytesRead > 0) {
            int framesRead = bytesRead / 8; 
            
            // FIX: Hardware Sync Mute Block
            // Intercepts and throws away I2S blocks while the ADC PLL is locking onto the new clock
            if (hardwareSyncMuteFrames > 0) {
                hardwareSyncMuteFrames--;
                memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                ui_audio_level = 0.0f;
                ui_output_level = 0.0f;
                
                // Flush the DC blocker state so it doesn't remember the transient spike
                for(int j = 0; j < 4; j++) { dc_state[j] = 0.0f; }
                
                size_t bytesWrittenCount; 
                i2s_channel_write(tx_chan, i2s_out_block, framesRead * 8, &bytesWrittenCount, portMAX_DELAY);
                continue; // Skip all DSP processing for this block
            }
            
            if (globalAudioResetRequested) {
                synthEnv = 0.0f; 
                synthFilter = 0.0f; 
                padFilter = 0.0f; 
                padEnv = 0.0f;
                inputEnvelope = 0.0f; 
                feedbackFilterVar = 0.0f;
                smoothedVolGain = 1.0f; // Reset smoothing safely on bypass
                
                freezeWriteIdxVar = 0; 
                freezePlayCounterVar = 0; 
                freezeStartIdxVar = 0;
                activeFreezeLength = currentSampleRate;
                fbDelayWriteIdx = 0;
                writeIndex = 0;
                
                for(int j = 0; j < 4; j++) { 
                    dc_state[j] = 0.0f; 
                }
                
                ui_audio_level = 0.0f; 
                ui_output_level = 0.0f; 
                
                clearBuffersRequested = true;
                globalAudioResetRequested = false;
            }

            bool blockIsMuted = clearBuffersRequested;
            uint32_t start_cycles = xthal_get_ccount(); 

            if (blockIsMuted) {
                memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                ui_audio_level = 0.0f;
                ui_output_level = 0.0f;
            } else {
                
                // SAFE BUFFER LOCK: Non-blocking zero-tick lock prevents audio clicks during UI-triggered resets
                if (audioBufferMutex != NULL && xSemaphoreTake(audioBufferMutex, 0) == pdTRUE) {
                    float targetWindow = LATENCY_WINDOWS[latencyMode];
                    
                    if (currentWindowSize != targetWindow) { 
                        currentWindowSize = targetWindow; 
                        uint32_t halfWindowFixed = ((uint32_t)targetWindow / 2) << 16;
                        
                        tap_w1_1 = 0; tap_w1_2 = halfWindowFixed; 
                        tap_w2_1 = 0; tap_w2_2 = halfWindowFixed;
                        tap_w3_1 = 0; tap_w3_2 = halfWindowFixed; 
                        tap_w4_1 = 0; tap_w4_2 = halfWindowFixed; 
                        tap_w5_1 = 0; tap_w5_2 = halfWindowFixed;
                    }
                    
                    uint32_t hannIntMult = (1024U << 16) / (uint32_t)currentWindowSize; 
                    uint32_t windowMask = (uint32_t)currentWindowSize - 1;
                    
                    float chorusPhaseIncr = 1536.0f / (float)currentSampleRate; 
                    float feedbackPhaseIncr = 5120.0f / (float)currentSampleRate;
                    
                    bool frzActive = ((activeEffectMode == 1 && isWhammyActive) || isFrozen); 
                    
                    if (frzActive && !wasFrozen) { 
                        freezePlayCounterVar = 0; 
                        
                        int bestStart = freezeWriteIdxVar;
                        for (int s = 0; s < 4000; s++) {
                            int idx = (freezeWriteIdxVar - s + freezeLength) % freezeLength;
                            int prev = (idx - 1 + freezeLength) % freezeLength;
                            if (freezeBuffer[idx] >= 0.0f && freezeBuffer[prev] < 0.0f) {
                                bestStart = idx;
                                break;
                            }
                        }
                        freezeStartIdxVar = bestStart;
                        
                        activeFreezeLength = freezeLength;
                        int searchEnd = (bestStart - 1 + freezeLength) % freezeLength;
                        for (int s = 0; s < 4000; s++) {
                            int idx = (searchEnd - s + freezeLength) % freezeLength;
                            int prev = (idx - 1 + freezeLength) % freezeLength;
                            if (freezeBuffer[idx] >= 0.0f && freezeBuffer[prev] < 0.0f) {
                                activeFreezeLength = freezeLength - s;
                                break;
                            }
                        }
                        
                        if (activeFreezeLength < freezeLength / 2) {
                            activeFreezeLength = freezeLength;
                        }
                    } 
                    
                    if (!frzActive && wasFrozen) {
                        apfNeedsClear = true;
                    }
                    wasFrozen = frzActive;
                    
                    float activeInvFreqLength = 1.0f / (float)activeFreezeLength;
                    
                    bool synthActive = ((activeEffectMode == 5 && isWhammyActive) || isSynthMode);
                    bool padActive = ((activeEffectMode == 6 && isWhammyActive) || isPadMode);
                    bool harmActive = ((activeEffectMode == 3 && isWhammyActive) || isHarmonizerMode);
                    bool swellActive = ((activeEffectMode == 8 && isWhammyActive) || isSwellMode);
                    bool chorusActive = ((activeEffectMode == 7 && isWhammyActive) || isChorusMode);
                    bool feedbackActive = ((activeEffectMode == 2 && isWhammyActive) || isFeedbackActive);
                    bool vibratoActive = ((activeEffectMode == 9 && isWhammyActive) || isVibratoMode);
                    bool capoActive = ((activeEffectMode == 4 && isWhammyActive) || isCapoMode);
                    
                    float peakInputVal = 0.0f; 
                    float peakOutputVal = 0.0f;
                    
                    // Buffer volatile RAM variables locally so the compiler can vectorize DSP loops
                    float localSwellGain = swellGain;
                    float localVolGain = volumePedalGain;
                    float localFrzRamp = freezeRamp;
                    float localFbRamp = feedbackRamp;

                    #pragma GCC ivdep
                    for (int i = 0; i < framesRead; i++) { 
                        // FIX: Bit-Mask the bottom 8 bits to eliminate floating ADC static noise
                        int32_t clean_sample = i2s_in_block[i * 2] & 0xFFFFFF00;
                        input_block[i] = ((float)clean_sample * normFactor); 
                    }
                    
                    dsps_biquad_f32(input_block, dc_block, framesRead, dc_coeffs, dc_state);
                    
                    float currentPitch = pitchShiftFactor; 
                    float harmPitch = currentPitch * globalHarmRatio;
                    float choPitch  = currentPitch * globalChorusRatio;
                    float fbPitch   = currentPitch * globalFbRatio;
                    
                    for (int i = 0; i < framesRead; i++) {
                        float inSample = dc_block[i]; 
                        inputEnvelope = inputEnvelope * 0.99f + fabsf(inSample) * 0.01f + DC_OFFSET;
                        
                        if (swellActive) {
                            if (inputEnvelope > 0.015f) { localSwellGain = fminf(1.0f, localSwellGain + 0.00002f); } 
                            else { localSwellGain = fmaxf(0.0f, localSwellGain - 0.00005f); }
                        } else { localSwellGain = 1.0f; }
                        
                        float procSample = inSample;
                        
                        if (synthActive) { 
                            if (inputEnvelope > 0.005f) { synthEnv = fminf(1.0f, synthEnv + 0.1f); } 
                            else { synthEnv = fmaxf(0.0f, synthEnv - 0.005f); }
                            
                            int waveIdx = constrain((int)((procSample + 1.0f) * 1023.5f), 0, WAVE_LUT_SIZE - 1);
                            procSample = synthLUT[waveIdx]; 
                            
                            synthFilter = synthFilter + (0.3f + 0.8f * synthEnv) * (procSample - synthFilter) + DC_OFFSET; 
                            procSample = synthFilter * 0.1f;
                        } 
                        
                        if (padActive) { 
                            if (inputEnvelope > 0.005f) { padEnv = fminf(1.0f, padEnv + 0.00002f); } 
                            else { padEnv = fmaxf(0.0f, padEnv - 0.000005f); }
                            procSample *= padEnv; 
                        }
                        
                        if (!frzActive) { 
                            freezeBuffer[freezeWriteIdxVar] = procSample;
                            freezeWriteIdxVar++;
                            if (freezeWriteIdxVar >= freezeLength) freezeWriteIdxVar = 0;
                        }
                        
                        if (localFrzRamp > 0.0f || frzActive) {
                            if (frzActive) { localFrzRamp = fminf(1.0f, localFrzRamp + 0.0002f); } 
                            else { localFrzRamp = fmaxf(0.0f, localFrzRamp - 0.00005f); }
                        }
                        
                        float fzOut = 0.0f;
                        
                        if (localFrzRamp > 0.0f) { 
                            float phaseRead = (float)freezePlayCounterVar * activeInvFreqLength; 
                            float phase2 = (phaseRead + 0.5f); 
                            if (phase2 >= 1.0f) { phase2 -= 1.0f; }
                            
                            int sum1 = freezeStartIdxVar + freezePlayCounterVar;
                            int idx1 = (sum1 >= freezeLength) ? (sum1 - freezeLength) : sum1;
                            
                            int counter2 = freezePlayCounterVar + (activeFreezeLength / 2);
                            if (counter2 >= activeFreezeLength) counter2 -= activeFreezeLength;
                            
                            int sum2 = freezeStartIdxVar + counter2;
                            int idx2 = (sum2 >= freezeLength) ? (sum2 - freezeLength) : sum2;
                            
                            float rFrz = (freezeBuffer[idx1] * hannLUT[(int)(phaseRead * 1023.0f)]) + 
                                        (freezeBuffer[idx2] * hannLUT[(int)(phase2 * 1023.0f)]);
                            
                            float d1 = apf1Buffer[apf1Idx]; 
                            float next_apf1 = rFrz + 0.6f * d1 + DC_OFFSET; 
                            float a1 = -0.6f * rFrz + d1; 
                            apf1Buffer[apf1Idx] = next_apf1;
                            apf1Idx++; if (apf1Idx >= 1009) { apf1Idx = 0; }
                            
                            float d2 = apf2Buffer[apf2Idx]; 
                            float next_apf2 = a1 + 0.6f * d2 + DC_OFFSET; 
                            float a2 = -0.6f * a1 + d2; 
                            apf2Buffer[apf2Idx] = next_apf2; 
                            apf2Idx++; if (apf2Idx >= 863) { apf2Idx = 0; }
                            
                            fzOut = a2 * localFrzRamp; 
                            freezePlayCounterVar++; 
                            if (freezePlayCounterVar >= activeFreezeLength) { freezePlayCounterVar = 0; }
                        } else if (apfNeedsClear) {
                            memset(apf1Buffer, 0, sizeof(apf1Buffer));
                            memset(apf2Buffer, 0, sizeof(apf2Buffer));
                            apfNeedsClear = false;
                        }
                        
                        float delayIn = (frzActive && localFrzRamp > 0.0f) ? fzOut : procSample; 
                        float boundedDelayIn = constrain(delayIn, -1.0f, 1.0f);
                        
                        int localWriteIdx = writeIndex; 
                        delayBuffer[localWriteIdx] = boundedDelayIn;
                        
                        float spd1 = currentPitch;
                        
                        if (vibratoActive) {
                            vibratoLfoPhase += globalVibratoPhaseInc; 
                            if (vibratoLfoPhase >= LFO_LUT_SIZE) { vibratoLfoPhase -= LFO_LUT_SIZE; }
                            spd1 *= lfoLUT[(int)vibratoLfoPhase];
                        }
                        
                        float spd2 = harmPitch; 
                        float spd3 = choPitch;
                        
                        if (chorusActive) { 
                            chorusLfoPhase += chorusPhaseIncr; 
                            if (chorusLfoPhase >= LFO_LUT_SIZE) { chorusLfoPhase -= LFO_LUT_SIZE; }
                            spd3 *= lfoLUT[(int)chorusLfoPhase]; 
                        }
                        
                        float spd4 = 1.0f; 
                        float spd5 = 1.0f;
                        
                        float w4 = 0.0f;
                        float w5 = 0.0f;
                        float fbOutNode = 0.0f;
                        
                        if (feedbackActive || localFbRamp > 0.0f) { 
                            feedbackLfoPhase += feedbackPhaseIncr; 
                            if (feedbackLfoPhase >= LFO_LUT_SIZE) { feedbackLfoPhase -= LFO_LUT_SIZE; }
                            float lfoVal = lfoLUT[(int)feedbackLfoPhase]; 
                            spd4 = lfoVal; 
                            spd5 = fbPitch * lfoVal; 
                            
                            w4 = processTap(tap_w4_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + 
                                processTap(tap_w4_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                            w5 = processTap(tap_w5_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + 
                                processTap(tap_w5_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                                
                            if (feedbackActive) {
                                if (inputEnvelope > 0.005f) { localFbRamp = fminf(1.0f, localFbRamp + 0.000011f); } 
                                else { localFbRamp = fmaxf(0.0f, localFbRamp - 0.005f); }
                            } else { localFbRamp = fmaxf(0.0f, localFbRamp - 0.0001f); }
                            
                            float mixV = fmaxf(0.0f, fminf((localFbRamp - 0.1f) * 2.0f, 1.0f));
                            float feedInput = (frzActive && localFrzRamp > 0.0f) ? fzOut : (w4 * (1.0f - mixV)) + (w5 * mixV);
                            
                            fbHpfState += 0.05f * (feedInput - fbHpfState) + DC_OFFSET; 
                            float gainDrive = constrain((feedInput - fbHpfState) * 30.0f, -1.0f, 1.0f); 
                            
                            feedbackFilterVar = feedbackFilterVar * 0.9f + gainDrive * 0.1f + DC_OFFSET;
                            float satFb = feedbackFilterVar * (localFbRamp * localFbRamp * localFbRamp) * 0.85f; 
                            
                            fbDelayBuffer[fbDelayWriteIdx] = satFb;
                            
                            int delaySamples = (int)(currentSampleRate * 0.02f);
                            int fbReadIdx = (fbDelayWriteIdx - delaySamples + FB_BUFFER_SIZE) & FB_BUFFER_MASK;
                            fbOutNode = fbDelayBuffer[fbReadIdx];
                            
                            fbDelayWriteIdx = (fbDelayWriteIdx + 1) & FB_BUFFER_MASK;
                        }
                        
                        float w1 = processTap(tap_w1_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + 
                                    processTap(tap_w1_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                        w1_block[i] = w1; 
                        
                        float w2 = 0.0f;
                        if (harmActive) {
                            w2 = processTap(tap_w2_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + 
                                processTap(tap_w2_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                        }
                        w2_block[i] = w2; 
                        
                        float w3 = 0.0f;
                        if (chorusActive) {
                            w3 = processTap(tap_w3_1, delayBuffer, localWriteIdx, windowMask, hannIntMult) + 
                                processTap(tap_w3_2, delayBuffer, localWriteIdx, windowMask, hannIntMult);
                        }
                        w3_block[i] = w3; 
                        
                        w4_block[i] = w4; 
                        w5_block[i] = w5; 
                        
                        int32_t step1 = (int32_t)((1.0f - spd1) * 65536.0f); tap_w1_1 += step1; tap_w1_2 += step1; 
                        int32_t step2 = (int32_t)((1.0f - spd2) * 65536.0f); tap_w2_1 += step2; tap_w2_2 += step2; 
                        int32_t step3 = (int32_t)((1.0f - spd3) * 65536.0f); tap_w3_1 += step3; tap_w3_2 += step3; 
                        int32_t step4 = (int32_t)((1.0f - spd4) * 65536.0f); tap_w4_1 += step4; tap_w4_2 += step4; 
                        int32_t step5 = (int32_t)((1.0f - spd5) * 65536.0f); tap_w5_1 += step5; tap_w5_2 += step5; 
                        
                        writeIndex = (writeIndex + 1) & BUFFER_MASK;
                        
                        if (padActive) { padFilter = padFilter * 0.95f + w1 * 0.05f + DC_OFFSET; } 
                        else { padFilter = padFilter * 0.95f + DC_OFFSET; }
                        
                        dry_block[i] = inSample;
                        fz_block[i] = fzOut;
                        pad_block[i] = padFilter;
                        fbOut_block[i] = fbOutNode;
                    }
                    
                    swellGain = localSwellGain;
                    freezeRamp = localFrzRamp;
                    feedbackRamp = localFbRamp;
                    
                    bool activeGroup = isWhammyActive || harmActive || chorusActive || feedbackActive || synthActive || padActive || frzActive || vibratoActive || capoActive;
                    bool dryGroup = chorusActive || padActive || frzActive || feedbackActive || (localFrzRamp > 0.0f) || (localFbRamp > 0.0f);
                    bool repeatGroup = capoActive || synthActive || vibratoActive || padActive || harmActive;
                    bool padIsAudible = padActive || (padFilter > 0.001f);
                    
                    if (!activeGroup && localFrzRamp <= 0.0f && localFbRamp <= 0.0f && !padIsAudible) {
                        memcpy(mix_block, dry_block, framesRead * sizeof(float));
                    } else {
                        float g_base = 0.0f;
                        if (dryGroup) {
                            if (!repeatGroup) g_base = 0.4f;
                        } else if (harmActive) {
                            g_base = 0.5f;
                        } else {
                            g_base = 1.0f;
                        }
                        
                        float g_w2 = harmActive ? 0.5f : 0.0f;
                        float g_w3 = chorusActive ? 0.4f : 0.0f;
                        float g_pad = padIsAudible ? 1.5f : 0.0f;
                        float g_frz = (!frzActive && localFrzRamp > 0.0f) ? 0.5f : 0.0f;
                        float g_fb = (feedbackActive || localFbRamp > 0.0f) ? 0.6f : 0.0f;
                        
                        float g_whammy = isWhammyActive ? 1.0f : 0.0f;
                        float g_dry = isWhammyActive ? 0.0f : 1.0f;
                        float* __restrict p_mix = mix_block;
                        const float* __restrict p_dry = dry_block;
                        const float* __restrict p_w1 = w1_block;
                        const float* __restrict p_w2 = w2_block;
                        const float* __restrict p_w3 = w3_block;
                        const float* __restrict p_pad = pad_block;
                        const float* __restrict p_fz = fz_block;
                        const float* __restrict p_fb = fbOut_block;
                        #pragma GCC ivdep
                        for (int i = 0; i < framesRead; i++) {
                            float baseSignal = (p_w1[i] * g_whammy) + (p_dry[i] * g_dry);
                            
                            float sMix = DC_OFFSET;
                            sMix += baseSignal * g_base;
                            sMix += p_w2[i] * g_w2;
                            sMix += p_w3[i] * g_w3;
                            sMix += p_pad[i] * g_pad;
                            sMix += p_fz[i] * g_frz;
                            sMix += p_fb[i] * g_fb;
                            
                            sMix = fmaxf(-1.8f, fminf(sMix, 1.8f));
                            p_mix[i] = sMix * (1.0f - (0.1f * sMix * sMix));
                        }
                    }
                    
                    #pragma GCC ivdep
                    for (int i = 0; i < framesRead; i++) {
                        // FIX: Smooth the discrete CC MIDI volume changes seamlessly per-sample
                        smoothedVolGain = smoothedVolGain * 0.99f + localVolGain * 0.01f;

                        float rawOut = mix_block[i] * localSwellGain * smoothedVolGain; 
                        
                        if (fabsf(dc_block[i]) > peakInputVal) peakInputVal = fabsf(dc_block[i]); 
                        if (fabsf(rawOut) > peakOutputVal) peakOutputVal = fabsf(rawOut); 
                        
                        float currentMasterScale = 2147483520.0f * localSwellGain * smoothedVolGain;
                        float scaledOut = mix_block[i] * currentMasterScale;
                        int32_t finalOut = (int32_t)fmaxf(-2147483520.0f, fminf(scaledOut, 2147483520.0f));
                        
                        // FIX: Bit-Mask out the bottom 8 bits for perfect 24-bit alignment to the DAC
                        finalOut &= 0xFFFFFF00;
                        i2s_out_block[i * 2] = finalOut; 
                        i2s_out_block[i * 2 + 1] = finalOut;
                    }

                    if (peakInputVal > ui_audio_level) { ui_audio_level = peakInputVal; } 
                    else { ui_audio_level *= 0.998f; if (ui_audio_level < 1e-5f) ui_audio_level = 0.0f; }
                    
                    if (peakOutputVal > ui_output_level) { ui_output_level = peakOutputVal; } 
                    else { ui_output_level *= 0.998f; if (ui_output_level < 1e-5f) ui_output_level = 0.0f; }
                    
                    xSemaphoreGive(audioBufferMutex);
                } else {
                    // MUTEX FALLBACK: If loop() is busy clearing buffers via CC3, output silence gracefully
                    memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                    ui_audio_level = 0.0f;
                    ui_output_level = 0.0f;
                }
            } // --- END OF !blockIsMuted DSP BYPASS ---
            
            uint32_t end_timer = xthal_get_ccount(); 
            float max_cycles = (240000000.0f / (float)currentSampleRate) * (float)framesRead;
            float currentLoadPercentage = ((float)(end_timer - start_cycles) / max_cycles) * 100.0f;
            core1_load = core1_load * 0.95f + fminf(100.0f, currentLoadPercentage) * 0.05f; 
            
            size_t bytesWrittenCount; 
            i2s_channel_write(tx_chan, i2s_out_block, framesRead * 8, &bytesWrittenCount, portMAX_DELAY);
        }
    }
}

void MidiTask(void * pvParameters) {
    static analog_t lastMidiA = 8192;
    static analog_t lastMidiB = 8192;
    static analog_t lastMidiC = 8192; 
    
    static bool lastBtState = false; 
    static uint8_t lastVolumeCC = 127;
    
    static int stableRawA = -1;
    static int stableRawB = -1;
    static int stableRawC = -1;
    
    static DebouncedButton carouselBtn(CAROUSEL_BUTTON_PIN); 
    carouselBtn.state = digitalRead(CAROUSEL_BUTTON_PIN); 
    carouselBtn.lastReading = carouselBtn.state; 
    
    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP);
    
    for (;;) {
        static unsigned long lastBatteryTime = 0;
        if (millis() - lastBatteryTime > 5000) {
            lastBatteryTime = millis();
            int rawBat = analogRead(BATTERY_PIN);
            float voltage = ((float)rawBat / 4095.0f) * 3.3f * 2.0f; 
            
            bool charging = (voltage > 4.25f);
            int newPercent = getBatteryPercentage(voltage);
            
            if (newPercent != currentBatteryPercent || charging != isBatteryCharging) {
                currentBatteryPercent = newPercent;
                isBatteryCharging = charging;
                forceUIUpdate = true;
            }
        }

        Control_Surface.loop(); 
        
        bool currentBtState = btmidi.isConnected();
        
        if (currentBtState != lastBtState) { 
            lastBtState = currentBtState; 
            forceUIUpdate = true; 
            if (isScreenOff) { turnScreenOn(); }
            lastActivityTime = millis(); 
        }
        
        if (currentBtState) { lastActivityTime = millis(); }
        
        if (!currentBtState && (millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT)) { goToLightSleep(); }
        if (!isScreenOff && (millis() - lastScreenActivityTime > SCREEN_OFF_TIMEOUT)) { turnScreenOff(); }
        
        if (carouselBtn.update(100)) {
            if (carouselBtn.state == LOW) { 
                carouselBtn.pressedTime = millis(); 
                carouselBtn.isActive = true; 
            } else if (carouselBtn.state == HIGH && carouselBtn.isActive) {
                carouselBtn.isActive = false; 
                if (millis() - carouselBtn.pressedTime < 400) { 
                    activeEffectMode = (activeEffectMode + 1) % 10; 
                    chorusLfoPhase = 0.0f; 
                    feedbackLfoPhase = 0.0f; 
                    vibratoLfoPhase = 0.0f; 
                    swellGain = 0.0f; 
                    isWhammyActive = true; 
                    lutNeedsUpdate = true; 
                    
                    settingsNeedSaving = true;
                    lastParameterChangeTime = millis();
                } 
                forceUIUpdate = true;
            }
        }
        
        filterPB.update(); 
        filterPB2.update(); 
        filterPB3.update();
        
        if (digitalRead(BOOT_SENSE_PIN) == HIGH) {
            analog_t rawA = filterPB.getValue();
            analog_t rawB = filterPB2.getValue();
            analog_t rawC = filterPB3.getValue();
            
            if (stableRawA < 0) { stableRawA = rawA; }
            if (abs((int)rawA - stableRawA) > 6) { stableRawA = rawA; }
            
            if (stableRawB < 0) { stableRawB = rawB; }
            if (abs((int)rawB - stableRawB) > 6) { stableRawB = rawB; }
            
            if (stableRawC < 0) { stableRawC = rawC; }
            if (abs((int)rawC - stableRawC) > 16) { stableRawC = rawC; }

            static bool unpluggedA = false;
            static bool unpluggedB = false;
            static int previousRawA = 2048;
            static int previousRawB = 2048;

            if ((stableRawA - previousRawA) > 1000 && stableRawA > 4050) { unpluggedA = true; } 
            else if (stableRawA < 3900) { unpluggedA = false; }

            if ((stableRawB - previousRawB) > 1000 && stableRawB > 4050) { unpluggedB = true; } 
            else if (stableRawB < 3900) { unpluggedB = false; }

            previousRawA = stableRawA;
            previousRawB = stableRawB;

            if (!unpluggedA) {
                if (stableRawA < PB1_raw_min) { PB1_raw_min = stableRawA; }
                if (stableRawA > PB1_raw_max) { PB1_raw_max = stableRawA; }
            }

            if (!unpluggedB) {
                if (stableRawB < PB2_raw_min) { PB2_raw_min = stableRawB; }
                if (stableRawB > PB2_raw_max) { PB2_raw_max = stableRawB; }
            }
            
            if (stableRawC < PB3_raw_min) { PB3_raw_min = stableRawC; }
            if (stableRawC > PB3_raw_max && stableRawC <= 4095) { PB3_raw_max = stableRawC; }
            
            analog_t calA = map_raw_deadzone(stableRawA, PB1_raw_center, PB1_raw_min, PB1_raw_max, deadzone_size);
            
            // NEW: Route PB2 dynamically based on the CC toggle
            analog_t calB;
            if (isPB2LinearMode) {
                // PB2 W (Wiper) - Uses PB1 Spring-Loaded Logic
                calB = map_raw_deadzone(stableRawB, PB2_raw_center, PB2_raw_min, PB2_raw_max, deadzone_size);
            } else {
                // PB2 H (Hall Effect) - Uses PB1 Spring-Loaded Logic
                calB = map_raw_deadzone(stableRawB, PB2_raw_center, PB2_raw_min, PB2_raw_max, deadzone_size);
            }
            
            analog_t calC = map_raw_expression(stableRawC, PB3_raw_min, PB3_raw_max, INVERT_PB3);
            
            if (unpluggedA) { calA = 8192; }
            if (unpluggedB) { calB = 8192; }
            
            bool moveA = (abs((int)calA - (int)lastMidiA) > 12) || ((calA == 8192 || calA == 0 || calA == 16383) && calA != lastMidiA);
            bool moveB = (abs((int)calB - (int)lastMidiB) > 12) || ((calB == 8192 || calB == 0 || calB == 16383) && calB != lastMidiB);
            bool moveC = (abs((int)calC - (int)lastMidiC) >= 128) || ((calC == 0 || calC == 16383) && calC != lastMidiC);
            
            if (moveA || moveB || moveC) {
                if (isScreenOff) { turnScreenOn(); }
                lastScreenActivityTime = millis();
                
                if (moveA) { 
                    Control_Surface.sendPitchBend(Channel_1, calA); 
                    lastMidiA = calA; 
                    currentPB1 = calA; 
                }
                
                if (moveB) { 
                    Control_Surface.sendPitchBend(Channel_2, calB); 
                    lastMidiB = calB; 
                    currentPB2 = calB; 
                }
                
                if (moveC) {
                    if (!isVolumeMode) {
                        Control_Surface.sendPitchBend(Channel_3, calC);
                    }
                    lastMidiC = calC; 
                    currentPB3 = calC; 
                }

                bool pitchChanged = false;
                if (moveA) { lastActivePedal = calA; pitchChanged = true; }
                if (moveB) { lastActivePedal = calB; pitchChanged = true; }
                if (moveC && !isVolumeMode) { lastActivePedal = calC; pitchChanged = true; }

                if (pitchChanged && !lutNeedsUpdate) {
                    pitchShiftFactor = pitchShiftLUT[constrain(lastActivePedal, 0, 16383)];
                }

                if (moveC && isVolumeMode) {
                    uint8_t vCC = map(calC, 0, 16383, 0, 127); 
                    if (vCC != lastVolumeCC) { 
                        Control_Surface.sendControlChange({19, Channel_1}, vCC); 
                        lastVolumeCC = vCC; 
                    } 
                    volumePedalGain = (float)calC / 16383.0f; 
                }
                
                forceUIUpdate = true;
            }
        } else {
            if (lastMidiA != 8192 || lastMidiB != 8192 || lastMidiC != 8192) {
                Control_Surface.sendPitchBend(Channel_1, 8192);
                Control_Surface.sendPitchBend(Channel_2, 8192);
                Control_Surface.sendPitchBend(Channel_3, 8192);
                
                lastMidiA = 8192; 
                lastMidiB = 8192; 
                lastMidiC = 8192; 
                
                currentPB1 = 8192; 
                currentPB2 = 8192; 
                currentPB3 = 8192; 
                
                lastActivePedal = 8192;
                
                if (!lutNeedsUpdate) {
                    pitchShiftFactor = pitchShiftLUT[8192]; 
                }
                
                if (isVolumeMode) { 
                    volumePedalGain = 8192.0f / 16383.0f; 
                }
                
                forceUIUpdate = true;
            }
            vTaskDelay(pdMS_TO_TICKS(250)); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// --- UPDATED INTERNAL TOGGLE LOGIC ---
bool channelMessageCallback(ChannelMessage cm) {
    if (cm.header == 0xB0) {
        
        // PB2 HARDWARE MODE TOGGLE (Hall vs Wiper)
        if (cm.data1 == 5 && cm.data2 >= 64) {
            isPB2LinearMode = !isPB2LinearMode;
            
            // Instantly take a new reading to establish a fresh center point
            int newCenter = filterPB2.getValue();
            if (newCenter < 4000 && newCenter > 100) {
                PB2_raw_center = newCenter;
            } else {
                PB2_raw_center = 2048; // Fallback to safe digital center
            }
            
            // FIX: Safely set initial tight bounds around the new center. 
            // This allows the auto-calibration in MidiTask to correctly expand outward.
            PB2_raw_min = PB2_raw_center - 200;
            PB2_raw_max = PB2_raw_center + 200;
            
            forceUIUpdate = true;
            
            settingsNeedSaving = true;
            lastParameterChangeTime = millis();
        }
        // VOLUME MODE TOGGLE
        else if (cm.data1 == 6 && cm.data2 >= 64) { 
            isVolumeMode = !isVolumeMode; 
            if (!isVolumeMode) { volumePedalGain = 1.0f; } 
            forceUIUpdate = true; 
        }
        // EFFECT MENU DOWN
        else if (cm.data1 == 0 && cm.data2 >= 64) { 
            if (activeEffectMode == 0) { activeEffectMode = 9; } 
            else { activeEffectMode = activeEffectMode - 1; }
            lutNeedsUpdate = true; forceUIUpdate = true; 
            
            settingsNeedSaving = true;
            lastParameterChangeTime = millis();
        }
        // EFFECT MENU UP
        else if (cm.data1 == 1 && cm.data2 >= 64) { 
            activeEffectMode = (activeEffectMode + 1) % 10; 
            lutNeedsUpdate = true; forceUIUpdate = true; 
            
            settingsNeedSaving = true;
            lastParameterChangeTime = millis();
        }
        // LATENCY MENU
        else if (cm.data1 == 2 && cm.data2 >= 64) { 
            latencyMode = (latencyMode + 1) % 4; 
            forceUIUpdate = true; 
            
            settingsNeedSaving = true;
            lastParameterChangeTime = millis();
        }
        // GLOBAL BYPASS / SMART RESET TOGGLE
        else if (cm.data1 == 3 && cm.data2 >= 64) {
            globalAudioResetRequested = true; 
            
            bool anyEffectOn = isWhammyActive || isFrozen || isFeedbackActive || 
                               isHarmonizerMode || isCapoMode || isSynthMode || 
                               isPadMode || isChorusMode || isSwellMode || 
                               isVibratoMode || isVolumeMode;

            if (anyEffectOn) {
                // FIRST PRESS: Panic Mute & Reset Variables
                isWhammyActive = false; 
                isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false;
                isCapoMode = false; isSynthMode = false; isPadMode = false; 
                isChorusMode = false; isSwellMode = false; isVibratoMode = false; 
                isVolumeMode = false;
                volumePedalGain = 1.0f; 
                
                // Yanks the user back to the default Whammy screen
                activeEffectMode = 0; 
            } else {
                // SECOND PRESS: Because activeEffectMode is now 0, this purely turns on the Whammy
                isWhammyActive = true;
            }
            
            lutNeedsUpdate = true; forceUIUpdate = true;
        }
        // SAMPLE RATE TOGGLE
        else if (cm.data1 == 4 && cm.data2 >= 64) { toggleSampleRate(); }
        
        // EFFECT SPECIFIC TOGGLES
        else if (cm.data1 == 8 && cm.data2 >= 64) { 
            isFrozen = !isFrozen; 
            if (activeEffectMode == 1) { isWhammyActive = isFrozen; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 9 && cm.data2 >= 64) { 
            isFeedbackActive = !isFeedbackActive; 
            if (activeEffectMode == 2) { isWhammyActive = isFeedbackActive; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 10 && cm.data2 >= 64) { 
            isHarmonizerMode = !isHarmonizerMode; 
            if (activeEffectMode == 3) { isWhammyActive = isHarmonizerMode; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 12 && cm.data2 >= 64) { 
            isCapoMode = !isCapoMode; 
            if (activeEffectMode == 4) { isWhammyActive = isCapoMode; } 
            lutNeedsUpdate = true; forceUIUpdate = true; 
        }
        else if (cm.data1 == 13 && cm.data2 >= 64) { 
            isSynthMode = !isSynthMode; 
            if (activeEffectMode == 5) { isWhammyActive = isSynthMode; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 14 && cm.data2 >= 64) { 
            isPadMode = !isPadMode; 
            if (activeEffectMode == 6) { isWhammyActive = isPadMode; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 15 && cm.data2 >= 64) { 
            isChorusMode = !isChorusMode; 
            if (activeEffectMode == 7) { isWhammyActive = isChorusMode; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 16 && cm.data2 >= 64) { 
            isSwellMode = !isSwellMode; 
            if (activeEffectMode == 8) { isWhammyActive = isSwellMode; } 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 7 && cm.data2 >= 64) { 
            isVibratoMode = !isVibratoMode; 
            if (activeEffectMode == 9) { isWhammyActive = isVibratoMode; } 
            forceUIUpdate = true; 
        }
        // PARAMETER TWEAKS (Pitch intervals, etc)
        else if (cm.data1 == 18 || cm.data1 == 17) {
            float direction = (cm.data2 < 64) ? 1.0f : -1.0f;
            
            if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
                if (cm.data1 == 18) { effectMemory[0] = constrain(effectMemory[0] + direction, -24.0f, 24.0f); } 
                else { effectMemory[5] = constrain(effectMemory[5] + direction, -24.0f, 24.0f); }
            } else if (activeEffectMode == 4) {
                float change = (cm.data1 == 18) ? 1.0f : 0.01f;
                effectMemory[4] = constrain(effectMemory[4] + change * direction, -24.0f, 24.0f);
            } else if (activeEffectMode == 2) { 
                if (direction > 0) { feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5; } 
                else { feedbackIntervalIdx = (feedbackIntervalIdx + 4) % 5; }
            } else { 
                int memIndex = activeEffectMode; 
                if (memIndex == 5) { memIndex = 6; } 
                else if (memIndex == 6) { memIndex = 7; } 
                else if (memIndex == 7) { memIndex = 8; } 
                else if (memIndex == 9) { memIndex = 9; }
                effectMemory[memIndex] = constrain(effectMemory[memIndex] + direction, -24.0f, 24.0f); 
            }
            
            lutNeedsUpdate = true; forceUIUpdate = true;
            
            settingsNeedSaving = true;
            lastParameterChangeTime = millis();
        }
        else if (cm.data1 == 11) { 
            uint16_t mappedCC = map(cm.data2, 0, 127, 0, 16383); 
            currentCC11 = mappedCC; 
            currentPB3 = mappedCC; 
            
            if (isVolumeMode) {
                volumePedalGain = (float)mappedCC / 16383.0f;
                Control_Surface.sendControlChange({19, Channel_1}, cm.data2); 
            } else {
                if (!lutNeedsUpdate) {
                    pitchShiftFactor = pitchShiftLUT[mappedCC]; 
                }
            }
            forceUIUpdate = true; 
        }
    }
    return false;
}

void setup() {
    // FIX: Initialize Mutex Lock for Safe Audio Buffer Wipes
    audioBufferMutex = xSemaphoreCreateMutex();

    // Completely disable Wi-Fi radio to eliminate 2.4 GHz RF coexistence interrupts
    WiFi.mode(WIFI_OFF);

    // --- LOAD SAVED SETTINGS FROM FLASH ---
    preferences.begin("whammy_cfg", true); // true = Read Only mode
    
    activeEffectMode = preferences.getInt("activeMode", 0);
    latencyMode = preferences.getInt("latMode", 0);
    isPB2LinearMode = preferences.getBool("pb2Linear", false);
    currentSampleRate = preferences.getUInt("sampleRate", 48000);
    
    for(int i = 0; i < 10; i++) {
        char key[8];
        sprintf(key, "fxMem%d", i);
        effectMemory[i] = preferences.getFloat(key, effectMemory[i]); 
    }
    preferences.end();
    // --------------------------------------

    pinMode(CAROUSEL_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(BATTERY_PIN, INPUT);
    
    pinMode(38, OUTPUT); 
    digitalWrite(38, LOW); 
    
    pinMode(15, OUTPUT); 
    digitalWrite(15, HIGH);
    
    Serial.begin(115200); 
    tft.init(); 
    tft.setRotation(1); 
    
    spr.createSprite(tft.width(), tft.height()); 
    meterSpr.createSprite(6, 98);
    
    tft.fillScreen(TFT_BLACK); 
    tft.setTextDatum(MC_DATUM); 
    tft.setTextSize(3); 
    tft.setTextColor(TFT_WHITE, TFT_BLACK); 
    tft.drawString("BOOTING...", 160, 85);
    
    delay(120); 
    digitalWrite(38, HIGH); 
    btmidi.setName("Whammy_S3"); 
    
    pinMode(pinPB, INPUT_PULLUP); 
    pinMode(pinPB2, INPUT_PULLUP); 
    pinMode(pinPB3, INPUT_PULLUP); 
    
    delayBuffer = (float*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    fbDelayBuffer = (float*)heap_caps_aligned_alloc(16, FB_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    freezeBuffer = (float*)heap_caps_aligned_alloc(16, FREEZE_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    pitchShiftLUT = (float*)heap_caps_aligned_alloc(16, 16384 * sizeof(float), MALLOC_CAP_SPIRAM);
    pitchShiftLUT_temp = (float*)heap_caps_aligned_alloc(16, 16384 * sizeof(float), MALLOC_CAP_SPIRAM);
    
    if (delayBuffer == nullptr || fbDelayBuffer == nullptr || freezeBuffer == nullptr || pitchShiftLUT == nullptr || pitchShiftLUT_temp == nullptr) {
        tft.fillScreen(TFT_RED); 
        tft.setTextColor(TFT_WHITE, TFT_RED); 
        tft.drawString("MEMORY ERROR", 160, 85);
        while(1) { delay(100); }
    }
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float)); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(float)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(float));
    memset(pitchShiftLUT, 0, 16384 * sizeof(float));
    memset(pitchShiftLUT_temp, 0, 16384 * sizeof(float));
    memset(hannLUT, 0, 1024 * sizeof(float));           
    
    for (int i = 0; i < 1024; i++) { 
        hannLUT[i] = 0.5f * (1.0f - cosf(TWO_PI * ((float)i / 1023.0f))); 
        lfoLUT[i] = powf(2.0f, (15.0f * sinf(TWO_PI * ((float)i / 1024.0f))) / 1200.0f); 
    }
    
    for (int i = 0; i < 2048; i++) { synthLUT[i] = sinf((((float)i - 1024.0f) / 1024.0f) * 45.0f); }
    
    FilteredAnalog<>::setupADC(); 
    
    // FIX: Hardware Auto-Calibration Delay
    // Wait for internal capacitors to fully charge and mechanical springs to snap exactly to rest position
    delay(500);
    calibratePBs(); 
    
    updateLUT();
    
    Control_Surface >> pipes >> btmidi; 
    Control_Surface >> pipes >> usbmidi; 
    usbmidi >> pipes >> Control_Surface; 
    btmidi >> pipes >> Control_Surface;
    
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr); 
    Control_Surface.begin();

    // --- BOOST BLUETOOTH TX POWER TO +9dBm (MAXIMUM) ---
    // Safely applied AFTER the BLE stack is fully initialized by Control_Surface
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
    
    i2s_chan_config_t i2sConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); 
    i2sConfig.dma_desc_num = 8; 
    i2sConfig.dma_frame_num = HOP_SIZE; 
    i2sConfig.auto_clear = true;
    
    i2s_new_channel(&i2sConfig, &tx_chan, &rx_chan);
    
    i2s_std_config_t stdConfig = { 
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate), 
        // Retained the standard Philips format from our earlier fix
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg = { 
            .mclk = GPIO_NUM_43, 
            .bclk = GPIO_NUM_44, 
            .ws = GPIO_NUM_18, 
            .dout = GPIO_NUM_16, 
            .din = GPIO_NUM_17 
        } 
    };
    
    i2s_channel_init_std_mode(tx_chan, &stdConfig); 
    i2s_channel_init_std_mode(rx_chan, &stdConfig); 
    
    i2s_channel_enable(tx_chan); 
    i2s_channel_enable(rx_chan);
    
    xTaskCreatePinnedToCore(DisplayTask, "UI", 8192, NULL, 1, NULL, 0); 
    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);
}

void loop() {
    if (lutNeedsUpdate) {
        updateLUT();
        pitchShiftFactor = pitchShiftLUT[constrain(lastActivePedal, 0, 16383)]; 
        lutNeedsUpdate = false;
    }
    
    if (clearBuffersRequested) {
        // FIX: Mutex Lock ensures loop() doesn't wipe memory while AudioDSPTask is actively reading it
        if (audioBufferMutex != NULL && xSemaphoreTake(audioBufferMutex, portMAX_DELAY) == pdTRUE) {
            memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float));
            memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(float));
            memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(float));
            clearBuffersRequested = false;
            xSemaphoreGive(audioBufferMutex);
        }
    }
    
    // NEW: Safe Delayed Save Execution
    if (settingsNeedSaving && (millis() - lastParameterChangeTime > 3000)) {
        saveSettings();
        settingsNeedSaving = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
}