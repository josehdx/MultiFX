#pragma GCC optimize ("O3, tree-vectorize")
#include <Arduino.h>
#include <Control_Surface.h>
#include <TFT_eSPI.h>
#include <driver/i2s_std.h> 
#include "driver/gpio.h" 
#include "freertos/FreeRTOS.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "driver/rtc_io.h"
#include <math.h>

// --- BARE-METAL PRE-BOOT ASSASSIN ---
void __attribute__((constructor)) pre_boot_kill_switch() {
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_38, 0); 
    
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_15, 0);
    
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 0);
}

/// --- GLOBALS & I2S ---
i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;
#define SAMPLING_FREQUENCY 96000 
#define HOP_SIZE 64            

// --- TIME-DOMAIN DSP BUFFERS (INTERNAL/PSRAM) ---
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF 

float* delayBuffer = nullptr;
float* freezeBuffer = nullptr;
float* fbDelayBuffer = nullptr; 
int writeIndex = 0;
int fbDelayWriteIdx = 0;

// --- DSP LOOK-UP TABLES ---
#define HANN_LUT_SIZE 1024
#define LFO_LUT_SIZE 1024
#define WAVE_LUT_SIZE 2048

DRAM_ATTR float hannLUT[HANN_LUT_SIZE];
DRAM_ATTR float lfoLUT[LFO_LUT_SIZE];
DRAM_ATTR float synthLUT[WAVE_LUT_SIZE];
volatile float pitchShiftLUT[16384]; 

// Tap States 
float tap1 = 0.0f;
float tap2 = 256.0f; 
float tap1_2 = 0.0f;
float tap2_2 = 256.0f;
float currentWindowSize = 1024.0f; 

// --- FREEZE STATE & ALL-PASS FILTERS ---
const int freezeLength = 48000; 
bool wasFrozen = false;
volatile float freezeRamp = 0.0f;

float apf1Buffer[1009] = { 0.0f }; 
int apf1Idx = 0;
float apf2Buffer[863] = { 0.0f };  
int apf2Idx = 0;

// --- INTERVAL CYCLERS ---
const float intervalList[] = {-12.0f, -7.0f, -5.0f, -2.0f, 0.0f, 2.0f, 5.0f, 7.0f, 12.0f};
int currentIntervalIdx = 8; 

volatile int feedbackIntervalIdx = 0; 

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

volatile float chorusLfoPhase = 0.0f;
volatile float feedbackLfoPhase = 0.0f;
volatile float vibratoLfoPhase = 0.0f;
volatile float swellGain = 0.0f; 

// VOLUME EFFECT VARIABLES
volatile bool isVolumeMode = false; 
volatile float volumePedalGain = 1.0f; 

volatile float feedbackRamp = 0.0f;
float fbHpfState = 0.0f;
float feedbackFilter = 0.0f;
volatile int latencyMode = 1; 
const float LATENCY_WINDOWS[] = {512.0f, 1024.0f, 2048.0f, 4096.0f};

// --- POWER SAVING & UI GLOBALS ---
unsigned long lastActivityTime = 0;       
unsigned long lastScreenActivityTime = 0;
const unsigned long LIGHT_SLEEP_TIMEOUT = 600000; 
const unsigned long SCREEN_OFF_TIMEOUT = 1200000;  
bool isScreenOff = false;
volatile bool wakeupPending = false; 
volatile float core1_load = 0.0f; 

// Safe RTOS Handshake variables for going to sleep without queue corruption
volatile bool sleepRequested = false;
volatile bool isSleeping = false;

// --- PIN ASSIGNMENTS ---
pin_t pinPB = 1;     
pin_t pinPB2 = 2;    
const int BOOT_SENSE_PIN = 0; 
const int CAROUSEL_BUTTON_PIN = 14; 
const int FREEZE_BUTTON_PIN = 18;    
const int INTERVAL_BUTTON_PIN = 4; 
const int FEEDBACK_BUTTON_PIN = 13;  

uint16_t lastMidiSent = 8192;
volatile uint16_t currentPB1 = 8192;
volatile uint16_t currentPB2 = 8192;
volatile uint16_t currentCC11 = 0;
volatile float ui_audio_level = 0.0f; 
volatile float ui_output_level = 0.0f;

// --- INDEPENDENT DUAL CALIBRATION VARIABLES ---
double PBdeadzoneMultiplier = 14;
double PBdeadzoneMinimum = 950;
double PBdeadzoneMaximum = 1600;
analog_t PBminimumValue = 0;
analog_t PBmaximumValue = 16383;

analog_t PBcenter1 = 8192;
analog_t PBdeadzone1 = PBdeadzoneMinimum;
bool PBwasOffCenter1 = false;

analog_t PBcenter2 = 8192;
analog_t PBdeadzone2 = PBdeadzoneMinimum;
bool PBwasOffCenter2 = false;

FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB2 = pinPB2;
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;

// --- SLEEP FUNCTIONS ---
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
    
    // Handshake 1: Request sleep, but timeout after 100ms if DSP gets stuck
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
    rtc_gpio_pulldown_dis(GPIO_NUM_14);
    esp_sleep_enable_ext1_wakeup(1ULL << 14, ESP_EXT1_WAKEUP_ANY_LOW);
    
    delay(50);
    esp_light_sleep_start();
    
    rtc_gpio_deinit(GPIO_NUM_14);
    pinMode(CAROUSEL_BUTTON_PIN, INPUT_PULLUP);
    
    i2s_channel_enable(tx_chan);
    i2s_channel_enable(rx_chan);
    
    // Handshake 2: Request wake, but timeout after 100ms if DSP gets stuck
    sleepRequested = false;
    timeoutCounter = 0;
    while (isSleeping && timeoutCounter < 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeoutCounter++;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200)); 
    turnScreenOn();
    
    lastActivityTime = millis();
    lastScreenActivityTime = millis();
}

// --- CUBIC HERMITE INTERPOLATION ENGINE (Zero Aliasing) ---
inline float IRAM_ATTR getHermiteSample(float tapPos, float* buffer, int writeIdx) {
    int iTap = (int)tapPos;
    float frac = tapPos - iTap;
    
    int idx0 = (writeIdx - iTap + 1 + MAX_BUFFER_SIZE) & BUFFER_MASK;
    int idx1 = (writeIdx - iTap + MAX_BUFFER_SIZE) & BUFFER_MASK;
    int idx2 = (writeIdx - iTap - 1 + MAX_BUFFER_SIZE) & BUFFER_MASK;
    int idx3 = (writeIdx - iTap - 2 + MAX_BUFFER_SIZE) & BUFFER_MASK;
    
    float y0 = buffer[idx0];
    float y1 = buffer[idx1];
    float y2 = buffer[idx2];
    float y3 = buffer[idx3];
    
    float c0 = y1;
    float c1 = 0.5f * (y2 - y0);
    float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    float c3 = -0.5f * y0 + 1.5f * y1 - 1.5f * y2 + 0.5f * y3;
    
    return ((c3 * frac + c2) * frac + c1) * frac + c0;
}

analog_t map_PB(analog_t raw, analog_t center, analog_t deadzone, bool &offCenterFlag) {
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    if (raw <= PBminimumValue + 150) { 
        offCenterFlag = true; 
        return 0; 
    }
    
    if (raw >= PBmaximumValue - 150) { 
        offCenterFlag = true; 
        return 16383; 
    }
    
    if (raw <= center - deadzone) { 
        offCenterFlag = true; 
        return map(raw, PBminimumValue, center - deadzone, 0, 8191); 
    }
    else if (raw >= center + deadzone) { 
        offCenterFlag = true; 
        return map(raw, center + deadzone, PBmaximumValue, 8191, 16383); 
    }
    else {
        return 8192; 
    }
}

void calibrateCenterAndDeadzone() {
    for(int i = 0; i < 50; i++) { 
        filterPB.update(); 
        filterPB2.update(); 
        delay(1); 
    }
    
    int iNumberOfSamples = 750;
    analog_t calibPBLow1 = 16383;
    analog_t calibPBHigh1 = 0; 
    long lSampleSumPB1 = 0;
    
    analog_t calibPBLow2 = 16383;
    analog_t calibPBHigh2 = 0; 
    long lSampleSumPB2 = 0;
  
    for (int iSample = 1; iSample <= iNumberOfSamples; iSample++) {
        filterPB.update(); 
        filterPB2.update();
        
        analog_t raw12_1 = filterPB.getValue();
        analog_t calibPB1 = map(raw12_1, 0, 4095, 0, 16383);
        lSampleSumPB1 += calibPB1;
        
        if (calibPB1 < calibPBLow1) {
            calibPBLow1 = calibPB1; 
        }
        if (calibPB1 > calibPBHigh1) {
            calibPBHigh1 = calibPB1; 
        }

        analog_t raw12_2 = filterPB2.getValue();
        analog_t calibPB2 = map(raw12_2, 0, 4095, 0, 16383); 
        lSampleSumPB2 += calibPB2;
        
        if (calibPB2 < calibPBLow2) {
            calibPBLow2 = calibPB2; 
        }
        if (calibPB2 > calibPBHigh2) {
            calibPBHigh2 = calibPB2; 
        }
        
        delay(1);
    }
    
    PBcenter1 = lSampleSumPB1 / iNumberOfSamples;
    PBdeadzone1 = (analog_t)constrain(((calibPBHigh1 - calibPBLow1) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
    
    PBcenter2 = lSampleSumPB2 / iNumberOfSamples;
    PBdeadzone2 = (analog_t)constrain(((calibPBHigh2 - calibPBLow2) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
}

void updateLUT() {
    bool isStaticIntervalMode = ((activeEffectMode == 2 && isWhammyActive) || isFeedbackActive);
    
    if (isStaticIntervalMode) {
        float intervals[5] = { 0.0f, 12.0f, 19.0f, 24.0f, 28.0f };
        float staticTotalShift = intervals[feedbackIntervalIdx % 5];
        float staticRatio = powf(2.0f, staticTotalShift / 12.0f);
        
        for (int i = 0; i < 16384; i++) {
            pitchShiftLUT[i] = staticRatio;
        }
    } else {
        float basePitch = 0.0f;
        
        if (isCapoMode || (activeEffectMode == 4 && isWhammyActive)) {
            basePitch += effectMemory[4]; 
        } 
        
        if (isSynthMode || (activeEffectMode == 5 && isWhammyActive)) {
            basePitch += effectMemory[6]; 
        } else if (isPadMode || (activeEffectMode == 6 && isWhammyActive)) {
            basePitch += effectMemory[7]; 
        } else if (isChorusMode || (activeEffectMode == 7 && isWhammyActive)) {
            basePitch += effectMemory[8]; 
        } else if (isVibratoMode || (activeEffectMode == 9 && isWhammyActive)) {
            basePitch += effectMemory[9];
        }
        
        float toeBend = effectMemory[0];
        float heelBend = effectMemory[5];
        
        for (int i = 0; i < 16384; i++) {
            float normalizedThrow;
            if (i >= 8192) {
                normalizedThrow = (float)(i - 8192) / 8191.0f;
            } else {
                normalizedThrow = (float)(i - 8192) / 8192.0f;
            }
            
            float dynamicBend = 0.0f;
            
            if (normalizedThrow >= 0.0f) {
                dynamicBend = toeBend * normalizedThrow;
            } else {
                dynamicBend = heelBend * std::abs(normalizedThrow);
            }
            
            float totalShift = basePitch + dynamicBend;
            pitchShiftLUT[i] = powf(2.0f, totalShift / 12.0f);
        }
    }
    
    if (!isVolumeMode) {
        pitchShiftFactor = pitchShiftLUT[constrain(currentPB1, 0, 16383)];
    }
}

void updateMeters() {
    int barHeight = 98;
    
    int inFillHeight = (int)(ui_audio_level * barHeight);
    if (inFillHeight > barHeight) {
        inFillHeight = barHeight;
    }
    uint32_t inColor = TFT_GREEN;
    if (ui_audio_level > 0.90f) {
        inColor = TFT_RED;
    }
    meterSpr.fillSprite(TFT_BLACK);
    meterSpr.fillRect(0, barHeight - inFillHeight, 6, inFillHeight, inColor);
    meterSpr.pushSprite(11, 31);
    
    int outFillHeight = (int)(ui_output_level * barHeight);
    if (outFillHeight > barHeight) {
        outFillHeight = barHeight;
    }
    uint32_t outColor = TFT_GREEN;
    if (ui_output_level > 0.90f) {
        outColor = TFT_RED;
    }
    meterSpr.fillSprite(TFT_BLACK);
    meterSpr.fillRect(0, barHeight - outFillHeight, 6, outFillHeight, outColor);
    meterSpr.pushSprite(spr.width() - 17, 31);
}

void updateDisplay() {
    spr.fillSprite(TFT_BLACK);
    spr.setTextDatum(MC_DATUM);
    spr.setTextSize(1);

    if (btmidi.isConnected()) {
        spr.setTextColor(TFT_GREEN, TFT_BLACK);
        spr.drawString("BT: Connected", spr.width() / 2, 10);
    } else {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        spr.drawString("BT: Waiting", spr.width() / 2, 10);
    }

    spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    char cpuStr[16];
    sprintf(cpuStr, "CPU:%2d%%", (int)core1_load);
    spr.drawString(cpuStr, 30, 10);

    int barWidth = 8;
    int barHeight = 100;
    int barY = 30;

    int inX = 10;
    
    spr.drawRect(inX, barY, barWidth, barHeight, TFT_DARKGREY);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString("IN", inX + (barWidth / 2), barY + barHeight + 10);

    int outX = spr.width() - 18;
    
    spr.drawRect(outX, barY, barWidth, barHeight, TFT_DARKGREY);
    spr.drawString("OUT", outX + (barWidth / 2), barY + barHeight + 10);

    // PERFECT SYNC LED LOGIC
    bool isCurrentEffectActive = false; 
    if (activeEffectMode == 0) isCurrentEffectActive = isWhammyActive;
    else if (activeEffectMode == 1) isCurrentEffectActive = (isWhammyActive || isFrozen);
    else if (activeEffectMode == 2) isCurrentEffectActive = (isWhammyActive || isFeedbackActive);
    else if (activeEffectMode == 3) isCurrentEffectActive = (isWhammyActive || isHarmonizerMode);
    else if (activeEffectMode == 4) isCurrentEffectActive = (isWhammyActive || isCapoMode);
    else if (activeEffectMode == 5) isCurrentEffectActive = (isWhammyActive || isSynthMode);
    else if (activeEffectMode == 6) isCurrentEffectActive = (isWhammyActive || isPadMode);
    else if (activeEffectMode == 7) isCurrentEffectActive = (isWhammyActive || isChorusMode);
    else if (activeEffectMode == 8) isCurrentEffectActive = (isWhammyActive || isSwellMode); 
    else if (activeEffectMode == 9) isCurrentEffectActive = (isWhammyActive || isVibratoMode);

    uint32_t ledColor = TFT_RED;
    if (isCurrentEffectActive) {
        ledColor = TFT_GREEN;
    }
    
    spr.fillCircle(spr.width() - 40, 25, 8, ledColor);
    spr.drawCircle(spr.width() - 40, 25, 8, TFT_WHITE);

    spr.setTextSize(3);
    
    switch(activeEffectMode) {
        case 0: 
            spr.setTextColor(TFT_ORANGE, TFT_BLACK); 
            spr.drawString("WHAMMY", spr.width() / 2 + 30, 40); 
            break;
        case 1: 
            spr.setTextColor(TFT_CYAN, TFT_BLACK); 
            spr.drawString("FREEZE", spr.width() / 2 + 30, 40); 
            break;
        case 2: 
            spr.setTextColor(TFT_RED, TFT_BLACK); 
            spr.drawString("FEEDBACK", spr.width() / 2 + 30, 40); 
            break;
        case 3: 
            spr.setTextColor(TFT_MAGENTA, TFT_BLACK); 
            spr.drawString("HARMONY", spr.width() / 2 + 30, 40); 
            break;
        case 4: 
            spr.setTextColor(TFT_GREEN, TFT_BLACK); 
            spr.drawString("CAPO", spr.width() / 2 + 30, 40); 
            break;
        case 5: 
            spr.setTextColor(TFT_YELLOW, TFT_BLACK); 
            spr.drawString("SYNTH", spr.width() / 2 + 30, 40); 
            break;
        case 6: 
            spr.setTextColor(TFT_PINK, TFT_BLACK); 
            spr.drawString("PAD", spr.width() / 2 + 30, 40); 
            break;
        case 7: 
            spr.setTextColor(TFT_SKYBLUE, TFT_BLACK); 
            spr.drawString("CHORUS", spr.width() / 2 + 30, 40); 
            break;
        case 8: 
            spr.setTextColor(TFT_WHITE, TFT_BLACK); 
            spr.drawString("SWELL", spr.width() / 2 + 30, 40); 
            break;
        case 9: 
            spr.setTextColor(TFT_PURPLE, TFT_BLACK); 
            spr.drawString("VIBRATO", spr.width() / 2 + 30, 40); 
            break;
    }

    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    
    float displayVal = effectMemory[activeEffectMode];
    
    if (activeEffectMode == 2) {
        float fbIntervals[5] = { 0.0f, 12.0f, 19.0f, 24.0f, 28.0f };
        displayVal = fbIntervals[feedbackIntervalIdx % 5];
    } else if (activeEffectMode == 5) {
        displayVal = effectMemory[6];
    } else if (activeEffectMode == 6) {
        displayVal = effectMemory[7];
    } else if (activeEffectMode == 7) {
        displayVal = effectMemory[8];
    } else if (activeEffectMode == 9) {
        displayVal = effectMemory[9];
    }

    spr.setTextSize(1);
    int lineTop = 30; 
    int lineBot = 130; 
    int x1 = 35; 
    int x2 = 70; 
    int x3 = 105;
    
    spr.drawString("PB1", x1, lineBot + 15); 
    spr.drawString("PB2", x2, lineBot + 15); 
    spr.drawString("CC11", x3, lineBot + 15);
    
    for (int y = lineTop; y <= lineBot; y += 5) { 
        spr.drawFastVLine(x1, y, 2, TFT_DARKGREY); 
        spr.drawFastVLine(x2, y, 2, TFT_DARKGREY); 
        spr.drawFastVLine(x3, y, 2, TFT_DARKGREY); 
    }
    
    spr.fillCircle(x1, map(currentPB1, 0, 16383, lineBot, lineTop), 4, TFT_CYAN);
    spr.fillCircle(x2, map(currentPB2, 0, 16383, lineBot, lineTop), 4, TFT_MAGENTA);
    spr.fillCircle(x3, map(currentCC11, 0, 16383, lineBot, lineTop), 4, TFT_GREEN);

    int textX = spr.width() / 2 + 35; 

    if (activeEffectMode == 0 || activeEffectMode == 8 || activeEffectMode == 1) { 
        char topStr[16]; 
        char botStr[16];
        spr.setTextSize(3); 
        if (effectMemory[0] > 0) {
            sprintf(topStr, "+%.1f", effectMemory[0]);
        } else {
            sprintf(topStr, "%.1f", effectMemory[0]);
        }
        
        if (effectMemory[5] > 0) {
            sprintf(botStr, "+%.1f", effectMemory[5]);
        } else {
            sprintf(botStr, "%.1f", effectMemory[5]);
        }
        spr.drawString(topStr, textX, (spr.height() / 2) - 15);
        spr.drawString(botStr, textX, (spr.height() / 2) + 15);
    } 
    else if (activeEffectMode == 4) {
        char topStr[16]; 
        char botStr[16];
        spr.setTextSize(2); 
        int totalCents = (int)round(effectMemory[4] * 100.0f);
        int cInt = totalCents / 100; 
        int cCents = totalCents % 100;
        
        if (cInt > 0) {
            sprintf(topStr, "Int: +%d", cInt);
        } else {
            sprintf(topStr, "Int: %d", cInt);
        }
        
        if (cCents > 0) {
            sprintf(botStr, "Ct: +%d", cCents);
        } else {
            sprintf(botStr, "Ct: %d", cCents);
        }
        spr.drawString(topStr, textX, (spr.height() / 2) - 15);
        spr.drawString(botStr, textX, (spr.height() / 2) + 15);
    } 
    else { 
        spr.setTextSize(4); 
        char intervalStr[16];
        
        if (displayVal > 0) {
            sprintf(intervalStr, "+%.1f", displayVal);
        } else {
            sprintf(intervalStr, "%.1f", displayVal);
        }
        spr.drawString(intervalStr, textX, spr.height() / 2 + 10);
    }

    spr.setTextSize(2); 
    int bannerCount = 0;
    
    auto drawBanner = [&](const char* text, uint32_t color, bool isCrossLined = false) {
        int col = bannerCount % 3; 
        int row = bannerCount / 3;
        int x = (spr.width() / 6) + (col * (spr.width() / 3));
        int y = (spr.height() - 55) + (row * 20);
        
        spr.setTextColor(color, TFT_BLACK); 
        spr.drawString(text, x, y);
        
        if (isCrossLined) {
            int textW = spr.textWidth(text);
            spr.drawFastHLine(x - (textW / 2), y, textW, color);
            spr.drawFastHLine(x - (textW / 2), y + 1, textW, color);
        }
        
        bannerCount++;
    };
    
    if (isFrozen && activeEffectMode != 1) drawBanner("FROZEN", TFT_CYAN);
    if (isFeedbackActive && activeEffectMode != 2) drawBanner("SCREAM", TFT_RED);
    if (isHarmonizerMode && activeEffectMode != 3) drawBanner("HARM", TFT_MAGENTA);
    if (isCapoMode && activeEffectMode != 4) drawBanner("CAPO", TFT_GREEN);
    if (isSynthMode && activeEffectMode != 5) drawBanner("SYNTH", TFT_YELLOW);
    if (isPadMode && activeEffectMode != 6) drawBanner("PAD", TFT_PINK);
    if (isChorusMode && activeEffectMode != 7) drawBanner("CHORUS", TFT_SKYBLUE);
    if (isSwellMode && activeEffectMode != 8) drawBanner("SWELL", TFT_WHITE); 
    if (isVibratoMode && activeEffectMode != 9) drawBanner("VIB", TFT_PURPLE); 
    if (isVolumeMode) drawBanner("VOLUME", TFT_DARKGREY);

    spr.setTextSize(1); 
    spr.setTextColor(TFT_WHITE);
    spr.drawRect(outX - 46, (spr.height() / 2) - 8, 40, 16, TFT_DARKGREY);
    const char* latLabels[] = {"U.Low", "Low", "Mid", "High"};
    spr.drawString(latLabels[latencyMode], outX - 26, (spr.height() / 2));
    
    spr.pushSprite(0, 0);
    
    updateMeters();
}

void DisplayTask(void * pvParameters) {
    for(;;) {
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
            updateDisplay(); 
            forceUIUpdate = false; 
        } else if (!isScreenOff && (ui_audio_level > 0.02f || ui_output_level > 0.02f)) {
            updateMeters();
        }
        
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

// --- AUDIO DSP TASK ---
void IRAM_ATTR AudioDSPTask(void * pvParameters) {
    static float dsp_out[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_in[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_out[HOP_SIZE * 2] __attribute__((aligned(16)));
    
    static float synthEnv = 0.0f;
    static float synthFilter = 0.0f;
    static float padFilter = 0.0f;
    static float padEnv = 0.0f;
    static float inputEnvelope = 0.0f;
    
    static float dc_state_in = 0.0f;
    static float dc_state_out = 0.0f;
    
    static int freezeWriteIdx = 0;
    static int freezePlayCounter = 0;
    static int freezeStartIdx = 0;
    
    const float norm = 1.0f / 2147483648.0f; 
    i2s_chan_info_t rx_info; 
    i2s_chan_info_t tx_info;
    
    for(;;) {
        if (sleepRequested) {
            isSleeping = true;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        isSleeping = false;

        i2s_channel_get_info(rx_chan, &rx_info); 
        i2s_channel_get_info(tx_chan, &tx_info);
        
        size_t bytes_read; 
        i2s_channel_read(rx_chan, i2s_in, sizeof(i2s_in), &bytes_read, portMAX_DELAY);
        
        if (bytes_read > 0) {
            uint32_t start_cycles = xthal_get_ccount();
            
            float targetWin = LATENCY_WINDOWS[latencyMode];
            if (currentWindowSize != targetWin) { 
                currentWindowSize = targetWin; 
                tap1 = 0.0f; 
                tap2 = targetWin / 2.0f; 
                tap1_2 = 0.0f; 
                tap2_2 = targetWin / 2.0f; 
            }
            
            float hannMultiplier = 1023.0f / currentWindowSize; 
            float invFreezeLength = 1.0f / 48000.0f;
            float chorusPhaseInc = 1536.0f / 96000.0f; 
            float feedbackPhaseInc = 5120.0f / 96000.0f; 
            
            float vibHz = (effectMemory[9] == 0.0f) ? 2.0f : fabsf(effectMemory[9]);
            float vibratoPhaseInc = (vibHz * LFO_LUT_SIZE) / SAMPLING_FREQUENCY;

            bool freezeActive = ((activeEffectMode == 1 && isWhammyActive) || isFrozen);
            
            if (freezeActive && !wasFrozen) {
                freezePlayCounter = 0;
                freezeStartIdx = freezeWriteIdx; 
            }
            wasFrozen = freezeActive;
            
            bool synth = ((activeEffectMode == 5 && isWhammyActive) || isSynthMode);
            bool pad = ((activeEffectMode == 6 && isWhammyActive) || isPadMode);
            bool harm = ((activeEffectMode == 3 && isWhammyActive) || isHarmonizerMode);
            bool swell = ((activeEffectMode == 8 && isWhammyActive) || isSwellMode); 
            bool chorus = ((activeEffectMode == 7 && isWhammyActive) || isChorusMode);
            bool feedback = ((activeEffectMode == 2 && isWhammyActive) || isFeedbackActive);
            bool vibrato = ((activeEffectMode == 9 && isWhammyActive) || isVibratoMode);
            bool capo = ((activeEffectMode == 4 && isWhammyActive) || isCapoMode);
            
            float pIn = 0.0f;
            float pOut = 0.0f;
            
            for (int i = 0; i < HOP_SIZE * 2; i += 2) {
                
                float raw_input = (float)i2s_in[i] * norm;
                float input = raw_input - dc_state_in + 0.995f * dc_state_out;
                dc_state_in = raw_input;
                dc_state_out = input;
                 
                inputEnvelope = inputEnvelope * 0.99f + fabsf(input) * 0.01f;
                
                if (swell) {
                    if (inputEnvelope > 0.015f) {
                        swellGain = fminf(1.0f, swellGain + 0.00015f); 
                    } else {
                        swellGain = fmaxf(0.0f, swellGain - 0.002f); 
                    }
                } else {
                    swellGain = 1.0f; 
                }

                float writeVal = input;
                
                if (!freezeActive) {
                    freezeBuffer[freezeWriteIdx] = writeVal;
                    freezeWriteIdx++;
                    if (freezeWriteIdx >= freezeLength) freezeWriteIdx = 0;
                }
                
                if (synth) {
                    if (inputEnvelope > 0.005f) {
                        synthEnv = fminf(1.0f, synthEnv + 0.1f);
                    } else {
                        synthEnv = fmaxf(0.0f, synthEnv - 0.005f);
                    }
                    
                    int lutIdx = (int)( (writeVal + 1.0f) * 0.5f * (WAVE_LUT_SIZE - 1) );
                    if (lutIdx < 0) {
                        lutIdx = 0;
                    } else if (lutIdx >= WAVE_LUT_SIZE) {
                        lutIdx = WAVE_LUT_SIZE - 1;
                    }
                    
                    writeVal = synthLUT[lutIdx]; 
                    synthFilter = synthFilter + (0.3f + 0.8f * synthEnv) * (writeVal - synthFilter);
                    writeVal = synthFilter * 0.1f; 
                } else if (pad) {
                    if (inputEnvelope > 0.005f) {
                        padEnv = fminf(1.0f, padEnv + 0.00002f);
                    } else {
                        padEnv = fmaxf(0.0f, padEnv - 0.000005f);
                    }
                    writeVal *= padEnv;
                }

                if (freezeRamp > 0.0f || freezeActive) {
                    if (freezeActive) freezeRamp = fminf(1.0f, freezeRamp + 0.0002f);
                    else freezeRamp = fmaxf(0.0f, freezeRamp - 0.00005f);
                }

                float freezeOut = 0.0f;
                if (freezeRamp > 0.0f) {
                    float phase = (float)freezePlayCounter * invFreezeLength;
                    
                    int i1 = (freezeStartIdx + freezePlayCounter);
                    while (i1 >= freezeLength) i1 -= freezeLength; 
                    
                    int i2 = (freezeStartIdx + freezePlayCounter + freezeLength / 2);
                    while (i2 >= freezeLength) i2 -= freezeLength; 
                    
                    float phase2 = phase + 0.5f;
                    if (phase2 >= 1.0f) {
                        phase2 -= 1.0f;
                    }
                    
                    int l1 = (int)(phase * 1023.0f);
                    int l2 = (int)(phase2 * 1023.0f);
                    float rawFreeze = (freezeBuffer[i1] * hannLUT[l1]) + (freezeBuffer[i2] * hannLUT[l2]);
                    
                    float d1 = apf1Buffer[apf1Idx]; 
                    float a1 = -0.6f * rawFreeze + d1; 
                    apf1Buffer[apf1Idx] = rawFreeze + 0.6f * d1; 
                    
                    apf1Idx++;
                    if (apf1Idx >= 1009) {
                        apf1Idx = 0;
                    }
                    
                    float d2 = apf2Buffer[apf2Idx]; 
                    float a2 = -0.6f * a1 + d2; 
                    apf2Buffer[apf2Idx] = a1 + 0.6f * d2; 
                    
                    apf2Idx++;
                    if (apf2Idx >= 863) {
                        apf2Idx = 0;
                    }
                    
                    freezeOut = a2 * freezeRamp;
                    
                    freezePlayCounter++;
                    if (freezePlayCounter >= freezeLength) {
                        freezePlayCounter = 0;
                    }
                }

                float finalWriteVal = writeVal;
                if (freezeActive && freezeRamp > 0.0f) {
                    finalWriteVal = freezeOut;
                }
                
                delayBuffer[writeIndex] = fmaxf(-1.0f, fminf(finalWriteVal, 1.0f));

                float f1 = pitchShiftFactor;
                float f2 = pitchShiftFactor;
                
                if (feedback) { 
                    feedbackLfoPhase += feedbackPhaseInc;
                    if (feedbackLfoPhase >= LFO_LUT_SIZE) {
                        feedbackLfoPhase -= LFO_LUT_SIZE;
                    }
                    
                    float drift = lfoLUT[(int)feedbackLfoPhase];
                    f1 = 1.0f * drift; 
                    f2 = pitchShiftFactor * drift; 
                }
                else if (harm) {
                    f2 = pitchShiftFactor * powf(2.0f, effectMemory[3] / 12.0f);
                }
                else if (chorus) { 
                    chorusLfoPhase += chorusPhaseInc;
                    if (chorusLfoPhase >= LFO_LUT_SIZE) {
                        chorusLfoPhase -= LFO_LUT_SIZE;
                    }
                    
                    int p1 = (int)chorusLfoPhase;
                    
                    float chorusRatio = powf(2.0f, effectMemory[8] / 12.0f);
                    f2 = pitchShiftFactor * chorusRatio * lfoLUT[p1]; 
                } 
                else if (vibrato) {
                    vibratoLfoPhase += vibratoPhaseInc;
                    if (vibratoLfoPhase >= LFO_LUT_SIZE) {
                        vibratoLfoPhase -= LFO_LUT_SIZE;
                    }
                    f1 = pitchShiftFactor * lfoLUT[(int)vibratoLfoPhase];
                }
                
                int idx1 = (int)(tap1 * hannMultiplier);
                int idx2 = (int)(tap2 * hannMultiplier);
                
                float w1 = (getHermiteSample(tap1 + 1.0f, delayBuffer, writeIndex) * hannLUT[idx1]) + 
                           (getHermiteSample(tap2 + 1.0f, delayBuffer, writeIndex) * hannLUT[idx2]);
                           
                float w2 = 0.0f;
                if (feedback || harm || chorus) {
                    int idx1_2 = (int)(tap1_2 * hannMultiplier);
                    int idx2_2 = (int)(tap2_2 * hannMultiplier);
                    
                    w2 = (getHermiteSample(tap1_2 + 1.0f, delayBuffer, writeIndex) * hannLUT[idx1_2]) + 
                         (getHermiteSample(tap2_2 + 1.0f, delayBuffer, writeIndex) * hannLUT[idx2_2]);
                }

                float r1 = 1.0f - f1;
                float r2 = 1.0f - f2;
                
                tap1 += r1; 
                while (tap1 >= currentWindowSize) tap1 -= currentWindowSize; 
                while (tap1 < 0.0f) tap1 += currentWindowSize;
                
                tap2 += r1; 
                while (tap2 >= currentWindowSize) tap2 -= currentWindowSize; 
                while (tap2 < 0.0f) tap2 += currentWindowSize;
                
                tap1_2 += r2; 
                while (tap1_2 >= currentWindowSize) tap1_2 -= currentWindowSize; 
                while (tap1_2 < 0.0f) tap1_2 += currentWindowSize;
                
                tap2_2 += r2; 
                while (tap2_2 >= currentWindowSize) tap2_2 -= currentWindowSize; 
                while (tap2_2 < 0.0f) tap2_2 += currentWindowSize;
                
                writeIndex = (writeIndex + 1) & BUFFER_MASK;

                float feedbackOut = 0.0f;
                if (feedback || feedbackRamp > 0.0f) {
                    if (feedback) {
                        if (inputEnvelope > 0.005f) {
                            feedbackRamp = fminf(1.0f, feedbackRamp + 0.000011f);
                        } else {
                            feedbackRamp = fmaxf(0.0f, feedbackRamp - 0.005f);
                        }
                    } else {
                        feedbackRamp = fmaxf(0.0f, feedbackRamp - 0.0001f);
                    }
                    
                    float bloom = fmaxf(0.0f, fminf((feedbackRamp - 0.1f) * 2.0f, 1.0f));
                    float bloomed;
                    
                    // If Freeze is running, feed the pure unbent drone directly into the Scream matrix
                    if (freezeActive && freezeRamp > 0.0f) {
                        bloomed = freezeOut; 
                    } else {
                        bloomed = (w1 * (1.0f - bloom)) + (w2 * bloom);
                    }
                    
                    fbHpfState += 0.05f * (bloomed - fbHpfState);
                    
                    float driven = (bloomed - fbHpfState) * 30.0f;
                    float scream = driven;
                    
                    if (driven > 1.0f) {
                        scream = 1.0f;
                    } else if (driven < -1.0f) {
                        scream = -1.0f;
                    }
                    
                    feedbackFilter = feedbackFilter * 0.9f + scream * 0.1f;
                    
                    float r = feedbackRamp;
                    float rampCubed = r * r * r;
                    float rawFb = feedbackFilter * rampCubed * 0.85f;
                    
                    int rIdx = (fbDelayWriteIdx - (int)(SAMPLING_FREQUENCY * (20.0f / 1000.0f)) + 8192) & 8191;
                    fbDelayBuffer[fbDelayWriteIdx] = rawFb; 
                    fbDelayWriteIdx = (fbDelayWriteIdx + 1) & 8191;
                    
                    feedbackOut = fbDelayBuffer[rIdx];
                }

                // --- PARALLEL SUMMING MIXER ---
                if (pad) {
                    padFilter = padFilter * 0.95f + w1 * 0.05f;
                } else {
                    padFilter = padFilter * 0.95f; 
                }

                bool isAnyEffectActive = isWhammyActive || harm || chorus || feedback || synth || pad || freezeActive || vibrato || capo;
                bool isBlendedEffect = harm || chorus || pad || freezeActive || feedback || (freezeRamp > 0.0f) || (feedbackRamp > 0.0f);

                float shiftedOutput = 0.0f;

                if (!isAnyEffectActive && freezeRamp <= 0.0f && feedbackRamp <= 0.0f && padFilter < 0.001f) {
                    // True Bypass (Nothing running)
                    shiftedOutput = input; 
                } else {
                    float mixedSignal = 0.0f;

                    // 1. Base Core (Dry + w1)
                    if (isBlendedEffect) {
                        mixedSignal += (input * 0.4f);
                        mixedSignal += (w1 * 0.4f);
                    } else {
                        // 100% Wet for pure pitch replacement (Whammy, Capo, Synth, Vibrato)
                        mixedSignal += w1; 
                    }

                    // 2. Harmony / Chorus (w2)
                    if (harm || chorus) {
                        mixedSignal += (w2 * 0.4f);
                    }

                    // 3. Pad
                    if (pad || padFilter > 0.001f) {
                        mixedSignal += (padFilter * 1.5f);
                    }

                    // 4. Freeze Drone
                    if (freezeActive || freezeRamp > 0.0f) {
                        mixedSignal += (freezeOut * 0.5f);
                    }

                    // 5. Feedback Scream
                    if (feedback || feedbackRamp > 0.0f) {
                        mixedSignal += (feedbackOut * 0.6f);
                    }

                    // 6. Soft Clipping / Saturation (Prevents harsh digital clipping when all effects stack)
                    if (mixedSignal > 1.25f) mixedSignal = 1.25f;
                    else if (mixedSignal < -1.25f) mixedSignal = -1.25f;
                    
                    shiftedOutput = mixedSignal * (1.0f - (0.1f * mixedSignal * mixedSignal));
                }

                float finalOutput = shiftedOutput * swellGain * volumePedalGain; 
                
                dsp_out[i] = finalOutput; 
                dsp_out[i+1] = finalOutput; 
                
                if (fabsf(input) > pIn) {
                    pIn = fabsf(input);
                }
                
                if (fabsf(finalOutput) > pOut) {
                    pOut = fabsf(finalOutput);
                }
            }
            
            uint32_t end_cycles = xthal_get_ccount();
            float current_load = ((float)(end_cycles - start_cycles) / 160000.0f) * 100.0f;
            
            if (current_load > 100.0f) {
                current_load = 100.0f;
            }
            
            core1_load = core1_load * 0.95f + current_load * 0.05f;

            float sc = 2147483647.0f; 
            dsps_mul_f32(dsp_out, &sc, dsp_out, HOP_SIZE * 2, 1, 0, 1);
            
            #pragma GCC ivdep
            for(int i = 0; i < HOP_SIZE * 2; i++) {
                i2s_out[i] = (int32_t)fmaxf(-2147483648.0f, fminf(dsp_out[i], 2147483647.0f));
            }
            
            if (pIn > ui_audio_level) {
                ui_audio_level = pIn; 
            } else {
                ui_audio_level = ui_audio_level * 0.998f; 
                if (ui_audio_level < 0.025f) ui_audio_level = 0.0f; 
            }
            
            if (pOut > ui_output_level) {
                ui_output_level = pOut; 
            } else {
                ui_output_level = ui_output_level * 0.998f; 
                if (ui_output_level < 0.025f) ui_output_level = 0.0f; 
            }
            
            size_t bw; 
            i2s_channel_write(tx_chan, i2s_out, sizeof(i2s_out), &bw, portMAX_DELAY);
        }
    }
}

struct DebouncedButton {
    uint8_t pin; 
    bool state; 
    bool lastReading;
    unsigned long lastDebounceTime; 
    unsigned long pressedTime; 
    bool isActive;
    
    DebouncedButton(uint8_t p) { 
        pin = p; 
        state = HIGH; 
        lastReading = HIGH; 
        lastDebounceTime = 0; 
        pressedTime = 0; 
        isActive = false; 
    }
    
    bool update(unsigned long debounceDelay = 50) {
        bool currentReading = digitalRead(pin); 
        bool stateChanged = false;
        
        if (currentReading != lastReading) {
            lastDebounceTime = millis();
        }
        
        if ((millis() - lastDebounceTime) > debounceDelay) {
            if (currentReading != state) { 
                state = currentReading; 
                stateChanged = true; 
            }
        }
        
        lastReading = currentReading; 
        return stateChanged;
    }
};

void MidiTask(void * pvParameters) {
    static analog_t lastMidiA = 8192;
    static analog_t lastMidiB = 8192; 
    static bool lastBtState = false; 
    static uint8_t lastVolumeCC = 127;

    static DebouncedButton btnCar(CAROUSEL_BUTTON_PIN);
    static DebouncedButton btnFreeze(FREEZE_BUTTON_PIN);
    static DebouncedButton btnFb(FEEDBACK_BUTTON_PIN);
    static DebouncedButton btnInterval(INTERVAL_BUTTON_PIN);

    btnCar.state = digitalRead(CAROUSEL_BUTTON_PIN); 
    btnCar.lastReading = btnCar.state;
    
    btnFreeze.state = digitalRead(FREEZE_BUTTON_PIN); 
    btnFreeze.lastReading = btnFreeze.state;
    
    btnFb.state = digitalRead(FEEDBACK_BUTTON_PIN); 
    btnFb.lastReading = btnFb.state;
    
    btnInterval.state = digitalRead(INTERVAL_BUTTON_PIN); 
    btnInterval.lastReading = btnInterval.state;

    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP);
    
    lastActivityTime = millis(); 
    lastScreenActivityTime = millis();

    for(;;) {
        Control_Surface.loop();
        
        bool currentBtState = btmidi.isConnected();
        
        if (currentBtState != lastBtState) {
            lastBtState = currentBtState; 
            forceUIUpdate = true;
            
            if (isScreenOff) {
                turnScreenOn();
            }
            
            lastActivityTime = millis();
        }

        if (currentBtState) {
            lastActivityTime = millis(); 
        }
        
        if (!currentBtState && (millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT)) {
            goToLightSleep();
        }
        
        if (!isScreenOff && (millis() - lastScreenActivityTime > SCREEN_OFF_TIMEOUT)) {
            turnScreenOff();
        }

        if (btnCar.update(100)) {
            if (btnCar.state == LOW) {
                btnCar.pressedTime = millis(); 
                btnCar.isActive = true; 
            } else if (btnCar.state == HIGH && btnCar.isActive) {
                btnCar.isActive = false; 
                unsigned long dur = millis() - btnCar.pressedTime;
                
                if (dur < 400) { 
                    activeEffectMode = (activeEffectMode + 1) % 10; 
                    
                    chorusLfoPhase = 0.0f;
                    feedbackLfoPhase = 0.0f;
                    vibratoLfoPhase = 0.0f;
                    swellGain = 0.0f;
                    
                    // Auto-Engage newly selected effect
                    isWhammyActive = true; 
                    
                    updateLUT(); 
                }
                forceUIUpdate = true;
            }
        }

        if (btnFreeze.update(100)) { 
            if (btnFreeze.state == LOW) { 
                isFrozen = !isFrozen; 
                if (activeEffectMode == 1) {
                    isWhammyActive = isFrozen; 
                }
                forceUIUpdate = true; 
            } 
        }
        
        if (btnFb.update(100)) { 
            if (btnFb.state == LOW) { 
                isFeedbackActive = !isFeedbackActive; 
                if (activeEffectMode == 2) {
                    isWhammyActive = isFeedbackActive; 
                }
                forceUIUpdate = true; 
            } 
        }
        
        if (btnInterval.update(100)) {
            if (btnInterval.state == LOW) {
                if (activeEffectMode == 2) {
                    feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5;
                } else {
                    currentIntervalIdx = (currentIntervalIdx + 1) % 9; 
                    
                    int slot = activeEffectMode;
                    
                    if (slot == 1) slot = 0; 
                    
                    if (slot == 5) slot = 6;
                    else if (slot == 6) slot = 7;
                    else if (slot == 7) slot = 8;
                    else if (slot == 9) slot = 9; 
                    
                    if (slot == 4) {
                        float currentCents = effectMemory[4] - (int)effectMemory[4];
                        effectMemory[4] = intervalList[currentIntervalIdx] + currentCents;
                    } 
                    else if (activeEffectMode == 8) {
                        // Do nothing
                    } 
                    else {
 
                        effectMemory[slot] = intervalList[currentIntervalIdx]; 
                    }
                }
                
                updateLUT(); 
                forceUIUpdate = true;
                lastActivityTime = millis();
            }
        }

        filterPB.update(); 
        filterPB2.update(); 
        
        bool isGhostButtonActive = (digitalRead(BOOT_SENSE_PIN) == LOW);

        if (!isGhostButtonActive) {
            analog_t raw14_A = map(filterPB.getValue(), 0, 4095, 0, 16383); 
            analog_t raw14_B = map(filterPB2.getValue(), 0, 4095, 0, 16383); 
            
            analog_t calibratedA = map_PB(raw14_A, PBcenter1, PBdeadzone1, PBwasOffCenter1); 
            analog_t calibratedB = map_PB(raw14_B, PBcenter2, PBdeadzone2, PBwasOffCenter2); 
            
            int diffA = abs((int)calibratedA - (int)lastMidiA);
            int diffB = abs((int)calibratedB - (int)lastMidiB);
            
            if (diffA > 256 || diffB > 256) {
                if (isScreenOff) {
                    turnScreenOn();
                }
                lastScreenActivityTime = millis();
            }

            // Both Pedals are forced OFF and will be completely ignored for Bench testing
            bool movedA = false; 
            bool movedB = false; 
            
            if (movedA || movedB) {
                
                if (movedA) {
                    currentPB1 = calibratedA; 
                    lastMidiA = calibratedA; 
                }
                
                if (movedB) {
                    currentPB2 = calibratedB; 
                    lastMidiB = calibratedB; 
                }
                
                analog_t activeMidi = movedB ? calibratedB : calibratedA;
                
                if (isVolumeMode) {
                    uint8_t ccVal = map(activeMidi, 0, 16383, 0, 127);
                    if (ccVal != lastVolumeCC) {
                        Control_Surface.sendControlChange({19, Channel_1}, ccVal);
                        lastVolumeCC = ccVal;
                        forceUIUpdate = true;
                    }
                    volumePedalGain = (float)activeMidi / 16383.0f; 
                } else {
                    pitchShiftFactor = pitchShiftLUT[constrain(activeMidi, 0, 16383)];
                    Control_Surface.sendPitchBend(Channel_1, activeMidi);
                    lastMidiSent = activeMidi; 
                    forceUIUpdate = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool channelMessageCallback(ChannelMessage cm) {
    if (cm.header == 0xB0) {
        
        if (cm.data1 == 20) {
            isVolumeMode = (cm.data2 >= 64);
            
            if (!isVolumeMode) {
                volumePedalGain = 1.0f; 
            }
            forceUIUpdate = true;
        }
        else if (cm.data1 == 4 && cm.data2 >= 64) { 
            if (activeEffectMode == 0) {
                activeEffectMode = 9;
            } else {
                activeEffectMode = activeEffectMode - 1;
            }
            
            chorusLfoPhase = 0.0f;
            feedbackLfoPhase = 0.0f;
            vibratoLfoPhase = 0.0f;
            swellGain = 0.0f;
            
            // Auto-Engage newly selected effect
            isWhammyActive = true; 
            
            updateLUT(); 
            forceUIUpdate = true;
        }
        else if (cm.data1 == 5 && cm.data2 >= 64) { 
            activeEffectMode = (activeEffectMode + 1) % 10; 
            
            chorusLfoPhase = 0.0f;
            feedbackLfoPhase = 0.0f;
            vibratoLfoPhase = 0.0f;
            swellGain = 0.0f;
            
            // Auto-Engage newly selected effect
            isWhammyActive = true; 
            
            updateLUT(); 
            forceUIUpdate = true;
        }
        else if (cm.data1 == 6 && cm.data2 >= 64) { 
            latencyMode = (latencyMode + 1) % 4; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 7) {
            if (cm.data2 >= 64) {
                isWhammyActive = false;
                isFrozen = false;
                isFeedbackActive = false; 
                isHarmonizerMode = false;
                isCapoMode = false;
                isSynthMode = false;
                isPadMode = false;
                isChorusMode = false;
                isSwellMode = false;
                isVibratoMode = false;
                
                // Release Volume Mode and return PB to pitch control
                isVolumeMode = false; 
                volumePedalGain = 1.0f; 
            } else {
                activeEffectMode = 0;
                isWhammyActive = true;
                isFrozen = false;
                isFeedbackActive = false; 
                isHarmonizerMode = false;
                isCapoMode = false;
                isSynthMode = false;
                isPadMode = false;
                isChorusMode = false;
                isSwellMode = false;
                isVibratoMode = false;
                
                // Release Volume Mode and return PB to pitch control
                isVolumeMode = false; 
                volumePedalGain = 1.0f; 
            }
            updateLUT();
            forceUIUpdate = true;
        }
        else if (cm.data1 == 8) { 
            isFrozen = (cm.data2 >= 64); 
            if (activeEffectMode == 1) isWhammyActive = isFrozen; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 9) { 
            isFeedbackActive = (cm.data2 >= 64); 
            if (activeEffectMode == 2) {
                isWhammyActive = isFeedbackActive; 
            }
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 10) { 
            isHarmonizerMode = (cm.data2 >= 64); 
            if (activeEffectMode == 3) isWhammyActive = isHarmonizerMode; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 12) { 
            isCapoMode = (cm.data2 >= 64); 
            if (activeEffectMode == 4) isWhammyActive = isCapoMode; 
            updateLUT(); 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 13) { 
            isSynthMode = (cm.data2 >= 64); 
            if (activeEffectMode == 5) isWhammyActive = isSynthMode; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 14) { 
            isPadMode = (cm.data2 >= 64); 
            if (activeEffectMode == 6) isWhammyActive = isPadMode; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 15) { 
            isChorusMode = (cm.data2 >= 64); 
            if (activeEffectMode == 7) isWhammyActive = isChorusMode; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 16) { 
            isSwellMode = (cm.data2 >= 64); 
            if (activeEffectMode == 8) isWhammyActive = isSwellMode; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 21) { 
            isVibratoMode = (cm.data2 >= 64); 
            if (activeEffectMode == 9) isWhammyActive = isVibratoMode; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 18) {
            if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
                if (cm.data2 < 64) {
                    effectMemory[0] = constrain(effectMemory[0] + 1.0f, -24.0f, 24.0f);
                } else {
                    effectMemory[0] = constrain(effectMemory[0] - 1.0f, -24.0f, 24.0f);
                }
            } else if (activeEffectMode == 4) {
                float newVal = effectMemory[4] + (cm.data2 < 64 ? 1.0f : -1.0f);
                effectMemory[4] = constrain(roundf(newVal * 100.0f) / 100.0f, -24.0f, 24.0f);
            } else if (activeEffectMode == 2) {
                if (cm.data2 < 64) {
                    feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5;
                } else {
                    feedbackIntervalIdx = (feedbackIntervalIdx - 1 + 5) % 5;
                }
            } else {
                int slot = activeEffectMode; 
                if (activeEffectMode == 5) slot = 6; 
                else if (activeEffectMode == 6) slot = 7; 
                else if (activeEffectMode == 7) slot = 8;
                else if (activeEffectMode == 9) slot = 9;

                if (cm.data2 < 64) {
                    effectMemory[slot] = constrain(effectMemory[slot] + 1.0f, -24.0f, 24.0f);
                } else {
                    effectMemory[slot] = constrain(effectMemory[slot] - 1.0f, -24.0f, 24.0f);
                }
            }
            updateLUT(); 
            forceUIUpdate = true;
        }
        else if (cm.data1 == 17) {
            if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
                if (cm.data2 < 64) {
                    effectMemory[5] = constrain(effectMemory[5] + 1.0f, -24.0f, 24.0f);
                } else {
                    effectMemory[5] = constrain(effectMemory[5] - 1.0f, -24.0f, 24.0f);
                }
            } else if (activeEffectMode == 4) {
                float newVal = effectMemory[4] + (cm.data2 < 64 ? 0.01f : -0.01f);
                effectMemory[4] = constrain(roundf(newVal * 100.0f) / 100.0f, -24.0f, 24.0f);
            } else if (activeEffectMode == 2) {
                if (cm.data2 < 64) {
                    feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5;
                } else {
                    feedbackIntervalIdx = (feedbackIntervalIdx - 1 + 5) % 5;
                }
            } else {
                int slot = activeEffectMode; 
                if (activeEffectMode == 5) slot = 6; 
                else if (activeEffectMode == 6) slot = 7; 
                else if (activeEffectMode == 7) slot = 8;
                else if (activeEffectMode == 9) slot = 9;

                if (cm.data2 < 64) {
                    effectMemory[slot] = constrain(effectMemory[slot] + 1.0f, -24.0f, 24.0f);
                } else {
                    effectMemory[slot] = constrain(effectMemory[slot] - 1.0f, -24.0f, 24.0f);
                }
            }
            updateLUT(); 
            forceUIUpdate = true;
        }
        else if (cm.data1 == 11) { 
            uint16_t m = map(cm.data2, 0, 127, 0, 16383); 
            currentCC11 = m; 
            currentPB1 = m; 
            pitchShiftFactor = pitchShiftLUT[m]; 
            forceUIUpdate = true;
        }
    }
    return false;
}

void setup() {
    pinMode(CAROUSEL_BUTTON_PIN, INPUT_PULLUP);
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
    tft.drawString("BOOTING...", tft.width() / 2, tft.height() / 2);
    
    delay(120); 
    digitalWrite(38, HIGH); 
    
    btmidi.setName("Whammy_S3");
    
    pinMode(FREEZE_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(INTERVAL_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(FEEDBACK_BUTTON_PIN, INPUT_PULLUP);
    
    pinMode(pinPB, INPUT); 
    pinMode(pinPB2, INPUT);
    
    delayBuffer = (float*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    fbDelayBuffer = (float*)heap_caps_aligned_alloc(16, 8192 * sizeof(float), MALLOC_CAP_SPIRAM);
    freezeBuffer = (float*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    
    if (delayBuffer == NULL || fbDelayBuffer == NULL || freezeBuffer == NULL) {
        Serial.println("FATAL ERROR: Failed to allocate DSP buffers in PSRAM!");
        tft.fillScreen(TFT_RED);
        tft.drawString("MEMORY ERROR", tft.width() / 2, tft.height() / 2);
        while(1) { 
            delay(100); 
        } 
    }
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float)); 
    memset(fbDelayBuffer, 0, 8192 * sizeof(float));
    memset(freezeBuffer, 0, MAX_BUFFER_SIZE * sizeof(float)); 
    
    for (int i = 0; i < HANN_LUT_SIZE; i++) {
        hannLUT[i] = sinf(PI * ((float)i / (float)(HANN_LUT_SIZE - 1)));
    }
    
    for (int i = 0; i < LFO_LUT_SIZE; i++) {
        lfoLUT[i] = powf(2.0f, (15.0f * sinf(TWO_PI * ((float)i / (float)LFO_LUT_SIZE))) / 1200.0f);
    }
    
    for (int i = 0; i < WAVE_LUT_SIZE; i++) {
        float in = ((float)i - (WAVE_LUT_SIZE / 2.0f)) / (WAVE_LUT_SIZE / 2.0f); 
        synthLUT[i] = sinf(in * 45.0f); 
    }
        
    FilteredAnalog<>::setupADC(); 
    calibrateCenterAndDeadzone(); 
    updateLUT(); 
    
    Control_Surface >> pipes >> btmidi; 
    Control_Surface >> pipes >> usbmidi;
    usbmidi >> pipes >> Control_Surface; 
    btmidi >> pipes >> Control_Surface;
    
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr); 
    Control_Surface.begin();
    
    i2s_chan_config_t c = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); 
    c.dma_desc_num = 6; 
    c.dma_frame_num = 256; 
    c.auto_clear = true; 
    i2s_new_channel(&c, &tx_chan, &rx_chan);
    
        i2s_std_config_t s = { 
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_FREQUENCY), 
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg = { 
            .mclk = GPIO_NUM_3,
            .bclk = GPIO_NUM_10, 
            .ws = GPIO_NUM_11, 
            .dout = GPIO_NUM_16, 
            .din = GPIO_NUM_17 
        } 
    };
    
    s.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH; 
    
    i2s_channel_init_std_mode(tx_chan, &s); 
    i2s_channel_init_std_mode(rx_chan, &s);
    i2s_channel_enable(tx_chan); 
    i2s_channel_enable(rx_chan);
    
    xTaskCreatePinnedToCore(DisplayTask, "UI", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);
}

void loop() {
    // Empty
}