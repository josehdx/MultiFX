#pragma GCC optimize ("O3, tree-vectorize")
#include <Arduino.h>
#include <Control_Surface.h>
#include <TFT_eSPI.h>
#include <driver/i2s_std.h> 
#include "driver/gpio.h" 
#include "freertos/FreeRTOS.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_biquad.h"
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

// --- DEDICATED INDEPENDENT TAP STATES ---
float tap_w1_1 = 0.0f;
float tap_w1_2 = 256.0f; 

float tap_w2_1 = 0.0f;
float tap_w2_2 = 256.0f; 

float tap_w3_1 = 0.0f;
float tap_w3_2 = 256.0f; 

float tap_w4_1 = 0.0f;
float tap_w4_2 = 256.0f; 

float tap_w5_1 = 0.0f;
float tap_w5_2 = 256.0f; 

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
volatile bool isVolumeMode = false; 

volatile float chorusLfoPhase = 0.0f;
volatile float feedbackLfoPhase = 0.0f;
volatile float vibratoLfoPhase = 0.0f;
volatile float swellGain = 0.0f; 
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

volatile bool sleepRequested = false;
volatile bool isSleeping = false;

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

// --- DUAL PB CALIBRATION VARIABLES ---
double PBdeadzoneMultiplier = 14;
double PBdeadzoneMinimum = 950;
double PBdeadzoneMaximum = 1600;
analog_t PBminimumValue = 0;
analog_t PBmaximumValue = 16383;

// PB1 Calibration
analog_t PBcenter1 = 8192;
analog_t PBdeadzone1 = PBdeadzoneMinimum;
bool PBwasOffCenter1 = false;

// PB2 Calibration
analog_t PBcenter2 = 8192;
analog_t PBdeadzone2 = PBdeadzoneMinimum;
bool PBwasOffCenter2 = false;

FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB2 = pinPB2;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB3 = pinPB3;
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;

// --- PB DEADZONE MAPPING FUNCTION ---
analog_t map_PB_deadzone(analog_t raw, analog_t center, analog_t deadzone, bool &offCenterFlag) {
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    if (raw <= PBminimumValue + 150) { 
        offCenterFlag = true; 
        return 0; 
    }
    
    if (raw >= PBmaximumValue - 150) { 
        offCenterFlag = true; 
        return 16383; 
    }
    
    int r = (int)raw; 
    int c = (int)center; 
    int d = (int)deadzone;
    
    if (r <= c - d) { 
        offCenterFlag = true; 
        return map(r, (int)PBminimumValue, c - d, 0, 8191); 
    }
    else if (r >= c + d) { 
        offCenterFlag = true; 
        return map(r, c + d, (int)PBmaximumValue, 8191, 16383); 
    }
    else { 
        offCenterFlag = false; 
        return 8192; 
    }
}

void calibratePBs() {
    Serial.println("Calibrating Centers and Deadzones for PB1 and PB2...");
    
    for(int i = 0; i < 50; i++) { 
        filterPB.update(); 
        filterPB2.update(); 
        delay(1); 
    }
  
    int iSamples = 750;
    
    analog_t low1 = 16383;
    analog_t high1 = 0;
    
    analog_t low2 = 16383;
    analog_t high2 = 0;
    
    long sum1 = 0;
    long sum2 = 0;

    for (int i = 1; i <= iSamples; i++) {
        filterPB.update(); 
        filterPB2.update();
        
        analog_t raw1 = map(filterPB.getValue(), 0, 4095, 0, 16383);
        analog_t raw2 = map(filterPB2.getValue(), 0, 4095, 0, 16383);
        
        sum1 += raw1; 
        sum2 += raw2;
        
        if (raw1 < low1) {
            low1 = raw1;
        }
        
        if (raw1 > high1) {
            high1 = raw1;
        }
        
        if (raw2 < low2) {
            low2 = raw2;
        }
        
        if (raw2 > high2) {
            high2 = raw2;
        }
        
        delay(1);
    }
  
    PBcenter1 = sum1 / iSamples;
    if (PBcenter1 < 2000 || PBcenter1 > 14000) {
        PBcenter1 = 8192;
    }
    
    PBdeadzone1 = (analog_t)constrain(((high1 - low1) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);

    PBcenter2 = sum2 / iSamples;
    if (PBcenter2 < 2000 || PBcenter2 > 14000) {
        PBcenter2 = 8192;
    }
    
    PBdeadzone2 = (analog_t)constrain(((high2 - low2) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
}

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

// --- OPTIMIZATION B: HORNER'S METHOD HERMITE INTERPOLATION ---
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
    float c3 = 1.5f * (y1 - y2) + 0.5f * (y3 - y0);
    float c2 = y0 - y1 + c1 - c3;
    
    return ((c3 * frac + c2) * frac + c1) * frac + c0;
}

void updateLUT() {
    float basePitch = 0.0f; 
    
    if (isCapoMode || (activeEffectMode == 4 && isWhammyActive)) {
        basePitch += effectMemory[4]; 
    }
    
    float toeBend = effectMemory[0]; 
    float heelBend = effectMemory[5];
    
    for (int i = 0; i < 16384; i++) {
        float normalizedThrow;
        
        if (i >= 8192) {
            normalizedThrow = ((float)(i - 8192) / 8191.0f);
        } else {
            normalizedThrow = ((float)(i - 8192) / 8192.0f);
        }
        
        float dynamicBend;
        
        if (normalizedThrow >= 0.0f) {
            dynamicBend = toeBend * normalizedThrow;
        } else {
            dynamicBend = heelBend * std::abs(normalizedThrow);
        }
        
        float totalShift = basePitch + dynamicBend; 
        pitchShiftLUT[i] = powf(2.0f, totalShift / 12.0f);
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

    // I/O METERS
    int barWidth = 8;
    int barHeight = 100;
    int barY = 30;
    int inX = 10;
    int outX = spr.width() - 18;
    
    spr.drawRect(inX, barY, barWidth, barHeight, TFT_DARKGREY); 
    spr.setTextColor(TFT_WHITE, TFT_BLACK); 
    spr.drawString("IN", inX + (barWidth / 2), barY + barHeight + 10);
    
    spr.drawRect(outX, barY, barWidth, barHeight, TFT_DARKGREY); 
    spr.drawString("OUT", outX + (barWidth / 2), barY + barHeight + 10);

    // LED 
    bool isEffectOn = false; 
    
    if (activeEffectMode == 0) {
        isEffectOn = isWhammyActive;
    } else if (activeEffectMode == 1) {
        isEffectOn = (isWhammyActive || isFrozen);
    } else if (activeEffectMode == 2) {
        isEffectOn = (isWhammyActive || isFeedbackActive);
    } else if (activeEffectMode == 3) {
        isEffectOn = (isWhammyActive || isHarmonizerMode);
    } else if (activeEffectMode == 4) {
        isEffectOn = (isWhammyActive || isCapoMode);
    } else if (activeEffectMode == 5) {
        isEffectOn = (isWhammyActive || isSynthMode);
    } else if (activeEffectMode == 6) {
        isEffectOn = (isWhammyActive || isPadMode);
    } else if (activeEffectMode == 7) {
        isEffectOn = (isWhammyActive || isChorusMode);
    } else if (activeEffectMode == 8) {
        isEffectOn = (isWhammyActive || isSwellMode); 
    } else if (activeEffectMode == 9) {
        isEffectOn = (isWhammyActive || isVibratoMode);
    }
    
    uint32_t ledColor = TFT_RED;
    
    if (isEffectOn) {
        ledColor = TFT_GREEN;
    }
    
    spr.fillCircle(spr.width() - 12, 12, 6, ledColor); 
    spr.drawCircle(spr.width() - 12, 12, 6, TFT_WHITE);

    // TITLE ANCHOR
    spr.setTextSize(3); 
    int titleX = 215;
    int titleY = 30;
    
    switch(activeEffectMode) {
        case 0: 
            spr.setTextColor(TFT_ORANGE, TFT_BLACK); 
            spr.drawString("WHAMMY", titleX, titleY); 
            break;
        case 1: 
            spr.setTextColor(TFT_CYAN, TFT_BLACK); 
            spr.drawString("FREEZE", titleX, titleY); 
            break;
        case 2: 
            spr.setTextColor(TFT_RED, TFT_BLACK); 
            spr.drawString("FEEDBACK", titleX, titleY); 
            break;
        case 3: 
            spr.setTextColor(TFT_MAGENTA, TFT_BLACK); 
            spr.drawString("HARMONY", titleX, titleY); 
            break;
        case 4: 
            spr.setTextColor(TFT_GREEN, TFT_BLACK); 
            spr.drawString("CAPO", titleX, titleY); 
            break;
        case 5: 
            spr.setTextColor(TFT_YELLOW, TFT_BLACK); 
            spr.drawString("SYNTH", titleX, titleY); 
            break;
        case 6: 
            spr.setTextColor(TFT_PINK, TFT_BLACK); 
            spr.drawString("PAD", titleX, titleY); 
            break;
        case 7: 
            spr.setTextColor(TFT_SKYBLUE, TFT_BLACK); 
            spr.drawString("CHORUS", titleX, titleY); 
            break;
        case 8: 
            spr.setTextColor(TFT_WHITE, TFT_BLACK); 
            spr.drawString("SWELL", titleX, titleY); 
            break;
        case 9: 
            spr.setTextColor(TFT_PURPLE, TFT_BLACK); 
            spr.drawString("VIBRATO", titleX, titleY); 
            break;
    }

    // PB & CC11 BARS
    spr.setTextColor(TFT_WHITE, TFT_BLACK); 
    spr.setTextSize(1); 
    
    int lineTop = 30;
    int lineBot = 125;
    int x1 = 40;
    int x2 = 75;
    int x3 = 110;
    int x4 = 145;
    
    spr.drawString("PB1", x1, lineBot + 15); 
    spr.drawString("PB2", x2, lineBot + 15); 
    spr.drawString("PB3", x3, lineBot + 15); 
    spr.drawString("CC11", x4, lineBot + 15);
    
    for (int y = lineTop; y <= lineBot; y += 5) { 
        spr.drawFastVLine(x1, y, 2, TFT_DARKGREY); 
        spr.drawFastVLine(x2, y, 2, TFT_DARKGREY); 
        spr.drawFastVLine(x3, y, 2, TFT_DARKGREY); 
        spr.drawFastVLine(x4, y, 2, TFT_DARKGREY); 
    }
    
    spr.fillCircle(x1, map(currentPB1, 0, 16383, lineBot, lineTop), 4, TFT_CYAN); 
    spr.fillCircle(x2, map(currentPB2, 0, 16383, lineBot, lineTop), 4, TFT_MAGENTA);
    spr.fillCircle(x3, map(currentPB3, 0, 16383, lineBot, lineTop), 4, TFT_YELLOW); 
    spr.fillCircle(x4, map(currentCC11, 0, 16383, lineBot, lineTop), 4, TFT_GREEN);

    // INTERVALS
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    
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
        
        spr.drawString(topStr, titleX, 60); 
        spr.drawString(botStr, titleX, 85);
    } else { 
        spr.setTextSize(4); 
        char intervalStr[16]; 
        float val = effectMemory[activeEffectMode];
        
        if (activeEffectMode == 2) { 
            float fbI[5] = { 0.0f, 12.0f, 19.0f, 24.0f, 28.0f }; 
            val = fbI[feedbackIntervalIdx % 5]; 
        } 
        
        if (val > 0) {
            sprintf(intervalStr, "+%.1f", val);
        } else {
            sprintf(intervalStr, "%.1f", val);
        }
        
        spr.drawString(intervalStr, titleX, 75);
    }

    // MULTI-EFFECT BANNERS
    spr.setTextSize(2); 
    int bannerCount = 0;
    
    auto drawBanner = [&](const char* text, uint32_t color) {
        int col = bannerCount % 2;
        int row = bannerCount / 2;
        int bx = 185 + (col * 70);
        int by = 110 + (row * 15);
        
        spr.setTextColor(color, TFT_BLACK); 
        spr.drawString(text, bx, by); 
        bannerCount++;
    };
    
    if (isFrozen && activeEffectMode != 1) {
        drawBanner("FROZEN", TFT_CYAN);
    }
    
    if (isFeedbackActive && activeEffectMode != 2) {
        drawBanner("SCREAM", TFT_RED);
    }
    
    if (isHarmonizerMode && activeEffectMode != 3) {
        drawBanner("HARM", TFT_MAGENTA);
    }
    
    if (isCapoMode && activeEffectMode != 4) {
        drawBanner("CAPO", TFT_GREEN);
    }
    
    if (isSynthMode && activeEffectMode != 5) {
        drawBanner("SYNTH", TFT_YELLOW);
    }
    
    if (isPadMode && activeEffectMode != 6) {
        drawBanner("PAD", TFT_PINK);
    }
    
    if (isChorusMode && activeEffectMode != 7) {
        drawBanner("CHORUS", TFT_SKYBLUE);
    }
    
    if (isSwellMode && activeEffectMode != 8) {
        drawBanner("SWELL", TFT_WHITE); 
    }
    
    if (isVibratoMode && activeEffectMode != 9) {
        drawBanner("VIB", TFT_PURPLE); 
    }
    
    if (isVolumeMode) {
        drawBanner("VOLUME", TFT_DARKGREY);
    }

    // --- SYSTEM STATS (CPU & MEMORY) ---
    spr.setTextSize(1); 
    spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK); 
    
    char cpuStr[16]; 
    sprintf(cpuStr, "CPU:%2d%%", (int)core1_load); 
    spr.drawString(cpuStr, titleX, 115);
    
    char sramStr[16];
    sprintf(sramStr, "SRM:%dK", heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    spr.drawString(sramStr, titleX, 127);
    
    char psramStr[16];
    sprintf(psramStr, "PSR:%dK", heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
    spr.drawString(psramStr, titleX, 139);

    // LATENCY LABEL
    spr.setTextColor(TFT_WHITE); 
    int latY = 158;
    
    spr.drawRect(titleX - 25, latY - 8, 50, 16, TFT_DARKGREY);
    const char* latLabels[] = {"U.Low", "Low", "Mid", "High"}; 
    spr.drawString(latLabels[latencyMode], titleX, latY);
    
    spr.pushSprite(0, 0); 
    updateMeters();
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

// --- OPTIMIZATION A & D: BLOCK PROCESSING AUDIO DSP TASK ---
void IRAM_ATTR AudioDSPTask(void * pvParameters) {
    // I/O Buffers
    static float dsp_out[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_in[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_out[HOP_SIZE * 2] __attribute__((aligned(16)));
    
    // Intermediate Vector Arrays for Block Processing
    static float in_block[HOP_SIZE] __attribute__((aligned(16)));
    static float dc_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w1_block[HOP_SIZE] __attribute__((aligned(16)));
    static float mix_block[HOP_SIZE] __attribute__((aligned(16)));
    
    // Hardware Biquad DC Blocker (Optimization D)
    static float dc_coeffs[5] = {1.0f, -1.0f, 0.0f, -0.995f, 0.0f}; 
    static float dc_state[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    static float synthEnv = 0.0f;
    static float synthFilter = 0.0f;
    static float padFilter = 0.0f;
    static float padEnv = 0.0f;
    static float inputEnvelope = 0.0f;
    
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
                tap_w1_1 = 0.0f; 
                tap_w1_2 = targetWin / 2.0f; 
                tap_w2_1 = 0.0f; 
                tap_w2_2 = targetWin / 2.0f; 
                tap_w3_1 = 0.0f; 
                tap_w3_2 = targetWin / 2.0f; 
                tap_w4_1 = 0.0f; 
                tap_w4_2 = targetWin / 2.0f; 
                tap_w5_1 = 0.0f; 
                tap_w5_2 = targetWin / 2.0f; 
            }
            
            float hannMultiplier = 1023.0f / currentWindowSize;
            float invFreezeLength = 1.0f / 48000.0f;
            float chorusPhaseInc = 1536.0f / 96000.0f;
            float feedbackPhaseInc = 5120.0f / 96000.0f; 
            
            float vibHz = 2.0f;
            if (effectMemory[9] != 0.0f) {
                vibHz = fabsf(effectMemory[9]);
            }
            
            float vibratoPhaseInc = (vibHz * LFO_LUT_SIZE) / SAMPLING_FREQUENCY;
            
            bool frz = ((activeEffectMode == 1 && isWhammyActive) || isFrozen);
            if (frz && !wasFrozen) { 
                freezePlayCounter = 0; 
                freezeStartIdx = freezeWriteIdx; 
            } 
            wasFrozen = frz;
            
            bool synth = ((activeEffectMode == 5 && isWhammyActive) || isSynthMode);
            bool pad = ((activeEffectMode == 6 && isWhammyActive) || isPadMode);
            bool harm = ((activeEffectMode == 3 && isWhammyActive) || isHarmonizerMode);
            bool swell = ((activeEffectMode == 8 && isWhammyActive) || isSwellMode);
            bool chorus = ((activeEffectMode == 7 && isWhammyActive) || isChorusMode);
            bool feedback = ((activeEffectMode == 2 && isWhammyActive) || isFeedbackActive);
            bool vibrato = ((activeEffectMode == 9 && isWhammyActive) || isVibratoMode);
            bool capo = ((activeEffectMode == 4 && isWhammyActive) || isCapoMode);
            
            float harmRatio = powf(2.0f, effectMemory[3] / 12.0f);
            float chorusRatio = powf(2.0f, effectMemory[8] / 12.0f);
            
            float fbI[5] = { 0.0f, 12.0f, 19.0f, 24.0f, 28.0f }; 
            float feedbackHarmonicRatio = powf(2.0f, fbI[feedbackIntervalIdx % 5] / 12.0f);
            
            float pIn = 0.0f;
            float pOut = 0.0f;

            // --- STAGE 1: GATHER & FILTER (VECTORIZED) ---
            #pragma GCC ivdep
            for(int i = 0; i < HOP_SIZE; i++) {
                in_block[i] = (float)i2s_in[i * 2] * norm;
            }
            
            // High-Speed Hardware Biquad DC Blocker
            dsps_biquad_f32(in_block, dc_block, HOP_SIZE, dc_coeffs, dc_state);

            // --- STAGE 2: COMPLEX SCALAR OPERATIONS ---
            for (int i = 0; i < HOP_SIZE; i++) {
                float input = dc_block[i]; 
                inputEnvelope = inputEnvelope * 0.99f + fabsf(input) * 0.01f;
                
                if (swell) { 
                    if (inputEnvelope > 0.015f) {
                        swellGain = fminf(1.0f, swellGain + 0.00002f); 
                    } else {
                        swellGain = fmaxf(0.0f, swellGain - 0.00005f); 
                    }
                } else { 
                    swellGain = 1.0f; 
                }
                
                float writeVal = input;
                
                if (synth) { 
                    if (inputEnvelope > 0.005f) {
                        synthEnv = fminf(1.0f, synthEnv + 0.1f); 
                    } else {
                        synthEnv = fmaxf(0.0f, synthEnv - 0.005f); 
                    }
                    
                    int lutIdx = (int)((writeVal + 1.0f) * 0.5f * (WAVE_LUT_SIZE - 1)); 
                    if (lutIdx < 0) {
                        lutIdx = 0; 
                    } else if (lutIdx >= WAVE_LUT_SIZE) {
                        lutIdx = WAVE_LUT_SIZE - 1; 
                    }
                    
                    writeVal = synthLUT[lutIdx]; 
                    synthFilter = synthFilter + (0.3f + 0.8f * synthEnv) * (writeVal - synthFilter); 
                    writeVal = synthFilter * 0.1f; 
                } 
                
                if (pad) { 
                    if (inputEnvelope > 0.005f) {
                        padEnv = fminf(1.0f, padEnv + 0.00002f); 
                    } else {
                        padEnv = fmaxf(0.0f, padEnv - 0.000005f); 
                    }
                    writeVal *= padEnv; 
                }
                
                if (!frz) { 
                    freezeBuffer[freezeWriteIdx] = writeVal; 
                    freezeWriteIdx++; 
                    if (freezeWriteIdx >= freezeLength) {
                        freezeWriteIdx = 0; 
                    }
                }
                
                if (freezeRamp > 0.0f || frz) { 
                    if (frz) {
                        freezeRamp = fminf(1.0f, freezeRamp + 0.0002f); 
                    } else {
                        freezeRamp = fmaxf(0.0f, freezeRamp - 0.00005f); 
                    }
                }
                
                float frzOut = 0.0f; 
                if (freezeRamp > 0.0f) { 
                    float ph = (float)freezePlayCounter * invFreezeLength; 
                    
                    int i1 = freezeStartIdx + freezePlayCounter;
                    while (i1 >= freezeLength) {
                        i1 -= freezeLength;
                    }
                    
                    int i2 = freezeStartIdx + freezePlayCounter + (freezeLength / 2);
                    while (i2 >= freezeLength) {
                        i2 -= freezeLength;
                    }
                    
                    float ph2;
                    if ((ph + 0.5f) >= 1.0f) {
                        ph2 = ph - 0.5f;
                    } else {
                        ph2 = ph + 0.5f;
                    }
                    
                    float raw = (freezeBuffer[i1] * hannLUT[(int)(ph * 1023.0f)]) + (freezeBuffer[i2] * hannLUT[(int)(ph2 * 1023.0f)]); 
                    
                    float d1 = apf1Buffer[apf1Idx]; 
                    float a1 = -0.6f * raw + d1; 
                    apf1Buffer[apf1Idx] = raw + 0.6f * d1; 
                    
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
                    
                    frzOut = a2 * freezeRamp; 
                    
                    freezePlayCounter++;
                    if (freezePlayCounter >= freezeLength) {
                        freezePlayCounter = 0;
                    }
                }
                
                float fv = writeVal; 
                if (frz && freezeRamp > 0.0f) {
                    fv = frzOut; 
                }
                delayBuffer[writeIndex] = fmaxf(-1.0f, fminf(fv, 1.0f));
                
                float f_w1 = pitchShiftFactor; 
                if (vibrato) { 
                    vibratoLfoPhase += vibratoPhaseInc;
                    while (vibratoLfoPhase >= LFO_LUT_SIZE) {
                        vibratoLfoPhase -= LFO_LUT_SIZE;
                    }
                    f_w1 = pitchShiftFactor * lfoLUT[(int)vibratoLfoPhase]; 
                }
                
                float f_w2 = pitchShiftFactor * harmRatio;
                float f_w3 = pitchShiftFactor * chorusRatio; 
                if (chorus) { 
                    chorusLfoPhase += chorusPhaseInc;
                    while (chorusLfoPhase >= LFO_LUT_SIZE) {
                        chorusLfoPhase -= LFO_LUT_SIZE;
                    }
                    f_w3 *= lfoLUT[(int)chorusLfoPhase]; 
                }
                
                float f_w4 = 1.0f;
                float f_w5 = 1.0f; 
                if (feedback || feedbackRamp > 0.0f) { 
                    feedbackLfoPhase += feedbackPhaseInc;
                    while (feedbackLfoPhase >= LFO_LUT_SIZE) {
                        feedbackLfoPhase -= LFO_LUT_SIZE;
                    }
                    float d = lfoLUT[(int)feedbackLfoPhase]; 
                    f_w4 = d; 
                    f_w5 = pitchShiftFactor * feedbackHarmonicRatio * d; 
                }
                
                float w1 = (getHermiteSample(tap_w1_1 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w1_1 * hannMultiplier)]) + 
                           (getHermiteSample(tap_w1_2 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w1_2 * hannMultiplier)]);
                
                w1_block[i] = w1; 

                float w2 = 0.0f;
                float w3 = 0.0f;
                float w4 = 0.0f;
                float w5 = 0.0f;
                
                if (harm) {
                    w2 = (getHermiteSample(tap_w2_1 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w2_1 * hannMultiplier)]) + 
                         (getHermiteSample(tap_w2_2 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w2_2 * hannMultiplier)]); 
                }
                
                if (chorus) {
                    w3 = (getHermiteSample(tap_w3_1 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w3_1 * hannMultiplier)]) + 
                         (getHermiteSample(tap_w3_2 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w3_2 * hannMultiplier)]); 
                }
                
                if (feedback || feedbackRamp > 0.0f) { 
                    w4 = (getHermiteSample(tap_w4_1 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w4_1 * hannMultiplier)]) + 
                         (getHermiteSample(tap_w4_2 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w4_2 * hannMultiplier)]); 
                    w5 = (getHermiteSample(tap_w5_1 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w5_1 * hannMultiplier)]) + 
                         (getHermiteSample(tap_w5_2 + 2.0f, delayBuffer, writeIndex) * hannLUT[(int)(tap_w5_2 * hannMultiplier)]); 
                }
                
                // Fast Phase Wraps replacing fmodf()
                float r1 = 1.0f - f_w1; 
                tap_w1_1 += r1; 
                while (tap_w1_1 >= currentWindowSize) tap_w1_1 -= currentWindowSize; 
                while (tap_w1_1 < 0.0f) tap_w1_1 += currentWindowSize;
                
                tap_w1_2 += r1; 
                while (tap_w1_2 >= currentWindowSize) tap_w1_2 -= currentWindowSize; 
                while (tap_w1_2 < 0.0f) tap_w1_2 += currentWindowSize;
                
                float r2 = 1.0f - f_w2; 
                tap_w2_1 += r2; 
                while (tap_w2_1 >= currentWindowSize) tap_w2_1 -= currentWindowSize; 
                while (tap_w2_1 < 0.0f) tap_w2_1 += currentWindowSize;
                
                tap_w2_2 += r2; 
                while (tap_w2_2 >= currentWindowSize) tap_w2_2 -= currentWindowSize; 
                while (tap_w2_2 < 0.0f) tap_w2_2 += currentWindowSize;
                
                float r3 = 1.0f - f_w3; 
                tap_w3_1 += r3; 
                while (tap_w3_1 >= currentWindowSize) tap_w3_1 -= currentWindowSize; 
                while (tap_w3_1 < 0.0f) tap_w3_1 += currentWindowSize;
                
                tap_w3_2 += r3; 
                while (tap_w3_2 >= currentWindowSize) tap_w3_2 -= currentWindowSize; 
                while (tap_w3_2 < 0.0f) tap_w3_2 += currentWindowSize;
                
                float r4 = 1.0f - f_w4; 
                tap_w4_1 += r4; 
                while (tap_w4_1 >= currentWindowSize) tap_w4_1 -= currentWindowSize; 
                while (tap_w4_1 < 0.0f) tap_w4_1 += currentWindowSize;
                
                tap_w4_2 += r4; 
                while (tap_w4_2 >= currentWindowSize) tap_w4_2 -= currentWindowSize; 
                while (tap_w4_2 < 0.0f) tap_w4_2 += currentWindowSize;
                
                float r5 = 1.0f - f_w5; 
                tap_w5_1 += r5; 
                while (tap_w5_1 >= currentWindowSize) tap_w5_1 -= currentWindowSize; 
                while (tap_w5_1 < 0.0f) tap_w5_1 += currentWindowSize;
                
                tap_w5_2 += r5; 
                while (tap_w5_2 >= currentWindowSize) tap_w5_2 -= currentWindowSize; 
                while (tap_w5_2 < 0.0f) tap_w5_2 += currentWindowSize;
                
                writeIndex = (writeIndex + 1) & BUFFER_MASK;
                
                float fbOut = 0.0f; 
                if (feedback || feedbackRamp > 0.0f) { 
                    if (feedback) {
                        float rampDelta;
                        if (inputEnvelope > 0.005f) {
                            rampDelta = 0.000011f;
                        } else {
                            rampDelta = -0.005f;
                        }
                        feedbackRamp = fminf(1.0f, feedbackRamp + rampDelta); 
                    } else {
                        feedbackRamp = fmaxf(0.0f, feedbackRamp - 0.0001f); 
                    }
                    
                    float bl = fmaxf(0.0f, fminf((feedbackRamp - 0.1f) * 2.0f, 1.0f)); 
                    float bld;
                    
                    if (frz && freezeRamp > 0.0f) {
                        bld = frzOut;
                    } else {
                        bld = (w4 * (1.0f - bl)) + (w5 * bl);
                    }
                    
                    fbHpfState += 0.05f * (bld - fbHpfState); 
                    float drv = constrain((bld - fbHpfState) * 30.0f, -1.0f, 1.0f); 
                    feedbackFilter = feedbackFilter * 0.9f + drv * 0.1f; 
                    float rfb = feedbackFilter * (feedbackRamp * feedbackRamp * feedbackRamp) * 0.85f; 
                    
                    fbDelayBuffer[fbDelayWriteIdx] = rfb; 
                    
                    int readIdx = (fbDelayWriteIdx - (int)(SAMPLING_FREQUENCY * 0.02f) + 8192) & 8191;
                    fbOut = fbDelayBuffer[readIdx]; 
                    
                    fbDelayWriteIdx = (fbDelayWriteIdx + 1) & 8191; 
                }
                
                if (pad) {
                    padFilter = padFilter * 0.95f + w1 * 0.05f; 
                } else {
                    padFilter = padFilter * 0.95f;
                }
                
                bool any = isWhammyActive || harm || chorus || feedback || synth || pad || frz || vibrato || capo;
                bool dry = chorus || pad || frz || feedback || (freezeRamp > 0.0f) || (feedbackRamp > 0.0f);
                bool rep = capo || synth || vibrato || pad || harm;
                
                float mix = 0.0f; 
                if (!any && freezeRamp <= 0.0f && feedbackRamp <= 0.0f && padFilter < 0.001f) { 
                    mix = input; 
                } else { 
                    if (dry) { 
                        if (!rep) {
                            mix += (input * 0.4f); 
                        }
                    } else if (harm) { 
                        mix += (w1 * 0.5f); 
                    } 
                    
                    if (harm) {
                        mix += (w2 * 0.5f); 
                    }
                    if (chorus) {
                        mix += (w3 * 0.4f); 
                    }
                    if (pad || padFilter > 0.001f) {
                        mix += (padFilter * 1.5f); 
                    }
                    if (!frz && freezeRamp > 0.0f) {
                        mix += (frzOut * 0.5f); 
                    }
                    if (feedback || feedbackRamp > 0.0f) {
                        mix += (fbOut * 0.6f); 
                    }
                    
                    mix = mix * (1.0f - (0.1f * mix * mix)); 
                }
                mix_block[i] = mix;
            }

            // --- STAGE 3: VECTORIZED MULTIPLY-ACCUMULATE MIXING ---
            bool dry = chorus || pad || frz || feedback || (freezeRamp > 0.0f) || (feedbackRamp > 0.0f);
            bool isAnyEffectActive = isWhammyActive || harm || chorus || feedback || synth || pad || frz || vibrato || capo;
            
            #pragma GCC ivdep
            for(int i = 0; i < HOP_SIZE; i++) {
                if (isAnyEffectActive || freezeRamp > 0.0f || feedbackRamp > 0.0f || padFilter > 0.001f) {
                    if (dry) {
                        mix_block[i] += (w1_block[i] * 0.4f);
                    } else if (!harm) {
                        mix_block[i] += w1_block[i];
                    }
                }
                
                float fo = mix_block[i] * swellGain * volumePedalGain; 
                dsp_out[i * 2] = fo; 
                dsp_out[(i * 2) + 1] = fo; 
                
                if (fabsf(dc_block[i]) > pIn) {
                    pIn = fabsf(dc_block[i]); 
                }
                if (fabsf(fo) > pOut) {
                    pOut = fabsf(fo);
                }
            }
            
            uint32_t end_cycles = xthal_get_ccount(); 
            float current_load_calc = ((float)(end_cycles - start_cycles) / 160000.0f) * 100.0f; 
            core1_load = core1_load * 0.95f + fminf(100.0f, current_load_calc) * 0.05f;
            
            float sc = 2147483647.0f; 
            dsps_mul_f32(dsp_out, &sc, dsp_out, HOP_SIZE * 2, 1, 0, 1);
            
            #pragma GCC ivdep
            for(int i = 0; i < HOP_SIZE * 2; i++) {
                i2s_out[i] = (int32_t)fmaxf(-2147483648.0f, fminf(dsp_out[i], 2147483647.0f));
            }
            
            if (pIn > ui_audio_level) {
                ui_audio_level = pIn;
            } else {
                ui_audio_level = fmaxf(0.0f, ui_audio_level * 0.998f);
            }
            
            if (pOut > ui_output_level) {
                ui_output_level = pOut;
            } else {
                ui_output_level = fmaxf(0.0f, ui_output_level * 0.998f);
            }
            
            size_t bw; 
            i2s_channel_write(tx_chan, i2s_out, sizeof(i2s_out), &bw, portMAX_DELAY);
        }
    }
}

void MidiTask(void * pvParameters) {
    static analog_t lastMidiA = 8192;
    static analog_t lastMidiB = 8192;
    static analog_t lastMidiC = 8192; 
    
    static bool lastBtState = false; 
    static uint8_t lastVolumeCC = 127;
    
    static float smoothRawA = -1.0f;
    static float smoothRawB = -1.0f;
    static float smoothRawC = -1.0f;
    
    static DebouncedButton btnCar(CAROUSEL_BUTTON_PIN); 
    btnCar.state = digitalRead(CAROUSEL_BUTTON_PIN); 
    btnCar.lastReading = btnCar.state;
    
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
                    isWhammyActive = true; 
                    updateLUT(); 
                } 
                forceUIUpdate = true; 
            } 
        }
        
        filterPB.update(); 
        filterPB2.update(); 
        filterPB3.update(); 
        
        if (digitalRead(BOOT_SENSE_PIN) == HIGH) {
            
            analog_t raw12_A = filterPB.getValue();
            analog_t raw12_B = filterPB2.getValue();
            analog_t raw12_C = filterPB3.getValue();
            
            if (smoothRawA < 0) {
                smoothRawA = raw12_A;
            }
            smoothRawA = smoothRawA * 0.5f + (float)raw12_A * 0.5f; 
            
            if (smoothRawB < 0) {
                smoothRawB = raw12_B;
            }
            smoothRawB = smoothRawB * 0.5f + (float)raw12_B * 0.5f; 
            
            if (smoothRawC < 0) {
                smoothRawC = raw12_C;
            }
            smoothRawC = smoothRawC * 0.5f + (float)raw12_C * 0.5f; 
            
            analog_t mappedRawA = map((int)smoothRawA, 0, 4095, 0, 16383);
            analog_t calA = map_PB_deadzone(mappedRawA, PBcenter1, PBdeadzone1, PBwasOffCenter1);
            
            analog_t mappedRawB = map((int)smoothRawB, 0, 4095, 0, 16383);
            analog_t calB = map_PB_deadzone(mappedRawB, PBcenter2, PBdeadzone2, PBwasOffCenter2);
            
            int constrainedRawC = constrain((int)smoothRawC, 40, 4055);
            analog_t calC = map(constrainedRawC, 40, 4055, 0, 16383);
            
            if (calC < 150) {
                calC = 0; 
            }
            if (calC > 16233) {
                calC = 16383;
            }
            
            // --- DISABLED FOR BENCH TESTING ---
            bool movedA = false; 
            bool movedB = false; 
            bool movedC = false;

            if (movedA || movedB || movedC) {
                if (isScreenOff) {
                    turnScreenOn(); 
                }
                
                lastScreenActivityTime = millis();
                
                if (movedA) { 
                    if (!isVolumeMode) {
                        Control_Surface.sendPitchBend(Channel_1, calA); 
                    }
                    lastMidiA = calA; 
                }
                
                if (movedB) { 
                    if (!isVolumeMode) {
                        Control_Surface.sendPitchBend(Channel_2, calB); 
                    }
                    lastMidiB = calB; 
                }
                
                if (movedC) { 
                    if (!isVolumeMode) {
                        Control_Surface.sendPitchBend(Channel_3, calC); 
                    }
                    lastMidiC = calC; 
                }
                
                analog_t act;
                if (movedC) {
                    act = calC;
                } else if (movedB) {
                    act = calB;
                } else {
                    act = calA;
                }
                
                if (isVolumeMode) { 
                    uint8_t cc = map(act, 0, 16383, 0, 127); 
                    if (cc != lastVolumeCC) { 
                        Control_Surface.sendControlChange({19, Channel_1}, cc); 
                        lastVolumeCC = cc; 
                    } 
                    volumePedalGain = (float)act / 16383.0f; 
                } else { 
                    pitchShiftFactor = pitchShiftLUT[constrain(act, 0, 16383)];
                }
                
                forceUIUpdate = true;
            }
            
            currentPB1 = calA;
            currentPB2 = calB;
            currentPB3 = calC;
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
                isVolumeMode = false; 
                volumePedalGain = 1.0f; 
            } 
            updateLUT(); 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 8) { 
            isFrozen = (cm.data2 >= 64); 
            if (activeEffectMode == 1) {
                isWhammyActive = isFrozen; 
            }
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
            if (activeEffectMode == 3) {
                isWhammyActive = isHarmonizerMode; 
            }
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 12) { 
            isCapoMode = (cm.data2 >= 64); 
            if (activeEffectMode == 4) {
                isWhammyActive = isCapoMode; 
            }
            updateLUT(); 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 13) { 
            isSynthMode = (cm.data2 >= 64); 
            if (activeEffectMode == 5) {
                isWhammyActive = isSynthMode; 
            }
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 14) { 
            isPadMode = (cm.data2 >= 64); 
            if (activeEffectMode == 6) {
                isWhammyActive = isPadMode; 
            }
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 15) { 
            isChorusMode = (cm.data2 >= 64); 
            if (activeEffectMode == 7) {
                isWhammyActive = isChorusMode; 
            }
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 16) { 
            isSwellMode = (cm.data2 >= 64); 
            if (activeEffectMode == 8) {
                isWhammyActive = isSwellMode; 
            }
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 21) { 
            isVibratoMode = (cm.data2 >= 64); 
            if (activeEffectMode == 9) {
                isWhammyActive = isVibratoMode; 
            }
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
                float newVal;
                if (cm.data2 < 64) {
                    newVal = effectMemory[4] + 1.0f;
                } else {
                    newVal = effectMemory[4] - 1.0f;
                }
                effectMemory[4] = constrain(roundf(newVal * 100.0f) / 100.0f, -24.0f, 24.0f); 
            } else if (activeEffectMode == 2) { 
                if (cm.data2 < 64) {
                    feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5;
                } else {
                    feedbackIntervalIdx = (feedbackIntervalIdx - 1 + 5) % 5;
                }
            } else { 
                int slot = activeEffectMode; 
                if (activeEffectMode == 5) {
                    slot = 6; 
                } else if (activeEffectMode == 6) {
                    slot = 7; 
                } else if (activeEffectMode == 7) {
                    slot = 8; 
                } else if (activeEffectMode == 9) {
                    slot = 9; 
                }
                
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
                float newVal;
                if (cm.data2 < 64) {
                    newVal = effectMemory[4] + 0.01f;
                } else {
                    newVal = effectMemory[4] - 0.01f;
                }
                effectMemory[4] = constrain(roundf(newVal * 100.0f) / 100.0f, -24.0f, 24.0f); 
            } else if (activeEffectMode == 2) { 
                if (cm.data2 < 64) {
                    feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5;
                } else {
                    feedbackIntervalIdx = (feedbackIntervalIdx - 1 + 5) % 5;
                }
            } else { 
                int slot = activeEffectMode; 
                if (activeEffectMode == 5) {
                    slot = 6; 
                } else if (activeEffectMode == 6) {
                    slot = 7; 
                } else if (activeEffectMode == 7) {
                    slot = 8; 
                } else if (activeEffectMode == 9) {
                    slot = 9; 
                }
                
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
            currentPB3 = m; 
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
    
    pinMode(pinPB, INPUT); 
    pinMode(pinPB2, INPUT); 
    pinMode(pinPB3, INPUT);
    
    // --- SRAM CHECK PRINTS ---
    Serial.printf("Total Free Internal SRAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    Serial.printf("Largest Free Internal Block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    Serial.printf("Total Free PSRAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    delayBuffer = (float*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    fbDelayBuffer = (float*)heap_caps_aligned_alloc(16, 8192 * sizeof(float), MALLOC_CAP_SPIRAM);
    freezeBuffer = (float*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    
    if (delayBuffer == NULL || fbDelayBuffer == NULL || freezeBuffer == NULL) { 
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
        synthLUT[i] = sinf((((float)i - (WAVE_LUT_SIZE / 2.0f)) / (WAVE_LUT_SIZE / 2.0f)) * 45.0f);
    }
    
    FilteredAnalog<>::setupADC(); 
    calibratePBs(); 
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
            .mclk = GPIO_NUM_13, 
            .bclk = GPIO_NUM_12, 
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

void loop() {}