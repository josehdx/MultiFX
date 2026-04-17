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

/// --- GLOBALS & I2S HANDLES ---
i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;
#define SAMPLING_FREQUENCY 96000 
#define HOP_SIZE 64            

// --- TIME-DOMAIN DSP BUFFERS ---
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF 
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 65536

float* delayBuffer = nullptr;    
float* fbDelayBuffer = nullptr;  
float* freezeBuffer = nullptr;   

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

float currentWindowSize = 1024.0f; 

// --- FREEZE STATE & ALL-PASS FILTERS ---
const int freezeLength = 48000; 
bool wasFrozen = false;
volatile float freezeRamp = 0.0f;

float apf1Buffer[1009] = { 0.0f }; 
int apf1Idx = 0;

float apf2Buffer[863] = { 0.0f };  
int apf2Idx = 0;

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

// --- GLOBAL BUFFER WIPE FLAG ---
volatile bool globalAudioResetRequested = false;

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

// --- CONTINUOUS AUTO-CALIBRATION BOUNDS ---
uint16_t PB1_raw_min = 1000;
uint16_t PB1_raw_max = 3000;
uint16_t PB1_raw_center = 2048;

uint16_t PB2_raw_min = 1000;
uint16_t PB2_raw_max = 3000;
uint16_t PB2_raw_center = 2048;

uint16_t PB3_raw_min = 1000;
uint16_t PB3_raw_max = 3000;

int deadzone_size = 250; // Physical deadzone buffer for self-centering pedals

FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB2 = pinPB2;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB3 = pinPB3;
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;

// --- DYNAMIC RAW DEADZONE MAPPING WITH DRIFT ANCHORS ---
// Operates exclusively on constrained integers to prevent Underflow Jump bug
analog_t map_raw_deadzone(float smoothRaw, uint16_t center, uint16_t rMin, uint16_t rMax, int dZone) {
    int raw = constrain((int)smoothRaw, rMin, rMax);
    
    // Massive 120-point lock zone to prevent floating/drifting at the edges
    int lockZone = 120;
    
    // Bottom Anchor Lock
    if (raw <= (rMin + lockZone)) {
        return 0;
    }
    
    // Top Anchor Lock
    if (raw >= (rMax - lockZone)) {
        return 16383;
    }
    
    // Smooth transition from the edge of the Anchor Lock to the Deadzone
    if (raw < (center - dZone)) {
        return map(raw, (rMin + lockZone), (center - dZone), 0, 8191);
    } else if (raw > (center + dZone)) {
        return map(raw, (center + dZone), (rMax - lockZone), 8191, 16383);
    } else {
        return 8192;
    }
}

void calibratePBs() {
    // Purge the initial power-on noise
    for (int i = 0; i < 50; i++) { 
        filterPB.update(); 
        filterPB2.update(); 
        filterPB3.update(); 
        delay(1); 
    }
    
    long sum1 = 0;
    long sum2 = 0;
    uint16_t min3 = 4095;
    uint16_t max3 = 0;
    
    // Boot Phase: Read the spring-loaded resting positions
    for (int i = 1; i <= 250; i++) {
        filterPB.update(); 
        filterPB2.update(); 
        filterPB3.update();
        
        uint16_t r1 = filterPB.getValue();
        uint16_t r2 = filterPB2.getValue();
        uint16_t r3 = filterPB3.getValue();
        
        sum1 += r1; 
        sum2 += r2;
        
        if (r3 < min3) {
            min3 = r3;
        }
        if (r3 > max3) {
            max3 = r3;
        }
        
        delay(1);
    }
    
    PB1_raw_center = sum1 / 250;
    PB2_raw_center = sum2 / 250;
    
    // Fallbacks just in case booted while unplugged or pressed
    if (PB1_raw_center > 4000 || PB1_raw_center < 100) {
        PB1_raw_center = 2048;
    }
    
    if (PB2_raw_center > 4000 || PB2_raw_center < 100) {
        PB2_raw_center = 2048;
    }
    
    // Set initial tight bounds around the center (will auto-expand when used)
    PB1_raw_min = PB1_raw_center - 200; 
    PB1_raw_max = PB1_raw_center + 200;
    
    PB2_raw_min = PB2_raw_center - 200; 
    PB2_raw_max = PB2_raw_center + 200;
    
    if ((max3 - min3) < 100) {
        PB3_raw_min = 1000; 
        PB3_raw_max = 3000; 
    } else {
        PB3_raw_min = min3; 
        PB3_raw_max = max3;
    }
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
inline float IRAM_ATTR processTap(uint32_t tapPhase, const float* buffer, int localWriteIdx, uint32_t windowMask, float hannMultiplier) {
    int T = (tapPhase >> 16) & windowMask;
    float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
    
    int effTap = T + 2; 
    int idx1 = localWriteIdx - effTap;
    int idx0 = idx1 + 1; 
    int idx2 = idx1 - 1; 
    int idx3 = idx1 - 2;
    
    float y0 = buffer[idx0]; 
    float y1 = buffer[idx1];
    float y2 = buffer[idx2]; 
    float y3 = buffer[idx3];
    
    float c0 = y1; 
    float c1 = 0.5f * (y2 - y0); 
    float c3 = 1.5f * (y1 - y2) + 0.5f * (y3 - y0); 
    float c2 = y0 - y1 + c1 - c3;
    
    float sample = ((c3 * frac + c2) * frac + c1) * frac + c0;
    
    return sample * hannLUT[(int)(T * hannMultiplier)];
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
        float dynamicBend = (normalizedThrow >= 0.0f) ? (toeBend * normalizedThrow) : (heelBend * std::abs(normalizedThrow));
        pitchShiftLUT[i] = powf(2.0f, (basePitch + dynamicBend) / 12.0f);
    }
    
    if (!isVolumeMode) { 
        pitchShiftFactor = pitchShiftLUT[constrain(currentPB1, 0, 16383)]; 
    }
    
    globalHarmRatio = powf(2.0f, effectMemory[3] / 12.0f);
    globalChorusRatio = powf(2.0f, effectMemory[8] / 12.0f);
    
    float fbIntervals[5] = {0.0f, 12.0f, 19.0f, 24.0f, 28.0f}; 
    globalFbRatio = powf(2.0f, fbIntervals[feedbackIntervalIdx % 5] / 12.0f);
    
    float vibHz = (effectMemory[9] != 0.0f) ? fabsf(effectMemory[9]) : 2.0f;
    globalVibratoPhaseInc = (vibHz * LFO_LUT_SIZE) / SAMPLING_FREQUENCY;
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
    spr.drawString("PB2", xPB2, gaugeBottomY + 15); 
    spr.drawString("PB3", xPB3, gaugeBottomY + 15); 
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
    spr.drawString(cpuUsageBuffer, 10, statsRowY);
    
    char internalSramBuffer[16]; 
    sprintf(internalSramBuffer, "SRM:%dK", (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024)); 
    spr.drawString(internalSramBuffer, 75, statsRowY);
    
    char psramBuffer[16]; 
    sprintf(psramBuffer, "PSR:%dK", (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024)); 
    spr.drawString(psramBuffer, 150, statsRowY);

    spr.setTextDatum(MC_DATUM); 
    spr.setTextColor(TFT_WHITE); 
    spr.drawRect(235, statsRowY - 7, 75, 14, TFT_DARKGREY);
    const char* latencyLabelStrings[] = {"U.Low Lat", "Low Lat", "Mid Lat", "High Lat"}; 
    spr.drawString(latencyLabelStrings[latencyMode], 272, statsRowY);

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
    
    if (isFrozen == true && activeEffectMode != 1) { 
        drawActiveEffectBanner("FRZ", TFT_CYAN); 
    } 
    if (isFeedbackActive == true && activeEffectMode != 2) { 
        drawActiveEffectBanner("SCM", TFT_RED); 
    }
    if (isHarmonizerMode == true && activeEffectMode != 3) { 
        drawActiveEffectBanner("HRM", TFT_MAGENTA); 
    } 
    if (isCapoMode == true && activeEffectMode != 4) { 
        drawActiveEffectBanner("CAP", TFT_GREEN); 
    }
    if (isSynthMode == true && activeEffectMode != 5) { 
        drawActiveEffectBanner("SYN", TFT_YELLOW); 
    } 
    if (isPadMode == true && activeEffectMode != 6) { 
        drawActiveEffectBanner("PAD", TFT_PINK); 
    }
    if (isChorusMode == true && activeEffectMode != 7) { 
        drawActiveEffectBanner("CHO", TFT_SKYBLUE); 
    } 
    if (isSwellMode == true && activeEffectMode != 8) { 
        drawActiveEffectBanner("SWL", TFT_WHITE); 
    }
    if (isVibratoMode == true && activeEffectMode != 9) { 
        drawActiveEffectBanner("VIB", TFT_PURPLE); 
    } 
    if (isVolumeMode == true) { 
        drawActiveEffectBanner("VOL", TFT_DARKGREY); 
    }

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
            updateDisplay(); 
            forceUIUpdate = false; 
        } else if (!isScreenOff && (ui_audio_level > 0.02f || ui_output_level > 0.02f)) { 
            updateMeters(); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

// --- HARDWARE ACCELERATED BLOCK PROCESSING AUDIO TASK ---
void IRAM_ATTR AudioDSPTask(void * pvParameters) {
    static float dsp_out_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_in_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    static int32_t i2s_out_block[HOP_SIZE * 2] __attribute__((aligned(16)));
    
    static float input_block[HOP_SIZE] __attribute__((aligned(16)));
    static float dc_block[HOP_SIZE] __attribute__((aligned(16)));
    static float w1_block[HOP_SIZE] __attribute__((aligned(16)));
    static float mix_block[HOP_SIZE] __attribute__((aligned(16)));
    
    static float internalDelayCache[4200] __attribute__((aligned(16)));
    static float freezeReadCache1[HOP_SIZE] __attribute__((aligned(16)));
    static float freezeReadCache2[HOP_SIZE] __attribute__((aligned(16)));
    static float freezeWriteCache[HOP_SIZE] __attribute__((aligned(16)));
    static float fbReadCache[HOP_SIZE] __attribute__((aligned(16)));
    static float fbWriteCache[HOP_SIZE] __attribute__((aligned(16)));
    
    static float dc_coeffs[5] = {1.0f, -1.0f, 0.0f, -0.995f, 0.0f}; 
    static float dc_state[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    static float synthEnv = 0.0f; 
    static float synthFilter = 0.0f;
    static float padFilter = 0.0f; 
    static float padEnv = 0.0f;
    static float inputEnvelope = 0.0f; 
    static float feedbackFilterVar = 0.0f;
    
    static int freezeWriteIdxVar = 0; 
    static int freezePlayCounterVar = 0; 
    static int freezeStartIdxVar = 0;
    
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
            if (globalAudioResetRequested) {
                synthEnv = 0.0f; 
                synthFilter = 0.0f; 
                padFilter = 0.0f; 
                padEnv = 0.0f;
                inputEnvelope = 0.0f; 
                feedbackFilterVar = 0.0f;
                
                for(int j = 0; j < 4; j++) { 
                    dc_state[j] = 0.0f; 
                }
                for(int j = 0; j < 4200; j++) { 
                    internalDelayCache[j] = 0.0f; 
                }
                
                memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float));
                memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(float));
                memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(float));
                
                ui_audio_level = 0.0f; 
                ui_output_level = 0.0f; 
                globalAudioResetRequested = false;
            }

            uint32_t start_cycles = xthal_get_ccount(); 
            float targetWindow = LATENCY_WINDOWS[latencyMode];
            
            if (currentWindowSize != targetWindow) { 
                currentWindowSize = targetWindow; 
                uint32_t halfWindowFixed = ((uint32_t)targetWindow / 2) << 16;
                
                tap_w1_1 = 0; 
                tap_w1_2 = halfWindowFixed; 
                
                tap_w2_1 = 0; 
                tap_w2_2 = halfWindowFixed;
                
                tap_w3_1 = 0; 
                tap_w3_2 = halfWindowFixed; 
                
                tap_w4_1 = 0; 
                tap_w4_2 = halfWindowFixed; 
                
                tap_w5_1 = 0; 
                tap_w5_2 = halfWindowFixed;
            }
            
            float hMultiplier = 1023.0f / currentWindowSize; 
            uint32_t windowMask = (uint32_t)currentWindowSize - 1;
            
            float invFreqLength = 1.0f / 48000.0f; 
            float chorusPhaseIncr = 1536.0f / 96000.0f; 
            float feedbackPhaseIncr = 5120.0f / 96000.0f;
            
            bool frzActive = ((activeEffectMode == 1 && isWhammyActive) || isFrozen); 
            if (frzActive && !wasFrozen) { 
                freezePlayCounterVar = 0; 
                freezeStartIdxVar = freezeWriteIdxVar; 
            } 
            wasFrozen = frzActive;
            
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
            
            int winSizeInt = (int)currentWindowSize; 
            int historySize = winSizeInt + 4;
            int psramStartIdx = (writeIndex - historySize + MAX_BUFFER_SIZE) & BUFFER_MASK;
            
            if (psramStartIdx + historySize > MAX_BUFFER_SIZE) {
                int part1 = MAX_BUFFER_SIZE - psramStartIdx; 
                int part2 = historySize - part1;
                memcpy(&internalDelayCache[0], &delayBuffer[psramStartIdx], part1 * sizeof(float));
                memcpy(&internalDelayCache[part1], &delayBuffer[0], part2 * sizeof(float));
            } else {
                memcpy(&internalDelayCache[0], &delayBuffer[psramStartIdx], historySize * sizeof(float));
            }
            
            if (freezeRamp > 0.0f || frzActive) {
                int start1 = (freezeStartIdxVar + freezePlayCounterVar) % freezeLength;
                if (start1 + HOP_SIZE > freezeLength) {
                    int p1 = freezeLength - start1;
                    memcpy(&freezeReadCache1[0], &freezeBuffer[start1], p1 * sizeof(float));
                    memcpy(&freezeReadCache1[p1], &freezeBuffer[0], (HOP_SIZE - p1) * sizeof(float));
                } else {
                    memcpy(&freezeReadCache1[0], &freezeBuffer[start1], HOP_SIZE * sizeof(float));
                }
                
                int start2 = (freezeStartIdxVar + freezePlayCounterVar + (freezeLength / 2)) % freezeLength;
                if (start2 + HOP_SIZE > freezeLength) {
                    int p1 = freezeLength - start2;
                    memcpy(&freezeReadCache2[0], &freezeBuffer[start2], p1 * sizeof(float));
                    memcpy(&freezeReadCache2[p1], &freezeBuffer[0], (HOP_SIZE - p1) * sizeof(float));
                } else {
                    memcpy(&freezeReadCache2[0], &freezeBuffer[start2], HOP_SIZE * sizeof(float));
                }
            }
            
            if (feedbackActive || feedbackRamp > 0.0f) {
                int fbReadStart = (fbDelayWriteIdx - (int)(SAMPLING_FREQUENCY * 0.02f) + FB_BUFFER_SIZE) & FB_BUFFER_MASK;
                if (fbReadStart + HOP_SIZE > FB_BUFFER_SIZE) {
                    int p1 = FB_BUFFER_SIZE - fbReadStart;
                    memcpy(&fbReadCache[0], &fbDelayBuffer[fbReadStart], p1 * sizeof(float));
                    memcpy(&fbReadCache[p1], &fbDelayBuffer[0], (HOP_SIZE - p1) * sizeof(float));
                } else {
                    memcpy(&fbReadCache[0], &fbDelayBuffer[fbReadStart], HOP_SIZE * sizeof(float));
                }
            }
            
            #pragma GCC ivdep
            for (int i = 0; i < HOP_SIZE; i++) { 
                input_block[i] = ((float)i2s_in_block[i * 2] * normFactor); 
            }
            
            dsps_biquad_f32(input_block, dc_block, HOP_SIZE, dc_coeffs, dc_state);
            
            int blockWriteStart = writeIndex; 
            int frzBlockStart = freezeWriteIdxVar; 
            int fbBlockStart = fbDelayWriteIdx;
            
            for (int i = 0; i < HOP_SIZE; i++) {
                float inSample = dc_block[i]; 
                inputEnvelope = inputEnvelope * 0.99f + fabsf(inSample) * 0.01f + DC_OFFSET;
                
                if (swellActive) {
                    if (inputEnvelope > 0.015f) { 
                        swellGain = fminf(1.0f, swellGain + 0.00002f); 
                    } else { 
                        swellGain = fmaxf(0.0f, swellGain - 0.00005f); 
                    }
                } else { 
                    swellGain = 1.0f; 
                }
                
                float procSample = inSample;
                
                if (synthActive) { 
                    if (inputEnvelope > 0.005f) { 
                        synthEnv = fminf(1.0f, synthEnv + 0.1f); 
                    } else { 
                        synthEnv = fmaxf(0.0f, synthEnv - 0.005f); 
                    }
                    
                    int waveIdx = constrain((int)((procSample + 1.0f) * 1023.5f), 0, WAVE_LUT_SIZE - 1);
                    procSample = synthLUT[waveIdx]; 
                    
                    synthFilter = synthFilter + (0.3f + 0.8f * synthEnv) * (procSample - synthFilter) + DC_OFFSET; 
                    procSample = synthFilter * 0.1f;
                } 
                
                if (padActive) { 
                    if (inputEnvelope > 0.005f) { 
                        padEnv = fminf(1.0f, padEnv + 0.00002f); 
                    } else { 
                        padEnv = fmaxf(0.0f, padEnv - 0.000005f); 
                    }
                    procSample *= padEnv; 
                }
                
                if (!frzActive) { 
                    freezeWriteCache[i] = procSample; 
                }
                
                if (freezeRamp > 0.0f || frzActive) {
                    if (frzActive) { 
                        freezeRamp = fminf(1.0f, freezeRamp + 0.0002f); 
                    } else { 
                        freezeRamp = fmaxf(0.0f, freezeRamp - 0.00005f); 
                    }
                }
                
                float fzOut = 0.0f;
                
                if (freezeRamp > 0.0f) { 
                    float phaseRead = (float)freezePlayCounterVar * invFreqLength; 
                    float phase2 = (phaseRead + 0.5f); 
                    
                    if (phase2 >= 1.0f) { 
                        phase2 -= 1.0f; 
                    }
                    
                    float rFrz = (freezeReadCache1[i] * hannLUT[(int)(phaseRead * 1023.0f)]) + 
                                 (freezeReadCache2[i] * hannLUT[(int)(phase2 * 1023.0f)]);
                    
                    float d1 = apf1Buffer[apf1Idx]; 
                    float a1 = -0.6f * rFrz + d1; 
                    apf1Buffer[apf1Idx] = rFrz + 0.6f * d1 + DC_OFFSET; 
                    apf1Idx++; 
                    
                    if (apf1Idx >= 1009) { 
                        apf1Idx = 0; 
                    }
                    
                    float d2 = apf2Buffer[apf2Idx]; 
                    float a2 = -0.6f * a1 + d2; 
                    apf2Buffer[apf2Idx] = a1 + 0.6f * d2 + DC_OFFSET; 
                    apf2Idx++; 
                    
                    if (apf2Idx >= 863) { 
                        apf2Idx = 0; 
                    }
                    
                    fzOut = a2 * freezeRamp; 
                    freezePlayCounterVar++; 
                    
                    if (freezePlayCounterVar >= freezeLength) { 
                        freezePlayCounterVar = 0; 
                    }
                }
                
                float delayIn = (frzActive && freezeRamp > 0.0f) ? fzOut : procSample; 
                float boundedDelayIn = constrain(delayIn, -1.0f, 1.0f);
                
                int localWriteIdx = historySize + i; 
                internalDelayCache[localWriteIdx] = boundedDelayIn;
                
                float spd1 = pitchShiftFactor;
                
                if (vibratoActive) {
                    vibratoLfoPhase += globalVibratoPhaseInc; 
                    if (vibratoLfoPhase >= LFO_LUT_SIZE) { 
                        vibratoLfoPhase -= LFO_LUT_SIZE; 
                    }
                    spd1 *= lfoLUT[(int)vibratoLfoPhase];
                }
                
                float spd2 = pitchShiftFactor * globalHarmRatio; 
                float spd3 = pitchShiftFactor * globalChorusRatio;
                
                if (chorusActive) { 
                    chorusLfoPhase += chorusPhaseIncr; 
                    if (chorusLfoPhase >= LFO_LUT_SIZE) { 
                        chorusLfoPhase -= LFO_LUT_SIZE; 
                    }
                    spd3 *= lfoLUT[(int)chorusLfoPhase]; 
                }
                
                float spd4 = 1.0f; 
                float spd5 = 1.0f;
                
                if (feedbackActive || feedbackRamp > 0.0f) { 
                    feedbackLfoPhase += feedbackPhaseIncr; 
                    if (feedbackLfoPhase >= LFO_LUT_SIZE) { 
                        feedbackLfoPhase -= LFO_LUT_SIZE; 
                    }
                    float lfoVal = lfoLUT[(int)feedbackLfoPhase]; 
                    spd4 = lfoVal; 
                    spd5 = pitchShiftFactor * globalFbRatio * lfoVal; 
                }
                
                float w1 = processTap(tap_w1_1, internalDelayCache, localWriteIdx, windowMask, hMultiplier) + 
                           processTap(tap_w1_2, internalDelayCache, localWriteIdx, windowMask, hMultiplier);
                
                w1_block[i] = w1; 
                
                float w2 = 0.0f;
                if (harmActive) {
                    w2 = processTap(tap_w2_1, internalDelayCache, localWriteIdx, windowMask, hMultiplier) + 
                         processTap(tap_w2_2, internalDelayCache, localWriteIdx, windowMask, hMultiplier);
                }
                
                float w3 = 0.0f;
                if (chorusActive) {
                    w3 = processTap(tap_w3_1, internalDelayCache, localWriteIdx, windowMask, hMultiplier) + 
                         processTap(tap_w3_2, internalDelayCache, localWriteIdx, windowMask, hMultiplier);
                }
                
                float w4 = 0.0f; 
                float w5 = 0.0f; 
                
                if (feedbackActive || feedbackRamp > 0.0f) { 
                    w4 = processTap(tap_w4_1, internalDelayCache, localWriteIdx, windowMask, hMultiplier) + 
                         processTap(tap_w4_2, internalDelayCache, localWriteIdx, windowMask, hMultiplier);
                    w5 = processTap(tap_w5_1, internalDelayCache, localWriteIdx, windowMask, hMultiplier) + 
                         processTap(tap_w5_2, internalDelayCache, localWriteIdx, windowMask, hMultiplier);
                }
                
                int32_t step1 = (int32_t)((1.0f - spd1) * 65536.0f); 
                tap_w1_1 += step1; 
                tap_w1_2 += step1; 
                
                int32_t step2 = (int32_t)((1.0f - spd2) * 65536.0f); 
                tap_w2_1 += step2; 
                tap_w2_2 += step2; 
                
                int32_t step3 = (int32_t)((1.0f - spd3) * 65536.0f); 
                tap_w3_1 += step3; 
                tap_w3_2 += step3; 
                
                int32_t step4 = (int32_t)((1.0f - spd4) * 65536.0f); 
                tap_w4_1 += step4; 
                tap_w4_2 += step4; 
                
                int32_t step5 = (int32_t)((1.0f - spd5) * 65536.0f); 
                tap_w5_1 += step5; 
                tap_w5_2 += step5; 
                
                writeIndex = (writeIndex + 1) & BUFFER_MASK;
                
                float fbOutNode = 0.0f; 
                if (feedbackActive || feedbackRamp > 0.0f) {
                    if (feedbackActive) {
                        if (inputEnvelope > 0.005f) { 
                            feedbackRamp = fminf(1.0f, feedbackRamp + 0.000011f); 
                        } else { 
                            feedbackRamp = fmaxf(0.0f, feedbackRamp - 0.005f); 
                        }
                    } else { 
                        feedbackRamp = fmaxf(0.0f, feedbackRamp - 0.0001f); 
                    }
                    
                    float mixV = fmaxf(0.0f, fminf((feedbackRamp - 0.1f) * 2.0f, 1.0f));
                    float feedInput = (frzActive && freezeRamp > 0.0f) ? fzOut : (w4 * (1.0f - mixV)) + (w5 * mixV);
                    
                    fbHpfState += 0.05f * (feedInput - fbHpfState) + DC_OFFSET; 
                    float gainDrive = constrain((feedInput - fbHpfState) * 30.0f, -1.0f, 1.0f); 
                    
                    feedbackFilterVar = feedbackFilterVar * 0.9f + gainDrive * 0.1f + DC_OFFSET;
                    float satFb = feedbackFilterVar * (feedbackRamp * feedbackRamp * feedbackRamp) * 0.85f; 
                    
                    fbWriteCache[i] = satFb; 
                    fbOutNode = fbReadCache[i]; 
                    
                    fbDelayWriteIdx = (fbDelayWriteIdx + 1) & FB_BUFFER_MASK;
                }
                
                if (padActive) { 
                    padFilter = padFilter * 0.95f + w1 * 0.05f + DC_OFFSET; 
                } else { 
                    padFilter = padFilter * 0.95f + DC_OFFSET; 
                }
                
                bool activeGroup = isWhammyActive || harmActive || chorusActive || feedbackActive || synthActive || padActive || frzActive || vibratoActive || capoActive;
                bool dryGroup = chorusActive || padActive || frzActive || feedbackActive || (freezeRamp > 0.0f) || (feedbackRamp > 0.0f);
                bool repeatGroup = capoActive || synthActive || vibratoActive || padActive || harmActive;
                
                float sMix = DC_OFFSET;
                
                if (!activeGroup && freezeRamp <= 0.0f && feedbackRamp <= 0.0f && padFilter < 0.001f) {
                    sMix = inSample;
                } else {
                    if (activeGroup || freezeRamp > 0.0f || feedbackRamp > 0.0f || padFilter > 0.001f) {
                        if (dryGroup) { 
                            if (!repeatGroup) { 
                                sMix += (inSample * 0.4f); 
                            } 
                        } else if (harmActive) { 
                            sMix += (w1 * 0.5f); 
                        } else { 
                            sMix += w1; 
                        }
                        
                        if (harmActive) { 
                            sMix += (w2 * 0.5f); 
                        }
                        
                        if (chorusActive) { 
                            sMix += (w3 * 0.4f); 
                        }
                        
                        if (padActive || padFilter > 0.001f) { 
                            sMix += (padFilter * 1.5f); 
                        }
                        
                        if (!frzActive && freezeRamp > 0.0f) { 
                            sMix += (fzOut * 0.5f); 
                        }
                        
                        if (feedbackActive || feedbackRamp > 0.0f) { 
                            sMix += (fbOutNode * 0.6f); 
                        }
                        
                        sMix = sMix * (1.0f - (0.1f * sMix * sMix));
                    }
                }
                
                mix_block[i] = sMix;
            }
            
            if (blockWriteStart + HOP_SIZE > MAX_BUFFER_SIZE) {
                int p1 = MAX_BUFFER_SIZE - blockWriteStart; 
                int p2 = HOP_SIZE - p1;
                memcpy(&delayBuffer[blockWriteStart], &internalDelayCache[historySize], p1 * sizeof(float));
                memcpy(&delayBuffer[0], &internalDelayCache[historySize + p1], p2 * sizeof(float));
            } else {
                memcpy(&delayBuffer[blockWriteStart], &internalDelayCache[historySize], HOP_SIZE * sizeof(float));
            }
            
            if (!frzActive) {
                if (frzBlockStart + HOP_SIZE > freezeLength) {
                    int p1 = freezeLength - frzBlockStart;
                    memcpy(&freezeBuffer[frzBlockStart], &freezeWriteCache[0], p1 * sizeof(float));
                    memcpy(&freezeBuffer[0], &freezeWriteCache[p1], (HOP_SIZE - p1) * sizeof(float));
                } else {
                    memcpy(&freezeBuffer[frzBlockStart], &freezeWriteCache[0], HOP_SIZE * sizeof(float));
                }
                freezeWriteIdxVar = (frzBlockStart + HOP_SIZE) % freezeLength;
            }
            
            if (feedbackActive || feedbackRamp > 0.0f) {
                if (fbBlockStart + HOP_SIZE > FB_BUFFER_SIZE) {
                    int p1 = FB_BUFFER_SIZE - fbBlockStart;
                    memcpy(&fbDelayBuffer[fbBlockStart], &fbWriteCache[0], p1 * sizeof(float));
                    memcpy(&fbDelayBuffer[0], &fbWriteCache[p1], (HOP_SIZE - p1) * sizeof(float));
                } else {
                    memcpy(&fbDelayBuffer[fbBlockStart], &fbWriteCache[0], HOP_SIZE * sizeof(float));
                }
            }
            
            bool dryPathActive = chorusActive || padActive || frzActive || feedbackActive || (freezeRamp > 0.0f) || (feedbackRamp > 0.0f);
            bool totalAnyActive = isWhammyActive || harmActive || chorusActive || feedbackActive || synthActive || padActive || frzActive || vibratoActive || capoActive;
            
            #pragma GCC ivdep
            for (int i = 0; i < HOP_SIZE; i++) {
                if (totalAnyActive || freezeRamp > 0.0f || feedbackRamp > 0.0f || padFilter > 0.001f) {
                    if (dryPathActive) { 
                        mix_block[i] += (w1_block[i] * 0.4f); 
                    } else if (!harmActive) { 
                        mix_block[i] += w1_block[i]; 
                    }
                }
                
                float outStage = mix_block[i] * swellGain * volumePedalGain; 
                dsp_out_block[i * 2] = outStage; 
                dsp_out_block[i * 2 + 1] = outStage;
                
                if (fabsf(dc_block[i]) > peakInputVal) { 
                    peakInputVal = fabsf(dc_block[i]); 
                }
                
                if (fabsf(outStage) > peakOutputVal) { 
                    peakOutputVal = fabsf(outStage); 
                }
            }
            
            uint32_t end_timer = xthal_get_ccount(); 
            float currentLoadPercentage = ((float)(end_timer - start_cycles) / 160000.0f) * 100.0f;
            core1_load = core1_load * 0.95f + fminf(100.0f, currentLoadPercentage) * 0.05f; 
            
            float bit32Scale = 2147483647.0f; 
            dsps_mul_f32(dsp_out_block, &bit32Scale, dsp_out_block, HOP_SIZE * 2, 1, 0, 1);
            
            #pragma GCC ivdep
            for (int i = 0; i < HOP_SIZE * 2; i++) {
                i2s_out_block[i] = (int32_t)fmaxf(-2147483648.0f, fminf(dsp_out_block[i], 2147483647.0f));
            }
            
            if (peakInputVal > ui_audio_level) { 
                ui_audio_level = peakInputVal; 
            } else { 
                ui_audio_level *= 0.998f; 
                if (ui_audio_level < 1e-5f) {
                    ui_audio_level = 0.0f;
                }
            }
            
            if (peakOutputVal > ui_output_level) { 
                ui_output_level = peakOutputVal; 
            } else { 
                ui_output_level *= 0.998f; 
                if (ui_output_level < 1e-5f) {
                    ui_output_level = 0.0f;
                }
            }
            
            size_t bytesWrittenCount; 
            i2s_channel_write(tx_chan, i2s_out_block, sizeof(i2s_out_block), &bytesWrittenCount, portMAX_DELAY);
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
    
    static bool unpluggedA = false;
    static bool unpluggedB = false;
    static bool unpluggedC = false;
    
    static DebouncedButton carouselBtn(CAROUSEL_BUTTON_PIN); 
    carouselBtn.state = digitalRead(CAROUSEL_BUTTON_PIN); 
    carouselBtn.lastReading = carouselBtn.state; 
    
    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP);
    
    for (;;) {
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
                    updateLUT(); 
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
            
            // --- HYSTERESIS UNPLUG DETECTION ---
            if (rawA >= 4094) {
                unpluggedA = true;
            } else if (rawA < 4080) {
                unpluggedA = false;
            }
            
            if (rawB >= 4094) {
                unpluggedB = true;
            } else if (rawB < 4080) {
                unpluggedB = false;
            }
            
            if (rawC >= 4094) {
                unpluggedC = true;
            } else if (rawC < 4080) {
                unpluggedC = false;
            }
            
            if (smoothRawA < 0) {
                smoothRawA = rawA;
            }
            smoothRawA = smoothRawA * 0.5f + (float)rawA * 0.5f; 
            
            if (smoothRawB < 0) {
                smoothRawB = rawB;
            }
            smoothRawB = smoothRawB * 0.5f + (float)rawB * 0.5f; 
            
            if (smoothRawC < 0) {
                smoothRawC = rawC;
            }
            smoothRawC = smoothRawC * 0.9f + (float)rawC * 0.1f;
            
            // --- CONTINUOUS AUTO-TRACKER ---
            if (!unpluggedA) {
                if (smoothRawA < PB1_raw_min && smoothRawA > 20) {
                    PB1_raw_min = smoothRawA;
                }
                if (smoothRawA > PB1_raw_max && smoothRawA < 4050) {
                    PB1_raw_max = smoothRawA;
                }
            }
            
            if (!unpluggedB) {
                if (smoothRawB < PB2_raw_min && smoothRawB > 20) {
                    PB2_raw_min = smoothRawB;
                }
                if (smoothRawB > PB2_raw_max && smoothRawB < 4050) {
                    PB2_raw_max = smoothRawB;
                }
            }
            
            if (!unpluggedC) {
                if (smoothRawC < PB3_raw_min && smoothRawC > 20) {
                    PB3_raw_min = smoothRawC;
                }
                if (smoothRawC > PB3_raw_max && smoothRawC < 4050) {
                    PB3_raw_max = smoothRawC;
                }
            }
            
            // Map the pedals cleanly using the dynamic bounds and heavy anchors
            analog_t calA = map_raw_deadzone(smoothRawA, PB1_raw_center, PB1_raw_min, PB1_raw_max, deadzone_size);
            analog_t calB = map_raw_deadzone(smoothRawB, PB2_raw_center, PB2_raw_min, PB2_raw_max, deadzone_size);
            
            // PB3 also gets a 120-point lock buffer built into its constrain and map
            int lockC = 120;
            int conC = constrain((int)smoothRawC, PB3_raw_min + lockC, PB3_raw_max - lockC); 
            analog_t calC = map(conC, PB3_raw_min + lockC, PB3_raw_max - lockC, 16383, 0); 
            
            if (calC < 50) {
                calC = 0;
            } 
            if (calC > 16333) {
                calC = 16383;
            } 
            
            // Enforce center if unplugged
            if (unpluggedA) {
                calA = 8192;
            }
            if (unpluggedB) {
                calB = 8192;
            }
            if (unpluggedC) {
                calC = 8192;
            }
            
            bool moveA = (abs((int)calA - (int)lastMidiA) > 12);
            bool moveB = (abs((int)calB - (int)lastMidiB) > 12);
            bool moveC = (abs((int)calC - (int)lastMidiC) > 12);
            
            if (moveA || moveB || moveC) {
                if (isScreenOff) { 
                    turnScreenOn(); 
                }
                
                lastScreenActivityTime = millis();
                
                if (moveA) { 
                    if (!isVolumeMode) { 
                        Control_Surface.sendPitchBend(Channel_1, calA); 
                    }
                    lastMidiA = calA; 
                    currentPB1 = calA; 
                }
                
                if (moveB) { 
                    if (!isVolumeMode) { 
                        Control_Surface.sendPitchBend(Channel_2, calB); 
                    }
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
                
                analog_t activePedal = calA;
                
                if (moveC) { 
                    activePedal = calC; 
                } else if (moveB) { 
                    activePedal = calB; 
                }
                
                if (isVolumeMode) { 
                    uint8_t vCC = map(activePedal, 0, 16383, 0, 127); 
                    if (vCC != lastVolumeCC) { 
                        Control_Surface.sendControlChange({19, Channel_1}, vCC); 
                        lastVolumeCC = vCC; 
                    } 
                    volumePedalGain = (float)activePedal / 16383.0f; 
                } else { 
                    pitchShiftFactor = pitchShiftLUT[constrain(activePedal, 0, 16383)]; 
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
                
                if (!isVolumeMode) { 
                    pitchShiftFactor = pitchShiftLUT[8192]; 
                } else { 
                    volumePedalGain = 8192.0f / 16383.0f; 
                }
                
                forceUIUpdate = true;
            }
            vTaskDelay(pdMS_TO_TICKS(250)); 
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
            updateLUT(); 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 5 && cm.data2 >= 64) { 
            activeEffectMode = (activeEffectMode + 1) % 10; 
            updateLUT(); 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 6 && cm.data2 >= 64) { 
            latencyMode = (latencyMode + 1) % 4; 
            forceUIUpdate = true; 
        }
        else if (cm.data1 == 7) {
            globalAudioResetRequested = true; 
            isWhammyActive = (cm.data2 < 64); 
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
        else if (cm.data1 == 18 || cm.data1 == 17) {
            float direction = (cm.data2 < 64) ? 1.0f : -1.0f;
            
            if (activeEffectMode == 0 || activeEffectMode == 1 || activeEffectMode == 8) {
                if (cm.data1 == 18) { 
                    effectMemory[0] = constrain(effectMemory[0] + direction, -24.0f, 24.0f); 
                } else { 
                    effectMemory[5] = constrain(effectMemory[5] + direction, -24.0f, 24.0f); 
                }
            } else if (activeEffectMode == 4) {
                float change = (cm.data1 == 18) ? 1.0f : 0.01f;
                effectMemory[4] = constrain(effectMemory[4] + change * direction, -24.0f, 24.0f);
            } else if (activeEffectMode == 2) { 
                if (direction > 0) { 
                    feedbackIntervalIdx = (feedbackIntervalIdx + 1) % 5; 
                } else { 
                    feedbackIntervalIdx = (feedbackIntervalIdx + 4) % 5; 
                }
            } else { 
                int memIndex = activeEffectMode; 
                
                if (memIndex == 5) { 
                    memIndex = 6; 
                } else if (memIndex == 6) { 
                    memIndex = 7; 
                } else if (memIndex == 7) { 
                    memIndex = 8; 
                } else if (memIndex == 9) { 
                    memIndex = 9; 
                }
                
                effectMemory[memIndex] = constrain(effectMemory[memIndex] + direction, -24.0f, 24.0f); 
            }
            
            updateLUT(); 
            forceUIUpdate = true;
        }
        else if (cm.data1 == 11) { 
            uint16_t mappedCC = map(cm.data2, 0, 127, 0, 16383); 
            currentCC11 = mappedCC; 
            currentPB3 = mappedCC; 
            pitchShiftFactor = pitchShiftLUT[mappedCC]; 
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
    
    if (delayBuffer == nullptr || fbDelayBuffer == nullptr || freezeBuffer == nullptr) {
        tft.fillScreen(TFT_RED); 
        tft.setTextColor(TFT_WHITE, TFT_RED); 
        tft.drawString("MEMORY ERROR", 160, 85);
        
        while(1) { 
            delay(100); 
        }
    }
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(float)); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(float)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(float));
    
    for (int i = 0; i < 1024; i++) { 
        hannLUT[i] = sinf(PI * ((float)i / 1023.0f)); 
        lfoLUT[i] = powf(2.0f, (15.0f * sinf(TWO_PI * ((float)i / 1024.0f))) / 1200.0f); 
    }
    
    for (int i = 0; i < 2048; i++) { 
        synthLUT[i] = sinf((((float)i - 1024.0f) / 1024.0f) * 45.0f); 
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
    
    i2s_chan_config_t i2sConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); 
    i2sConfig.dma_desc_num = 6; 
    i2sConfig.dma_frame_num = 256; 
    i2sConfig.auto_clear = true;
    
    i2s_new_channel(&i2sConfig, &tx_chan, &rx_chan);
    
    i2s_std_config_t stdConfig = { 
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_FREQUENCY), 
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
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
    // Everything handled in RTOS Tasks
}