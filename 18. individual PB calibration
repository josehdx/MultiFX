#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <TFT_eSPI.h>
#include <dsps_fft2r.h>   
#include <dsps_wind.h>
#include <driver/i2s_std.h> 
#include "driver/gpio.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "dsps_mul.h"
#include "driver/rtc_io.h"

// --- BARE-METAL PRE-BOOT ASSASSIN ---
void __attribute__((constructor)) pre_boot_kill_switch() {
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_38, 0); 
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_15, 0);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 0);
}

/// --- TYPE DEFINITIONS & GLOBALS ---
i2s_chan_handle_t tx_chan;
i2s_chan_handle_t rx_chan;

// --- DUAL SLEEP CONFIGURATION ---
unsigned long lastActivityTime = 0;       
unsigned long lastScreenActivityTime = 0; 
const unsigned long LIGHT_SLEEP_TIMEOUT = 60000; 
const unsigned long SCREEN_OFF_TIMEOUT = 120000;  
bool isScreenOff = false;
TaskHandle_t audioTaskHandle = NULL;

// --- TFT DISPLAY & SPRITE OBJECTS ---
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft); 
volatile bool forceUIUpdate = true; 

// --- THE LOOKUP TABLE ---
float pitchShiftLUT[16384]; 

// --- Configuration & Constants ---
#define SAMPLES 512             
#define HOP_SIZE 64            
#define SAMPLING_FREQUENCY 96000 
#define SERIAL_BAUDRATE 115200
#define STREAM_BUFFER_SIZE (SAMPLES * 4 * sizeof(int16_t)) 
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// --- Global DSP Buffers (Core 1) ---
float fft_buffer[SAMPLES * 2] __attribute__((aligned(16))); 
float window_coefficients[SAMPLES];
float prevAnalysisPhase[SAMPLES / 2];
float prevSynthesisPhase[SAMPLES / 2];
volatile float pitchShiftFactor = 1.0; 
volatile bool isFrozen = false;
float frozenMag[SAMPLES / 2] = {0};
float padMagBuffer[SAMPLES / 2] = {0}; // Infinite sustain buffer for Pad Mode

// --- HARDWARE PINS (Core 0) ---
pin_t pinPB = A0;
pin_t pinPB2 = A2;
const int CAROUSEL_BUTTON_PIN = 14; 
const int FREEZE_BUTTON_PIN = 2;    
const int INTERVAL_BUTTON_PIN = 43; 
const int FEEDBACK_BUTTON_PIN = 44; 

// Button States
bool lastCarouselState = HIGH;
bool lastBootState = HIGH;      
bool lastButtonState = HIGH;
bool lastIntervalButtonState = HIGH;
bool lastFbState = HIGH;
unsigned long carouselPressedTime = 0; 
unsigned long bootPressedTime = 0;     
unsigned long intervalButtonPressedTime = 0;

// --- INDEPENDENT EFFECT MEMORY BANKS ---
int activeEffectMode = 0; // 0=WHAMMY, 1=FREEZE, 2=FEEDBACK, 3=HARMONY, 4=CAPO, 5=SYNTH, 6=PAD

float effectMemory[8] = { 
    12.0f,  // 0: Whammy TOE 
    12.0f,  // 1: Freeze
    12.0f,  // 2: Feedback
    -5.0f,  // 3: Harmony
    -2.0f,  // 4: Capo
    -12.0f, // 5: Whammy HEEL
    -12.0f, // 6: Synth
    12.0f   // 7: Pad
};

volatile bool isHarmonizerMode = false; 
volatile float feedbackRamp = 0.0f;
volatile bool isFeedbackActive = false;
volatile bool isCapoMode = false;
volatile bool isSynthMode = false; 
volatile bool isPadMode = false;   
volatile float ui_audio_level = 0.0f;
volatile float ui_output_level = 0.0f;

BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;
Bank<16> bankChannel;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB2 = pinPB2;
uint16_t lastMidiSent = 8192;
volatile uint16_t currentPB1 = 8192;
volatile uint16_t currentPB2 = 8192;
volatile uint16_t currentCC11 = 0;

// --- NEW: INDEPENDENT DUAL CALIBRATION VARIABLES ---
double PBdeadzoneMultiplier = 14;
double PBdeadzoneMinimum = 950;
double PBdeadzoneMaximum = 1600;
analog_t PBminimumValue = 0;
analog_t PBmaximumValue = 16383;

// PB1 specific
analog_t PBcenter1 = 8192;
analog_t PBdeadzone1 = PBdeadzoneMinimum;
bool PBwasOffCenter1 = false;

// PB2 specific
analog_t PBcenter2 = 8192;
analog_t PBdeadzone2 = PBdeadzoneMinimum;
bool PBwasOffCenter2 = false;

// --- NEW: DYNAMIC MAPPING FUNCTION ---
analog_t map_PB(analog_t raw, analog_t center, analog_t deadzone, bool &offCenterFlag) {
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    if (raw <= PBminimumValue + 150) { offCenterFlag = true; return 0; }
    if (raw >= PBmaximumValue - 150) { offCenterFlag = true; return 16383; }
    if (raw <= center - deadzone) { offCenterFlag = true; return map(raw, PBminimumValue, center - deadzone, 0, 8191); }
    else if (raw >= center + deadzone) { offCenterFlag = true; return map(raw, center + deadzone, PBmaximumValue, 8191, 16383); }
    else { return 8192; }
}

void goToLightSleep() {
    Serial.println("Entering Light Sleep...");
    digitalWrite(38, LOW);  
    digitalWrite(15, LOW);  
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); 
    rtc_gpio_pullup_en(GPIO_NUM_0); 
    rtc_gpio_pulldown_dis(GPIO_NUM_0);
    if (audioTaskHandle != NULL) vTaskSuspend(audioTaskHandle); 
    i2s_channel_disable(tx_chan);      
    i2s_channel_disable(rx_chan);      
    esp_sleep_wakeup_cause_t wakeup_reason;
    do {
        esp_light_sleep_start();
        wakeup_reason = esp_sleep_get_wakeup_cause();
    } while (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0);
    i2s_channel_enable(tx_chan);
    i2s_channel_enable(rx_chan);
    if (audioTaskHandle != NULL) vTaskResume(audioTaskHandle); 
    vTaskDelay(pdMS_TO_TICKS(200)); 
    rtc_gpio_deinit(GPIO_NUM_0);    
    pinMode(0, INPUT_PULLUP);       
    pinMode(15, OUTPUT); digitalWrite(15, HIGH); 
    tft.init(); 
    forceUIUpdate = true;
    delay(120); 
    pinMode(38, OUTPUT);    
    digitalWrite(38, HIGH); 
    isScreenOff = false; 
    lastActivityTime = millis();
    lastScreenActivityTime = millis();
    Serial.println("System Woken Up!");
}

// --- NEW: INDEPENDENT DUAL CALIBRATION LOGIC ---
void calibrateCenterAndDeadzone() {
  Serial.println("Calibrating Center and Deadzones for PB1 and PB2...");
  for(int i=0; i<50; i++) { filterPB.update(); filterPB2.update(); delay(1); }
  
  int iNumberOfSamples = 750;
  
  analog_t calibPBLow1 = 16383; analog_t calibPBHigh1 = 0; long lSampleSumPB1 = 0;
  analog_t calibPBLow2 = 16383; analog_t calibPBHigh2 = 0; long lSampleSumPB2 = 0;

  for (int iSample = 1; iSample <= iNumberOfSamples; iSample++) {
    filterPB.update(); 
    filterPB2.update(); 

    // Process PB1
    analog_t raw12_1 = filterPB.getValue();
    analog_t calibPB1 = map(raw12_1, 0, 4095, 0, 16383); 
    lSampleSumPB1 += calibPB1;
    if (calibPB1 < calibPBLow1) { calibPBLow1 = calibPB1; } 
    if (calibPB1 > calibPBHigh1) { calibPBHigh1 = calibPB1; } 

    // Process PB2
    analog_t raw12_2 = filterPB2.getValue();
    analog_t calibPB2 = map(raw12_2, 0, 4095, 0, 16383); 
    lSampleSumPB2 += calibPB2;
    if (calibPB2 < calibPBLow2) { calibPBLow2 = calibPB2; } 
    if (calibPB2 > calibPBHigh2) { calibPBHigh2 = calibPB2; } 

    delay(1);
  }
  
  // Assign independent centers and deadzones
  PBcenter1 = lSampleSumPB1 / iNumberOfSamples;
  PBdeadzone1 = (analog_t)constrain(((calibPBHigh1 - calibPBLow1) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);

  PBcenter2 = lSampleSumPB2 / iNumberOfSamples;
  PBdeadzone2 = (analog_t)constrain(((calibPBHigh2 - calibPBLow2) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
}

void updateLUT() {
    for (int i = 0; i < 16384; i++) {
        float normalizedThrow = (float(i) - 8192.0f) / 8192.0f; 
        float currentWhammyShift = 0.0f;

        if (activeEffectMode == 0 || activeEffectMode == 4) {
            // Asymmetrical Whammy & Capo Sweep
            if (normalizedThrow >= 0.0f) {
                currentWhammyShift = effectMemory[0] * normalizedThrow; 
            } else {
                currentWhammyShift = effectMemory[5] * (-normalizedThrow); 
            }
        } else {
            // Symmetrical sweep for all other effects (including Synth and Pad)
            int memSlot = activeEffectMode;
            if (activeEffectMode == 5) memSlot = 6;
            else if (activeEffectMode == 6) memSlot = 7;
            currentWhammyShift = effectMemory[memSlot] * normalizedThrow;
        }
        
        float currentCapoShift = isCapoMode ? effectMemory[4] : 0.0f;
        float totalSemitones = currentCapoShift + currentWhammyShift;
        
        pitchShiftLUT[i] = powf(2.0f, totalSemitones / 12.0f);
    }
}

inline float IRAM_ATTR fast_mag(float re, float im) {
    float abs_re = fabsf(re);
    float abs_im = fabsf(im);
    float max_val = (abs_re > abs_im) ? abs_re : abs_im;
    float min_val = (abs_re < abs_im) ? abs_re : abs_im;
    return max_val + 0.337f * min_val;
}

const int TRIG_LUT_SIZE = 4096; 
float sinLUT[TRIG_LUT_SIZE];

void turnScreenOff() {
    if (!isScreenOff) {
        digitalWrite(38, LOW);  
        digitalWrite(15, LOW);  
        isScreenOff = true;
    }
}

void turnScreenOn() {
    if (isScreenOff) {
        pinMode(15, OUTPUT); digitalWrite(15, HIGH); 
        tft.init(); 
        forceUIUpdate = true;
        delay(120);
        pinMode(38, OUTPUT); digitalWrite(38, HIGH); 
        isScreenOff = false;
    }
}

void initTrigLUT() {
    for (int i = 0; i < TRIG_LUT_SIZE; i++) {
        sinLUT[i] = sinf((float)i / (float)TRIG_LUT_SIZE * 2.0f * PI);
    }
}

inline float IRAM_ATTR fast_sin(float phase) {
    float wrapped = phase - (TWO_PI * (int)(phase / TWO_PI));
    if (wrapped < 0) wrapped += TWO_PI;
    int index = (int)((wrapped / TWO_PI) * TRIG_LUT_SIZE);
    return sinLUT[index % TRIG_LUT_SIZE];
}

inline float IRAM_ATTR fast_cos(float phase) {
    return fast_sin(phase + (PI / 2.0f));
}

// --- CORE 1: AUDIO DSP TASK (Priority 24) ---
void IRAM_ATTR AudioDSPTask(void * pvParameters) { 
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) { vTaskDelete(NULL); }
    dsps_wind_hann_f32(window_coefficients, SAMPLES);

    static int32_t in_block[HOP_SIZE];
    static int32_t out_block[HOP_SIZE];
    static float ola_buffer[SAMPLES] = {0};
    static float prev_energy = 0.0f;

    for(;;) {
        size_t bytes_read;
        i2s_channel_read(rx_chan, in_block, sizeof(in_block), &bytes_read, portMAX_DELAY);

        if (bytes_read > 0) {
            const bool _frozen = isFrozen;
            const bool _harmonizer = isHarmonizerMode;
            const bool _synth = isSynthMode;
            const bool _pad = isPadMode;
            const float _feedback = feedbackRamp;
            const float _pitchFactor = pitchShiftFactor;

            float current_energy = 0.0f;
            float peak_val = 0.0f; 
            for (int i = 0; i < HOP_SIZE; i++) {
                float sample = (float)in_block[i] / 8388608.0f; 
                current_energy += sample * sample; 
                if (fabsf(sample) > peak_val) peak_val = fabsf(sample); 
            }
            
            if (peak_val > ui_audio_level) ui_audio_level = peak_val; 
            else ui_audio_level *= 0.96f; 

            bool is_transient = (current_energy > 0.005f) && (current_energy > prev_energy * 5.0f);
            prev_energy = current_energy;

            // --- SYNTH PRE-PROCESSING: HARD CLIPPING ---
            if (_synth) {
                for (int i = 0; i < HOP_SIZE; i++) {
                    float val = (float)in_block[i] / 8388608.0f;
                    val *= 30.0f; // Extreme gain for fuzz square wave
                    if (val > 0.8f) val = 0.8f;
                    if (val < -0.8f) val = -0.8f;
                    in_block[i] = (int32_t)(val * 8388608.0f);
                }
            }

            memmove(fft_buffer, &fft_buffer[HOP_SIZE * 2], (SAMPLES - HOP_SIZE) * 2 * sizeof(float));
            for (int i = 0; i < HOP_SIZE; i++) {
                fft_buffer[(SAMPLES - HOP_SIZE + i) * 2] = (float)in_block[i] / 8388608.0f; 
                fft_buffer[(SAMPLES - HOP_SIZE + i) * 2 + 1] = 0.0f; 
            }

            dsps_mul_f32(fft_buffer, window_coefficients, fft_buffer, SAMPLES, 2, 1, 2);
            for (int i = 0; i < SAMPLES; i++) { fft_buffer[i * 2 + 1] = 0; }

            dsps_fft2r_fc32(fft_buffer, SAMPLES);
            dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

            float newMag[SAMPLES / 2] = {0}, newPhase[SAMPLES / 2] = {0};
            int lastPeak = 0; 
            const float FREQ_CONST = (2.0f * PI) / (float)SAMPLES;
            const float INV_HOP = 1.0f / (float)HOP_SIZE;

            for (int i = 1; i < (SAMPLES / 2) - 1; i++) {
                float re = fft_buffer[i * 2], im = fft_buffer[i * 2 + 1];
                float liveMag = fast_mag(re, im);
                
                // SYNTH LPF: Remove harsh high-end digital fizz
                if (_synth && i > (SAMPLES / 2) * 0.15f) liveMag *= 0.1f;

                // PAD BUFFER: Infinite sustain accumulation
                if (_pad) padMagBuffer[i] = (padMagBuffer[i] * 0.92f) + (liveMag * 0.15f);
                else padMagBuffer[i] = 0.0f;

                if (!_frozen && _feedback == 0.0f) { frozenMag[i] = liveMag; }

                float pM = _frozen ? frozenMag[i-1] : fast_mag(fft_buffer[(i-1)*2], fft_buffer[(i-1)*2+1]);
                float nM = _frozen ? frozenMag[i+1] : fast_mag(fft_buffer[(i+1)*2], fft_buffer[(i+1)*2+1]);
                float currentMagForPeak = _frozen ? frozenMag[i] : liveMag;
                if (currentMagForPeak > pM && currentMagForPeak > nM) lastPeak = i;

                float phase;
                float deltaPhase = 0.0f;
                float binFreq = (float)lastPeak * FREQ_CONST;

                if (unlikely(_frozen)) {
                    phase = prevAnalysisPhase[i] + (binFreq * HOP_SIZE);
                    prevAnalysisPhase[i] = phase;
                } else {
                    float abs_re = fabsf(re), abs_im = fabsf(im);
                    float abs_re_plus = abs_re + 1e-7f; 
                    if (abs_re >= abs_im) {
                        float r = abs_im / abs_re_plus;
                        phase = r * (0.9724f - 0.1919f * r * r); 
                    } else {
                        float r = abs_re / (abs_im + 1e-7f);
                        phase = PI / 2.0f - r * (0.9724f - 0.1919f * r * r);
                    }
                    if (re < 0) phase = PI - phase;
                    if (im < 0) phase = -phase;
                    deltaPhase = (phase - prevAnalysisPhase[i]) - (binFreq * HOP_SIZE);
                    prevAnalysisPhase[i] = phase; 
                }
                
                // MAIN PATH (Dry for Pad/Harmonizer, Shifted for Synth/Whammy)
                float mainFactor = (_harmonizer || _pad) ? 1.0f : _pitchFactor;
                int targetBin = (int)((i * mainFactor) + 0.5f);
                if (targetBin < SAMPLES / 2) {
                    newMag[targetBin] += _frozen ? frozenMag[i] : liveMag;
                    if (unlikely(is_transient)) {
                        newPhase[targetBin] = phase; 
                    } else {
                        float shiftedFreq = (binFreq + (deltaPhase * INV_HOP)) * mainFactor;
                        newPhase[targetBin] = prevSynthesisPhase[targetBin] + (shiftedFreq * HOP_SIZE);
                    }
                    prevSynthesisPhase[targetBin] = newPhase[targetBin];
                }

                // PAD PATH
                if (_pad) {
                    int padTargetBin = (int)((i * _pitchFactor) + 0.5f);
                    if (padTargetBin < SAMPLES / 2) {
                        newMag[padTargetBin] += padMagBuffer[i];
                        if (unlikely(is_transient)) {
                            newPhase[padTargetBin] = phase;
                        } else {
                            float padShiftedFreq = (binFreq + (deltaPhase * INV_HOP)) * _pitchFactor;
                            newPhase[padTargetBin] = prevSynthesisPhase[padTargetBin] + (padShiftedFreq * HOP_SIZE);
                        }
                        prevSynthesisPhase[padTargetBin] = newPhase[padTargetBin];
                    }
                }

                // FEEDBACK PATH
                if (_feedback > 0.0f) {
                    float fbFactor = _pitchFactor * (1.0f + _feedback); 
                    int fbTargetBin = (int)((i * fbFactor) + 0.5f);
                    if (fbTargetBin < SAMPLES / 2) {
                        newMag[fbTargetBin] += (frozenMag[i] * _feedback); 
                        if (unlikely(is_transient)) {
                            newPhase[fbTargetBin] = phase;
                        } else {
                            float fbShiftedFreq = (binFreq * fbFactor);
                            newPhase[fbTargetBin] = prevSynthesisPhase[fbTargetBin] + (fbShiftedFreq * HOP_SIZE);
                        }
                        prevSynthesisPhase[fbTargetBin] = newPhase[fbTargetBin];
                    }
                }

                // HARMONY PATH
                if (_harmonizer && fabsf(_pitchFactor - 1.0f) > 0.001f) {
                    int harmTargetBin = (int)((i * _pitchFactor) + 0.5f);
                    if (harmTargetBin < SAMPLES / 2) {
                        newMag[harmTargetBin] += _frozen ? frozenMag[i] : liveMag; 
                        if (unlikely(is_transient)) {
                            newPhase[harmTargetBin] = phase;
                        } else {
                            float harmShiftedFreq = (binFreq + (deltaPhase * INV_HOP)) * _pitchFactor;
                            newPhase[harmTargetBin] = prevSynthesisPhase[harmTargetBin] + (harmShiftedFreq * HOP_SIZE);
                        }
                        prevSynthesisPhase[harmTargetBin] = newPhase[harmTargetBin];
                    }
                }
            }

            #pragma GCC unroll 4
            for (int i = 0; i < SAMPLES / 2; i++) {
                fft_buffer[i * 2] = newMag[i] * fast_cos(newPhase[i]);
                fft_buffer[i * 2 + 1] = newMag[i] * fast_sin(newPhase[i]);
            }
            dsps_fft2r_fc32(fft_buffer, SAMPLES);
            dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

            for (int i = 0; i < SAMPLES; i++) { ola_buffer[i] += fft_buffer[i * 2]; }

            // --- DYNAMIC HEADROOM MANAGEMENT ---
            float dynamicHeadroom = 1.0f; 
            if (_synth) dynamicHeadroom = 0.60f;
            else if (_pad) dynamicHeadroom = 0.50f;
            else if (_harmonizer) dynamicHeadroom = 0.55f; 
            else if (_feedback > 0.0f) dynamicHeadroom = 1.0f / (1.0f + _feedback); 

            float scale = 8388607.0f * dynamicHeadroom;
            dsps_mul_f32(ola_buffer, &scale, ola_buffer, HOP_SIZE, 1, 0, 1);

            float out_peak = 0.0f; 

            #pragma GCC unroll 4
            for (int i = 0; i < HOP_SIZE; i++) {
                float scaled_sample = ola_buffer[i];
                if (scaled_sample > 8388607.0f) scaled_sample = 8388607.0f;
                if (scaled_sample < -8388608.0f) scaled_sample = -8388608.0f;
                out_block[i] = (int32_t)scaled_sample; 

                float abs_out = fabsf(scaled_sample) / 8388608.0f;
                if (abs_out > out_peak) out_peak = abs_out; 
            }

            if (out_peak > ui_output_level) ui_output_level = out_peak; 
            else ui_output_level *= 0.96f; 

            memmove(ola_buffer, &ola_buffer[HOP_SIZE], (SAMPLES - HOP_SIZE) * sizeof(float));
            memset(&ola_buffer[SAMPLES - HOP_SIZE], 0, HOP_SIZE * sizeof(float));

            size_t bw;
            i2s_channel_write(tx_chan, out_block, sizeof(out_block), &bw, portMAX_DELAY);
        }
    }
}

// --- SCREEN RENDER LOGIC ---
void updateDisplay() {
    spr.fillSprite(TFT_BLACK); 
    spr.setTextDatum(MC_DATUM);

    int barWidth = 8;
    int barHeight = 100;
    int barY = 30; 

    // IN Bar
    int inX = 10;
    int inFillHeight = (int)(ui_audio_level * barHeight);
    if (inFillHeight > barHeight) inFillHeight = barHeight;
    uint32_t inColor = (ui_audio_level > 0.90f) ? TFT_RED : TFT_GREEN;
    spr.drawRect(inX, barY, barWidth, barHeight, TFT_DARKGREY);
    spr.fillRect(inX, barY + (barHeight - inFillHeight), barWidth, inFillHeight, inColor);
    spr.setTextSize(1);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString("IN", inX + (barWidth/2), barY + barHeight + 10);

    // OUT Bar 
    int outX = spr.width() - 18;
    int outFillHeight = (int)(ui_output_level * barHeight);
    if (outFillHeight > barHeight) outFillHeight = barHeight;
    uint32_t outColor = (ui_output_level > 0.90f) ? TFT_RED : TFT_GREEN;
    spr.drawRect(outX, barY, barWidth, barHeight, TFT_DARKGREY);
    spr.fillRect(outX, barY + (barHeight - outFillHeight), barWidth, outFillHeight, outColor);
    spr.drawString("OUT", outX + (barWidth/2), barY + barHeight + 10);

    // --- ACTIVE EFFECT LED ---
    bool isCurrentEffectActive = false;
    switch(activeEffectMode) {
        case 0: isCurrentEffectActive = true; break; 
        case 1: isCurrentEffectActive = isFrozen; break;
        case 2: isCurrentEffectActive = isFeedbackActive; break;
        case 3: isCurrentEffectActive = isHarmonizerMode; break;
        case 4: isCurrentEffectActive = isCapoMode; break;
        case 5: isCurrentEffectActive = isSynthMode; break;
        case 6: isCurrentEffectActive = isPadMode; break;
    }
    uint32_t ledColor = isCurrentEffectActive ? TFT_GREEN : TFT_RED;
    spr.fillCircle(spr.width() - 40, 25, 8, ledColor); 
    spr.drawCircle(spr.width() - 40, 25, 8, TFT_WHITE);

    // --- EFFECT TITLES ---
    spr.setTextSize(3);
    switch(activeEffectMode) {
        case 0:
            spr.setTextColor(TFT_ORANGE, TFT_BLACK);
            spr.drawString("WHAMMY", spr.width() / 2 + 30, 40); 
            break;
        case 1:
            spr.setTextColor(TFT_CYAN, TFT_BLACK);
            spr.drawString("FREEZE", spr.width() / 2, 40);
            break;
        case 2:
            spr.setTextColor(TFT_RED, TFT_BLACK);
            spr.drawString("FEEDBACK", spr.width() / 2, 40);
            break;
        case 3:
            spr.setTextColor(TFT_MAGENTA, TFT_BLACK);
            spr.drawString("HARMONY", spr.width() / 2, 40);
            break;
        case 4:
            spr.setTextColor(TFT_GREEN, TFT_BLACK);
            spr.drawString("CAPO", spr.width() / 2, 40);
            break;
        case 5:
            spr.setTextColor(TFT_YELLOW, TFT_BLACK);
            spr.drawString("SYNTH", spr.width() / 2, 40);
            break;
        case 6:
            spr.setTextColor(TFT_PINK, TFT_BLACK);
            spr.drawString("PAD", spr.width() / 2, 40);
            break;
    }

    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // --- DETERMINE DISPLAY VALUE ---
    float displayVal = effectMemory[activeEffectMode];
    if (activeEffectMode == 5) displayVal = effectMemory[6];
    else if (activeEffectMode == 6) displayVal = effectMemory[7];

    if (activeEffectMode == 0) { 
        spr.setTextSize(1);
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        int lineTop = 30, lineBot = 130;
        int x1 = 35, x2 = 70, x3 = 105; 
        
        spr.drawString("PB1", x1, lineBot + 15);
        spr.drawString("PB2", x2, lineBot + 15);
        spr.drawString("CC11", x3, lineBot + 15);
        
        for (int y = lineTop; y <= lineBot; y += 5) {
            spr.drawFastVLine(x1, y, 2, TFT_DARKGREY);
            spr.drawFastVLine(x2, y, 2, TFT_DARKGREY);
            spr.drawFastVLine(x3, y, 2, TFT_DARKGREY);
        }
        
        int y1 = map(currentPB1, 0, 16383, lineBot, lineTop);
        int y2 = map(currentPB2, 0, 16383, lineBot, lineTop);
        int y3 = map(currentCC11, 0, 16383, lineBot, lineTop);
        
        spr.fillCircle(x1, y1, 4, TFT_CYAN);
        spr.fillCircle(x2, y2, 4, TFT_MAGENTA);
        spr.fillCircle(x3, y3, 4, TFT_GREEN);

        spr.setTextSize(3); 
        char topStr[16]; char botStr[16];
        float toeVal = effectMemory[0];
        float heelVal = effectMemory[5]; 
        int textX = spr.width() / 2 + 35; 
        
        sprintf(topStr, "%s%.1f", toeVal > 0 ? "+" : "", toeVal);
        spr.drawString(topStr, textX, (spr.height() / 2) - 15);
        
        sprintf(botStr, "%s%.1f", heelVal > 0 ? "+" : "", heelVal);
        spr.drawString(botStr, textX, (spr.height() / 2) + 15);

    } else { 
        spr.setTextSize(4); 
        char intervalStr[16];
        sprintf(intervalStr, "%s%.1f", displayVal > 0 ? "+" : "", displayVal);
        spr.drawString(intervalStr, spr.width() / 2, spr.height() / 2 + 10);
    }

    spr.setTextSize(3);
    if (feedbackRamp > 0.0f && activeEffectMode != 2) {
        spr.setTextColor(TFT_RED, TFT_BLACK);
        spr.drawString("* SCREAMING *", spr.width() / 2, spr.height() - 70);
        int swellWidth = (int)(feedbackRamp * 100); 
        int swellX = (spr.width() / 2) - 50; 
        spr.drawRect(swellX, spr.height() - 55, 100, 6, TFT_DARKGREY); 
        spr.fillRect(swellX, spr.height() - 55, swellWidth, 6, TFT_RED); 
    }
    
    if (isFrozen && activeEffectMode != 1) {
        spr.setTextColor(TFT_CYAN, TFT_BLACK);
        spr.drawString("* FROZEN *", spr.width() / 2, spr.height() - 40);
    }
    
    if (activeEffectMode == 0) {
        spr.setTextSize(1); 
        int btX = spr.width() / 2 + 40; 
        if (btmidi.isConnected()) {
            spr.setTextColor(TFT_GREEN, TFT_BLACK);
            spr.drawString("BT: Connected", btX, spr.height() - 15);
        } else {
            spr.setTextColor(TFT_YELLOW, TFT_BLACK);
            spr.drawString("BT: Waiting", btX, spr.height() - 15);
        }
    } else {
        spr.setTextSize(2); 
        if (btmidi.isConnected()) {
            spr.setTextColor(TFT_GREEN, TFT_BLACK);
            spr.drawString("BT: Connected", spr.width() / 2, spr.height() - 20);
        } else {
            spr.setTextColor(TFT_YELLOW, TFT_BLACK);
            spr.drawString("BT: Waiting", spr.width() / 2, spr.height() - 20);
        }
    }
    spr.pushSprite(0, 0); 
} 

void DisplayTask(void * pvParameters) {
    for(;;) {
        if (forceUIUpdate || (!isScreenOff && (ui_audio_level > 0.02f || ui_output_level > 0.02f))) {
            updateDisplay();
            forceUIUpdate = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void MidiTask(void * pvParameters) {
    static bool lastBtState = false; 
    static analog_t lastMidiA = 8192; 
    static analog_t lastMidiB = 8192; 
    
    lastActivityTime = millis(); 
    lastScreenActivityTime = millis(); 

    for(;;) {
        if (pitchShiftLUT[8192] == 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        Control_Surface.loop();

        if (isFeedbackActive && feedbackRamp < 1.0f) {
            feedbackRamp += 0.02f;
            if (feedbackRamp > 1.0f) feedbackRamp = 1.0f;
            forceUIUpdate = true;
        } else if (!isFeedbackActive && feedbackRamp > 0.0f) {
            feedbackRamp -= 0.02f;
            if (feedbackRamp < 0.0f) feedbackRamp = 0.0f;
            forceUIUpdate = true;
        }

        bool btConnected = btmidi.isConnected();
        if (btConnected != lastBtState) {
            if (!isScreenOff) forceUIUpdate = true;
            lastBtState = btConnected;
            lastActivityTime = millis(); 
        }

        if (!isScreenOff && (millis() - lastScreenActivityTime > SCREEN_OFF_TIMEOUT)) {
            turnScreenOff();
        }
        if (!btConnected && (millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT)) {
            goToLightSleep();
            lastScreenActivityTime = millis(); 
        }

        // --- 1. CAROUSEL BUTTON (IO14) LOGIC ---
        bool currentCarouselState = digitalRead(CAROUSEL_BUTTON_PIN);
        if (currentCarouselState == LOW && lastCarouselState == HIGH) {
            carouselPressedTime = millis();
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        else if (currentCarouselState == HIGH && lastCarouselState == LOW) {
            unsigned long pressDuration = millis() - carouselPressedTime;
            
            if (pressDuration < 400) {
                activeEffectMode++;
                if (activeEffectMode > 6) activeEffectMode = 0;
                
                isHarmonizerMode = false; isCapoMode = false;
                isFeedbackActive = false; feedbackRamp = 0.0f; isFrozen = false; 
                isSynthMode = false; isPadMode = false;

                switch(activeEffectMode) {
                    case 1: isFrozen = true; break;
                    case 2: isFeedbackActive = true; break; 
                    case 3: isHarmonizerMode = true; break;
                    case 4: isCapoMode = true; break;
                    case 5: isSynthMode = true; break;
                    case 6: isPadMode = true; break;
                }
            } else {
                int memSlot = activeEffectMode;
                if (activeEffectMode == 0) memSlot = 0; // Toe
                else if (activeEffectMode == 5) memSlot = 6;
                else if (activeEffectMode == 6) memSlot = 7;

                if (pressDuration < 1500) {
                    effectMemory[memSlot] += 1.0f; 
                } else {
                    effectMemory[memSlot] -= 1.0f; 
                }
                if (effectMemory[memSlot] > 24.0f) effectMemory[memSlot] = 24.0f;
                if (effectMemory[memSlot] < -24.0f) effectMemory[memSlot] = -24.0f;
            }
            updateLUT(); forceUIUpdate = true; lastActivityTime = millis();
        }
        lastCarouselState = currentCarouselState;

        // --- 2. BOOT BUTTON (GPIO 0) LOGIC ---
        bool currentBootState = digitalRead(0);
        if (currentBootState == LOW && lastBootState == HIGH) {
            bootPressedTime = millis();
            vTaskDelay(pdMS_TO_TICKS(50));
        } else if (currentBootState == HIGH && lastBootState == LOW) {
            unsigned long pressDuration = millis() - bootPressedTime;
            
            if (pressDuration >= 400) { 
                int memSlot = activeEffectMode;
                if (activeEffectMode == 0) memSlot = 5; // Heel
                else if (activeEffectMode == 5) memSlot = 6;
                else if (activeEffectMode == 6) memSlot = 7;

                if (pressDuration < 1500) {
                    effectMemory[memSlot] -= 1.0f; 
                } else {
                    effectMemory[memSlot] += 1.0f; 
                }
                if (effectMemory[memSlot] > 24.0f) effectMemory[memSlot] = 24.0f;
                if (effectMemory[memSlot] < -24.0f) effectMemory[memSlot] = -24.0f;
                
                updateLUT(); forceUIUpdate = true;
            }
            lastActivityTime = millis();
        }
        lastBootState = currentBootState;

        // Freeze Button
        bool currentButtonState = digitalRead(FREEZE_BUTTON_PIN);
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            isFrozen = !isFrozen; forceUIUpdate = true; lastActivityTime = millis();
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        lastButtonState = currentButtonState;
    
        if (currentCarouselState == LOW || currentBootState == LOW || currentButtonState == LOW) {
            lastActivityTime = millis();
            lastScreenActivityTime = millis(); 
            if (isScreenOff) turnScreenOn();   
        }
    
        filterPB.update();
        filterPB2.update();

        analog_t raw14_A = map(filterPB.getValue(), 0, 4095, 0, 16383);
        analog_t raw14_B = map(filterPB2.getValue(), 0, 4095, 0, 16383);

        // --- RESTORED: Map using independent variables for perfect alignment! ---
        analog_t calibratedA = map_PB(raw14_A, PBcenter1, PBdeadzone1, PBwasOffCenter1);
        analog_t calibratedB = map_PB(raw14_B, PBcenter2, PBdeadzone2, PBwasOffCenter2);

        currentPB1 = calibratedA;
        currentPB2 = calibratedB;

        // --- RESTORED: Actually check if pedals moved! ---
        bool movedA = abs((int)calibratedA - (int)lastMidiA) > 8;
        bool movedB = abs((int)calibratedB - (int)lastMidiB) > 8; 

        if (movedA || movedB) {
            if (isScreenOff) turnScreenOn(); 
            lastScreenActivityTime = millis(); 

            analog_t activeMidi = movedB ? calibratedB : calibratedA;
            pitchShiftFactor = pitchShiftLUT[constrain(activeMidi, 0, 16383)];
            Control_Surface.sendPitchBend(Channel_1, activeMidi);

            lastMidiA = calibratedA;
            lastMidiB = calibratedB;
            lastMidiSent = activeMidi;
            
            forceUIUpdate = true; 
        }
        vTaskDelay(pdMS_TO_TICKS(5)); 
    } 
} 

void init_i2s_modern() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_FREQUENCY),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)10, 
            .ws   = (gpio_num_t)12,
            .dout = (gpio_num_t)16,
            .din  = (gpio_num_t)17,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    i2s_channel_init_std_mode(tx_chan, &std_cfg);
    i2s_channel_init_std_mode(rx_chan, &std_cfg);
    i2s_channel_enable(tx_chan);
    i2s_channel_enable(rx_chan);
}

bool channelMessageCallback(ChannelMessage cm) {
    int newMode = -1;

    if (cm.header == 0xC0) {
        uint8_t pcValue = cm.data1; 
        if (pcValue >= 0 && pcValue <= 6) {
            newMode = pcValue; 
            Serial.printf("Received PC: Switching to Effect %d\n", newMode);
        }
    }
    else if (cm.header == 0xB0) {
        uint8_t ccNumber = cm.data1; 
        uint8_t ccValue = cm.data2;  

        if (ccNumber == 3) {
            if (ccValue == 127) {
                newMode = activeEffectMode + 1;
                if (newMode > 6) newMode = 0; 
                Serial.printf("Received CC 3: Advancing Carousel to Effect %d\n", newMode);
            }
        }
        else if (ccNumber == 110) {
            if (ccValue == 127) {
                isFrozen = true;
            } else if (ccValue == 0) {
                isFrozen = false;
            }
            forceUIUpdate = true;
            if (isScreenOff) turnScreenOn(); 
            lastActivityTime = millis(); 
            lastScreenActivityTime = millis();
        }
        else if (ccNumber == 111) {
            if (ccValue == 127) {
                isFeedbackActive = true;
            } else if (ccValue == 0) {
                isFeedbackActive = false;
            }
            forceUIUpdate = true;
            if (isScreenOff) turnScreenOn(); 
            lastActivityTime = millis(); 
            lastScreenActivityTime = millis();
        }
        else if (ccNumber == 11) {
            uint16_t mappedMidi = map(ccValue, 0, 127, 0, 16383);
            currentCC11 = mappedMidi; 
            pitchShiftFactor = pitchShiftLUT[mappedMidi];
            lastMidiSent = mappedMidi;
            forceUIUpdate = true; 
            if (isScreenOff) turnScreenOn(); 
            lastActivityTime = millis(); 
            lastScreenActivityTime = millis();
        }
    }

    if (newMode != -1 && newMode != activeEffectMode) {
        activeEffectMode = newMode;
        isHarmonizerMode = false;
        isCapoMode = false;
        isFeedbackActive = false; 
        feedbackRamp = 0.0f;
        isFrozen = false;
        isSynthMode = false;
        isPadMode = false;

       switch(activeEffectMode) {
            case 0: break; 
            case 1: break; 
            case 2: break; 
            case 3: isHarmonizerMode = true; break;
            case 4: isCapoMode = true; break;
            case 5: isSynthMode = true; break;
            case 6: isPadMode = true; break;
        }

        updateLUT();
        forceUIUpdate = true; 
        if (isScreenOff) turnScreenOn(); 
        lastActivityTime = millis(); 
        lastScreenActivityTime = millis();
    }
    return false; 
}

void setup() {
    pinMode(38, OUTPUT); digitalWrite(38, LOW); 
    pinMode(15, OUTPUT); digitalWrite(15, HIGH); 

    Serial.begin(SERIAL_BAUDRATE);

    tft.init();
    tft.setRotation(1); 
    spr.createSprite(tft.width(), tft.height());

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("BOOTING...", tft.width() / 2, tft.height() / 2);

    delay(120);
    digitalWrite(38, HIGH);

    btmidi.setName("Whammy_S3");
    pinMode(CAROUSEL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FREEZE_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(0, INPUT_PULLUP); 

    // --- OVERRIDES REMOVED: Hardware pedals are completely active! ---
    // pinMode(pinPB, INPUT_PULLUP);  
    // pinMode(pinPB2, INPUT_PULLUP); 
    
    FilteredAnalog<>::setupADC();
    Control_Surface >> pipes >> btmidi;
    Control_Surface >> pipes >> usbmidi;
    btmidi >> pipes >> Control_Surface;
    usbmidi >> pipes >> Control_Surface;
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr);
    Control_Surface.begin();

    init_i2s_modern();
    calibrateCenterAndDeadzone();
    Control_Surface.sendPitchBend(Channel_1, 8192);
    lastMidiSent = 8192;
    
    initTrigLUT();
    updateLUT();

    xTaskCreatePinnedToCore(DisplayTask, "Display", 8192, NULL, 1, NULL, 0); 
    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 2, NULL, 0);       
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);
}

void loop() { }