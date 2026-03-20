#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <dsps_fft2r.h>   
#include <dsps_wind.h>
#include <driver/i2s.h>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "dsps_mul.h"
#include <TFT_eSPI.h>

// --- THE LOOKUP TABLE ---
float pitchShiftLUT[16384]; // 64KB array to hold all possible pedal multipliers

// --- TFT DISPLAY OBJECT ---
TFT_eSPI tft = TFT_eSPI();

// --- GitHub Configuration & Constants ---
#define SAMPLES 512             
#define HOP_SIZE 64            
#define SAMPLING_FREQUENCY 96000 
#define SERIAL_BAUDRATE 115200
#define FORCE_CENTER_UPDATE_DELAY 250
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
QueueHandle_t i2s_event_queue;

// --- MIDI & Full Calibration Suite (Core 0) ---
pin_t pinPB = A0;
const int FREEZE_BUTTON_PIN = 2; 
const int INTERVAL_BUTTON_PIN = 43; // Safely moved away from screen pins
const int FEEDBACK_BUTTON_PIN = 44; // Safely moved away from screen pins

bool lastButtonState = HIGH;
bool lastIntervalButtonState = HIGH;
unsigned long intervalButtonPressedTime = 0;
float maxSemitones = 12.0f; 

const int HARMONY_BUTTON_PIN = 3;  // Safely moved away from screen pins
bool lastHarmonyButtonState = HIGH;
volatile bool isHarmonizerMode = false; 

volatile float feedbackRamp = 0.0f;

const int CAPO_BUTTON_PIN = 10;
bool lastCapoButtonState = HIGH;
volatile bool isCapoMode = false;
float capoSemitones = -2.0f; 

BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<2> pipes;
Bank<16> bankChannel;
FilteredAnalog<12, 2, uint32_t, uint32_t> filterPB = pinPB;
uint16_t lastMidiSent = 8192;

// Calibration Variables
double PBdeadzoneMultiplier = 14;
double PBdeadzoneMinimum = 950;
double PBdeadzoneMaximum = 1600;
analog_t PBcenter = 8192;
analog_t PBdeadzone = PBdeadzoneMinimum;
analog_t PBminimumValue = 0;
analog_t PBmaximumValue = 16383;
bool PBwasOffCenter = false;
long PBlastCenteredOn = millis();
static bool lockedAtMin = false;
static bool lockedAtMax = false;

// --- Full Mapping Logic ---
analog_t map_PB_Full(analog_t raw) {
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    if (raw <= PBminimumValue + 150) { 
        PBwasOffCenter = true;
        return 0; 
    }
    if (raw >= PBmaximumValue - 150) {
        PBwasOffCenter = true;
        return 16383; 
    }
    
    if (raw <= PBcenter - PBdeadzone) {
        PBwasOffCenter = true;
        return map(raw, PBminimumValue, PBcenter - PBdeadzone, 0, 8191);
    }
    else if (raw >= PBcenter + PBdeadzone) {
        PBwasOffCenter = true;
        return map(raw, PBcenter + PBdeadzone, PBmaximumValue, 8191, 16383);
    }
    else {
        return 8192; 
    }
}

void calibrateCenterAndDeadzone() {
  Serial.println("Calibrating Center and Deadzones...");
  for(int i=0; i<50; i++) { filterPB.update(); delay(1); }

  int iNumberOfSamples = 750;
  analog_t calibPBLow = 16383; 
  analog_t calibPBHigh = 0; 
  long lSampleSumPB = 0;

  for (int iSample = 1; iSample <= iNumberOfSamples; iSample++) {
    filterPB.update(); 
    analog_t raw12 = filterPB.getValue();
    analog_t calibPB = map(raw12, 0, 4095, 0, 16383); 
    
    lSampleSumPB += calibPB;
    if (calibPB < calibPBLow) { calibPBLow = calibPB; } 
    if (calibPB > calibPBHigh) { calibPBHigh = calibPB; } 
    delay(1);
  }
  
  PBcenter = lSampleSumPB / iNumberOfSamples;
  Serial.print("PB Center: "); Serial.println(PBcenter);

  PBdeadzone = (analog_t)constrain(((calibPBHigh - calibPBLow) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
  Serial.print("PB Deadzone: "); Serial.println(PBdeadzone);
}

void updateLUT() {
    for (int i = 0; i < 16384; i++) {
        float normalizedThrow = (float(i) - 8192.0f) / 8192.0f;
        float currentWhammyShift = maxSemitones * normalizedThrow;
        float currentCapoShift = isCapoMode ? capoSemitones : 0.0f;
        float totalSemitones = currentCapoShift + currentWhammyShift;
        pitchShiftLUT[i] = powf(2.0f, totalSemitones / 12.0f);
    }
    Serial.println("System: DSP Math Table Recalculated!");
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

void initTrigLUT() {
    for (int i = 0; i < TRIG_LUT_SIZE; i++) {
        sinLUT[i] = sinf((float)i / (float)TRIG_LUT_SIZE * 2.0f * PI);
    }
    Serial.println("System: Trigonometry LUT Initialized!");
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

// --- CORE 1: PHASE-LOCKED DSP TASK ---
void IRAM_ATTR AudioDSPTask(void * pvParameters) { 
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) { vTaskDelete(NULL); }
    
    dsps_wind_hann_f32(window_coefficients, SAMPLES);

    static int16_t in_block[HOP_SIZE];
    static int16_t out_block[HOP_SIZE];
    static float ola_buffer[SAMPLES] = {0};

    for(;;) {
        i2s_event_t event;
        if (xQueueReceive(i2s_event_queue, &event, portMAX_DELAY) == pdPASS) {
            if (event.type == I2S_EVENT_RX_DONE) {
                
                const bool _frozen = isFrozen;
                const bool _harmonizer = isHarmonizerMode;
                const float _feedback = feedbackRamp;
                const float _pitchFactor = pitchShiftFactor;

                memmove(fft_buffer, &fft_buffer[HOP_SIZE * 2], (SAMPLES - HOP_SIZE) * 2 * sizeof(float));
                size_t bytes_read;
                i2s_read(I2S_NUM_0, in_block, sizeof(in_block), &bytes_read, portMAX_DELAY);

                for (int i = 0; i < HOP_SIZE; i++) {
                    fft_buffer[(SAMPLES - HOP_SIZE + i) * 2] = (float)in_block[i] / 32768.0f; 
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
                    
                    float mainFactor = _harmonizer ? 1.0f : _pitchFactor;
                    int targetBin = (int)((i * mainFactor) + 0.5f);
                    if (targetBin < SAMPLES / 2) {
                        newMag[targetBin] += _frozen ? frozenMag[i] : liveMag;
                        float shiftedFreq = (binFreq + (deltaPhase * INV_HOP)) * mainFactor;
                        newPhase[targetBin] = prevSynthesisPhase[targetBin] + (shiftedFreq * HOP_SIZE);
                        prevSynthesisPhase[targetBin] = newPhase[targetBin];
                    }

                    if (_feedback > 0.0f) {
                        float fbFactor = _pitchFactor * (1.0f + _feedback); 
                        int fbTargetBin = (int)((i * fbFactor) + 0.5f);
                        if (fbTargetBin < SAMPLES / 2) {
                            newMag[fbTargetBin] += (frozenMag[i] * _feedback); 
                            float fbShiftedFreq = (binFreq * fbFactor);
                            newPhase[fbTargetBin] = prevSynthesisPhase[fbTargetBin] + (fbShiftedFreq * HOP_SIZE);
                            prevSynthesisPhase[fbTargetBin] = newPhase[fbTargetBin];
                        }
                    }

                    if (_harmonizer && fabsf(_pitchFactor - 1.0f) > 0.001f) {
                        int harmTargetBin = (int)((i * _pitchFactor) + 0.5f);
                        if (harmTargetBin < SAMPLES / 2) {
                            newMag[harmTargetBin] += _frozen ? frozenMag[i] : liveMag; 
                            float harmShiftedFreq = (binFreq + (deltaPhase * INV_HOP)) * _pitchFactor;
                            newPhase[harmTargetBin] = prevSynthesisPhase[harmTargetBin] + (harmShiftedFreq * HOP_SIZE);
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

                for (int i = 0; i < SAMPLES; i++) {
                    ola_buffer[i] += fft_buffer[i * 2]; 
                }

                float scale = 32767.0f;
                dsps_mul_f32(ola_buffer, &scale, ola_buffer, HOP_SIZE, 1, 0, 1);

                #pragma GCC unroll 4
                for (int i = 0; i < HOP_SIZE; i++) {
                    float scaled_sample = ola_buffer[i];
                    if (scaled_sample > 32767.0f) scaled_sample = 32767.0f;
                    if (scaled_sample < -32768.0f) scaled_sample = -32768.0f;
                    out_block[i] = (int16_t)scaled_sample;
                }

                memmove(ola_buffer, &ola_buffer[HOP_SIZE], (SAMPLES - HOP_SIZE) * sizeof(float));
                memset(&ola_buffer[SAMPLES - HOP_SIZE], 0, HOP_SIZE * sizeof(float));

                size_t bw;
                i2s_write(I2S_NUM_0, out_block, sizeof(out_block), &bw, portMAX_DELAY);
            }
        }
    }
}

// --- CORE 0: MIDI & ADVANCED CONTROL ---
void MidiTask(void * pvParameters) {
    for(;;) {
        Control_Surface.loop();
        
        bool currentButtonState = digitalRead(FREEZE_BUTTON_PIN);
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            isFrozen = !isFrozen;
            if (isFrozen) { Serial.println("FREEZE ON - Snapshot Captured!"); } 
            else { Serial.println("FREEZE OFF - Back to Live Audio"); }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        lastButtonState = currentButtonState;

        bool currentHarmonyState = digitalRead(HARMONY_BUTTON_PIN);
        if (currentHarmonyState == LOW && lastHarmonyButtonState == HIGH) {
            isHarmonizerMode = !isHarmonizerMode;
            if (isHarmonizerMode) { Serial.println("HARMONY MODE ON"); } 
            else { Serial.println("WHAMMY MODE ON"); }
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        lastHarmonyButtonState = currentHarmonyState;

        bool currentCapoState = digitalRead(CAPO_BUTTON_PIN);
        if (currentCapoState == LOW && lastCapoButtonState == HIGH) {
            isCapoMode = !isCapoMode;
            updateLUT();
            if (isCapoMode) { Serial.println("CAPO MODE ON"); } 
            else { Serial.println("CAPO MODE OFF"); }
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        lastCapoButtonState = currentCapoState;

        bool currentIntervalButtonState = digitalRead(INTERVAL_BUTTON_PIN);
        if (currentIntervalButtonState == LOW && lastIntervalButtonState == HIGH) {
            intervalButtonPressedTime = millis();
            vTaskDelay(pdMS_TO_TICKS(50)); 
        } 
        else if (currentIntervalButtonState == HIGH && lastIntervalButtonState == LOW) {
            unsigned long pressDuration = millis() - intervalButtonPressedTime;
            if (pressDuration < 1000) {
                maxSemitones += 0.5f;
                if (maxSemitones > 24.0f) maxSemitones = 24.0f; 
                Serial.print("Interval Up: +/- "); Serial.println(maxSemitones);
                updateLUT();
            } else {
                maxSemitones -= 0.5f;
                if (maxSemitones < -24.0f) maxSemitones = -24.0f; 
                Serial.print("Interval Down: +/- "); Serial.println(maxSemitones);
                updateLUT();
            }
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        lastIntervalButtonState = currentIntervalButtonState;

        bool currentFbState = digitalRead(FEEDBACK_BUTTON_PIN);
      if (currentFbState == LOW) { 
            feedbackRamp = feedbackRamp + 0.01f; // Replaced += 
            if (feedbackRamp > 1.0f) feedbackRamp = 1.0f;
        } else { 
            feedbackRamp = feedbackRamp - 0.02f; // Replaced -= 
            if (feedbackRamp < 0.0f) feedbackRamp = 0.0f;
        }
    
        filterPB.update();
        analog_t raw12 = filterPB.getValue(); 
        analog_t raw14 = map(raw12, 0, 4095, 0, 16383);
        analog_t calibratedMidi = map_PB_Full(raw14);

        if (calibratedMidi != lastMidiSent) {
            Control_Surface.sendPitchBend(Channel_1, calibratedMidi);
            lastMidiSent = calibratedMidi;
        }

        if (calibratedMidi == 0) {
            if (!lockedAtMin) { lockedAtMin = true; Serial.println("[LIMIT: MIN]"); }
        } else { lockedAtMin = false; }

        if (calibratedMidi == 16383) { 
            if (!lockedAtMax) { lockedAtMax = true; Serial.println("[LIMIT: MAX]"); }
        } else { lockedAtMax = false; }

        if (calibratedMidi == 8192 && PBwasOffCenter) {
            PBwasOffCenter = false;
            Serial.println("[FORCE MIDI CENTER]");
        }

        int safeIndex = constrain(calibratedMidi, 0, 16383);
        pitchShiftFactor = pitchShiftLUT[safeIndex];
          
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000));

    Serial.println("--- SYSTEM BOOTING ---");

    // --- 1. TURN ON SCREEN HARDWARE ---
    pinMode(15, OUTPUT); 
    digitalWrite(15, HIGH);
    
    pinMode(38, OUTPUT); 
    digitalWrite(38, HIGH);

    // --- 2. INITIALIZE TFT ---
    tft.init();
    tft.setRotation(1); 
    
    // --- 3. DRAW THE UI ---
    tft.fillScreen(TFT_WHITE);              
    tft.setTextColor(TFT_BLACK, TFT_WHITE); 
    tft.setTextDatum(MC_DATUM);             
    tft.setTextSize(4);                     

    tft.drawString("HELLO", tft.width() / 2, tft.height() / 2, 1); 

    Serial.println("Screen Initialized and HELLO drawn!");

    btmidi.setName("Whammy_S3");
    pinMode(FREEZE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(INTERVAL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FEEDBACK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(HARMONY_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CAPO_BUTTON_PIN, INPUT_PULLUP);
    FilteredAnalog<>::setupADC();
    Control_Surface >> pipes >> btmidi;
    Control_Surface >> pipes >> usbmidi;
    Control_Surface.begin();
    
    calibrateCenterAndDeadzone();
    
    Control_Surface.sendPitchBend(Channel_1, 8192);
    lastMidiSent = 8192;

    i2s_config_t i2c = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
        .sample_rate = SAMPLING_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 4,
        .dma_buf_len = HOP_SIZE
    };

    i2s_driver_install(I2S_NUM_0, &i2c, 4, &i2s_event_queue);

    i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   
        .ws_io_num = 4,    
        .data_out_num = 16, 
        .data_in_num = 17   
    };

    i2s_set_pin(I2S_NUM_0, &pin_config);
    initTrigLUT();
    updateLUT();

    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}

void loop() {
    // The main loop is empty because all the work is done in the RTOS tasks
}