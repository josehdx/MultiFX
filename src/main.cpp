#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <dsps_fft2r.h>   
#include <dsps_wind.h>
#include <driver/i2s.h>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "dsps_mul.h"

// --- THE LOOKUP TABLE ---
float pitchShiftLUT[16384]; // 64KB array to hold all possible pedal multipliers


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
const int FREEZE_BUTTON_PIN = 2; // Set this to your desired GPIO (e.g., 0, 4, or 5)
const int INTERVAL_BUTTON_PIN = 8;
const int FEEDBACK_BUTTON_PIN = 9;

bool lastButtonState = HIGH;
bool lastIntervalButtonState = HIGH;
unsigned long intervalButtonPressedTime = 0;
float maxSemitones = 12.0f; // Defaults to +/- 1 Octave on boot

const int HARMONY_BUTTON_PIN = 6; 
bool lastHarmonyButtonState = HIGH;
volatile bool isHarmonizerMode = false; 

volatile float feedbackRamp = 0.0f;

const int CAPO_BUTTON_PIN = 10;
bool lastCapoButtonState = HIGH;
volatile bool isCapoMode = false;
float capoSemitones = -2.0f; // Set your fixed Capo interval here

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
    // raw is already 14-bit! No up-mapping needed.
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    // HARD FLOOR
    if (raw <= PBminimumValue + 150) { 
        PBwasOffCenter = true;
        return 0; 
    }
    
    // HARD CEILING
    if (raw >= PBmaximumValue - 150) {
        PBwasOffCenter = true;
        return 16383; 
    }
    
    // Deadzone Math
    if (raw <= PBcenter - PBdeadzone) {
        PBwasOffCenter = true;
        return map(raw, PBminimumValue, PBcenter - PBdeadzone, 0, 8191);
    }
    else if (raw >= PBcenter + PBdeadzone) {
        PBwasOffCenter = true;
        return map(raw, PBcenter + PBdeadzone, PBmaximumValue, 8191, 16383);
    }
    else {
        return 8192; // Solid center
    }
}

// --- GitHub Calibration Routine (All Serial Lines Restored) ---
void calibrateCenterAndDeadzone() {
  Serial.println("Calibrating Center and Deadzones...");
  
  // WARM UP THE FILTER SO IT DOESN'T START AT 0
  for(int i=0; i<50; i++) { filterPB.update(); delay(1); }

  int iNumberOfSamples = 750;
  analog_t calibPBLow = 16383; // Start at 14-bit max
  analog_t calibPBHigh = 0; 
  long lSampleSumPB = 0;

  for (int iSample = 1; iSample <= iNumberOfSamples; iSample++) {
    filterPB.update(); // Update the filter
   
    analog_t raw12 = filterPB.getValue();
    analog_t calibPB = map(raw12, 0, 4095, 0, 16383); 
    
    lSampleSumPB += calibPB;
    if (calibPB < calibPBLow) { calibPBLow = calibPB; } 
    if (calibPB > calibPBHigh) { calibPBHigh = calibPB; } 
    delay(1);
  }
  
  PBcenter = lSampleSumPB / iNumberOfSamples;
  Serial.print("PB Center: "); Serial.println(PBcenter);

  // No mapping needed because it's already 14-bit
  PBdeadzone = (analog_t)constrain(((calibPBHigh - calibPBLow) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
  Serial.print("PB Deadzone: "); Serial.println(PBdeadzone);
}


void updateLUT() {
    // Loop through every possible expression pedal position (0 to 16383)
    for (int i = 0; i < 16384; i++) {
        float normalizedThrow = (float(i) - 8192.0f) / 8192.0f;
        float currentWhammyShift = maxSemitones * normalizedThrow;
        float currentCapoShift = isCapoMode ? capoSemitones : 0.0f;
        float totalSemitones = currentCapoShift + currentWhammyShift;
        
        // Calculate the math heavily once, and store the answer
        pitchShiftLUT[i] = powf(2.0f, totalSemitones / 12.0f);
    }
    Serial.println("System: DSP Math Table Recalculated!");
}

// --- GitHub debugPrint() - Fully Restored ---
void debugPrint() {
    //Serial.print("Factor: "); Serial.print(pitchShiftFactor); Serial.print("\t");
  /* Serial.print("AR: ");
  Serial.print(analogRead(pinPB)); Serial.print("\t");
  Serial.print("PB Min/Cen/Max/Range/DZ: ");
  Serial.print(PBminimumValue); Serial.print(" ");
  Serial.print(PBcenter); Serial.print(" ");
  Serial.print(PBmaximumValue); Serial.print(" ");
  Serial.print(PBmaximumValue-PBminimumValue); Serial.print(" ");
  Serial.print(PBdeadzone); Serial.print(" ");
  Serial.print("PB OffCenter/Raw(14bit)/Val(12bit): ");
  Serial.print(PBwasOffCenter); Serial.print("\t");
  */
  
  Serial.println();
}

inline float IRAM_ATTR fast_mag(float re, float im) {
    float abs_re = fabsf(re);
    float abs_im = fabsf(im);
    float max_val = (abs_re > abs_im) ? abs_re : abs_im;
    float min_val = (abs_re < abs_im) ? abs_re : abs_im;
    return max_val + 0.337f * min_val;
}

// --- TRIGONOMETRY LUT ---
const int TRIG_LUT_SIZE = 4096; // 4096 entries is plenty for audio phase accuracy
float sinLUT[TRIG_LUT_SIZE];

void initTrigLUT() {
    for (int i = 0; i < TRIG_LUT_SIZE; i++) {
        sinLUT[i] = sinf((float)i / (float)TRIG_LUT_SIZE * 2.0f * PI);
    }
    Serial.println("System: Trigonometry LUT Initialized!");
}

// Super-fast 2-cycle Sine/Cosine
inline float IRAM_ATTR fast_sin(float phase) {
    // Fast phase wrapping to 0 -> 2PI range
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

    // --- STATIC ALLOCATION: Shaves microseconds off every frame ---
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
                    
                    // Path 1: Dry/Shifted
                    float mainFactor = _harmonizer ? 1.0f : _pitchFactor;
                    int targetBin = (int)((i * mainFactor) + 0.5f);
                    if (targetBin < SAMPLES / 2) {
                        newMag[targetBin] += _frozen ? frozenMag[i] : liveMag;
                        float shiftedFreq = (binFreq + (deltaPhase * INV_HOP)) * mainFactor;
                        newPhase[targetBin] = prevSynthesisPhase[targetBin] + (shiftedFreq * HOP_SIZE);
                        prevSynthesisPhase[targetBin] = newPhase[targetBin];
                    }

                    // Path 2: Feedback
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

                    // Path 3: Harmonizer
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
                    // --- FAST TRIG LUT IN USE ---
                    fft_buffer[i * 2] = newMag[i] * fast_cos(newPhase[i]);
                    fft_buffer[i * 2 + 1] = newMag[i] * fast_sin(newPhase[i]);
                }
                dsps_fft2r_fc32(fft_buffer, SAMPLES);
                dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

                for (int i = 0; i < SAMPLES; i++) {
                    ola_buffer[i] += fft_buffer[i * 2]; 
                }

                // --- VECTORIZED SIMD SCALING: Blitzes the whole array in 1 cycle ---
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
        
        // --- FREEZE BUTTON LOGIC ---
        bool currentButtonState = digitalRead(FREEZE_BUTTON_PIN);
        
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            isFrozen = !isFrozen;
            if (isFrozen) {
                Serial.println("FREEZE ON - Snapshot Captured!");
            } else {
                Serial.println("FREEZE OFF - Back to Live Audio");
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Simple debounce delay
        }
        lastButtonState = currentButtonState;


        // --- HARMONY MODE TOGGLE ---
        bool currentHarmonyState = digitalRead(HARMONY_BUTTON_PIN);
        if (currentHarmonyState == LOW && lastHarmonyButtonState == HIGH) {
            isHarmonizerMode = !isHarmonizerMode;
            if (isHarmonizerMode) {
                Serial.println("HARMONY MODE ON: Dry Signal + Sweeping Harmony");
            } else {
                Serial.println("WHAMMY MODE ON: Pitch Bend Only");
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        lastHarmonyButtonState = currentHarmonyState;

        // --- CAPO MODE TOGGLE ---
        bool currentCapoState = digitalRead(CAPO_BUTTON_PIN);
        if (currentCapoState == LOW && lastCapoButtonState == HIGH) {
            isCapoMode = !isCapoMode;
            updateLUT();
            if (isCapoMode) {
                Serial.println("CAPO MODE ON: Fixed Pitch Shift");
            } else {
                Serial.println("CAPO MODE OFF: Expression Pedal Active");
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        lastCapoButtonState = currentCapoState;


        // --- 2. INTERVAL BUTTON LOGIC (Short vs Long Press) ---
        bool currentIntervalButtonState = digitalRead(INTERVAL_BUTTON_PIN);
        
        if (currentIntervalButtonState == LOW && lastIntervalButtonState == HIGH) {
            // Button was just pressed, start the timer
            intervalButtonPressedTime = millis();
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        } 
        else if (currentIntervalButtonState == HIGH && lastIntervalButtonState == LOW) {
            // Button was just released, check how long it was held
            unsigned long pressDuration = millis() - intervalButtonPressedTime;
            
            if (pressDuration < 1000) {
                // Short Press: Increment by 0.5 semitones
                maxSemitones += 0.5f;
                if (maxSemitones > 24.0f) maxSemitones = 24.0f; // Max +/- 24 semitones
                Serial.print("Interval Up: +/- "); Serial.println(maxSemitones);
                updateLUT();
            } else {
                // Long Press (>= 1 second): Decrement by 0.5 semitones
                maxSemitones -= 0.5f;
                if (maxSemitones < -24.0f) maxSemitones = -24.0f; // Min 0 (No shift)
                Serial.print("Interval Down: +/- "); Serial.println(maxSemitones);
                updateLUT();
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        
        lastIntervalButtonState = currentIntervalButtonState;

        // --- 3. AUTO-FEEDBACKER ENVELOPE ---
        // Momentary action: only active while your foot is holding it down
        bool currentFbState = digitalRead(FEEDBACK_BUTTON_PIN);
        
        if (currentFbState == LOW) { 
            feedbackRamp += 0.01f; // Swell attack (approx 500ms fade-in)
            if (feedbackRamp > 1.0f) feedbackRamp = 1.0f;
        } else { 
            feedbackRamp -= 0.02f; // Fade out release (approx 250ms decay)
            if (feedbackRamp < 0.0f) feedbackRamp = 0.0f;
        }
    


        // --- 4. WHAMMY POTENTIOMETER LOGIC ---
        filterPB.update();
        
        // 1. Get the raw 12-bit hardware reading (0 to 4095)
        analog_t raw12 = filterPB.getValue(); 
        
        // 2. "Stretch" it to the 14-bit MIDI scale (0 to 16383)
        analog_t raw14 = map(raw12, 0, 4095, 0, 16383);
        
        // 3. Pass the stretched 14-bit value into your deadzone math
        analog_t calibratedMidi = map_PB_Full(raw14);

        // Send MIDI ONLY if the pedal actually moved!
        if (calibratedMidi != lastMidiSent) {
            Control_Surface.sendPitchBend(Channel_1, calibratedMidi);
            lastMidiSent = calibratedMidi;
        }

        // --- LOW BOUNDARY (HEEL DOWN) ---
        if (calibratedMidi == 0) {
            if (!lockedAtMin) {
                lockedAtMin = true;
                Serial.println("[LIMIT: MIN]");
            }
        } else { lockedAtMin = false; }

        // --- HIGH BOUNDARY (TOE DOWN) ---
        if (calibratedMidi == 16383) { 
            if (!lockedAtMax) {
                lockedAtMax = true;
                Serial.println("[LIMIT: MAX]");
            }
        } else { lockedAtMax = false; }

        // --- RE-CENTERING LOGIC ---
        if (calibratedMidi == 8192 && PBwasOffCenter) {
            PBwasOffCenter = false;
            Serial.println("[FORCE MIDI CENTER]");
        }

        // --- 5. DYNAMIC PITCH SHIFT MATH ---
        int safeIndex = constrain(calibratedMidi, 0, 16383);
        pitchShiftFactor = pitchShiftLUT[safeIndex];
          
        
        //debugPrint();
        vTaskDelay(pdMS_TO_TICKS(5)); // Yield to RTOS scheduler
    }
}

void setup() {

    Serial.begin(SERIAL_BAUDRATE);

    // Wait up to 4 seconds for the Serial Monitor to physically open on the PC
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000));

    Serial.println("--- SYSTEM BOOTING ---");

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
        .bck_io_num = 14,   // Replace with your BCLK pin
        .ws_io_num = 15,    // Replace with your LRCK/WS pin
        .data_out_num = 16, // Replace with your DAC Data pin
        .data_in_num = 17   // Replace with your ADC Data pin
    };


    i2s_set_pin(I2S_NUM_0, &pin_config);
    initTrigLUT();
    updateLUT();

    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, NULL, 1);

}

    void loop() {
    }