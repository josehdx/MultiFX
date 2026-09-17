#ifndef STRESS_TESTER_H
#define STRESS_TESTER_H

#include <Arduino.h>
#include "SystemState.h"

// --- TEST SELECTION MACRO ---
// 1 = Original Polyphonic Stack Test
// 2 = New MonoPoly Algorithm & Stack Test
#define SELECTED_STRESS_TEST 2 

typedef void (*ModeSwitchFn)(int);
typedef void (*PanicResetFn)();

struct StressParams {
    volatile int* pb1;
    volatile int* pb2;
    volatile int* pb3;
    ModeSwitchFn switchMode;
    PanicResetFn triggerPanic;
    volatile bool* srToggleReq;
};

class StressTester {
public:
    // --- 1. ORIGINAL POLYPHONIC STRESS TEST ---
    static void StressTask(void* pvParameters) {
        StressParams* p = (StressParams*)pvParameters;
        int currentFxCount = 1;
        unsigned long lastPedalRandomize = 0;
        unsigned long lastSrToggle = millis();
        unsigned long lastFxStep = millis();
        bool isHoldingMaxLoad = false;
        unsigned long maxLoadStartTime = 0;

        for (;;) {
            unsigned long now = millis();

            // Automated Bounds-Checking: Reboot on Latency Spike
            uint32_t currentLatency = max_loop_latency_ms.load(std::memory_order_relaxed);
            if (currentLatency > 250) { // Increased from 50 to allow USB flash writes
                Serial0.printf("\n[FATAL ERROR] Loop Latency spiked to %lu ms! Initiating Auto-Reboot...\n", currentLatency);
                vTaskDelay(pdMS_TO_TICKS(100));
                ESP.restart();
            }

            if (now - lastPedalRandomize >= 20) {
                *p->pb1 = random(0, 4095); *p->pb2 = random(0, 4095); *p->pb3 = random(0, 4095);
                lastPedalRandomize = now;
            }

            if (now - lastSrToggle >= 2000) {
                if (p->srToggleReq != nullptr) *p->srToggleReq = true;
                Serial0.println("[STRESS] 2s TIMER -> SAMPLE RATE TOGGLE REQUESTED (48k <-> 96k)");
                lastSrToggle = now;
            }

            if (!isHoldingMaxLoad) {
                if (now - lastFxStep >= 500) {
                    if (currentFxCount < 10) {
                        Serial0.printf("[STRESS] Mode Switch to %d (Stacking FX #%d)\n", currentFxCount, currentFxCount + 1);
                        p->switchMode(currentFxCount);
                        currentFxCount++;
                    } else {
                        isHoldingMaxLoad = true;
                        maxLoadStartTime = now;
                        Serial0.println("==================================================================");
                        Serial0.println("[STRESS] ALL 10 FX STACKED! HOLDING MAX LOAD FOR 10 SECONDS...");
                        Serial0.println("==================================================================");
                    }
                    lastFxStep = now;
                }
            } else {
                if (now - maxLoadStartTime >= 10000) {
                    Serial0.println("[STRESS] 10s HOLD COMPLETE -> FIRING PANIC RESET");
                    p->triggerPanic();
                    currentFxCount = 1;
                    isHoldingMaxLoad = false;
                    lastFxStep = now + 1000; 
                }
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    // --- 2. MONOPOLY ALGORITHM & STACK STRESS TEST ---
    static void MonoPolyStressTask(void* pvParameters) {
        StressParams* p = (StressParams*)pvParameters;
        int currentAlgo = 0;
        int currentFxCount = 1;
        unsigned long lastPedalRandomize = 0;
        unsigned long lastSrToggle = millis();
        unsigned long lastFxStep = millis();
        bool isHoldingMaxLoad = false;
        unsigned long maxLoadStartTime = 0;

        p->triggerPanic();
        vTaskDelay(pdMS_TO_TICKS(100));

        for (;;) {
            unsigned long now = millis();

            // Automated Bounds-Checking: Reboot on Latency Spike
            uint32_t currentLatency = max_loop_latency_ms.load(std::memory_order_relaxed);
            if (currentLatency > 250) { // Increased from 50 to allow USB flash writes
                Serial0.printf("\n[FATAL ERROR] Loop Latency spiked to %lu ms! Initiating Auto-Reboot...\n", currentLatency);
                vTaskDelay(pdMS_TO_TICKS(100));
                ESP.restart();
            }

            // 1. Pedal Randomization
            if (now - lastPedalRandomize >= 20) {
                *p->pb1 = random(0, 4095); *p->pb2 = random(0, 4095); *p->pb3 = random(0, 4095);
                lastPedalRandomize = now;
            }

            // 2. Sample Rate Swap (48k <-> 96k)
            if (now - lastSrToggle >= 2000) {
                if (p->srToggleReq != nullptr) *p->srToggleReq = true;
                Serial0.println("[MONO-STRESS] 2s TIMER -> SAMPLE RATE TOGGLE (48k <-> 96k)");
                lastSrToggle = now;
            }

            // 3. Algorithm & Secondary FX Stacking Logic
            if (!isHoldingMaxLoad) {
                if (now - lastFxStep >= 500) {
                    if (currentFxCount == 1) {
                        // Override main engine with MonoPoly
                        isMonoPolyActive.store(true, std::memory_order_release);
                        isWhammyActive = false;
                        
                        // Pass active algorithm selection (0 through 4) to parameter P1
                        fxParams[0][0] = (float)currentAlgo * 0.25f; 
                        dspNeedsCommit = true;
                        
                        Serial0.printf("\n==================================================\n");
                        Serial0.printf("[MONO-STRESS] STARTING CYCLE -> MonoPoly Algo %d\n", currentAlgo);
                        Serial0.printf("==================================================\n");
                        currentFxCount++;
                    } 
                    else if (currentFxCount <= 10) {
                        // Sequentially stack remaining 9 effects
                        switch(currentFxCount) {
                            case 2: isFrozen = true; Serial0.println("[MONO-STRESS] Stacking: FREEZE"); break;
                            case 3: isFeedbackActive = true; Serial0.println("[MONO-STRESS] Stacking: FEEDBACK"); break;
                            case 4: isHarmonizerMode = true; Serial0.println("[MONO-STRESS] Stacking: HARMONY"); break;
                            case 5: isCapoMode = true; Serial0.println("[MONO-STRESS] Stacking: CAPO"); break;
                            case 6: isSynthMode = true; Serial0.println("[MONO-STRESS] Stacking: SYNTH"); break;
                            case 7: isPadMode = true; Serial0.println("[MONO-STRESS] Stacking: PAD"); break;
                            case 8: isChorusMode = true; Serial0.println("[MONO-STRESS] Stacking: CHORUS"); break;
                            case 9: isSwellMode = true; Serial0.println("[MONO-STRESS] Stacking: SWELL"); break;
                            case 10: isVibratoMode = true; Serial0.println("[MONO-STRESS] Stacking: VIBRATO"); break;
                        }
                        dspNeedsCommit = true;
                        currentFxCount++;
                    } 
                    else {
                        isHoldingMaxLoad = true;
                        maxLoadStartTime = now;
                        Serial0.println("==================================================================");
                        Serial0.printf("[MONO-STRESS] ALL 9 FX STACKED ON ALGO %d! HOLDING FOR 10s...\n", currentAlgo);
                        Serial0.println("==================================================================");
                    }
                    lastFxStep = now;
                }
            } else {
                if (now - maxLoadStartTime >= 10000) {
                    Serial0.println("[MONO-STRESS] 10s HOLD COMPLETE -> FIRING PANIC RESET");
                    p->triggerPanic();
                    vTaskDelay(pdMS_TO_TICKS(50));
                    
                    // Increment algorithm index and restart stacking loop
                    currentAlgo = (currentAlgo + 1) % 5;
                    currentFxCount = 1;
                    isHoldingMaxLoad = false;
                    lastFxStep = now + 1000;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(2)); 
        }
    }
};

#endif