#ifndef STRESS_TESTER_H
#define STRESS_TESTER_H

#include <Arduino.h>

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

            // 1. Aggressive Pedal Randomization (Throttled to 20 ms to prevent BLE assert)
            // Constantly triggers dynamic LUT generation and MIDI routing logic
            if (now - lastPedalRandomize >= 20) {
                *p->pb1 = random(0, 4095);
                *p->pb2 = random(0, 4095);
                *p->pb3 = random(0, 4095);
                lastPedalRandomize = now;
            }

            // 2. Faster Sample Rate Swap Request (Every 2000 ms)
            // Violently tears down and rebuilds the I2S/DMA hardware pipeline
            if (now - lastSrToggle >= 2000) {
                if (p->srToggleReq != nullptr) {
                    *p->srToggleReq = true;
#if defined(TARGET_BANANA)
                    Serial0.println("[STRESS] 2s TIMER -> SAMPLE RATE TOGGLE REQUESTED (48k <-> 96k)");
#else
                    Serial.println("[STRESS] 2s TIMER -> SAMPLE RATE TOGGLE REQUESTED (48k <-> 96k)");
#endif
                }
                lastSrToggle = now;
            }

            // 3. Rapid FX Stacking & Panic Reset State Machine
            if (!isHoldingMaxLoad) {
                // Switch modes rapid-fire (Every 500 ms)
                if (now - lastFxStep >= 500) {
                    if (currentFxCount < 10) {
#if defined(TARGET_BANANA)
                        Serial0.printf("[STRESS] Mode Switch to %d (Stacking FX #%d)\n", currentFxCount, currentFxCount + 1);
#else
                        Serial.printf("[STRESS] Mode Switch to %d (Stacking FX #%d)\n", currentFxCount, currentFxCount + 1);
#endif
                        p->switchMode(currentFxCount);
                        currentFxCount++;
                    } else {
                        isHoldingMaxLoad = true;
                        maxLoadStartTime = now;
#if defined(TARGET_BANANA)
                        Serial0.println("==================================================================");
                        Serial0.println("[STRESS] ALL 10 FX STACKED! HOLDING MAX LOAD FOR 10 SECONDS...");
                        Serial0.println("==================================================================");
#else
                        Serial.println("==================================================================");
                        Serial.println("[STRESS] ALL 10 FX STACKED! HOLDING MAX LOAD FOR 10 SECONDS...");
                        Serial.println("==================================================================");
#endif
                    }
                    lastFxStep = now;
                }
            } else {
                // Hold max load state for 10 seconds before panic resetting
                if (now - maxLoadStartTime >= 10000) {
#if defined(TARGET_BANANA)
                    Serial0.println("[STRESS] 10s HOLD COMPLETE -> FIRING PANIC RESET");
#else
                    Serial.println("[STRESS] 10s HOLD COMPLETE -> FIRING PANIC RESET");
#endif
                    p->triggerPanic();
                    
                    currentFxCount = 1;
                    isHoldingMaxLoad = false;
                    lastFxStep = now + 1000; // 1s grace period post-reset before restarting cycle
                }
            }

            // Yield only 2 ms (starves idle processes and pushes FreeRTOS harder)
            vTaskDelay(pdMS_TO_TICKS(2)); 
        }
    }
};

#endif