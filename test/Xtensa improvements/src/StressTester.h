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
}; // FIXED: Added missing closing brace

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

            // 1. Hyper-Aggressive Pedal Randomization (Every 5 ms)
            if (now - lastPedalRandomize >= 5) {
                *p->pb1 = random(0, 4095);
                *p->pb2 = random(0, 4095);
                *p->pb3 = random(0, 4095);
                lastPedalRandomize = now;
            }

            // 2. Faster Sample Rate Swap Request (Every 2000 ms)
            if (now - lastSrToggle >= 2000) {
                if (p->srToggleReq != nullptr) {
                    *p->srToggleReq = true;
                    Serial.println("[STRESS] 2s TIMER -> SAMPLE RATE TOGGLE REQUESTED (48k <-> 96k)");
                }
                lastSrToggle = now;
            }

            // 3. Rapid FX Stacking & Panic Reset State Machine
            if (!isHoldingMaxLoad) {
                if (now - lastFxStep >= 500) {
                    if (currentFxCount < 10) {
                        Serial.printf("[STRESS] Mode Switch to %d (Stacking FX #%d)\n", currentFxCount, currentFxCount + 1);
                        p->switchMode(currentFxCount);
                        currentFxCount++;
                    } else {
                        isHoldingMaxLoad = true;
                        maxLoadStartTime = now;
                        Serial.println("==================================================================");
                        Serial.println("[STRESS] ALL 10 FX STACKED! HOLDING MAX LOAD FOR 10 SECONDS...");
                        Serial.println("==================================================================");
                    }
                    lastFxStep = now;
                }
            } else {
                if (now - maxLoadStartTime >= 10000) {
                    Serial.println("[STRESS] 10s HOLD COMPLETE -> FIRING PANIC RESET");
                    p->triggerPanic();
                    
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