#pragma once
#include "SystemState.h"
#include "LUTManager.h"
#include "SpinlockGuard.h"

// Changed 'inline' to 'static' to fix Xtensa l32r literal relocation error
static void IRAM_ATTR LUTUpdateTask(void * pvParameters) {
    for(;;) {
        if(__builtin_expect(dsp_is_paused.load(std::memory_order_acquire), 0)) {
            lut_ack_parked.store(true, std::memory_order_release);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            lut_ack_parked.store(false, std::memory_order_release);
        }
        if (asyncLutUpdateRequested.exchange(false, std::memory_order_acq_rel)) {
            DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
            float safeEffectMemory[10];
            {
                CriticalSectionGuard lock(MidiRouter::paramMux);
                for(int i=0; i<10; i++) safeEffectMemory[i] = effectMemory[i];
            }
            float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
            float* targetLUT = (currentLUT == pitchLutBufferA) ? pitchLutBufferB : pitchLutBufferA;
            LUTManager::updateDynamicLUT(activeDSP->cp, activeDSP->w, activeDSP->activeMode, safeEffectMemory, activeDSP->fbIdx, currentSampleRate.load(std::memory_order_acquire));
            float* sourceLUT = LUTManager::pitchShiftLUT.load(std::memory_order_acquire);
            if(sourceLUT && targetLUT) { memcpy(targetLUT, sourceLUT, 16384 * sizeof(float)); }
            std::atomic_thread_fence(std::memory_order_release);
            activePitchLUT.store(targetLUT, std::memory_order_release);
            pitchShiftFactor.store(targetLUT[constrain(lastActivePedal, 0, 16383)], std::memory_order_release); 
        }
        vTaskDelay(pdMS_TO_TICKS(15)); 
    }
}