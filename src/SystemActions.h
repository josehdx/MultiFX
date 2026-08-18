#pragma once
#include "SystemState.h"
#include "BoardHAL.h"
#include "MidiRouter.h"
#include "I2SManager.h"
#include "SpinlockGuard.h"

inline void switchEffectMode(int newMode) {
    int cmode = (newMode % 10 + 10) % 10;
    activeEffectMode.store(cmode, std::memory_order_release);
    if (cmode == 0) {
        isWhammyActive = true; isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false;
        isCapoMode = false; isSynthMode = false; isPadMode = false; isChorusMode = false; isSwellMode = false; isVibratoMode = false;
    } else {
        isWhammyActive = true;
        if (cmode == 1) isFrozen = true; if (cmode == 2) isFeedbackActive = true; if (cmode == 3) isHarmonizerMode = true;
        if (cmode == 4) isCapoMode = true; if (cmode == 5) isSynthMode = true; if (cmode == 6) isPadMode = true;
        if (cmode == 7) isChorusMode = true; if (cmode == 8) isSwellMode = true; if (cmode == 9) isVibratoMode = true;
    } 
    Serial.printf("[SYSTEM] Switched Effect Mode to: %d\n", cmode);
    dspNeedsCommit = true; lutNeedsUpdate = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
}

inline void triggerPanicReset() {
    panicResetRequested.store(true, std::memory_order_release);
    activeEffectMode.store(0, std::memory_order_release);
    isWhammyActive = true; isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false; 
    isCapoMode = false; isSynthMode = false; isPadMode = false; isChorusMode = false; isSwellMode = false; isVibratoMode = false; 
    dspNeedsCommit = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
}

inline bool commitDSPState() {
    if (!dspAckCommit.load(std::memory_order_acquire)) return false; 
    DSPCoreState* backBuffer = &dspStates[dspWriteIndex];
    {
        CriticalSectionGuard lock(MidiRouter::paramMux);
        for(int i=0; i<10; i++) { 
             backBuffer->fxMem[i] = effectMemory[i]; 
             for(int j=0; j<5; j++) backBuffer->params[i][j] = fxParams[i][j]; 
         }
    }
    backBuffer->activeMode = activeEffectMode.load(std::memory_order_relaxed); backBuffer->latMode = latencyMode.load(std::memory_order_relaxed); backBuffer->fbIdx = feedbackIntervalIdx.load(std::memory_order_relaxed);
    backBuffer->w = isWhammyActive; backBuffer->fz = isFrozen; backBuffer->fb = isFeedbackActive; backBuffer->hr = isHarmonizerMode; backBuffer->cp = isCapoMode; backBuffer->sy = isSynthMode; backBuffer->pd = isPadMode; backBuffer->ch = isChorusMode; backBuffer->sw = isSwellMode; backBuffer->vb = isVibratoMode; backBuffer->vg = volumePedalGain;
    dspAckCommit.store(false, std::memory_order_release); 
    dspActiveState.store(backBuffer, std::memory_order_release);
    dspWriteIndex = (dspWriteIndex + 1) & 1; 
    return true;
}

inline void calibratePBs() {
    for(int i=0; i<50; i++) { BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat); vTaskDelay(pdMS_TO_TICKS(1)); }
    long s1=0, s2=0, s3=0; 
    for(int i=1; i<=250; i++) { BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat); s1+=latestPB1; s2+=latestPB2; s3+=latestPB3; } 
    pedals.setCenters(s1/250, s2/250, s3/250);
}

inline void cycleLatencyMode() {
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    latencyMode.store((latencyMode.load(std::memory_order_acquire) + 1) % 4, std::memory_order_release);
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchLow, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(sramPitchHigh, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t));
    if (diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    globalAudioResetRequested.store(true, std::memory_order_release);
    dspNeedsCommit = true; std::atomic_thread_fence(std::memory_order_seq_cst);
    dsp_is_paused.store(false, std::memory_order_release);
    if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
    if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
    while(dsp_ack_parked.load(std::memory_order_acquire) || lut_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    settingsNeedSaving = true; lastParameterChangeTime = millis();
}

inline void toggleSampleRate() {
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    I2SManager::disableAndDestroyChannels();
    uint32_t newSr = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? 48000 : 96000;
    currentSampleRate.store(newSr, std::memory_order_release); settingsNeedSaving = false; lutNeedsUpdate = true; 
    Serial.printf("[SYSTEM] Toggling Sample Rate to %lu Hz...\n", newSr);
    padVectorFilter.setLPF(1200.0f, (float)newSr);
    i2s_std_config_t stdConfig = BoardHAL::getI2SConfig(newSr);
    I2SManager::initChannels(stdConfig, HOP_SIZE);
    freezeLength = newSr;
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchLow, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t)); 
    memset(sramPitchHigh, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t)); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t)); 
    if(diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float)); 
    vTaskDelay(pdMS_TO_TICKS(30));
    I2SManager::enableChannels();
    globalAudioResetRequested.store(true, std::memory_order_release); hardwareSyncMuteFrames.store((newSr/HOP_SIZE)*0.40f, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_seq_cst); 
    dsp_is_paused.store(false, std::memory_order_release);
    if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
    if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
    while(dsp_ack_parked.load(std::memory_order_acquire) || lut_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    pedals.triggerSystemRecovery(); settingsNeedSaving=true; lastParameterChangeTime = millis();
}