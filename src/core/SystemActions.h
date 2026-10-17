#pragma once

#include "SystemState.h"
#include "BoardHAL.h"
#include "MidiRouter.h"
#include "I2SManager.h"
#include "SpinlockGuard.h"

inline void switchEffectMode(int newMode) {
    int cmode = (newMode % 10 + 10) % 10;
    activeEffectMode.store(cmode, std::memory_order_release);
    
    // Clear secondary effect states first for a clean mode swap
    isFrozen         = false;
    isFeedbackActive = false;
    isHarmonizerMode = false;
    isCapoMode       = false;
    isSynthMode      = false;
    isPadMode        = false;
    isChorusMode     = false;
    isSwellMode      = false;
    isVibratoMode    = false;
    
    // Polyphonic Whammy is global whenever Mono Whammy is not active
    bool monoActive = isMonoPolyActive.load(std::memory_order_acquire);
    isWhammyActive = !monoActive;
    
    // Activate only the selected mode's effect
    switch (cmode) {
        case 0: /* Mode 0 is Whammy itself; isWhammyActive already set */ break;
        case 1: isFrozen         = true; break;
        case 2: isFeedbackActive = true; break;
        case 3: isHarmonizerMode = true; break;
        case 4: isCapoMode       = true; break;
        case 5: isSynthMode      = true; break;
        case 6: isPadMode        = true; break;
        case 7: isChorusMode     = true; break;
        case 8: isSwellMode      = true; break;
        case 9: isVibratoMode    = true; break;
    }
    Serial0.printf("[SYSTEM] Switched Effect Mode cleanly to: %d\n", cmode);
    
    dspNeedsCommit = true;
    lutNeedsUpdate = true;
    settingsNeedSaving = true;
    lastParameterChangeTime = millis();
}

inline void triggerPanicReset() {
    panicResetRequested.store(true, std::memory_order_release);
    
    // Disable Mono Whammy and re-engage Poly Whammy
    isMonoPolyActive.store(false, std::memory_order_release);
    isWhammyActive = true;
    
    if (isVolumeMode) {
        isVolumeMode = false;
        volumePedalGain = 1.0f; 
        pedals.lockPB3Whammy(); 
        currentPB3 = 8192; 
        lastActivePedal = 8192;
        activePedalSource = 0;
        Control_Surface.sendPitchBend(Channel_3, 8192);
        
        float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
        if (currentLUT) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release);
    }
    
    switchEffectMode(0);
}

inline bool commitDSPState() {
    if (!dspAckCommit.load(std::memory_order_acquire)) return false;
    
    CriticalSectionGuard lock(MidiRouter::paramMux);
    DSPCoreState* backBuffer = &dspStates[dspWriteIndex];
    
    for(int i=0; i<10; i++) {
        backBuffer->fxMem[i] = effectMemory[i];
        for(int j=0; j<5; j++) backBuffer->params[i][j] = fxParams[i][j];
    }
    
    backBuffer->activeMode = activeEffectMode.load(std::memory_order_relaxed);
    backBuffer->latMode = latencyMode.load(std::memory_order_relaxed);
    backBuffer->fbIdx = feedbackIntervalIdx.load(std::memory_order_relaxed);
    
    backBuffer->w = isWhammyActive;
    backBuffer->fz = isFrozen;
    backBuffer->fb = isFeedbackActive;
    backBuffer->hr = isHarmonizerMode;
    backBuffer->cp = isCapoMode;
    backBuffer->sy = isSynthMode;
    backBuffer->pd = isPadMode;
    backBuffer->ch = isChorusMode;
    backBuffer->sw = isSwellMode;
    backBuffer->vb = isVibratoMode;
    backBuffer->vg = volumePedalGain;
    
    dspAckCommit.store(false, std::memory_order_release); 
    dspActiveState.store(backBuffer, std::memory_order_release);
    dspWriteIndex = (dspWriteIndex + 1) & 1; 
    
    return true;
}

inline void calibratePBs() {
    for(int i=0; i<50; i++) { 
        BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat); 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    long s1=0, s2=0, s3=0; 
    for(int i=1; i<=250; i++) { 
        BoardHAL::fetchADC(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestPar1, latestBat); 
        s1+=latestPB1; s2+=latestPB2; s3+=latestPB3; 
    } 
    pedals.setCenters(s1/250, s2/250, s3/250);
}

inline void cycleLatencyMode() {
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) { 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    latencyMode.store((latencyMode.load(std::memory_order_acquire) + 1) % 4, std::memory_order_release);
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchLow, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(sramPitchHigh, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t));
    if (diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    
    globalAudioResetRequested.store(true, std::memory_order_release);
    dspNeedsCommit = true; 
    std::atomic_thread_fence(std::memory_order_seq_cst);
    
    dsp_is_paused.store(false, std::memory_order_release);
    
    if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
    if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
    
    while(dsp_ack_parked.load(std::memory_order_acquire) || lut_ack_parked.load(std::memory_order_acquire)) { 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    settingsNeedSaving = true; 
    lastParameterChangeTime = millis();
}

inline void setSampleRate(uint32_t targetSr) {
    if (currentSampleRate.load(std::memory_order_acquire) == targetSr) return;
    
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) { 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    currentSampleRate.store(targetSr, std::memory_order_release); 
    settingsNeedSaving = false; 
    lutNeedsUpdate = true;
    
    Serial0.printf("[SYSTEM] Reconfiguring I2S Clock to %lu Hz...\n", targetSr);
    padVectorFilter.setLPF(1200.0f, (float)targetSr);
    
    I2SManager::disableAndDestroyChannels();
    
    vTaskDelay(pdMS_TO_TICKS(25));
    
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(targetSr);
    // Reverted to 384x to restore PCM1808 / PCM5102A audio stream
    clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
    
    I2SManager::initChannels(BoardHAL::getI2SConfig(targetSr), HOP_SIZE);
    
    freezeLength = targetSr;
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchLow, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t)); 
    memset(sramPitchHigh, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t)); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t)); 
    if(diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    
    vTaskDelay(pdMS_TO_TICKS(30));
    
    I2SManager::enableChannels();
    
    globalAudioResetRequested.store(true, std::memory_order_release); 
    hardwareSyncMuteFrames.store((targetSr/HOP_SIZE)*0.40f, std::memory_order_release);
    
    std::atomic_thread_fence(std::memory_order_seq_cst);
    
    dsp_is_paused.store(false, std::memory_order_release);
    
    if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
    if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
    
    while(dsp_ack_parked.load(std::memory_order_acquire) || lut_ack_parked.load(std::memory_order_acquire)) { 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    pedals.triggerSystemRecovery(); 
    settingsNeedSaving = true; 
    lastParameterChangeTime = millis();
}

inline void turnScreenOff() {
    if(!isScreenOff.load(std::memory_order_acquire)) {
        digitalWrite(15, LOW);
        isScreenOff.store(true, std::memory_order_release);
    }
}

inline void turnScreenOn() {
    if(isScreenOff.load(std::memory_order_acquire)) {
        digitalWrite(15, HIGH);
        isScreenOff.store(false, std::memory_order_release);
    }
}

inline void goToLightSleep() {
    turnScreenOff();
    
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) { 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    I2SManager::disableAndDestroyChannels();
    
    isAdcPaused = true;
    if (multifx_adc_handle != NULL) {
        adc_continuous_stop(multifx_adc_handle);
    }
    
    if (dsp_cpu_lock != NULL) esp_pm_lock_release(dsp_cpu_lock);
    
    while ((REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) &&
           (REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN))) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);
    
    if (multifx_adc_handle != NULL) {
        adc_continuous_start(multifx_adc_handle);
    }
    isAdcPaused = false;
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchLow, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(sramPitchHigh, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t));
    if (diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    
    I2SManager::initChannels(BoardHAL::getI2SConfig(currentSampleRate.load(std::memory_order_acquire)), HOP_SIZE);
    I2SManager::enableChannels();
    
    globalAudioResetRequested.store(true, std::memory_order_release);
    
    std::atomic_thread_fence(std::memory_order_seq_cst);
    dsp_is_paused.store(false, std::memory_order_release);
    
    if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
    if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
    
    while(dsp_ack_parked.load(std::memory_order_acquire) || lut_ack_parked.load(std::memory_order_acquire)) { 
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    turnScreenOn();
    pedals.triggerSystemRecovery(); 
    lastActivityTime = millis();
}