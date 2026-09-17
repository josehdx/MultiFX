#pragma once
#include "SystemState.h"
#include "SystemActions.h"

class InputManager {
public:
    static void processInputs(bool currentBtState) {
        static unsigned long gpio14PressTime = 0, lastDebounceTime = 0; 
        static bool gpio14LastState = HIGH, lastBootState = HIGH;

        bool reading = (REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN)) != 0;
        if (reading != gpio14LastState && (millis() - lastDebounceTime) > 50) {
            lastDebounceTime = millis();
            if (!reading) { 
                gpio14PressTime = millis(); 
                lastActivityTime = millis(); 
            } else {
                unsigned long pressDuration = millis() - gpio14PressTime;
                if (pressDuration >= 1000) { 
                    #if defined(FW_MODE_HYBRID)
                        DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                        uint32_t watermarkVal = 0;
                        #ifdef ENABLE_ADVANCED_TELEMETRY
                            watermarkVal = lut_stack_watermark.load(std::memory_order_relaxed);
                        #endif

                        if (isKnobEditMode) {
                            showSavingScreen = true;
                            // BRIDGED FLAG ADDED HERE
                            BoardHAL::updateUI(activeDSP, currentPB1, currentPB2, currentPB3, currentCC11, ui_audio_level.load(std::memory_order_acquire), ui_output_level.load(std::memory_order_acquire), core0_dsp_load.load(std::memory_order_relaxed), core1_ctrl_load.load(std::memory_order_relaxed), currentSampleRate.load(std::memory_order_acquire), max_loop_latency_ms.exchange(0, std::memory_order_relaxed), audio_underflow_count.load(std::memory_order_relaxed), watermarkVal, currentBtState, latestBat, isMonoPolyActive.load(std::memory_order_acquire));
                            vTaskDelay(pdMS_TO_TICKS(150)); 
                            
                            dsp_is_paused.store(true, std::memory_order_release);
                            while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }

                            AppSettings cs; 
                            {
                                CriticalSectionGuard lock(MidiRouter::paramMux);
                                for(int i=0; i<10; i++) { 
                                    cs.fxMem[i]=activeDSP->fxMem[i]; 
                                    for(int p=0; p<5; p++) cs.params[i][p]=activeDSP->params[i][p]; 
                                }
                            }
                            uint16_t fxStates=0; if(activeDSP->w) fxStates|=(1<<0); if(activeDSP->fz) fxStates|=(1<<1); if(activeDSP->fb) fxStates|=(1<<2); if(activeDSP->hr) fxStates|=(1<<3); if(activeDSP->cp) fxStates|=(1<<4); if(activeDSP->sy) fxStates|=(1<<5); if(activeDSP->pd) fxStates|=(1<<6); if(activeDSP->ch) fxStates|=(1<<7); if(activeDSP->sw) fxStates|=(1<<8); if(activeDSP->vb) fxStates|=(1<<9);
                            settingsMgr.save(preferences, activeDSP->activeMode, activeDSP->latMode, constrain(activeDSP->fbIdx,0,4), isPB2WiperMode, isVolumeMode, fxStates, currentSampleRate.load(std::memory_order_acquire), &cs, sizeof(AppSettings));
                        } else {
                            showKnobModeScreen = true;
                            // BRIDGED FLAG ADDED HERE
                            BoardHAL::updateUI(activeDSP, currentPB1, currentPB2, currentPB3, currentCC11, ui_audio_level.load(std::memory_order_acquire), ui_output_level.load(std::memory_order_acquire), core0_dsp_load.load(std::memory_order_relaxed), core1_ctrl_load.load(std::memory_order_relaxed), currentSampleRate.load(std::memory_order_acquire), max_loop_latency_ms.exchange(0, std::memory_order_relaxed), audio_underflow_count.load(std::memory_order_relaxed), watermarkVal, currentBtState, latestBat, isMonoPolyActive.load(std::memory_order_acquire));
                            vTaskDelay(pdMS_TO_TICKS(150)); 
                        }
                        
                        preferences.putBool("knobEditMode", !isKnobEditMode);
                        I2SManager::disableAndDestroyChannels();
                        ESP.restart(); 
                    #endif
                } else if (pressDuration >= 50) { 
                    #if defined(FW_MODE_HYBRID)
                        if (isKnobEditMode) { showBleWarning = true; warningTimer = millis(); } else { cycleLatencyMode(); }
                    #else
                        cycleLatencyMode(); 
                    #endif
                }
            }
            gpio14LastState = reading;
        }

        if (showBleWarning && (millis() - warningTimer > 3000)) showBleWarning = false;

        bool currentBootState = (REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) != 0;
        if(!currentBootState && lastBootState) { switchEffectMode(activeEffectMode.load(std::memory_order_acquire) + 1); lastActivityTime = millis(); vTaskDelay(pdMS_TO_TICKS(50)); }
        lastBootState = currentBootState;
    }

    static void processPedals() {
        pedals.process(latestPB1, latestPB2, latestPB3, isVolumeMode, INVERT_PB3);
        currentPB1 = pedals.getCalA(); currentPB2 = pedals.getCalB(); currentPB3 = pedals.getCalC(); 
        
        if (pedals.hasMovedC()) {
            lastActivePedal = currentPB3;
            if (isVolumeMode) {
                volumePedalGain = (float)currentPB3 / 16383.0f;
                dspNeedsCommit = true;
            } else {
                if (!asyncLutUpdateRequested.load(std::memory_order_acquire)) {
                    float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
                    if (currentLUT) pitchShiftFactor.store(currentLUT[currentPB3], std::memory_order_release);
                }
            }
            pedals.updateLastMidiC();
        }
    }
};