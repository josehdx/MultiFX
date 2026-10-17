#pragma once
#include "SystemState.h"
#include "SystemActions.h"
#include "MidiRouter.h"
#include "BluetoothManager.h"

inline char lastMidiDebug[64] = "MIDI: WAITING FOR PACKET...";

inline bool channelMessageCallback(ChannelMessage cm) {
    lastActivityTime = millis();
    
    snprintf(lastMidiDebug, sizeof(lastMidiDebug), "RX -> H:0x%02X D1:%03d D2:%03d", cm.header, cm.data1, cm.data2);
    Serial0.println(lastMidiDebug);

    if (cm.header != 0xB0) return false;

    MidiAction action = MidiRouter::parseMessage(cm.data1, cm.data2);
    if (action.event == MidiEvent::NONE) return false;

    int currentMode = activeEffectMode.load(std::memory_order_acquire);
    
    switch (action.event) {
        case MidiEvent::EXPRESSION_UPDATE: {
            CriticalSectionGuard lock(MidiRouter::paramMux);
            currentCC11 = action.rawValue; 
            lastActivePedal = action.rawValue;
            activePedalSource = 0; // MIDI Active
            
            uint8_t original7Bit = action.val;

            if (isVolumeMode) {
                volumePedalGain = (float)action.rawValue / 16383.0f;
                dspNeedsCommit = true;
                
                // Transmit CC 19 (Volume) over both USB-MIDI and BLE-MIDI (7-bit)
                Control_Surface.sendControlChange({19, Channel_1}, original7Bit);
                BluetoothManager::sendMidiToHost(0xB0, 19, original7Bit);
            } else {
                float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
                if (currentLUT) pitchShiftFactor.store(currentLUT[action.rawValue], std::memory_order_release);
                
                // Transmit Pitch Bend on Channel 4 (14-bit interpolated)
                Control_Surface.sendPitchBend(Channel_4, action.rawValue);
                BluetoothManager::sendMidiToHost(0xE3, action.rawValue & 0x7F, (action.rawValue >> 7) & 0x7F);
            }
            break;
        }

        case MidiEvent::KNOB_UPDATE: {
            // updateParameter already handles its own paramMux internally
            MidiRouter::updateParameter(action.cc, action.val, currentMode, effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx);
            settingsNeedSaving = true; 
            lastParameterChangeTime = millis();
            break;
        }

        case MidiEvent::PREV_MODE: {
            static unsigned long lastPrev = 0;
            if (action.val > 0 && millis() - lastPrev > 250) {
                lastPrev = millis();
                switchEffectMode(currentMode - 1); 
            }
            break;
        }

        case MidiEvent::NEXT_MODE: {
            static unsigned long lastNext = 0;
            if (action.val > 0 && millis() - lastNext > 250) {
                lastNext = millis();
                switchEffectMode(currentMode + 1); 
            }
            break;
        }

        case MidiEvent::LATENCY_CYCLE: {
            static unsigned long lastLat = 0;
            if (action.val > 0 && millis() - lastLat > 500) {
                lastLat = millis();
                cycleLatencyMode(); 
            }
            break;
        }

        case MidiEvent::PANIC_RESET: {
            static unsigned long lastPanic = 0;
            if (action.val > 0 && millis() - lastPanic > 500) {
                lastPanic = millis();
                triggerPanicReset(); 
            }
            break;
        }

        case MidiEvent::SR_TOGGLE: {
            static unsigned long lastSrToggle = 0;
            if (millis() - lastSrToggle > 500) {
                lastSrToggle = millis();
                
                uint32_t currentSr = currentSampleRate.load(std::memory_order_acquire);
                uint32_t targetSr = (currentSr == 96000) ? 48000 : 96000;
                
                // NO CriticalSectionGuard here - setSampleRate uses vTaskDelay internally
                setSampleRate(targetSr);
            }
            break;
        }

        case MidiEvent::PB2_WIPER_TOGGLE: {
            bool targetState = (action.val >= 64);
            if (isPB2WiperMode != targetState) {
                CriticalSectionGuard lock(MidiRouter::paramMux);
                isPB2WiperMode = targetState; 
                dspNeedsCommit = true; 
                pb2ToggleRequested = true; 
            }
            break;
        }

        case MidiEvent::VOL_MODE_TOGGLE: {
            static unsigned long lastVolToggle = 0;
            if (action.val > 0 && millis() - lastVolToggle > 250) {
                lastVolToggle = millis();
                
                CriticalSectionGuard lock(MidiRouter::paramMux);
                isVolumeMode = !isVolumeMode;
                
                if (!isVolumeMode) {
                    volumePedalGain = 1.0f; 
                    pedals.lockPB3Whammy(); 
                    currentPB3 = 8192; 
                    lastActivePedal = 8192;
                    activePedalSource = 0;
                    Control_Surface.sendPitchBend(Channel_3, 8192);
                } else {
                    pedals.lockPB3Volume(); 
                    lastActivePedal = 8192; 
                    activePedalSource = 0;
                    volumePedalGain = (float)currentPB3 / 16383.0f;
                }
                
                float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
                if (currentLUT) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release);
                
                dspNeedsCommit = true; 
                settingsNeedSaving = true; 
                lastParameterChangeTime = millis();
            }
            break;
        }

        case MidiEvent::TOGGLE_EFFECT: {
            static unsigned long lastTgl = 0;
            if (action.val > 0 && millis() - lastTgl > 250) {
                lastTgl = millis();
                
                CriticalSectionGuard lock(MidiRouter::paramMux);
                if (action.targetEffect == 99) {
                    bool newState = !isMonoPolyActive.load(std::memory_order_acquire);
                    isMonoPolyActive.store(newState, std::memory_order_release);
                    isWhammyActive = !newState;
                }
                else if (action.targetEffect == 1) isFrozen = !isFrozen;
                else if (action.targetEffect == 2) isFeedbackActive = !isFeedbackActive;
                else if (action.targetEffect == 3) isHarmonizerMode = !isHarmonizerMode;
                else if (action.targetEffect == 4) {
                    isCapoMode = !isCapoMode;
                    lutNeedsUpdate = true;
                    asyncLutUpdateRequested.store(true, std::memory_order_release);
                }
                else if (action.targetEffect == 5) isSynthMode = !isSynthMode;
                else if (action.targetEffect == 6) isPadMode = !isPadMode;
                else if (action.targetEffect == 7) isChorusMode = !isChorusMode;
                else if (action.targetEffect == 8) isSwellMode = !isSwellMode;
                else if (action.targetEffect == 9) isVibratoMode = !isVibratoMode;
                
                dspNeedsCommit = true; 
                settingsNeedSaving = true; 
                lastParameterChangeTime = millis();
            }
            break;
        }

        case MidiEvent::STEP_TOE: {
            float step = (action.val >= 64) ? -1.0f : 1.0f;
            {
                CriticalSectionGuard lock(MidiRouter::paramMux);
                if (currentMode == 0 || currentMode == 1 || currentMode == 8) {
                    effectMemory[0] = constrain(effectMemory[0] + step, -24.0f, 24.0f);
                } else if (currentMode == 4) {
                    fxParams[4][1] = constrain(fxParams[4][1] + (step * 0.01f), -1.0f, 1.0f);
                } else {
                    effectMemory[currentMode] = constrain(effectMemory[currentMode] + step, -24.0f, 24.0f);
                }
            }
            lutNeedsUpdate = true; 
            asyncLutUpdateRequested.store(true, std::memory_order_release);
            dspNeedsCommit = true; 
            settingsNeedSaving = true; 
            lastParameterChangeTime = millis();
            break;
        }

        case MidiEvent::STEP_HEEL: {
            float step = (action.val >= 64) ? -1.0f : 1.0f;
            {
                CriticalSectionGuard lock(MidiRouter::paramMux);
                if (currentMode == 0 || currentMode == 1 || currentMode == 8) {
                    effectMemory[1] = constrain(effectMemory[1] + step, -24.0f, 24.0f);
                } else if (currentMode == 4) {
                    effectMemory[4] = constrain(effectMemory[4] + step, -24.0f, 24.0f);
                } else if (currentMode == 2) {
                    int fbIdx = feedbackIntervalIdx.load(std::memory_order_acquire);
                    feedbackIntervalIdx.store(step > 0 ? (fbIdx + 1) % 5 : (fbIdx + 4) % 5, std::memory_order_release);
                } else {
                    effectMemory[currentMode] = constrain(effectMemory[currentMode] + step, -24.0f, 24.0f);
                }
            }
            lutNeedsUpdate = true; 
            asyncLutUpdateRequested.store(true, std::memory_order_release);
            dspNeedsCommit = true; 
            settingsNeedSaving = true; 
            lastParameterChangeTime = millis();
            break;
        }

        case MidiEvent::NONE: 
            break;
    }

    return false;
}