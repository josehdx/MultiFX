#pragma once

#include "SystemState.h"
#include "SystemActions.h"
#include "MidiRouter.h"

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
        case MidiEvent::EXPRESSION_UPDATE:
            currentCC11 = action.rawValue; 
            currentPB3 = action.rawValue; 
            lastActivePedal = action.rawValue;
            activePedalSource = 0; // CC11 Active
            pedals.updateLastMidiC();

            if (isVolumeMode) {
                volumePedalGain = (float)action.rawValue / 16383.0f; 
                dspNeedsCommit = true;
                Control_Surface.sendControlChange({19, Channel_1}, action.val);
            } else {
                // FIX: Removed asyncLutUpdateRequested guard. The guard was preventing
                // pitchShiftFactor from updating during a LUT rebuild (which can take
                // hundreds of ms), making the pitch shift effect feel unresponsive.
                // The atomic activePitchLUT pointer is safe to read concurrently; the
                // null-check below is the only guard needed.
                float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
                if (currentLUT) pitchShiftFactor.store(currentLUT[action.rawValue], std::memory_order_release);
            }
            break;

        case MidiEvent::KNOB_UPDATE: {
            CriticalSectionGuard lock(MidiRouter::paramMux);
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
            // FIX: Added 500ms debounce to prevent rapid-fire MIDI CCs from stacking
            // multiple concurrent setSampleRate() calls. Each call blocks with a
            // vTaskDelay loop waiting for dsp_ack_parked — overlapping calls cause
            // USBMscManager's resume path to race and prematurely clear dsp_is_paused,
            // leaving setSampleRate() spinning forever in the ack-wait loop (system hang).
            static unsigned long lastSrToggle = 0;
            if (millis() - lastSrToggle > 500) {
                lastSrToggle = millis();
                
                // Read the current state and invert it, ignoring action.val
                uint32_t currentSr = currentSampleRate.load(std::memory_order_acquire);
                uint32_t targetSr = (currentSr == 96000) ? 48000 : 96000;
                
                setSampleRate(targetSr);
            }
            break;
        }

        case MidiEvent::PB2_WIPER_TOGGLE: {
            bool targetState = (action.val >= 64);
            if (isPB2WiperMode != targetState) {
                isPB2WiperMode = targetState; 
                dspNeedsCommit = true; 
                pb2ToggleRequested = true; 
            }
            break;
        }

        case MidiEvent::VOL_MODE_TOGGLE: {
            bool targetState = (action.val >= 64);
            if (isVolumeMode != targetState) {
                isVolumeMode = targetState;
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
                // FIX: Removed asyncLutUpdateRequested guard (same reason as EXPRESSION_UPDATE).
                // When switching volume mode, the pitch must snap to centre immediately.
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
                if (action.targetEffect == 99) {
                    bool newState = !isMonoPolyActive.load(std::memory_order_acquire);
                    isMonoPolyActive.store(newState, std::memory_order_release);
                    if (newState) isWhammyActive = false;
                }
                if (action.targetEffect == 1) isFrozen = !isFrozen;
                else if (action.targetEffect == 2) isFeedbackActive = !isFeedbackActive;
                else if (action.targetEffect == 3) isHarmonizerMode = !isHarmonizerMode;
                else if (action.targetEffect == 4) isCapoMode = !isCapoMode;
                else if (action.targetEffect == 5) isSynthMode = !isSynthMode;
                else if (action.targetEffect == 6) isPadMode = !isPadMode;
                else if (action.targetEffect == 7) isChorusMode = !isChorusMode;
                else if (action.targetEffect == 8) isSwellMode = !isSwellMode;
                else if (action.targetEffect == 9) isVibratoMode = !isVibratoMode;
                
                if (currentMode == action.targetEffect) {
                    isWhammyActive = !isWhammyActive;
                    if (currentMode == 4 && isCapoMode) {
                        lutNeedsUpdate = true;
                        asyncLutUpdateRequested.store(true, std::memory_order_release);
                    }
                }
                dspNeedsCommit = true; 
                settingsNeedSaving = true; 
                lastParameterChangeTime = millis();
            }
            break;
        }

        // CC 17: Controls TOE Range (127 = Decrease, 0 = Increase)
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

        // CC 18: Controls HEEL Range (127 = Decrease, 0 = Increase)
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