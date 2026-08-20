#pragma once
#include "SystemState.h"
#include "SystemActions.h"

inline bool channelMessageCallback(ChannelMessage cm) {
    lastActivityTime = millis();
    if (cm.header != 0xB0) return false;
    MidiAction action = MidiRouter::parseMessage(cm.data1, cm.data2);
    if (action.event == MidiEvent::NONE) return false;
    int currentMode = activeEffectMode.load(std::memory_order_acquire);
    
    switch (action.event) {
        case MidiEvent::EXPRESSION_UPDATE:
            currentCC11 = action.rawValue; currentPB3 = action.rawValue; lastActivePedal = action.rawValue;
            pedals.updateLastMidiC();
            if (isVolumeMode) {
                volumePedalGain = (float)action.rawValue / 16383.0f; dspNeedsCommit = true;
                Control_Surface.sendControlChange({19, Channel_1}, action.val);
            } else {
                if (!asyncLutUpdateRequested.load(std::memory_order_acquire)) {
                    float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
                    if (currentLUT) pitchShiftFactor.store(currentLUT[action.rawValue], std::memory_order_release);
                }
            }
            break;
        case MidiEvent::KNOB_UPDATE: {
            CriticalSectionGuard lock(MidiRouter::paramMux);
            MidiRouter::updateParameter(action.cc, action.val, currentMode, effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx);
            settingsNeedSaving = true; lastParameterChangeTime = millis();
            break;
        }
        case MidiEvent::PREV_MODE: switchEffectMode(currentMode - 1); break;
        case MidiEvent::NEXT_MODE: switchEffectMode(currentMode + 1); break;
        case MidiEvent::LATENCY_CYCLE: cycleLatencyMode(); break;
        case MidiEvent::PANIC_RESET: triggerPanicReset(); break;
        case MidiEvent::SR_TOGGLE: sampleRateToggleRequested = true; break;
        case MidiEvent::PB2_WIPER_TOGGLE: isPB2WiperMode = !isPB2WiperMode; dspNeedsCommit = true; pb2ToggleRequested = true; break;
        case MidiEvent::VOL_MODE_TOGGLE:
            isVolumeMode = !isVolumeMode;
            if (!isVolumeMode) {
                volumePedalGain = 1.0f; pedals.lockPB3Whammy(); currentPB3 = 8192; lastActivePedal = 8192;
                Control_Surface.sendPitchBend(Channel_3, 8192);
            } else {
                pedals.lockPB3Volume(); lastActivePedal = 8192; volumePedalGain = (float)currentPB3 / 16383.0f;
            }
            if (!asyncLutUpdateRequested.load(std::memory_order_acquire)) {
                float* currentLUT = activePitchLUT.load(std::memory_order_acquire);
                if (currentLUT) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release);
            }
            dspNeedsCommit = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
            break;
        case MidiEvent::TOGGLE_EFFECT:
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
                if (currentMode == 4 && isCapoMode) lutNeedsUpdate = true;
            }
            dspNeedsCommit = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
            break;
        case MidiEvent::STEP_PARAM_UP:
        case MidiEvent::STEP_PARAM_DOWN: {
            float step = (action.event == MidiEvent::STEP_PARAM_DOWN) ? -1.0f : 1.0f;
            {
                CriticalSectionGuard lock(MidiRouter::paramMux);
                if (currentMode == 0 || currentMode == 1 || currentMode == 8) effectMemory[1] = constrain(effectMemory[1] + step, -24.0f, 24.0f);
                else if (currentMode == 4) effectMemory[4] = constrain(effectMemory[4] + step, -24.0f, 24.0f);
                else if (currentMode == 2) {
                    int fbIdx = feedbackIntervalIdx.load(std::memory_order_acquire);
                    feedbackIntervalIdx.store(step > 0 ? (fbIdx + 1) % 5 : (fbIdx + 4) % 5, std::memory_order_release);
                } else {
                    effectMemory[currentMode] = constrain(effectMemory[currentMode] + step, -24.0f, 24.0f);
                }
            }
            lutNeedsUpdate = true; dspNeedsCommit = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
            break;
        }
        case MidiEvent::NONE: break;
    }
    return false;
}