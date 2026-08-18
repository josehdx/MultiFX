#ifndef MIDI_ROUTER_H
#define MIDI_ROUTER_H

#include <Arduino.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "SpinlockGuard.h"

enum class MidiEvent : uint8_t {
    NONE = 0,
    EXPRESSION_UPDATE,
    KNOB_UPDATE,
    PREV_MODE,
    NEXT_MODE,
    LATENCY_CYCLE,
    PANIC_RESET,
    SR_TOGGLE,
    PB2_WIPER_TOGGLE,
    VOL_MODE_TOGGLE,
    TOGGLE_EFFECT,
    STEP_PARAM_UP,
    STEP_PARAM_DOWN
};

struct MidiAction {
    MidiEvent event = MidiEvent::NONE;
    uint8_t cc = 0;
    uint8_t val = 0;
    uint16_t rawValue = 0;
    int targetEffect = 0;
};

class MidiRouter {
public:
    inline static portMUX_TYPE paramMux = portMUX_INITIALIZER_UNLOCKED;

    static inline MidiAction parseMessage(uint8_t data1, uint8_t data2) {
        MidiAction action;
        action.cc = data1;
        action.val = data2;

        switch (data1) {
            // --- SYSTEM & GLOBAL CONTROLS ---
            case 0: action.event = MidiEvent::PREV_MODE; break;       // Cycles backward
            case 1: action.event = MidiEvent::NEXT_MODE; break;       // Cycles forward
            case 2: action.event = MidiEvent::SR_TOGGLE; break;       // Sample rate toggle
            case 3: action.event = MidiEvent::LATENCY_CYCLE; break;   // Latency toggle
            case 4: action.event = MidiEvent::PANIC_RESET; break;     // Panic reset
            // case 5: No assignment yet
            case 6: action.event = MidiEvent::VOL_MODE_TOGGLE; break; // Vol toggle
            
            case 11: // Expression Pedal / Pitch Bend proxy
                action.event = MidiEvent::EXPRESSION_UPDATE;
                action.rawValue = (uint16_t)data2 * 128; 
                break;
                
            case 17: action.event = MidiEvent::STEP_PARAM_DOWN; break; // Preserved for param steps
            case 18: action.event = MidiEvent::STEP_PARAM_UP; break;   // Preserved for param steps

            // --- DIRECT EFFECT TOGGLES ---
            case 7:  action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 9; break; // Vibrato
            case 8:  action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 1; break; // Freeze
            case 9:  action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 2; break; // Feedback
            case 10: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 3; break; // Harmonizer
            case 12: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 4; break; // Capo
            case 13: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 5; break; // Synth
            case 14: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 6; break; // Pad
            case 15: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 7; break; // Chorus
            case 16: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 8; break; // Swell

            // --- HARDWARE KNOB PARAMETERS ---
            case 24: case 25: case 26: case 27: case 28:
                action.event = MidiEvent::KNOB_UPDATE;
                break;

            default:
                action.event = MidiEvent::NONE;
                break;
        }
        return action;
    }

    static inline void updateParameter(
        uint8_t cc, 
        uint8_t val, 
        int currentMode, 
        volatile float* effectMemory, 
        float fxParams[10][5], 
        volatile bool& lutNeedsUpdate, 
        volatile bool& dspNeedsCommit, 
        std::atomic<int>& feedbackIntervalIdx) 
    {
        // Calculate the parameter index (0-4) using the new base CC 24
        int paramIdx = cc - 24; 
        if (paramIdx < 0 || paramIdx >= 5) return;
        
        float normalizedVal = (float)val / 127.0f;
        {
            CriticalSectionGuard lock(paramMux);
            fxParams[currentMode][paramIdx] = normalizedVal;
            
            if (currentMode == 2 && paramIdx == 0) { // Feedback Interval Mode
                int fbIdx = (int)(normalizedVal * 4.99f);
                feedbackIntervalIdx.store(fbIdx, std::memory_order_release);
            }
        }
        lutNeedsUpdate = true;
        dspNeedsCommit = true;
    }
};

#endif // MIDI_ROUTER_H