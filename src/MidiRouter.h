#pragma once
#include <Arduino.h>
#include <atomic>

// 1. Define the possible actions the MIDI controller can trigger
enum class MidiEvent {
    NONE,
    PREV_MODE, NEXT_MODE, LATENCY_CYCLE, PANIC_RESET, SR_TOGGLE,
    PB2_WIPER_TOGGLE, VOL_MODE_TOGGLE, TOGGLE_EFFECT,
    STEP_PARAM_UP, STEP_PARAM_DOWN, EXPRESSION_UPDATE, KNOB_UPDATE
};

// 2. Define the payload passed back to main.cpp
struct MidiAction {
    MidiEvent event = MidiEvent::NONE;
    int targetEffect = -1;  // Used for effect toggles
    uint16_t rawValue = 0;  // Used for 14-bit expression mapping
    uint8_t cc = 0;
    uint8_t val = 0;
};

class MidiRouter {
public:
    // 3. The Decoupled Translator
    static MidiAction parseMessage(uint8_t cc, uint8_t val) {
        MidiAction action;
        action.cc = cc;
        action.val = val;

        // Route Expression Pedal
        if (cc == 11) {
            action.event = MidiEvent::EXPRESSION_UPDATE;
            action.rawValue = map(val, 0, 127, 0, 16383);
            return action;
        }
        
        // Route Dynamic Parameter Knobs
        if (cc >= 24 && cc <= 28) {
            action.event = MidiEvent::KNOB_UPDATE;
            return action;
        }

        // Ignore button releases (values < 64) for toggle switches
        if (val < 64) return action;

        // Route System Commands & Toggles
        switch (cc) {
            case 0:  action.event = MidiEvent::PREV_MODE; break;
            case 1:  action.event = MidiEvent::NEXT_MODE; break;
            case 2:  action.event = MidiEvent::LATENCY_CYCLE; break;
            case 3:  action.event = MidiEvent::PANIC_RESET; break;
            case 4:  action.event = MidiEvent::SR_TOGGLE; break;
            case 5:  action.event = MidiEvent::PB2_WIPER_TOGGLE; break;
            case 6:  action.event = MidiEvent::VOL_MODE_TOGGLE; break;
            case 7:  action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 9; break; // Vibrato
            case 8:  action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 1; break; // Freeze
            case 9:  action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 2; break; // Feedback
            case 10: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 3; break; // Harmony
            case 12: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 4; break; // Capo
            case 13: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 5; break; // Synth
            case 14: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 6; break; // Pad
            case 15: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 7; break; // Chorus
            case 16: action.event = MidiEvent::TOGGLE_EFFECT; action.targetEffect = 8; break; // Swell
            case 17: action.event = MidiEvent::STEP_PARAM_DOWN; break;
            case 18: action.event = MidiEvent::STEP_PARAM_UP; break;
        }
        return action;
    }

    // 4. The Parameter Engine
    static void updateParameter(uint8_t cc, uint8_t val, int currentMode, volatile float* effectMemory, volatile float fxParams[10][5], volatile bool& lutNeedsUpdate, volatile bool& dspNeedsCommit, std::atomic<int>& feedbackIntervalIdx) {
        float norm = (float)val / 127.0f; 
        int pIdx = cc - 24; 
        
        if (currentMode == 0) { 
            if (pIdx == 0) { effectMemory[1] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) { effectMemory[0] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 2) fxParams[0][0] = norm; 
            if (pIdx == 3) fxParams[0][1] = norm; 
        } 
        else if (currentMode == 1) { 
            if (pIdx == 0) { effectMemory[1] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) { effectMemory[0] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 2) fxParams[1][0] = 0.0f + (norm * 0.95f);               
            if (pIdx == 3) fxParams[1][1] = 0.00001f + (norm * 0.001f);          
            if (pIdx == 4) fxParams[1][2] = 0.00001f + (norm * 0.0005f);         
        }
        else if (currentMode == 2) { 
            if (pIdx == 0) { 
                int newIdx = constrain((int)roundf(norm * 4.0f), 0, 4);
                if (newIdx != feedbackIntervalIdx.load(std::memory_order_acquire)) {
                    feedbackIntervalIdx.store(newIdx, std::memory_order_release);
                    lutNeedsUpdate = true;
                }
            }
            if (pIdx == 1) fxParams[2][0] = 1000.0f + (norm * 10000.0f);         
            if (pIdx == 2) fxParams[2][1] = 1.0f + (norm * 100.0f);             
            if (pIdx == 3) fxParams[2][2] = 0.005f + (norm * 0.045f);            
        }
        else if (currentMode == 3) { 
            if (pIdx == 0) { effectMemory[3] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) fxParams[3][0] = norm;                                
        }
        else if (currentMode == 4) { 
            if (pIdx == 0) { 
                int cents = (int)roundf((effectMemory[4] - (float)roundf(effectMemory[4])) * 100.0f); 
                effectMemory[4] = constrain(roundf((norm * 48.0f) - 24.0f) + ((float)cents / 100.0f), -24.0f, 24.0f);
                lutNeedsUpdate = true; 
            }
            if (pIdx == 1) { 
                int semi = (int)roundf(effectMemory[4]); 
                float c = roundf((norm * 100.0f) - 50.0f) / 100.0f; 
                effectMemory[4] = constrain((float)semi + c, -24.0f, 24.0f);
                lutNeedsUpdate = true; 
            }
        }
        else if (currentMode == 5) { 
            if (pIdx == 0) { effectMemory[5] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) fxParams[5][0] = 0.01f + (norm * 0.5f);               
            if (pIdx == 2) fxParams[5][1] = 0.001f + (norm * 0.05f);             
            if (pIdx == 3) fxParams[5][2] = 0.1f + (norm * 0.8f);               
            if (pIdx == 4) fxParams[5][3] = norm;                                
        }
        else if (currentMode == 6) { 
            if (pIdx == 0) { effectMemory[6] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) fxParams[6][0] = 0.8f + (norm * 0.199f);              
            if (pIdx == 2) fxParams[6][1] = norm * 3.0f;                         
        }
        else if (currentMode == 7) { 
            if (pIdx == 0) { effectMemory[7] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) fxParams[7][0] = 500.0f + (norm * 4500.0f);           
            if (pIdx == 2) fxParams[7][1] = norm;                                
        }
        else if (currentMode == 8) { 
            if (pIdx == 0) { effectMemory[1] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) { effectMemory[0] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 2) fxParams[8][0] = 0.001f + (norm * 0.05f);             
            if (pIdx == 3) fxParams[8][1] = 0.00001f + (norm * 0.0005f);        
            if (pIdx == 4) fxParams[8][2] = 0.00001f + (norm * 0.0005f);        
        }
        else if (currentMode == 9) { 
            if (pIdx == 0) { effectMemory[9] = roundf((norm * 48.0f) - 24.0f); lutNeedsUpdate = true; } 
            if (pIdx == 1) fxParams[9][0] = norm * 2.0f;                         
        }
        dspNeedsCommit = true;
    }
};