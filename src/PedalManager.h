#pragma once
#include <Arduino.h>

// =========================================================================
// 🔒 EXPRESSION PEDAL HARDWARE CALIBRATION - DO NOT TOUCH 🔒
// These values are perfectly tuned for TRS capacitance, insertion shorts, 
// and mechanical slop. Modifying them will break pedal tracking.
// =========================================================================
const int TRS_UNPLUGGED_VOLTAGE = 4050; // Pin floats high when unplugged
const int TRS_PLUGGED_VOLTAGE   = 3900; // Safe threshold for seated plug
const int TRS_INSERTION_LOCKOUT = 300;  // 1.5s (300 frames) transient blind spot
const int PEDAL_CENTER_FLEX     = 200;  // Dynamic bounds (Center ± 200)
const int PEDAL_OUTER_DEADZONE  = 150;  // Absorbs physical heel/toe slop
const int PB3_DEFAULT_MIN       = 1000; // PB3 Base min limit
const int PB3_DEFAULT_MAX       = 3000; // PB3 Base max limit
const int CENTER_MIDI_VAL       = 8192; // 50% Neutral
const int MAX_MIDI_VAL          = 16383;// 100% Toe / Full Volume
const int MIN_MIDI_VAL          = 0;    // 0% Heel
const int DEADZONE_SIZE         = 400;  // Center snapping deadzone
// =========================================================================

class PedalManager {
private:
    int pb1_center = 2048, pb1_min = 1000, pb1_max = 3000;
    int pb2_center = 2048, pb2_min = 1000, pb2_max = 3000;
    int pb3_min = 1000, pb3_max = 3000;
    
    int stableRawA = -1, stableRawB = -1, stableRawC = -1;
    bool unpluggedA = false, unpluggedB = false, unpluggedC = false;
    
    int recoveryA = 0, recoveryB = 0, recoveryC = 0;
    int systemRecoveryFrames = 50;

    int calA = CENTER_MIDI_VAL, calB = CENTER_MIDI_VAL, calC = CENTER_MIDI_VAL;
    int lastMidiA = CENTER_MIDI_VAL, lastMidiB = CENTER_MIDI_VAL, lastMidiC = CENTER_MIDI_VAL;

    // Movement Pick-Up Lock states for PB3 (Whammy and Volume Modes)
    bool pb3_whammy_lock = true;
    int pb3_whammy_ref = -1;
    
    bool pb3_vol_lock = true;
    int pb3_vol_ref = -1;

    int map_raw_deadzone(int raw, uint16_t center, uint16_t rMin, uint16_t rMax, int dZone) {
        int deadLower = center - dZone; 
        int deadUpper = center + dZone;
        int effMin = rMin + PEDAL_OUTER_DEADZONE; 
        int effMax = rMax - PEDAL_OUTER_DEADZONE;
        
        // Prevent Integer Division-by-Zero Kernel Panic
        if (deadLower - effMin < 1) effMin = deadLower - 1; 
        if (effMax - deadUpper < 1) effMax = deadUpper + 1;
        
        if (raw <= effMin) return MIN_MIDI_VAL; 
        if (raw >= effMax) return MAX_MIDI_VAL; 
        if (raw >= deadLower && raw <= deadUpper) return CENTER_MIDI_VAL;
        
        long mappedValue;
        if (raw < deadLower) { 
            mappedValue = map(raw, effMin, deadLower, MIN_MIDI_VAL, 8191); 
        } else { 
            mappedValue = map(raw, deadUpper, effMax, 8193, MAX_MIDI_VAL); 
        }
        return constrain(mappedValue, MIN_MIDI_VAL, MAX_MIDI_VAL);
    }

    int map_raw_expression(int raw, uint16_t rMin, uint16_t rMax, bool invert) {
        int heelLockZone = 350; int toeLockZone = 300; int lowerLimit, upperLimit;
        if (!invert) {
            lowerLimit = rMin + heelLockZone; upperLimit = rMax - toeLockZone;
            if (lowerLimit >= upperLimit) return MIN_MIDI_VAL; // Guard against division-by-zero
            if (raw <= lowerLimit) return MIN_MIDI_VAL; if (raw >= upperLimit) return MAX_MIDI_VAL; 
            return map(raw, lowerLimit, upperLimit, MIN_MIDI_VAL, MAX_MIDI_VAL);
        } else {
            lowerLimit = rMin + toeLockZone; upperLimit = rMax - heelLockZone;
            if (lowerLimit >= upperLimit) return MAX_MIDI_VAL; // Guard against division-by-zero
            if (raw <= lowerLimit) return MAX_MIDI_VAL; if (raw >= upperLimit) return MIN_MIDI_VAL; 
            return map(raw, lowerLimit, upperLimit, MAX_MIDI_VAL, MIN_MIDI_VAL);
        }
    }

public:
    void lockPB3Whammy() {
        pb3_whammy_lock = true;
        pb3_whammy_ref = -1;
    }

    void lockPB3Volume() {
        pb3_vol_lock = true;
        pb3_vol_ref = -1;
    }

    void setCenters(int cA, int cB, int cC) {
        pb1_center = (cA > 4000 || cA < 100) ? 2048 : cA;
        pb2_center = (cB > 4000 || cB < 100) ? 2048 : cB;
        
        unpluggedA = (cA > 4000);
        unpluggedB = (cB > 4000);
        unpluggedC = (cC > 4000);

        pb1_min = pb1_center - PEDAL_CENTER_FLEX;
        pb1_max = pb1_center + PEDAL_CENTER_FLEX;
        pb2_min = pb2_center - PEDAL_CENTER_FLEX;
        pb2_max = pb2_center + PEDAL_CENTER_FLEX;
        pb3_min = PB3_DEFAULT_MIN;
        pb3_max = PB3_DEFAULT_MAX;
    }

    void triggerSystemRecovery() {
        systemRecoveryFrames = 50;
        pb1_min = pb1_center - PEDAL_CENTER_FLEX;
        pb1_max = pb1_center + PEDAL_CENTER_FLEX;
        pb2_min = pb2_center - PEDAL_CENTER_FLEX;
        pb2_max = pb2_center + PEDAL_CENTER_FLEX;
    }

    void resetToCenter() {
        calA = CENTER_MIDI_VAL;
        calB = CENTER_MIDI_VAL;
        calC = CENTER_MIDI_VAL;
        lastMidiA = CENTER_MIDI_VAL;
        lastMidiB = CENTER_MIDI_VAL;
        lastMidiC = CENTER_MIDI_VAL;
    }

    void process(int rawA, int rawB, int rawC, bool isVolumeMode, bool invertPB3) {
        // Continuous Exponential Moving Average (EMA) filter
        if (stableRawA < 0) stableRawA = rawA; else stableRawA = (stableRawA * 7 + rawA) / 8;
        if (stableRawB < 0) stableRawB = rawB; else stableRawB = (stableRawB * 7 + rawB) / 8;
        if (stableRawC < 0) stableRawC = rawC; else stableRawC = (stableRawC * 7 + rawC) / 8;

        // Unplug State Machine
        if (stableRawA > TRS_UNPLUGGED_VOLTAGE) unpluggedA = true;
        else if (stableRawA < TRS_PLUGGED_VOLTAGE) unpluggedA = false;

        if (stableRawB > TRS_UNPLUGGED_VOLTAGE) unpluggedB = true;
        else if (stableRawB < TRS_PLUGGED_VOLTAGE) unpluggedB = false;
        
        if (stableRawC > TRS_UNPLUGGED_VOLTAGE) unpluggedC = true;
        else if (stableRawC < TRS_PLUGGED_VOLTAGE) unpluggedC = false;

        if (systemRecoveryFrames > 0) systemRecoveryFrames--;

        // PB1 Tracking & Recovery
        if (unpluggedA) {
            recoveryA = TRS_INSERTION_LOCKOUT;
            pb1_min = pb1_center - PEDAL_CENTER_FLEX;
            pb1_max = pb1_center + PEDAL_CENTER_FLEX;
        } else if (recoveryA > 0) {
            recoveryA--;
        } else if (systemRecoveryFrames == 0) {
            if (stableRawA < pb1_min && stableRawA > 200) pb1_min = stableRawA;
            if (stableRawA > pb1_max && stableRawA <= 4095) pb1_max = stableRawA;
        }

        // PB2 Tracking & Recovery
        if (unpluggedB) {
            recoveryB = TRS_INSERTION_LOCKOUT;
            pb2_min = pb2_center - PEDAL_CENTER_FLEX;
            pb2_max = pb2_center + PEDAL_CENTER_FLEX;
        } else if (recoveryB > 0) {
            recoveryB--;
        } else if (systemRecoveryFrames == 0) {
            if (stableRawB < pb2_min && stableRawB > 200) pb2_min = stableRawB;
            if (stableRawB > pb2_max && stableRawB <= 4095) pb2_max = stableRawB;
        }
        
        // PB3 Tracking & Recovery
        if (unpluggedC) {
            recoveryC = TRS_INSERTION_LOCKOUT;
            pb3_min = PB3_DEFAULT_MIN;
            pb3_max = PB3_DEFAULT_MAX;
        } else if (recoveryC > 0) {
            recoveryC--;
        } else if (systemRecoveryFrames == 0) {
            if (stableRawC < pb3_min && stableRawC > 200) pb3_min = stableRawC;
            if (stableRawC > pb3_max && stableRawC <= 4095) pb3_max = stableRawC;
        }

        // Leaky boundary recovery to prevent permanent tracking glitches
        static int leakTimer = 0;
        if (++leakTimer > 100) { // Every ~500ms
            if (pb1_min < pb1_center - PEDAL_CENTER_FLEX) pb1_min++;
            if (pb1_max > pb1_center + PEDAL_CENTER_FLEX) pb1_max--;
            if (pb2_min < pb2_center - PEDAL_CENTER_FLEX) pb2_min++;
            if (pb2_max > pb2_center + PEDAL_CENTER_FLEX) pb2_max--;
            if (pb3_min < PB3_DEFAULT_MIN) pb3_min++;
            if (pb3_max > PB3_DEFAULT_MAX) pb3_max--;
            leakTimer = 0;
        }
        
        // Apply deadzone and expression mappings
        calA = map_raw_deadzone(stableRawA, pb1_center, pb1_min, pb1_max, DEADZONE_SIZE);
        calB = map_raw_deadzone(stableRawB, pb2_center, pb2_min, pb2_max, DEADZONE_SIZE);
        calC = map_raw_expression(stableRawC, pb3_min, pb3_max, invertPB3);
        
        // Applies Movement Pick-Up Lock to force PB3 outputs to neutral states until physically moved
        if (!isVolumeMode) {
            if (pb3_whammy_lock) {
                if (pb3_whammy_ref == -1) pb3_whammy_ref = calC;
                if (abs(calC - pb3_whammy_ref) > 400) { 
                    pb3_whammy_lock = false;
                } else {
                    calC = CENTER_MIDI_VAL; // Lock at Center (0 pitch shift)
                }
            }
        } else {
            if (pb3_vol_lock) {
                if (pb3_vol_ref == -1) pb3_vol_ref = calC;
                if (abs(calC - pb3_vol_ref) > 400) {
                    pb3_vol_lock = false;
                } else {
                    calC = MAX_MIDI_VAL; // Lock at Toe Down (100% volume)
                }
            }
        }

        // Override outputs to safe defaults during unplug/transient states
        if (unpluggedA || systemRecoveryFrames > 0 || recoveryA > 0) calA = CENTER_MIDI_VAL;
        if (unpluggedB || systemRecoveryFrames > 0 || recoveryB > 0) calB = CENTER_MIDI_VAL;
        if (unpluggedC || systemRecoveryFrames > 0 || recoveryC > 0) {
            calC = isVolumeMode ? MAX_MIDI_VAL : CENTER_MIDI_VAL;
        }
    }

    bool hasMovedA() { return (abs(calA - lastMidiA) > 12) || ((calA == CENTER_MIDI_VAL || calA == MIN_MIDI_VAL || calA == MAX_MIDI_VAL) && calA != lastMidiA); }
    bool hasMovedB() { return (abs(calB - lastMidiB) > 12) || ((calB == CENTER_MIDI_VAL || calB == MIN_MIDI_VAL || calB == MAX_MIDI_VAL) && calB != lastMidiB); }
    bool hasMovedC() { return (abs(calC - lastMidiC) >= 128) || ((calC == MIN_MIDI_VAL || calC == MAX_MIDI_VAL || calC == CENTER_MIDI_VAL) && calC != lastMidiC); }

    int getCalA() { return calA; }
    int getCalB() { return calB; }
    int getCalC() { return calC; }

    void updateLastMidiA() { lastMidiA = calA; }
    void updateLastMidiB() { lastMidiB = calB; }
    void updateLastMidiC() { lastMidiC = calC; }
};