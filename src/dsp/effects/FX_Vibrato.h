#ifndef FX_VIBRATO_H
#define FX_VIBRATO_H

#include "DSPEngine.h"

class FX_Vibrato {
public:
    static inline float getSpd(bool active, float currentPitch, volatile float& localVibPhase, float phaseIncr, float depth, const float* lfoLUT) __attribute__((always_inline)) {
        float spd = currentPitch;
        if (active) {
            localVibPhase += phaseIncr;
            if (localVibPhase >= 1024.0f) localVibPhase -= 1024.0f;
            spd *= 1.0f + ((DSPEngine::getLfoInterpolated(localVibPhase, lfoLUT) - 1.0f) * depth);
        }
        return spd;
    }
};

#endif