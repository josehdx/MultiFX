#ifndef FX_CHORUS_H
#define FX_CHORUS_H

#include "DSPEngine.h"

class FX_Chorus {
public:
    static inline float getSpd(bool active, float currentPitch, float chorusRatio, volatile float& localChoPhase, float phaseIncr, const float* lfoLUT) __attribute__((always_inline)) {
        float spd = currentPitch * chorusRatio;
        if (active) {
            localChoPhase += phaseIncr;
            if (localChoPhase >= 1024.0f) localChoPhase -= 1024.0f;
            spd *= DSPEngine::getLfoInterpolated(localChoPhase, lfoLUT);
        }
        return spd;
    }
};

#endif