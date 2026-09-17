#ifndef FX_SYNTH_H
#define FX_SYNTH_H

#include "DSPEngine.h"

class FX_Synth {
public:
    static inline void process(bool active, float env, float att, float rel, float flt, float mix, float srScale, const float* synthLUT, float& inSample, float& synthEnv, float& synthFilter, float& synthBandpass) __attribute__((always_inline)) {
        if(active) {
            synthEnv = (env > 0.005f) ? __builtin_fminf(1.0f, __builtin_fmaf(att, srScale, synthEnv)) : __builtin_fmaxf(0.0f, __builtin_fmaf(-rel, srScale, synthEnv));
            float clampedProc = __builtin_fmaxf(-1.0f, __builtin_fminf(inSample, 1.0f));
            int waveIdx = (int)((clampedProc + 1.0f) * 1023.5f) & 2047;
            float procSample = synthLUT[waveIdx];
            float f1 = __builtin_fmaxf(0.001f, __builtin_fminf(0.45f, __builtin_fmaf(0.5f * synthEnv, srScale, flt * srScale)));
            float hp = procSample - synthFilter - (0.5f * synthBandpass);
            synthFilter = DSPEngine::AntiDenormal(synthFilter + f1 * synthBandpass + 1e-9f);
            synthBandpass = DSPEngine::AntiDenormal(synthBandpass + f1 * hp + 1e-9f);
            inSample = synthFilter * mix;
        }
    }
};

#endif