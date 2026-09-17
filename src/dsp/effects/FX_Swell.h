#ifndef FX_SWELL_H
#define FX_SWELL_H

#include <math.h>

class FX_Swell {
public:
    static inline void process(bool active, float env, float thr, float att, float rel, float srScale, float& swellGain) __attribute__((always_inline)) {
        if(active) {
            swellGain = (env > thr) ? __builtin_fminf(1.0f, __builtin_fmaf(att, srScale, swellGain)) : __builtin_fmaxf(0.0f, __builtin_fmaf(-rel, srScale, swellGain));
        } else {
            swellGain = __builtin_fminf(1.0f, __builtin_fmaf(0.005f, srScale, swellGain));
        }
    }
};

#endif