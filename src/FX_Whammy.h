#ifndef FX_WHAMMY_H
#define FX_WHAMMY_H

class FX_Whammy {
public:
    // Core whammy acts as the fundamental multiplier passthrough for spd1
    static inline float getSpd(float targetPitch) __attribute__((always_inline)) {
        return targetPitch;
    }
};

#endif