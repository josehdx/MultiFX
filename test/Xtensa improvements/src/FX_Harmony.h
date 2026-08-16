#ifndef FX_HARMONY_H
#define FX_HARMONY_H

class FX_Harmony {
public:
    static inline float getSpd(float currentPitch, float harmRatio) __attribute__((always_inline)) {
        return currentPitch * harmRatio;
    }
};

#endif