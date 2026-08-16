#ifndef FX_CAPO_H
#define FX_CAPO_H

class FX_Capo {
public:
    // Capo operates purely in the LUT mapping phase as a permanent base pitch offset
    static inline float getBasePitch(bool active, float capoParam) __attribute__((always_inline)) {
        return active ? capoParam : 0.0f;
    }
};

#endif