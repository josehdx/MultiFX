#ifndef FX_PAD_H
#define FX_PAD_H

#include "DSPEngine.h"

class FX_Pad {
public:
    static inline void processEnv(bool active, float env, float srScale, float& padEnv, float& inSample) __attribute__((always_inline)) {
        if(active) {
            padEnv = (env > 0.005f) ? __builtin_fminf(1.0f, __builtin_fmaf(0.00002f, srScale, padEnv)) : __builtin_fmaxf(0.0f, __builtin_fmaf(-0.000005f, srScale, padEnv));
            inSample *= padEnv;
        }
    }
    
    static inline void processDiffuser(bool chorusActive, bool padActive, float padOut, float diffInBase, float* diffuserBuf, int& diffuserIdx, float& w3Out, float& padFilterOut) __attribute__((always_inline)) {
        float diffIn = diffInBase + padOut;
        float diffOut = diffuserBuf ? diffuserBuf[diffuserIdx] : 0.0f;
        float diffNext = DSPEngine::AntiDenormal(diffIn + 0.6f * diffOut);
        if(diffuserBuf) diffuserBuf[diffuserIdx] = diffNext;
        float diffFinal = DSPEngine::AntiDenormal(diffOut - 0.6f * diffNext);
        diffuserIdx = (diffuserIdx + 1) & 1023;
        
        if(chorusActive && padActive) {
            w3Out = diffFinal * 0.5f;
            padFilterOut = diffFinal * 0.5f;
        } else if(chorusActive) {
            w3Out = diffFinal;
            padFilterOut = padOut;
        } else if(padActive) {
            w3Out = 0.0f;
            padFilterOut = diffFinal;
        } else {
            padFilterOut = padOut;
        }
    }
};

#endif