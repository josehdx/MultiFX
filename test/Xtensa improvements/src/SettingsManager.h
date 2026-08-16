#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <Preferences.h>
#include <string.h>

class SettingsManager {
private:
    int lastSavedMode = -1;
    int lastSavedLatMode = -1;
    int lastSavedFbIdx = -1;
    bool lastSavedPb2Wiper = false;
    bool lastSavedVolMode = false;
    uint16_t lastSavedFxStates = 0xFFFF;
    uint32_t lastSavedSampleRate = 0;
    uint8_t lastSavedDspData[256]; 
    bool firstSave = true;

public:
    void init(Preferences& prefs) {
        prefs.begin("whammy_cfg", false);
    }

    void save(Preferences& prefs, int mode, int lat, int fbIdx, bool pb2Wiper, bool volMode, uint16_t fxStates, uint32_t sr, void* dspData, size_t dataSize) {
        if (firstSave || mode != lastSavedMode) { prefs.putInt("activeMode", mode); lastSavedMode = mode; }
        if (firstSave || lat != lastSavedLatMode) { prefs.putInt("latMode", lat); lastSavedLatMode = lat; }
        if (firstSave || pb2Wiper != lastSavedPb2Wiper) { prefs.putBool("pb2Wiper", pb2Wiper); lastSavedPb2Wiper = pb2Wiper; }
        if (firstSave || volMode != lastSavedVolMode) { prefs.putBool("volMode", volMode); lastSavedVolMode = volMode; }
        if (firstSave || fxStates != lastSavedFxStates) { prefs.putUShort("fxStates", fxStates); lastSavedFxStates = fxStates; }
        if (firstSave || sr != lastSavedSampleRate) { prefs.putUInt("sampleRate", sr); lastSavedSampleRate = sr; }
        if (firstSave || fbIdx != lastSavedFbIdx) { prefs.putInt("fbIdx", fbIdx); lastSavedFbIdx = fbIdx; }
        
        bool dspDataChanged = firstSave;
        if (!dspDataChanged) {
            if (memcmp(dspData, lastSavedDspData, dataSize) != 0) dspDataChanged = true;
        }
        if (dspDataChanged) {
            prefs.putBytes("dspData", dspData, dataSize);
            memcpy(lastSavedDspData, dspData, dataSize);
        }
        firstSave = false;
    }
};

#endif