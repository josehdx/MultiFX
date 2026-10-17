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
    
public:
    void init(Preferences& prefs) {
        prefs.begin("whammy_cfg", false);
        
        // CRITICAL FIX: Pre-populate tracking cache with current NVS values to completely
        // eliminate the dangerous firstSave boolean which burnt flash cycles.
        lastSavedMode = prefs.getInt("activeMode", 0);
        lastSavedLatMode = prefs.getInt("latMode", 0);
        lastSavedFbIdx = prefs.getInt("fbIdx", 0);
        lastSavedPb2Wiper = prefs.getBool("pb2Wiper", false);
        lastSavedVolMode = prefs.getBool("volMode", false);
        lastSavedFxStates = prefs.getUShort("fxStates", 0xFFFF);
        lastSavedSampleRate = prefs.getUInt("sampleRate", 96000);
        
        size_t len = prefs.getBytes("dspData", lastSavedDspData, sizeof(lastSavedDspData));
        if (len == 0) memset(lastSavedDspData, 0, sizeof(lastSavedDspData));
    }

    void save(Preferences& prefs, int mode, int lat, int fbIdx, bool pb2Wiper, bool volMode, uint16_t fxStates, uint32_t sr, void* dspData, size_t dataSize) {
        if (mode != lastSavedMode) { prefs.putInt("activeMode", mode); lastSavedMode = mode; }
        if (lat != lastSavedLatMode) { prefs.putInt("latMode", lat); lastSavedLatMode = lat; }
        if (pb2Wiper != lastSavedPb2Wiper) { prefs.putBool("pb2Wiper", pb2Wiper); lastSavedPb2Wiper = pb2Wiper; }
        if (volMode != lastSavedVolMode) { prefs.putBool("volMode", volMode); lastSavedVolMode = volMode; }
        if (fxStates != lastSavedFxStates) { prefs.putUShort("fxStates", fxStates); lastSavedFxStates = fxStates; }
        if (sr != lastSavedSampleRate) { prefs.putUInt("sampleRate", sr); lastSavedSampleRate = sr; }
        if (fbIdx != lastSavedFbIdx) { prefs.putInt("fbIdx", fbIdx); lastSavedFbIdx = fbIdx; }
        
        bool dspDataChanged = false;
        if (memcmp(dspData, lastSavedDspData, dataSize) != 0) dspDataChanged = true;
        
        if (dspDataChanged) {
            prefs.putBytes("dspData", dspData, dataSize);
            memcpy(lastSavedDspData, dspData, dataSize);
        }
    }
};
#endif