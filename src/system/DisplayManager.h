#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#if defined(TARGET_LILYGO)
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <math.h>

extern char lastMidiDebug[64];
extern char bleEventDebug[64];
extern char bleScanDebug[64];
extern char bleLatchedDiscReason[64];

#define SHOW_TELEMETRY true
#define DISP_COLOR_OFF  0x001F 
#define DISP_COLOR_ON   TFT_GREEN

struct DisplayData {
    float batVoltage;
    int batPercent;
    bool isCharging;
    
    bool bleConnected;
    bool bleHostConnected;
    bool blePedalConnected;
    bool isNuxConnected;
    
    int activeMode;
    bool fxStates[10];
    uint16_t pb1, pb2, pb3, cc11;
    float paramVals[5];
    const char* paramNames[5];
    float inMeter, outMeter;
    float dspCoreLoad, ctrlCoreLoad;
    uint32_t freeSRAM, freePSRAM;
    uint32_t sampleRate;
    uint32_t peakLatency;
    uint32_t underflows, dmaCount, stackWatermark;
    
    int latencyMode;
    
    bool isKnobEditMode;
    bool showBleWarning;
    bool showSavingScreen;
    bool showKnobModeScreen;
    
    bool isMonoPolyActive;
    bool isVolumeMode;
    
    float whammyToe;
    float whammyHeel;
    
    char debugMsg[64];

    // Advanced Debug Tracking
    uint32_t peakRxCyc;
    uint32_t peakDspCyc;
    uint32_t peakTxCyc;
    uint32_t partialRx;
    uint32_t phaseTears;
    uint32_t rxErrors;
    uint32_t txErrors;
};

class DisplayManager {
private:
    TFT_eSPI tft = TFT_eSPI();
    TFT_eSprite spr = TFT_eSprite(&tft);
    TFT_eSprite bgSpr = TFT_eSprite(&tft);

    DisplayData cache = {};
    char cachedMidiDebug[64] = "";
    char cachedBleEvent[64] = "";
    char cachedBleScan[64] = "";
    char cachedDiscReason[64] = "";
    int lastRenderMode = -1;
    bool lastMonoPolyActive = false;
    bool lastVolumeMode = false;
    int lastMonoPolyAlgo = -1;
    
    int bootLogY = 8;

    const char* FX_NAMES[10] = {"WH", "FZ", "FB", "HR", "CP", "SY", "PD", "CH", "SW", "VB"};
    const char* MODE_NAMES[10] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};
    
    const char* MONO_ALGOS[5] = {"PSOLA", "YIN SYNTH", "ECKF", "VAR DELAY", "VOCODER"};

    const char* MONO_LABELS[5][5] = {
        {"ALGO", "RNG",  "FRM",  "TOL",  "MIX"},
        {"ALGO", "WAV",  "GLD",  "OCT",  "MIX"},
        {"ALGO", "Q-M",  "R-M",  "SPD",  "MIX"},
        {"ALGO", "WIN",  "Z-C",  "TRN",  "MIX"},
        {"ALGO", "FFT",  "PHS",  "TRN",  "MIX"}
    };

    const char* POLY_LABELS[10][5] = {
        {"DRY",  "WET",  "TOE",  "HEEL", "GLD"}, 
        {"APF",  "ATTK", "REL",  "FADE", "MIX"},
        {"SPD",  "DRV",  "OFFS", "INTV", "MIX"},
        {"MIX",  "VOIC", "DETN", "DRY",  "WET"},
        {"KEY",  "FINE", "DRY",  "WET",  "MIX"},
        {"ATTK", "REL",  "FLTR", "MIX",  "RESO"},
        {"SMTH", "MIX",  "DIFF", "DAMP", "TONE"},
        {"RATE", "MIX",  "DPTH", "WDTH", "TONE"},
        {"THRS", "ATTK", "REL",  "SENS", "MIX"},
        {"DPTH", "RATE", "WAVE", "SENS", "MIX"}
    };

    void drawKnobStatic(TFT_eSprite& s, int cx, int cy, int r, const char* label) {
        s.drawCircle(cx, cy, r, TFT_DARKGREY);
        if (label) {
            s.setTextDatum(TC_DATUM);
            s.setTextColor(TFT_WHITE, TFT_BLACK);
            s.drawString(label, cx, cy - r - 9, 1);
        }
    }

    void drawKnobDynamic(TFT_eSprite& s, int cx, int cy, int r, float normVal, bool isBipolar, const char* valStr) {
        float angleDeg = isBipolar ? (normVal * 135.0f) : (-135.0f + (normVal * 270.0f));
        float rad = (angleDeg - 90.0f) * 0.0174532925f;
        int px = cx + (int)(cosf(rad) * (r - 3));
        int py = cy + (int)(sinf(rad) * (r - 3));
        
        uint16_t color = isBipolar ? TFT_CYAN : TFT_GREEN;
        s.drawLine(cx, cy, px, py, color);
        s.fillCircle(px, py, 2, color);
        
        if (valStr) {
            s.setTextDatum(TC_DATUM);
            s.setTextColor(TFT_WHITE, TFT_BLACK);
            s.drawString(valStr, cx, cy + r + 2, 1);
        }
    }

    bool checkIsDirty(const DisplayData& d) {
        if (strcmp(lastMidiDebug, cachedMidiDebug) != 0 ||
            strcmp(bleEventDebug, cachedBleEvent) != 0 ||
            strcmp(bleScanDebug, cachedBleScan) != 0 ||
            strcmp(bleLatchedDiscReason, cachedDiscReason) != 0 ||
            d.bleHostConnected != cache.bleHostConnected ||
            d.blePedalConnected != cache.blePedalConnected ||
            d.isNuxConnected != cache.isNuxConnected) {
            
            strlcpy(cachedMidiDebug, lastMidiDebug, sizeof(cachedMidiDebug));
            strlcpy(cachedBleEvent, bleEventDebug, sizeof(cachedBleEvent));
            strlcpy(cachedBleScan, bleScanDebug, sizeof(cachedBleScan));
            strlcpy(cachedDiscReason, bleLatchedDiscReason, sizeof(cachedDiscReason));
            return true;
        }

        if (d.isKnobEditMode != cache.isKnobEditMode || 
            d.showBleWarning != cache.showBleWarning || 
            d.showSavingScreen != cache.showSavingScreen || 
            d.showKnobModeScreen != cache.showKnobModeScreen ||
            d.isMonoPolyActive != cache.isMonoPolyActive ||
            d.isVolumeMode != cache.isVolumeMode ||
            d.isCharging != cache.isCharging) return true;
            
        if (d.activeMode != cache.activeMode) return true;
        if (d.latencyMode != cache.latencyMode) return true;
        if (d.pb1 != cache.pb1 || d.pb2 != cache.pb2 || d.pb3 != cache.pb3 || d.cc11 != cache.cc11) return true;
        if (fabsf(d.inMeter - cache.inMeter) > 0.02f || fabsf(d.outMeter - cache.outMeter) > 0.02f) return true;
        if (fabsf(d.dspCoreLoad - cache.dspCoreLoad) > 1.0f || fabsf(d.ctrlCoreLoad - cache.ctrlCoreLoad) > 1.0f) return true;
        if (d.batPercent != cache.batPercent || d.sampleRate != cache.sampleRate) return true;
        if (d.peakLatency != cache.peakLatency || d.underflows != cache.underflows || d.dmaCount != cache.dmaCount || d.stackWatermark != cache.stackWatermark) return true;
        if ((d.freeSRAM / 1024) != (cache.freeSRAM / 1024) || (d.freePSRAM / 1024) != (cache.freePSRAM / 1024)) return true;
        
        if (d.whammyToe != cache.whammyToe || d.whammyHeel != cache.whammyHeel) return true;
        if (strcmp(d.debugMsg, cache.debugMsg) != 0) return true;

        if (d.peakDspCyc != cache.peakDspCyc || d.phaseTears != cache.phaseTears || 
            d.rxErrors != cache.rxErrors || d.txErrors != cache.txErrors || 
            d.partialRx != cache.partialRx) return true;

        for (int i = 0; i < 10; i++) {
            if (d.fxStates[i] != cache.fxStates[i]) return true;
        }
        for (int i = 0; i < 5; i++) {
            if (fabsf(d.paramVals[i] - cache.paramVals[i]) > 0.005f) return true;
        }

        return false;
    }

public:
    void begin() {
        pinMode(15, OUTPUT);
        digitalWrite(15, HIGH);
        
        pinMode(38, OUTPUT);
        digitalWrite(38, HIGH); 
        delay(100);                         
        
        tft.init();
        tft.setRotation(1);
        tft.fillScreen(TFT_BLACK);
        digitalWrite(38, HIGH); 

        spr.setColorDepth(16);
        bgSpr.setColorDepth(16);
        
        spr.createSprite(320, 170);
        bgSpr.createSprite(320, 170);
        
        spr.setTextColor(TFT_WHITE, TFT_BLACK);

        lastRenderMode = -1;
        lastMonoPolyActive = false;
        lastVolumeMode = false;
        lastMonoPolyAlgo = -1;
        
        bootLogY = 8;
    }

    void drawBootLogLine(const char* line, uint16_t color = 0x07E0) {
        tft.setTextColor(color, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(line, 6, bootLogY, 2);
        bootLogY += 19;
        if (bootLogY > 150) {
            bootLogY = 8;
            tft.fillScreen(TFT_BLACK);
        }
    }

    void drawBootProgress(int percent) {
        tft.drawRect(20, 130, 280, 14, TFT_WHITE);
        int fillW = constrain((percent * 276) / 100, 0, 276);
        tft.fillRect(22, 132, fillW, 10, TFT_GREEN);
        
        char pBuf[16];
        snprintf(pBuf, sizeof(pBuf), "%d%%", percent);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString(pBuf, 160, 118, 2);
    }

    void render(const DisplayData& d) {
        if (!checkIsDirty(d)) return;
        cache = d;
        strlcpy(cache.debugMsg, d.debugMsg, sizeof(cache.debugMsg));

        if (!spr.getPointer() || !bgSpr.getPointer()) return;

        int pbCenters[4] = {53, 124, 195, 266};
        int parCenters[5] = {46, 103, 160, 217, 274};

        int monoAlgo = 0;
        if (d.isMonoPolyActive) {
            monoAlgo = (int)(d.paramVals[0] * 4.99f);
            monoAlgo = constrain(monoAlgo, 0, 4);
        }

        if (d.isMonoPolyActive != lastMonoPolyActive ||
            (d.isMonoPolyActive && monoAlgo != lastMonoPolyAlgo) ||
            d.isVolumeMode != lastVolumeMode) {
            lastRenderMode = -1; 
        }

        if (d.activeMode != lastRenderMode) {
            bgSpr.fillSprite(TFT_BLACK);
            bgSpr.setTextColor(TFT_WHITE, TFT_BLACK);
            
            bgSpr.drawFastHLine(0, 18, 320, TFT_WHITE);
            
            bgSpr.drawRect(2, 24, 10, 106, TFT_WHITE);
            bgSpr.setTextDatum(TC_DATUM);
            bgSpr.drawString("IN", 7, 132, 1);

            bgSpr.drawRect(308, 24, 10, 106, TFT_WHITE);
            bgSpr.drawString("OUT", 313, 132, 1);
            
            drawKnobStatic(bgSpr, pbCenters[0], 48, 14, "PB1");
            drawKnobStatic(bgSpr, pbCenters[1], 48, 14, "PB2");
            
            const char* pb3Label = d.isVolumeMode ? "Vol" : "PB3";
            drawKnobStatic(bgSpr, pbCenters[2], 48, 14, pb3Label);
            
            const char* pb4Label = d.isVolumeMode ? "Vol" : "PB4-14B";
            drawKnobStatic(bgSpr, pbCenters[3], 48, 14, pb4Label);
            
            const char* currentNames[5];
            
            if (d.isMonoPolyActive && d.activeMode == 0) {
                for (int i = 0; i < 5; i++) currentNames[i] = MONO_LABELS[monoAlgo][i];
            } else {
                int actMode = d.activeMode % 10;
                for (int i = 0; i < 5; i++) currentNames[i] = POLY_LABELS[actMode][i];
            }

            for (int i = 0; i < 5; i++) {
                drawKnobStatic(bgSpr, parCenters[i], 110, 12, currentNames[i]);
            }
            
            lastRenderMode = d.activeMode;
            lastMonoPolyActive = d.isMonoPolyActive;
            lastMonoPolyAlgo = monoAlgo;
            lastVolumeMode = d.isVolumeMode;
        }

        spr.pushImage(0, 0, 320, 170, (uint16_t*)bgSpr.getPointer());

        spr.setTextDatum(TL_DATUM);
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        
        char batBuf[32];
        if (d.isCharging) {
            snprintf(batBuf, sizeof(batBuf), "%.2fV CHARGING", d.batVoltage);
        } else {
            snprintf(batBuf, sizeof(batBuf), "%.2fV %d%%", d.batVoltage, d.batPercent);
        }
        spr.drawString(batBuf, 5, 2, 2); 

        spr.setTextDatum(TC_DATUM);
        char headerBuf[64];
        
        if (d.isVolumeMode) {
            spr.setTextColor(TFT_ORANGE, TFT_BLACK);
            snprintf(headerBuf, sizeof(headerBuf), "*** VOLUME MODE ***");
        } else if (d.isMonoPolyActive && d.activeMode == 0) {
            spr.setTextColor(TFT_CYAN, TFT_BLACK);
            snprintf(headerBuf, sizeof(headerBuf), "Mono Whammy [ T:%+d | H:%+d ]", (int)d.whammyToe, (int)d.whammyHeel);
        } else {
            spr.setTextColor(TFT_WHITE, TFT_BLACK);
            snprintf(headerBuf, sizeof(headerBuf), "%s [ T:%+d | H:%+d ]", MODE_NAMES[d.activeMode % 10], (int)d.whammyToe, (int)d.whammyHeel);
        }
        spr.drawString(headerBuf, 160, 2, 2);
        
        spr.setTextDatum(TR_DATUM);
        if (d.isKnobEditMode) {
            spr.setTextColor(DISP_COLOR_OFF, TFT_BLACK);
            spr.drawString("KNOB MODE", 318, 2, 2);
        } else {
            // Line 1: M-vave Foot Controller
            spr.setTextColor(d.blePedalConnected ? DISP_COLOR_ON : DISP_COLOR_OFF, TFT_BLACK);
            spr.drawString("M-vave", 318, 1, 1);
            
            // Line 2: Independent PC and NUX indicators
            spr.setTextColor(d.bleHostConnected ? DISP_COLOR_ON : DISP_COLOR_OFF, TFT_BLACK);
            spr.drawString("PC", 294, 10, 1);
            
            spr.setTextColor(d.isNuxConnected ? DISP_COLOR_ON : DISP_COLOR_OFF, TFT_BLACK);
            spr.drawString("NUX", 318, 10, 1);
        }

        spr.setTextColor(TFT_WHITE, TFT_BLACK);

        int inH = constrain((int)(d.inMeter * 104.0f), 0, 104);
        int outH = constrain((int)(d.outMeter * 104.0f), 0, 104);
        
        spr.fillRect(3, 129 - inH, 8, inH, TFT_GREEN);
        spr.fillRect(309, 129 - outH, 8, outH, TFT_GREEN);

        float pb1_norm = ((float)d.pb1 - 8192.0f) / 8191.5f;
        float pb2_norm = ((float)d.pb2 - 8192.0f) / 8191.5f;
        float pb3_norm = (float)d.pb3 / 16383.0f;
        float cc11_norm = (float)d.cc11 / 16383.0f;

        char buf[16];

        snprintf(buf, sizeof(buf), "%d%%", (int)(pb1_norm * 100.0f));
        drawKnobDynamic(spr, pbCenters[0], 48, 14, pb1_norm, true, buf);

        snprintf(buf, sizeof(buf), "%d%%", (int)(pb2_norm * 100.0f));
        drawKnobDynamic(spr, pbCenters[1], 48, 14, pb2_norm, true, buf);

        snprintf(buf, sizeof(buf), "%d%%", (int)(pb3_norm * 100.0f));
        drawKnobDynamic(spr, pbCenters[2], 48, 14, pb3_norm, false, buf);
        
        snprintf(buf, sizeof(buf), "%d%%", (int)(cc11_norm * 100.0f));
        drawKnobDynamic(spr, pbCenters[3], 48, 14, cc11_norm, false, buf);

        for (int i = 0; i < 5; i++) {
            bool isBip = false;
            float pNorm = d.paramVals[i];
            
            if (d.isMonoPolyActive && d.activeMode == 0 && i == 0) {
                snprintf(buf, sizeof(buf), "%s", MONO_ALGOS[monoAlgo]);
                pNorm = monoAlgo / 4.0f; 
                isBip = false;
            } 
            else if (d.activeMode == 0) {
                if (i == 2 || i == 3) {
                    snprintf(buf, sizeof(buf), "%+d", (int)d.paramVals[i]);
                    pNorm = d.paramVals[i] / 24.0f;
                    isBip = true;
                } else {
                    snprintf(buf, sizeof(buf), "%.2f", d.paramVals[i]);
                    isBip = false;
                }
            } 
            else {
                snprintf(buf, sizeof(buf), "%.2f", d.paramVals[i]);
                isBip = (i == 0 && d.paramVals[i] < 0.0f) || (i == 0);
                pNorm = isBip ? (d.paramVals[i] / 24.0f) : d.paramVals[i];
            }

            drawKnobDynamic(spr, parCenters[i], 110, 12, constrain(pNorm, isBip ? -1.0f : 0.0f, 1.0f), isBip, buf);
        }

        const int CHAIN_ORDER[10] = {8, 1, 4, 0, 5, 3, 7, 9, 6, 2};
        
        int activeIndices[10];
        int activeCount = 0;
        for (int i = 0; i < 10; i++) {
            int fxIdx = CHAIN_ORDER[i];
            bool isEffectActive = (fxIdx == 0) ? (d.fxStates[0] || d.isMonoPolyActive) : d.fxStates[fxIdx];
            if (isEffectActive) activeIndices[activeCount++] = fxIdx;
        }

        if (activeCount > 0) {
            int badgeW = 26; 
            int totalW = (activeCount * badgeW) + ((activeCount - 1) * 2);
            int startX = (320 - totalW) / 2;

            for (int k = 0; k < activeCount; k++) {
                int fxIdx = activeIndices[k];
                int bx = startX + (k * (badgeW + 2));
                uint16_t bColor = (fxIdx == 0 && d.isMonoPolyActive) ? TFT_DARKGREY : TFT_BLUE;
                
                const char* bName = FX_NAMES[fxIdx];
                if (fxIdx == 0 && d.isMonoPolyActive) bName = "MW";

                spr.fillRect(bx, 75, badgeW, 14, bColor);
                spr.setTextColor(TFT_WHITE, bColor);
                spr.setTextDatum(TC_DATUM);
                spr.drawString(bName, bx + (badgeW / 2), 79, 1);
            }
        }

        if (SHOW_TELEMETRY) {
            spr.fillRect(16, 130, 288, 38, TFT_BLACK); 
            spr.drawRect(16, 130, 288, 38, TFT_YELLOW);
            
            spr.setTextDatum(TL_DATUM);
            spr.setTextColor(TFT_YELLOW, TFT_BLACK);
            
            char line1[128];
            snprintf(line1, sizeof(line1), "CPU0:%d%% CPU1:%d%% | SR:%luHz | LAT:%ums", 
                     (int)d.dspCoreLoad, (int)d.ctrlCoreLoad, (unsigned long)d.sampleRate,
                     (unsigned int)d.peakLatency);
                     
            char line2[128];
            snprintf(line2, sizeof(line2), "RAM:%uK PS:%uK UDF:%u | %s", 
                     (unsigned int)(d.freeSRAM / 1024), (unsigned int)(d.freePSRAM / 1024), 
                     (unsigned int)d.underflows, d.debugMsg);

            char line3[128];
            snprintf(line3, sizeof(line3), "PkDSP:%lu | Err:%lu | Tear:%lu | pRX:%lu",
                     (unsigned long)d.peakDspCyc, 
                     (unsigned long)(d.rxErrors + d.txErrors), 
                     (unsigned long)d.phaseTears, 
                     (unsigned long)d.partialRx);

            spr.drawString(line1, 20, 132, 1);
            spr.drawString(line2, 20, 144, 1);
            
            // Turn text RED if there is a DSP deadline violation or Phase Sync failure
            if (d.peakDspCyc > 160000 || d.rxErrors > 0 || d.txErrors > 0 || d.phaseTears > 0 || d.partialRx > 0) {
                spr.setTextColor(TFT_RED, TFT_BLACK);
            }
            spr.drawString(line3, 20, 156, 1);
            spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        }

        if (d.showSavingScreen) {
            spr.fillRect(60, 45, 200, 60, TFT_BLUE);
            spr.drawRect(60, 45, 200, 60, TFT_WHITE);
            spr.setTextColor(TFT_WHITE, TFT_BLUE);
            spr.setTextDatum(MC_DATUM);
            spr.drawString("ENABLING BLE", 160, 65, 2);
            spr.drawString("Rebooting...", 160, 85, 2);
            
        } else if (d.showKnobModeScreen) {
            spr.fillRect(60, 45, 200, 60, TFT_DARKGREEN);
            spr.drawRect(60, 45, 200, 60, TFT_WHITE);
            spr.setTextColor(TFT_WHITE, TFT_DARKGREEN);
            spr.setTextDatum(MC_DATUM);
            spr.drawString("KNOB MODE", 160, 65, 2);
            spr.drawString("Rebooting...", 160, 85, 2);
            
        } else if (d.showBleWarning) {
            spr.fillRect(60, 45, 200, 60, DISP_COLOR_OFF);
            spr.drawRect(60, 45, 200, 60, TFT_WHITE);
            spr.setTextColor(TFT_WHITE, DISP_COLOR_OFF);
            spr.setTextDatum(MC_DATUM);
            spr.drawString("HOLD TO ENABLE BLE", 160, 65, 2);
            spr.drawString("Reboot Required", 160, 85, 2);
        }

        spr.pushSprite(0, 0);
    }
};

#endif
#endif