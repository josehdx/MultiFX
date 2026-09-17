#ifndef DISPLAY_MANAGER_H  
#define DISPLAY_MANAGER_H  

#if defined(TARGET_LILYGO)  
#include <TFT_eSPI.h>  
#include <Arduino.h>  
#include <math.h>  

struct DisplayData {          
    float batVoltage;          
    int batPercent;          
    bool bleConnected;          
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
    
    // UI Warning State Flags          
    bool isKnobEditMode;          
    bool showBleWarning;          
    bool showSavingScreen;          
    bool showKnobModeScreen;      
    bool isMonoPolyActive;          
    
    // Pitch bounds tracking for UI background header
    float whammyToe;     
    float whammyHeel; 
};

class DisplayManager {  
private:          
    TFT_eSPI tft = TFT_eSPI();          
    TFT_eSprite spr = TFT_eSprite(&tft);          
    TFT_eSprite bgSpr = TFT_eSprite(&tft);      
    DisplayData cache = {};          
    int lastRenderMode = -1;          
    bool lastMonoPolyActive = false;     
    int lastMonoPolyAlgo = -1;     
    
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

    // Dedicated parameter labels for standard polyphonic effects
    const char* POLY_LABELS[10][5] = {
        {"DRY",  "WET",  "TOE",  "HEEL", "MODE"}, // 0: WHAMMY
        {"APF",  "ATTK", "REL",  "FADE", "MIX"},  // 1: FREEZE
        {"SPD",  "DRV",  "OFFS", "INTV", "MIX"},  // 2: FEEDBACK
        {"MIX",  "VOIC", "DETN", "DRY",  "WET"},  // 3: HARMONY
        {"KEY",  "FINE", "DRY",  "WET",  "MIX"},  // 4: CAPO
        {"ATTK", "REL",  "FLTR", "MIX",  "RESO"}, // 5: SYNTH
        {"SMTH", "MIX",  "DIFF", "DAMP", "TONE"}, // 6: PAD
        {"RATE", "MIX",  "DPTH", "WDTH", "TONE"}, // 7: CHORUS
        {"THRS", "ATTK", "REL",  "SENS", "MIX"},  // 8: SWELL
        {"DPTH", "RATE", "WAVE", "SENS", "MIX"}   // 9: VIBRATO
    };

    void drawKnobStatic(TFT_eSprite& s, int cx, int cy, int r, const char* label) {                  
        s.drawCircle(cx, cy, r, TFT_DARKGREY);                  
        if (label) {                          
            s.setTextDatum(TC_DATUM);                          
            s.setTextColor(TFT_WHITE, TFT_BLACK);                          
            s.drawString(label, cx, cy - r - 10, 1);                  
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
        if (d.isKnobEditMode != cache.isKnobEditMode ||                           
            d.showBleWarning != cache.showBleWarning ||                           
            d.showSavingScreen != cache.showSavingScreen ||                          
            d.showKnobModeScreen != cache.showKnobModeScreen ||             
            d.isMonoPolyActive != cache.isMonoPolyActive) return true;                                        
            
        if (d.activeMode != cache.activeMode) return true;                  
        if (d.pb1 != cache.pb1 || d.pb2 != cache.pb2 || d.pb3 != cache.pb3 || d.cc11 != cache.cc11) return true;                  
        if (fabsf(d.inMeter - cache.inMeter) > 0.02f || fabsf(d.outMeter - cache.outMeter) > 0.02f) return true;                  
        if (fabsf(d.dspCoreLoad - cache.dspCoreLoad) > 1.0f || fabsf(d.ctrlCoreLoad - cache.ctrlCoreLoad) > 1.0f) return true;                  
        if (d.batPercent != cache.batPercent || d.bleConnected != cache.bleConnected || d.sampleRate != cache.sampleRate) return true;                  
        if (d.peakLatency != cache.peakLatency || d.underflows != cache.underflows || d.dmaCount != cache.dmaCount || d.stackWatermark != cache.stackWatermark) return true;                  
        if ((d.freeSRAM / 1024) != (cache.freeSRAM / 1024) || (d.freePSRAM / 1024) != (cache.freePSRAM / 1024)) return true;                           
        
        if (d.whammyToe != cache.whammyToe || d.whammyHeel != cache.whammyHeel) return true;                  
        
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
        tft.init();                  
        tft.setRotation(1);                  
        tft.fillScreen(TFT_BLACK);                           
        
        spr.setColorDepth(16);                  
        bgSpr.setColorDepth(16);                           
        
        spr.createSprite(320, 240);                  
        bgSpr.createSprite(320, 240);                           
        
        spr.setTextColor(TFT_WHITE, TFT_BLACK);                  
        lastRenderMode = -1;              
        lastMonoPolyActive = false;         
        lastMonoPolyAlgo = -1;     
    }          

    void render(const DisplayData& d) {                  
        if (!checkIsDirty(d)) return;                  
        cache = d;                  
        if (!spr.getPointer() || !bgSpr.getPointer()) return;                  
        
        int pbCenters[4] = {53, 124, 195, 266};                  
        int parCenters[5] = {46, 103, 160, 217, 274};                  
        
        int monoAlgo = 0;         
        if (d.isMonoPolyActive) {             
            monoAlgo = (int)(d.paramVals[0] * 4.99f);             
            monoAlgo = constrain(monoAlgo, 0, 4);         
        }         

        if (d.isMonoPolyActive != lastMonoPolyActive || (d.isMonoPolyActive && monoAlgo != lastMonoPolyAlgo)) {             
            lastRenderMode = -1;          
        }         

        if (d.activeMode != lastRenderMode) {                          
            bgSpr.fillSprite(TFT_BLACK);                          
            bgSpr.setTextColor(TFT_WHITE, TFT_BLACK);                                       
            
            bgSpr.drawFastHLine(0, 18, 320, TFT_WHITE);                          
            bgSpr.drawFastHLine(15, 74, 290, TFT_DARKGREY);                          
            bgSpr.drawFastHLine(0, 130, 320, TFT_DARKGREY);                          
            bgSpr.drawFastHLine(0, 154, 320, TFT_WHITE);                                       
            
            bgSpr.drawRect(2, 24, 10, 92, TFT_WHITE);                          
            bgSpr.setTextDatum(TC_DATUM);                          
            bgSpr.drawString("IN", 7, 118, 1);                          
            bgSpr.drawRect(308, 24, 10, 92, TFT_WHITE);                          
            bgSpr.drawString("OUT", 313, 118, 1);                                       
            
            drawKnobStatic(bgSpr, pbCenters[0], 42, 13, "PB1");                          
            drawKnobStatic(bgSpr, pbCenters[1], 42, 13, "PB2");                          
            drawKnobStatic(bgSpr, pbCenters[2], 42, 13, "PB3");                          
            drawKnobStatic(bgSpr, pbCenters[3], 42, 13, "CC11");                                       
            
            const char* currentNames[5];             
            if (d.isMonoPolyActive) {                 
                for (int i = 0; i < 5; i++) currentNames[i] = MONO_LABELS[monoAlgo][i];             
            } else {                 
                int actMode = d.activeMode % 10;
                for (int i = 0; i < 5; i++) currentNames[i] = POLY_LABELS[actMode][i];             
            }             

            for (int i = 0; i < 5; i++) {                              
                drawKnobStatic(bgSpr, parCenters[i], 102, 10, currentNames[i]);                          
            }                                       
            
            lastRenderMode = d.activeMode;                      
            lastMonoPolyActive = d.isMonoPolyActive;             
            lastMonoPolyAlgo = monoAlgo;         
        }                           

        spr.pushImage(0, 0, 320, 240, (uint16_t*)bgSpr.getPointer());                           

        // --- 1. TOP STATUS BAR ---                  
        spr.setTextDatum(TL_DATUM);                  
        spr.printf("%.2fV %d%%", d.batVoltage, d.batPercent);                  
        spr.setTextDatum(TC_DATUM);                  
        
        char headerBuf[64];
        if (d.isMonoPolyActive) {             
            spr.setTextColor(TFT_CYAN, TFT_BLACK);             
            snprintf(headerBuf, sizeof(headerBuf), "%s [ T:%+d | H:%+d ]", MONO_ALGOS[monoAlgo], (int)d.whammyToe, (int)d.whammyHeel);             
        } else {             
            spr.setTextColor(TFT_WHITE, TFT_BLACK);
            snprintf(headerBuf, sizeof(headerBuf), "%s [ T:%+d | H:%+d ]", MODE_NAMES[d.activeMode % 10], (int)d.whammyToe, (int)d.whammyHeel);
        }         
        spr.drawString(headerBuf, 160, 2, 2);             
        spr.setTextColor(TFT_WHITE, TFT_BLACK);

        spr.setTextDatum(TR_DATUM);                  
        if (d.isKnobEditMode) {                          
            spr.setTextColor(TFT_RED, TFT_BLACK);                          
            spr.drawString("EDIT MODE", 318, 2);                          
            spr.setTextColor(TFT_WHITE, TFT_BLACK);                  
        } else {                          
            spr.drawString(d.bleConnected ? "BT: CONN" : "BT: WAIT", 318, 2);                  
        }                           

        // --- 2. VERTICAL VU METERS (IN Left | OUT Right) ---                  
        int inH = constrain((int)(d.inMeter * 90.0f), 0, 90);                  
        int outH = constrain((int)(d.outMeter * 90.0f), 0, 90);                           
        
        spr.fillRect(3, 115 - inH, 8, inH, TFT_GREEN);                  
        spr.fillRect(309, 115 - outH, 8, outH, TFT_GREEN);                           

        // --- 3. TOP ROW: 4 CIRCULAR PBs ---                  
        float pb1_norm = ((float)d.pb1 - 8192.0f) / 8191.5f;                  
        float pb2_norm = ((float)d.pb2 - 8192.0f) / 8191.5f;                  
        float pb3_norm = (float)d.pb3 / 16383.0f;                  
        float cc11_norm = (float)d.cc11 / 16383.0f;                  
        char buf[16];                  
        
        snprintf(buf, sizeof(buf), "%d%%", (int)(pb1_norm * 100.0f));                  
        drawKnobDynamic(spr, pbCenters[0], 42, 13, pb1_norm, true, buf);                  
        snprintf(buf, sizeof(buf), "%d%%", (int)(pb2_norm * 100.0f));                  
        drawKnobDynamic(spr, pbCenters[1], 42, 13, pb2_norm, true, buf);                  
        snprintf(buf, sizeof(buf), "%d%%", (int)(pb3_norm * 100.0f));                  
        drawKnobDynamic(spr, pbCenters[2], 42, 13, pb3_norm, false, buf);                  
        snprintf(buf, sizeof(buf), "%d%%", (int)(cc11_norm * 100.0f));                  
        drawKnobDynamic(spr, pbCenters[3], 42, 13, cc11_norm, false, buf);                           

        // --- 4. MIDDLE ROW: 5 CIRCULAR PARAMS ---                  
        for (int i = 0; i < 5; i++) {                          
            snprintf(buf, sizeof(buf), "%.2f", d.paramVals[i]);                          
            bool isBip = (i == 0 && d.paramVals[i] < 0.0f) || (i == 0);                          
            float pNorm = isBip ? (d.paramVals[i] / 24.0f) : d.paramVals[i];                                       
            
            if (d.isMonoPolyActive && i == 0) {                 
                snprintf(buf, sizeof(buf), "%s", MONO_ALGOS[monoAlgo]);                 
                pNorm = monoAlgo / 4.0f;                  
                isBip = false;             
            }             
            drawKnobDynamic(spr, parCenters[i], 102, 10, constrain(pNorm, isBip ? -1.0f : 0.0f, 1.0f), isBip, buf);                  
        }                           

        // --- 5. DYNAMIC FX BADGES ---                  
        int activeIndices[10];                  
        int activeCount = 0;                  
        for (int i = 0; i < 10; i++) {                          
            if (d.fxStates[i]) activeIndices[activeCount++] = i;                  
        }                  
        if (activeCount > 0) {                          
            int badgeW = 28;                          
            int totalW = (activeCount * badgeW) + ((activeCount - 1) * 4);                          
            int startX = (320 - totalW) / 2;                          
            for (int k = 0; k < activeCount; k++) {                                  
                int fxIdx = activeIndices[k];                                  
                int bx = startX + (k * (badgeW + 4));                                                                    
                
                uint16_t bColor = (fxIdx == 0 && d.isMonoPolyActive) ? TFT_DARKGREY : TFT_BLUE;                                  
                spr.fillRect(bx, 134, badgeW, 16, bColor);                                  
                spr.setTextColor(TFT_WHITE, bColor);                                  
                spr.setTextDatum(TC_DATUM);                                  
                spr.drawString(FX_NAMES[fxIdx], bx + (badgeW / 2), 138, 1);                          
            }                  
        }                  
        spr.setTextColor(TFT_WHITE, TFT_BLACK);                            

        // --- 6. SYSTEM DIAGNOSTICS BAR ---                  
        spr.setTextDatum(TL_DATUM);                  
        spr.printf("CPU: %d%%  SRM:%dK  PSR:%dK  SR:%dkHz  PLL: %dms\n",                            
                    (int)d.dspCoreLoad, d.freeSRAM, d.freePSRAM, d.sampleRate / 1000, d.peakLatency);                  
        spr.printf("UDF: %d      DMA: %d      STK: %dB\n",                            
                    d.underflows, d.dmaCount, d.stackWatermark);                           

        // --- 7. WARNING OVERLAYS ---                  
        if (d.showSavingScreen) {                          
            spr.fillRect(60, 70, 200, 60, TFT_BLUE);                          
            spr.drawRect(60, 70, 200, 60, TFT_WHITE);                                       
            spr.setTextColor(TFT_WHITE, TFT_BLUE);                          
            spr.setTextDatum(MC_DATUM);                                       
            spr.drawString("SAVING...", 160, 92, 2);                          
            spr.drawString("Please Wait", 160, 114, 2);                                       
            spr.setTextColor(TFT_WHITE, TFT_BLACK);                  
        } else if (d.showKnobModeScreen) {                          
            spr.fillRect(60, 70, 200, 60, TFT_DARKGREEN);                          
            spr.drawRect(60, 70, 200, 60, TFT_WHITE);                                       
            spr.setTextColor(TFT_WHITE, TFT_DARKGREEN);                          
            spr.setTextDatum(MC_DATUM);                                       
            spr.drawString("KNOB MODE", 160, 92, 2);                          
            spr.drawString("Rebooting...", 160, 114, 2);                                       
            spr.setTextColor(TFT_WHITE, TFT_BLACK);                  
        } else if (d.showBleWarning) {                          
            spr.fillRect(60, 70, 200, 60, TFT_RED);                          
            spr.drawRect(60, 70, 200, 60, TFT_WHITE);                                       
            spr.setTextColor(TFT_WHITE, TFT_RED);                          
            spr.setTextDatum(MC_DATUM);                                       
            spr.drawString("REBOOT REQUIRED", 160, 92, 2);                          
            spr.drawString("To Enable BLE", 160, 114, 2);                                       
            spr.setTextColor(TFT_WHITE, TFT_BLACK);                  
        }                  
        spr.pushSprite(0, 0);          
    }  
}; // Properly closes class DisplayManager

#endif  
#endif