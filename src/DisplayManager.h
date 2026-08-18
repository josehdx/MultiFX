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
};

class DisplayManager {
private:
    TFT_eSPI tft = TFT_eSPI();
    TFT_eSprite spr = TFT_eSprite(&tft);
    TFT_eSprite bgSpr = TFT_eSprite(&tft); // Added Background Sprite
    DisplayData cache = {};
    int lastRenderMode = -1;

    const char* FX_NAMES[10] = {"WH", "FZ", "FB", "HR", "CP", "SY", "PD", "CH", "SW", "VB"};
    const char* MODE_NAMES[10] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};

    // Draws unchanging boundaries, backgrounds, and fixed labels to the background sprite
    void drawKnobStatic(TFT_eSprite& s, int cx, int cy, int r, const char* label) {
        s.drawCircle(cx, cy, r, TFT_DARKGREY);
        if (label) {
            s.setTextDatum(TC_DATUM);
            s.setTextColor(TFT_WHITE, TFT_BLACK);
            s.drawString(label, cx, cy - r - 10, 1);
        }
    }

    // Calculates trig only for the moving needles and draws text to the active sprite
    void drawKnobDynamic(TFT_eSprite& s, int cx, int cy, int r, float normVal, bool isBipolar, const char* valStr) {
        float angleDeg = isBipolar ? (normVal * 135.0f) : (-135.0f + (normVal * 270.0f));
        float rad = (angleDeg - 90.0f) * 0.0174532925f; // 0  = UP
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

    // Prevents wasting 100% of render cost if no visual values have meaningfully changed
    bool checkIsDirty(const DisplayData& d) {
        // Trigger screen update instantly if warnings toggle
        if (d.isKnobEditMode != cache.isKnobEditMode || 
            d.showBleWarning != cache.showBleWarning || 
            d.showSavingScreen != cache.showSavingScreen ||
            d.showKnobModeScreen != cache.showKnobModeScreen) return true;
        
        if (d.activeMode != cache.activeMode) return true;
        if (d.pb1 != cache.pb1 || d.pb2 != cache.pb2 || d.pb3 != cache.pb3 || d.cc11 != cache.cc11) return true;
        if (fabsf(d.inMeter - cache.inMeter) > 0.02f || fabsf(d.outMeter - cache.outMeter) > 0.02f) return true;
        if (fabsf(d.dspCoreLoad - cache.dspCoreLoad) > 1.0f || fabsf(d.ctrlCoreLoad - cache.ctrlCoreLoad) > 1.0f) return true;
        if (d.batPercent != cache.batPercent || d.bleConnected != cache.bleConnected || d.sampleRate != cache.sampleRate) return true;
        if (d.peakLatency != cache.peakLatency || d.underflows != cache.underflows || d.dmaCount != cache.dmaCount || d.stackWatermark != cache.stackWatermark) return true;
        if ((d.freeSRAM / 1024) != (cache.freeSRAM / 1024) || (d.freePSRAM / 1024) != (cache.freePSRAM / 1024)) return true;
        
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
        
        // Lock to 16-bit color to halve memory overhead
        spr.setColorDepth(16);
        bgSpr.setColorDepth(16);
        
        spr.createSprite(320, 240);
        bgSpr.createSprite(320, 240);
        
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        lastRenderMode = -1;
    }

    void render(const DisplayData& d) {
        // [1] Frame-Skip Optimization
        if (!checkIsDirty(d)) return;
        cache = d;

        if (!spr.getPointer() || !bgSpr.getPointer()) return;

        int pbCenters[4] = {53, 124, 195, 266};
        int parCenters[5] = {46, 103, 160, 217, 274};

        // [2] Static Background Caching Strategy (Only updates when mode changes)
        if (d.activeMode != lastRenderMode) {
            bgSpr.fillSprite(TFT_BLACK);
            bgSpr.setTextColor(TFT_WHITE, TFT_BLACK);
            
            // Draw all horizontal static lines
            bgSpr.drawFastHLine(0, 18, 320, TFT_WHITE);
            bgSpr.drawFastHLine(15, 74, 290, TFT_DARKGREY);
            bgSpr.drawFastHLine(0, 130, 320, TFT_DARKGREY);
            bgSpr.drawFastHLine(0, 154, 320, TFT_WHITE);

            // Draw VU Meter Borders
            bgSpr.drawRect(2, 24, 10, 92, TFT_WHITE);
            bgSpr.setTextDatum(TC_DATUM);
            bgSpr.drawString("IN", 7, 118, 1);

            bgSpr.drawRect(308, 24, 10, 92, TFT_WHITE);
            bgSpr.drawString("OUT", 313, 118, 1);

            // Draw fixed Expression Pedal Knob Outlines
            drawKnobStatic(bgSpr, pbCenters[0], 42, 13, "PB1");
            drawKnobStatic(bgSpr, pbCenters[1], 42, 13, "PB2");
            drawKnobStatic(bgSpr, pbCenters[2], 42, 13, "PB3");
            drawKnobStatic(bgSpr, pbCenters[3], 42, 13, "CC11");

            // Draw dynamic Mode-based Parameter Knob Outlines
            for (int i = 0; i < 5; i++) {
                const char* pName = d.paramNames[i] ? d.paramNames[i] : "--";
                drawKnobStatic(bgSpr, parCenters[i], 102, 10, pName);
            }
            
            lastRenderMode = d.activeMode;
        }

        // Fast-copy the pre-rendered static background to the foreground sprite
        spr.pushImage(0, 0, 320, 240, (uint16_t*)bgSpr.getPointer());

        // --- 1. TOP STATUS BAR ---
        spr.setTextDatum(TL_DATUM);
        spr.printf("%.2fV %d%%", d.batVoltage, d.batPercent);

        spr.setTextDatum(TC_DATUM);
        spr.drawString(MODE_NAMES[d.activeMode % 10], 160, 2, 2);

        // Edit Mode Flag override for Bluetooth text area
        spr.setTextDatum(TR_DATUM);
        if (d.isKnobEditMode) {
            spr.setTextColor(TFT_RED, TFT_BLACK);
            spr.drawString("EDIT MODE", 318, 2);
            spr.setTextColor(TFT_WHITE, TFT_BLACK); // Reset Color
        } else {
            spr.drawString(d.bleConnected ? "BT: CONN" : "BT: WAIT", 318, 2);
        }

        // --- 2. VERTICAL VU METERS (IN Left | OUT Right) ---
        int inH = constrain((int)(d.inMeter * 90.0f), 0, 90);
        int outH = constrain((int)(d.outMeter * 90.0f), 0, 90);
        
        // The background sprite handles the borders; we just fill the color
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
                
                spr.fillRect(bx, 134, badgeW, 16, TFT_BLUE);
                spr.setTextColor(TFT_WHITE, TFT_BLUE);
                spr.setTextDatum(TC_DATUM);
                spr.drawString(FX_NAMES[fxIdx], bx + (badgeW / 2), 138, 1);
            }
        }
        spr.setTextColor(TFT_WHITE, TFT_BLACK); // Reset for diagnostic text

        // --- 6. SYSTEM DIAGNOSTICS BAR ---
        spr.setTextDatum(TL_DATUM);
        spr.printf("CPU: %d%%  SRM:%dK  PSR:%dK  SR:%dkHz  PLL: %dms\n",
             (int)d.dspCoreLoad, d.freeSRAM, d.freePSRAM, d.sampleRate / 1000, d.peakLatency);
        spr.printf("UDF: %d      DMA: %d      STK: %dB\n",
             d.underflows, d.dmaCount, d.stackWatermark);

        // --- 7. WARNING OVERLAYS (Drawn last to sit on top of UI) ---
        if (d.showSavingScreen) {
            spr.fillRect(60, 70, 200, 60, TFT_BLUE);
            spr.drawRect(60, 70, 200, 60, TFT_WHITE);
            
            spr.setTextColor(TFT_WHITE, TFT_BLUE);
            spr.setTextDatum(MC_DATUM);
            
            spr.drawString("SAVING...", 160, 92, 2);
            spr.drawString("Please Wait", 160, 114, 2);
            
            spr.setTextColor(TFT_WHITE, TFT_BLACK); // Reset Color
        } 
        else if (d.showKnobModeScreen) {
            spr.fillRect(60, 70, 200, 60, TFT_DARKGREEN);
            spr.drawRect(60, 70, 200, 60, TFT_WHITE);
            
            spr.setTextColor(TFT_WHITE, TFT_DARKGREEN);
            spr.setTextDatum(MC_DATUM);
            
            spr.drawString("KNOB MODE", 160, 92, 2);
            spr.drawString("Rebooting...", 160, 114, 2);
            
            spr.setTextColor(TFT_WHITE, TFT_BLACK); // Reset Color
        }
        else if (d.showBleWarning) {
            spr.fillRect(60, 70, 200, 60, TFT_RED);
            spr.drawRect(60, 70, 200, 60, TFT_WHITE);
            
            spr.setTextColor(TFT_WHITE, TFT_RED);
            spr.setTextDatum(MC_DATUM);
            
            spr.drawString("REBOOT REQUIRED", 160, 92, 2);
            spr.drawString("To Enable BLE", 160, 114, 2);
            
            spr.setTextColor(TFT_WHITE, TFT_BLACK); // Reset Color
        }

        // Push everything to the physical screen
        spr.pushSprite(0, 0);
    }
};

#endif // TARGET_LILYGO

#endif // DISPLAY_MANAGER_H