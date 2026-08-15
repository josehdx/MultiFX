#ifndef LILYGO_UI_H
#define LILYGO_UI_H

#include <TFT_eSPI.h>
#include <math.h>

// Data structure passed from Core 1 DisplayTask to the UI Renderer
struct UIData {
    int activeMode;
    float batV;
    int batPct;
    bool btConn;
    bool bleEnabled;
    float inLvl;
    float outLvl;
    float cpuLoad;
    uint32_t srVal;
    uint32_t pllVal;
    
    bool fxStates[10];
    float fxMem[10];
    float params[10][5];
    
    uint16_t pb1;
    uint16_t pb2;
    uint16_t pb3;
    uint16_t cc11;
    
    bool showTelemetry;
    uint32_t underflows;
    uint32_t dmaCount;
    uint32_t stackWatermark;
};

class LilyGoUI {
private:
    TFT_eSPI tft;
    TFT_eSprite spr;
    TFT_eSprite bgSpr;

    int lastRenderMode;
    bool isDeadState;
    UIData cache;

    const char* EFFECT_NAMES[10] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};
    
    const char* PAR_LABELS[10][5] = {
        {"HEEL", "TOE ", "DRY ", "WET ", "--- "}, // 0: WHAMMY
        {"HEEL", "TOE ", "DIFF", "ATTK", "REL "}, // 1: FREEZE
        {"INTV", "RATE", "DRV ", "DLY ", "--- "}, // 2: FEEDBACK
        {"INTV", "MIX ", "--- ", "--- ", "--- "}, // 3: HARMONY
        {"SEMI", "CENT", "--- ", "--- ", "--- "}, // 4: CAPO
        {"PTCH", "ATTK", "REL ", "FLTR", "MIX "}, // 5: SYNTH
        {"PTCH", "SMTH", "MIX ", "--- ", "--- "}, // 6: PAD
        {"PTCH", "RATE", "MIX ", "--- ", "--- "}, // 7: CHORUS
        {"HEEL", "TOE ", "THRS", "ATTK", "REL "}, // 8: SWELL
        {"PTCH", "DPTH", "--- ", "--- ", "--- "}  // 9: VIBRATO
    };
    
    const int PAR_COUNT[10] = {4, 5, 4, 2, 2, 5, 3, 3, 5, 2};
    const char* FX_BADGES[10] = {"[WH]", "[FZ]", "[FB]", "[HR]", "[CP]", "[SY]", "[PD]", "[CH]", "[SW]", "[VB]"};

    void drawCircularSlider(TFT_eSprite& sprite, int x, int y, int width, float normVal, bool isBipolar) {
        int trackHeight = 4;
        int circleRadius = 5;
        
        sprite.fillRect(x, y - (trackHeight/2), width, trackHeight, TFT_DARKGREY);
        
        int circleX;
        if (isBipolar) {
            float clamped = constrain(normVal, -1.0f, 1.0f);
            circleX = x + (width / 2) + (int)((clamped) * (width / 2));
        } else {
            float clamped = constrain(normVal, 0.0f, 1.0f);
            circleX = x + (int)(clamped * width);
        }
        
        sprite.fillCircle(circleX, y, circleRadius, TFT_WHITE);
        sprite.drawCircle(circleX, y, circleRadius, TFT_BLACK); 
    }

    bool checkIsDirty(const UIData& data) {
        if (data.activeMode != cache.activeMode) return true;
        if (data.pb1 != cache.pb1 || data.pb2 != cache.pb2 || data.pb3 != cache.pb3 || data.cc11 != cache.cc11) return true;
        if (fabsf(data.inLvl - cache.inLvl) > 0.01f || fabsf(data.outLvl - cache.outLvl) > 0.01f) return true;
        if (fabsf(data.cpuLoad - cache.cpuLoad) > 1.0f) return true;
        if (data.batPct != cache.batPct || data.btConn != cache.btConn || data.srVal != cache.srVal || data.pllVal != cache.pllVal) return true;
        for(int i = 0; i < 10; i++) {
            if (data.fxStates[i] != cache.fxStates[i]) return true;
        }
        return false;
    }

public:
    LilyGoUI() : tft(), spr(&tft), bgSpr(&tft), lastRenderMode(-1), isDeadState(false) {}

    void init() {
        tft.init();
        tft.setRotation(1);
        tft.fillScreen(TFT_BLACK);
        
        spr.setColorDepth(16);
        spr.createSprite(320, 170);
        bgSpr.setColorDepth(16);
        bgSpr.createSprite(320, 170);

        if (bgSpr.getPointer()) {
            bgSpr.fillSprite(TFT_BLACK);
            int cx_top[4] = {40, 120, 200, 280};
            int cy1 = 55;
            int r = 18; 
            
            for(int i = 0; i < 4; i++) { bgSpr.drawCircle(cx_top[i], cy1, r, TFT_DARKGREY); }
            bgSpr.setTextColor(TFT_WHITE, TFT_BLACK);
            const char* topLabels[] = {"PB1 [BIP]", "PB2 [BIP]", "PB3 [UNI]", "CC11[UNI]"};
            for(int i = 0; i < 4; i++) { bgSpr.drawCentreString(topLabels[i], cx_top[i], cy1 - 28, 1); }
            bgSpr.setTextColor(TFT_DARKGREY, TFT_BLACK);
            bgSpr.drawString("IN", 1, 125, 1);
            bgSpr.drawString("OUT", 300, 125, 1);
        }
    }

    void drawDeadBattery() {
        if (!isDeadState) {
            tft.fillScreen(TFT_RED);
            tft.setTextColor(TFT_WHITE);
            tft.drawString("BATTERY DEAD", 40, 60, 4);
            isDeadState = true;
        }
    }

    void resetDeadState() {
        isDeadState = false;
    }

    void render(const UIData& data) {
        if (!checkIsDirty(data)) return;
        
        // Cache update
        cache = data;

        if (!spr.getPointer() || !bgSpr.getPointer()) return;

        spr.pushImage(0, 0, 320, 170, (uint16_t*)bgSpr.getPointer());

        int cx_top[4] = {40, 120, 200, 280};
        int cx_bot[5] = {32, 96, 160, 224, 288};
        int cy1 = 55, cy2 = 105;
        int r = 18;
        char buf[64];

        if(data.activeMode != lastRenderMode) {
            spr.fillRect(0, 0, 320, 20, TFT_NAVY);
            spr.fillRect(10, 70, 300, 60, TFT_BLACK); 
            int activeParams = PAR_COUNT[data.activeMode];
            for(int i = 0; i < activeParams; i++) { spr.drawCircle(cx_bot[i], cy2, r, TFT_DARKGREY); }
            lastRenderMode = data.activeMode;
        }
        
        spr.setTextColor(TFT_WHITE, TFT_NAVY);
        sprintf(buf, "%4.2fV %3d%%  ", data.batV, data.batPct);
        spr.drawString(buf, 4, 2, 2);
        sprintf(buf, "   %-10s   ", EFFECT_NAMES[data.activeMode]);
        spr.drawCentreString(buf, 160, 2, 2);
        
        if (!data.bleEnabled) {
            sprintf(buf, "  BT: OFF ");
        } else {
            sprintf(buf, "  BT: %-4s", data.btConn ? "CONN" : "WAIT");
        }
        spr.drawString(buf, 240, 2, 2);

        int inH = (int)(data.inLvl * 90.0f); if(inH > 90) inH = 90;
        int outH = (int)(data.outLvl * 90.0f); if(outH > 90) outH = 90;
        spr.fillRect(2, 30, 6, 90 - inH, TFT_BLACK);
        spr.fillRect(2, 120 - inH, 6, inH, TFT_GREEN);
        spr.fillRect(312, 30, 6, 90 - outH, TFT_BLACK);
        spr.fillRect(312, 120 - outH, 6, outH, TFT_GREEN);
        
        spr.setTextColor(TFT_CYAN, TFT_BLACK);
        float pb1_norm = ((float)data.pb1 - 8192.0f) / 8191.5f;
        float pb2_norm = ((float)data.pb2 - 8192.0f) / 8191.5f;
        float pb3_norm = (float)data.pb3 / 16383.0f;
        float cc11_norm = (float)data.cc11 / 16383.0f;
        
        sprintf(buf, "%4d%%", (int)std::roundf(pb1_norm * 100.0f)); spr.drawCentreString(buf, cx_top[0], cy1-4, 1);
        sprintf(buf, "%4d%%", (int)std::roundf(pb2_norm * 100.0f)); spr.drawCentreString(buf, cx_top[1], cy1-4, 1);
        sprintf(buf, "%4d%%", (int)std::roundf(pb3_norm * 100.0f)); spr.drawCentreString(buf, cx_top[2], cy1-4, 1);
        sprintf(buf, "%4d%%", (int)std::roundf(cc11_norm * 100.0f)); spr.drawCentreString(buf, cx_top[3], cy1-4, 1);

        drawCircularSlider(spr, cx_top[0]-20, cy1+14, 40, pb1_norm, true);
        drawCircularSlider(spr, cx_top[1]-20, cy1+14, 40, pb2_norm, true);
        drawCircularSlider(spr, cx_top[2]-20, cy1+14, 40, pb3_norm, false);
        drawCircularSlider(spr, cx_top[3]-20, cy1+14, 40, cc11_norm, false);

        spr.setTextColor(TFT_WHITE, TFT_BLACK);
        int activeParams = PAR_COUNT[data.activeMode];
        for(int i = 0; i < activeParams; i++) { spr.drawCentreString(PAR_LABELS[data.activeMode][i], cx_bot[i], cy2-28, 1); }

        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        float dispVals[5] = {0.0f};
        switch(data.activeMode) {
            case 0: dispVals[0]=data.fxMem[1]; dispVals[1]=data.fxMem[0]; dispVals[2]=data.params[0][0]; dispVals[3]=data.params[0][1]; break;
            case 1: dispVals[0]=data.fxMem[1]; dispVals[1]=data.fxMem[0]; dispVals[2]=data.params[1][0]; dispVals[3]=data.params[1][1]; dispVals[4]=data.params[1][2]; break;
            case 2: dispVals[0]=data.fxMem[2]; dispVals[1]=data.params[2][0]; dispVals[2]=data.params[2][1]; dispVals[3]=data.params[2][2]; break;
            case 3: dispVals[0]=data.fxMem[3]; dispVals[1]=data.params[3][0]; break;
            case 4: dispVals[0]=data.fxMem[4]; dispVals[1]=data.fxMem[4]; break;
            case 5: dispVals[0]=data.fxMem[5]; dispVals[1]=data.params[5][0]; dispVals[2]=data.params[5][1]; dispVals[3]=data.params[5][2]; dispVals[4]=data.params[5][3]; break;
            case 6: dispVals[0]=data.fxMem[6]; dispVals[1]=data.params[6][0]; dispVals[2]=data.params[6][1]; break;
            case 7: dispVals[0]=data.fxMem[7]; dispVals[1]=data.params[7][0]; dispVals[2]=data.params[7][1]; break;
            case 8: dispVals[0]=data.fxMem[1]; dispVals[1]=data.fxMem[0]; dispVals[2]=data.params[8][0]; dispVals[3]=data.params[8][1]; dispVals[4]=data.params[8][2]; break;
            case 9: dispVals[0]=data.fxMem[9]; dispVals[1]=data.params[9][0]; break;
        }
        
        for(int i = 0; i < activeParams; i++) {
            if (strcmp(PAR_LABELS[data.activeMode][i], "--- ") != 0) {
                sprintf(buf, "%6.2f", dispVals[i]); 
                spr.drawCentreString(buf, cx_bot[i], cy2-4, 1);
                
                bool isBip = (i == 0 && dispVals[i] < 0.0f) || (i == 0); 
                float normalized = isBip ? (dispVals[i] / 24.0f) : dispVals[i]; 
                drawCircularSlider(spr, cx_bot[i]-20, cy2+12, 40, normalized, isBip);
            }
        }

        int fxX = 22;
        for(int i = 0; i < 10; i++) {
            spr.setTextColor(data.fxStates[i] ? TFT_BLACK : TFT_DARKGREY, data.fxStates[i] ? TFT_GREEN : TFT_BLACK);
            spr.drawString(FX_BADGES[i], fxX, 138, 1);
            fxX += 28;
        }

        spr.fillRect(0, 153, 320, 17, TFT_DARKGREY);
        spr.setTextColor(TFT_WHITE, TFT_DARKGREY);
        
        sprintf(buf, "CPU:%2d%% SRM:%3dK PSR:%4dK SR:%2dkHz PLL:%dms", 
            (int)data.cpuLoad, 
            (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1024), 
            (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1024),
            data.srVal/1000,
            (int)data.pllVal);
        spr.drawString(buf, 5, 155, 1);

        if (data.showTelemetry) {
            spr.fillRect(0, 142, 320, 11, TFT_BLACK);
            spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
            sprintf(buf, "UDF:%lu   DMA:%lu   STK:%luB", 
                data.underflows, data.dmaCount, data.stackWatermark);
            spr.drawString(buf, 5, 143, 1);
        }

        spr.pushSprite(0, 0);
    }
};

#endif