#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <driver/i2s_std.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_biquad.h"
#include "driver/rtc_io.h"
#include "esp_bt.h"
#include <math.h>
#include <Preferences.h>
#include "esp_private/brownout.h"
#include "PedalManager.h"

#define ENABLE_PAR_KNOBS false
struct AppSettings { float fxMem[10]; float params[10][5]; };
Preferences preferences; volatile bool settingsNeedSaving = false; volatile unsigned long lastParameterChangeTime = 0;
volatile float fxParams[10][5] = {{0.0f,1.0f,0.0f,0.0f,0.0f},{0.6f,0.0002f,0.00005f,0.0f,0.0f},{5120.0f,30.0f,0.02f,0.0f,0.0f},{0.5f,0.0f,0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f,0.0f,0.0f},{0.1f,0.005f,0.3f,0.1f,0.0f},{0.95f,1.5f,0.0f,0.0f,0.0f},{1536.0f,0.4f,0.0f,0.0f,0.0f},{0.015f,0.00002f,0.00005f,0.0f,0.0f},{1.0f,0.0f,0.0f,0.0f,0.0f}};
const bool INVERT_PB3 = false;

void __attribute__((constructor)) pre_boot_kill_switch() { gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_38, 0); gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_15, 0); gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_5, 0); }

void updateLUT(); void DisplayTask(void * pvParameters); void MidiTask(void * pvParameters); void AudioDSPTask(void * pvParameters); bool channelMessageCallback(ChannelMessage cm); void updateParameterFromCC(uint8_t cc, uint8_t val);

i2s_chan_handle_t tx_chan, rx_chan; SemaphoreHandle_t audioBufferMutex = NULL; TaskHandle_t audioTaskHandle = NULL;
volatile uint32_t currentSampleRate = 48000;

#define HOP_SIZE 64
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 131072
int16_t *delayBuffer = nullptr, *fbDelayBuffer = nullptr, *freezeBuffer = nullptr;
float *volatile pitchShiftLUT = nullptr, *pitchShiftLUT_temp = nullptr;
int writeIndex = 0, fbDelayWriteIdx = 0;

#define HANN_LUT_SIZE 1024
#define LFO_LUT_SIZE 1024
#define WAVE_LUT_SIZE 2048
DRAM_ATTR float hannLUT[HANN_LUT_SIZE] __attribute__((aligned(16))); DRAM_ATTR float lfoLUT[LFO_LUT_SIZE] __attribute__((aligned(16))); DRAM_ATTR float synthLUT[WAVE_LUT_SIZE] __attribute__((aligned(16)));

volatile float globalHarmRatio=1.0f, globalChorusRatio=1.0f, globalFbRatio=1.0f, globalVibratoPhaseInc=0.0f;
uint32_t tap_w1_1=0, tap_w1_2=256<<16, tap_w2_1=0, tap_w2_2=256<<16, tap_w3_1=0, tap_w3_2=256<<16, tap_w4_1=0, tap_w4_2=256<<16, tap_w5_1=0, tap_w5_2=256<<16;
float currentWindowSize = 512.0f; int freezeLength = 48000; bool wasFrozen = false; volatile bool apfNeedsClear = false; volatile float freezeRamp = 0.0f;
float apf1Buffer[1009] = {0.0f}, apf2Buffer[863] = {0.0f}; int apf1Idx = 0, apf2Idx = 0; volatile int feedbackIntervalIdx = 0; volatile bool lutNeedsUpdate = false; volatile uint16_t lastActivePedal = 8192;

TFT_eSPI tft = TFT_eSPI(); TFT_eSprite spr = TFT_eSprite(&tft), meterSpr = TFT_eSprite(&tft);
volatile bool forceUIUpdate=true; volatile int activeEffectMode=0; volatile float effectMemory[10]={12.0f,-12.0f,0.0f,5.0f,-2.0f,-12.0f,-12.0f,12.0f,0.0f,0.0f}; volatile float pitchShiftFactor=1.0f;
volatile bool isWhammyActive=true, isFrozen=false, isFeedbackActive=false, isHarmonizerMode=false, isSynthMode=false, isPadMode=false, isCapoMode=false, isChorusMode=false, isSwellMode=false, isVibratoMode=false, isVolumeMode=false, isPB2WiperMode=false;
volatile float chorusLfoPhase=0.0f, feedbackLfoPhase=0.0f, vibratoLfoPhase=0.0f, swellGain=0.0f, volumePedalGain=1.0f, feedbackRamp=0.0f;
float fbHpfState=0.0f, feedbackFilter=0.0f; volatile int latencyMode=0; const float LATENCY_WINDOWS[]={512.0f,1024.0f,2048.0f,4096.0f};
volatile bool globalAudioResetRequested=false, sampleRateToggleRequested=false, pb2ToggleRequested=false, panicResetRequested=false;
volatile int hardwareSyncMuteFrames=0; unsigned long lastActivityTime=0, lastScreenActivityTime=0; const unsigned long LIGHT_SLEEP_TIMEOUT=25000, SCREEN_OFF_TIMEOUT=15000;
volatile bool isScreenOff=false, wakeupPending=false, sleepRequested=false, isSleeping=false; volatile float core1_load=0.0f; volatile bool isAdcPaused=false;

adc_continuous_handle_t multifx_adc_handle = NULL; volatile int latestPB1=2048, latestPB2=2048, latestPB3=2048, latestBat=2048, currentBatteryPercent=100; volatile float currentBatteryVoltage=4.00f; volatile bool isBatteryCharging=false;
const int BATTERY_PIN=4, BOOT_SENSE_PIN=0; pin_t pinPB=1, pinPB2=2, pinPB3=10, pinPar1=3, pinPar2=11, pinPar3=12, pinPar4=13, pinPar5=14;
uint16_t lastMidiSent=8192; volatile uint16_t currentPB1=8192, currentPB2=8192, currentPB3=8192, currentCC11=0; volatile float ui_audio_level=0.0f, ui_output_level=0.0f;

#define HW_UART_TX_PIN 21
#define HW_UART_RX_PIN -1
FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar1=pinPar1, filterPar2=pinPar2, filterPar3=pinPar3, filterPar4=pinPar4, filterPar5=pinPar5;
BluetoothMIDI_Interface btmidi; USBMIDI_Interface usbmidi; MIDI_PipeFactory<4> pipes; PedalManager pedals;

bool IRAM_ATTR i2s_rx_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) { BaseType_t high_task_wakeup=pdFALSE; if(audioTaskHandle!=NULL) vTaskNotifyGiveFromISR(audioTaskHandle, &high_task_wakeup); return high_task_wakeup==pdTRUE; }

void fetchADCDMA() {
    if(isAdcPaused) return; uint8_t result[1024]; uint32_t ret_num=0; esp_err_t err;
    while(true) {
        err=adc_continuous_read(multifx_adc_handle, result, sizeof(result), &ret_num, 0);
        if(err==ESP_OK) {
            if(ret_num>0) {
                for(int i=0; i<ret_num; i+=SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p=(adc_digi_output_data_t*)&result[i];
                    if(p->type2.channel==ADC_CHANNEL_0) latestPB1=p->type2.data;
                    if(p->type2.channel==ADC_CHANNEL_1) latestPB2=p->type2.data;
                    if(p->type2.channel==ADC_CHANNEL_9) latestPB3=p->type2.data;
                    if(p->type2.channel==ADC_CHANNEL_3) latestBat=p->type2.data;
                }
            }
        } else if(err==ESP_ERR_TIMEOUT) break; else { adc_continuous_stop(multifx_adc_handle); adc_continuous_start(multifx_adc_handle); break; }
    }
}

void switchEffectMode(int newMode) {
    if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    activeEffectMode=(newMode%10+10)%10; isWhammyActive=true; isFrozen=false; isFeedbackActive=false; isHarmonizerMode=false; isSynthMode=false; isPadMode=false; isCapoMode=false; isChorusMode=false; isSwellMode=false; isVibratoMode=false;
    if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex);
    lutNeedsUpdate=true; forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis();
}

void saveSettings() {
    AppSettings cs; if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    for(int i=0; i<10; i++) { cs.fxMem[i]=effectMemory[i]; for(int p=0; p<5; p++) cs.params[i][p]=fxParams[i][p]; }
    uint16_t fxStates=0; if(isWhammyActive) fxStates|=(1<<0); if(isFrozen) fxStates|=(1<<1); if(isFeedbackActive) fxStates|=(1<<2); if(isHarmonizerMode) fxStates|=(1<<3); if(isCapoMode) fxStates|=(1<<4); if(isSynthMode) fxStates|=(1<<5); if(isPadMode) fxStates|=(1<<6); if(isChorusMode) fxStates|=(1<<7); if(isSwellMode) fxStates|=(1<<8); if(isVibratoMode) fxStates|=(1<<9);
    if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex);
    preferences.begin("whammy_cfg", false); preferences.putInt("activeMode", (int)activeEffectMode); preferences.putInt("latMode", (int)latencyMode); preferences.putBool("pb2Wiper", isPB2WiperMode); preferences.putBool("volMode", isVolumeMode); preferences.putUShort("fxStates", fxStates); preferences.putUInt("sampleRate", currentSampleRate); preferences.putInt("fbIdx", constrain((int)feedbackIntervalIdx,0,4)); preferences.putBytes("dspData", &cs, sizeof(AppSettings)); preferences.end();
}

int getBatteryPercentage(float voltage) {
    float cv = __builtin_fmaxf(3.30f, __builtin_fminf(4.15f, voltage));
    if(cv>=4.15f) return 100; if(cv<=3.30f) return 0; if(cv>=4.00f) return 90+(int)((cv-4.00f)/0.15f*10.0f); if(cv>=3.90f) return 80+(int)((cv-3.90f)/0.10f*10.0f); if(cv>=3.80f) return 70+(int)((cv-3.80f)/0.10f*10.0f); if(cv>=3.75f) return 60+(int)((cv-3.75f)/0.05f*10.0f); if(cv>=3.70f) return 50+(int)((cv-3.70f)/0.05f*10.0f); if(cv>=3.65f) return 40+(int)((cv-3.65f)/0.05f*10.0f); if(cv>=3.60f) return 30+(int)((cv-3.60f)/0.05f*10.0f); if(cv>=3.55f) return 20+(int)((cv-3.55f)/0.05f*10.0f); if(cv>=3.50f) return 10+(int)((cv-3.50f)/0.05f*10.0f); return (int)(__builtin_fmaxf(0.0f,cv-3.30f)/0.20f*10.0f);
}

void calibratePBs() {
    for(int i=0; i<50; i++) { fetchADCDMA(); vTaskDelay(pdMS_TO_TICKS(1)); }
    long s1=0, s2=0, s3=0; for(int i=1; i<=250; i++) { fetchADCDMA(); s1+=latestPB1; s2+=latestPB2; s3+=latestPB3; } pedals.setCenters(s1/250, s2/250, s3/250);
}

void toggleSampleRate() {
    sleepRequested = true; 
    int timeoutCounter = 0;
    while (!isSleeping && timeoutCounter < 40) { 
        vTaskDelay(pdMS_TO_TICKS(5)); 
        timeoutCounter++; 
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    i2s_channel_disable(tx_chan); 
    i2s_channel_disable(rx_chan); 
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);

    vTaskDelay(pdMS_TO_TICKS(10)); 
    
    currentSampleRate = (currentSampleRate == 96000) ? 48000 : 96000;
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate); 
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg); 
    i2s_channel_reconfig_std_clock(rx_chan, &clk_cfg);
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    saveSettings(); settingsNeedSaving = false; updateLUT(); lutNeedsUpdate = false;
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    freezeLength = currentSampleRate; 
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t));
    i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); 
    globalAudioResetRequested = true; 
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    pedals.triggerSystemRecovery(); sleepRequested = false; forceUIUpdate = true; lastParameterChangeTime = millis();
}

void turnScreenOff() { if(!isScreenOff) { digitalWrite(38, LOW); digitalWrite(15, LOW); isScreenOff=true; } }
void turnScreenOn() { if(isScreenOff && !wakeupPending) wakeupPending=true; }

void goToLightSleep() {
    turnScreenOff(); sleepRequested = true; int timeoutCounter = 0;
    while (!isSleeping && timeoutCounter < 10) { vTaskDelay(pdMS_TO_TICKS(10)); timeoutCounter++; }
    vTaskDelay(pdMS_TO_TICKS(20));
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    i2s_channel_disable(tx_chan); i2s_channel_disable(rx_chan);
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    while (digitalRead(BOOT_SENSE_PIN) == HIGH) { 
        uint8_t trash[1024]; uint32_t ret_num = 0;
        adc_continuous_read(multifx_adc_handle, trash, sizeof(trash), &ret_num, 0);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    
    if (audioBufferMutex != NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); globalAudioResetRequested = true;
    if (audioBufferMutex != NULL) xSemaphoreGive(audioBufferMutex);
    
    sleepRequested = false; timeoutCounter = 0;
    while (isSleeping && timeoutCounter < 10) { vTaskDelay(pdMS_TO_TICKS(10)); timeoutCounter++; }
    vTaskDelay(pdMS_TO_TICKS(200)); if (isScreenOff) turnScreenOn(); 
    
    pedals.triggerSystemRecovery(); lastActivityTime = millis(); lastScreenActivityTime = millis();
}

inline float IRAM_ATTR __attribute__((hot)) __attribute__((always_inline)) processTap(uint32_t tapPhase, const int16_t* buffer, int currentWriteIdx, uint32_t windowMask, uint32_t hannIntMult) {
    int T=(tapPhase>>16)&windowMask; float frac=(tapPhase&0xFFFF)*0.0000152587890625f; int effTap=T+2;
    int idx1=(currentWriteIdx-effTap+MAX_BUFFER_SIZE)&BUFFER_MASK, idx0=(idx1+1)&BUFFER_MASK, idx2=(idx1-1+MAX_BUFFER_SIZE)&BUFFER_MASK, idx3=(idx1-2+MAX_BUFFER_SIZE)&BUFFER_MASK;
    float y0=(float)buffer[idx0]*3.0517578125e-5f, y1=(float)buffer[idx1]*3.0517578125e-5f, y2=(float)buffer[idx2]*3.0517578125e-5f, y3=(float)buffer[idx3]*3.0517578125e-5f;
    float c0=y1, c1=0.5f*(y2-y0), c3=1.5f*(y1-y2)+0.5f*(y3-y0), c2=y0-y1+c1-c3; float sample=((c3*frac+c2)*frac+c1)*frac+c0; int lutIdx=((uint32_t)(T*hannIntMult)>>16)&1023; return sample*hannLUT[lutIdx];
}

void updateLUT() {
    float basePitch=0.0f; if(isCapoMode || (activeEffectMode==4 && isWhammyActive)) basePitch+=effectMemory[4];
    float toeBend=effectMemory[0], heelBend=effectMemory[1], harmRatioMem=effectMemory[3], chorusRatioMem=effectMemory[7], vibHzMem=effectMemory[9]; int fbIntervalIdxLocal=feedbackIntervalIdx;
    for(int i=0; i<16384; i++) {
        float normalizedThrow=(i>=8192)?((float)(i-8192)/8191.0f):((float)(i-8192)/8192.0f); float dynamicBend=(normalizedThrow>=0.0f)?(toeBend*normalizedThrow):(heelBend*fabsf(normalizedThrow));
        pitchShiftLUT_temp[i]=powf(2.0f,(basePitch+dynamicBend)/12.0f); if(i%2048==0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    float* tempPtr=pitchShiftLUT; pitchShiftLUT=pitchShiftLUT_temp; pitchShiftLUT_temp=tempPtr;
    globalHarmRatio=powf(2.0f, harmRatioMem/12.0f); globalChorusRatio=powf(2.0f, chorusRatioMem/12.0f); float fbIntervals[5]={0.0f,12.0f,19.0f,24.0f,28.0f}; globalFbRatio=powf(2.0f, fbIntervals[constrain(fbIntervalIdxLocal,0,4)]/12.0f);
    float vibHz=(vibHzMem!=0.0f)?fabsf(vibHzMem):2.0f; globalVibratoPhaseInc=(vibHz*LFO_LUT_SIZE)/(float)currentSampleRate;
    if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex);
}

void updateMeters() {
    int barHeight=98, inFillHeight=constrain((int)(ui_audio_level*barHeight),0,barHeight); meterSpr.fillSprite(TFT_BLACK); meterSpr.fillRect(0, barHeight-inFillHeight, 6, inFillHeight, (ui_audio_level>0.90f)?TFT_RED:TFT_GREEN); meterSpr.pushSprite(11,31);
    int outFillHeight=constrain((int)(ui_output_level*barHeight),0,barHeight); meterSpr.fillSprite(TFT_BLACK); meterSpr.fillRect(0, barHeight-outFillHeight, 6, outFillHeight, (ui_output_level>0.90f)?TFT_RED:TFT_GREEN); meterSpr.pushSprite(spr.width()-17, 31);
}

void drawCircularGauge(TFT_eSprite& sprite, int x, int y, int radius, float value, const char* label, const char* valStr, uint32_t color, int type) {
    int thickness=3; sprite.drawArc(x, y, radius, radius-thickness, 210, 150, TFT_DARKGREY, TFT_BLACK, true);
    if(type==1) { float clamped=fmaxf(-1.0f,fminf(1.0f,value)); int sweep=(int)(clamped*150.0f); if(sweep>0) sprite.drawArc(x,y,radius,radius-thickness,0,sweep,color,TFT_BLACK,true); else if(sweep<0) sprite.drawArc(x,y,radius,radius-thickness,360+sweep,360,color,TFT_BLACK,true); }
    else if(type==2) { float clamped=fmaxf(0.0f,fminf(1.0f,value)); int sweep=(int)(clamped*300.0f); if(sweep>0) { int startAngle=(150-sweep+360)%360; sprite.drawArc(x,y,radius,radius-thickness,startAngle,150,color,TFT_BLACK,true); } }
    else { float clamped=fmaxf(0.0f,fminf(1.0f,value)); int sweep=(int)(clamped*300.0f); if(sweep>0) { int endAngle=(210+sweep)%360; sprite.drawArc(x,y,radius,radius-thickness,210,endAngle,color,TFT_BLACK,true); } }
    sprite.setTextDatum(MC_DATUM); sprite.setTextColor(TFT_WHITE, TFT_BLACK); sprite.setTextSize(1); sprite.drawString(valStr,x,y-4); sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK); sprite.drawString(label,x,y+14);
}
struct GaugeDef { int type; float normVal; const char* label; char valStr[16]; };

void updateDisplay() {
    spr.fillSprite(TFT_BLACK); int renderMode=activeEffectMode; char batStr[24];
    if(isBatteryCharging) { snprintf(batStr, sizeof(batStr), "CHG %.2fV %d%%", currentBatteryVoltage, currentBatteryPercent); spr.setTextColor(TFT_GREEN, TFT_BLACK); }
    else { snprintf(batStr, sizeof(batStr), "%.2fV %d%%", currentBatteryVoltage, currentBatteryPercent); spr.setTextColor((currentBatteryPercent>20)?TFT_GREEN:TFT_RED, TFT_BLACK); }
    spr.setTextDatum(TL_DATUM); spr.setTextSize(1); spr.drawString(batStr, 5, 2); spr.setTextDatum(TR_DATUM);
    if(btmidi.isConnected()) { spr.setTextColor(TFT_GREEN, TFT_BLACK); spr.drawString("BT: CONN", 315, 2); } else { spr.setTextColor(TFT_YELLOW, TFT_BLACK); spr.drawString("BT: WAIT", 315, 2); }
    const char* effectTitleNames[]={"WHAMMY","FREEZE","FEEDBACK","HARMONY","CAPO","SYNTH","PAD","CHORUS","SWELL","VIBRATO"};
    uint32_t effectTitleColors[]={TFT_ORANGE,TFT_CYAN,TFT_RED,TFT_MAGENTA,TFT_GREEN,TFT_YELLOW,TFT_PINK,TFT_SKYBLUE,TFT_WHITE,TFT_PURPLE};
    spr.setTextDatum(MC_DATUM); spr.setTextSize(2); spr.setTextColor(effectTitleColors[renderMode], TFT_BLACK); spr.drawString(effectTitleNames[renderMode], 160, 15);
    bool effectIsActive=false;
    if(renderMode==0) effectIsActive=isWhammyActive; else if(renderMode==1) effectIsActive=(isWhammyActive||isFrozen); else if(renderMode==2) effectIsActive=(isWhammyActive||isFeedbackActive); else if(renderMode==3) effectIsActive=(isWhammyActive||isHarmonizerMode); else if(renderMode==4) effectIsActive=(isWhammyActive||isCapoMode); else if(renderMode==5) effectIsActive=(isWhammyActive||isSynthMode); else if(renderMode==6) effectIsActive=(isWhammyActive||isPadMode); else if(renderMode==7) effectIsActive=(isWhammyActive||isChorusMode); else if(renderMode==8) effectIsActive=(isWhammyActive||isSwellMode); else if(renderMode==9) effectIsActive=(isWhammyActive||isVibratoMode);
    spr.fillCircle(240, 15, 6, effectIsActive?TFT_GREEN:TFT_RED); spr.drawCircle(240, 15, 6, TFT_WHITE); spr.drawRect(10, 30, 8, 100, TFT_DARKGREY); spr.setTextColor(TFT_WHITE, TFT_BLACK); spr.setTextSize(1); spr.drawString("IN", 14, 140); spr.drawRect(spr.width()-18, 30, 8, 100, TFT_DARKGREY); spr.drawString("OUT", spr.width()-14, 140);
    int topY=55, topR=17; char valBuf[16]; float pb1Val=(currentPB1-8192)/8192.0f; snprintf(valBuf,sizeof(valBuf),"%d%%",(int)(pb1Val*100)); drawCircularGauge(spr,65,topY,topR,pb1Val,"PB1",valBuf,TFT_CYAN,1);
    float pb2Val=(currentPB2-8192)/8192.0f; snprintf(valBuf,sizeof(valBuf),"%d%%",(int)(pb2Val*100)); drawCircularGauge(spr,125,topY,topR,pb2Val,isPB2WiperMode?"PB2 W":"PB2 H",valBuf,TFT_MAGENTA,1);
    float pb3Val=currentPB3/16383.0f; snprintf(valBuf,sizeof(valBuf),"%d%%",(int)(pb3Val*100)); drawCircularGauge(spr,185,topY,topR,pb3Val,isVolumeMode?"VOL":"PB3",valBuf,TFT_YELLOW,0);
    float cc11Val=currentCC11/16383.0f; snprintf(valBuf,sizeof(valBuf),"%d%%",(int)(cc11Val*100)); drawCircularGauge(spr,245,topY,topR,cc11Val,"CC11",valBuf,TFT_GREEN,0);
    GaugeDef bGauges[6]; int numG=0;
    if(renderMode==0||renderMode==1||renderMode==8) { bGauges[numG++]={1,effectMemory[1]/24.0f,"HEEL",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",effectMemory[1]); bGauges[numG++]={1,effectMemory[0]/24.0f,"TOE",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",effectMemory[0]); }
    else if(renderMode==2) { float fbi[]={0.0f,12.0f,19.0f,24.0f,28.0f}; float val=fbi[constrain((int)feedbackIntervalIdx,0,4)]; bGauges[numG++]={2,val/28.0f,"OVT",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",val); }
    else if(renderMode==4) { int semi=(int)roundf(effectMemory[4]), cents=(int)roundf((effectMemory[4]-(float)semi)*100.0f); bGauges[numG++]={1,semi/24.0f,"SEMI",""}; snprintf(bGauges[numG-1].valStr,16,"%+d",semi); bGauges[numG++]={1,cents/50.0f,"CENT",""}; snprintf(bGauges[numG-1].valStr,16,"%+d",cents); bGauges[numG++]={1,effectMemory[1]/24.0f,"HEEL",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",effectMemory[1]); bGauges[numG++]={1,effectMemory[0]/24.0f,"TOE",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",effectMemory[0]); }
    else { float val=effectMemory[renderMode]; if(renderMode==3) { bGauges[numG++]={1,val/24.0f,"INT",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",val); } else if(renderMode==5) { bGauges[numG++]={1,val/24.0f,"OSC",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",val); } else if(renderMode==6) { bGauges[numG++]={1,val/24.0f,"SHFT",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",val); } else if(renderMode==7) { bGauges[numG++]={1,val/24.0f,"SHFT",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",val); } else if(renderMode==9) { bGauges[numG++]={1,val/24.0f,"BASE",""}; snprintf(bGauges[numG-1].valStr,16,"%+.1f",val); } }
    if(renderMode==0) { bGauges[numG++]={2,fxParams[0][0],"D.MIX",""}; snprintf(bGauges[numG-1].valStr,16,"%.1f",fxParams[0][0]); bGauges[numG++]={2,fxParams[0][1],"W.MIX",""}; snprintf(bGauges[numG-1].valStr,16,"%.1f",fxParams[0][1]); }
    else if(renderMode==1) { bGauges[numG++]={2,fxParams[1][0]/0.95f,"RVB",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[1][0]); bGauges[numG++]={2,(fxParams[1][1]-0.00001f)/0.001f,"ATK",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[1][1]*1000.0f); bGauges[numG++]={2,(fxParams[1][2]-0.00001f)/0.0005f,"REL",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[1][2]*1000.0f); }
    else if(renderMode==2) { bGauges[numG++]={2,(fxParams[2][0]-1000.0f)/10000.0f,"SPD",""}; snprintf(bGauges[numG-1].valStr,16,"%.0f",fxParams[2][0]); bGauges[numG++]={2,(fxParams[2][1]-1.0f)/100.0f,"DRV",""}; snprintf(bGauges[numG-1].valStr,16,"%.1f",fxParams[2][1]); bGauges[numG++]={2,(fxParams[2][2]-0.005f)/0.045f,"DLY",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[2][2]); }
    else if(renderMode==3) { bGauges[numG++]={2,fxParams[3][0],"MIX",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[3][0]); }
    else if(renderMode==5) { bGauges[numG++]={2,(fxParams[5][0]-0.01f)/0.5f,"ATK",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[5][0]); bGauges[numG++]={2,(fxParams[5][1]-0.001f)/0.05f,"REL",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[5][1]); bGauges[numG++]={2,(fxParams[5][2]-0.1f)/0.8f,"FLT",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[5][2]); bGauges[numG++]={2,fxParams[5][3],"MIX",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[5][3]); }
    else if(renderMode==6) { bGauges[numG++]={2,(fxParams[6][0]-0.8f)/0.199f,"TON",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[6][0]); bGauges[numG++]={2,fxParams[6][1]/3.0f,"MIX",""}; snprintf(bGauges[numG-1].valStr,16,"%.1f",fxParams[6][1]); }
    else if(renderMode==7) { bGauges[numG++]={2,(fxParams[7][0]-500.0f)/4500.0f,"SPD",""}; snprintf(bGauges[numG-1].valStr,16,"%.0f",fxParams[7][0]); bGauges[numG++]={2,fxParams[7][1],"MIX",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[7][1]); }
    else if(renderMode==8) { bGauges[numG++]={2,(fxParams[8][0]-0.001f)/0.05f,"THR",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[8][0]); bGauges[numG++]={2,(fxParams[8][1]-0.00001f)/0.0005f,"ATK",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[8][1]*1000.0f); bGauges[numG++]={2,(fxParams[8][2]-0.00001f)/0.0005f,"REL",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[8][2]*1000.0f); }
    else if(renderMode==9) { bGauges[numG++]={2,fxParams[9][0]/2.0f,"DEP",""}; snprintf(bGauges[numG-1].valStr,16,"%.2f",fxParams[9][0]); }
    int bY=115, bR=17; uint32_t actCol=effectTitleColors[renderMode]; for(int i=0; i<numG; i++) { int xPos=160-((numG-1)*26)+(i*52); drawCircularGauge(spr,xPos,bY,bR,bGauges[i].normVal,bGauges[i].label,bGauges[i].valStr,actCol,bGauges[i].type); }
    spr.setTextDatum(ML_DATUM); spr.setTextSize(1); int bannerX=25; auto drawBanner=[&](const char* lbl, uint32_t col){ spr.setTextColor(col,TFT_BLACK); spr.drawString(lbl,bannerX,150); bannerX+=28; };
    if(isFrozen && renderMode!=1) drawBanner("FRZ",TFT_CYAN); if(isFeedbackActive && renderMode!=2) drawBanner("SCM",TFT_RED); if(isHarmonizerMode && renderMode!=3) drawBanner("HRM",TFT_MAGENTA); if(isCapoMode && renderMode!=4) drawBanner("CAP",TFT_GREEN); if(isSynthMode && renderMode!=5) drawBanner("SYN",TFT_YELLOW); if(isPadMode && renderMode!=6) drawBanner("PAD",TFT_PINK); if(isChorusMode && renderMode!=7) drawBanner("CHO",TFT_SKYBLUE); if(isSwellMode && renderMode!=8) drawBanner("SWL",TFT_WHITE); if(isVibratoMode && renderMode!=9) drawBanner("VIB",TFT_PURPLE); if(isVolumeMode) drawBanner("VOL",TFT_DARKGREY);
    int statsRowY=162; spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK); spr.setTextDatum(ML_DATUM);
    char cpuUsageBuffer[16]; snprintf(cpuUsageBuffer,16,"CPU:%2d%%",(int)core1_load); spr.drawString(cpuUsageBuffer,25,statsRowY); char internalSramBuffer[16]; snprintf(internalSramBuffer,16,"SRM:%dK",(int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1024)); spr.drawString(internalSramBuffer,85,statsRowY); char psramBuffer[16]; snprintf(psramBuffer,16,"PSR:%dK",(int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1024)); spr.drawString(psramBuffer,150,statsRowY);
    spr.setTextDatum(MC_DATUM); spr.setTextColor(TFT_WHITE); spr.drawRect(210,statsRowY-7,40,14,TFT_DARKGREY); spr.drawString((currentSampleRate==96000)?"96k":"48k",230,statsRowY); spr.drawRect(255,statsRowY-7,40,14,TFT_DARKGREY); const char* latencyLabelStrings[]={"U.Low","Low","Mid","High"}; spr.drawString(latencyLabelStrings[latencyMode],275,statsRowY);
    spr.pushSprite(0,0); updateMeters();
}

void DisplayTask(void * pvParameters) {
    bool metersNeedClear=false;
    for(;;) {
        if(wakeupPending) { pinMode(15,OUTPUT); digitalWrite(15,HIGH); vTaskDelay(pdMS_TO_TICKS(150)); tft.init(); tft.setRotation(1); pinMode(38,OUTPUT); digitalWrite(38,HIGH); isScreenOff=false; wakeupPending=false; forceUIUpdate=true; }
        if(forceUIUpdate && !isScreenOff) { forceUIUpdate=false; updateDisplay(); metersNeedClear=true; }
        else if(!isScreenOff) { if(ui_audio_level>0.02f || ui_output_level>0.02f) { updateMeters(); metersNeedClear=true; } else if(metersNeedClear) { ui_audio_level=0.0f; ui_output_level=0.0f; updateMeters(); metersNeedClear=false; } }
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

void IRAM_ATTR __attribute__((hot)) AudioDSPTask(void * pvParameters) {
    int32_t i2s_in_block[HOP_SIZE*2] __attribute__((aligned(16))), i2s_out_block[HOP_SIZE*2] __attribute__((aligned(16)));
    float input_dc_offset=0.0f, synthEnv=0.0f, synthFilter=0.0f, padFilter=0.0f, padEnv=0.0f, inputEnvelope=0.0f, feedbackFilterVar=0.0f, smoothedVolGain=1.0f, currentPitch=1.0f, fbOutNode=0.0f, smoothed_delay_samples=0.0f;
    bool wasFeedbackActive=false; int freezeWriteIdxVar=0, freezePlayCounterVar=0, freezeStartIdxVar=0, activeFreezeLength=48000;
    bool wasSleeping=false; float c_fx[10][5]; int c_lat=0, c_act=0;
    bool c_w=true, c_fz=false, c_fb=false, c_hr=false, c_cp=false, c_sy=false, c_pd=false, c_ch=false, c_sw=false, c_vb=false; float c_pt=1.0f, c_vg=1.0f; extern volatile bool panicResetRequested;
    const float normFactor=1.0f/2147483648.0f, DC_OFFSET=1e-9f; float inBuf[HOP_SIZE] __attribute__((aligned(16))), envBuf[HOP_SIZE] __attribute__((aligned(16))), fzOutBuf[HOP_SIZE] __attribute__((aligned(16))); float masterGainBuf[HOP_SIZE] __attribute__((aligned(16))), w1Buf[HOP_SIZE] __attribute__((aligned(16))), w2Buf[HOP_SIZE] __attribute__((aligned(16))); float w3Buf[HOP_SIZE] __attribute__((aligned(16))), padFilterBuf[HOP_SIZE] __attribute__((aligned(16))), dryBuf[HOP_SIZE] __attribute__((aligned(16))); float fbOutBuf[HOP_SIZE] __attribute__((aligned(16))), sMixBuf[HOP_SIZE] __attribute__((aligned(16)));
    
    for(;;) {
        if(sleepRequested) { isSleeping=true; wasSleeping=true; vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        isSleeping=false;
        if(wasSleeping) { size_t dummyBytes; int32_t flushBuf[HOP_SIZE*2]; ulTaskNotifyTake(pdTRUE, 0); while(i2s_channel_read(rx_chan,flushBuf,sizeof(flushBuf),&dummyBytes,0)==ESP_OK && dummyBytes>0) {} wasSleeping=false; }
        
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)); size_t bytesRead; i2s_channel_read(rx_chan, i2s_in_block, sizeof(i2s_in_block), &bytesRead, pdMS_TO_TICKS(10));
        if(bytesRead>0) {
            int framesRead=bytesRead/8; framesRead&=~3;
            if(framesRead>0) {
                if(panicResetRequested) {
                    synthEnv=0.0f; synthFilter=0.0f; padFilter=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate; fbDelayWriteIdx=0; apfNeedsClear=true; freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f;
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16; tap_w1_1=0; tap_w1_2=halfWinFixed; tap_w2_1=0; tap_w2_2=halfWinFixed; tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    memset(delayBuffer,0,MAX_BUFFER_SIZE*sizeof(int16_t)); memset(fbDelayBuffer,0,FB_BUFFER_SIZE*sizeof(int16_t)); memset(freezeBuffer,0,FREEZE_BUFFER_SIZE*sizeof(int16_t)); panicResetRequested=false;
                }
                if(globalAudioResetRequested) {
                    synthEnv=0.0f; synthFilter=0.0f; padFilter=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; smoothedVolGain=volumePedalGain; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate; fbDelayWriteIdx=0; writeIndex=0; apfNeedsClear=true; input_dc_offset=0.0f; ui_audio_level=0.0f; ui_output_level=0.0f; freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f;
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16; tap_w1_1=0; tap_w1_2=halfWinFixed; tap_w2_1=0; tap_w2_2=halfWinFixed; tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    globalAudioResetRequested=false; smoothed_delay_samples=0.0f; if(hardwareSyncMuteFrames<10) hardwareSyncMuteFrames=(currentSampleRate/HOP_SIZE)*0.15f;
                }
                if(hardwareSyncMuteFrames>0) { hardwareSyncMuteFrames--; memset(i2s_out_block,0,framesRead*2*sizeof(int32_t)); ui_audio_level=0.0f; ui_output_level=0.0f; for(int i=0; i<framesRead; i++) { int32_t clean_sample=i2s_in_block[i*2]&0xFFFFFF00; float raw_in=((float)clean_sample*normFactor); input_dc_offset=(input_dc_offset*0.95f)+(raw_in*0.05f); } size_t bytesWrittenCount; i2s_channel_write(tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20)); continue; }
                uint32_t start_cycles=xthal_get_ccount(); float srScale=48000.0f/(float)currentSampleRate;
                if(audioBufferMutex!=NULL && xSemaphoreTake(audioBufferMutex, 0)==pdTRUE) { for(int j=0; j<10; j++) for(int k=0; k<5; k++) c_fx[j][k]=fxParams[j][k]; c_lat=latencyMode; c_act=activeEffectMode; c_w=isWhammyActive; c_fz=isFrozen; c_fb=isFeedbackActive; c_hr=isHarmonizerMode; c_cp=isCapoMode; c_sy=isSynthMode; c_pd=isPadMode; c_ch=isChorusMode; c_sw=isSwellMode; c_vb=isVibratoMode; c_pt=pitchShiftFactor; c_vg=volumePedalGain; xSemaphoreGive(audioBufferMutex); }
                float targetWindow=LATENCY_WINDOWS[c_lat]; if(currentWindowSize!=targetWindow) { currentWindowSize=targetWindow; uint32_t halfWindowFixed=((uint32_t)targetWindow/2)<<16; tap_w1_1=0; tap_w1_2=halfWindowFixed; tap_w2_1=0; tap_w2_2=halfWindowFixed; tap_w3_1=0; tap_w3_2=halfWindowFixed; tap_w4_1=0; tap_w4_2=halfWindowFixed; tap_w5_1=0; tap_w5_2=halfWindowFixed; }
                uint32_t hannIntMult=(1024U<<16)/(uint32_t)currentWindowSize, windowMask=(uint32_t)currentWindowSize-1; float p_w_dry=c_fx[0][0], p_w_wet=c_fx[0][1], p_fz_apf=c_fx[1][0], p_fz_att=c_fx[1][1], p_fz_rel=c_fx[1][2], p_fb_spd=c_fx[2][0], p_fb_drv=c_fx[2][1], p_fb_off=c_fx[2][2], p_hr_mix=c_fx[3][0], p_sy_att=c_fx[5][0], p_sy_rel=c_fx[5][1], p_sy_flt=c_fx[5][2], p_sy_mix=c_fx[5][3], p_pd_sm=c_fx[6][0], p_pd_mix=c_fx[6][1], p_ch_spd=c_fx[7][0], p_ch_mix=c_fx[7][1], p_sw_thr=c_fx[8][0], p_sw_att=c_fx[8][1], p_sw_rel=c_fx[8][2], p_vb_dep=c_fx[9][0]; float chorusPhaseIncr=p_ch_spd/(float)currentSampleRate, feedbackPhaseIncr=p_fb_spd/(float)currentSampleRate, targetPitch=c_pt, pitchInc=(targetPitch-currentPitch)/(float)framesRead;
                bool frzActive=((c_act==1&&c_w)||c_fz);
                if(frzActive && !wasFrozen) { freezePlayCounterVar=0; int bestStart=freezeWriteIdxVar, tempIdx=freezeWriteIdxVar; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { bestStart=tempIdx; break; } tempIdx=prev; } freezeStartIdxVar=bestStart; activeFreezeLength=freezeLength; int searchEnd=bestStart-1; if(searchEnd<0) searchEnd+=freezeLength; tempIdx=searchEnd; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { activeFreezeLength=s; break; } tempIdx=prev; } if(activeFreezeLength<64) activeFreezeLength=freezeLength; }
                if(!frzActive && wasFrozen) apfNeedsClear=true; wasFrozen=frzActive; float activeInvFreqLength=1.0f/(float)activeFreezeLength; bool synthActive=((c_act==5&&c_w)||c_sy), padActive=((c_act==6&&c_w)||c_pd), harmActive=((c_act==3&&c_w)||c_hr), swellActive=((c_act==8&&c_w)||c_sw), chorusActive=((c_act==7&&c_w)||c_ch), feedbackActive=((c_act==2&&c_w)||c_fb);
                if(feedbackActive && !wasFeedbackActive) { fbOutNode=0.0f; fbHpfState=0.0f; feedbackFilterVar=0.0f; } wasFeedbackActive=feedbackActive; bool vibratoActive=((c_act==9&&c_w)||c_vb), capoActive=((c_act==4&&c_w)||c_cp);
                float peakInputVal=0.0f, peakOutputVal=0.0f, localSwellGain=swellGain, localVolGain=c_vg, localFrzRamp=freezeRamp, localFbRamp=feedbackRamp, pdSmCoeff=powf(p_pd_sm, srScale), target_delay=constrain((float)(currentSampleRate*p_fb_off), 0.0f, (float)(FB_BUFFER_SIZE-1)); smoothed_delay_samples+= (target_delay-smoothed_delay_samples)*0.01f*srScale+DC_OFFSET; int delaySamples=(int)smoothed_delay_samples; float fbHpfCoeff=(currentSampleRate==96000)?0.025f:0.05f, fbLpfCoeff=(currentSampleRate==96000)?0.05f:0.1f, fbLpfRetain=1.0f-fbLpfCoeff, dc_alpha=(currentSampleRate==96000)?0.0005f:0.001f; int halfWindow=(int)currentWindowSize/2; bool activeGroup=c_w||harmActive||chorusActive||feedbackActive||synthActive||padActive||frzActive||vibratoActive||capoActive, dryGroup=chorusActive||padActive||frzActive||feedbackActive||(localFrzRamp>0.0f)||(localFbRamp>0.0f), repeatGroup=capoActive||synthActive||vibratoActive||padActive||harmActive;
                float g_base=0.0f; if(dryGroup) { if(!repeatGroup) g_base=0.4f; } else if(harmActive) g_base=0.5f; else g_base=1.0f; float g_w2=harmActive?p_hr_mix:0.0f, g_w3=chorusActive?p_ch_mix:0.0f; bool padIsAudible=padActive||(fabsf(padFilter)>0.001f); float g_pad=padIsAudible?p_pd_mix:0.0f, g_frz=(!frzActive&&localFrzRamp>0.0f)?0.5f:0.0f, g_fb=(feedbackActive||localFbRamp>0.0f)?0.6f:0.0f, g_whammy=c_w?p_w_wet:0.0f, g_dry=c_w?p_w_dry:1.0f, vol_alpha=0.01f*srScale, meter_decay=(currentSampleRate==96000)?0.999f:0.998f, envRetain=powf(0.99f,srScale), envAttack=1.0f-envRetain;
                if(isnan(synthFilter)||isinf(synthFilter)) synthFilter=0.0f; if(isnan(padFilter)||isinf(padFilter)) padFilter=0.0f; if(isnan(feedbackFilterVar)||isinf(feedbackFilterVar)) feedbackFilterVar=0.0f; if(isnan(fbHpfState)||isinf(fbHpfState)) fbHpfState=0.0f; float localVibPhase=vibratoLfoPhase, localChoPhase=chorusLfoPhase, localFbPhase=feedbackLfoPhase, localFbHpf=fbHpfState; int prefetchIdxW1=(writeIndex-halfWindow+MAX_BUFFER_SIZE)&BUFFER_MASK, prefetchIdxFB=(fbDelayWriteIdx-delaySamples+FB_BUFFER_SIZE)&FB_BUFFER_MASK; __builtin_prefetch(&delayBuffer[prefetchIdxW1], 0, 3); __builtin_prefetch(&fbDelayBuffer[prefetchIdxFB], 0, 3);
                
                for(int i=0; i<framesRead; i++) {
                    currentPitch+=pitchInc; int32_t clean_sample=i2s_in_block[i*2]&0xFFFFFF00; float raw_in=((float)clean_sample*normFactor); input_dc_offset=(input_dc_offset*(1.0f-dc_alpha))+(raw_in*dc_alpha); float inSample=raw_in-input_dc_offset; inputEnvelope=inputEnvelope*envRetain+fabsf(inSample)*envAttack+DC_OFFSET; envBuf[i]=inputEnvelope;
                    if(swellActive) localSwellGain=(inputEnvelope>p_sw_thr)?__builtin_fminf(1.0f, localSwellGain+(p_sw_att*srScale)):__builtin_fmaxf(0.0f, localSwellGain-(p_sw_rel*srScale)); else localSwellGain=__builtin_fminf(1.0f, localSwellGain+(0.005f*srScale));
                    smoothedVolGain=smoothedVolGain*(1.0f-vol_alpha)+localVolGain*vol_alpha+DC_OFFSET; masterGainBuf[i]=2147483520.0f*localSwellGain*smoothedVolGain; inBuf[i]=inSample; fzOutBuf[i]=0.0f;
                }
                if(synthActive) for(int i=0; i<framesRead; i++) { synthEnv=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,synthEnv+(p_sy_att*srScale)):__builtin_fmaxf(0.0f,synthEnv-(p_sy_rel*srScale)); float clampedProc=__builtin_fmaxf(-1.0f,__builtin_fminf(inBuf[i],1.0f)); int waveIdx=(int)((clampedProc+1.0f)*1023.5f); float procSample=synthLUT[waveIdx]; float fltCoeff=__builtin_fmaxf(0.001f,__builtin_fminf(0.99f,(p_sy_flt+0.6f*synthEnv)*srScale)); synthFilter=synthFilter+fltCoeff*(procSample-synthFilter)+DC_OFFSET; inBuf[i]=synthFilter*p_sy_mix; }
                if(padActive) for(int i=0; i<framesRead; i++) { padEnv=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,padEnv+(0.00002f*srScale)):__builtin_fmaxf(0.0f,padEnv-(0.000005f*srScale)); inBuf[i]*=padEnv; }
                
                for(int i=0; i<framesRead; i++) {
                    float procSample=inBuf[i]; if(!frzActive) { freezeBuffer[freezeWriteIdxVar]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(procSample,1.0f))*32767.0f); freezeWriteIdxVar++; if(freezeWriteIdxVar>=freezeLength) freezeWriteIdxVar=0; }
                    if(localFrzRamp>0.0f||frzActive) localFrzRamp=frzActive?__builtin_fminf(1.0f,localFrzRamp+(p_fz_att*srScale)):__builtin_fmaxf(0.0f,localFrzRamp-(p_fz_rel*srScale));
                    if(localFrzRamp>0.0f) {
                        float phaseRead=(float)freezePlayCounterVar*activeInvFreqLength, phase2=(phaseRead+0.5f); if(phase2>=1.0f) phase2-=1.0f;
                        int sum1=freezeStartIdxVar+freezePlayCounterVar, idx1=(freezeLength>0)?(sum1%freezeLength):0, activeLen=(activeFreezeLength>=64)?activeFreezeLength:freezeLength, counter2=(activeLen>0)?((freezePlayCounterVar+(activeLen/2))%activeLen):0, sum2=freezeStartIdxVar+counter2, idx2=(freezeLength>0)?(sum2%freezeLength):0, lutIdx1=(int)(phaseRead*1023.0f)&1023, lutIdx2=(int)(phase2*1023.0f)&1023;
                        float rFrz=((float)freezeBuffer[idx1]*3.0517578125e-5f*hannLUT[lutIdx1])+((float)freezeBuffer[idx2]*3.0517578125e-5f*hannLUT[lutIdx2]);
                        float d1=apf1Buffer[apf1Idx], next_apf1=rFrz+p_fz_apf*d1+DC_OFFSET, a1=-p_fz_apf*rFrz+d1; apf1Buffer[apf1Idx]=next_apf1; apf1Idx++; if(apf1Idx>=1009) apf1Idx=0; float d2=apf2Buffer[apf2Idx], next_apf2=a1+p_fz_apf*d2+DC_OFFSET, a2=-p_fz_apf*a1+d2; apf2Buffer[apf2Idx]=next_apf2; apf2Idx++; if(apf2Idx>=863) apf2Idx=0; fzOutBuf[i]=a2*localFrzRamp; freezePlayCounterVar++; if(freezePlayCounterVar>=activeFreezeLength) freezePlayCounterVar=0;
                    } else if(apfNeedsClear) { memset(apf1Buffer,0,sizeof(apf1Buffer)); memset(apf2Buffer,0,sizeof(apf2Buffer)); apf1Idx=0; apf2Idx=0; apfNeedsClear=false; }
                    
                    float delayIn=(localFrzRamp>0.0f)?(procSample*(1.0f-localFrzRamp))+fzOutBuf[i]:procSample; delayBuffer[writeIndex]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(delayIn,1.0f))*32767.0f);
                    float spd1=currentPitch; if(vibratoActive) { localVibPhase+=globalVibratoPhaseInc; if(localVibPhase>=LFO_LUT_SIZE) localVibPhase-=LFO_LUT_SIZE; spd1*=1.0f+((lfoLUT[(int)localVibPhase&1023]-1.0f)*p_vb_dep); }
                    float spd2=currentPitch*globalHarmRatio, spd3=currentPitch*globalChorusRatio; if(chorusActive) { localChoPhase+=chorusPhaseIncr; if(localChoPhase>=LFO_LUT_SIZE) localChoPhase-=LFO_LUT_SIZE; spd3*=lfoLUT[(int)localChoPhase&1023]; } float spd4=1.0f, spd5=1.0f;
                    
                    if(feedbackActive||localFbRamp>0.0f) {
                        localFbPhase+=feedbackPhaseIncr; if(localFbPhase>=LFO_LUT_SIZE) localFbPhase-=LFO_LUT_SIZE; float lfoVal=lfoLUT[(int)localFbPhase&1023]; spd4=lfoVal; spd5=currentPitch*globalFbRatio*lfoVal;
                        float w4=processTap(tap_w4_1,delayBuffer,writeIndex,windowMask,hannIntMult)+processTap(tap_w4_2,delayBuffer,writeIndex,windowMask,hannIntMult), w5=processTap(tap_w5_1,delayBuffer,writeIndex,windowMask,hannIntMult)+processTap(tap_w5_2,delayBuffer,writeIndex,windowMask,hannIntMult);
                        if(feedbackActive) localFbRamp=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,localFbRamp+(0.000011f*srScale)):__builtin_fmaxf(0.0f,localFbRamp-(0.005f*srScale)); else localFbRamp=__builtin_fmaxf(0.0f,localFbRamp-(0.0001f*srScale));
                        float mixV=__builtin_fmaxf(0.0f,__builtin_fminf((localFbRamp-0.1f)*2.0f,1.0f)), feedInput=(frzActive&&localFrzRamp>0.0f)?fzOutBuf[i]:(w4*(1.0f-mixV))+(w5*mixV)+(fbOutNode*0.95f); localFbHpf+=fbHpfCoeff*(feedInput-localFbHpf)+DC_OFFSET; float rawDrive=(feedInput-localFbHpf)*p_fb_drv, boundedDrive=__builtin_fmaxf(-1.5f,__builtin_fminf(rawDrive,1.5f)), gainDrive=boundedDrive*(1.0f-(0.15f*boundedDrive*boundedDrive)); feedbackFilterVar=feedbackFilterVar*fbLpfRetain+gainDrive*fbLpfCoeff+DC_OFFSET; float satFb=feedbackFilterVar*(localFbRamp*localFbRamp*localFbRamp)*0.85f; fbDelayBuffer[fbDelayWriteIdx]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(satFb,1.0f))*32767.0f);
                        int fbReadIdx=(fbDelayWriteIdx-delaySamples+FB_BUFFER_SIZE)&FB_BUFFER_MASK; fbOutNode=(float)fbDelayBuffer[fbReadIdx]*3.0517578125e-5f; fbDelayWriteIdx=(fbDelayWriteIdx+1)&FB_BUFFER_MASK;
                    } else { fbDelayBuffer[fbDelayWriteIdx]=0; fbOutNode=0.0f; fbDelayWriteIdx=(fbDelayWriteIdx+1)&FB_BUFFER_MASK; }
                    
                    w1Buf[i]=processTap(tap_w1_1,delayBuffer,writeIndex,windowMask,hannIntMult)+processTap(tap_w1_2,delayBuffer,writeIndex,windowMask,hannIntMult); w2Buf[i]=0.0f; if(harmActive) w2Buf[i]=processTap(tap_w2_1,delayBuffer,writeIndex,windowMask,hannIntMult)+processTap(tap_w2_2,delayBuffer,writeIndex,windowMask,hannIntMult); w3Buf[i]=0.0f; if(chorusActive) w3Buf[i]=processTap(tap_w3_1,delayBuffer,writeIndex,windowMask,hannIntMult)+processTap(tap_w3_2,delayBuffer,writeIndex,windowMask,hannIntMult);
                    int32_t step1=(int32_t)((1.0f-spd1)*65536.0f); tap_w1_1+=step1; tap_w1_2+=step1; int32_t step2=(int32_t)((1.0f-spd2)*65536.0f); tap_w2_1+=step2; tap_w2_2+=step2; int32_t step3=(int32_t)((1.0f-spd3)*65536.0f); tap_w3_1+=step3; tap_w3_2+=step3; int32_t step4=(int32_t)((1.0f-spd4)*65536.0f); tap_w4_1+=step4; tap_w4_2+=step4; int32_t step5=(int32_t)((1.0f-spd5)*65536.0f); tap_w5_1+=step5; tap_w5_2+=step5;
                    if(padActive) padFilter=padFilter*pdSmCoeff+w1Buf[i]*(1.0f-pdSmCoeff)+DC_OFFSET; else padFilter=padFilter*pdSmCoeff+DC_OFFSET; int dryIdx=(writeIndex-halfWindow+MAX_BUFFER_SIZE)&BUFFER_MASK; dryBuf[i]=(float)delayBuffer[dryIdx]*3.0517578125e-5f; padFilterBuf[i]=padFilter; fbOutBuf[i]=fbOutNode; writeIndex=(writeIndex+1)&BUFFER_MASK;
                }
                vibratoLfoPhase=localVibPhase; chorusLfoPhase=localChoPhase; feedbackLfoPhase=localFbPhase; fbHpfState=localFbHpf;
                
                #pragma GCC ivdep
                for(int i=0; i<framesRead; i++) { float sMix=DC_OFFSET; if(!activeGroup&&localFrzRamp<=0.0f&&localFbRamp<=0.0f&&padFilterBuf[i]<=0.001f) { sMix=dryBuf[i]; } else { float baseSignal=(w1Buf[i]*g_whammy)+(dryBuf[i]*g_dry); sMix+=baseSignal*g_base+w2Buf[i]*g_w2+w3Buf[i]*g_w3+padFilterBuf[i]*g_pad+fzOutBuf[i]*g_frz+fbOutBuf[i]*g_fb; sMix=__builtin_fmaxf(-1.8f,__builtin_fminf(sMix,1.8f)); sMix=sMix*(1.0f-(0.1f*sMix*sMix))*0.82f; } sMixBuf[i]=sMix; }
                dsps_mul_f32(sMixBuf, masterGainBuf, sMixBuf, framesRead, 1, 1, 1);
                
                for(int i=0; i<framesRead; i++) { if(fabsf(inBuf[i])>peakInputVal) peakInputVal=fabsf(inBuf[i]); float rawOut=sMixBuf[i]*(1.0f/2147483520.0f); if(fabsf(rawOut)>peakOutputVal) peakOutputVal=fabsf(rawOut); int32_t finalOut=(int32_t)__builtin_fmaxf(-2147483520.0f,__builtin_fminf(sMixBuf[i],2147483520.0f)); finalOut&=0xFFFFFF00; i2s_out_block[i*2]=finalOut; i2s_out_block[i*2+1]=finalOut; }
                swellGain=localSwellGain; freezeRamp=localFrzRamp; feedbackRamp=localFbRamp;
                
                if(peakInputVal>ui_audio_level) ui_audio_level=peakInputVal; else { ui_audio_level*=meter_decay; if(ui_audio_level<1e-5f) ui_audio_level=0.0f; }
                if(peakOutputVal>ui_output_level) ui_output_level=peakOutputVal; else { ui_output_level*=meter_decay; if(ui_output_level<1e-5f) ui_output_level=0.0f; }
                uint32_t end_timer=xthal_get_ccount(); float max_cycles=(240000000.0f/(float)currentSampleRate)*(float)framesRead; float currentLoadPercentage=((float)(end_timer-start_cycles)/max_cycles)*100.0f; core1_load=core1_load*0.95f+__builtin_fminf(100.0f,currentLoadPercentage)*0.05f;
                
                size_t bytesWrittenCount; i2s_channel_write(tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20));
            }
        }
    }
}

void updateParameterFromCC(uint8_t cc, uint8_t val) {
    float norm=(float)val/127.0f; int pIdx=cc-24;
    if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    if(activeEffectMode==0) { if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==2) fxParams[0][0]=norm; if(pIdx==3) fxParams[0][1]=norm; }
    else if(activeEffectMode==1) { if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==2) fxParams[1][0]=0.0f+(norm*0.95f); if(pIdx==3) fxParams[1][1]=0.00001f+(norm*0.001f); if(pIdx==4) fxParams[1][2]=0.00001f+(norm*0.0005f); }
    else if(activeEffectMode==2) { if(pIdx==0) { int newIdx=constrain((int)roundf(norm*4.0f),0,4); if(newIdx!=feedbackIntervalIdx) { feedbackIntervalIdx=newIdx; lutNeedsUpdate=true; forceUIUpdate=true; } } if(pIdx==1) fxParams[2][0]=1000.0f+(norm*10000.0f); if(pIdx==2) fxParams[2][1]=1.0f+(norm*100.0f); if(pIdx==3) fxParams[2][2]=0.005f+(norm*0.045f); }
    else if(activeEffectMode==3) { if(pIdx==0) { effectMemory[3]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) fxParams[3][0]=norm; }
    else if(activeEffectMode==4) { if(pIdx==0) { int cents=(int)roundf((effectMemory[4]-(float)roundf(effectMemory[4]))*100.0f); effectMemory[4]=constrain(roundf((norm*48.0f)-24.0f)+((float)cents/100.0f),-24.0f,24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) { int semi=(int)roundf(effectMemory[4]); float c=roundf((norm*100.0f)-50.0f)/100.0f; effectMemory[4]=constrain((float)semi+c,-24.0f,24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } }
    else if(activeEffectMode==5) { if(pIdx==0) { effectMemory[5]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) fxParams[5][0]=0.01f+(norm*0.5f); if(pIdx==2) fxParams[5][1]=0.001f+(norm*0.05f); if(pIdx==3) fxParams[5][2]=0.1f+(norm*0.8f); if(pIdx==4) fxParams[5][3]=norm; }
    else if(activeEffectMode==6) { if(pIdx==0) { effectMemory[6]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) fxParams[6][0]=0.8f+(norm*0.199f); if(pIdx==2) fxParams[6][1]=norm*3.0f; }
    else if(activeEffectMode==7) { if(pIdx==0) { effectMemory[7]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) fxParams[7][0]=500.0f+(norm*4500.0f); if(pIdx==2) fxParams[7][1]=norm; }
    else if(activeEffectMode==8) { if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==2) fxParams[8][0]=0.001f+(norm*0.05f); if(pIdx==3) fxParams[8][1]=0.00001f+(norm*0.001f); if(pIdx==4) fxParams[8][2]=0.00001f+(norm*0.0005f); }
    else if(activeEffectMode==9) { if(pIdx==0) { effectMemory[9]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; forceUIUpdate=true; } if(pIdx==1) fxParams[9][0]=norm*2.0f; }
    if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex);
    settingsNeedSaving=true; lastParameterChangeTime=millis();
}

void MidiTask(void * pvParameters) {
    pedals.resetToCenter(); static bool lastBtState=false; static uint8_t lastVolumeCC=127;
    #if ENABLE_PAR_KNOBS
        static int lastCcOut[5]={-1,-1,-1,-1,-1};
    #endif
    static unsigned long lastBatteryTime=0; static float smoothedRawBat=0.0f;
    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP); lastActivityTime=millis(); lastScreenActivityTime=millis();
    
    for(;;) {
        Control_Surface.loop(); bool currentBtState=btmidi.isConnected();
        if(currentBtState!=lastBtState) { lastBtState=currentBtState; forceUIUpdate=true; if(isScreenOff) turnScreenOn(); lastActivityTime=millis(); lastScreenActivityTime=millis(); }
        if(!currentBtState && (millis()-lastActivityTime>LIGHT_SLEEP_TIMEOUT)) goToLightSleep();
        if(!isScreenOff && (millis()-lastScreenActivityTime>SCREEN_OFF_TIMEOUT)) turnScreenOff();
        
        #if ENABLE_PAR_KNOBS
            if(filterPar1.update()) { int cc1=map(filterPar1.getValue(),0,4095,0,127); if(cc1!=lastCcOut[0]) { Control_Surface.sendControlChange({24,Channel_1},cc1); updateParameterFromCC(24,cc1); lastCcOut[0]=cc1; } }
            if(filterPar2.update()) { int cc2=map(filterPar2.getValue(),0,4095,0,127); if(cc2!=lastCcOut[1]) { Control_Surface.sendControlChange({25,Channel_1},cc2); updateParameterFromCC(25,cc2); lastCcOut[1]=cc2; } }
            if(filterPar3.update()) { int cc3=map(filterPar3.getValue(),0,4095,0,127); if(cc3!=lastCcOut[2]) { Control_Surface.sendControlChange({26,Channel_1},cc3); updateParameterFromCC(26,cc3); lastCcOut[2]=cc3; } }
            if(filterPar4.update()) { int cc4=map(filterPar4.getValue(),0,4095,0,127); if(cc4!=lastCcOut[3]) { Control_Surface.sendControlChange({27,Channel_1},cc4); updateParameterFromCC(27,cc4); lastCcOut[3]=cc4; } }
            if(filterPar5.update()) { int cc5=map(filterPar5.getValue(),0,4095,0,127); if(cc5!=lastCcOut[4]) { Control_Surface.sendControlChange({28,Channel_1},cc5); updateParameterFromCC(28,cc5); lastCcOut[4]=cc5; } }
        #endif
        
        fetchADCDMA();
        if(digitalRead(BOOT_SENSE_PIN)==HIGH) {
            pedals.process(latestPB1, latestPB2, latestPB3, isVolumeMode, INVERT_PB3);
            int calA=pedals.getCalA(), calB=pedals.getCalB(), calC=pedals.getCalC(); bool moveA=pedals.hasMovedA(), moveB=pedals.hasMovedB(), moveC=pedals.hasMovedC();
            if(moveA||moveB||moveC) {
                if(isScreenOff) turnScreenOn(); lastScreenActivityTime=millis(); lastActivityTime=millis();
                if(moveA) { Control_Surface.sendPitchBend(Channel_1, calA); pedals.updateLastMidiA(); currentPB1=calA; }
                if(moveB) { Control_Surface.sendPitchBend(Channel_2, calB); pedals.updateLastMidiB(); currentPB2=calB; }
                if(moveC) { if(!isVolumeMode) Control_Surface.sendPitchBend(Channel_3, calC); pedals.updateLastMidiC(); currentPB3=calC; }
                bool pitchChanged=false;
                if(moveA) { lastActivePedal=calA; pitchChanged=true; }
                if(moveB) { lastActivePedal=calB; pitchChanged=true; }
                if(moveC&&!isVolumeMode) { lastActivePedal=calC; pitchChanged=true; }
                if(pitchChanged) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); pitchShiftFactor=pitchShiftLUT[constrain(lastActivePedal,0,16383)]; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); }
                if(moveC&&isVolumeMode) { uint8_t vCC=map(calC,0,16383,0,127); if(vCC!=lastVolumeCC) { Control_Surface.sendControlChange({19,Channel_1},vCC); lastVolumeCC=vCC; } if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); volumePedalGain=(float)calC/16383.0f; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); }
                forceUIUpdate=true;
            }
        } else {
            static bool wasBypassed=false;
            if(!wasBypassed) { pedals.resetToCenter(); lastActivePedal=8192; if(!lutNeedsUpdate) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); pitchShiftFactor=pitchShiftLUT[8192]; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); } if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); if(isVolumeMode) volumePedalGain=1.0f; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; wasBypassed=true; }
        }
        
        smoothedRawBat=(smoothedRawBat==0.0f)?(float)latestBat:(smoothedRawBat*0.95f)+((float)latestBat*0.05f);
        if(millis()-lastBatteryTime>1000) { lastBatteryTime=millis(); const float CALIBRATION_MULTIPLIER=1.04571f; float instantVoltage=(smoothedRawBat/4095.0f)*3.3f*2.0f*CALIBRATION_MULTIPLIER; if(instantVoltage>2.0f) { bool charging=(instantVoltage>4.20f); int newPercent=getBatteryPercentage(instantVoltage); bool stateChanged=(newPercent!=currentBatteryPercent)||(charging!=isBatteryCharging); currentBatteryVoltage=instantVoltage; currentBatteryPercent=newPercent; isBatteryCharging=charging; if(stateChanged) forceUIUpdate=true; } }
        
        static unsigned long lastLutUpdate=0;
        if(lutNeedsUpdate && (millis()-lastLutUpdate>40)) { lutNeedsUpdate=false; updateLUT(); if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); pitchShiftFactor=pitchShiftLUT[constrain(lastActivePedal,0,16383)]; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); lastLutUpdate=millis(); }
        if(settingsNeedSaving && (millis()-lastParameterChangeTime>2000)) { if(ui_audio_level<0.01f || (millis()-lastParameterChangeTime>10000)) { settingsNeedSaving=false; sleepRequested=true; int timeoutCounter=0; while(!isSleeping && timeoutCounter<40) { vTaskDelay(pdMS_TO_TICKS(5)); timeoutCounter++; } if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); i2s_channel_disable(tx_chan); i2s_channel_disable(rx_chan); if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); saveSettings(); if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); sleepRequested=false; } }
        if(sampleRateToggleRequested) { sampleRateToggleRequested=false; toggleSampleRate(); }
        if(pb2ToggleRequested) { pb2ToggleRequested=false; calibratePBs(); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool channelMessageCallback(ChannelMessage cm) {
    if(isScreenOff) turnScreenOn(); lastActivityTime=millis(); lastScreenActivityTime=millis();
    if(cm.header==0xB0) {
        if(cm.data1==11) { uint16_t mappedCC=map(cm.data2,0,127,0,16383); currentCC11=mappedCC; currentPB3=mappedCC; lastActivePedal=mappedCC; if(isVolumeMode) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); volumePedalGain=(float)mappedCC/16383.0f; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); Control_Surface.sendControlChange({19,Channel_1},cm.data2); } else { if(!lutNeedsUpdate) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); pitchShiftFactor=pitchShiftLUT[mappedCC]; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); } } forceUIUpdate=true; return false; }
        if(cm.data1>=24 && cm.data1<=28) { updateParameterFromCC(cm.data1, cm.data2); return false; }
        if(cm.data1==5 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isPB2WiperMode=!isPB2WiperMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); pb2ToggleRequested=true; }
        else if(cm.data1==6 && cm.data2>=64) { bool sendCenterMidi=false; if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isVolumeMode=!isVolumeMode; if(!isVolumeMode) { volumePedalGain=1.0f; pedals.lockPB3Whammy(); sendCenterMidi=true; currentPB3=8192; lastActivePedal=8192; if(!lutNeedsUpdate && pitchShiftLUT!=nullptr) pitchShiftFactor=pitchShiftLUT[8192]; } else { pedals.lockPB3Volume(); lastActivePedal=8192; volumePedalGain=1.0f; if(!lutNeedsUpdate && pitchShiftLUT!=nullptr) pitchShiftFactor=pitchShiftLUT[8192]; } if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); if(sendCenterMidi) Control_Surface.sendPitchBend(Channel_3, 8192); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==0 && cm.data2>=64) switchEffectMode(activeEffectMode-1);
        else if(cm.data1==1 && cm.data2>=64) switchEffectMode(activeEffectMode+1);
        else if(cm.data1==2 && cm.data2>=64) sampleRateToggleRequested=true;
        else if(cm.data1==3 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); latencyMode=(latencyMode+1)%4; globalAudioResetRequested=true; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==4 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); bool anyEffectOn=isWhammyActive||isFrozen||isFeedbackActive||isHarmonizerMode||isCapoMode||isSynthMode||isPadMode||isChorusMode||isSwellMode||isVibratoMode; if(anyEffectOn||activeEffectMode!=0) { isWhammyActive=false; isFrozen=false; isFeedbackActive=false; isHarmonizerMode=false; isCapoMode=false; isSynthMode=false; isPadMode=false; isChorusMode=false; isSwellMode=false; isVibratoMode=false; activeEffectMode=0; panicResetRequested=true; } else { isWhammyActive=true; } if(isVolumeMode) { isVolumeMode=false; pedals.lockPB3Whammy(); } volumePedalGain=1.0f; currentPB1=8192; currentPB2=8192; currentPB3=8192; lastActivePedal=8192; if(!lutNeedsUpdate && pitchShiftLUT!=nullptr) pitchShiftFactor=pitchShiftLUT[8192]; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); Control_Surface.sendPitchBend(Channel_1, 8192); Control_Surface.sendPitchBend(Channel_2, 8192); Control_Surface.sendPitchBend(Channel_3, 8192); lutNeedsUpdate=true; forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==8 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isFrozen=!isFrozen; if(activeEffectMode==1) isWhammyActive=isFrozen; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==9 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isFeedbackActive=!isFeedbackActive; if(activeEffectMode==2) isWhammyActive=isFeedbackActive; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==10 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isHarmonizerMode=!isHarmonizerMode; if(activeEffectMode==3) isWhammyActive=isHarmonizerMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==12 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isCapoMode=!isCapoMode; if(activeEffectMode==4) isWhammyActive=isCapoMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); lutNeedsUpdate=true; forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==13 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isSynthMode=!isSynthMode; if(activeEffectMode==5) isWhammyActive=isSynthMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==14 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isPadMode=!isPadMode; if(activeEffectMode==6) isWhammyActive=isPadMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==15 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isChorusMode=!isChorusMode; if(activeEffectMode==7) isWhammyActive=isChorusMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==16 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isSwellMode=!isSwellMode; if(activeEffectMode==8) isWhammyActive=isSwellMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==7 && cm.data2>=64) { if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); isVibratoMode=!isVibratoMode; if(activeEffectMode==9) isWhammyActive=isVibratoMode; if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
        else if(cm.data1==17 || cm.data1==18) { float step=(cm.data2>=64)?-1.0f:1.0f; if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY); if(cm.data1==17) { if(activeEffectMode==0||activeEffectMode==1||activeEffectMode==8) { effectMemory[1]=constrain(effectMemory[1]+step,-24.0f,24.0f); } else if(activeEffectMode==4) { effectMemory[4]=constrain(effectMemory[4]+step,-24.0f,24.0f); } else if(activeEffectMode==2) { if(step>0) feedbackIntervalIdx=(feedbackIntervalIdx+1)%5; else feedbackIntervalIdx=(feedbackIntervalIdx+4)%5; } else { effectMemory[activeEffectMode]=constrain(effectMemory[activeEffectMode]+step,-24.0f,24.0f); } } else if(cm.data1==18) { if(activeEffectMode==0||activeEffectMode==1||activeEffectMode==8) { effectMemory[0]=constrain(effectMemory[0]+step,-24.0f,24.0f); } else if(activeEffectMode==4) { float centStep=step*0.01f; effectMemory[4]=constrain(effectMemory[4]+centStep,-24.0f,24.0f); } } if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex); lutNeedsUpdate=true; forceUIUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis(); }
    }
    return false;
}

void setup() {
    esp_brownout_init(); audioBufferMutex=xSemaphoreCreateMutex(); WiFi.mode(WIFI_OFF);
    adc_continuous_handle_cfg_t adc_config={}; adc_config.max_store_buf_size=16384; adc_config.conv_frame_size=1024;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &multifx_adc_handle));
    adc_continuous_config_t dig_cfg={}; dig_cfg.sample_freq_hz=20*1000; dig_cfg.conv_mode=ADC_CONV_SINGLE_UNIT_1; dig_cfg.format=ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    adc_digi_pattern_config_t adc_pattern[4]={ {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_0,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_1,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_9,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_3,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH} };
    dig_cfg.pattern_num=4; dig_cfg.adc_pattern=adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(multifx_adc_handle, &dig_cfg)); ESP_ERROR_CHECK(adc_continuous_start(multifx_adc_handle));
    
    preferences.begin("whammy_cfg", true); activeEffectMode=0; preferences.getInt("activeMode", 0);
    latencyMode=constrain(preferences.getInt("latMode", 0), 0, 3); isPB2WiperMode=preferences.getBool("pb2Wiper", false); isVolumeMode=false; currentSampleRate=preferences.getUInt("sampleRate", 48000); feedbackIntervalIdx=constrain(preferences.getInt("fbIdx", 0), 0, 4);
    AppSettings savedSettings; size_t len=preferences.getBytes("dspData", &savedSettings, sizeof(AppSettings));
    if(len==sizeof(AppSettings)) { for(int i=0; i<10; i++) { effectMemory[i]=savedSettings.fxMem[i]; for(int p=0; p<5; p++) fxParams[i][p]=savedSettings.params[i][p]; } }
    uint16_t fxStates=preferences.getUShort("fxStates", 1);
    isWhammyActive=true; isFrozen=(fxStates&(1<<1))!=0; isFeedbackActive=(fxStates&(1<<2))!=0; isHarmonizerMode=(fxStates&(1<<3))!=0; isCapoMode=(fxStates&(1<<4))!=0; isSynthMode=(fxStates&(1<<5))!=0; isPadMode=(fxStates&(1<<6))!=0; isChorusMode=(fxStates&(1<<7))!=0; isSwellMode=(fxStates&(1<<8))!=0; isVibratoMode=(fxStates&(1<<9))!=0;
    preferences.end();
    
    pinMode(BATTERY_PIN, INPUT); pinMode(38, OUTPUT); digitalWrite(38, LOW); pinMode(15, OUTPUT); digitalWrite(15, HIGH);
    tft.init(); tft.setRotation(1); spr.createSprite(tft.width(), tft.height()); meterSpr.createSprite(6, 98);
    tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM); tft.setTextSize(3); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString("BOOTING...", 160, 85);
    delay(120); digitalWrite(38, HIGH); btmidi.setName("Whammy_S3");
    
    pinMode(pinPB, INPUT_PULLUP); pinMode(pinPB2, INPUT_PULLUP); pinMode(pinPB3, INPUT_PULLUP);
    delayBuffer=(int16_t*)heap_caps_aligned_alloc(16, MAX_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM); fbDelayBuffer=(int16_t*)heap_caps_aligned_alloc(16, FB_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM); freezeBuffer=(int16_t*)heap_caps_aligned_alloc(16, FREEZE_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM); pitchShiftLUT=(float*)heap_caps_aligned_alloc(16, 16384*sizeof(float), MALLOC_CAP_SPIRAM); pitchShiftLUT_temp=(float*)heap_caps_aligned_alloc(16, 16384*sizeof(float), MALLOC_CAP_SPIRAM);
    if(delayBuffer==nullptr||fbDelayBuffer==nullptr||freezeBuffer==nullptr||pitchShiftLUT==nullptr||pitchShiftLUT_temp==nullptr) { tft.fillScreen(TFT_RED); tft.setTextColor(TFT_WHITE, TFT_RED); tft.drawString("MEMORY ERROR", 160, 85); while(1) delay(100); }
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE*sizeof(int16_t)); memset(fbDelayBuffer, 0, FB_BUFFER_SIZE*sizeof(int16_t)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE*sizeof(int16_t)); memset(pitchShiftLUT, 0, 16384*sizeof(float)); memset(pitchShiftLUT_temp, 0, 16384*sizeof(float)); memset(hannLUT, 0, 1024*sizeof(float));
    for(int i=0; i<1024; i++) { hannLUT[i]=0.5f*(1.0f-cosf(TWO_PI*((float)i/1023.0f))); lfoLUT[i]=powf(2.0f,(15.0f*sinf(TWO_PI*((float)i/1024.0f)))/1200.0f); }
    for(int i=0; i<2048; i++) { synthLUT[i]=sinf((((float)i-1024.0f)/1024.0f)*45.0f); }
    
    calibratePBs(); updateLUT();
    if(audioBufferMutex!=NULL) xSemaphoreTake(audioBufferMutex, portMAX_DELAY);
    pitchShiftFactor=pitchShiftLUT[8192];
    if(audioBufferMutex!=NULL) xSemaphoreGive(audioBufferMutex);
    
    Control_Surface >> pipes >> btmidi; Control_Surface >> pipes >> usbmidi; usbmidi >> pipes >> Control_Surface; btmidi >> pipes >> Control_Surface;
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr); Control_Surface.begin();
    
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9); esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
    
    i2s_chan_config_t i2sConfig=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); i2sConfig.dma_desc_num=3; i2sConfig.dma_frame_num=HOP_SIZE; i2sConfig.auto_clear=true; i2s_new_channel(&i2sConfig, &tx_chan, &rx_chan);
    i2s_std_config_t stdConfig={ .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate), .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), .gpio_cfg={ .mclk=GPIO_NUM_43, .bclk=GPIO_NUM_44, .ws=GPIO_NUM_18, .dout=GPIO_NUM_16, .din=GPIO_NUM_17 } };
    i2s_channel_init_std_mode(tx_chan, &stdConfig); i2s_channel_init_std_mode(rx_chan, &stdConfig);
    
    i2s_event_callbacks_t cbs={ .on_recv=i2s_rx_callback, .on_recv_q_ovf=NULL, .on_sent=NULL, .on_send_q_ovf=NULL }; i2s_channel_register_event_callback(rx_chan, &cbs, NULL);
    i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); settingsNeedSaving=false;
    
    xTaskCreatePinnedToCore(DisplayTask, "UI", 8192, NULL, 1, NULL, 0); xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 3, NULL, 0); xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES-1, &audioTaskHandle, 1);
}

void loop() { vTaskDelete(NULL); }