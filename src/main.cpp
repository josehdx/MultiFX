// v1.29 LilyGo Release Controller (Raw IDF BLE Fix & I2S Thread Safety)
#pragma GCC optimize ("Ofast")
#include <Arduino.h>
#include <Control_Surface.h>
#include <WiFi.h>
#include <driver/i2s_std.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"
#include <math.h>
#include <atomic>
#include "esp_private/brownout.h"
#include "esp_pm.h"
#include "esp_task_wdt.h" 
#include "PedalManager.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"

// --- SHARED MODULAR COMPONENTS ---
#include "SettingsManager.h"
#include "DSPEngine.h"
#include "MidiRouter.h"
#include "BluetoothManager.h"

// --- LILYGO-SPECIFIC COMPONENTS ---
#include "PowerManager.h"
#include "DisplayManager.h"

#define ENABLE_ADVANCED_TELEMETRY 
#define ENABLE_PAR_KNOBS false

#ifdef ENABLE_ADVANCED_TELEMETRY
    std::atomic<uint32_t> audio_underflow_count{0};
    std::atomic<uint32_t> dsp_stack_watermark{0};
#endif

struct AppSettings { float fxMem[10]; float params[10][5]; };
Preferences preferences;
SettingsManager settingsMgr;
DisplayManager display;

volatile bool settingsNeedSaving = false;
volatile unsigned long lastParameterChangeTime = 0;
float fxParams[10][5] = {{0.0f,1.0f,0.0f,0.0f,0.0f},{0.6f,0.0002f,0.00005f,0.0f,0.0f},{5120.0f,30.0f,0.02f,0.0f,0.0f},{0.5f,0.0f,0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f,0.0f,0.0f},{0.1f,0.005f,0.3f,0.1f,0.0f},{0.95f,1.5f,0.0f,0.0f,0.0f},{1536.0f,0.4f,0.0f,0.0f,0.0f},{0.015f,0.00002f,0.00005f,0.0f,0.0f},{1.0f,0.0f,0.0f,0.0f,0.0f}};
const bool INVERT_PB3 = false;

std::atomic<bool> dsp_is_paused{false}, dsp_ack_parked{false}, ui_clear_meters_requested{false};
std::atomic<float> pitchShiftFactor{1.0f};
std::atomic<bool> globalAudioResetRequested{false}, panicResetRequested{false}, bleEnabled{true};
std::atomic<bool> isScreenOff{false}, wakeupPending{false};
std::atomic<uint32_t> currentSampleRate{48000}; 

const float LATENCY_WINDOWS[]={512.0f,1024.0f,2048.0f,4096.0f};
int32_t *i2s_in_block = nullptr, *i2s_out_block = nullptr;
float *inBuf=nullptr, *envBuf=nullptr, *fzOutBuf=nullptr, *masterGainBuf=nullptr;
float *w1Buf=nullptr, *w2Buf=nullptr, *w3Buf=nullptr, *padFilterBuf=nullptr;
float *dryBuf=nullptr, *fbOutBuf=nullptr, *sMixBuf=nullptr;
esp_pm_lock_handle_t dsp_cpu_lock = NULL;

__attribute__((aligned(64))) uint8_t dmaPingBuffer[256];
__attribute__((aligned(64))) uint8_t dmaPongBuffer[256];
int16_t* activeDmaReadBuf = (int16_t*)dmaPingBuffer;
int16_t* activeDmaWriteBuf = (int16_t*)dmaPongBuffer;
async_memcpy_handle_t dma_memcpy_handle = NULL;
std::atomic<bool> dma_transfer_done{true};
volatile uint32_t dma_success_count = 0;

static bool IRAM_ATTR dma_memcpy_cb(async_memcpy_handle_t mcp_hdl, async_memcpy_event_t *event, void *cb_args) {
    static_cast<std::atomic<bool>*>(cb_args)->store(true, std::memory_order_release);
    dma_success_count = dma_success_count + 1;
    return false;
}

volatile i2s_chan_handle_t tx_chan = NULL, rx_chan = NULL;

struct __attribute__((aligned(64))) DSPCoreState {
    float fxMem[10]; float params[10][5];
    int activeMode; int latMode; int fbIdx;
    bool w, fz, fb, hr, cp, sy, pd, ch, sw, vb; float vg; uint8_t _padding[52]; 
};
DSPCoreState dspStates[2];
std::atomic<DSPCoreState*> dspActiveState{&dspStates[0]};
int dspWriteIndex = 1; volatile bool dspNeedsCommit = false; std::atomic<bool> dspAckCommit{true}; 

volatile uint16_t lastActivePedal = 8192;
volatile float effectMemory[10]={12.0f,-12.0f,0.0f,5.0f,-2.0f,-12.0f,-12.0f,12.0f,0.0f,0.0f}; 
volatile bool isWhammyActive=true, isFrozen=false, isFeedbackActive=false, isHarmonizerMode=false, isSynthMode=false, isPadMode=false, isCapoMode=false, isChorusMode=false, isSwellMode=false, isVibratoMode=false, isVolumeMode=false, isPB2WiperMode=false;
volatile float volumePedalGain=1.0f;
std::atomic<int> latencyMode{0}, activeEffectMode{0}, feedbackIntervalIdx{0}; 

#define HOP_SIZE 64
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 131072

int16_t *sramPitchBuffer = nullptr, *delayBuffer = nullptr, *fbDelayBuffer = nullptr, *freezeBuffer = nullptr;
float *diffuserBuf = nullptr; int diffuserIdx = 0;

TaskHandle_t audioTaskHandle = NULL; StackType_t* dspTaskStack = nullptr; StaticTask_t* dspTaskTCB = nullptr;
std::atomic<float*> pitchShiftLUT{nullptr}; float *pitchShiftLUT_temp = nullptr;
int writeIndex = 0, fbDelayWriteIdx = 0, sramWriteIdx = 0;

#define HANN_LUT_SIZE 4096
#define LFO_LUT_SIZE 1024
#define WAVE_LUT_SIZE 2048

float DRAM_ATTR hannLUT[HANN_LUT_SIZE] __attribute__((aligned(64)));
float DRAM_ATTR lfoLUT[LFO_LUT_SIZE] __attribute__((aligned(64)));
float DRAM_ATTR synthLUT[WAVE_LUT_SIZE] __attribute__((aligned(64)));

std::atomic<float> globalHarmRatio{1.0f}, globalChorusRatio{1.0f}, globalFbRatio{1.0f}, globalVibratoPhaseInc{0.0f};
uint32_t tap_w1_1=0, tap_w1_2=256<<16, tap_w2_1=0, tap_w2_2=256<<16, tap_w3_1=0, tap_w3_2=256<<16, tap_w4_1=0, tap_w4_2=256<<16, tap_w5_1=0, tap_w5_2=256<<16;
float currentWindowSize = 512.0f; int freezeLength = 48000; bool wasFrozen = false;
volatile bool apfNeedsClear = false; volatile float freezeRamp = 0.0f;
float apf1Buffer[1009] = {0.0f}, apf2Buffer[863] = {0.0f}; int apf1Idx = 0, apf2Idx = 0; 
volatile bool lutNeedsUpdate = false; volatile float chorusLfoPhase=0.0f, feedbackLfoPhase=0.0f, vibratoLfoPhase=0.0f, swellGain=0.0f, feedbackRamp=0.0f; float fbHpfState=0.0f, feedbackFilter=0.0f; 

std::atomic<int> hardwareSyncMuteFrames{0}; volatile bool sampleRateToggleRequested=false, pb2ToggleRequested=false; unsigned long lastActivityTime=0;
const unsigned long LIGHT_SLEEP_TIMEOUT=120000;

std::atomic<float> core0_dsp_load __attribute__((aligned(64))) {0.0f};
std::atomic<float> core1_ctrl_load __attribute__((aligned(64))) {0.0f};
std::atomic<uint32_t> max_loop_latency_ms{0}; 
std::atomic<float> ui_audio_level __attribute__((aligned(64))) {0.0f}, ui_output_level __attribute__((aligned(64))) {0.0f};

volatile bool isAdcPaused=false; adc_continuous_handle_t multifx_adc_handle = NULL;
volatile int latestPB1=2048, latestPB2=2048, latestPB3=2048; std::atomic<int> latestBat{2048};
const int BATTERY_PIN=4, BOOT_SENSE_PIN=0, BLE_TOGGLE_PIN=14, SYSTEM_POWER_LATCH_PIN=5, TFT_BL_PIN=15;
pin_t pinPB=1, pinPB2=2, pinPB3=10, pinPar1=3, pinPar2=11, pinPar3=12, pinPar4=13, pinPar5=16;

volatile uint16_t currentPB1=8192, currentPB2=8192, currentPB3=8192, currentCC11=0; 
FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar1=pinPar1, filterPar2=pinPar2, filterPar3=pinPar3, filterPar4=pinPar4, filterPar5=pinPar5;

BluetoothMIDI_Interface btmidi; USBMIDI_Interface usbmidi; MIDI_PipeFactory<4> pipes; PedalManager pedals;

const char* MODE_PARAM_NAMES[10][5] = {
    {"HEEL", "TOE", "DRY", "WET", "--"},      
    {"HEEL", "TOE", "APF", "ATTK", "REL"},    
    {"INTRVL", "SPEED", "DRIVE", "OFFSET", "--"}, 
    {"INTRVL", "MIX", "--", "--", "--"},      
    {"SEMI", "CENTS", "--", "--", "--"},     
    {"PITCH", "ATTK", "REL", "FLTR", "MIX"},   
    {"PITCH", "SMOOTH", "MIX", "--", "--"},   
    {"PITCH", "SPEED", "MIX", "--", "--"},    
    {"HEEL", "TOE", "THRESH", "ATTK", "REL"}, 
    {"PITCH", "DEPTH", "--", "--", "--"}      
};

inline void safeDmaWait() {
    uint32_t spinCount = 0;
    while (!dma_transfer_done.load(std::memory_order_acquire)) {
        std::atomic_thread_fence(std::memory_order_acquire);
        __asm__ __volatile__ ("nop");
        if (++spinCount > 50000) {
            dma_transfer_done.store(true, std::memory_order_release);
            break;
        }
    }
}

void switchEffectMode(int newMode) {
    int cmode = (newMode % 10 + 10) % 10;
    activeEffectMode.store(cmode, std::memory_order_release);
    
    if (cmode == 0) {
        isWhammyActive = true;
        isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false;
        isCapoMode = false; isSynthMode = false; isPadMode = false;
        isChorusMode = false; isSwellMode = false; isVibratoMode = false;
    } else {
        isWhammyActive = true; 
        if (cmode == 1) isFrozen = true;
        if (cmode == 2) isFeedbackActive = true;
        if (cmode == 3) isHarmonizerMode = true;
        if (cmode == 4) isCapoMode = true;
        if (cmode == 5) isSynthMode = true;
        if (cmode == 6) isPadMode = true;
        if (cmode == 7) isChorusMode = true;
        if (cmode == 8) isSwellMode = true;
        if (cmode == 9) isVibratoMode = true;
    }

    dspNeedsCommit = true; lutNeedsUpdate = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
}

void triggerPanicReset() {
    panicResetRequested.store(true, std::memory_order_release); 
    activeEffectMode.store(0, std::memory_order_release);
    
    isWhammyActive = true; 
    isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false; 
    isCapoMode = false; isSynthMode = false; isPadMode = false; 
    isChorusMode = false; isSwellMode = false; isVibratoMode = false; 

    dspNeedsCommit = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
}

bool commitDSPState() {
    if (!dspAckCommit.load(std::memory_order_acquire)) return false; 
    DSPCoreState* backBuffer = &dspStates[dspWriteIndex];
    for(int i=0; i<10; i++) { backBuffer->fxMem[i] = effectMemory[i]; for(int j=0; j<5; j++) backBuffer->params[i][j] = fxParams[i][j]; }
    backBuffer->activeMode = activeEffectMode.load(std::memory_order_relaxed); backBuffer->latMode = latencyMode.load(std::memory_order_relaxed); backBuffer->fbIdx = feedbackIntervalIdx.load(std::memory_order_relaxed);
    backBuffer->w = isWhammyActive; backBuffer->fz = isFrozen; backBuffer->fb = isFeedbackActive; backBuffer->hr = isHarmonizerMode; backBuffer->cp = isCapoMode; backBuffer->sy = isSynthMode; backBuffer->pd = isPadMode; backBuffer->ch = isChorusMode; backBuffer->sw = isSwellMode; backBuffer->vb = isVibratoMode; backBuffer->vg = volumePedalGain;
    dspActiveState.store(backBuffer, std::memory_order_release);
    dspWriteIndex = (dspWriteIndex + 1) & 1; dspAckCommit.store(false, std::memory_order_release); 
    return true;
}

void calibratePBs() {
    for(int i=0; i<50; i++) { PowerManager::fetchADCDMA(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestBat); vTaskDelay(pdMS_TO_TICKS(1)); }
    long s1=0, s2=0, s3=0; 
    for(int i=1; i<=250; i++) { PowerManager::fetchADCDMA(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestBat); s1+=latestPB1; s2+=latestPB2; s3+=latestPB3; } 
    pedals.setCenters(s1/250, s2/250, s3/250);
}

void cycleLatencyMode() {
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    latencyMode.store((latencyMode.load(std::memory_order_acquire) + 1) % 4, std::memory_order_release);
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t));
    memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t));
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t));
    if (diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    globalAudioResetRequested.store(true, std::memory_order_release);
    dspNeedsCommit = true;
    std::atomic_thread_fence(std::memory_order_seq_cst);
    dsp_is_paused.store(false, std::memory_order_release);
    while(dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    settingsNeedSaving = true; lastParameterChangeTime = millis();
}

void toggleSampleRate() {
    // 1. Tell DSP task to park FIRST to avoid i2s_channel_write errors
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }

    // 2. Safely disable and delete channels after DSP task is parked
    i2s_channel_disable((i2s_chan_handle_t)tx_chan); 
    i2s_channel_disable((i2s_chan_handle_t)rx_chan); 

    i2s_del_channel((i2s_chan_handle_t)tx_chan); 
    i2s_del_channel((i2s_chan_handle_t)rx_chan);

    uint32_t newSr = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? 48000 : 96000;
    currentSampleRate.store(newSr, std::memory_order_release);
    settingsNeedSaving = false; lutNeedsUpdate = true;

    i2s_chan_config_t i2sConfig=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2sConfig.dma_desc_num=8; i2sConfig.dma_frame_num=HOP_SIZE; i2sConfig.auto_clear=true;
    i2s_chan_handle_t t_tx, t_rx; i2s_new_channel(&i2sConfig, &t_tx, &t_rx); tx_chan = t_tx; rx_chan = t_rx;
    
    i2s_std_config_t stdConfig={ 
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(newSr), 
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg={ .mclk=GPIO_NUM_43, .bclk=GPIO_NUM_44, .ws=GPIO_NUM_18, .dout=GPIO_NUM_21, .din=GPIO_NUM_17 } 
    };
    stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; 
    
    i2s_channel_init_std_mode((i2s_chan_handle_t)tx_chan, &stdConfig); 
    i2s_channel_init_std_mode((i2s_chan_handle_t)rx_chan, &stdConfig);
    
    freezeLength = newSr;
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t)); 
    esp_cache_msync((void*)delayBuffer, MAX_BUFFER_SIZE * sizeof(int16_t), ESP_CACHE_MSYNC_FLAG_DIR_C2M); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t)); 
    if(diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    vTaskDelay(pdMS_TO_TICKS(30)); 
    
    i2s_channel_enable((i2s_chan_handle_t)tx_chan); 
    i2s_channel_enable((i2s_chan_handle_t)rx_chan);

    globalAudioResetRequested.store(true, std::memory_order_release); 
    hardwareSyncMuteFrames.store((newSr/HOP_SIZE)*0.40f, std::memory_order_release);
    
    std::atomic_thread_fence(std::memory_order_seq_cst); 
    dsp_is_paused.store(false, std::memory_order_release);
    while(dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }

    pedals.triggerSystemRecovery(); settingsNeedSaving=true; lastParameterChangeTime = millis();
}

void goToLightSleep() {
    if(!isScreenOff.load(std::memory_order_acquire)) { digitalWrite(TFT_BL_PIN, LOW); isScreenOff.store(true, std::memory_order_release); }
    
    // 1. Tell DSP task to park FIRST
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }

    // 2. Now safe to disable channels without race conditions
    i2s_channel_disable((i2s_chan_handle_t)tx_chan); 
    i2s_channel_disable((i2s_chan_handle_t)rx_chan);
    
    adc_continuous_stop(multifx_adc_handle);
    if (dsp_cpu_lock != NULL) esp_pm_lock_release(dsp_cpu_lock);
    while ((REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) && (REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN))) { vTaskDelay(pdMS_TO_TICKS(50)); }
    while (((REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) == 0) || ((REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN)) == 0)) { vTaskDelay(pdMS_TO_TICKS(10)); }
    if (dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);
    adc_continuous_start(multifx_adc_handle);
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t)); 
    esp_cache_msync((void*)delayBuffer, MAX_BUFFER_SIZE * sizeof(int16_t), ESP_CACHE_MSYNC_FLAG_DIR_C2M); 
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t)); 
    if(diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    
    i2s_channel_enable((i2s_chan_handle_t)tx_chan); 
    i2s_channel_enable((i2s_chan_handle_t)rx_chan); 
    
    globalAudioResetRequested.store(true, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_seq_cst); 
    dsp_is_paused.store(false, std::memory_order_release);
    while(dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    
    if(isScreenOff.load(std::memory_order_acquire) && !wakeupPending.load(std::memory_order_acquire)) { wakeupPending.store(true, std::memory_order_release); digitalWrite(TFT_BL_PIN, HIGH); isScreenOff.store(false, std::memory_order_release); wakeupPending.store(false, std::memory_order_release); }
    pedals.triggerSystemRecovery(); lastActivityTime = millis();
}

void IRAM_ATTR __attribute__((optimize("Ofast"))) updateLUT() {
    static std::atomic<bool> lutBusy{false};
    if (lutBusy.exchange(true, std::memory_order_acquire)) return; 
    if (pitchShiftLUT_temp == nullptr) { lutBusy.store(false, std::memory_order_release); return; }
    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    float basePitch=0.0f; if(activeDSP->cp || (activeDSP->activeMode==4 && activeDSP->w)) basePitch+=activeDSP->fxMem[4];
    float toeBend=activeDSP->fxMem[0], heelBend=activeDSP->fxMem[1], harmRatioMem=activeDSP->fxMem[3], chorusRatioMem=activeDSP->fxMem[7], vibHzMem=activeDSP->fxMem[9]; int fbIntervalIdxLocal=activeDSP->fbIdx;
    for(int i=0; i<16384; i++) {
        float normalizedThrow=(i>=8192)?((float)(i-8192)/8191.0f):((float)(i-8192)/8192.0f); 
        pitchShiftLUT_temp[i]=powf(2.0f,(basePitch+((normalizedThrow>=0.0f)?(toeBend*normalizedThrow):(heelBend*fabsf(normalizedThrow))))/12.0f); 
        if(i > 0 && i % 2048 == 0) vTaskDelay(1);
    }
    float* tempPtr = pitchShiftLUT.load(std::memory_order_relaxed);
    pitchShiftLUT.store(pitchShiftLUT_temp, std::memory_order_release); pitchShiftLUT_temp = tempPtr;
    globalHarmRatio.store(powf(2.0f, harmRatioMem/12.0f), std::memory_order_release); globalChorusRatio.store(powf(2.0f, chorusRatioMem/12.0f), std::memory_order_release); 
    float fbIntervals[5]={0.0f,12.0f,19.0f,24.0f,28.0f}; globalFbRatio.store(powf(2.0f, fbIntervals[constrain(fbIntervalIdxLocal,0,4)]/12.0f), std::memory_order_release);
    uint32_t sr = currentSampleRate.load(std::memory_order_acquire);
    if (__builtin_expect(sr > 0, 1)) globalVibratoPhaseInc.store((((vibHzMem!=0.0f)?fabsf(vibHzMem):2.0f)*LFO_LUT_SIZE)/(float)sr, std::memory_order_release);
    lutBusy.store(false, std::memory_order_release);
}

bool channelMessageCallback(ChannelMessage cm) {
    lastActivityTime=millis();
    if(cm.header==0xB0) {
        uint8_t cc = cm.data1;
        uint8_t val = cm.data2;

        if(cc==11) { 
            uint16_t mappedCC=map(val,0,127,0,16383); currentCC11=mappedCC; currentPB3=mappedCC; lastActivePedal=mappedCC; 
            if(isVolumeMode) { volumePedalGain=(float)mappedCC/16383.0f; dspNeedsCommit = true; Control_Surface.sendControlChange({19,Channel_1},val); } else { if(!lutNeedsUpdate) { float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire); if(currentLUT) pitchShiftFactor.store(currentLUT[mappedCC], std::memory_order_release); } } return false; 
        }
        
        if(cc>=24 && cc<=28) { MidiRouter::updateParameter(cc, val, activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx); return false; }
        if(cc==5 && val>=64) { isPB2WiperMode=!isPB2WiperMode; dspNeedsCommit = true; pb2ToggleRequested=true; return false; }
        if(cc==6 && val>=64) { 
            bool sendCenterMidi=false; isVolumeMode=!isVolumeMode; float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire);
            if(!isVolumeMode) { volumePedalGain=1.0f; pedals.lockPB3Whammy(); sendCenterMidi=true; currentPB3=8192; lastActivePedal=8192; if(!lutNeedsUpdate && currentLUT!=nullptr) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release); } else { pedals.lockPB3Volume(); lastActivePedal=8192; volumePedalGain=(float)currentPB3 / 16383.0f; if(!lutNeedsUpdate && currentLUT!=nullptr) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release); } 
            dspNeedsCommit = true; if(sendCenterMidi) Control_Surface.sendPitchBend(Channel_3, 8192); settingsNeedSaving=true; lastParameterChangeTime=millis(); return false; 
        }
        if (cc == 0 && val >= 64) { switchEffectMode(activeEffectMode.load(std::memory_order_acquire) - 1); return false; }
        if (cc == 1 && val >= 64) { switchEffectMode(activeEffectMode.load(std::memory_order_acquire) + 1); return false; }
        if (cc == 2 && val >= 64) { sampleRateToggleRequested = true; return false; }
        if (cc == 3 && val >= 64) { cycleLatencyMode(); return false; }
        if (cc == 4 && val >= 64) { triggerPanicReset(); return false; }
        
        if (val >= 64 && cc >= 7 && cc <= 16) {
            bool toggled = true;
            int curMode = activeEffectMode.load(std::memory_order_acquire);
            if (cc == 7)       { isVibratoMode = !isVibratoMode; if (curMode == 9) isWhammyActive = isVibratoMode; }
            else if (cc == 8)  { isFrozen = !isFrozen; if (curMode == 1) isWhammyActive = isFrozen; }
            else if (cc == 9)  { isFeedbackActive = !isFeedbackActive; if (curMode == 2) isWhammyActive = isFeedbackActive; }
            else if (cc == 10) { isHarmonizerMode = !isHarmonizerMode; if (curMode == 3) isWhammyActive = isHarmonizerMode; }
            else if (cc == 12) { isCapoMode = !isCapoMode; if (curMode == 4) isWhammyActive = isCapoMode; lutNeedsUpdate = true; }
            else if (cc == 13) { isSynthMode = !isSynthMode; if (curMode == 5) isWhammyActive = isSynthMode; }
            else if (cc == 14) { isPadMode = !isPadMode; if (curMode == 6) isWhammyActive = isPadMode; }
            else if (cc == 15) { isChorusMode = !isChorusMode; if (curMode == 7) isWhammyActive = isChorusMode; }
            else if (cc == 16) { isSwellMode = !isSwellMode; if (curMode == 8) isWhammyActive = isSwellMode; }
            else { toggled = false; }

            if (toggled) { dspNeedsCommit = true; settingsNeedSaving = true; lastParameterChangeTime = millis(); return false; }
        }

        if (cc == 17 || cc == 18) {
            float step = (val >= 64) ? -1.0f : 1.0f;
            int renderMode = activeEffectMode.load(std::memory_order_acquire);
            if (cc == 17) {
                if (renderMode == 0 || renderMode == 1 || renderMode == 8) { effectMemory[1] = constrain(effectMemory[1] + step, -24.0f, 24.0f); } 
                else if (renderMode == 4) { effectMemory[4] = constrain(effectMemory[4] + step, -24.0f, 24.0f); } 
                else if (renderMode == 2) { int curFb = feedbackIntervalIdx.load(std::memory_order_acquire); if (step > 0) feedbackIntervalIdx.store((curFb + 1) % 5, std::memory_order_release); else feedbackIntervalIdx.store((curFb + 4) % 5, std::memory_order_release); } 
                else { effectMemory[renderMode] = constrain(effectMemory[renderMode] + step, -24.0f, 24.0f); }
            } else if (cc == 18) {
                if (renderMode == 0 || renderMode == 1 || renderMode == 8) { effectMemory[0] = constrain(effectMemory[0] + step, -24.0f, 24.0f); } 
                else if (renderMode == 4) { float centStep = step * 0.01f; effectMemory[4] = constrain(effectMemory[4] + centStep, -24.0f, 24.0f); }
            }
            dspNeedsCommit = true; lutNeedsUpdate = true; settingsNeedSaving = true; lastParameterChangeTime = millis();
            return false;
        }
    }
    return false;
}

void IRAM_ATTR __attribute__((optimize("Ofast"))) AudioDSPTask(void * pvParameters) {
    float input_dc_offset=0.0f, synthEnv=0.0f, synthFilter=0.0f, synthBandpass=0.0f, padFilter=0.0f, padEnv=0.0f, inputEnvelope=0.0f, feedbackFilterVar=0.0f, smoothedVolGain=1.0f, currentPitch=1.0f, fbOutNode=0.0f, smoothed_delay_samples=0.0f, dampState=0.0f, wowState=0.0f;
    uint32_t wowRng = 123456789; bool wasFeedbackActive=false; int freezeWriteIdxVar=0, freezePlayCounterVar=0, freezeStartIdxVar=0, activeFreezeLength=48000;
    float c_fx[10][5] = {0.0f}; int c_lat=0, c_act=0; bool c_w=true, c_fz=false, c_fb=false, c_hr=false, c_cp=false, c_sy=false, c_pd=false, c_ch=false, c_sw=false, c_vb=false; float c_vg=1.0f;
    const float normFactor=1.0f/2147483648.0f, DC_OFFSET=1e-9f;
    
    for(;;) {
        if(__builtin_expect(dsp_is_paused.load(std::memory_order_acquire), 0)) {
            dsp_ack_parked.store(true, std::memory_order_release); while(dsp_is_paused.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(2)); } dsp_ack_parked.store(false, std::memory_order_release);
        }
        size_t bytesRead; i2s_channel_read((i2s_chan_handle_t)rx_chan, i2s_in_block, HOP_SIZE*2*sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(10));
        if(__builtin_expect(bytesRead > 0, 1)) {
            int framesRead=bytesRead/8;
            if(__builtin_expect(framesRead == HOP_SIZE, 1)) {
                if(__builtin_expect(panicResetRequested.load(std::memory_order_acquire), 0)) {
                    safeDmaWait();
                    memset(dmaPingBuffer, 0, sizeof(dmaPingBuffer)); memset(dmaPongBuffer, 0, sizeof(dmaPongBuffer));
                    synthEnv=0.0f; synthFilter=0.0f; synthBandpass=0.0f; padFilter=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate.load(std::memory_order_acquire); fbDelayWriteIdx=0; apfNeedsClear=true; freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f; dampState=0.0f; wowState=0.0f; diffuserIdx=0; if(diffuserBuf) memset(diffuserBuf, 0, 1024*sizeof(float));
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16; tap_w1_1=0; tap_w1_2=halfWinFixed; tap_w2_1=0; tap_w2_2=halfWinFixed; tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed; panicResetRequested.store(false, std::memory_order_release);
                }
                if(__builtin_expect(globalAudioResetRequested.load(std::memory_order_acquire), 0)) {
                    safeDmaWait();
                    synthEnv=0.0f; synthFilter=0.0f; synthBandpass=0.0f; padFilter=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; smoothedVolGain=volumePedalGain; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate.load(std::memory_order_acquire); fbDelayWriteIdx=0; writeIndex=0; sramWriteIdx=0; apfNeedsClear=true; input_dc_offset=0.0f; ui_audio_level.store(0.0f, std::memory_order_release); ui_output_level.store(0.0f, std::memory_order_release); freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f; dampState=0.0f; wowState=0.0f; diffuserIdx=0; if(diffuserBuf) memset(diffuserBuf, 0, 1024*sizeof(float));
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16; tap_w1_1=0; tap_w1_2=halfWinFixed; tap_w2_1=0; tap_w2_2=halfWinFixed; tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    memset(dmaPingBuffer, 0, sizeof(dmaPingBuffer)); memset(dmaPongBuffer, 0, sizeof(dmaPongBuffer));
                    globalAudioResetRequested.store(false, std::memory_order_release); smoothed_delay_samples=0.0f; 
                    if(hardwareSyncMuteFrames.load(std::memory_order_acquire) < 10) hardwareSyncMuteFrames.store((currentSampleRate.load(std::memory_order_acquire)/HOP_SIZE)*0.40f, std::memory_order_release);
                }
                
                int currentMute = hardwareSyncMuteFrames.load(std::memory_order_acquire); bool isMuted = false;
                if(__builtin_expect(currentMute > 0, 0)) { hardwareSyncMuteFrames.store(currentMute - 1, std::memory_order_release); isMuted = true; }
                uint32_t start_cycles=xthal_get_ccount(); float srScale=48000.0f/(float)currentSampleRate.load(std::memory_order_acquire);
                DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                for(int j=0; j<10; j++) for(int k=0; k<5; k++) c_fx[j][k] = activeDSP->params[j][k]; 
                c_lat = activeDSP->latMode; c_act = activeDSP->activeMode; c_w = activeDSP->w; c_fz = activeDSP->fz; c_fb = activeDSP->fb; c_hr = activeDSP->hr; c_cp = activeDSP->cp; c_sy = activeDSP->sy; c_pd = activeDSP->pd; c_ch = activeDSP->ch; c_sw = activeDSP->sw; c_vb = activeDSP->vb; c_vg = activeDSP->vg; dspAckCommit.store(true, std::memory_order_release); 
                float c_pt = pitchShiftFactor.load(std::memory_order_acquire);
                
                float targetWindow = LATENCY_WINDOWS[c_lat];
                if(__builtin_expect(currentWindowSize!=targetWindow, 0)) { currentWindowSize=targetWindow; uint32_t halfWindowFixed=((uint32_t)targetWindow/2)<<16; tap_w1_1=0; tap_w1_2=halfWindowFixed; tap_w2_1=0; tap_w2_2=halfWindowFixed; tap_w3_1=0; tap_w3_2=halfWindowFixed; tap_w4_1=0; tap_w4_2=halfWindowFixed; tap_w5_1=0; tap_w5_2=halfWindowFixed; }
                uint32_t hannIntMult=(4096U<<16)/(uint32_t)currentWindowSize, windowMask=(uint32_t)currentWindowSize-1; float p_w_dry=c_fx[0][0], p_w_wet=c_fx[0][1], p_fz_apf=c_fx[1][0], p_fz_att=c_fx[1][1], p_fz_rel=c_fx[1][2], p_fb_spd=c_fx[2][0], p_fb_drv=c_fx[2][1], p_fb_off=c_fx[2][2], p_hr_mix=c_fx[3][0], p_sy_att=c_fx[5][0], p_sy_rel=c_fx[5][1], p_sy_flt=c_fx[5][2], p_sy_mix=c_fx[5][3], p_pd_sm=c_fx[6][0], p_pd_mix=c_fx[6][1], p_ch_spd=c_fx[7][0], p_ch_mix=c_fx[7][1], p_sw_thr=c_fx[8][0], p_sw_att=c_fx[8][1], p_sw_rel=c_fx[8][2], p_vb_dep=c_fx[9][0]; float chorusPhaseIncr=p_ch_spd/(float)currentSampleRate.load(std::memory_order_acquire), feedbackPhaseIncr=p_fb_spd/(float)currentSampleRate.load(std::memory_order_acquire), targetPitch=c_pt;
                bool frzActive=((c_act==1&&c_w)||c_fz);
                if(__builtin_expect(frzActive && !wasFrozen, 0)) { freezePlayCounterVar=0; int bestStart=freezeWriteIdxVar, tempIdx=freezeWriteIdxVar; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { bestStart=tempIdx; break; } tempIdx=prev; } freezeStartIdxVar=bestStart; activeFreezeLength=freezeLength; int searchEnd=bestStart-1; if(searchEnd<0) searchEnd+=freezeLength; tempIdx=searchEnd; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { activeFreezeLength=s; break; } tempIdx=prev; } if(activeFreezeLength<64) activeFreezeLength=freezeLength; }
                if(__builtin_expect(!frzActive && wasFrozen, 0)) apfNeedsClear=true; wasFrozen=frzActive; float activeInvFreqLength=1.0f/(float)activeFreezeLength; bool synthActive=((c_act==5&&c_w)||c_sy), padActive=((c_act==6&&c_w)||c_pd), harmActive=((c_act==3&&c_w)||c_hr), swellActive=((c_act==8&&c_w)||c_sw), chorusActive=((c_act==7&&c_w)||c_ch), feedbackActive=((c_act==2&&c_w)||c_fb);
                if(__builtin_expect(feedbackActive && !wasFeedbackActive, 0)) { fbOutNode=0.0f; fbHpfState=0.0f; feedbackFilterVar=0.0f; } wasFeedbackActive=feedbackActive; bool vibratoActive=((c_act==9&&c_w)||c_vb), capoActive=((c_act==4&&c_w)||c_cp);
                float localSwellGain=swellGain, localVolGain=c_vg, localFrzRamp=freezeRamp, localFbRamp=feedbackRamp, pdSmCoeff=powf(p_pd_sm, srScale), target_delay=constrain((float)(currentSampleRate.load(std::memory_order_acquire)*p_fb_off), 0.0f, (float)(FB_BUFFER_SIZE-1)); smoothed_delay_samples+= (target_delay-smoothed_delay_samples)*0.01f*srScale+DC_OFFSET; int delaySamples=(int)smoothed_delay_samples; float fbHpfCoeff=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.025f:0.05f, fbLpfCoeff=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.05f:0.1f, fbLpfRetain=1.0f-fbLpfCoeff, dc_alpha=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.0005f:0.001f; int halfWindow=(int)currentWindowSize/2; bool activeGroup=c_w||harmActive||chorusActive||feedbackActive||synthActive||padActive||frzActive||vibratoActive||capoActive, dryGroup=chorusActive||padActive||frzActive||feedbackActive||(localFrzRamp>0.0f)||(localFbRamp>0.0f), repeatGroup=capoActive||synthActive||vibratoActive||padActive||harmActive;
                float g_base=0.0f; if(dryGroup) { if(!repeatGroup) g_base=0.4f; } else if(harmActive) g_base=0.5f; else g_base=1.0f; float g_w2=harmActive?p_hr_mix:0.0f, g_w3=chorusActive?p_ch_mix:0.0f; bool padIsAudible=padActive||(fabsf(padFilter)>0.001f); float g_pad=padIsAudible?p_pd_mix:0.0f, g_frz=(!frzActive&&localFrzRamp>0.0f)?0.5f:0.0f, g_fb=(feedbackActive||localFbRamp>0.0f)?0.6f:0.0f, g_whammy=c_w?p_w_wet:0.0f, g_dry=c_w?p_w_dry:1.0f, vol_alpha=0.01f*srScale, meter_decay=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.999f:0.998f, envRetain=powf(0.99f,srScale), envAttack=1.0f-envRetain;
                
                if(__builtin_expect(isnan(synthFilter)||isinf(synthFilter), 0)) synthFilter=0.0f; if(__builtin_expect(isnan(synthBandpass)||isinf(synthBandpass), 0)) synthBandpass=0.0f; if(__builtin_expect(isnan(padFilter)||isinf(padFilter), 0)) padFilter=0.0f; if(__builtin_expect(isnan(feedbackFilterVar)||isinf(feedbackFilterVar), 0)) feedbackFilterVar=0.0f; if(__builtin_expect(isnan(fbHpfState)||isinf(fbHpfState), 0)) fbHpfState=0.0f; 
                float localVibPhase=vibratoLfoPhase, localChoPhase=chorusLfoPhase, localFbPhase=feedbackLfoPhase, localFbHpf=fbHpfState; 
                
                int currentDryIdx = (writeIndex - halfWindow + MAX_BUFFER_SIZE) & BUFFER_MASK; int nextDryIdx = (currentDryIdx + HOP_SIZE) & BUFFER_MASK;
                dma_transfer_done.store(false, std::memory_order_release);
                if (__builtin_expect(MAX_BUFFER_SIZE - nextDryIdx >= HOP_SIZE, 1)) {
                    esp_cache_msync((void*)&delayBuffer[nextDryIdx], HOP_SIZE * sizeof(int16_t), ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                    if (esp_async_memcpy(dma_memcpy_handle, activeDmaWriteBuf, &delayBuffer[nextDryIdx], HOP_SIZE * sizeof(int16_t), dma_memcpy_cb, (void*)&dma_transfer_done) != ESP_OK) {
                        for(int i = 0; i < HOP_SIZE; i++) activeDmaWriteBuf[i] = delayBuffer[(nextDryIdx + i) & BUFFER_MASK]; dma_transfer_done.store(true, std::memory_order_release);
                    }
                } else { for(int i = 0; i < HOP_SIZE; i++) activeDmaWriteBuf[i] = delayBuffer[(nextDryIdx + i) & BUFFER_MASK]; dma_transfer_done.store(true, std::memory_order_release); }
                int prefetchIdxFB = (fbDelayWriteIdx - delaySamples + FB_BUFFER_SIZE) & FB_BUFFER_MASK; int aheadFB = (prefetchIdxFB + 32) & FB_BUFFER_MASK; __builtin_prefetch(&fbDelayBuffer[aheadFB], 0, 3);
                
                float peakInputVal=0.0f, peakOutputVal=0.0f;
                DSPEngine::processInput(framesRead, i2s_in_block, normFactor, dc_alpha, envRetain, envAttack, p_sw_thr, p_sw_att, p_sw_rel, srScale, swellActive, localVolGain, vol_alpha, input_dc_offset, inputEnvelope, localSwellGain, smoothedVolGain, currentPitch, targetPitch, envBuf, masterGainBuf, inBuf, fzOutBuf);
                
                if(__builtin_expect(synthActive, 0)) for(int i=0; i<framesRead; i++) { 
                    synthEnv=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,__builtin_fmaf(p_sy_att, srScale, synthEnv)):__builtin_fmaxf(0.0f,__builtin_fmaf(-p_sy_rel, srScale, synthEnv)); 
                    float clampedProc=__builtin_fmaxf(-1.0f,__builtin_fminf(inBuf[i],1.0f)); 
                    int waveIdx=(int)((clampedProc+1.0f)*1023.5f) & 2047; 
                    float procSample=synthLUT[waveIdx]; 
                    float f1 = __builtin_fmaxf(0.001f, __builtin_fminf(0.45f, __builtin_fmaf(0.5f * synthEnv, srScale, p_sy_flt * srScale)));
                    float q1 = 0.5f; float hp = procSample - synthFilter - (q1 * synthBandpass);
                    synthFilter = DSPEngine::AntiDenormal(synthFilter + f1 * synthBandpass + DC_OFFSET);
                    synthBandpass = DSPEngine::AntiDenormal(synthBandpass + f1 * hp + DC_OFFSET);
                    inBuf[i] = synthFilter * p_sy_mix; 
                }
                if(__builtin_expect(padActive, 0)) for(int i=0; i<framesRead; i++) { padEnv=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,__builtin_fmaf(0.00002f, srScale, padEnv)):__builtin_fmaxf(0.0f,__builtin_fmaf(-0.000005f, srScale, padEnv)); inBuf[i]*=padEnv; }
                
                safeDmaWait();
                
                for(int i=0; i<framesRead; i++) {
                    float procSample=inBuf[i]; if(__builtin_expect(!frzActive, 1)) { freezeBuffer[freezeWriteIdxVar]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(procSample,1.0f))*32767.0f); freezeWriteIdxVar++; if(freezeWriteIdxVar>=freezeLength) freezeWriteIdxVar=0; }
                    if(__builtin_expect(localFrzRamp>0.0f||frzActive, 0)) localFrzRamp=frzActive?__builtin_fminf(1.0f,__builtin_fmaf(p_fz_att, srScale, localFrzRamp)):__builtin_fmaxf(0.0f,__builtin_fmaf(-p_fz_rel, srScale, localFrzRamp));
                    if(__builtin_expect(localFrzRamp>0.0f, 0)) {
                        float phaseRead=(float)freezePlayCounterVar*activeInvFreqLength, phase2=(phaseRead+0.5f); if(phase2>=1.0f) phase2-=1.0f;
                        int idx1 = freezeStartIdxVar + freezePlayCounterVar; if (__builtin_expect(idx1 >= freezeLength, 0)) idx1 -= freezeLength;
                        int activeLen = (activeFreezeLength >= 64) ? activeFreezeLength : freezeLength; int counter2 = freezePlayCounterVar + (activeLen / 2); if (__builtin_expect(counter2 >= activeLen, 0)) counter2 -= activeLen;
                        int idx2 = freezeStartIdxVar + counter2; if (__builtin_expect(idx2 >= freezeLength, 0)) idx2 -= freezeLength;
                        int lutIdx1=(int)(phaseRead*4095.0f)&4095, lutIdx2=(int)(phase2*4095.0f)&4095;
                        float rFrz=__builtin_fmaf((float)freezeBuffer[idx1]*3.0517578125e-5f, hannLUT[lutIdx1], (float)freezeBuffer[idx2]*3.0517578125e-5f*hannLUT[lutIdx2]);
                        float d1=apf1Buffer[apf1Idx], next_apf1=__builtin_fmaf(p_fz_apf, d1, rFrz+DC_OFFSET), a1=__builtin_fmaf(-p_fz_apf, rFrz, d1); apf1Buffer[apf1Idx]=next_apf1; apf1Idx++; if(apf1Idx>=1009) apf1Idx=0; float d2=apf2Buffer[apf2Idx], next_apf2=__builtin_fmaf(p_fz_apf, d2, a1+DC_OFFSET), a2=__builtin_fmaf(-p_fz_apf, a1, d2); apf2Buffer[apf2Idx]=next_apf2; apf2Idx++; if(apf2Idx>=863) apf2Idx=0; fzOutBuf[i]=a2*localFrzRamp; freezePlayCounterVar++; if(freezePlayCounterVar>=activeFreezeLength) freezePlayCounterVar=0;
                    } else if(__builtin_expect(apfNeedsClear, 0)) { memset(apf1Buffer,0,sizeof(apf1Buffer)); memset(apf2Buffer,0,sizeof(apf2Buffer)); apf1Idx=0; apf2Idx=0; apfNeedsClear=false; }
                    float delayIn=(localFrzRamp>0.0f)?__builtin_fmaf(procSample, (1.0f-localFrzRamp), fzOutBuf[i]):procSample; 
                    int16_t sample16 = (int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(delayIn,1.0f))*32767.0f); delayBuffer[writeIndex] = sample16; sramPitchBuffer[sramWriteIdx] = sample16; 
                    
                    float spd1=currentPitch; 
                    if(__builtin_expect(vibratoActive, 0)) { localVibPhase+=globalVibratoPhaseInc.load(std::memory_order_acquire); if(localVibPhase>=LFO_LUT_SIZE) localVibPhase-=LFO_LUT_SIZE; spd1*=1.0f+((DSPEngine::getLfoInterpolated(localVibPhase, lfoLUT)-1.0f)*p_vb_dep); }
                    float spd2=currentPitch*globalHarmRatio.load(std::memory_order_acquire); float spd3=currentPitch*globalChorusRatio.load(std::memory_order_acquire); 
                    if(__builtin_expect(chorusActive, 0)) { localChoPhase+=chorusPhaseIncr; if(localChoPhase>=LFO_LUT_SIZE) localChoPhase-=LFO_LUT_SIZE; spd3*=DSPEngine::getLfoInterpolated(localChoPhase, lfoLUT); } 
                    float spd4=1.0f, spd5=1.0f;
                    
                    if(__builtin_expect(feedbackActive||localFbRamp>0.0f, 0)) {
                        localFbPhase+=feedbackPhaseIncr; if(localFbPhase>=LFO_LUT_SIZE) localFbPhase-=LFO_LUT_SIZE; float lfoVal=DSPEngine::getLfoInterpolated(localFbPhase, lfoLUT); spd4=lfoVal; spd5=currentPitch*globalFbRatio.load(std::memory_order_acquire)*lfoVal;
                        wowRng = wowRng * 1664525U + 1013904223U; float rawNoise = ((float)(wowRng & 0xFFFF) * 0.0000305185f) - 1.0f; wowState = DSPEngine::AntiDenormal(__builtin_fmaf(rawNoise - wowState, 0.0005f * srScale, wowState)); float wowMod = 1.0f + (wowState * 0.0015f); spd4 *= wowMod; spd5 *= wowMod;
                        float w4=DSPEngine::processHermiteTap(tap_w4_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT)+DSPEngine::processHermiteTap(tap_w4_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT);
                        float w5=DSPEngine::processHermiteTap(tap_w5_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT)+DSPEngine::processHermiteTap(tap_w5_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT);
                        if(feedbackActive) localFbRamp=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,__builtin_fmaf(0.000011f, srScale, localFbRamp)):__builtin_fmaxf(0.0f,__builtin_fmaf(-0.0005f, srScale, localFbRamp)); else localFbRamp=__builtin_fmaxf(0.0f,__builtin_fmaf(-0.0001f, srScale, localFbRamp));
                        float mixV=__builtin_fmaxf(0.0f,__builtin_fminf((localFbRamp-0.1f)*2.0f,1.0f)), feedInput=(frzActive&&localFrzRamp>0.0f)?fzOutBuf[i]:__builtin_fmaf(w4, (1.0f-mixV), __builtin_fmaf(w5, mixV, fbOutNode*0.95f)); 
                        localFbHpf=DSPEngine::AntiDenormal(__builtin_fmaf(fbHpfCoeff, (feedInput-localFbHpf), DC_OFFSET)); 
                        float rawDrive=(feedInput-localFbHpf)*p_fb_drv, boundedDrive=__builtin_fmaxf(-1.5f,__builtin_fminf(rawDrive,1.5f)), gainDrive=boundedDrive*(1.0f-(0.15f*boundedDrive*boundedDrive)); feedbackFilterVar=DSPEngine::AntiDenormal(__builtin_fmaf(gainDrive, fbLpfCoeff, __builtin_fmaf(feedbackFilterVar, fbLpfRetain, DC_OFFSET))); float satFb=feedbackFilterVar*(localFbRamp*localFbRamp*localFbRamp)*0.85f; fbDelayBuffer[fbDelayWriteIdx]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(satFb,1.0f))*32767.0f);
                        int fbReadIdx=(fbDelayWriteIdx-delaySamples+FB_BUFFER_SIZE)&FB_BUFFER_MASK; fbOutNode=DSPEngine::AntiDenormal((float)fbDelayBuffer[fbReadIdx]*3.0517578125e-5f); fbDelayWriteIdx=(fbDelayWriteIdx+1)&FB_BUFFER_MASK;
                    } else { fbDelayBuffer[fbDelayWriteIdx]=0; fbOutNode=0.0f; fbDelayWriteIdx=(fbDelayWriteIdx+1)&FB_BUFFER_MASK; }
                    
                    float rawW1 = DSPEngine::processSincTap(tap_w1_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT)+DSPEngine::processSincTap(tap_w1_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT); 
                    float dampCutoff = (currentPitch > 1.498f) ? __builtin_fmaxf(0.1f, 1.0f - (currentPitch - 1.498f) * 0.5f) : 1.0f;
                    dampState = DSPEngine::AntiDenormal(__builtin_fmaf(dampCutoff, (rawW1 - dampState), dampState)); w1Buf[i] = dampState;
                    w2Buf[i]=0.0f; if(__builtin_expect(harmActive, 0)) w2Buf[i]=DSPEngine::processHermiteTap(tap_w2_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT)+DSPEngine::processHermiteTap(tap_w2_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT); 
                    w3Buf[i]=0.0f; if(__builtin_expect(chorusActive, 0)) w3Buf[i]=DSPEngine::processHermiteTap(tap_w3_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT)+DSPEngine::processHermiteTap(tap_w3_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult,hannLUT);
                    
                    int32_t step1=(int32_t)((1.0f-spd1)*65536.0f); tap_w1_1+=step1; tap_w1_2+=step1; int32_t step2=(int32_t)((1.0f-spd2)*65536.0f); tap_w2_1+=step2; tap_w2_2+=step2; int32_t step3=(int32_t)((1.0f-spd3)*65536.0f); tap_w3_1+=step3; tap_w3_2+=step3; int32_t step4=(int32_t)((1.0f-spd4)*65536.0f); tap_w4_1+=step4; tap_w4_2+=step4; int32_t step5=(int32_t)((1.0f-spd5)*65536.0f); tap_w5_1+=step5; tap_w5_2+=step5;
                    if(__builtin_expect(padActive, 0)) padFilter=DSPEngine::AntiDenormal(__builtin_fmaf(padFilter, pdSmCoeff, __builtin_fmaf(w1Buf[i], (1.0f-pdSmCoeff), DC_OFFSET))); else padFilter=DSPEngine::AntiDenormal(__builtin_fmaf(padFilter, pdSmCoeff, DC_OFFSET)); 
                    
                    float pad_out = padFilter, diffIn = w3Buf[i] + pad_out, diffOut = diffuserBuf ? diffuserBuf[diffuserIdx] : 0.0f;
                    float diffNext = DSPEngine::AntiDenormal(diffIn + 0.6f * diffOut); if(diffuserBuf) diffuserBuf[diffuserIdx] = diffNext;
                    float diffFinal = DSPEngine::AntiDenormal(diffOut - 0.6f * diffNext); diffuserIdx = (diffuserIdx + 1) & 1023;
                    if(chorusActive && padActive) { w3Buf[i] = diffFinal * 0.5f; padFilterBuf[i] = diffFinal * 0.5f; } else if(chorusActive) { w3Buf[i] = diffFinal; padFilterBuf[i] = pad_out; } else if(padActive) { w3Buf[i] = 0.0f; padFilterBuf[i] = diffFinal; } else { padFilterBuf[i] = pad_out; }
                    dryBuf[i] = (float)activeDmaReadBuf[i] * 3.0517578125e-5f; fbOutBuf[i]=fbOutNode; writeIndex=(writeIndex+1)&BUFFER_MASK; sramWriteIdx=(sramWriteIdx+1)&SRAM_PITCH_BUF_MASK;
                }
                vibratoLfoPhase=localVibPhase; chorusLfoPhase=localChoPhase; feedbackLfoPhase=localFbPhase; fbHpfState=localFbHpf;
                int16_t* tempDmaPtr = activeDmaReadBuf; activeDmaReadBuf = activeDmaWriteBuf; activeDmaWriteBuf = tempDmaPtr;
                
                DSPEngine::mixdownAndOutput(framesRead, activeGroup, localFrzRamp, localFbRamp, g_whammy, g_dry, g_base, g_w2, g_w3, g_pad, g_frz, g_fb, padFilterBuf, dryBuf, w1Buf, w2Buf, w3Buf, fzOutBuf, fbOutBuf, sMixBuf, masterGainBuf, inBuf, i2s_out_block, peakInputVal, peakOutputVal);
                swellGain=localSwellGain; freezeRamp=localFrzRamp; feedbackRamp=localFbRamp;
                
                if (__builtin_expect(ui_clear_meters_requested.exchange(false, std::memory_order_acq_rel), 0)) { ui_audio_level.store(0.0f, std::memory_order_release); ui_output_level.store(0.0f, std::memory_order_release); } 
                else { float current_in = ui_audio_level.load(std::memory_order_acquire); if(peakInputVal > current_in) { ui_audio_level.store(peakInputVal, std::memory_order_release); } else { current_in *= meter_decay; ui_audio_level.store((current_in < 1e-5f) ? 0.0f : current_in, std::memory_order_release); } float current_out = ui_output_level.load(std::memory_order_acquire); if(peakOutputVal > current_out) { ui_output_level.store(peakOutputVal, std::memory_order_release); } else { current_out *= meter_decay; ui_output_level.store((current_out < 1e-5f) ? 0.0f : current_out, std::memory_order_release); } }
                
                uint32_t end_timer=xthal_get_ccount(); float max_cycles = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? (2500.0f * (float)framesRead) : (5000.0f * (float)framesRead);
                
                core0_dsp_load.store(__builtin_fmaf(core0_dsp_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, (((float)(end_timer - start_cycles) / max_cycles) * 100.0f)) * 0.05f), std::memory_order_relaxed);
                
                if(__builtin_expect(isMuted, 0)) memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                size_t bytesWrittenCount; i2s_channel_write((i2s_chan_handle_t)tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20));
            } else {
                #ifdef ENABLE_ADVANCED_TELEMETRY
                    audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
                #endif
                vTaskDelay(pdMS_TO_TICKS(1)); 
            }
        } else {
            #ifdef ENABLE_ADVANCED_TELEMETRY
                audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
            #endif
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }
    }
}

void setup() {
    Serial.begin(115200); esp_brownout_init(); WiFi.mode(WIFI_OFF);
    
    // 1. MUST HAVE: 1000ms delay to let LilyGO's display and USB tasks settle the memory heap
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); err = nvs_flash_init(); }
    ESP_ERROR_CHECK(err);

    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        esp_task_wdt_config_t wdt_cfg = {
            .timeout_ms = 30000,
            .idle_core_mask = (1 << 1),
            .trigger_panic = true
        };
        esp_task_wdt_reconfigure(&wdt_cfg);
    #else
        esp_task_wdt_init(30, true);
    #endif

    pinMode(TFT_BL_PIN, OUTPUT); digitalWrite(TFT_BL_PIN, HIGH);
    display.begin(); 
    
    // 2. MANUALLY PRE-INITIALIZE THE RAW ESP-IDF CONTROLLER & HCI
    BluetoothManager::initHCI();

    // 3. ESTABLISH STATIC MIDI ROUTING PIPES
    btmidi.setName("LilyGO_FX"); 
    Control_Surface >> pipes >> btmidi; 
    Control_Surface >> pipes >> usbmidi; 
    usbmidi >> pipes >> Control_Surface; 
    btmidi >> pipes >> Control_Surface;
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr); 
    
    // 4. START CONTROL SURFACE (Boots USB and BLE safely)
    Control_Surface.begin();

    // 5. CONFIGURE MAXIMUM TX POWER (+9dBm) AND READ MAC
    BluetoothManager::configurePowerAndMac();

    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "DSP_Max_CPU", &dsp_cpu_lock); if(dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);
    
    adc_continuous_handle_cfg_t adc_config={}; adc_config.max_store_buf_size=16384; adc_config.conv_frame_size=128; 
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &multifx_adc_handle));
    adc_continuous_config_t dig_cfg={}; dig_cfg.sample_freq_hz = 2 * 1000; dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1; dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    adc_digi_pattern_config_t adc_pattern[4]={ {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_0,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_1,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_9,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}, {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_3,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH} };
    dig_cfg.pattern_num=4; dig_cfg.adc_pattern=adc_pattern; ESP_ERROR_CHECK(adc_continuous_config(multifx_adc_handle, &dig_cfg)); ESP_ERROR_CHECK(adc_continuous_start(multifx_adc_handle));
    
    settingsMgr.init(preferences);
    
    // Enforce Clean Boot State
    activeEffectMode.store(0, std::memory_order_release);
    isWhammyActive = true; 
    isFrozen = false; isFeedbackActive = false; isHarmonizerMode = false;
    isCapoMode = false; isSynthMode = false; isPadMode = false;
    isChorusMode = false; isSwellMode = false; isVibratoMode = false;

    latencyMode.store(constrain(preferences.getInt("latMode", 0), 0, 3), std::memory_order_release); 
    isPB2WiperMode=preferences.getBool("pb2Wiper", false); isVolumeMode=false; currentSampleRate.store(48000, std::memory_order_release); feedbackIntervalIdx.store(constrain(preferences.getInt("fbIdx", 0), 0, 4), std::memory_order_release);
    AppSettings savedSettings; size_t len=preferences.getBytes("dspData", &savedSettings, sizeof(AppSettings));
    if(len==sizeof(AppSettings)) { for(int i=0; i<10; i++) { effectMemory[i]=savedSettings.fxMem[i]; for(int p=0; p<5; p++) fxParams[i][p]=savedSettings.params[i][p]; } }
    commitDSPState();
    
    pedals.resetToCenter(); pinMode(BOOT_SENSE_PIN, INPUT_PULLUP); pinMode(BLE_TOGGLE_PIN, INPUT_PULLUP); lastActivityTime=millis();

    delayBuffer=(int16_t*)heap_caps_aligned_alloc(64, MAX_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    sramPitchBuffer=(int16_t*)heap_caps_aligned_alloc(64, SRAM_PITCH_BUF_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    fbDelayBuffer=(int16_t*)heap_caps_aligned_alloc(64, FB_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    freezeBuffer=(int16_t*)heap_caps_aligned_alloc(64, FREEZE_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    diffuserBuf = (float*)heap_caps_aligned_alloc(64, 1024 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    float* initialLUT = (float*)heap_caps_aligned_alloc(64, 16384*sizeof(float), MALLOC_CAP_SPIRAM); pitchShiftLUT.store(initialLUT, std::memory_order_relaxed); pitchShiftLUT_temp=(float*)heap_caps_aligned_alloc(64, 16384*sizeof(float), MALLOC_CAP_SPIRAM);
    i2s_in_block = (int32_t*)heap_caps_aligned_alloc(64, HOP_SIZE * 2 * sizeof(int32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); i2s_out_block = (int32_t*)heap_caps_aligned_alloc(64, HOP_SIZE * 2 * sizeof(int32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    inBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); envBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); fzOutBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); masterGainBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); w1Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); w2Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); w3Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); padFilterBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); dryBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); fbOutBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); sMixBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!delayBuffer || !freezeBuffer || !i2s_in_block || !i2s_out_block || !pitchShiftLUT_temp || !inBuf || !fbDelayBuffer || !sramPitchBuffer || !diffuserBuf) { Serial.println("FATAL ERROR"); while(1) { vTaskDelay(pdMS_TO_TICKS(100)); } }
    
    memset(i2s_in_block, 0, HOP_SIZE * 2 * sizeof(int32_t)); memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t)); memset(inBuf, 0, HOP_SIZE * sizeof(float)); memset(envBuf, 0, HOP_SIZE * sizeof(float)); memset(fzOutBuf, 0, HOP_SIZE * sizeof(float)); memset(masterGainBuf, 0, HOP_SIZE * sizeof(float)); memset(w1Buf, 0, HOP_SIZE * sizeof(float)); memset(w2Buf, 0, HOP_SIZE * sizeof(float)); memset(w3Buf, 0, HOP_SIZE * sizeof(float)); memset(padFilterBuf, 0, HOP_SIZE * sizeof(float)); memset(dryBuf, 0, HOP_SIZE * sizeof(float)); memset(fbOutBuf, 0, HOP_SIZE * sizeof(float)); memset(sMixBuf, 0, HOP_SIZE * sizeof(float));
    memset(delayBuffer, 0, MAX_BUFFER_SIZE*sizeof(int16_t)); memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE*sizeof(int16_t)); memset(fbDelayBuffer, 0, FB_BUFFER_SIZE*sizeof(int16_t)); memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE*sizeof(int16_t)); memset(diffuserBuf, 0, 1024*sizeof(float));
    float* initLUTPtr = pitchShiftLUT.load(std::memory_order_relaxed); if(initLUTPtr) memset(initLUTPtr, 0, 16384*sizeof(float)); memset(pitchShiftLUT_temp, 0, 16384*sizeof(float)); 
    memset(dmaPingBuffer, 0, sizeof(dmaPingBuffer)); memset(dmaPongBuffer, 0, sizeof(dmaPongBuffer));

    // Re-initialize fast memory arrays
    for(int i=0; i<4096; i++) { hannLUT[i]=0.5f*(1.0f-cosf(TWO_PI*((float)i/4095.0f))); } 
    for(int i=0; i<1024; i++) { lfoLUT[i]=powf(2.0f,(15.0f*sinf(TWO_PI*((float)i/1024.0f)))/1200.0f); } 
    for(int i=0; i<2048; i++) { synthLUT[i]=sinf((((float)i-1024.0f)/1024.0f)*45.0f); }

    async_memcpy_config_t dma_config = ASYNC_MEMCPY_DEFAULT_CONFIG(); dma_config.backlog = 8; ESP_ERROR_CHECK(esp_async_memcpy_install(&dma_config, &dma_memcpy_handle));
    calibratePBs(); updateLUT(); float* currLut = pitchShiftLUT.load(std::memory_order_acquire); if (currLut) pitchShiftFactor.store(currLut[8192], std::memory_order_release);
    
    i2s_chan_config_t i2sConfig=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); i2sConfig.dma_desc_num=8; i2sConfig.dma_frame_num=HOP_SIZE; i2sConfig.auto_clear=true; 
    i2s_chan_handle_t t_tx, t_rx; i2s_new_channel(&i2sConfig, &t_tx, &t_rx); tx_chan = t_tx; rx_chan = t_rx;
    i2s_std_config_t stdConfig={ 
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate.load(std::memory_order_acquire)), 
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg={ .mclk=GPIO_NUM_43, .bclk=GPIO_NUM_44, .ws=GPIO_NUM_18, .dout=GPIO_NUM_21, .din=GPIO_NUM_17 } 
    };
    stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; 
    
    i2s_channel_init_std_mode((i2s_chan_handle_t)tx_chan, &stdConfig); 
    i2s_channel_init_std_mode((i2s_chan_handle_t)rx_chan, &stdConfig);
    
    // Stack Size Reduced to 8192 bytes
    dspTaskStack = (StackType_t*)heap_caps_aligned_alloc(16, 8192, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); dspTaskTCB = (StaticTask_t*)heap_caps_aligned_alloc(16, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (dspTaskStack != nullptr && dspTaskTCB != nullptr) { audioTaskHandle = xTaskCreateStaticPinnedToCore(AudioDSPTask, "DSP", 8192, NULL, configMAX_PRIORITIES - 1, dspTaskStack, dspTaskTCB, 0); } else { while(1) vTaskDelay(100); }
    i2s_channel_enable((i2s_chan_handle_t)tx_chan); i2s_channel_enable((i2s_chan_handle_t)rx_chan); 
}

void loop() {
    unsigned long loop_start_time = micros();
    static unsigned long lastLoopMicro = micros();
    static bool lastBtState=false; static uint8_t lastVolumeCC=127;
    #if ENABLE_PAR_KNOBS
        static int lastCcOut[5]={-1,-1,-1,-1,-1};
    #endif
    static unsigned long gpio14PressTime = 0; static bool gpio14LastState = HIGH; static unsigned long lastDebounceTime = 0; static bool lastBootState = HIGH;

    if (bleEnabled.load(std::memory_order_relaxed)) { Control_Surface.loop(); } else { Control_Surface.updateMidiInput(); }

    bool currentBtState=btmidi.isConnected(); if(currentBtState!=lastBtState) { lastBtState=currentBtState; }
    
    bool reading = (REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN)) != 0;
    if (reading != gpio14LastState && (millis() - lastDebounceTime) > 50) {
        lastDebounceTime = millis();
        if (!reading) { gpio14PressTime = millis(); lastActivityTime = millis(); } 
        else { 
            unsigned long pressDuration = millis() - gpio14PressTime;
            if (pressDuration >= 1000) { if (bleEnabled.load(std::memory_order_relaxed)) { btStop(); bleEnabled.store(false, std::memory_order_relaxed); } else { btStart(); bleEnabled.store(true, std::memory_order_relaxed); } } else if (pressDuration >= 50) { cycleLatencyMode(); }
        }
        gpio14LastState = reading;
    }
    
    #if ENABLE_PAR_KNOBS
        if(filterPar1.update()) { int cc1=map(filterPar1.getValue(),0,4095,0,127); if(cc1!=lastCcOut[0]) { Control_Surface.sendControlChange({24,Channel_1},cc1); MidiRouter::updateParameter(24,cc1, activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx); lastCcOut[0]=cc1; } }
        if(filterPar2.update()) { int cc2=map(filterPar2.getValue(),0,4095,0,127); if(cc2!=lastCcOut[1]) { Control_Surface.sendControlChange({25,Channel_1},cc2); MidiRouter::updateParameter(25,cc2, activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx); lastCcOut[1]=cc2; } }
        if(filterPar3.update()) { int cc3=map(filterPar3.getValue(),0,4095,0,127); if(cc3!=lastCcOut[2]) { Control_Surface.sendControlChange({26,Channel_1},cc3); MidiRouter::updateParameter(26,cc3, activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx); lastCcOut[2]=cc3; } }
        if(filterPar4.update()) { int cc4=map(filterPar4.getValue(),0,4095,0,127); if(cc4!=lastCcOut[3]) { Control_Surface.sendControlChange({27,Channel_1},cc4); MidiRouter::updateParameter(27,cc4, activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx); lastCcOut[3]=cc4; } }
        if(filterPar5.update()) { int cc5=map(filterPar5.getValue(),0,4095,0,127); if(cc5!=lastCcOut[4]) { Control_Surface.sendControlChange({28,Channel_1},cc5); MidiRouter::updateParameter(28,cc5, activeEffectMode.load(std::memory_order_acquire), effectMemory, fxParams, lutNeedsUpdate, dspNeedsCommit, feedbackIntervalIdx); lastCcOut[4]=cc5; } }
    #endif
    
    PowerManager::fetchADCDMA(multifx_adc_handle, isAdcPaused, latestPB1, latestPB2, latestPB3, latestBat);
    
    bool currentBootState = (REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) != 0;
    if(!currentBootState && lastBootState) { switchEffectMode(activeEffectMode.load(std::memory_order_acquire) + 1); lastActivityTime = millis(); vTaskDelay(pdMS_TO_TICKS(50)); }
    lastBootState = currentBootState;

    if (millis() - lastActivityTime > LIGHT_SLEEP_TIMEOUT) {
        goToLightSleep();
    }
    
    pedals.process(latestPB1, latestPB2, latestPB3, isVolumeMode, INVERT_PB3);
    int calA=pedals.getCalA(), calB=pedals.getCalB(), calC=pedals.getCalC(); bool moveA=pedals.hasMovedA(), moveB=pedals.hasMovedB(), moveC=pedals.hasMovedC();
    if(moveA||moveB||moveC) {
        lastActivityTime=millis();
        if(moveA) { Control_Surface.sendPitchBend(Channel_1, calA); pedals.updateLastMidiA(); currentPB1=calA; }
        if(moveB) { Control_Surface.sendPitchBend(Channel_2, calB); pedals.updateLastMidiB(); currentPB2=calB; }
        if(moveC) { if(!isVolumeMode) Control_Surface.sendPitchBend(Channel_3, calC); pedals.updateLastMidiC(); currentPB3=calC; }
        bool pitchChanged=false; if(moveA) { lastActivePedal=calA; pitchChanged=true; } if(moveB) { lastActivePedal=calB; pitchChanged=true; } if(moveC&&!isVolumeMode) { lastActivePedal=calC; pitchChanged=true; }
        if(pitchChanged) { float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire); if(currentLUT) pitchShiftFactor.store(currentLUT[constrain(lastActivePedal,0,16383)], std::memory_order_release); }
        if(moveC&&isVolumeMode) { uint8_t vCC=map(calC,0,16383,0,127); if(vCC!=lastVolumeCC) { Control_Surface.sendControlChange({19,Channel_1},vCC); lastVolumeCC=vCC; } volumePedalGain=(float)calC/16383.0f; dspNeedsCommit = true; }
    }
    
    static unsigned long lastLutUpdate=0;
    if(lutNeedsUpdate && (millis()-lastLutUpdate>40)) { lutNeedsUpdate=false; updateLUT(); float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire); if(currentLUT) pitchShiftFactor.store(currentLUT[constrain(lastActivePedal,0,16383)], std::memory_order_release); lastLutUpdate=millis(); }
    
    if(settingsNeedSaving && (millis()-lastParameterChangeTime>2000)) { 
        if (millis()-lastParameterChangeTime>10000) {
            settingsNeedSaving=false; dsp_is_paused.store(true, std::memory_order_release); while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
            
            DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
            AppSettings cs; for(int i=0; i<10; i++) { cs.fxMem[i]=activeDSP->fxMem[i]; for(int p=0; p<5; p++) cs.params[i][p]=activeDSP->params[i][p]; }
            uint16_t fxStates=0; if(activeDSP->w) fxStates|=(1<<0); if(activeDSP->fz) fxStates|=(1<<1); if(activeDSP->fb) fxStates|=(1<<2); if(activeDSP->hr) fxStates|=(1<<3); if(activeDSP->cp) fxStates|=(1<<4); if(activeDSP->sy) fxStates|=(1<<5); if(activeDSP->pd) fxStates|=(1<<6); if(activeDSP->ch) fxStates|=(1<<7); if(activeDSP->sw) fxStates|=(1<<8); if(activeDSP->vb) fxStates|=(1<<9);
            settingsMgr.save(preferences, activeDSP->activeMode, activeDSP->latMode, constrain(activeDSP->fbIdx,0,4), isPB2WiperMode, isVolumeMode, fxStates, currentSampleRate.load(std::memory_order_acquire), &cs, sizeof(AppSettings));
            
            dsp_is_paused.store(false, std::memory_order_release);
        }
    }
    
    if(sampleRateToggleRequested) { sampleRateToggleRequested=false; toggleSampleRate(); } if(pb2ToggleRequested) { pb2ToggleRequested=false; calibratePBs(); settingsNeedSaving=true; lastParameterChangeTime=millis(); }
    if (dspNeedsCommit) { if (commitDSPState()) dspNeedsCommit = false; }
    
    unsigned long loopBusyTime = micros() - loop_start_time;
    unsigned long totalLoopTime = micros() - lastLoopMicro;
    lastLoopMicro = micros();
    if (totalLoopTime > 0) {
        float c1Load = ((float)loopBusyTime / (float)totalLoopTime) * 100.0f;
        core1_ctrl_load.store(__builtin_fmaf(core1_ctrl_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, c1Load) * 0.05f), std::memory_order_relaxed);
    }

    uint32_t iter_latency = (micros() - loop_start_time) / 1000; 
    if (iter_latency > max_loop_latency_ms.load(std::memory_order_relaxed)) max_loop_latency_ms.store(iter_latency, std::memory_order_relaxed);

    #ifdef ENABLE_ADVANCED_TELEMETRY
        if (audioTaskHandle != NULL) {
            uint32_t freeStackWords = uxTaskGetStackHighWaterMark(audioTaskHandle);
            dsp_stack_watermark.store(freeStackWords * sizeof(StackType_t), std::memory_order_relaxed);
        }
    #endif

    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    DisplayData dData;
    
    dData.batVoltage = PowerManager::getBatteryVoltage(latestBat.load(std::memory_order_relaxed));
    dData.batPercent = PowerManager::getBatteryPercentage(dData.batVoltage);
    dData.bleConnected = btmidi.isConnected();
    dData.activeMode = activeDSP->activeMode;
    
    dData.fxStates[0] = activeDSP->w;  dData.fxStates[1] = activeDSP->fz;
    dData.fxStates[2] = activeDSP->fb; dData.fxStates[3] = activeDSP->hr;
    dData.fxStates[4] = activeDSP->cp; dData.fxStates[5] = activeDSP->sy;
    dData.fxStates[6] = activeDSP->pd; dData.fxStates[7] = activeDSP->ch;
    dData.fxStates[8] = activeDSP->sw; dData.fxStates[9] = activeDSP->vb;
    
    dData.pb1 = currentPB1;
    dData.pb2 = currentPB2;
    dData.pb3 = currentPB3;
    dData.cc11 = currentCC11;
    
    int curModeIdx = activeDSP->activeMode % 10;
    for (int p = 0; p < 5; p++) {
        dData.paramVals[p] = activeDSP->params[curModeIdx][p];
        dData.paramNames[p] = MODE_PARAM_NAMES[curModeIdx][p];
    }

    dData.inMeter = ui_audio_level.load(std::memory_order_acquire);
    dData.outMeter = ui_output_level.load(std::memory_order_acquire);
    dData.dspCoreLoad = core0_dsp_load.load(std::memory_order_relaxed);
    dData.ctrlCoreLoad = core1_ctrl_load.load(std::memory_order_relaxed);
    dData.freeSRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
    dData.freePSRAM = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    dData.sampleRate = currentSampleRate.load(std::memory_order_acquire);
    dData.peakLatency = max_loop_latency_ms.exchange(0, std::memory_order_relaxed);

    #ifdef ENABLE_ADVANCED_TELEMETRY
        dData.underflows = audio_underflow_count.load(std::memory_order_relaxed);
        dData.dmaCount = dma_success_count;
        dData.stackWatermark = dsp_stack_watermark.load(std::memory_order_relaxed);
    #else
        dData.underflows = 0;
        dData.dmaCount = 0;
        dData.stackWatermark = 0;
    #endif

    // --- 33ms (30 FPS) THROTTLED DISPLAY RENDER TIMER ---
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 33) {
        lastDisplayUpdate = millis();
        display.render(dData);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}