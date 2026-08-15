// v1.16 Lilygo Release Controller (Production Audit & Hardened DSP Edition)
#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <WiFi.h>
#include <driver/i2s_std.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_continuous.h"
#include "dsps_mul.h"
#include "dsps_add.h"
#include "dsps_biquad.h"
#include "driver/rtc_io.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include <math.h>
#include <atomic>
#include <Preferences.h>
#include "esp_private/brownout.h"
#include "esp_pm.h"
#include "PedalManager.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include <TFT_eSPI.h> 

#define ENABLE_PAR_KNOBS false

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);     // Working Off-screen Canvas (PSRAM)
TFT_eSprite bgSpr = TFT_eSprite(&tft);   // Pre-rendered Static Background (PSRAM)

struct AppSettings {
    float fxMem[10];
    float params[10][5];
};
Preferences preferences;

const char* EFFECT_NAMES[] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};

volatile bool settingsNeedSaving = false;
volatile unsigned long lastParameterChangeTime = 0;
float fxParams[10][5] = {{0.0f,1.0f,0.0f,0.0f,0.0f},{0.6f,0.0002f,0.00005f,0.0f,0.0f},{5120.0f,30.0f,0.02f,0.0f,0.0f},{0.5f,0.0f,0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f,0.0f,0.0f},{0.1f,0.005f,0.3f,0.1f,0.0f},{0.95f,1.5f,0.0f,0.0f,0.0f},{1536.0f,0.4f,0.0f,0.0f,0.0f},{0.015f,0.00002f,0.00005f,0.0f,0.0f},{1.0f,0.0f,0.0f,0.0f,0.0f}};
const bool INVERT_PB3 = false;

std::atomic<bool> dsp_is_paused{false};
std::atomic<bool> dsp_ack_parked{false};
std::atomic<bool> ui_clear_meters_requested{false};
std::atomic<bool> isScreenOff{false};
std::atomic<bool> wakeupPending{false};
std::atomic<bool> isBatteryDead{false};
std::atomic<float> pitchShiftFactor{1.0f};
std::atomic<bool> globalAudioResetRequested{false};
std::atomic<bool> panicResetRequested{false};
std::atomic<bool> bleEnabled{true};

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

void updateLUT(); 
void DisplayTask(void * pvParameters); 
void AudioDSPTask(void * pvParameters);
bool channelMessageCallback(ChannelMessage cm); 
void updateParameterFromCC(uint8_t cc, uint8_t val);

volatile i2s_chan_handle_t tx_chan = NULL;
volatile i2s_chan_handle_t rx_chan = NULL;

struct __attribute__((aligned(64))) DSPCoreState {
    float fxMem[10];
    float params[10][5];
    int activeMode;
    int latMode;
    int fbIdx;
    bool w, fz, fb, hr, cp, sy, pd, ch, sw, vb;
    float vg;
    uint8_t _padding[52]; 
};
DSPCoreState dspStates[2];
std::atomic<DSPCoreState*> dspActiveState{&dspStates[0]};
int dspWriteIndex = 1;
volatile bool dspNeedsCommit = false;
std::atomic<bool> dspAckCommit{true}; 

volatile uint16_t lastActivePedal = 8192;
volatile float effectMemory[10]={12.0f,-12.0f,0.0f,5.0f,-2.0f,-12.0f,-12.0f,12.0f,0.0f,0.0f}; 

volatile bool isWhammyActive=true, isFrozen=false, isFeedbackActive=false, isHarmonizerMode=false, isSynthMode=false, isPadMode=false, isCapoMode=false, isChorusMode=false, isSwellMode=false, isVibratoMode=false, isVolumeMode=false, isPB2WiperMode=false;

volatile float volumePedalGain=1.0f;
std::atomic<int> latencyMode{0};
std::atomic<int> activeEffectMode{0};
std::atomic<int> feedbackIntervalIdx{0}; 

#define HOP_SIZE 64
#define MAX_BUFFER_SIZE 65536
#define BUFFER_MASK 0xFFFF
#define FB_BUFFER_SIZE 8192
#define FB_BUFFER_MASK 0x1FFF
#define FREEZE_BUFFER_SIZE 131072

// --- ZERO WAIT-STATE INTERNAL SRAM PITCH BUFFER ---
#define SRAM_PITCH_BUF_SIZE 8192
#define SRAM_PITCH_BUF_MASK 0x1FFF
int16_t *sramPitchBuffer = nullptr; 

int16_t *delayBuffer = nullptr, *fbDelayBuffer = nullptr, *freezeBuffer = nullptr;

// 1024-SAMPLE SPATIAL DIFFUSER BUFFER (4KB in Internal SRAM)
float *diffuserBuf = nullptr;
int diffuserIdx = 0;

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
    settingsNeedSaving = true;
    lastParameterChangeTime = millis();
}

bool commitDSPState() {
    if (!dspAckCommit.load(std::memory_order_acquire)) return false; 

    DSPCoreState* backBuffer = &dspStates[dspWriteIndex];
    for(int i=0; i<10; i++) {
        backBuffer->fxMem[i] = effectMemory[i];
        for(int j=0; j<5; j++) backBuffer->params[i][j] = fxParams[i][j];
    }
    backBuffer->activeMode = activeEffectMode.load(std::memory_order_relaxed);
    backBuffer->latMode = latencyMode.load(std::memory_order_relaxed);
    backBuffer->fbIdx = feedbackIntervalIdx.load(std::memory_order_relaxed);
    
    backBuffer->w = isWhammyActive;
    backBuffer->fz = isFrozen;
    backBuffer->fb = isFeedbackActive;
    backBuffer->hr = isHarmonizerMode;
    backBuffer->cp = isCapoMode;
    backBuffer->sy = isSynthMode;
    backBuffer->pd = isPadMode;
    backBuffer->ch = isChorusMode;
    backBuffer->sw = isSwellMode;
    backBuffer->vb = isVibratoMode;
    backBuffer->vg = volumePedalGain;
    
    dspActiveState.store(backBuffer, std::memory_order_release);
    dspWriteIndex = (dspWriteIndex + 1) & 1; 
    dspAckCommit.store(false, std::memory_order_release); 
    
    return true;
}

TaskHandle_t audioTaskHandle = NULL;
StackType_t* dspTaskStack = nullptr;
StaticTask_t* dspTaskTCB = nullptr;

std::atomic<float*> pitchShiftLUT{nullptr};
float *pitchShiftLUT_temp = nullptr;
int writeIndex = 0, fbDelayWriteIdx = 0, sramWriteIdx = 0;

// 4096-POINT ULTRA-RESOLUTION HANN WINDOW
#define HANN_LUT_SIZE 4096
#define LFO_LUT_SIZE 1024
#define WAVE_LUT_SIZE 2048
float hannLUT[HANN_LUT_SIZE] __attribute__((aligned(64)));
float lfoLUT[LFO_LUT_SIZE] __attribute__((aligned(64)));
float synthLUT[WAVE_LUT_SIZE] __attribute__((aligned(64)));

std::atomic<float> globalHarmRatio{1.0f}, globalChorusRatio{1.0f}, globalFbRatio{1.0f}, globalVibratoPhaseInc{0.0f};
uint32_t tap_w1_1=0, tap_w1_2=256<<16, tap_w2_1=0, tap_w2_2=256<<16, tap_w3_1=0, tap_w3_2=256<<16, tap_w4_1=0, tap_w4_2=256<<16, tap_w5_1=0, tap_w5_2=256<<16;
float currentWindowSize = 512.0f;
int freezeLength = 48000;
bool wasFrozen = false;
volatile bool apfNeedsClear = false;
volatile float freezeRamp = 0.0f;
float apf1Buffer[1009] = {0.0f}, apf2Buffer[863] = {0.0f};
int apf1Idx = 0, apf2Idx = 0; 

volatile bool lutNeedsUpdate = false;
volatile float chorusLfoPhase=0.0f, feedbackLfoPhase=0.0f, vibratoLfoPhase=0.0f, swellGain=0.0f, feedbackRamp=0.0f;
float fbHpfState=0.0f, feedbackFilter=0.0f; 

std::atomic<int> hardwareSyncMuteFrames{0}; 
volatile bool sampleRateToggleRequested=false, pb2ToggleRequested=false;
unsigned long lastActivityTime=0;

const unsigned long LIGHT_SLEEP_TIMEOUT=120000; 
const unsigned long SCREEN_OFF_TIMEOUT=30000;   

std::atomic<float> core1_load __attribute__((aligned(64))) {0.0f};
std::atomic<float> ui_audio_level __attribute__((aligned(64))) {0.0f};
std::atomic<float> ui_output_level __attribute__((aligned(64))) {0.0f};

volatile bool isAdcPaused=false;
adc_continuous_handle_t multifx_adc_handle = NULL;
volatile int latestPB1=2048, latestPB2=2048, latestPB3=2048;
std::atomic<int> latestBat{2048};
std::atomic<int> currentBatteryPercent{100};
volatile float currentBatteryVoltage=4.00f;
std::atomic<bool> isBatteryCharging{false};
const int BATTERY_PIN=4, BOOT_SENSE_PIN=0, BLE_TOGGLE_PIN=14, SYSTEM_POWER_LATCH_PIN=5;

pin_t pinPB=1, pinPB2=2, pinPB3=10, pinPar1=3, pinPar2=11, pinPar3=12, pinPar4=13, pinPar5=16;
uint16_t lastMidiSent=8192;
volatile uint16_t currentPB1=8192, currentPB2=8192, currentPB3=8192, currentCC11=0; 

FilteredAnalog<12, 4, uint32_t, uint32_t> filterPar1=pinPar1, filterPar2=pinPar2, filterPar3=pinPar3, filterPar4=pinPar4, filterPar5=pinPar5;
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;
PedalManager pedals;

void fetchADCDMA() {
    if(isAdcPaused) return;
    uint8_t result[128] __attribute__((aligned(4)));
    uint32_t ret_num=0;
    esp_err_t err;
    int loop_bound = 0;
    while(loop_bound++ < 4) {
        err=adc_continuous_read(multifx_adc_handle, result, sizeof(result), &ret_num, 0);
        if(err==ESP_OK && ret_num>0) {
            for(int i=0; i<ret_num; i+=SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p=(adc_digi_output_data_t*)&result[i];
                if(p->type2.channel==ADC_CHANNEL_0) latestPB1=p->type2.data;
                if(p->type2.channel==ADC_CHANNEL_1) latestPB2=p->type2.data;
                if(p->type2.channel==ADC_CHANNEL_9) latestPB3=p->type2.data;
                if(p->type2.channel==ADC_CHANNEL_3) latestBat.store(p->type2.data, std::memory_order_relaxed);
            }
        } else if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) { 
            adc_continuous_stop(multifx_adc_handle); 
            vTaskDelay(pdMS_TO_TICKS(2));
            adc_continuous_start(multifx_adc_handle); 
            break; 
        } else { 
            adc_continuous_stop(multifx_adc_handle); 
            adc_continuous_start(multifx_adc_handle); 
            break; 
        }
    }
}

void switchEffectMode(int newMode) {
    int cmode = (newMode%10+10)%10;
    activeEffectMode.store(cmode, std::memory_order_release);
    isWhammyActive=(cmode==0); isFrozen=(cmode==1); isFeedbackActive=(cmode==2); 
    isHarmonizerMode=(cmode==3); isCapoMode=(cmode==4); isSynthMode=(cmode==5); 
    isPadMode=(cmode==6); isChorusMode=(cmode==7); isSwellMode=(cmode==8); isVibratoMode=(cmode==9);
    dspNeedsCommit = true;
    lutNeedsUpdate=true; settingsNeedSaving=true; lastParameterChangeTime=millis();
}

void saveSettings() {
    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    AppSettings cs;
    for(int i=0; i<10; i++) { cs.fxMem[i]=activeDSP->fxMem[i]; for(int p=0; p<5; p++) cs.params[i][p]=activeDSP->params[i][p]; }
    
    uint16_t fxStates=0; 
    if(activeDSP->w) fxStates|=(1<<0); 
    if(activeDSP->fz) fxStates|=(1<<1); 
    if(activeDSP->fb) fxStates|=(1<<2); 
    if(activeDSP->hr) fxStates|=(1<<3); 
    if(activeDSP->cp) fxStates|=(1<<4); 
    if(activeDSP->sy) fxStates|=(1<<5); 
    if(activeDSP->pd) fxStates|=(1<<6); 
    if(activeDSP->ch) fxStates|=(1<<7); 
    if(activeDSP->sw) fxStates|=(1<<8); 
    if(activeDSP->vb) fxStates|=(1<<9);
    
    int modeCopy = activeDSP->activeMode;
    int latCopy = activeDSP->latMode;
    int fbCopy = constrain(activeDSP->fbIdx,0,4);
    bool pb2Copy = isPB2WiperMode, volCopy = isVolumeMode; 
    uint32_t srCopy = currentSampleRate.load(std::memory_order_acquire);
    
    preferences.begin("whammy_cfg", false); 
    preferences.putInt("activeMode", modeCopy); 
    preferences.putInt("latMode", latCopy); 
    preferences.putBool("pb2Wiper", pb2Copy); 
    preferences.putBool("volMode", volCopy); 
    preferences.putUShort("fxStates", fxStates); 
    preferences.putUInt("sampleRate", srCopy); 
    preferences.putInt("fbIdx", fbCopy); 
    preferences.putBytes("dspData", &cs, sizeof(AppSettings)); 
    preferences.end();
}

int getBatteryPercentage(float voltage) {
    float cv = __builtin_fmaxf(3.30f, __builtin_fminf(4.15f, voltage));
    if(cv>=4.15f) return 100; if(cv<=3.30f) return 0; 
    if(cv>=4.00f) return 90+(int)((cv-4.00f)/0.15f*10.0f); 
    if(cv>=3.90f) return 80+(int)((cv-3.90f)/0.10f*10.0f); 
    if(cv>=3.80f) return 70+(int)((cv-3.80f)/0.10f*10.0f); 
    if(cv>=3.75f) return 60+(int)((cv-3.75f)/0.05f*10.0f); 
    if(cv>=3.70f) return 50+(int)((cv-3.70f)/0.05f*10.0f); 
    if(cv>=3.65f) return 40+(int)((cv-3.65f)/0.05f*10.0f); 
    if(cv>=3.60f) return 30+(int)((cv-3.60f)/0.05f*10.0f); 
    if(cv>=3.55f) return 20+(int)((cv-3.55f)/0.05f*10.0f); 
    if(cv>=3.50f) return 10+(int)((cv-3.50f)/0.05f*10.0f); 
    return (int)(__builtin_fmaxf(0.0f,cv-3.30f)/0.20f*10.0f);
}

void calibratePBs() {
    for(int i=0; i<50; i++) { fetchADCDMA(); vTaskDelay(pdMS_TO_TICKS(1)); }
    long s1=0, s2=0, s3=0; 
    for(int i=1; i<=250; i++) { fetchADCDMA(); s1+=latestPB1; s2+=latestPB2; s3+=latestPB3; } 
    pedals.setCenters(s1/250, s2/250, s3/250);
}

void toggleSampleRate() {
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    
    i2s_channel_disable((i2s_chan_handle_t)tx_chan); i2s_channel_disable((i2s_chan_handle_t)rx_chan); 
    i2s_del_channel((i2s_chan_handle_t)tx_chan); i2s_del_channel((i2s_chan_handle_t)rx_chan);
    currentSampleRate.store((currentSampleRate.load(std::memory_order_acquire) == 96000) ? 48000 : 96000, std::memory_order_release);
    settingsNeedSaving = false; updateLUT(); lutNeedsUpdate = false;
    
    i2s_chan_config_t i2sConfig=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2sConfig.dma_desc_num=8; i2sConfig.dma_frame_num=HOP_SIZE; i2sConfig.auto_clear=true;
    i2s_chan_handle_t t_tx, t_rx;
    i2s_new_channel(&i2sConfig, &t_tx, &t_rx);
    tx_chan = t_tx; rx_chan = t_rx;
    
    i2s_std_config_t stdConfig={ 
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate.load(std::memory_order_acquire)), 
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg={ .mclk=GPIO_NUM_43, .bclk=GPIO_NUM_44, .ws=GPIO_NUM_18, .dout=GPIO_NUM_21, .din=GPIO_NUM_17 } 
    };
    stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; 
    i2s_channel_init_std_mode((i2s_chan_handle_t)tx_chan, &stdConfig); 
    i2s_channel_init_std_mode((i2s_chan_handle_t)rx_chan, &stdConfig);
    
    freezeLength = currentSampleRate.load(std::memory_order_acquire);
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    esp_cache_msync((void*)delayBuffer, MAX_BUFFER_SIZE * sizeof(int16_t), ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t)); 
    if(diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    
    vTaskDelay(pdMS_TO_TICKS(30));
    
    i2s_channel_enable((i2s_chan_handle_t)tx_chan); i2s_channel_enable((i2s_chan_handle_t)rx_chan);
    globalAudioResetRequested.store(true, std::memory_order_release);
    hardwareSyncMuteFrames.store((currentSampleRate.load(std::memory_order_acquire)/HOP_SIZE)*0.40f, std::memory_order_release);
    
    std::atomic_thread_fence(std::memory_order_seq_cst); 
    dsp_is_paused.store(false, std::memory_order_release);
    while(dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    pedals.triggerSystemRecovery(); settingsNeedSaving=true; lastParameterChangeTime = millis();
}

void turnScreenOff() { 
    if(!isScreenOff.load(std::memory_order_acquire)) { 
        digitalWrite(15, LOW);
        isScreenOff.store(true, std::memory_order_release); 
    } 
}

void turnScreenOn() { 
    if(isScreenOff.load(std::memory_order_acquire) && !wakeupPending.load(std::memory_order_acquire)) { 
        wakeupPending.store(true, std::memory_order_release); 
        digitalWrite(15, HIGH);
        isScreenOff.store(false, std::memory_order_release);
        wakeupPending.store(false, std::memory_order_release); 
    } 
}

void goToLightSleep() {
    turnScreenOff(); 
    
    dsp_is_paused.store(true, std::memory_order_release);
    while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    
    i2s_channel_disable((i2s_chan_handle_t)tx_chan); 
    i2s_channel_disable((i2s_chan_handle_t)rx_chan);
    adc_continuous_stop(multifx_adc_handle);

    if (dsp_cpu_lock != NULL) esp_pm_lock_release(dsp_cpu_lock);
    
    while ((REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) && 
           (REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN))) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    while (((REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) == 0) || 
           ((REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN)) == 0)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);
    
    adc_continuous_start(multifx_adc_handle);
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE * sizeof(int16_t)); 
    memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE * sizeof(int16_t));
    esp_cache_msync((void*)delayBuffer, MAX_BUFFER_SIZE * sizeof(int16_t), ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE * sizeof(int16_t));
    if(diffuserBuf) memset(diffuserBuf, 0, 1024 * sizeof(float));
    
    i2s_channel_enable((i2s_chan_handle_t)tx_chan); i2s_channel_enable((i2s_chan_handle_t)rx_chan); 
    globalAudioResetRequested.store(true, std::memory_order_release);
    
    std::atomic_thread_fence(std::memory_order_seq_cst);
    dsp_is_paused.store(false, std::memory_order_release);
    while(dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
    
    turnScreenOn();
    pedals.triggerSystemRecovery(); lastActivityTime = millis();
}

void IRAM_ATTR __attribute__((optimize("O3"))) updateLUT() {
    if (pitchShiftLUT_temp == nullptr) return; 
    DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
    float basePitch=0.0f; 
    if(activeDSP->cp || (activeDSP->activeMode==4 && activeDSP->w)) basePitch+=activeDSP->fxMem[4];
    float toeBend=activeDSP->fxMem[0], heelBend=activeDSP->fxMem[1], harmRatioMem=activeDSP->fxMem[3], chorusRatioMem=activeDSP->fxMem[7], vibHzMem=activeDSP->fxMem[9]; 
    int fbIntervalIdxLocal=activeDSP->fbIdx;
    
    for(int i=0; i<16384; i++) {
        float normalizedThrow=(i>=8192)?((float)(i-8192)/8191.0f):((float)(i-8192)/8192.0f); 
        float dynamicBend=(normalizedThrow>=0.0f)?(toeBend*normalizedThrow):(heelBend*fabsf(normalizedThrow));
        pitchShiftLUT_temp[i]=powf(2.0f,(basePitch+dynamicBend)/12.0f); 
        if(i > 0 && i % 2048 == 0) vTaskDelay(1);
    }
    
    float* tempPtr = pitchShiftLUT.load(std::memory_order_relaxed);
    pitchShiftLUT.store(pitchShiftLUT_temp, std::memory_order_release);
    pitchShiftLUT_temp = tempPtr;
    
    globalHarmRatio.store(powf(2.0f, harmRatioMem/12.0f), std::memory_order_release); 
    globalChorusRatio.store(powf(2.0f, chorusRatioMem/12.0f), std::memory_order_release); 
    float fbIntervals[5]={0.0f,12.0f,19.0f,24.0f,28.0f}; 
    globalFbRatio.store(powf(2.0f, fbIntervals[constrain(fbIntervalIdxLocal,0,4)]/12.0f), std::memory_order_release);
    float vibHz=(vibHzMem!=0.0f)?fabsf(vibHzMem):2.0f; 
    globalVibratoPhaseInc.store((vibHz*LFO_LUT_SIZE)/(float)currentSampleRate.load(std::memory_order_acquire), std::memory_order_release);
}

// --- PSRAM UI PRE-RENDER & DOUBLE-BUFFERED TASK ---
void DisplayTask(void * pvParameters) {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    // Allocate PSRAM Working Canvas and Static Background
    spr.setColorDepth(16);
    spr.createSprite(320, 170);
    bgSpr.setColorDepth(16);
    bgSpr.createSprite(320, 170);

    // NULL Pointer Safeguard for PSRAM Sprites
    if (!spr.getPointer() || !bgSpr.getPointer()) {
        Serial.println("Warning: PSRAM Sprite Creation Failed. Falling back to direct TFT rendering.");
    } else {
        bgSpr.fillSprite(TFT_BLACK);
        int cx_top[4] = {40, 120, 200, 280};
        int cy1 = 55;
        int r = 18; 
        
        for(int i=0; i<4; i++) { bgSpr.drawCircle(cx_top[i], cy1, r, TFT_DARKGREY); }
        bgSpr.setTextColor(TFT_WHITE, TFT_BLACK);
        const char* topLabels[] = {"PB1", "PB2", "PB3", "CC11"};
        for(int i=0; i<4; i++) { bgSpr.drawCentreString(topLabels[i], cx_top[i], cy1-28, 1); }
        bgSpr.setTextColor(TFT_DARKGREY, TFT_BLACK);
        bgSpr.drawString("IN", 1, 125, 1);
        bgSpr.drawString("OUT", 300, 125, 1);
    }

    int cx_top[4] = {40, 120, 200, 280};
    int cx_bot[5] = {32, 96, 160, 224, 288};
    int cy1 = 55, cy2 = 105;
    int r = 18;

    char buf[64];
    const int PAR_COUNT[10] = {4, 5, 4, 2, 2, 5, 3, 3, 5, 2};
    int lastRenderMode = -1;

    // Cache Telemetry Values for Dirty-Frame Bypassing
    uint16_t cachePB1=9999, cachePB2=9999, cachePB3=9999, cacheCC11=9999;
    float cacheInLvl=-1.0f, cacheOutLvl=-1.0f, cacheCpu=-1.0f, cacheBatV=-1.0f;
    int cacheBatPct=-1, cacheMode=-1;
    bool cacheBt=false, cacheDead=false;
    uint32_t cacheSR=0;
    bool cacheFxStates[10] = {false};

    for(;;) {
        bool isDead = isBatteryDead.load(std::memory_order_acquire);
        if(__builtin_expect(isDead, 0)) {
            if(cacheDead != isDead) {
                tft.fillScreen(TFT_RED);
                tft.setTextColor(TFT_WHITE);
                tft.drawString("BATTERY DEAD", 40, 60, 4);
                cacheDead = isDead;
            }
            vTaskDelay(portMAX_DELAY);
        }

        if(!isScreenOff.load(std::memory_order_acquire)) {
            DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
            // Defensively constrain activeMode to prevent Out-Of-Bounds Array Read
            int renderMode = constrain(activeDSP->activeMode, 0, 9); 
            float inLvl = ui_audio_level.load(std::memory_order_acquire);
            float outLvl = ui_output_level.load(std::memory_order_acquire);
            float cpuLoad = core1_load.load(std::memory_order_relaxed);
            float batV = currentBatteryVoltage;
            int batPct = currentBatteryPercent.load(std::memory_order_relaxed);
            uint32_t srVal = currentSampleRate.load(std::memory_order_acquire);
            bool btConn = btmidi.isConnected();
            bool fxStates[10] = {activeDSP->w, activeDSP->fz, activeDSP->fb, activeDSP->hr, activeDSP->cp, activeDSP->sy, activeDSP->pd, activeDSP->ch, activeDSP->sw, activeDSP->vb};

            // DIRTY-FRAME CHECK: Skip rendering if no parameters moved
            bool isDirty = (renderMode != cacheMode) || (currentPB1 != cachePB1) || (currentPB2 != cachePB2) || 
                           (currentPB3 != cachePB3) || (currentCC11 != cacheCC11) || (fabsf(inLvl - cacheInLvl) > 0.01f) || 
                           (fabsf(outLvl - cacheOutLvl) > 0.01f) || (fabsf(cpuLoad - cacheCpu) > 1.0f) || 
                           (batPct != cacheBatPct) || (btConn != cacheBt) || (srVal != cacheSR);

            if(!isDirty) {
                for(int i=0; i<10; i++) { if(fxStates[i] != cacheFxStates[i]) { isDirty = true; break; } }
            }

            if(isDirty) {
                // Update Cache
                cacheMode = renderMode; cachePB1 = currentPB1; cachePB2 = currentPB2; cachePB3 = currentPB3;
                cacheCC11 = currentCC11; cacheInLvl = inLvl; cacheOutLvl = outLvl; cacheCpu = cpuLoad;
                cacheBatPct = batPct; cacheBt = btConn; cacheSR = srVal;
                for(int i=0; i<10; i++) cacheFxStates[i] = fxStates[i];

                if (spr.getPointer() && bgSpr.getPointer()) {
                    // FAST PSRAM BLIT: Copy static background to working canvas
                    spr.pushImage(0, 0, 320, 170, (uint16_t*)bgSpr.getPointer());

                    if(renderMode != lastRenderMode) {
                        spr.fillRect(0, 0, 320, 20, TFT_NAVY);
                        spr.fillRect(10, 70, 300, 60, TFT_BLACK); 
                        int activeParams = PAR_COUNT[renderMode];
                        for(int i=0; i<activeParams; i++) { spr.drawCircle(cx_bot[i], cy2, r, TFT_DARKGREY); }
                        lastRenderMode = renderMode;
                    }
                    
                    spr.setTextColor(TFT_WHITE, TFT_NAVY);
                    sprintf(buf, "%4.2fV %3d%%  ", batV, batPct);
                    spr.drawString(buf, 4, 2, 2);
                    sprintf(buf, "   %-10s   ", EFFECT_NAMES[renderMode]);
                    spr.drawCentreString(buf, 160, 2, 2);
                    
                    if (!bleEnabled.load(std::memory_order_relaxed)) {
                        sprintf(buf, "  BT: OFF ");
                    } else {
                        sprintf(buf, "  BT: %-4s", btConn ? "CONN" : "WAIT");
                    }
                    spr.drawString(buf, 240, 2, 2);

                    int inH = (int)(inLvl * 90.0f); if(inH>90) inH=90;
                    int outH = (int)(outLvl * 90.0f); if(outH>90) outH=90;
                    spr.fillRect(2, 30, 6, 90-inH, TFT_BLACK);
                    spr.fillRect(2, 120-inH, 6, inH, TFT_GREEN);
                    spr.fillRect(312, 30, 6, 90-outH, TFT_BLACK);
                    spr.fillRect(312, 120-outH, 6, outH, TFT_GREEN);
                    
                    spr.setTextColor(TFT_CYAN, TFT_BLACK);
                    int pb1_pct = (int)std::roundf(((float)currentPB1 - 8192.0f) / 81.915f);
                    int pb2_pct = (int)std::roundf(((float)currentPB2 - 8192.0f) / 81.915f);
                    int pb3_pct = (int)std::roundf(currentPB3 / 163.83f);
                    int cc11_pct = (int)std::roundf(currentCC11 / 163.83f);
                    
                    sprintf(buf, "%4d%%", pb1_pct); spr.drawCentreString(buf, cx_top[0], cy1-4, 1);
                    sprintf(buf, "%4d%%", pb2_pct); spr.drawCentreString(buf, cx_top[1], cy1-4, 1);
                    sprintf(buf, "%4d%%", pb3_pct); spr.drawCentreString(buf, cx_top[2], cy1-4, 1);
                    sprintf(buf, "%4d%%", cc11_pct); spr.drawCentreString(buf, cx_top[3], cy1-4, 1);

                    spr.setTextColor(TFT_WHITE, TFT_BLACK);
                    const char* botLabels[] = {"PAR 1", "PAR 2", "PAR 3", "PAR 4", "PAR 5"};
                    int activeParams = PAR_COUNT[renderMode];
                    for(int i=0; i<activeParams; i++) { spr.drawCentreString(botLabels[i], cx_bot[i], cy2-28, 1); }

                    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
                    float dispVals[5] = {0.0f};
                    switch(renderMode) {
                        case 0: dispVals[0]=activeDSP->fxMem[1]; dispVals[1]=activeDSP->fxMem[0]; dispVals[2]=activeDSP->params[0][0]; dispVals[3]=activeDSP->params[0][1]; break;
                        case 1: dispVals[0]=activeDSP->fxMem[1]; dispVals[1]=activeDSP->fxMem[0]; dispVals[2]=activeDSP->params[1][0]; dispVals[3]=activeDSP->params[1][1]; dispVals[4]=activeDSP->params[1][2]; break;
                        case 2: dispVals[0]=(float)activeDSP->fbIdx; dispVals[1]=activeDSP->params[2][0]; dispVals[2]=activeDSP->params[2][1]; dispVals[3]=activeDSP->params[2][2]; break;
                        case 3: dispVals[0]=activeDSP->fxMem[3]; dispVals[1]=activeDSP->params[3][0]; break;
                        case 4: dispVals[0]=activeDSP->fxMem[4]; dispVals[1]=activeDSP->fxMem[4]; break;
                        case 5: dispVals[0]=activeDSP->fxMem[5]; dispVals[1]=activeDSP->params[5][0]; dispVals[2]=activeDSP->params[5][1]; dispVals[3]=activeDSP->params[5][2]; dispVals[4]=activeDSP->params[5][3]; break;
                        case 6: dispVals[0]=activeDSP->fxMem[6]; dispVals[1]=activeDSP->params[6][0]; dispVals[2]=activeDSP->params[6][1]; break;
                        case 7: dispVals[0]=activeDSP->fxMem[7]; dispVals[1]=activeDSP->params[7][0]; dispVals[2]=activeDSP->params[7][1]; break;
                        case 8: dispVals[0]=activeDSP->fxMem[1]; dispVals[1]=activeDSP->fxMem[0]; dispVals[2]=activeDSP->params[8][0]; dispVals[3]=activeDSP->params[8][1]; dispVals[4]=activeDSP->params[8][2]; break;
                        case 9: dispVals[0]=activeDSP->fxMem[9]; dispVals[1]=activeDSP->params[9][0]; break;
                    }
                    
                    for(int i=0; i<activeParams; i++) {
                        sprintf(buf, "%6.2f", dispVals[i]); 
                        spr.drawCentreString(buf, cx_bot[i], cy2-4, 1);
                    }

                    const char* fxNamesShort[] = {"[WH]", "[FZ]", "[FB]", "[HR]", "[CP]", "[SY]", "[PD]", "[CH]", "[SW]", "[VB]"};
                    int fxX = 22;
                    for(int i=0; i<10; i++) {
                        spr.setTextColor(fxStates[i] ? TFT_BLACK : TFT_DARKGREY, fxStates[i] ? TFT_GREEN : TFT_BLACK);
                        spr.drawString(fxNamesShort[i], fxX, 138, 1);
                        fxX += 28;
                    }

                    spr.fillRect(0, 155, 320, 15, TFT_DARKGREY);
                    spr.setTextColor(TFT_WHITE, TFT_DARKGREY);
                    sprintf(buf, "CPU:%3d%%  SRM:%3dK  PSR:%4dK  SR:%2dkHz  ", 
                        (int)cpuLoad, 
                        (int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1024), 
                        (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1024),
                        srVal/1000);
                    spr.drawString(buf, 10, 156, 1);

                    // SINGLE CONTIGUOUS SPI DMA PUSH TO TFT
                    spr.pushSprite(0, 0);
                }
            }
        }

        if(__builtin_expect(ui_clear_meters_requested.exchange(false, std::memory_order_acq_rel), 0)) {
            ui_audio_level.store(0.0f, std::memory_order_release);
            ui_output_level.store(0.0f, std::memory_order_release);
        } else if (__builtin_expect(!dsp_is_paused.load(std::memory_order_acquire), 1)) {
            float current_ui_in = ui_audio_level.load(std::memory_order_acquire);
            float current_ui_out = ui_output_level.load(std::memory_order_acquire);
            if(current_ui_in <= 0.02f && current_ui_out <= 0.02f) {
                ui_audio_level.store(0.0f, std::memory_order_release);
                ui_output_level.store(0.0f, std::memory_order_release);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(33)); 
    }
}

inline float IRAM_ATTR __attribute__((always_inline)) __attribute__((optimize("O3"))) AntiDenormal(float value) {
    union { float f; uint32_t i; } u = { .f = value };
    if (__builtin_expect((u.i & 0x7F800000) == 0, 0)) return 0.0f;
    return value;
}

// FRACTIONAL LFO INTERPOLATOR
inline float IRAM_ATTR __attribute__((always_inline)) getLfoInterpolated(float phase) {
    int idx = (int)phase & 1023;
    float frac = phase - (float)idx;
    float v1 = lfoLUT[idx];
    float v2 = lfoLUT[(idx + 1) & 1023];
    return __builtin_fmaf(v2 - v1, frac, v1);
}

// ZERO WAIT-STATE 6-POINT WINDOWED SINC INTERPOLATION (PRIMARY TAP)
inline float IRAM_ATTR __attribute__((hot)) __attribute__((always_inline)) __attribute__((optimize("O3"))) processSincTap(uint32_t tapPhase, const int16_t* sramBuffer, int currentSramWriteIdx, uint32_t windowMask, uint32_t hannIntMult) {
    int T = (tapPhase >> 16) & windowMask; 
    float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
    int effTap = T + 3;
    
    int idx_m2 = (currentSramWriteIdx - effTap + 1 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_m1 = (currentSramWriteIdx - effTap + 2 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_0  = (currentSramWriteIdx - effTap + 3 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_1  = (currentSramWriteIdx - effTap + 4 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_2  = (currentSramWriteIdx - effTap + 5 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_3  = (currentSramWriteIdx - effTap + 6 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    
    float s_m2 = (float)sramBuffer[idx_m2] * 3.0517578125e-5f;
    float s_m1 = (float)sramBuffer[idx_m1] * 3.0517578125e-5f;
    float s_0  = (float)sramBuffer[idx_0]  * 3.0517578125e-5f;
    float s_1  = (float)sramBuffer[idx_1]  * 3.0517578125e-5f;
    float s_2  = (float)sramBuffer[idx_2]  * 3.0517578125e-5f;
    float s_3  = (float)sramBuffer[idx_3]  * 3.0517578125e-5f;
    
    float fm1 = frac + 1.0f;
    float f0  = frac;
    float f1  = 1.0f - frac;
    float f2  = 2.0f - frac;

    float w_m1 = __builtin_fmaf(-0.16666667f * f0, f1 * f2, 0.0f);
    float w_0  = __builtin_fmaf(0.5f * fm1, f1 * f2, 0.0f);
    float w_1  = __builtin_fmaf(0.5f * fm1, f0 * f2, 0.0f);
    float w_2  = __builtin_fmaf(-0.16666667f * fm1, f0 * f1, 0.0f);

    float interpSample = __builtin_fmaf(s_m1, w_m1, __builtin_fmaf(s_0, w_0, __builtin_fmaf(s_1, w_1, s_2 * w_2)));
    
    // 4096-Point Ultra-Resolution Hann Lookup
    int lutIdx = ((uint32_t)(T * hannIntMult) >> 16) & 4095; 
    return AntiDenormal(interpSample * hannLUT[lutIdx]);
}

// 4-POINT 3RD-ORDER HERMITE INTERPOLATOR (SECONDARY TAPS)
inline float IRAM_ATTR __attribute__((hot)) __attribute__((always_inline)) __attribute__((optimize("O3"))) processHermiteTap(uint32_t tapPhase, const int16_t* sramBuffer, int currentSramWriteIdx, uint32_t windowMask, uint32_t hannIntMult) {
    int T = (tapPhase >> 16) & windowMask; 
    float frac = (tapPhase & 0xFFFF) * 0.0000152587890625f; 
    
    int idx_m1 = (currentSramWriteIdx - T + 1 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_0  = (currentSramWriteIdx - T + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_1  = (currentSramWriteIdx - T - 1 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    int idx_2  = (currentSramWriteIdx - T - 2 + SRAM_PITCH_BUF_SIZE) & SRAM_PITCH_BUF_MASK;
    
    float y_m1 = (float)sramBuffer[idx_m1] * 3.0517578125e-5f;
    float y0   = (float)sramBuffer[idx_0]  * 3.0517578125e-5f;
    float y1   = (float)sramBuffer[idx_1]  * 3.0517578125e-5f;
    float y2   = (float)sramBuffer[idx_2]  * 3.0517578125e-5f;
    
    float c0 = y0;
    float c1 = 0.5f * (y1 - y_m1);
    float c2 = y_m1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    float c3 = 0.5f * (y2 - y_m1) + 1.5f * (y0 - y1);
    
    float interpSample = ((c3 * frac + c2) * frac + c1) * frac + c0;
    
    // 4096-Point Ultra-Resolution Hann Lookup
    int lutIdx = ((uint32_t)(T * hannIntMult) >> 16) & 4095; 
    return AntiDenormal(interpSample * hannLUT[lutIdx]);
}

void IRAM_ATTR __attribute__((optimize("O3"))) DSP_ProcessInput(
    int framesRead, int32_t* i2s_in_block, float normFactor, float dc_alpha, 
    float envRetain, float envAttack, float p_sw_thr, float p_sw_att, 
    float p_sw_rel, float srScale, bool swellActive, float localVolGain, 
    float vol_alpha, float& input_dc_offset, float& inputEnvelope, 
    float& localSwellGain, float& smoothedVolGain, float& currentPitch, 
    float targetPitch, float* envBuf, float* masterGainBuf, float* inBuf, float* fzOutBuf) {
    
    if (__builtin_expect(isnan(input_dc_offset) || isinf(input_dc_offset), 0)) input_dc_offset = 0.0f;
    if (__builtin_expect(isnan(inputEnvelope) || isinf(inputEnvelope), 0)) inputEnvelope = 0.0f;

    float pitchInc = 0.0f;
    if (__builtin_expect(fabsf(targetPitch - currentPitch) < 1e-6f, 1)) {
        currentPitch = targetPitch;
    } else {
        float invFrames = 1.0f / (float)framesRead;
        pitchInc = (targetPitch - currentPitch) * invFrames;
    }
    float localPitch = currentPitch;
    float localDC = input_dc_offset;
    float localEnv = inputEnvelope;
    float localSwell = localSwellGain;
    float localSmVol = smoothedVolGain;
    
    #pragma GCC unroll 4
    for(int i = 0; i < framesRead; i++) {
        localPitch += pitchInc;
        
        int32_t raw_sample = i2s_in_block[i * 2];
        int32_t clean_sample = raw_sample & 0xFFFFFF00;
        float raw_in = ((float)clean_sample * normFactor);
        
        localDC = AntiDenormal(__builtin_fmaf(raw_in, dc_alpha, localDC * (1.0f - dc_alpha)));
        float inSample = raw_in - localDC;
        
        localEnv = AntiDenormal(__builtin_fmaf(fabsf(inSample), envAttack, __builtin_fmaf(localEnv, envRetain, 1e-9f)));
        
        if(__builtin_expect(swellActive, 0)) {
            localSwell = (localEnv > p_sw_thr) ? 
                __builtin_fminf(1.0f, __builtin_fmaf(p_sw_att, srScale, localSwell)) : 
                __builtin_fmaxf(0.0f, __builtin_fmaf(-p_sw_rel, srScale, localSwell));
        } else {
            localSwell = __builtin_fminf(1.0f, __builtin_fmaf(0.005f, srScale, localSwell));
        }
        
        localSmVol = __builtin_fmaf(localVolGain, vol_alpha, __builtin_fmaf(localSmVol, (1.0f - vol_alpha), 1e-9f));
        float mGain = localSwell * localSmVol;
        
        envBuf[i] = localEnv;
        masterGainBuf[i] = mGain;
        inBuf[i] = inSample;
        fzOutBuf[i] = 0.0f;
    }
    
    currentPitch = localPitch;
    input_dc_offset = localDC;
    inputEnvelope = localEnv;
    localSwellGain = localSwell;
    smoothedVolGain = localSmVol;
}

void IRAM_ATTR __attribute__((optimize("O3"))) DSP_MixdownAndOutput(
    int framesRead, bool activeGroup, float localFrzRamp, float localFbRamp,
    float g_whammy, float g_dry, float g_base, float g_w2, float g_w3,
    float g_pad, float g_frz, float g_fb,
    float* padFilterBuf, float* dryBuf, float* w1Buf, float* w2Buf,
    float* w3Buf, float* fzOutBuf, float* fbOutBuf, float* sMixBuf,
    float* masterGainBuf, float* inBuf, int32_t* i2s_out_block,
    float& peakInputVal, float& peakOutputVal) {
    
    #pragma GCC ivdep
    #pragma GCC unroll 4
    for(int i = 0; i < framesRead; i++) {
        float b_pad = padFilterBuf[i];
        float b_dry = dryBuf[i];
        float b_w1 = w1Buf[i];
        float b_w2 = w2Buf[i];
        float b_w3 = w3Buf[i];
        float b_fz = fzOutBuf[i];
        float b_fb = fbOutBuf[i];
        
        float sMix = 1e-9f;
        if(__builtin_expect(!activeGroup && localFrzRamp <= 0.0f && localFbRamp <= 0.0f && b_pad <= 0.001f, 0)) {
             sMix = b_dry;
         } else {
             float baseSignal = __builtin_fmaf(b_w1, g_whammy, b_dry * g_dry);
             sMix = __builtin_fmaf(baseSignal, g_base, sMix);
             sMix = __builtin_fmaf(b_w2, g_w2, sMix);
             sMix = __builtin_fmaf(b_w3, g_w3, sMix);
             sMix = __builtin_fmaf(b_pad, g_pad, sMix);
             sMix = __builtin_fmaf(b_fz, g_frz, sMix);
             sMix = __builtin_fmaf(b_fb, g_fb, sMix);
            
             // POLYNOMIAL SOFT-CLIPPING TUBE SATURATOR
             float x = __builtin_fmaxf(-1.15f, __builtin_fminf(sMix * 0.85f, 1.15f)); 
             sMix = __builtin_fmaf(1.5f, x, -0.5f * x * x * x);
         }
        sMixBuf[i] = sMix;
    }
    dsps_mul_f32(sMixBuf, masterGainBuf, sMixBuf, framesRead, 1, 1, 1);
    
    float localPeakIn = peakInputVal;
    float localPeakOut = peakOutputVal;
    
    #pragma GCC unroll 4
    for(int i = 0; i < framesRead; i++) {
        float b_in = inBuf[i];
        float b_mix = sMixBuf[i];
        
        float abs_in = fabsf(b_in);
        float abs_out = fabsf(b_mix);
        
        if(__builtin_expect(abs_in > localPeakIn, 0)) localPeakIn = abs_in;
        if(__builtin_expect(abs_out > localPeakOut, 0)) localPeakOut = abs_out;
        
        float clamped_mix = __builtin_fmaxf(-0.999f, __builtin_fminf(b_mix, 0.999f));
        int32_t finalOut = (int32_t)(clamped_mix * 2147483520.0f);
        finalOut &= 0xFFFFFF00;
        
        i2s_out_block[i * 2] = finalOut;
        i2s_out_block[i * 2 + 1] = finalOut;
    }
    
    peakInputVal = localPeakIn;
    peakOutputVal = localPeakOut;
}

void IRAM_ATTR __attribute__((optimize("O3"))) AudioDSPTask(void * pvParameters) {
    float input_dc_offset=0.0f, synthEnv=0.0f, synthFilter=0.0f, synthBandpass=0.0f, padFilter=0.0f, padEnv=0.0f, inputEnvelope=0.0f, feedbackFilterVar=0.0f, smoothedVolGain=1.0f, currentPitch=1.0f, fbOutNode=0.0f, smoothed_delay_samples=0.0f;
    float dampState = 0.0f; 
    uint32_t wowRng = 123456789;
    float wowState = 0.0f;

    bool wasFeedbackActive=false; int freezeWriteIdxVar=0, freezePlayCounterVar=0, freezeStartIdxVar=0, activeFreezeLength=48000;
    float c_fx[10][5] = {0.0f}; int c_lat=0, c_act=0;
    bool c_w=true, c_fz=false, c_fb=false, c_hr=false, c_cp=false, c_sy=false, c_pd=false, c_ch=false, c_sw=false, c_vb=false; float c_vg=1.0f;
    const float normFactor=1.0f/2147483648.0f, DC_OFFSET=1e-9f;
    
    for(;;) {
        if(__builtin_expect(dsp_is_paused.load(std::memory_order_acquire), 0)) {
            dsp_ack_parked.store(true, std::memory_order_release);
            while(dsp_is_paused.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(2)); }
            dsp_ack_parked.store(false, std::memory_order_release);
        }
        
        size_t bytesRead; 
        i2s_channel_read((i2s_chan_handle_t)rx_chan, i2s_in_block, HOP_SIZE*2*sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(10));
        
        if(__builtin_expect(bytesRead>0, 1)) {
            int framesRead=bytesRead/8;
            if(__builtin_expect(framesRead == HOP_SIZE, 1)) {
                if(__builtin_expect(panicResetRequested.load(std::memory_order_acquire), 0)) {
                    while(!dma_transfer_done.load(std::memory_order_acquire)) { std::atomic_thread_fence(std::memory_order_acquire); __asm__ __volatile__ ("nop"); }
                    memset(dmaPingBuffer, 0, sizeof(dmaPingBuffer));
                    memset(dmaPongBuffer, 0, sizeof(dmaPongBuffer));
                    synthEnv=0.0f; synthFilter=0.0f; synthBandpass=0.0f; padFilter=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate.load(std::memory_order_acquire); fbDelayWriteIdx=0; apfNeedsClear=true; freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f; dampState=0.0f; wowState=0.0f; diffuserIdx=0; if(diffuserBuf) memset(diffuserBuf, 0, 1024*sizeof(float));
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16; tap_w1_1=0; tap_w1_2=halfWinFixed; tap_w2_1=0; tap_w2_2=halfWinFixed; tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    panicResetRequested.store(false, std::memory_order_release);
                }
                if(__builtin_expect(globalAudioResetRequested.load(std::memory_order_acquire), 0)) {
                    while(!dma_transfer_done.load(std::memory_order_acquire)) { std::atomic_thread_fence(std::memory_order_acquire); __asm__ __volatile__ ("nop"); }
                    
                    synthEnv=0.0f; synthFilter=0.0f; synthBandpass=0.0f; padFilter=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; smoothedVolGain=volumePedalGain; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate.load(std::memory_order_acquire); fbDelayWriteIdx=0; writeIndex=0; sramWriteIdx=0; apfNeedsClear=true; input_dc_offset=0.0f; ui_audio_level.store(0.0f, std::memory_order_release); ui_output_level.store(0.0f, std::memory_order_release); freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f; dampState=0.0f; wowState=0.0f; diffuserIdx=0; if(diffuserBuf) memset(diffuserBuf, 0, 1024*sizeof(float));
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16; tap_w1_1=0; tap_w1_2=halfWinFixed; tap_w2_1=0; tap_w2_2=halfWinFixed; tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    
                    memset(dmaPingBuffer, 0, sizeof(dmaPingBuffer));
                    memset(dmaPongBuffer, 0, sizeof(dmaPongBuffer));
                    
                    globalAudioResetRequested.store(false, std::memory_order_release); smoothed_delay_samples=0.0f; 
                    int targetMute = (currentSampleRate.load(std::memory_order_acquire)/HOP_SIZE)*0.40f;
                    if(hardwareSyncMuteFrames.load(std::memory_order_acquire) < 10) hardwareSyncMuteFrames.store(targetMute, std::memory_order_release);
                }
                
                int currentMute = hardwareSyncMuteFrames.load(std::memory_order_acquire);
                bool isMuted = false;
                if(__builtin_expect(currentMute > 0, 0)) {
                     hardwareSyncMuteFrames.store(currentMute - 1, std::memory_order_release);
                     isMuted = true;
                 }
                 
                uint32_t start_cycles=xthal_get_ccount(); float srScale=48000.0f/(float)currentSampleRate.load(std::memory_order_acquire);
                
                DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                for(int j=0; j<10; j++) for(int k=0; k<5; k++) c_fx[j][k] = activeDSP->params[j][k]; 
                c_lat = activeDSP->latMode; 
                c_act = activeDSP->activeMode; 
                c_w = activeDSP->w; c_fz = activeDSP->fz; c_fb = activeDSP->fb; c_hr = activeDSP->hr; 
                c_cp = activeDSP->cp; c_sy = activeDSP->sy; c_pd = activeDSP->pd; c_ch = activeDSP->ch; 
                c_sw = activeDSP->sw; c_vb = activeDSP->vb; c_vg = activeDSP->vg;

                dspAckCommit.store(true, std::memory_order_release); 
                
                float c_pt = pitchShiftFactor.load(std::memory_order_acquire);
                
                float targetWindow = LATENCY_WINDOWS[c_lat];
                if(__builtin_expect(currentWindowSize!=targetWindow, 0)) { currentWindowSize=targetWindow; uint32_t halfWindowFixed=((uint32_t)targetWindow/2)<<16; tap_w1_1=0; tap_w1_2=halfWindowFixed; tap_w2_1=0; tap_w2_2=halfWindowFixed; tap_w3_1=0; tap_w3_2=halfWindowFixed; tap_w4_1=0; tap_w4_2=halfWindowFixed; tap_w5_1=0; tap_w5_2=halfWindowFixed; }
                
                // 4096-Point Hann Window Scaling
                uint32_t hannIntMult=(4096U<<16)/(uint32_t)currentWindowSize, windowMask=(uint32_t)currentWindowSize-1; float p_w_dry=c_fx[0][0], p_w_wet=c_fx[0][1], p_fz_apf=c_fx[1][0], p_fz_att=c_fx[1][1], p_fz_rel=c_fx[1][2], p_fb_spd=c_fx[2][0], p_fb_drv=c_fx[2][1], p_fb_off=c_fx[2][2], p_hr_mix=c_fx[3][0], p_sy_att=c_fx[5][0], p_sy_rel=c_fx[5][1], p_sy_flt=c_fx[5][2], p_sy_mix=c_fx[5][3], p_pd_sm=c_fx[6][0], p_pd_mix=c_fx[6][1], p_ch_spd=c_fx[7][0], p_ch_mix=c_fx[7][1], p_sw_thr=c_fx[8][0], p_sw_att=c_fx[8][1], p_sw_rel=c_fx[8][2], p_vb_dep=c_fx[9][0]; float chorusPhaseIncr=p_ch_spd/(float)currentSampleRate.load(std::memory_order_acquire), feedbackPhaseIncr=p_fb_spd/(float)currentSampleRate.load(std::memory_order_acquire), targetPitch=c_pt;
                bool frzActive=((c_act==1&&c_w)||c_fz);
                if(__builtin_expect(frzActive && !wasFrozen, 0)) { freezePlayCounterVar=0; int bestStart=freezeWriteIdxVar, tempIdx=freezeWriteIdxVar; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { bestStart=tempIdx; break; } tempIdx=prev; } freezeStartIdxVar=bestStart; activeFreezeLength=freezeLength; int searchEnd=bestStart-1; if(searchEnd<0) searchEnd+=freezeLength; tempIdx=searchEnd; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { activeFreezeLength=s; break; } tempIdx=prev; } if(activeFreezeLength<64) activeFreezeLength=freezeLength; }
                if(__builtin_expect(!frzActive && wasFrozen, 0)) apfNeedsClear=true; wasFrozen=frzActive; float activeInvFreqLength=1.0f/(float)activeFreezeLength; bool synthActive=((c_act==5&&c_w)||c_sy), padActive=((c_act==6&&c_w)||c_pd), harmActive=((c_act==3&&c_w)||c_hr), swellActive=((c_act==8&&c_w)||c_sw), chorusActive=((c_act==7&&c_w)||c_ch), feedbackActive=((c_act==2&&c_w)||c_fb);
                if(__builtin_expect(feedbackActive && !wasFeedbackActive, 0)) { fbOutNode=0.0f; fbHpfState=0.0f; feedbackFilterVar=0.0f; } wasFeedbackActive=feedbackActive; bool vibratoActive=((c_act==9&&c_w)||c_vb), capoActive=((c_act==4&&c_w)||c_cp);
                float peakInputVal=0.0f, peakOutputVal=0.0f, localSwellGain=swellGain, localVolGain=c_vg, localFrzRamp=freezeRamp, localFbRamp=feedbackRamp, pdSmCoeff=powf(p_pd_sm, srScale), target_delay=constrain((float)(currentSampleRate.load(std::memory_order_acquire)*p_fb_off), 0.0f, (float)(FB_BUFFER_SIZE-1)); smoothed_delay_samples+= (target_delay-smoothed_delay_samples)*0.01f*srScale+DC_OFFSET; int delaySamples=(int)smoothed_delay_samples; float fbHpfCoeff=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.025f:0.05f, fbLpfCoeff=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.05f:0.1f, fbLpfRetain=1.0f-fbLpfCoeff, dc_alpha=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.0005f:0.001f; int halfWindow=(int)currentWindowSize/2; bool activeGroup=c_w||harmActive||chorusActive||feedbackActive||synthActive||padActive||frzActive||vibratoActive||capoActive, dryGroup=chorusActive||padActive||frzActive||feedbackActive||(localFrzRamp>0.0f)||(localFbRamp>0.0f), repeatGroup=capoActive||synthActive||vibratoActive||padActive||harmActive;
                float g_base=0.0f; if(dryGroup) { if(!repeatGroup) g_base=0.4f; } else if(harmActive) g_base=0.5f; else g_base=1.0f; float g_w2=harmActive?p_hr_mix:0.0f, g_w3=chorusActive?p_ch_mix:0.0f; bool padIsAudible=padActive||(fabsf(padFilter)>0.001f); float g_pad=padIsAudible?p_pd_mix:0.0f, g_frz=(!frzActive&&localFrzRamp>0.0f)?0.5f:0.0f, g_fb=(feedbackActive||localFbRamp>0.0f)?0.6f:0.0f, g_whammy=c_w?p_w_wet:0.0f, g_dry=c_w?p_w_dry:1.0f, vol_alpha=0.01f*srScale, meter_decay=(currentSampleRate.load(std::memory_order_acquire)==96000)?0.999f:0.998f, envRetain=powf(0.99f,srScale), envAttack=1.0f-envRetain;
                
                // HARDENED DSP SANITIZATION: Validate both synth state variables to prevent NaN pollution
                if(__builtin_expect(isnan(synthFilter)||isinf(synthFilter), 0)) synthFilter=0.0f; 
                if(__builtin_expect(isnan(synthBandpass)||isinf(synthBandpass), 0)) synthBandpass=0.0f;
                if(__builtin_expect(isnan(padFilter)||isinf(padFilter), 0)) padFilter=0.0f; 
                if(__builtin_expect(isnan(feedbackFilterVar)||isinf(feedbackFilterVar), 0)) feedbackFilterVar=0.0f; 
                if(__builtin_expect(isnan(fbHpfState)||isinf(fbHpfState), 0)) fbHpfState=0.0f; 
                
                float localVibPhase=vibratoLfoPhase, localChoPhase=chorusLfoPhase, localFbPhase=feedbackLfoPhase, localFbHpf=fbHpfState; 
                
                int currentDryIdx = (writeIndex - halfWindow + MAX_BUFFER_SIZE) & BUFFER_MASK;
                int nextDryIdx = (currentDryIdx + HOP_SIZE) & BUFFER_MASK;
                
                dma_transfer_done.store(false, std::memory_order_release);
                if (__builtin_expect(MAX_BUFFER_SIZE - nextDryIdx >= HOP_SIZE, 1)) {
                    esp_cache_msync((void*)&delayBuffer[nextDryIdx], HOP_SIZE * sizeof(int16_t), ESP_CACHE_MSYNC_FLAG_DIR_C2M);
                    if (esp_async_memcpy(dma_memcpy_handle, activeDmaWriteBuf, &delayBuffer[nextDryIdx], HOP_SIZE * sizeof(int16_t), dma_memcpy_cb, (void*)&dma_transfer_done) != ESP_OK) {
                        for(int i = 0; i < HOP_SIZE; i++) activeDmaWriteBuf[i] = delayBuffer[(nextDryIdx + i) & BUFFER_MASK];
                        dma_transfer_done.store(true, std::memory_order_release);
                    }
                } else {
                    for(int i = 0; i < HOP_SIZE; i++) activeDmaWriteBuf[i] = delayBuffer[(nextDryIdx + i) & BUFFER_MASK];
                    dma_transfer_done.store(true, std::memory_order_release);
                }
                
                int prefetchIdxFB = (fbDelayWriteIdx - delaySamples + FB_BUFFER_SIZE) & FB_BUFFER_MASK; 
                int aheadFB = (prefetchIdxFB + 32) & FB_BUFFER_MASK; 
                __builtin_prefetch(&fbDelayBuffer[aheadFB], 0, 3);
                
                DSP_ProcessInput(
                    framesRead, i2s_in_block, normFactor, dc_alpha, envRetain, envAttack, 
                    p_sw_thr, p_sw_att, p_sw_rel, srScale, swellActive, localVolGain, vol_alpha, 
                    input_dc_offset, inputEnvelope, localSwellGain, smoothedVolGain, 
                    currentPitch, targetPitch, envBuf, masterGainBuf, inBuf, fzOutBuf
                );
                
                // CHAMBERLIN STATE-VARIABLE FILTER (SYNTH)
                if(__builtin_expect(synthActive, 0)) for(int i=0; i<framesRead; i++) { 
                    synthEnv=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,__builtin_fmaf(p_sy_att, srScale, synthEnv)):__builtin_fmaxf(0.0f,__builtin_fmaf(-p_sy_rel, srScale, synthEnv)); 
                    float clampedProc=__builtin_fmaxf(-1.0f,__builtin_fminf(inBuf[i],1.0f)); 
                    int waveIdx=(int)((clampedProc+1.0f)*1023.5f) & 2047; 
                    float procSample=synthLUT[waveIdx]; 
                    
                    float f1 = __builtin_fmaxf(0.001f, __builtin_fminf(0.45f, __builtin_fmaf(0.5f * synthEnv, srScale, p_sy_flt * srScale)));
                    float q1 = 0.5f; 
                    float hp = procSample - synthFilter - (q1 * synthBandpass);
                    synthFilter = AntiDenormal(synthFilter + f1 * synthBandpass + DC_OFFSET);
                    synthBandpass = AntiDenormal(synthBandpass + f1 * hp + DC_OFFSET);
                    inBuf[i] = synthFilter * p_sy_mix; 
                }
                if(__builtin_expect(padActive, 0)) for(int i=0; i<framesRead; i++) { padEnv=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,__builtin_fmaf(0.00002f, srScale, padEnv)):__builtin_fmaxf(0.0f,__builtin_fmaf(-0.000005f, srScale, padEnv)); inBuf[i]*=padEnv; }
                
                while(!dma_transfer_done.load(std::memory_order_acquire)) { std::atomic_thread_fence(std::memory_order_acquire); __asm__ __volatile__ ("nop"); }
                
                for(int i=0; i<framesRead; i++) {
                    float procSample=inBuf[i]; if(__builtin_expect(!frzActive, 1)) { freezeBuffer[freezeWriteIdxVar]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(procSample,1.0f))*32767.0f); freezeWriteIdxVar++; if(freezeWriteIdxVar>=freezeLength) freezeWriteIdxVar=0; }
                    if(__builtin_expect(localFrzRamp>0.0f||frzActive, 0)) localFrzRamp=frzActive?__builtin_fminf(1.0f,__builtin_fmaf(p_fz_att, srScale, localFrzRamp)):__builtin_fmaxf(0.0f,__builtin_fmaf(-p_fz_rel, srScale, localFrzRamp));
                    if(__builtin_expect(localFrzRamp>0.0f, 0)) {
                        float phaseRead=(float)freezePlayCounterVar*activeInvFreqLength, phase2=(phaseRead+0.5f); if(phase2>=1.0f) phase2-=1.0f;
                        int idx1 = freezeStartIdxVar + freezePlayCounterVar;
                        if (__builtin_expect(idx1 >= freezeLength, 0)) idx1 -= freezeLength;
                        int activeLen = (activeFreezeLength >= 64) ? activeFreezeLength : freezeLength;
                        int counter2 = freezePlayCounterVar + (activeLen / 2);
                        if (__builtin_expect(counter2 >= activeLen, 0)) counter2 -= activeLen;
                        int idx2 = freezeStartIdxVar + counter2;
                        if (__builtin_expect(idx2 >= freezeLength, 0)) idx2 -= freezeLength;
                        
                        // 4096-Point Hann Window Lookup
                        int lutIdx1=(int)(phaseRead*4095.0f)&4095, lutIdx2=(int)(phase2*4095.0f)&4095;
                        float rFrz=__builtin_fmaf((float)freezeBuffer[idx1]*3.0517578125e-5f, hannLUT[lutIdx1], (float)freezeBuffer[idx2]*3.0517578125e-5f*hannLUT[lutIdx2]);
                        float d1=apf1Buffer[apf1Idx], next_apf1=__builtin_fmaf(p_fz_apf, d1, rFrz+DC_OFFSET), a1=__builtin_fmaf(-p_fz_apf, rFrz, d1); apf1Buffer[apf1Idx]=next_apf1; apf1Idx++; if(apf1Idx>=1009) apf1Idx=0; float d2=apf2Buffer[apf2Idx], next_apf2=__builtin_fmaf(p_fz_apf, d2, a1+DC_OFFSET), a2=__builtin_fmaf(-p_fz_apf, a1, d2); apf2Buffer[apf2Idx]=next_apf2; apf2Idx++; if(apf2Idx>=863) apf2Idx=0; fzOutBuf[i]=a2*localFrzRamp; freezePlayCounterVar++; if(freezePlayCounterVar>=activeFreezeLength) freezePlayCounterVar=0;
                    } else if(__builtin_expect(apfNeedsClear, 0)) { memset(apf1Buffer,0,sizeof(apf1Buffer)); memset(apf2Buffer,0,sizeof(apf2Buffer)); apf1Idx=0; apf2Idx=0; apfNeedsClear=false; }
                    
                    float delayIn=(localFrzRamp>0.0f)?__builtin_fmaf(procSample, (1.0f-localFrzRamp), fzOutBuf[i]):procSample; 
                    
                    int16_t sample16 = (int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(delayIn,1.0f))*32767.0f);
                    delayBuffer[writeIndex] = sample16;
                    sramPitchBuffer[sramWriteIdx] = sample16; 
                    
                    // FRACTIONAL LFO INTERPOLATION
                    float spd1=currentPitch; 
                    if(__builtin_expect(vibratoActive, 0)) { 
                        localVibPhase+=globalVibratoPhaseInc.load(std::memory_order_acquire); 
                        if(localVibPhase>=LFO_LUT_SIZE) localVibPhase-=LFO_LUT_SIZE; 
                        spd1*=1.0f+((getLfoInterpolated(localVibPhase)-1.0f)*p_vb_dep); 
                    }
                    float spd2=currentPitch*globalHarmRatio.load(std::memory_order_acquire);
                    float spd3=currentPitch*globalChorusRatio.load(std::memory_order_acquire); 
                    if(__builtin_expect(chorusActive, 0)) { 
                        localChoPhase+=chorusPhaseIncr; 
                        if(localChoPhase>=LFO_LUT_SIZE) localChoPhase-=LFO_LUT_SIZE; 
                        spd3*=getLfoInterpolated(localChoPhase); 
                    } 
                    float spd4=1.0f, spd5=1.0f;
                    
                    if(__builtin_expect(feedbackActive||localFbRamp>0.0f, 0)) {
                        localFbPhase+=feedbackPhaseIncr; 
                        if(localFbPhase>=LFO_LUT_SIZE) localFbPhase-=LFO_LUT_SIZE; 
                        float lfoVal=getLfoInterpolated(localFbPhase); 
                        spd4=lfoVal; spd5=currentPitch*globalFbRatio.load(std::memory_order_acquire)*lfoVal;
                        
                        // TAPE WOW & FLUTTER INJECTION
                        wowRng = wowRng * 1664525U + 1013904223U;
                        float rawNoise = ((float)(wowRng & 0xFFFF) * 0.0000305185f) - 1.0f;
                        wowState = AntiDenormal(__builtin_fmaf(rawNoise - wowState, 0.0005f * srScale, wowState));
                        float wowMod = 1.0f + (wowState * 0.0015f);
                        spd4 *= wowMod; spd5 *= wowMod;

                        // 4-POINT HERMITE INTERPOLATION FOR SECONDARY TAPS
                        float w4=processHermiteTap(tap_w4_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult)+processHermiteTap(tap_w4_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult);
                        float w5=processHermiteTap(tap_w5_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult)+processHermiteTap(tap_w5_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult);
                        
                        if(feedbackActive) localFbRamp=(envBuf[i]>0.005f)?__builtin_fminf(1.0f,__builtin_fmaf(0.000011f, srScale, localFbRamp)):__builtin_fmaxf(0.0f,__builtin_fmaf(-0.0005f, srScale, localFbRamp)); else localFbRamp=__builtin_fmaxf(0.0f,__builtin_fmaf(-0.0001f, srScale, localFbRamp));
                        float mixV=__builtin_fmaxf(0.0f,__builtin_fminf((localFbRamp-0.1f)*2.0f,1.0f)), feedInput=(frzActive&&localFrzRamp>0.0f)?fzOutBuf[i]:__builtin_fmaf(w4, (1.0f-mixV), __builtin_fmaf(w5, mixV, fbOutNode*0.95f)); 
                        localFbHpf=AntiDenormal(__builtin_fmaf(fbHpfCoeff, (feedInput-localFbHpf), DC_OFFSET)); 
                        float rawDrive=(feedInput-localFbHpf)*p_fb_drv, boundedDrive=__builtin_fmaxf(-1.5f,__builtin_fminf(rawDrive,1.5f)), gainDrive=boundedDrive*(1.0f-(0.15f*boundedDrive*boundedDrive)); feedbackFilterVar=AntiDenormal(__builtin_fmaf(gainDrive, fbLpfCoeff, __builtin_fmaf(feedbackFilterVar, fbLpfRetain, DC_OFFSET))); float satFb=feedbackFilterVar*(localFbRamp*localFbRamp*localFbRamp)*0.85f; fbDelayBuffer[fbDelayWriteIdx]=(int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(satFb,1.0f))*32767.0f);
                        int fbReadIdx=(fbDelayWriteIdx-delaySamples+FB_BUFFER_SIZE)&FB_BUFFER_MASK; fbOutNode=AntiDenormal((float)fbDelayBuffer[fbReadIdx]*3.0517578125e-5f); fbDelayWriteIdx=(fbDelayWriteIdx+1)&FB_BUFFER_MASK;
                    } else { fbDelayBuffer[fbDelayWriteIdx]=0; fbOutNode=0.0f; fbDelayWriteIdx=(fbDelayWriteIdx+1)&FB_BUFFER_MASK; }
                    
                    // PRIMARY PITCH (WHAMMY - 6-POINT SINC)
                    float rawW1 = processSincTap(tap_w1_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult)+processSincTap(tap_w1_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult); 
                    
                    // DYNAMIC ADAPTIVE DAMPING
                    float dampCutoff = (currentPitch > 1.498f) ? __builtin_fmaxf(0.1f, 1.0f - (currentPitch - 1.498f) * 0.5f) : 1.0f;
                    dampState = AntiDenormal(__builtin_fmaf(dampCutoff, (rawW1 - dampState), dampState));
                    w1Buf[i] = dampState;
                    
                    w2Buf[i]=0.0f; if(__builtin_expect(harmActive, 0)) w2Buf[i]=processHermiteTap(tap_w2_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult)+processHermiteTap(tap_w2_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult); 
                    w3Buf[i]=0.0f; if(__builtin_expect(chorusActive, 0)) w3Buf[i]=processHermiteTap(tap_w3_1,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult)+processHermiteTap(tap_w3_2,sramPitchBuffer,sramWriteIdx,windowMask,hannIntMult);
                    
                    int32_t step1=(int32_t)((1.0f-spd1)*65536.0f); tap_w1_1+=step1; tap_w1_2+=step1; int32_t step2=(int32_t)((1.0f-spd2)*65536.0f); tap_w2_1+=step2; tap_w2_2+=step2; int32_t step3=(int32_t)((1.0f-spd3)*65536.0f); tap_w3_1+=step3; tap_w3_2+=step3; int32_t step4=(int32_t)((1.0f-spd4)*65536.0f); tap_w4_1+=step4; tap_w4_2+=step4; int32_t step5=(int32_t)((1.0f-spd5)*65536.0f); tap_w5_1+=step5; tap_w5_2+=step5;
                    
                    if(__builtin_expect(padActive, 0)) padFilter=AntiDenormal(__builtin_fmaf(padFilter, pdSmCoeff, __builtin_fmaf(w1Buf[i], (1.0f-pdSmCoeff), DC_OFFSET))); else padFilter=AntiDenormal(__builtin_fmaf(padFilter, pdSmCoeff, DC_OFFSET)); 
                    
                    // 1024-SAMPLE SPATIAL DIFFUSION WITH DENORMAL PROTECTION
                    float pad_out = padFilter;
                    float diffIn = w3Buf[i] + pad_out;
                    float diffOut = diffuserBuf ? diffuserBuf[diffuserIdx] : 0.0f;
                    float diffNext = AntiDenormal(diffIn + 0.6f * diffOut);
                    if(diffuserBuf) diffuserBuf[diffuserIdx] = diffNext;
                    float diffFinal = AntiDenormal(diffOut - 0.6f * diffNext);
                    diffuserIdx = (diffuserIdx + 1) & 1023;
                    
                    if(chorusActive && padActive) {
                        w3Buf[i] = diffFinal * 0.5f;
                        padFilterBuf[i] = diffFinal * 0.5f;
                    } else if(chorusActive) {
                        w3Buf[i] = diffFinal;
                        padFilterBuf[i] = pad_out;
                    } else if(padActive) {
                        w3Buf[i] = 0.0f;
                        padFilterBuf[i] = diffFinal;
                    } else {
                        padFilterBuf[i] = pad_out;
                    }

                    dryBuf[i] = (float)activeDmaReadBuf[i] * 3.0517578125e-5f; 
                    fbOutBuf[i]=fbOutNode; 
                    
                    writeIndex=(writeIndex+1)&BUFFER_MASK;
                    sramWriteIdx=(sramWriteIdx+1)&SRAM_PITCH_BUF_MASK;
                }
                vibratoLfoPhase=localVibPhase; chorusLfoPhase=localChoPhase; feedbackLfoPhase=localFbPhase; fbHpfState=localFbHpf;
                
                int16_t* tempDmaPtr = activeDmaReadBuf;
                activeDmaReadBuf = activeDmaWriteBuf;
                activeDmaWriteBuf = tempDmaPtr;

                DSP_MixdownAndOutput(
                    framesRead, activeGroup, localFrzRamp, localFbRamp,
                    g_whammy, g_dry, g_base, g_w2, g_w3, g_pad, g_frz, g_fb,
                    padFilterBuf, dryBuf, w1Buf, w2Buf, w3Buf, fzOutBuf, fbOutBuf,
                    sMixBuf, masterGainBuf, inBuf, i2s_out_block, 
                    peakInputVal, peakOutputVal
                );
                swellGain=localSwellGain; freezeRamp=localFrzRamp; feedbackRamp=localFbRamp;
                
                if (__builtin_expect(ui_clear_meters_requested.load(std::memory_order_acquire), 0)) {
                    ui_audio_level.store(0.0f, std::memory_order_release);
                    ui_output_level.store(0.0f, std::memory_order_release);
                    ui_clear_meters_requested.store(false, std::memory_order_release);
                } else {
                    float current_in = ui_audio_level.load(std::memory_order_acquire);
                    if(peakInputVal > current_in) {
                        ui_audio_level.store(peakInputVal, std::memory_order_release);
                    } else {
                        current_in *= meter_decay;
                        ui_audio_level.store((current_in < 1e-5f) ? 0.0f : current_in, std::memory_order_release);
                    }
                    float current_out = ui_output_level.load(std::memory_order_acquire);
                    if(peakOutputVal > current_out) {
                        ui_output_level.store(peakOutputVal, std::memory_order_release);
                    } else {
                        current_out *= meter_decay;
                        ui_output_level.store((current_out < 1e-5f) ? 0.0f : current_out, std::memory_order_release);
                    }
                }
                
                uint32_t end_timer=xthal_get_ccount(); 
                float max_cycles = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? (2500.0f * (float)framesRead) : (5000.0f * (float)framesRead);
                float currentLoadPercentage=((float)(end_timer-start_cycles)/max_cycles)*100.0f; 
                core1_load.store(__builtin_fmaf(core1_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f,currentLoadPercentage)*0.05f), std::memory_order_relaxed);
                
                if(__builtin_expect(isMuted, 0)) {
                    memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                }
                
                size_t bytesWrittenCount; i2s_channel_write((i2s_chan_handle_t)tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20));
            } else {
                vTaskDelay(pdMS_TO_TICKS(1)); 
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }
    }
}

void updateParameterFromCC(uint8_t cc, uint8_t val) {
    float norm=(float)val/127.0f; int pIdx=cc-24;
    
    int currentMode = activeEffectMode.load(std::memory_order_acquire);
    
    if(currentMode==0) { 
        if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==2) fxParams[0][0]=norm; 
        if(pIdx==3) fxParams[0][1]=norm; 
    }
    else if(currentMode==1) { 
        if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==2) fxParams[1][0]=0.0f+(norm*0.95f); 
        if(pIdx==3) fxParams[1][1]=0.00001f+(norm*0.001f); 
        if(pIdx==4) fxParams[1][2]=0.00001f+(norm*0.0005f); 
    }
    else if(currentMode==2) { 
        if(pIdx==0) { 
            int newIdx=constrain((int)roundf(norm*4.0f),0,4); 
            if(newIdx!=feedbackIntervalIdx.load(std::memory_order_acquire)) { 
                feedbackIntervalIdx.store(newIdx, std::memory_order_release); lutNeedsUpdate=true; 
            } 
        } 
        if(pIdx==1) fxParams[2][0]=1000.0f+(norm*10000.0f); 
        if(pIdx==2) fxParams[2][1]=1.0f+(norm*100.0f); 
        if(pIdx==3) fxParams[2][2]=0.005f+(norm*0.045f); 
    }
    else if(currentMode==3) { 
        if(pIdx==0) { effectMemory[3]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) fxParams[3][0]=norm; 
    }
    else if(currentMode==4) { 
        if(pIdx==0) { 
            int cents=(int)roundf((effectMemory[4]-(float)roundf(effectMemory[4]))*100.0f); 
            effectMemory[4]=constrain(roundf((norm*48.0f)-24.0f)+((float)cents/100.0f),-24.0f,24.0f); lutNeedsUpdate=true; 
        } 
        if(pIdx==1) { 
            int semi=(int)roundf(effectMemory[4]); float c=roundf((norm*100.0f)-50.0f)/100.0f; 
            effectMemory[4]=constrain((float)semi+c,-24.0f,24.0f); lutNeedsUpdate=true; 
        } 
    }
    else if(currentMode==5) { 
        if(pIdx==0) { effectMemory[5]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) fxParams[5][0]=0.01f+(norm*0.5f); 
        if(pIdx==2) fxParams[5][1]=0.001f+(norm*0.05f); 
        if(pIdx==3) fxParams[5][2]=0.1f+(norm*0.8f); 
        if(pIdx==4) fxParams[5][3]=norm; 
    }
    else if(currentMode==6) { 
        if(pIdx==0) { effectMemory[6]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) fxParams[6][0]=0.8f+(norm*0.199f); 
        if(pIdx==2) fxParams[6][1]=norm*3.0f; 
    }
    else if(currentMode==7) { 
        if(pIdx==0) { effectMemory[7]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) fxParams[7][0]=500.0f+(norm*4500.0f); 
        if(pIdx==2) fxParams[7][1]=norm; 
    }
    else if(currentMode==8) { 
        if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==2) fxParams[8][0]=0.001f+(norm*0.05f); 
        if(pIdx==3) fxParams[8][1]=0.00001f+(norm*0.001f); 
        if(pIdx==4) fxParams[8][2]=0.00001f+(norm*0.0005f); 
    }
    else if(currentMode==9) { 
        if(pIdx==0) { effectMemory[9]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
        if(pIdx==1) fxParams[9][0]=norm*2.0f; 
    }
    
    dspNeedsCommit = true;
    settingsNeedSaving=true; lastParameterChangeTime=millis();
}

bool channelMessageCallback(ChannelMessage cm) {
    lastActivityTime=millis();
    if(cm.header==0xB0) {
        if(cm.data1==11) { 
            uint16_t mappedCC=map(cm.data2,0,127,0,16383); currentCC11=mappedCC; currentPB3=mappedCC; lastActivePedal=mappedCC; 
            if(isVolumeMode) { 
                volumePedalGain=(float)mappedCC/16383.0f; 
                dspNeedsCommit = true;
                Control_Surface.sendControlChange({19,Channel_1},cm.data2); 
            } else { 
                if(!lutNeedsUpdate) { 
                    float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire);
                    if(currentLUT) pitchShiftFactor.store(currentLUT[mappedCC], std::memory_order_release); 
                } 
            } 
            return false; 
        }
        if(cm.data1>=24 && cm.data1<=28) { updateParameterFromCC(cm.data1, cm.data2); return false; }
        if(cm.data1==5 && cm.data2>=64) { isPB2WiperMode=!isPB2WiperMode; dspNeedsCommit = true; pb2ToggleRequested=true; }
        else if(cm.data1==6 && cm.data2>=64) { 
            bool sendCenterMidi=false; isVolumeMode=!isVolumeMode; 
            float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire);
            if(!isVolumeMode) { 
                volumePedalGain=1.0f; pedals.lockPB3Whammy(); sendCenterMidi=true; 
                currentPB3=8192; lastActivePedal=8192; 
                if(!lutNeedsUpdate && currentLUT!=nullptr) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release); 
            } 
            else { 
                pedals.lockPB3Volume(); lastActivePedal=8192; 
                volumePedalGain=(float)currentPB3 / 16383.0f; 
                if(!lutNeedsUpdate && currentLUT!=nullptr) pitchShiftFactor.store(currentLUT[8192], std::memory_order_release); 
            } 
            dspNeedsCommit = true;
            if(sendCenterMidi) Control_Surface.sendPitchBend(Channel_3, 8192); 
            settingsNeedSaving=true; lastParameterChangeTime=millis(); 
        }
        if(cm.data1==7 && cm.data2>=64) { 
            if(isWhammyActive) { isWhammyActive=false; isFrozen=false; isFeedbackActive=false; isHarmonizerMode=false; isCapoMode=false; isSynthMode=false; isPadMode=false; isChorusMode=false; isSwellMode=false; isVibratoMode=false; }
            else { int cmode=activeEffectMode.load(std::memory_order_acquire); isWhammyActive=(cmode==0); isFrozen=(cmode==1); isFeedbackActive=(cmode==2); isHarmonizerMode=(cmode==3); isCapoMode=(cmode==4); isSynthMode=(cmode==5); isPadMode=(cmode==6); isChorusMode=(cmode==7); isSwellMode=(cmode==8); isVibratoMode=(cmode==9); }
            dspNeedsCommit = true;
            settingsNeedSaving=true; lastParameterChangeTime=millis();
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200); 
    esp_brownout_init(); WiFi.mode(WIFI_OFF);
    
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    pinMode(SYSTEM_POWER_LATCH_PIN, OUTPUT);
    digitalWrite(SYSTEM_POWER_LATCH_PIN, HIGH);

    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "DSP_Max_CPU", &dsp_cpu_lock);
    if(dsp_cpu_lock != NULL) esp_pm_lock_acquire(dsp_cpu_lock);
    
    adc_continuous_handle_cfg_t adc_config={};
    adc_config.max_store_buf_size=16384; 
    adc_config.conv_frame_size=128; 
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &multifx_adc_handle));
    
    adc_continuous_config_t dig_cfg={}; 
    dig_cfg.sample_freq_hz = 2 * 1000; 
    dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1; 
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    
    adc_digi_pattern_config_t adc_pattern[4]={
        {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_0,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH},
        {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_1,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH},
        {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_9,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH},
        {.atten=ADC_ATTEN_DB_12,.channel=ADC_CHANNEL_3,.unit=ADC_UNIT_1,.bit_width=SOC_ADC_DIGI_MAX_BITWIDTH}
    };
    dig_cfg.pattern_num=4; dig_cfg.adc_pattern=adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(multifx_adc_handle, &dig_cfg)); ESP_ERROR_CHECK(adc_continuous_start(multifx_adc_handle));
    
    preferences.begin("whammy_cfg", true); 
    // HARDENED NVS LOAD: Constrain activeMode to safe range [0, 9]
    activeEffectMode.store(constrain(preferences.getInt("activeMode", 0), 0, 9), std::memory_order_release);
    latencyMode.store(constrain(preferences.getInt("latMode", 0), 0, 3), std::memory_order_release); 
    isPB2WiperMode=preferences.getBool("pb2Wiper", false); isVolumeMode=false; 
    currentSampleRate.store(48000, std::memory_order_release); 
    feedbackIntervalIdx.store(constrain(preferences.getInt("fbIdx", 0), 0, 4), std::memory_order_release);
    AppSettings savedSettings; size_t len=preferences.getBytes("dspData", &savedSettings, sizeof(AppSettings));
    if(len==sizeof(AppSettings)) {
        for(int i=0; i<10; i++) { effectMemory[i]=savedSettings.fxMem[i]; for(int p=0; p<5; p++) fxParams[i][p]=savedSettings.params[i][p]; }
    }
    
    uint16_t fxStates=preferences.getUShort("fxStates", 1);
    isWhammyActive=(fxStates&(1<<0)); isFrozen=(fxStates&(1<<1)); isFeedbackActive=(fxStates&(1<<2)); isHarmonizerMode=(fxStates&(1<<3)); isCapoMode=(fxStates&(1<<4)); isSynthMode=(fxStates&(1<<5)); isPadMode=(fxStates&(1<<6)); isChorusMode=(fxStates&(1<<7)); isSwellMode=(fxStates&(1<<8)); isVibratoMode=(fxStates&(1<<9));
    preferences.end();
    
    commitDSPState();
    
    pinMode(BATTERY_PIN, INPUT); pinMode(38, OUTPUT); digitalWrite(38, LOW); pinMode(15, OUTPUT); digitalWrite(15, HIGH);
    
    pedals.resetToCenter();
    pinMode(BOOT_SENSE_PIN, INPUT_PULLUP); 
    pinMode(BLE_TOGGLE_PIN, INPUT_PULLUP);
    lastActivityTime=millis();

    delayBuffer=(int16_t*)heap_caps_aligned_alloc(64, MAX_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    
    sramPitchBuffer=(int16_t*)heap_caps_aligned_alloc(64, SRAM_PITCH_BUF_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    // HARD-LOCKED TO ZERO-WAIT-STATE INTERNAL SRAM (NO PSRAM FALLBACK)
    fbDelayBuffer=(int16_t*)heap_caps_aligned_alloc(64, FB_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    freezeBuffer=(int16_t*)heap_caps_aligned_alloc(64, FREEZE_BUFFER_SIZE*sizeof(int16_t), MALLOC_CAP_SPIRAM);
    
    // 1024-SAMPLE SPATIAL DIFFUSER (4KB ALLOCATED IN INTERNAL SRAM)
    diffuserBuf = (float*)heap_caps_aligned_alloc(64, 1024 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    float* initialLUT = (float*)heap_caps_aligned_alloc(64, 16384*sizeof(float), MALLOC_CAP_SPIRAM);
    pitchShiftLUT.store(initialLUT, std::memory_order_relaxed);
    pitchShiftLUT_temp=(float*)heap_caps_aligned_alloc(64, 16384*sizeof(float), MALLOC_CAP_SPIRAM);
    
    i2s_in_block = (int32_t*)heap_caps_aligned_alloc(64, HOP_SIZE * 2 * sizeof(int32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    i2s_out_block = (int32_t*)heap_caps_aligned_alloc(64, HOP_SIZE * 2 * sizeof(int32_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    inBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    envBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    fzOutBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    masterGainBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    w1Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    w2Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    w3Buf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    padFilterBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    dryBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    fbOutBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    sMixBuf = (float*)heap_caps_aligned_alloc(64, HOP_SIZE * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!delayBuffer || !freezeBuffer || !i2s_in_block || !i2s_out_block || !pitchShiftLUT_temp || !inBuf || !fbDelayBuffer || !sramPitchBuffer || !diffuserBuf) {
        Serial.println("FATAL ERROR: Heap Allocation Failed (PSRAM or Internal). Halting DSP.");
        while(1) { vTaskDelay(pdMS_TO_TICKS(100)); }
    }
    
    memset(i2s_in_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
    memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
    memset(inBuf, 0, HOP_SIZE * sizeof(float));
    memset(envBuf, 0, HOP_SIZE * sizeof(float));
    memset(fzOutBuf, 0, HOP_SIZE * sizeof(float));
    memset(masterGainBuf, 0, HOP_SIZE * sizeof(float));
    memset(w1Buf, 0, HOP_SIZE * sizeof(float));
    memset(w2Buf, 0, HOP_SIZE * sizeof(float));
    memset(w3Buf, 0, HOP_SIZE * sizeof(float));
    memset(padFilterBuf, 0, HOP_SIZE * sizeof(float));
    memset(dryBuf, 0, HOP_SIZE * sizeof(float));
    memset(fbOutBuf, 0, HOP_SIZE * sizeof(float));
    memset(sMixBuf, 0, HOP_SIZE * sizeof(float));
    
    memset(delayBuffer, 0, MAX_BUFFER_SIZE*sizeof(int16_t)); 
    memset(sramPitchBuffer, 0, SRAM_PITCH_BUF_SIZE*sizeof(int16_t));
    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE*sizeof(int16_t)); 
    memset(freezeBuffer, 0, FREEZE_BUFFER_SIZE*sizeof(int16_t)); 
    memset(diffuserBuf, 0, 1024*sizeof(float));
    
    float* initLUTPtr = pitchShiftLUT.load(std::memory_order_relaxed);
    if(initLUTPtr) memset(initLUTPtr, 0, 16384*sizeof(float)); 
    memset(pitchShiftLUT_temp, 0, 16384*sizeof(float)); 
    
    // 4096-POINT HANN WINDOW INITIALIZATION
    for(int i=0; i<4096; i++) { hannLUT[i]=0.5f*(1.0f-cosf(TWO_PI*((float)i/4095.0f))); }
    for(int i=0; i<1024; i++) { lfoLUT[i]=powf(2.0f,(15.0f*sinf(TWO_PI*((float)i/1024.0f)))/1200.0f); }
    for(int i=0; i<2048; i++) { synthLUT[i]=sinf((((float)i-1024.0f)/1024.0f)*45.0f); }
    
    memset(dmaPingBuffer, 0, sizeof(dmaPingBuffer));
    memset(dmaPongBuffer, 0, sizeof(dmaPongBuffer));

    async_memcpy_config_t dma_config = ASYNC_MEMCPY_DEFAULT_CONFIG();
    dma_config.backlog = 8;
    ESP_ERROR_CHECK(esp_async_memcpy_install(&dma_config, &dma_memcpy_handle));

    calibratePBs(); updateLUT();
    float* currLut = pitchShiftLUT.load(std::memory_order_acquire);
    if (currLut) pitchShiftFactor.store(currLut[8192], std::memory_order_release);
    
    btmidi.setName("Whammy_S3");
    Control_Surface >> pipes >> btmidi; 
    Control_Surface >> pipes >> usbmidi; 
    usbmidi >> pipes >> Control_Surface; 
    btmidi >> pipes >> Control_Surface;
    Control_Surface.setMIDIInputCallbacks(channelMessageCallback, nullptr, nullptr, nullptr); 
    Control_Surface.begin();
    
    i2s_chan_config_t i2sConfig=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2sConfig.dma_desc_num=8; 
    i2sConfig.dma_frame_num=HOP_SIZE; i2sConfig.auto_clear=true; 
    i2s_chan_handle_t t_tx, t_rx;
    i2s_new_channel(&i2sConfig, &t_tx, &t_rx);
    tx_chan = t_tx; rx_chan = t_rx;
    
    i2s_std_config_t stdConfig={ 
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(currentSampleRate.load(std::memory_order_acquire)), 
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg={ .mclk=GPIO_NUM_43, .bclk=GPIO_NUM_44, .ws=GPIO_NUM_18, .dout=GPIO_NUM_21, .din=GPIO_NUM_17 } 
    };
    stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; 
    i2s_channel_init_std_mode((i2s_chan_handle_t)tx_chan, &stdConfig); i2s_channel_init_std_mode((i2s_chan_handle_t)rx_chan, &stdConfig);
    
    settingsNeedSaving=false;
    
    xTaskCreatePinnedToCore(DisplayTask, "UI", 16384, NULL, 1, NULL, 1); 

    dspTaskStack = (StackType_t*)heap_caps_aligned_alloc(16, 16384, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    dspTaskTCB = (StaticTask_t*)heap_caps_aligned_alloc(16, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (dspTaskStack != nullptr && dspTaskTCB != nullptr) {
        audioTaskHandle = xTaskCreateStaticPinnedToCore(
            AudioDSPTask, "DSP", 16384, NULL, configMAX_PRIORITIES - 1, 
            dspTaskStack, dspTaskTCB, 0
        );
    } else {
        Serial.println("Error: Internal SRAM allocation failed for DSP Task!");
        while(1) vTaskDelay(100);
    }
    
    i2s_channel_enable((i2s_chan_handle_t)tx_chan); i2s_channel_enable((i2s_chan_handle_t)rx_chan); 
}

void loop() {
    static bool lastBtState=false; 
    static uint8_t lastVolumeCC=127;
    #if ENABLE_PAR_KNOBS
        static int lastCcOut[5]={-1,-1,-1,-1,-1};
    #endif
    static unsigned long lastBatteryTime=0; 
    static float smoothedRawBat=0.0f;
    static unsigned long gpio14PressTime = 0;
    static bool gpio14LastState = HIGH;
    static unsigned long lastDebounceTime = 0;
    static bool lastBootState = HIGH;

    if (bleEnabled.load(std::memory_order_relaxed)) {
        Control_Surface.loop(); 
    } else {
        Control_Surface.updateMidiInput(); 
    }

    bool currentBtState=btmidi.isConnected();
    if(currentBtState!=lastBtState) { 
        lastBtState=currentBtState; 
    }
    if(millis()-lastActivityTime>LIGHT_SLEEP_TIMEOUT) goToLightSleep();
    
    bool reading = (REG_READ(GPIO_IN_REG) & (1 << BLE_TOGGLE_PIN)) != 0;
    if (reading != gpio14LastState && (millis() - lastDebounceTime) > 50) {
        lastDebounceTime = millis();
        if (!reading) { 
            gpio14PressTime = millis(); 
            lastActivityTime = millis();
        } else { 
            unsigned long pressDuration = millis() - gpio14PressTime;
            if (pressDuration >= 1000) {
                if (bleEnabled.load(std::memory_order_relaxed)) {
                    btStop();
                    bleEnabled.store(false, std::memory_order_relaxed);
                } else {
                    btStart();
                    bleEnabled.store(true, std::memory_order_relaxed);
                }
            } else if (pressDuration >= 50) {
                cycleLatencyMode();
            }
        }
        gpio14LastState = reading;
    }
    
    #if ENABLE_PAR_KNOBS
        if(filterPar1.update()) { int cc1=map(filterPar1.getValue(),0,4095,0,127); if(cc1!=lastCcOut[0]) { Control_Surface.sendControlChange({24,Channel_1},cc1); updateParameterFromCC(24,cc1); lastCcOut[0]=cc1; } }
        if(filterPar2.update()) { int cc2=map(filterPar2.getValue(),0,4095,0,127); if(cc2!=lastCcOut[1]) { Control_Surface.sendControlChange({25,Channel_1},cc2); updateParameterFromCC(25,cc2); lastCcOut[1]=cc2; } }
        if(filterPar3.update()) { int cc3=map(filterPar3.getValue(),0,4095,0,127); if(cc3!=lastCcOut[2]) { Control_Surface.sendControlChange({26,Channel_1},cc3); updateParameterFromCC(26,cc3); lastCcOut[2]=cc3; } }
        if(filterPar4.update()) { int cc4=map(filterPar4.getValue(),0,4095,0,127); if(cc4!=lastCcOut[3]) { Control_Surface.sendControlChange({27,Channel_1},cc4); updateParameterFromCC(27,cc4); lastCcOut[3]=cc4; } }
        if(filterPar5.update()) { int cc5=map(filterPar5.getValue(),0,4095,0,127); if(cc5!=lastCcOut[4]) { Control_Surface.sendControlChange({28,Channel_1},cc5); updateParameterFromCC(28,cc5); lastCcOut[4]=cc5; } }
    #endif
    
    fetchADCDMA();
    
    bool currentBootState = (REG_READ(GPIO_IN_REG) & (1 << BOOT_SENSE_PIN)) != 0;
    if(!currentBootState && lastBootState) { 
        switchEffectMode(activeEffectMode.load(std::memory_order_acquire) + 1);
        lastActivityTime = millis();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    lastBootState = currentBootState;
    
    pedals.process(latestPB1, latestPB2, latestPB3, isVolumeMode, INVERT_PB3);
    int calA=pedals.getCalA(), calB=pedals.getCalB(), calC=pedals.getCalC(); bool moveA=pedals.hasMovedA(), moveB=pedals.hasMovedB(), moveC=pedals.hasMovedC();
    if(moveA||moveB||moveC) {
        lastActivityTime=millis();
        if(moveA) { Control_Surface.sendPitchBend(Channel_1, calA); pedals.updateLastMidiA(); currentPB1=calA; }
        if(moveB) { Control_Surface.sendPitchBend(Channel_2, calB); pedals.updateLastMidiB(); currentPB2=calB; }
        if(moveC) { if(!isVolumeMode) Control_Surface.sendPitchBend(Channel_3, calC); pedals.updateLastMidiC(); currentPB3=calC; }
        bool pitchChanged=false;
        if(moveA) { lastActivePedal=calA; pitchChanged=true; }
        if(moveB) { lastActivePedal=calB; pitchChanged=true; }
        if(moveC&&!isVolumeMode) { lastActivePedal=calC; pitchChanged=true; }
        if(pitchChanged) { 
            float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire);
            if(currentLUT) pitchShiftFactor.store(currentLUT[constrain(lastActivePedal,0,16383)], std::memory_order_release); 
        }
        if(moveC&&isVolumeMode) { 
            uint8_t vCC=map(calC,0,16383,0,127); 
            if(vCC!=lastVolumeCC) { Control_Surface.sendControlChange({19,Channel_1},vCC); lastVolumeCC=vCC; } 
            volumePedalGain=(float)calC/16383.0f; 
            dspNeedsCommit = true; 
        }
    }
    
    int currentBatVal = latestBat.load(std::memory_order_relaxed);
    smoothedRawBat = (smoothedRawBat == 0.0f) ? (float)currentBatVal : (smoothedRawBat * 0.95f) + ((float)currentBatVal * 0.05f);
    if(millis()-lastBatteryTime>1000) { 
        lastBatteryTime=millis(); 
        const float CALIBRATION_MULTIPLIER=1.04571f; 
        float instantVoltage=(smoothedRawBat/4095.0f)*3.3f*2.0f*CALIBRATION_MULTIPLIER; 
        
        if(instantVoltage>2.0f) { 
            if(instantVoltage <= 3.40f) {
                dsp_is_paused.store(true, std::memory_order_release);
                while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
                i2s_channel_disable((i2s_chan_handle_t)tx_chan); 
                i2s_channel_disable((i2s_chan_handle_t)rx_chan);
                
                saveSettings();
                
                isBatteryDead.store(true, std::memory_order_release);
                
                vTaskDelay(pdMS_TO_TICKS(3000));
                REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (38 - 32));
                
                digitalWrite(SYSTEM_POWER_LATCH_PIN, LOW); 
                
                if (bleEnabled.load(std::memory_order_relaxed)) {
                    btStop();
                    bleEnabled.store(false, std::memory_order_relaxed);
                }
                
                while(true) {
                    vTaskDelay(portMAX_DELAY);
                }
            }
            bool charging=(instantVoltage>4.20f); 
            int newPercent=getBatteryPercentage(instantVoltage); 
            currentBatteryVoltage=instantVoltage; 
            currentBatteryPercent.store(newPercent, std::memory_order_relaxed); 
            isBatteryCharging.store(charging, std::memory_order_relaxed); 
        } 
    }
    
    static unsigned long lastLutUpdate=0;
    if(lutNeedsUpdate && (millis()-lastLutUpdate>40)) { 
        lutNeedsUpdate=false; 
        updateLUT(); 
        float* currentLUT = pitchShiftLUT.load(std::memory_order_acquire);
        if(currentLUT) pitchShiftFactor.store(currentLUT[constrain(lastActivePedal,0,16383)], std::memory_order_release); 
        lastLutUpdate=millis(); 
    }
    
    if(settingsNeedSaving && (millis()-lastParameterChangeTime>2000)) { 
        if (ui_audio_level.load(std::memory_order_acquire) < 0.02f || (millis()-lastParameterChangeTime>10000)) {
            settingsNeedSaving=false; 
            dsp_is_paused.store(true, std::memory_order_release);
            while(!dsp_ack_parked.load(std::memory_order_acquire)) { vTaskDelay(pdMS_TO_TICKS(1)); }
            saveSettings();
            dsp_is_paused.store(false, std::memory_order_release);
        }
    }
    
    if(sampleRateToggleRequested) { sampleRateToggleRequested=false; toggleSampleRate(); }
    if(pb2ToggleRequested) { pb2ToggleRequested=false; calibratePBs(); settingsNeedSaving=true; lastParameterChangeTime=millis(); }

    if (dspNeedsCommit) {
        if (commitDSPState()) {
            dspNeedsCommit = false;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}