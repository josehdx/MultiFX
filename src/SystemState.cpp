#include "SystemState.h"

VectorBiquadS3 padVectorFilter;
Preferences preferences;
SettingsManager settingsMgr;
volatile bool settingsNeedSaving = false;
volatile unsigned long lastParameterChangeTime = 0;
float fxParams[10][5] = {{0.0f,1.0f,0.0f,0.0f,0.0f},{0.6f,0.0002f,0.00005f,0.0f,0.0f},{5120.0f,30.0f,0.02f,0.0f,0.0f},{0.5f,0.0f,0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f,0.0f,0.0f},{0.1f,0.005f,0.3f,0.1f,0.0f},{0.95f,1.5f,0.0f,0.0f,0.0f},{1536.0f,0.4f,0.0f,0.0f,0.0f},{0.015f,0.00002f,0.00005f,0.0f,0.0f},{1.0f,0.0f,0.0f,0.0f,0.0f}};
const bool INVERT_PB3 = false;

std::atomic<bool> dsp_is_paused{false}, dsp_ack_parked{false}, lut_ack_parked{false}, ui_clear_meters_requested{false}, globalAudioResetRequested{false}, panicResetRequested{false}, bleEnabled{true};
std::atomic<float> pitchShiftFactor{1.0f};
std::atomic<uint32_t> currentSampleRate{96000}; 

bool isKnobEditMode = false, showBleWarning = false, showSavingScreen = false, showKnobModeScreen = false;
unsigned long warningTimer = 0;
const float LATENCY_WINDOWS[]={512.0f, 1024.0f, 2048.0f, 4096.0f};

int32_t *i2s_in_block = nullptr, *i2s_out_block = nullptr;
float *inBuf=nullptr, *envBuf=nullptr, *fzOutBuf=nullptr, *masterGainBuf=nullptr, *w1Buf=nullptr, *w2Buf=nullptr, *w3Buf=nullptr, *padFilterBuf=nullptr, *dryBuf=nullptr, *fbOutBuf=nullptr, *sMixBuf=nullptr;
esp_pm_lock_handle_t dsp_cpu_lock = NULL;

__attribute__((aligned(64))) int16_t dmaPingBuffer[HOP_SIZE] = {0};
__attribute__((aligned(64))) int16_t dmaPongBuffer[HOP_SIZE] = {0};
int16_t* activeDmaReadBuf = dmaPingBuffer;
int16_t* activeDmaWriteBuf = dmaPongBuffer;
__attribute__((aligned(64))) int16_t sramDryBlock[HOP_SIZE] = {0};

DSPCoreState dspStates[2];
std::atomic<DSPCoreState*> dspActiveState{&dspStates[0]};
int dspWriteIndex = 1;
volatile bool dspNeedsCommit = false;
std::atomic<bool> dspAckCommit{true}; 

volatile uint16_t lastActivePedal = 8192;
volatile float effectMemory[10]={12.0f,-12.0f,0.0f,5.0f,-2.0f,-12.0f,-12.0f,12.0f,0.0f,0.0f}; 

volatile bool isWhammyActive=true, isFrozen=false, isFeedbackActive=false, isHarmonizerMode=false, isSynthMode=false, isPadMode=false, isCapoMode=false, isChorusMode=false, isSwellMode=false, isVibratoMode=false, isVolumeMode=false, isPB2WiperMode=false;
volatile float volumePedalGain=1.0f;
std::atomic<int> latencyMode{0}, activeEffectMode{0}, feedbackIntervalIdx{0}; 

int16_t *sramPitchLow = nullptr, *sramPitchHigh = nullptr, *delayBuffer = nullptr, *fbDelayBuffer = nullptr, *freezeBuffer = nullptr;
float *diffuserBuf = nullptr;
int diffuserIdx = 0;

TaskHandle_t audioTaskHandle = NULL, lutTaskHandle = NULL;
StackType_t *dspTaskStack = nullptr, *lutTaskStack = nullptr;
StaticTask_t *dspTaskTCB = nullptr, *lutTaskTCB = nullptr;
int writeIndex = 0, fbDelayWriteIdx = 0, sramWriteIdx = 0;

std::atomic<bool> asyncLutUpdateRequested{false};
volatile bool lutNeedsUpdate = false;
float *pitchLutBufferA = nullptr, *pitchLutBufferB = nullptr;
std::atomic<float*> activePitchLUT{nullptr};

uint32_t tap_w1_lo_1=0, tap_w1_lo_2=2048<<16, tap_w1_hi_1=0, tap_w1_hi_2=256<<16;
uint32_t tap_w2_lo_1=0, tap_w2_lo_2=2048<<16, tap_w2_hi_1=0, tap_w2_hi_2=256<<16;
uint32_t tap_w3_1=0, tap_w3_2=256<<16, tap_w4_1=0, tap_w4_2=256<<16, tap_w5_1=0, tap_w5_2=256<<16;
float currentWindowSize = 512.0f;
int freezeLength = 96000;
bool wasFrozen = false;
volatile bool apfNeedsClear = false;
volatile float freezeRamp = 0.0f, chorusLfoPhase=0.0f, feedbackLfoPhase=0.0f, vibratoLfoPhase=0.0f, swellGain=0.0f, feedbackRamp=0.0f;
float fbHpfState=0.0f, feedbackFilter=0.0f;
std::atomic<int> hardwareSyncMuteFrames{0};
volatile bool sampleRateToggleRequested=false, pb2ToggleRequested=false;
unsigned long lastActivityTime=0;

std::atomic<float> core0_dsp_load __attribute__((aligned(64))) {0.0f};
std::atomic<float> core1_ctrl_load __attribute__((aligned(64))) {0.0f};
std::atomic<uint32_t> max_loop_latency_ms{0}; 
std::atomic<float> ui_audio_level __attribute__((aligned(64))) {0.0f}, ui_output_level __attribute__((aligned(64))) {0.0f};

#ifdef ENABLE_ADVANCED_TELEMETRY
    std::atomic<uint32_t> audio_underflow_count{0};
    std::atomic<uint32_t> dsp_stack_watermark{0};
    std::atomic<uint32_t> lut_stack_watermark{0};
#endif

volatile bool isAdcPaused=false;
adc_continuous_handle_t multifx_adc_handle = NULL;
volatile int latestPB1=2048, latestPB2=2048, latestPB3=2048, latestPar1=2048;
std::atomic<int> latestBat{2048};

const int BOOT_SENSE_PIN=0, BLE_TOGGLE_PIN=14;
volatile uint16_t currentPB1=8192, currentPB2=8192, currentPB3=8192, currentCC11=0; 

#if !defined(FW_MODE_KNOBS_ONLY)
BluetoothMIDI_Interface* btmidi = nullptr;
#endif
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<4> pipes;
PedalManager pedals;