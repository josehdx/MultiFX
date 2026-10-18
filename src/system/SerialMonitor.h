#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

#include <Arduino.h>

extern char lastMidiDebug[64];

struct TelemetryData {
    float dspCoreLoad;
    float ctrlCoreLoad;
    uint32_t sampleRate;
    int latencyMode;
    float batVoltage;
    int batPercent;
    bool isCharging;

    bool bleConnected;
    bool bleHostConnected;
    bool blePedalConnected;
    
    char bleHostStatus[16];
    char blePedalStatus[16];
    char bleTarget[32];
    char bleLastSeen[32];
    
    uint32_t blePackets;

    int activeMode;
    bool isMonoPolyActive;
    int monoPolyAlgo;
    bool fxStates[10];

    uint16_t pb1, pb2, pb3, cc11;
    float inMeter, outMeter;

    uint32_t audioUnderflows;
    uint32_t dspStackWatermark;
    uint32_t peakLoopLatency;
    uint32_t dmaTransfers;

    // Advanced Debug Tracking
    uint32_t peakRxCyc;
    uint32_t peakDspCyc;
    uint32_t peakTxCyc;
    uint32_t partialRx;
    uint32_t phaseTears;
    uint32_t rxErrors;
    uint32_t txErrors;
};

class SerialMonitor {
private:
    unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 1000;

    const char* EFFECT_NAMES[10] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};
    const char* MONO_ALGOS[5] = {"TD-PSOLA", "YIN SYNTH", "ECKF", "VAR DELAY", "VOCODER"};

    void dualPrintf(const char* fmt, ...) {
        char buf[256];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        
        Serial0.print(buf); 
    }

    void dualPrintln(const char* str) {
        Serial0.println(str); 
    }

public:
    void printMetrics(const TelemetryData& data, const char* bleInitStep) {
        if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
            lastPrintTime = millis();
            
            uint32_t freeSRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
            uint32_t freePSRAM = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;

            char activeFxStr[128] = "";
            int count = 0;
            for (int i = 0; i < 10; i++) {
                if (data.fxStates[i]) {
                    if (count > 0) strcat(activeFxStr, " + ");
                    strcat(activeFxStr, EFFECT_NAMES[i]);
                    count++;
                }
            }
            if (count == 0) strcpy(activeFxStr, "ALL BYPASSED");

            dualPrintln("\n================ TELEMETRY DEBUGGER ================");
            dualPrintf("DSP Core 0 Load  : %d%%\n", (int)data.dspCoreLoad);
            dualPrintf("Ctrl Core 1 Load : %d%%\n", (int)data.ctrlCoreLoad);
            dualPrintf("Internal SRAM    : %luK Free\n", freeSRAM);
            dualPrintf("External PSRAM   : %luK Free\n", freePSRAM);
            dualPrintf("Sample Rate      : %lu Hz\n", data.sampleRate);
            dualPrintf("Latency Mode     : %d\n", data.latencyMode);
            
            dualPrintf("Battery State    : %.2fV (%d%%) - Charging: %s\n", 
                       data.batVoltage, data.batPercent, data.isCharging ? "YES" : "NO");

            dualPrintln("----------- DUAL-ROLE BLE TELEMETRY -----------");
            dualPrintf("Init Sequence    : %s\n", bleInitStep);
            dualPrintf("Host (Mac/iOS)   : %s\n", data.bleHostConnected ? "CONNECTED" : "WAITING");
            dualPrintf("Pedal (M-Vave)   : %s\n", data.blePedalConnected ? "CONNECTED" : "WAITING");
            dualPrintf("Total Packets    : %lu\n", data.blePackets);
            dualPrintf("Target Pedal     : %s\n", data.bleTarget[0] ? data.bleTarget : "NONE");
            dualPrintf("Last Scanned BLE : %s\n", data.bleLastSeen[0] ? data.bleLastSeen : "WAITING...");
            dualPrintf("Last MIDI Packet : %s\n", lastMidiDebug);

            dualPrintln("-------------- AUDIO & DSP ENGINE -------------");
            if (data.isMonoPolyActive) {
                int algo = constrain(data.monoPolyAlgo, 0, 4);
                dualPrintf("Active Mode      : MONOPOLY [%s] (Override)\n", MONO_ALGOS[algo]);
            } else {
                dualPrintf("Active Mode      : %s (%d)\n", EFFECT_NAMES[data.activeMode % 10], data.activeMode);
            }
            
            dualPrintf("Active Effects   : %s\n", activeFxStr);
            dualPrintf("Pedal Vals       : PB1:%d | PB2:%d | PB3:%d | CC11:%d\n", 
                       data.pb1, data.pb2, data.pb3, data.cc11);
            dualPrintf("Audio Meters     : IN: %.3f | OUT: %.3f\n", data.inMeter, data.outMeter);
            dualPrintf("Audio Underflows : %lu\n", data.audioUnderflows);
            dualPrintf("DSP Min Stack RAM: %lu Bytes\n", data.dspStackWatermark);
            dualPrintf("Peak Loop Latency: %lu ms\n", data.peakLoopLatency);
            
            dualPrintln("-------------- CORE 0 DIAGNOSTICS -------------");
            dualPrintf("Peak RX Wait Cyc : %lu\n", data.peakRxCyc);
            dualPrintf("Peak DSP Loop Cyc: %lu\n", data.peakDspCyc);
            dualPrintf("Peak TX Wait Cyc : %lu\n", data.peakTxCyc);
            dualPrintf("Partial RX Reads : %lu\n", data.partialRx);
            dualPrintf("Word Phase Tears : %lu\n", data.phaseTears);
            dualPrintf("RX Driver Errors : %lu\n", data.rxErrors);
            dualPrintf("TX Driver Errors : %lu\n", data.txErrors);
            dualPrintln("====================================================");
        }
    }
};

#endif