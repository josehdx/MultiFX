#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

#include <Arduino.h>

struct TelemetryData {
    float dspCoreLoad;
    float ctrlCoreLoad;
    uint32_t sampleRate;
    int latencyMode;
    float batVoltage;
    int batPercent;
    bool isCharging;
    bool bleConnected;
    int activeMode;
    bool isMonoPolyActive; // Added flag for MonoPoly override state
    int monoPolyAlgo;      // Added algorithm index
    bool fxStates[10];
    uint16_t pb1, pb2, pb3, cc11;
    float inMeter, outMeter;
    uint32_t audioUnderflows;
    uint32_t dspStackWatermark;
    uint32_t peakLoopLatency;
    uint32_t dmaTransfers;
};

class SerialMonitor {
private:
    unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 500; // 0.5-second interval
    const char* EFFECT_NAMES[10] = {"WHAMMY", "FREEZE", "FEEDBACK", "HARMONY", "CAPO", "SYNTH", "PAD", "CHORUS", "SWELL", "VIBRATO"};
    const char* MONO_ALGOS[5] = {"TD-PSOLA", "YIN SYNTH", "ECKF", "VAR DELAY", "VOCODER"};

public:
    void printMetrics(const TelemetryData& data) {
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

            // Output to Native USB CDC
            Serial.println("================ TELEMETRY DEBUGGER ================");
            Serial.printf("DSP Core 0 Load  : %d%%\n", (int)data.dspCoreLoad);
            Serial.printf("Ctrl Core 1 Load : %d%%\n", (int)data.ctrlCoreLoad);
            Serial.printf("Internal SRAM    : %luK Free\n", freeSRAM);
            Serial.printf("External PSRAM   : %luK Free\n", freePSRAM);
            Serial.printf("Sample Rate      : %lu Hz\n", data.sampleRate);
            Serial.printf("Latency Mode     : %d\n", data.latencyMode);
            
            Serial.printf("Battery State    : %.2fV (%d%%) - Charging: %s\n", 
                  data.batVoltage, data.batPercent, data.isCharging ? "YES" : "NO");
            Serial.printf("BLE MIDI Conn    : %s\n", data.bleConnected ? "CONNECTED" : "WAITING");

            // Format Mode String based on MonoPoly Override State
            if (data.isMonoPolyActive) {
                int algo = constrain(data.monoPolyAlgo, 0, 4);
                Serial.printf("Active Mode      : MONOPOLY [%s] (Override)\n", MONO_ALGOS[algo]);
            } else {
                Serial.printf("Active Mode      : %s (%d)\n", 
                      EFFECT_NAMES[data.activeMode % 10], data.activeMode);
            }
            
            Serial.printf("Active Effects   : %s\n", activeFxStr);
            Serial.printf("Pedal Vals       : PB1:%d | PB2:%d | PB3:%d | CC11:%d\n", 
                  data.pb1, data.pb2, data.pb3, data.cc11);
            Serial.printf("Audio Meters     : IN: %.3f | OUT: %.3f\n", 
                  data.inMeter, data.outMeter);
                  
            Serial.println("--- SYSTEM STARVATION & DIAGNOSTICS ---");
            Serial.printf("Audio Underflows : %lu\n", data.audioUnderflows);
            Serial.printf("DSP Min Stack RAM: %lu Bytes\n", data.dspStackWatermark);
            Serial.printf("Peak Loop Latency: %lu ms\n", data.peakLoopLatency);
            Serial.printf("DMA Transfers    : %lu\n\n", data.dmaTransfers);
        }
    }
};

#endif