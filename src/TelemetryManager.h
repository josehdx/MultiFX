#pragma once

#include <Arduino.h>
#include "SystemState.h"
#include "SerialMonitor.h"

class TelemetryManager {
private:
    inline static SerialMonitor serialMonitor;

public:
    static void report(DSPCoreState* activeDSP) {
        if (!activeDSP) return;

        TelemetryData tData = {}; 
        tData.dspCoreLoad = core0_dsp_load.load(std::memory_order_relaxed); 
        tData.ctrlCoreLoad = core1_ctrl_load.load(std::memory_order_relaxed);
        tData.sampleRate = currentSampleRate.load(std::memory_order_acquire);
        tData.latencyMode = latencyMode.load(std::memory_order_relaxed);
        tData.batVoltage = 4.00f; tData.batPercent = 100; tData.isCharging = false;
        
    #if !defined(FW_MODE_KNOBS_ONLY)
        tData.bleConnected = btmidi != nullptr ? btmidi->isConnected() : false; 
    #else
        tData.bleConnected = false;
    #endif

        tData.activeMode = activeDSP->activeMode;
        tData.isMonoPolyActive = isMonoPolyActive.load(std::memory_order_acquire);
        tData.monoPolyAlgo = constrain((int)(fxParams[activeDSP->activeMode][0] * 4.99f), 0, 4);

        tData.fxStates[0] = activeDSP->w;  tData.fxStates[1] = activeDSP->fz; tData.fxStates[2] = activeDSP->fb; 
        tData.fxStates[3] = activeDSP->hr; tData.fxStates[4] = activeDSP->cp; tData.fxStates[5] = activeDSP->sy; 
        tData.fxStates[6] = activeDSP->pd; tData.fxStates[7] = activeDSP->ch; tData.fxStates[8] = activeDSP->sw; 
        tData.fxStates[9] = activeDSP->vb;
        
        tData.pb1 = currentPB1; tData.pb2 = currentPB2; tData.pb3 = currentPB3; tData.cc11 = currentCC11;
        tData.inMeter = ui_audio_level.load(std::memory_order_acquire); 
        tData.outMeter = ui_output_level.load(std::memory_order_acquire);
        
    #ifdef ENABLE_ADVANCED_TELEMETRY
        tData.audioUnderflows = audio_underflow_count.load(std::memory_order_relaxed);
        tData.dmaTransfers = 0;
        tData.dspStackWatermark = dsp_stack_watermark.load(std::memory_order_relaxed); 
    #else
        tData.audioUnderflows = 0; tData.dmaTransfers = 0; tData.dspStackWatermark = 0;
    #endif
        
        tData.peakLoopLatency = max_loop_latency_ms.exchange(0, std::memory_order_relaxed);

        static unsigned long lastTelemetryPrint = 0;
        if (millis() - lastTelemetryPrint >= 2000) {
            lastTelemetryPrint = millis(); 
            serialMonitor.printMetrics(tData); 
        }
    }
};