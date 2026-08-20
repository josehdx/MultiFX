#pragma once

#include "SystemState.h"
#include "LUTManager.h"

// Bring in FX dependencies for the DSP thread
#include "FX_Whammy.h"
#include "FX_Freeze.h"
#include "FX_Feedback.h"
#include "FX_Harmony.h"
#include "FX_Capo.h"
#include "FX_Synth.h"
#include "FX_Pad.h"
#include "FX_Chorus.h"
#include "FX_Swell.h"
#include "FX_Vibrato.h"

static void IRAM_ATTR __attribute__((optimize("Ofast"))) AudioDSPTask(void * pvParameters) {
    float input_dc_offset=0.0f, synthEnv=0.0f, synthFilter=0.0f, synthBandpass=0.0f, padEnv=0.0f, inputEnvelope=0.0f, feedbackFilterVar=0.0f, smoothedVolGain=1.0f, currentPitch=1.0f, fbOutNode=0.0f, smoothed_delay_samples=0.0f, dampState=0.0f, wowState=0.0f;
    uint32_t wowRng = 123456789; bool wasFeedbackActive=false; int freezeWriteIdxVar=0, freezePlayCounterVar=0, freezeStartIdxVar=0, activeFreezeLength=96000;
    float c_fx[10][5] = {0.0f}; int c_lat=0, c_act=0; bool c_w=true, c_fz=false, c_fb=false, c_hr=false, c_cp=false, c_sy=false, c_pd=false, c_ch=false, c_sw=false, c_vb=false; float c_vg=1.0f;
    const float normFactor=1.0f/2147483648.0f, DC_OFFSET=1e-9f;
    float lastPadCutoff = -1.0f; int apf1Idx = 0, apf2Idx = 0;
    float cross_lp1 = 0.0f, cross_lp2 = 0.0f;
    DSPCoreState* lastAckedDSP = nullptr;

    while (LUTManager::hannLUT == nullptr || LUTManager::lfoLUT == nullptr || LUTManager::synthLUT == nullptr || LUTManager::apf1Buffer == nullptr || LUTManager::apf2Buffer == nullptr || LUTManager::pitchSincLUT == nullptr) { 
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }

    for(;;) {
#ifdef ENABLE_ADVANCED_TELEMETRY
        // Record remaining Stack RAM in Bytes for Core 0 Audio Task
        dsp_stack_watermark.store((uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t), std::memory_order_relaxed);
#endif

        if(__builtin_expect(dsp_is_paused.load(std::memory_order_acquire), 0)) {
            dsp_ack_parked.store(true, std::memory_order_release);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            dsp_ack_parked.store(false, std::memory_order_release);
        }

        size_t bytesRead = 0; 
        i2s_channel_read((i2s_chan_handle_t)I2SManager::rx_chan, i2s_in_block, HOP_SIZE*2*sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(10));

        if(__builtin_expect(bytesRead > 0, 1)) {
            int framesRead = bytesRead / 8;
            if(__builtin_expect(framesRead == HOP_SIZE, 1)) {
                if(__builtin_expect(panicResetRequested.load(std::memory_order_acquire), 0)) {
                    memset(activeDmaReadBuf, 0, HOP_SIZE * sizeof(int16_t)); memset(activeDmaWriteBuf, 0, HOP_SIZE * sizeof(int16_t));
                    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t));
                    synthEnv=0.0f; synthFilter=0.0f; synthBandpass=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate.load(std::memory_order_acquire); fbDelayWriteIdx=0; apfNeedsClear=true; freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f; dampState=0.0f; wowState=0.0f; diffuserIdx=0; if(diffuserBuf) memset(diffuserBuf, 0, 1024*sizeof(float));
                    cross_lp1 = 0.0f; cross_lp2 = 0.0f;
                    tap_w1_lo_1=0; tap_w1_lo_2=2048<<16; tap_w1_hi_1=0; tap_w1_hi_2=256<<16;
                    tap_w2_lo_1=0; tap_w2_lo_2=2048<<16; tap_w2_hi_1=0; tap_w2_hi_2=256<<16;
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16;
                    tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    panicResetRequested.store(false, std::memory_order_release); padVectorFilter.reset(); lastPadCutoff = -1.0f;
                }

                if(__builtin_expect(globalAudioResetRequested.load(std::memory_order_acquire), 0)) {
                    memset(activeDmaReadBuf, 0, HOP_SIZE * sizeof(int16_t)); memset(activeDmaWriteBuf, 0, HOP_SIZE * sizeof(int16_t));
                    memset(fbDelayBuffer, 0, FB_BUFFER_SIZE * sizeof(int16_t));
                    synthEnv=0.0f; synthFilter=0.0f; synthBandpass=0.0f; padEnv=0.0f; inputEnvelope=0.0f; feedbackFilterVar=0.0f; smoothedVolGain=volumePedalGain; currentPitch=1.0f; freezeWriteIdxVar=0; freezePlayCounterVar=0; freezeStartIdxVar=0; activeFreezeLength=currentSampleRate.load(std::memory_order_acquire); fbDelayWriteIdx=0; writeIndex=0; sramWriteIdx=0; apfNeedsClear=true; input_dc_offset=0.0f; ui_audio_level.store(0.0f, std::memory_order_release); ui_output_level.store(0.0f, std::memory_order_release); freezeRamp=0.0f; feedbackRamp=0.0f; vibratoLfoPhase=0.0f; chorusLfoPhase=0.0f; feedbackLfoPhase=0.0f; dampState=0.0f; wowState=0.0f; diffuserIdx=0; if(diffuserBuf) memset(diffuserBuf, 0, 1024*sizeof(float));
                    cross_lp1 = 0.0f; cross_lp2 = 0.0f;
                    tap_w1_lo_1=0; tap_w1_lo_2=2048<<16; tap_w1_hi_1=0; tap_w1_hi_2=256<<16;
                    tap_w2_lo_1=0; tap_w2_lo_2=2048<<16; tap_w2_hi_1=0; tap_w2_hi_2=256<<16;
                    uint32_t halfWinFixed=((uint32_t)currentWindowSize/2)<<16;
                    tap_w3_1=0; tap_w3_2=halfWinFixed; tap_w4_1=0; tap_w4_2=halfWinFixed; tap_w5_1=0; tap_w5_2=halfWinFixed;
                    globalAudioResetRequested.store(false, std::memory_order_release); smoothed_delay_samples=0.0f;
                    if(hardwareSyncMuteFrames.load(std::memory_order_acquire) < 10) hardwareSyncMuteFrames.store((currentSampleRate.load(std::memory_order_acquire)/HOP_SIZE)*0.40f, std::memory_order_release);
                    lastPadCutoff = -1.0f; padVectorFilter.reset();
                }

                int currentMute = hardwareSyncMuteFrames.load(std::memory_order_acquire); bool isMuted = false;
                if(__builtin_expect(currentMute > 0, 0)) { hardwareSyncMuteFrames.store(currentMute - 1, std::memory_order_release); isMuted = true; }
                uint32_t start_cycles=xthal_get_ccount(); float srScale = 48000.0f / (float)currentSampleRate.load(std::memory_order_relaxed);
                DSPCoreState* activeDSP = dspActiveState.load(std::memory_order_acquire);
                for(int j=0; j<10; j++) for(int k=0; k<5; k++) c_fx[j][k] = activeDSP->params[j][k];
                c_lat = activeDSP->latMode; c_act = activeDSP->activeMode; c_w = activeDSP->w; c_fz = activeDSP->fz; c_fb = activeDSP->fb; c_hr = activeDSP->hr; c_cp = activeDSP->cp; c_sy = activeDSP->sy; c_pd = activeDSP->pd; c_ch = activeDSP->ch; c_sw = activeDSP->sw; c_vb = activeDSP->vb; c_vg = activeDSP->vg;

                float c_pt = pitchShiftFactor.load(std::memory_order_acquire);
                float targetWindow = LATENCY_WINDOWS[c_lat];

                if(__builtin_expect(currentWindowSize!=targetWindow, 0)) {
                    currentWindowSize=targetWindow; uint32_t halfWindowFixed=((uint32_t)targetWindow/2)<<16;
                    tap_w3_1=0; tap_w3_2=halfWindowFixed; tap_w4_1=0; tap_w4_2=halfWindowFixed; tap_w5_1=0; tap_w5_2=halfWindowFixed;
                }

                uint32_t windowSizeLow = 4096; uint32_t windowSizeHigh = 512;
                uint32_t maskLow = windowSizeLow - 1; uint32_t maskHigh = windowSizeHigh - 1;
                uint32_t hannMultLow = (4096U<<16)/windowSizeLow; uint32_t hannMultHigh = (4096U<<16)/windowSizeHigh;
                uint32_t hannIntMult=(4096U<<16)/(uint32_t)currentWindowSize, windowMask=(uint32_t)currentWindowSize-1;

                float p_w_dry=c_fx[0][0], p_w_wet=c_fx[0][1], p_fz_apf=c_fx[1][0], p_fz_att=c_fx[1][1], p_fz_rel=c_fx[1][2], p_fb_spd=c_fx[2][0], p_fb_drv=c_fx[2][1], p_fb_off=c_fx[2][2], p_hr_mix=c_fx[3][0], p_sy_att=c_fx[5][0], p_sy_rel=c_fx[5][1], p_sy_flt=c_fx[5][2], p_sy_mix=c_fx[5][3], p_pd_sm=c_fx[6][0], p_pd_mix=c_fx[6][1], p_ch_spd=c_fx[7][0], p_ch_mix=c_fx[7][1], p_sw_thr=c_fx[8][0], p_sw_att=c_fx[8][1], p_sw_rel=c_fx[8][2], p_vb_dep=c_fx[9][0];

                float chorusPhaseIncr = p_ch_spd / (float)currentSampleRate.load(std::memory_order_acquire);
                float feedbackPhaseIncr = p_fb_spd / (float)currentSampleRate.load(std::memory_order_acquire);
                float targetPitch = FX_Whammy::getSpd(c_pt);
                bool frzActive=((c_act==1&&c_w)||c_fz);

                if(__builtin_expect(frzActive && !wasFrozen, 0)) { freezePlayCounterVar=0; int bestStart=freezeWriteIdxVar, tempIdx=freezeWriteIdxVar; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { bestStart=tempIdx; break; } tempIdx=prev; } freezeStartIdxVar=bestStart; activeFreezeLength=freezeLength; int searchEnd=bestStart-1; if(searchEnd<0) searchEnd+=freezeLength; tempIdx=searchEnd; for(int s=0; s<4000; s++) { int prev=tempIdx-1; if(prev<0) prev+=freezeLength; if(freezeBuffer[tempIdx]>=0 && freezeBuffer[prev]<0) { activeFreezeLength=s; break; } tempIdx=prev; } if(activeFreezeLength<64) activeFreezeLength=freezeLength; }
                if(__builtin_expect(!frzActive && wasFrozen, 0)) apfNeedsClear=true; wasFrozen=frzActive; float activeInvFreqLength=1.0f/(float)activeFreezeLength; bool synthActive=((c_act==5&&c_w)||c_sy), padActive=((c_act==6&&c_w)||c_pd), harmActive=((c_act==3&&c_w)||c_hr), swellActive=((c_act==8&&c_w)||c_sw), chorusActive=((c_act==7&&c_w)||c_ch), feedbackActive=((c_act==2&&c_w)||c_fb);

                if(__builtin_expect(feedbackActive && !wasFeedbackActive, 0)) { fbOutNode=0.0f; fbHpfState=0.0f; feedbackFilterVar=0.0f; } wasFeedbackActive=feedbackActive; bool vibratoActive=((c_act==9&&c_w)||c_vb), capoActive=((c_act==4&&c_w)||c_cp);

                bool localMonoPolyActive = isMonoPolyActive.load(std::memory_order_acquire);

                float localSwellGain=swellGain, localVolGain=c_vg, localFrzRamp=freezeRamp, localFbRamp=feedbackRamp, target_delay=constrain((float)(currentSampleRate.load(std::memory_order_acquire)*p_fb_off), 0.0f, (float)(FB_BUFFER_SIZE-1)); smoothed_delay_samples+= (target_delay-smoothed_delay_samples)*0.01f*srScale+DC_OFFSET; int delaySamples=(int)smoothed_delay_samples;

                float fbHpfCoeff = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? 0.025f : 0.05f;
                float fbLpfCoeff = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? 0.05f : 0.1f;
                float fbLpfRetain = 1.0f - fbLpfCoeff; float dc_alpha = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? 0.0005f : 0.001f;
                float meter_decay = (currentSampleRate.load(std::memory_order_acquire) == 96000) ? 0.999f : 0.998f;
                int halfWindow=(int)currentWindowSize/2; bool activeGroup=c_w||harmActive||chorusActive||feedbackActive||synthActive||padActive||frzActive||vibratoActive||capoActive, dryGroup=chorusActive||padActive||frzActive||feedbackActive||(localFrzRamp>0.0f)||(localFbRamp>0.0f), repeatGroup=capoActive||synthActive||vibratoActive||padActive||harmActive;

                float g_base=0.0f; if(dryGroup) { if(!repeatGroup) g_base=0.4f; } else if(harmActive) g_base=0.5f; else g_base=1.0f; float g_w2=harmActive?p_hr_mix:0.0f, g_w3=chorusActive?p_ch_mix:0.0f;

                bool padIsAudible = padActive || (fabsf(padVectorFilter.delay_state[0]) > 0.001f);
                float g_pad=padIsAudible?p_pd_mix:0.0f, g_frz=(!frzActive&&localFrzRamp>0.0f)?0.5f:0.0f, g_fb=(feedbackActive||localFbRamp>0.0f)?0.6f:0.0f, g_whammy=c_w?p_w_wet:0.0f, g_dry=c_w?p_w_dry:1.0f, vol_alpha=0.01f*srScale, envRetain=powf(0.99f,srScale), envAttack=1.0f-envRetain;

                if(__builtin_expect(isnan(synthFilter)||isinf(synthFilter), 0)) synthFilter=0.0f; if(__builtin_expect(isnan(synthBandpass)||isinf(synthBandpass), 0)) synthBandpass=0.0f; if(__builtin_expect(isnan(feedbackFilterVar)||isinf(feedbackFilterVar), 0)) feedbackFilterVar=0.0f; if(__builtin_expect(isnan(fbHpfState)||isinf(fbHpfState), 0)) fbHpfState=0.0f;

                float localVibPhase=vibratoLfoPhase, localChoPhase=chorusLfoPhase, localFbPhase=feedbackLfoPhase, localFbHpf=fbHpfState;

                float cross_alpha = (currentSampleRate.load(std::memory_order_relaxed) == 96000) ? 0.0163f : 0.0326f;
                int prefetchIdx = (writeIndex - halfWindow + MAX_BUFFER_SIZE) & BUFFER_MASK;
                if (__builtin_expect(MAX_BUFFER_SIZE - prefetchIdx >= HOP_SIZE, 1)) {
                    memcpy(activeDmaWriteBuf, &delayBuffer[prefetchIdx], HOP_SIZE * sizeof(int16_t));
                } else {
                    for (int i = 0; i < HOP_SIZE; i++) activeDmaWriteBuf[i] = delayBuffer[(prefetchIdx + i) & BUFFER_MASK];
                }
                int prefetchIdxFB = (fbDelayWriteIdx - delaySamples + FB_BUFFER_SIZE) & FB_BUFFER_MASK;
                int aheadFB = (prefetchIdxFB + 32) & FB_BUFFER_MASK;
                __builtin_prefetch(&fbDelayBuffer[aheadFB], 0, 3);

                float peakInputVal=0.0f, peakOutputVal=0.0f;
                DSPEngine::processInput(framesRead, i2s_in_block, normFactor, dc_alpha, envRetain, envAttack, p_sw_thr, p_sw_att, p_sw_rel, srScale, swellActive, localVolGain, vol_alpha, input_dc_offset, inputEnvelope, localSwellGain, smoothedVolGain, currentPitch, targetPitch, envBuf, masterGainBuf, inBuf, fzOutBuf);

                if(__builtin_expect(synthActive, 0)) for(int i=0; i<framesRead; i++) FX_Synth::process(synthActive, envBuf[i], p_sy_att, p_sy_rel, p_sy_flt, p_sy_mix, srScale, LUTManager::synthLUT, inBuf[i], synthEnv, synthFilter, synthBandpass);
                if(__builtin_expect(padActive, 0)) for(int i=0; i<framesRead; i++) FX_Pad::processEnv(padActive, envBuf[i], srScale, padEnv, inBuf[i]);

                for(int i=0; i<framesRead; i++) {
                    float procSample=inBuf[i];

                    if(__builtin_expect(apfNeedsClear, 0)) { memset(LUTManager::apf1Buffer,0,1009*sizeof(float)); memset(LUTManager::apf2Buffer,0,863*sizeof(float)); apf1Idx=0; apf2Idx=0; apfNeedsClear=false; }

                    FX_Freeze::process(frzActive, procSample, p_fz_att, p_fz_rel, srScale, p_fz_apf, activeInvFreqLength, activeFreezeLength, freezeLength, LUTManager::hannLUT, freezeBuffer, LUTManager::apf1Buffer, LUTManager::apf2Buffer, freezeWriteIdxVar, freezePlayCounterVar, freezeStartIdxVar, localFrzRamp, fzOutBuf[i], apf1Idx, apf2Idx);
                    float delayIn=(localFrzRamp>0.0f)?__builtin_fmaf(procSample, (1.0f-localFrzRamp), fzOutBuf[i]):procSample;

                    cross_lp1 = DSPEngine::AntiDenormal(cross_lp1 + cross_alpha * (delayIn - cross_lp1));
                    cross_lp2 = DSPEngine::AntiDenormal(cross_lp2 + cross_alpha * (cross_lp1 - cross_lp2));

                    float lowBand = cross_lp2; float highBand = delayIn - lowBand;

                    sramDryBlock[i] = (int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(delayIn,1.0f))*32767.0f);
                    sramPitchLow[sramWriteIdx] = (int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(lowBand,1.0f))*32767.0f);
                    sramPitchHigh[sramWriteIdx] = (int16_t)(__builtin_fmaxf(-1.0f,__builtin_fminf(highBand,1.0f))*32767.0f);

                    float spd1 = FX_Vibrato::getSpd(vibratoActive, currentPitch, localVibPhase, LUTManager::globalVibratoPhaseInc.load(std::memory_order_relaxed), p_vb_dep, LUTManager::lfoLUT);
                    float spd2 = FX_Harmony::getSpd(currentPitch, LUTManager::globalHarmRatio.load(std::memory_order_relaxed));
                    float spd3 = FX_Chorus::getSpd(chorusActive, currentPitch, LUTManager::globalChorusRatio.load(std::memory_order_relaxed), localChoPhase, chorusPhaseIncr, LUTManager::lfoLUT);
                    float spd4 = 1.0f, spd5 = 1.0f;

                    float w4 = 0.0f, w5 = 0.0f;
                    if (__builtin_expect(localMonoPolyActive, 0)) {
                        monoPolyEngine.process(
                            dryBuf[i],
                            targetPitch,
                            w4,
                            dryBuf[i],
                            c_fx[c_act][0],
                            c_fx[c_act][1],
                            c_fx[c_act][2],
                            c_fx[c_act][3],
                            c_fx[c_act][4]
                        );
                        w5 = 0.0f;
                    }
                    else if (__builtin_expect(feedbackActive || localFbRamp > 0.0f, 0)) {
                        w4 = DSPEngine::processDualPitchTap_SIMD(tap_w4_1, tap_w4_2, sramPitchHigh, sramWriteIdx, windowMask, hannIntMult, LUTManager::hannLUT, LUTManager::pitchSincLUT);
                        w5 = DSPEngine::processDualPitchTap_SIMD(tap_w5_1, tap_w5_2, sramPitchHigh, sramWriteIdx, windowMask, hannIntMult, LUTManager::hannLUT, LUTManager::pitchSincLUT);
                    }

                    FX_Feedback::process(feedbackActive, srScale, p_fb_drv, fbHpfCoeff, fbLpfCoeff, fbLpfRetain, LUTManager::globalFbRatio.load(std::memory_order_relaxed), currentPitch, LUTManager::lfoLUT, localFbPhase, feedbackPhaseIncr, wowRng, wowState, spd4, spd5, w4, w5, envBuf[i], localFbRamp, fzOutBuf[i], frzActive, localFrzRamp, localFbHpf, feedbackFilterVar, fbDelayBuffer, delaySamples, fbDelayWriteIdx, fbOutNode);

                    float w1_lo = DSPEngine::processDualPitchTap_SIMD(tap_w1_lo_1, tap_w1_lo_2, sramPitchLow, sramWriteIdx, maskLow, hannMultLow, LUTManager::hannLUT, LUTManager::pitchSincLUT);
                    float w1_hi = DSPEngine::processDualPitchTap_SIMD(tap_w1_hi_1, tap_w1_hi_2, sramPitchHigh, sramWriteIdx, maskHigh, hannMultHigh, LUTManager::hannLUT, LUTManager::pitchSincLUT);
                    float rawW1 = w1_lo + w1_hi;

                    float dampCutoff = (currentPitch > 1.498f) ? __builtin_fmaxf(0.1f, 1.0f - (currentPitch - 1.498f) * 0.5f) : 1.0f;
                    dampState = DSPEngine::AntiDenormal(__builtin_fmaf(dampCutoff, (rawW1 - dampState), dampState)); w1Buf[i] = dampState;

                    w2Buf[i]=0.0f;
                    if(__builtin_expect(harmActive, 0)) {
                        float w2_lo = DSPEngine::processDualPitchTap_SIMD(tap_w2_lo_1, tap_w2_lo_2, sramPitchLow, sramWriteIdx, maskLow, hannMultLow, LUTManager::hannLUT, LUTManager::pitchSincLUT);
                        float w2_hi = DSPEngine::processDualPitchTap_SIMD(tap_w2_hi_1, tap_w2_hi_2, sramPitchHigh, sramWriteIdx, maskHigh, hannMultHigh, LUTManager::hannLUT, LUTManager::pitchSincLUT);
                        w2Buf[i] = w2_lo + w2_hi;
                    }

                    w3Buf[i]=0.0f; if(__builtin_expect(chorusActive, 0)) w3Buf[i]=DSPEngine::processDualPitchTap_SIMD(tap_w3_1, tap_w3_2, sramPitchHigh, sramWriteIdx, windowMask, hannIntMult, LUTManager::hannLUT, LUTManager::pitchSincLUT);

                    int32_t step1=(int32_t)((1.0f-spd1)*65536.0f); tap_w1_lo_1+=step1; tap_w1_lo_2+=step1; tap_w1_hi_1+=step1; tap_w1_hi_2+=step1;
                    int32_t step2=(int32_t)((1.0f-spd2)*65536.0f); tap_w2_lo_1+=step2; tap_w2_lo_2+=step2; tap_w2_hi_1+=step2; tap_w2_hi_2+=step2;
                    int32_t step3=(int32_t)((1.0f-spd3)*65536.0f); tap_w3_1+=step3; tap_w3_2+=step3;
                    int32_t step4=(int32_t)((1.0f-spd4)*65536.0f); tap_w4_1+=step4; tap_w4_2+=step4;
                    int32_t step5=(int32_t)((1.0f-spd5)*65536.0f); tap_w5_1+=step5; tap_w5_2+=step5;

                    dryBuf[i] = (float)activeDmaReadBuf[i] * 3.0517578125e-5f; fbOutNode = DSPEngine::AntiDenormal(fbOutNode); fbOutBuf[i]=fbOutNode;
                    writeIndex=(writeIndex+1)&BUFFER_MASK; sramWriteIdx=(sramWriteIdx+1)&SRAM_PITCH_BUF_MASK;
                }

                vibratoLfoPhase=localVibPhase; chorusLfoPhase=localChoPhase; feedbackLfoPhase=localFbPhase; fbHpfState=localFbHpf;

                int16_t* tempDmaPtr = activeDmaReadBuf; activeDmaReadBuf = activeDmaWriteBuf; activeDmaWriteBuf = tempDmaPtr;
                int targetPsramIdx = (writeIndex - framesRead + MAX_BUFFER_SIZE) & BUFFER_MASK;
                if (__builtin_expect(MAX_BUFFER_SIZE - targetPsramIdx >= HOP_SIZE, 1)) {
                    memcpy(&delayBuffer[targetPsramIdx], sramDryBlock, HOP_SIZE * sizeof(int16_t));
                } else {
                    for (int i = 0; i < HOP_SIZE; i++) delayBuffer[(targetPsramIdx + i) & BUFFER_MASK] = sramDryBlock[i];
                }

                if (__builtin_expect(padActive, 0)) {
                    float padCutoff = __builtin_fmaxf(150.0f, (1.0f - p_pd_sm) * 12000.0f);
                    if (fabsf(padCutoff - lastPadCutoff) > 10.0f) { padVectorFilter.setLPF(padCutoff, currentSampleRate.load(std::memory_order_relaxed)); lastPadCutoff = padCutoff; }
                    padVectorFilter.process(w1Buf, padFilterBuf, framesRead);
                } else { memset(padFilterBuf, 0, framesRead * sizeof(float)); }
                for(int i = 0; i < framesRead; i++) FX_Pad::processDiffuser(chorusActive, padActive, padFilterBuf[i], w3Buf[i], diffuserBuf, diffuserIdx, w3Buf[i], padFilterBuf[i]);
                DSPEngine::mixdownAndOutput(framesRead, activeGroup, localFrzRamp, localFbRamp, g_whammy, g_dry, g_base, g_w2, g_w3, g_pad, g_frz, g_fb, padFilterBuf, dryBuf, w1Buf, w2Buf, w3Buf, fzOutBuf, fbOutBuf, sMixBuf, masterGainBuf, inBuf, i2s_out_block, peakInputVal, peakOutputVal);
                swellGain=localSwellGain; freezeRamp=localFrzRamp; feedbackRamp=localFbRamp;

                if (__builtin_expect(ui_clear_meters_requested.exchange(false, std::memory_order_acq_rel), 0)) {
                    ui_audio_level.store(0.0f, std::memory_order_release); ui_output_level.store(0.0f, std::memory_order_release);
                } else {
                    float current_in = ui_audio_level.load(std::memory_order_acquire);
                    if(peakInputVal > current_in) { ui_audio_level.store(peakInputVal, std::memory_order_release); }
                    else { current_in *= meter_decay; ui_audio_level.store((current_in < 1e-5f) ? 0.0f : current_in, std::memory_order_release); }

                    float current_out = ui_output_level.load(std::memory_order_acquire);
                    if(peakOutputVal > current_out) { ui_output_level.store(peakOutputVal, std::memory_order_release); }
                    else { current_out *= meter_decay; ui_output_level.store((current_out < 1e-5f) ? 0.0f : current_out, std::memory_order_release); }
                }

                uint32_t end_timer=xthal_get_ccount(); float max_cycles = (currentSampleRate.load(std::memory_order_relaxed) == 96000) ? (2500.0f * (float)framesRead) : (5000.0f * (float)framesRead);
                core0_dsp_load.store(__builtin_fmaf(core0_dsp_load.load(std::memory_order_relaxed), 0.95f, __builtin_fminf(100.0f, (((float)(end_timer - start_cycles) / max_cycles) * 100.0f)) * 0.05f), std::memory_order_relaxed);

                if(__builtin_expect(isMuted, 0)) memset(i2s_out_block, 0, framesRead * 2 * sizeof(int32_t));
                size_t bytesWrittenCount;
                i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, framesRead*8, &bytesWrittenCount, pdMS_TO_TICKS(20));
                if (__builtin_expect(activeDSP != lastAckedDSP, 0)) {
                    lastAckedDSP = activeDSP;
                    dspAckCommit.store(true, std::memory_order_release);
                }
            } else {
#ifdef ENABLE_ADVANCED_TELEMETRY
                audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
#endif
                memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
                size_t dummyBytes;
                i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, HOP_SIZE*8, &dummyBytes, 0);
                vTaskDelay(pdMS_TO_TICKS(1)); // Prevents tight CPU 0 loop when framesRead != HOP_SIZE
            }
        } else {
#ifdef ENABLE_ADVANCED_TELEMETRY
            audio_underflow_count.fetch_add(1, std::memory_order_relaxed);
#endif
            memset(i2s_out_block, 0, HOP_SIZE * 2 * sizeof(int32_t));
            size_t dummyBytes;
            i2s_channel_write((i2s_chan_handle_t)I2SManager::tx_chan, i2s_out_block, HOP_SIZE*8, &dummyBytes, 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // Prevents Task WDT trigger during I2S clock reconfig
        }
    }
}