#ifndef MIDI_ROUTER_H
#define MIDI_ROUTER_H

#include <Arduino.h>
#include <math.h>

class MidiRouter {
public:
    static void updateParameter(uint8_t cc, uint8_t val, int currentMode, volatile float* effectMemory, float fxParams[10][5], volatile bool& lutNeedsUpdate, volatile bool& dspNeedsCommit, std::atomic<int>& fbIdx) {
        float norm=(float)val/127.0f; int pIdx=cc-24;
        if(currentMode==0) { 
            if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==2) fxParams[0][0]=norm; if(pIdx==3) fxParams[0][1]=norm; 
        }
        else if(currentMode==1) { 
            if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==2) fxParams[1][0]=0.0f+(norm*0.95f); if(pIdx==3) fxParams[1][1]=0.00001f+(norm*0.001f); if(pIdx==4) fxParams[1][2]=0.00001f+(norm*0.0005f); 
        }
        else if(currentMode==2) { 
            if(pIdx==0) { int newIdx=constrain((int)roundf(norm*4.0f),0,4); if(newIdx!=fbIdx.load(std::memory_order_acquire)) { fbIdx.store(newIdx, std::memory_order_release); lutNeedsUpdate=true; } } 
            if(pIdx==1) fxParams[2][0]=1000.0f+(norm*10000.0f); if(pIdx==2) fxParams[2][1]=1.0f+(norm*100.0f); if(pIdx==3) fxParams[2][2]=0.005f+(norm*0.045f); 
        }
        else if(currentMode==3) { 
            if(pIdx==0) { effectMemory[3]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) fxParams[3][0]=norm; 
        }
        else if(currentMode==4) { 
            if(pIdx==0) { int cents=(int)roundf((effectMemory[4]-(float)roundf(effectMemory[4]))*100.0f); effectMemory[4]=constrain(roundf((norm*48.0f)-24.0f)+((float)cents/100.0f),-24.0f,24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) { int semi=(int)roundf(effectMemory[4]); float c=roundf((norm*100.0f)-50.0f)/100.0f; effectMemory[4]=constrain((float)semi+c,-24.0f,24.0f); lutNeedsUpdate=true; } 
        }
        else if(currentMode==5) { 
            if(pIdx==0) { effectMemory[5]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) fxParams[5][0]=0.01f+(norm*0.5f); if(pIdx==2) fxParams[5][1]=0.001f+(norm*0.05f); if(pIdx==3) fxParams[5][2]=0.1f+(norm*0.8f); if(pIdx==4) fxParams[5][3]=norm; 
        }
        else if(currentMode==6) { 
            if(pIdx==0) { effectMemory[6]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) fxParams[6][0]=0.8f+(norm*0.199f); if(pIdx==2) fxParams[6][1]=norm*3.0f; 
        }
        else if(currentMode==7) { 
            if(pIdx==0) { effectMemory[7]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) fxParams[7][0]=500.0f+(norm*4500.0f); if(pIdx==2) fxParams[7][1]=norm; 
        }
        else if(currentMode==8) { 
            if(pIdx==0) { effectMemory[1]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) { effectMemory[0]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==2) fxParams[8][0]=0.001f+(norm*0.05f); if(pIdx==3) fxParams[8][1]=0.00001f+(norm*0.001f); if(pIdx==4) fxParams[8][2]=0.00001f+(norm*0.0005f); 
        }
        else if(currentMode==9) { 
            if(pIdx==0) { effectMemory[9]=roundf((norm*48.0f)-24.0f); lutNeedsUpdate=true; } 
            if(pIdx==1) fxParams[9][0]=norm*2.0f; 
        }
        dspNeedsCommit = true;
    }
};

#endif