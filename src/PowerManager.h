#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include <driver/i2s_std.h>
#include "esp_adc/adc_continuous.h"
#include <atomic>

class PowerManager {
public:
    static i2s_std_config_t getI2SConfig(uint32_t sampleRate) {
        i2s_std_config_t stdConfig = {
             .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate),
             .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
             .gpio_cfg = {
                 .mclk = GPIO_NUM_43,
                 .bclk = GPIO_NUM_44,
                 .ws   = GPIO_NUM_18,
                 .dout = GPIO_NUM_21,
                 .din  = GPIO_NUM_17
             }
         };
        stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
        return stdConfig;
    }

    static float getBatteryVoltage(int raw) {
        return ((float)raw / 4095.0f) * 2.0f * 3.3f;
    }

    static int getBatteryPercentage(float voltage) {
        float cv = __builtin_fmaxf(3.30f, __builtin_fminf(4.15f, voltage));
        if(cv>=4.15f) return 100; if(cv<=3.30f) return 0; 
        if(cv>=4.00f) return 90+(int)((cv-4.00f)/0.15f*10.0f); 
        if(cv>=3.90f) return 80+(int)((cv-3.90f)/0.10f*10.0f); 
        if(cv>=3.80f) return 70+(int)((cv-3.80f)/0.10f*10.0f); 
        if(cv>=3.75f) return 60+(int)((cv-3.75f)/0.05f*10.0f); 
        if(cv>=3.70f) return 50+(int)((cv-3.70f)/0.05f*10.0f); 
        if(cv>=3.65f) return 40+(int)((cv-3.65f)/0.05f*10.0f); 
        if(cv>=3.60f) return 30+(int)((cv-3.60f)/0.10f*10.0f); 
        if(cv>=3.55f) return 20+(int)((cv-3.55f)/0.05f*10.0f); 
        if(cv>=3.50f) return 10+(int)((cv-3.50f)/0.05f*10.0f); 
        return (int)(__builtin_fmaxf(0.0f,cv-3.30f)/0.20f*10.0f);
    }

    static void fetchADCDMA(adc_continuous_handle_t handle, volatile bool& isPaused, volatile int& pb1, volatile int& pb2, volatile int& pb3, volatile int& par1, std::atomic<int>& bat) {
        if(isPaused) return;
        uint8_t result[128] __attribute__((aligned(4)));
        uint32_t ret_num=0; esp_err_t err; int loop_bound = 0;
        
        while(loop_bound++ < 4) {
            err=adc_continuous_read(handle, result, sizeof(result), &ret_num, 0);
            if(err==ESP_OK && ret_num>0) {
                for(int i=0; i<ret_num; i+=SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p=(adc_digi_output_data_t*)&result[i];
                    if(p->type2.channel==ADC_CHANNEL_0) pb1=p->type2.data;
                    if(p->type2.channel==ADC_CHANNEL_1) pb2=p->type2.data;
                    if(p->type2.channel==ADC_CHANNEL_9) pb3=p->type2.data;
                    if(p->type2.channel==ADC_CHANNEL_2) par1=p->type2.data; // Mapped PAR 1 natively
                    if(p->type2.channel==ADC_CHANNEL_3) bat.store(p->type2.data, std::memory_order_relaxed);
                }
            } else if (err == ESP_ERR_TIMEOUT) { 
                break; 
            } else if (err == ESP_ERR_INVALID_STATE) {
                adc_continuous_stop(handle); vTaskDelay(pdMS_TO_TICKS(2)); adc_continuous_start(handle); break; 
            } else { 
                break; 
            }
        }
    }
};

#endif