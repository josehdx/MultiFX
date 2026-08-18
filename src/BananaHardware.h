#ifndef BANANA_HARDWARE_H
#define BANANA_HARDWARE_H

#include <Arduino.h>
#include <driver/i2s_std.h>
#include "esp_adc/adc_continuous.h"
#include <atomic>

class BananaHardware {
public:
    // Configures external hardware UART0 on GPIO 43 (TX) and GPIO 44 (RX)
    static void initUART(uint32_t baudRate = 115200) {
        Serial0.begin(baudRate, SERIAL_8N1, 44, 43); // RX: 44, TX: 43[cite: 4]
    }

    static i2s_std_config_t getI2SConfig(uint32_t sampleRate) {
        i2s_std_config_t stdConfig = { 
             .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate),
             .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
             .gpio_cfg = {
                 .mclk = GPIO_NUM_6,   // Frees GPIO 43 for TX[cite: 6]
                 .bclk = GPIO_NUM_7,   // Frees GPIO 44 for RX[cite: 6]
                 .ws   = GPIO_NUM_18,  //[cite: 6]
                 .dout = GPIO_NUM_21,  //[cite: 6]
                 .din  = GPIO_NUM_17   //[cite: 6]
             } 
         };
        stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; //[cite: 6]
        return stdConfig; //[cite: 6]
    }

    static int getBatteryPercentage(float voltage) {
        return 100; //[cite: 6]
    }

    static void fetchADCDMA(adc_continuous_handle_t handle, volatile bool& isPaused, volatile int& pb1, volatile int& pb2, volatile int& pb3, std::atomic<int>& bat) {
        if(isPaused) return; //[cite: 6]
        uint8_t result[128] __attribute__((aligned(4))); //[cite: 6]
        uint32_t ret_num = 0; //[cite: 6]
        esp_err_t err; //[cite: 6]
        int loop_bound = 0; //[cite: 6]
        
        while(loop_bound++ < 4) { //[cite: 6]
            err = adc_continuous_read(handle, result, sizeof(result), &ret_num, 0); //[cite: 6]
            if(err == ESP_OK && ret_num > 0) { //[cite: 6]
                for(int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) { //[cite: 6]
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i]; //[cite: 6]
                    if(p->type2.channel == ADC_CHANNEL_0) pb1 = p->type2.data; //[cite: 6]
                    if(p->type2.channel == ADC_CHANNEL_1) pb2 = p->type2.data; //[cite: 6]
                    if(p->type2.channel == ADC_CHANNEL_9) pb3 = p->type2.data; //[cite: 6]
                    if(p->type2.channel == ADC_CHANNEL_3) bat.store(p->type2.data, std::memory_order_relaxed); //[cite: 6]
                }
            } else if (err == ESP_ERR_TIMEOUT) { //[cite: 6]
                break; //[cite: 6]
            } else if (err == ESP_ERR_INVALID_STATE) { //[cite: 6]
                adc_continuous_stop(handle); //[cite: 6]
                vTaskDelay(pdMS_TO_TICKS(2)); //[cite: 6]
                adc_continuous_start(handle); //[cite: 6]
                break; //[cite: 6]
            } else {
                break; //[cite: 6]
            }
        }
    }
};

#endif