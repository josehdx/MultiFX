#ifndef I2S_MANAGER_H
#define I2S_MANAGER_H

#include <Arduino.h>
#include <driver/i2s_std.h>

class I2SManager {
public:
    inline static volatile i2s_chan_handle_t tx_chan = NULL;
    inline static volatile i2s_chan_handle_t rx_chan = NULL;

    static void initChannels(i2s_std_config_t stdConfig, uint32_t hopSize) {
        i2s_chan_config_t i2sConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
        
        // CRITICAL FIX: Align DMA frames strictly to hopSize (64) to eliminate burst-processing jitter.
        // 24 descriptors * 64 frames = 1536 frames (~16ms buffer at 96kHz).
        i2sConfig.dma_desc_num = 24; 
        i2sConfig.dma_frame_num = hopSize; 
        i2sConfig.auto_clear = true;
        
        i2s_chan_handle_t t_tx = NULL, t_rx = NULL; 
        esp_err_t err = i2s_new_channel(&i2sConfig, &t_tx, &t_rx);
        
        if (err == ESP_OK) {
            // CRITICAL FIX: Initialize the channels BEFORE exposing them to the global pointers.
            // If the DSP task reads the channel while it is still configuring, it causes a fatal state corruption.
            i2s_channel_init_std_mode(t_tx, &stdConfig); 
            i2s_channel_init_std_mode(t_rx, &stdConfig);
            
            tx_chan = t_tx; 
            rx_chan = t_rx;
        } else {
            Serial0.printf("[I2S] CRITICAL ERROR: Failed to allocate DMA channels: %d\n", err);
        }
    }

    static void enableChannels() { 
        if (tx_chan) i2s_channel_enable(tx_chan); 
        if (rx_chan) i2s_channel_enable(rx_chan); 
    }

    static void disableChannels() {
        if (tx_chan) i2s_channel_disable(tx_chan);
        if (rx_chan) i2s_channel_disable(rx_chan);
    }

    static void disableAndDestroyChannels() {
        if(tx_chan) { i2s_channel_disable(tx_chan); i2s_del_channel(tx_chan); tx_chan = NULL; }
        if(rx_chan) { i2s_channel_disable(rx_chan); i2s_del_channel(rx_chan); rx_chan = NULL; }
    }
};
#endif