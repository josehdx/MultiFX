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
        i2sConfig.dma_desc_num = 8; i2sConfig.dma_frame_num = hopSize; i2sConfig.auto_clear = true; 
        i2s_chan_handle_t t_tx, t_rx; i2s_new_channel(&i2sConfig, &t_tx, &t_rx); 
        tx_chan = t_tx; rx_chan = t_rx;
        i2s_channel_init_std_mode(tx_chan, &stdConfig); i2s_channel_init_std_mode(rx_chan, &stdConfig);
    }

    static void enableChannels() { i2s_channel_enable(tx_chan); i2s_channel_enable(rx_chan); }
    static void disableAndDestroyChannels() {
        if(tx_chan) { i2s_channel_disable(tx_chan); i2s_del_channel(tx_chan); tx_chan = NULL; }
        if(rx_chan) { i2s_channel_disable(rx_chan); i2s_del_channel(rx_chan); rx_chan = NULL; }
    }
};

#endif