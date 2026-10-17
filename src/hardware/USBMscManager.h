#pragma once
#include <Arduino.h>
#include "USB.h"
#include "USBMSC.h"
#include "esp_partition.h"
#include "SystemState.h"
#include "I2SManager.h"
#include "BoardHAL.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class USBMscManager {
private:
    static inline USBMSC MSC;
    static inline const esp_partition_t* ffat_partition = NULL;
    static inline uint8_t msc_cache[4096];
    static inline uint32_t msc_cache_sector = 0xFFFFFFFF;
    static inline bool msc_cache_dirty = false;
    static inline unsigned long msc_last_write_time = 0;
    
    // Safety flag to prevent MSC from unpausing DSP if a different system triggered the pause
    static inline bool msc_paused_dsp = false; 

    // Dedicated background worker handle
    static inline TaskHandle_t usbMscTaskHandle = NULL;

    static void ensureDSPPaused() {
        if (!dsp_is_paused.load(std::memory_order_acquire)) {
            Serial0.println("[USB] File copy in progress: Stopping DSP to protect flash...");
            
            // 1. Tell the tasks to park (The audio task will disable I2S automatically)
            dsp_is_paused.store(true, std::memory_order_release);
            
            // 2. Wait for tasks to acknowledge they are parked
            while(!dsp_ack_parked.load(std::memory_order_acquire) || !lut_ack_parked.load(std::memory_order_acquire)) {
                 vTaskDelay(pdMS_TO_TICKS(1)); 
             }
            msc_paused_dsp = true;
        }
    }

    static void flush_cb() {
        if (msc_cache_dirty && msc_cache_sector != 0xFFFFFFFF && ffat_partition) {
            ensureDSPPaused();
            esp_partition_erase_range(ffat_partition, msc_cache_sector * 4096, 4096);
            esp_partition_write(ffat_partition, msc_cache_sector * 4096, msc_cache, 4096);
            msc_cache_dirty = false;
        }
    }

    static int32_t read_cb(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
        if (!ffat_partition) return -1;

        uint32_t abs_offset = (lba * 4096) + offset;
        esp_partition_read(ffat_partition, abs_offset, buffer, bufsize);
        
        if (msc_cache_dirty && msc_cache_sector != 0xFFFFFFFF) {
            uint32_t cache_start = msc_cache_sector * 4096;
            uint32_t cache_end = cache_start + 4096;
            uint32_t read_start = abs_offset;
            uint32_t read_end = abs_offset + bufsize;

            if (read_start < cache_end && read_end > cache_start) {
                uint32_t overlap_start = max(read_start, cache_start);
                uint32_t overlap_end = min(read_end, cache_end);
                uint32_t overlap_len = overlap_end - overlap_start;
                uint32_t buffer_offset = overlap_start - read_start;
                uint32_t cache_offset = overlap_start - cache_start;

                memcpy((uint8_t*)buffer + buffer_offset, msc_cache + cache_offset, overlap_len);
            }
        }
        return bufsize;
    }

    static int32_t write_cb(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
        if (!ffat_partition) return -1;

        uint32_t abs_offset = (lba * 4096) + offset;
        for (uint32_t i = 0; i < bufsize; ) {
            uint32_t current_addr = abs_offset + i;
            uint32_t sector = current_addr / 4096;
            uint32_t sec_offset = current_addr % 4096;
            uint32_t bytes_to_write = min(bufsize - i, 4096 - sec_offset);
            
            if (sector != msc_cache_sector) {
                flush_cb();
                msc_cache_sector = sector;
                esp_partition_read(ffat_partition, sector * 4096, msc_cache, 4096);
            }

            memcpy(msc_cache + sec_offset, buffer + i, bytes_to_write);
            msc_cache_dirty = true;
            i += bytes_to_write;
        }

        msc_last_write_time = millis();
        return bufsize;
    }

    static bool start_stop_cb(uint8_t power_condition, bool start, bool load_eject) {
        flush_cb();
        if (!start) {
            Serial0.println("[USB] USB Storage safely ejected by host OS.");
        }
        return true;
    }

    static void usbMscWorkerTask(void* pvParameters) {
        for (;;) {
            handleCacheFlush();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

public:
    static void ensureMBR() {
        if (!ffat_partition) return;
        const uint32_t SECTOR_SIZE = 4096;
        static uint8_t sec[4096];

        if (esp_partition_read(ffat_partition, 0, sec, SECTOR_SIZE) != ESP_OK) return;

        if (sec[510] == 0x55 && sec[511] == 0xAA && sec[446 + 4] == 0x01) return;

        Serial0.println("[USB] Provisioning FAT12 filesystem with MBR partition table...");

        uint32_t totalSectors = ffat_partition->size / SECTOR_SIZE;
        uint32_t partSectors = totalSectors - 1;

        memset(sec, 0, SECTOR_SIZE);
        uint8_t* p = &sec[446]; 
        p[0] = 0x00;            
        p[1] = 0x01; p[2] = 0x01; p[3] = 0x00; 
        p[4] = 0x01;            
        p[5] = 0xFE; p[6] = 0xFF; p[7] = 0xFF; 
        *(uint32_t*)&p[8]  = 1;           
        *(uint32_t*)&p[12] = partSectors; 
        sec[510] = 0x55;
        sec[511] = 0xAA;
        esp_partition_erase_range(ffat_partition, 0, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 0, sec, SECTOR_SIZE);

        memset(sec, 0, SECTOR_SIZE);
        sec[0] = 0xEB; sec[1] = 0x3C; sec[2] = 0x90; 
        memcpy(&sec[3], "MSDOS5.0", 8);              
        *(uint16_t*)&sec[11] = 4096;                 
        sec[13] = 1;                                 
        *(uint16_t*)&sec[14] = 1;                    
        sec[16] = 2;                                 
        *(uint16_t*)&sec[17] = 512;                  
        *(uint16_t*)&sec[19] = (partSectors <= 65535) ? (uint16_t)partSectors : 0; 
        sec[21] = 0xF8;                              
        *(uint16_t*)&sec[22] = 1;                    
        *(uint16_t*)&sec[24] = 32;                   
        *(uint16_t*)&sec[26] = 16;                   
        *(uint32_t*)&sec[28] = 1;                    
        *(uint32_t*)&sec[32] = (partSectors > 65535) ? partSectors : 0; 
        sec[36] = 0x80;                              
        sec[38] = 0x29;                              
        *(uint32_t*)&sec[39] = 0x12345678;           
        memcpy(&sec[43], "MULTIFX    ", 11);         
        memcpy(&sec[54], "FAT12   ", 8);             
        sec[510] = 0x55;
        sec[511] = 0xAA;

        esp_partition_erase_range(ffat_partition, 1 * SECTOR_SIZE, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 1 * SECTOR_SIZE, sec, SECTOR_SIZE);

        memset(sec, 0, SECTOR_SIZE);
        sec[0] = 0xF8; sec[1] = 0xFF; sec[2] = 0xFF; 
        esp_partition_erase_range(ffat_partition, 2 * SECTOR_SIZE, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 2 * SECTOR_SIZE, sec, SECTOR_SIZE);

        esp_partition_erase_range(ffat_partition, 3 * SECTOR_SIZE, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 3 * SECTOR_SIZE, sec, SECTOR_SIZE);

        memset(sec, 0, SECTOR_SIZE);
        for (int s = 4; s <= 7; s++) {
            esp_partition_erase_range(ffat_partition, s * SECTOR_SIZE, SECTOR_SIZE);
            esp_partition_write(ffat_partition, s * SECTOR_SIZE, sec, SECTOR_SIZE);
        }

        Serial0.println("[USB] Partition layout written. Volume ready to mount.");
    }

    static void init() {
        ffat_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");
        if (ffat_partition) {
            ensureMBR();
            uint32_t block_count = ffat_partition->size / 4096;
            
            MSC.vendorID("MultiFX");
            MSC.productID("Storage");
            MSC.productRevision("1.0");
            MSC.onRead(read_cb);
            MSC.onWrite(write_cb);
            MSC.onStartStop(start_stop_cb);
            MSC.mediaPresent(true);
            MSC.isWritable(true);
            MSC.begin(block_count, 4096);
            
            Serial0.println("[USB] Mass Storage Class (MSC) initialized with 4K Sectors.");

            xTaskCreatePinnedToCore(
                usbMscWorkerTask, 
                "USB_MSC_Task", 
                4096, 
                NULL, 
                2, 
                &usbMscTaskHandle, 
                1 
            );
        }
    }

    static void handleCacheFlush() {
        // CRITICAL FIX: Detect physical USB detachment and forcefully trigger flush/unpause
        bool usbConnected = (bool)USB;
        
        if (msc_cache_dirty && (!usbConnected || (millis() - msc_last_write_time > 2000))) {
            flush_cb();
        }

        if (msc_paused_dsp && !msc_cache_dirty && (!usbConnected || (millis() - msc_last_write_time > 2000))) {
            Serial0.println("[USB] File copy complete or physically disconnected: Resuming DSP...");
            
            // Re-init the I2S channels using the currently saved sample rate
            I2SManager::initChannels(BoardHAL::getI2SConfig(currentSampleRate.load(std::memory_order_acquire)), HOP_SIZE);
            I2SManager::enableChannels();
            
            // Request full audio buffer zero-out before unpausing to prevent artifacts
            globalAudioResetRequested.store(true, std::memory_order_release);
            std::atomic_thread_fence(std::memory_order_seq_cst);
            dsp_is_paused.store(false, std::memory_order_release);
            msc_paused_dsp = false;
            
            if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
            if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
        }
    }
};