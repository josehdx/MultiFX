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
    
    // Dedicated background worker handle
    static inline TaskHandle_t usbMscTaskHandle = NULL;

    static void ensureDSPPaused() {
        if (!dsp_is_paused.load(std::memory_order_acquire)) {
            Serial0.println("[USB] File copy in progress: Stopping DSP to protect flash...");
            dsp_is_paused.store(true, std::memory_order_release);
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
    // Programmatically generates a standard MBR + FAT12 structure on first boot.
    // Handles any partition size (LilyGO 16MB or BananaPi 8MB) dynamically.
    static void ensureMBR() {
        if (!ffat_partition) return;

        const uint32_t SECTOR_SIZE = 4096;
        static uint8_t sec[4096];
        if (esp_partition_read(ffat_partition, 0, sec, SECTOR_SIZE) != ESP_OK) return;

        // Check if Sector 0 has a valid MBR (signature 0x55AA and partition type 0x01 FAT12)
        if (sec[510] == 0x55 && sec[511] == 0xAA && sec[446 + 4] == 0x01) {
            return; // Valid MBR already provisioned
        }

        Serial0.println("[USB] Provisioning FAT12 filesystem with MBR partition table...");

        uint32_t totalSectors = ffat_partition->size / SECTOR_SIZE;
        uint32_t partSectors = totalSectors - 1; // Sector 0 is MBR; partition starts at Sector 1

        // 1. SECTOR 0: Master Boot Record (MBR)
        memset(sec, 0, SECTOR_SIZE);
        uint8_t* p = &sec[446]; // Partition Entry 1
        p[0] = 0x00;            // Status: non-bootable
        p[1] = 0x01; p[2] = 0x01; p[3] = 0x00; // Starting CHS
        p[4] = 0x01;            // Partition Type: 0x01 (FAT12)
        p[5] = 0xFE; p[6] = 0xFF; p[7] = 0xFF; // Ending CHS
        *(uint32_t*)&p[8]  = 1;           // Starting LBA: Sector 1
        *(uint32_t*)&p[12] = partSectors; // Sector Count
        sec[510] = 0x55;
        sec[511] = 0xAA;

        esp_partition_erase_range(ffat_partition, 0, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 0, sec, SECTOR_SIZE);

        // 2. SECTOR 1: FAT12 Volume Boot Record (BPB)
        memset(sec, 0, SECTOR_SIZE);
        sec[0] = 0xEB; sec[1] = 0x3C; sec[2] = 0x90; // Jump boot
        memcpy(&sec[3], "MSDOS5.0", 8);              // OEM Name
        *(uint16_t*)&sec[11] = 4096;                 // Bytes per sector (4KB)
        sec[13] = 1;                                 // Sectors per cluster (1 sector = 4KB)
        *(uint16_t*)&sec[14] = 1;                    // Reserved sector count (Sector 1)
        sec[16] = 2;                                 // Number of FATs
        *(uint16_t*)&sec[17] = 512;                  // Root directory entries (512 * 32 = 16KB = 4 sectors)
        *(uint16_t*)&sec[19] = (partSectors <= 65535) ? (uint16_t)partSectors : 0; // Total sectors (16-bit)
        sec[21] = 0xF8;                              // Media descriptor: Fixed Disk
        *(uint16_t*)&sec[22] = 1;                    // Sectors per FAT (1 sector covers up to 2730 FAT12 clusters)
        *(uint16_t*)&sec[24] = 32;                   // Sectors per track
        *(uint16_t*)&sec[26] = 16;                   // Number of heads
        *(uint32_t*)&sec[28] = 1;                    // Hidden sectors (LBA Start = 1)
        *(uint32_t*)&sec[32] = (partSectors > 65535) ? partSectors : 0; // Total sectors (32-bit)
        sec[36] = 0x80;                              // Drive number
        sec[38] = 0x29;                              // Extended boot signature
        *(uint32_t*)&sec[39] = 0x12345678;           // Volume serial number
        memcpy(&sec[43], "MULTIFX    ", 11);         // Volume label (11 characters)
        memcpy(&sec[54], "FAT12   ", 8);             // Filesystem type
        sec[510] = 0x55;
        sec[511] = 0xAA;

        esp_partition_erase_range(ffat_partition, 1 * SECTOR_SIZE, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 1 * SECTOR_SIZE, sec, SECTOR_SIZE);

        // 3. SECTOR 2 & 3: FAT 1 and FAT 2 Tables
        memset(sec, 0, SECTOR_SIZE);
        sec[0] = 0xF8; sec[1] = 0xFF; sec[2] = 0xFF; // FAT12 media descriptor & end-of-chain marks

        esp_partition_erase_range(ffat_partition, 2 * SECTOR_SIZE, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 2 * SECTOR_SIZE, sec, SECTOR_SIZE);

        esp_partition_erase_range(ffat_partition, 3 * SECTOR_SIZE, SECTOR_SIZE);
        esp_partition_write(ffat_partition, 3 * SECTOR_SIZE, sec, SECTOR_SIZE);

        // 4. SECTORS 4 THROUGH 7: Root Directory Area (512 entries / 128 per sector = 4 sectors)
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
            // Guarantee valid MBR and FAT12 tables exist prior to TinyUSB startup
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

            // Dedicated worker on Core 1 prevents USB operations from stalling DSP pipeline
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
        if (msc_cache_dirty && (millis() - msc_last_write_time > 2000)) {
            flush_cb();
            
            if (dsp_is_paused.load(std::memory_order_acquire)) {
                Serial0.println("[USB] File copy complete: Resuming DSP...");
                dsp_is_paused.store(false, std::memory_order_release);
                
                if (audioTaskHandle) xTaskNotifyGive(audioTaskHandle);
                if (lutTaskHandle) xTaskNotifyGive(lutTaskHandle);
            }
        }
    }
};