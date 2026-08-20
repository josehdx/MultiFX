#pragma once
#include <Arduino.h>
#include "USB.h"
#include "USBMSC.h"
#include "esp_partition.h"

class USBMscManager {
private:
    static inline USBMSC MSC;
    static inline const esp_partition_t* ffat_partition = NULL;
    static inline uint8_t msc_cache[4096];
    static inline uint32_t msc_cache_sector = 0xFFFFFFFF;
    static inline bool msc_cache_dirty = false;
    static inline unsigned long msc_last_write_time = 0;

    static void flush_cb() {
        if (msc_cache_dirty && msc_cache_sector != 0xFFFFFFFF && ffat_partition) {
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
        return true;
    }

public:
    static void init() {
        ffat_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");
        if (ffat_partition) {
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
        }
    }

    static void handleCacheFlush() {
        if (msc_cache_dirty && (millis() - msc_last_write_time > 500)) {
            flush_cb();
        }
    }
};