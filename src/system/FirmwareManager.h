#pragma once

#include <Arduino.h>
#include <Update.h>
#include <dirent.h>
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_vfs_fat.h"
#include "esp_app_format.h"

#if defined(TARGET_LILYGO)
#include "DisplayManager.h"
extern DisplayManager displayManager;
#endif

class FirmwareManager {
private:
    static void logOutput(const char* msg, uint16_t color = 0x07E0) {
        Serial.println(msg);
        Serial0.println(msg);
#if defined(TARGET_LILYGO)
        displayManager.drawBootLogLine(msg, color);
#endif
    }

    static void deleteFirmwareFileRaw() {
        const esp_partition_t* part = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");

        if (!part) return;

        uint8_t sectorBuf[4096];
        bool fileDeleted = false;

        uint32_t maxScanBytes = min((uint32_t)part->size, (uint32_t)(1024 * 1024));

        for (uint32_t offset = 0; offset < maxScanBytes; offset += 4096) {
            if (esp_partition_read(part, offset, sectorBuf, 4096) != ESP_OK) continue;

            bool sectorModified = false;
            for (int i = 0; i < 4096; i += 32) {
                uint8_t firstByte = sectorBuf[i];
                if (firstByte == 0x00) break;
                if (firstByte == 0xE5) continue;

                bool isTargetSFN = (memcmp(&sectorBuf[i], "FIRMWARE", 8) == 0) ||
                                   (memcmp(&sectorBuf[i], "FIRMWA~1", 8) == 0);

                bool isTargetLFN = false;
                if (sectorBuf[i + 11] == 0x0F) {
                    if ((sectorBuf[i + 1] == 'f' || sectorBuf[i + 1] == 'F') &&
                        (sectorBuf[i + 3] == 'i' || sectorBuf[i + 3] == 'I') &&
                        (sectorBuf[i + 5] == 'r' || sectorBuf[i + 5] == 'R') &&
                        (sectorBuf[i + 7] == 'm' || sectorBuf[i + 7] == 'M')) {
                        isTargetLFN = true;
                    }
                }

                if (isTargetSFN || isTargetLFN) {
                    sectorBuf[i] = 0xE5;
                    sectorModified = true;
                    fileDeleted = true;
                }
            }

            if (sectorModified) {
                esp_partition_erase_range(part, offset, 4096);
                esp_partition_write(part, offset, sectorBuf, 4096);
            }
        }

        if (fileDeleted) {
            logOutput("[OTA] firmware.bin erased from storage.");
        }
    }

    static bool scanAndFlashRawPartition(const esp_partition_t* part) {
        logOutput("[OTA] Scanning partition sectors directly...", 0xFFE0);

        uint8_t sec[4096];
        uint32_t foundOffset = 0;
        bool found = false;

        uint32_t scanLimit = min((uint32_t)part->size, (uint32_t)(4 * 1024 * 1024));

        for (uint32_t off = 0; off < scanLimit; off += 4096) {
            if (esp_partition_read(part, off, sec, 4096) != ESP_OK) continue;

            // Check for ESP32-S3 app binary magic byte 0xE9 and descriptor magic 0xABCD5432
            if (sec[0] == 0xE9) {
                uint32_t descMagic = *(uint32_t*)&sec[0x20];
                if (descMagic == 0xABCD5432) {
                    foundOffset = off;
                    found = true;
                    break;
                }
            }
        }

        if (!found) {
            logOutput("[OTA] No firmware image found on flash.");
            delay(500);
            return false;
        }

        esp_app_desc_t new_app;
        esp_partition_read(part, foundOffset + 0x20, &new_app, sizeof(esp_app_desc_t));

        const esp_partition_t* running = esp_ota_get_running_partition();
        esp_app_desc_t running_app;
        esp_ota_get_partition_description(running, &running_app);

        if (memcmp(new_app.app_elf_sha256, running_app.app_elf_sha256, sizeof(new_app.app_elf_sha256)) == 0) {
            logOutput("[OTA] Binary matches running image. Done.");
            deleteFirmwareFileRaw();
            delay(800);
            return false;
        }

        char infoBuf[96];
        snprintf(infoBuf, sizeof(infoBuf), "[OTA] Flashing %s (%s)...", new_app.project_name, new_app.version);
        logOutput(infoBuf, 0x07FF);

        // Estimate size from binary header
        uint32_t binSize = *(uint32_t*)&sec[foundOffset + 12];
        if (binSize < 65536 || binSize > 3 * 1024 * 1024) binSize = 1800000;

        if (!Update.begin(binSize, U_FLASH)) {
            logOutput("[OTA] ERROR: Update.begin failed!", 0xF800);
            return false;
        }

        size_t written = 0;
        int lastPct = -1;

        for (uint32_t off = foundOffset; off < foundOffset + binSize; off += 4096) {
            esp_partition_read(part, off, sec, 4096);
            Update.write(sec, 4096);
            written += 4096;

            int pct = (int)((written * 100) / binSize);
            if (pct != lastPct && pct % 10 == 0) {
                lastPct = pct;
#if defined(TARGET_LILYGO)
                displayManager.drawBootProgress(pct);
#endif
            }
        }

        if (Update.end(true)) {
            logOutput("[OTA] Update complete! Rebooting...", 0x07E0);
            
            // Erase the first sector of the found firmware payload to destroy the 0xE9 magic byte
            esp_partition_erase_range(part, foundOffset, 4096);
            
            deleteFirmwareFileRaw();
            delay(1200);
            ESP.restart();
            return true;
        }

        return false;
    }

public:
    static void checkAndApplyUpdate() {
        logOutput("[SYSTEM] Checking storage partition...");

        const esp_partition_t* part = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");
            
        if (!part) {
            logOutput("[SYSTEM] FAT partition 'ffat' not found!", 0xF800);
            delay(800);
            return;
        }

        char partBuf[64];
        snprintf(partBuf, sizeof(partBuf), "[SYSTEM] 'ffat' partition: %u KB", (unsigned int)(part->size / 1024));
        logOutput(partBuf, 0x07E0);

        esp_vfs_fat_mount_config_t mount_config = {};
        mount_config.max_files = 4;
        mount_config.format_if_mount_failed = true; // Auto-format enabled to prevent raw fallback loop
        mount_config.allocation_unit_size = 4096;

        // Try raw flash mount (without wear levelling)
        esp_err_t err = esp_vfs_fat_rawflash_mount("/ffat", "ffat", &mount_config);
        
        if (err != ESP_OK) {
            // Direct partition sector scan fallback (handles sector sizes != 512 and custom FAT formats)
            scanAndFlashRawPartition(part);
            return;
        }

        // Standard POSIX file check if filesystem mounted
        const char* candidatePaths[] = {"/ffat/firmware.bin", "/ffat/FIRMWARE.BIN", "/ffat/firmware.bin.bin"};
        FILE* updateFile = nullptr;
        const char* targetPath = nullptr;

        for (const char* p : candidatePaths) {
            updateFile = fopen(p, "rb");
            if (updateFile) {
                targetPath = p;
                break;
            }
        }

        if (updateFile) {
            fseek(updateFile, 0, SEEK_END);
            size_t fileSize = ftell(updateFile);
            fseek(updateFile, 0, SEEK_SET);

            char foundLog[80];
            snprintf(foundLog, sizeof(foundLog), "[OTA] Found %s (%u B)", targetPath, (unsigned int)fileSize);
            logOutput(foundLog, 0x07FF);

            if (fileSize > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                esp_app_desc_t new_app_info;
                fseek(updateFile, sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), SEEK_SET);
                fread(&new_app_info, 1, sizeof(esp_app_desc_t), updateFile);

                const esp_partition_t* running = esp_ota_get_running_partition();
                esp_app_desc_t running_app_info;
                esp_ota_get_partition_description(running, &running_app_info);

                if (memcmp(new_app_info.app_elf_sha256, running_app_info.app_elf_sha256, sizeof(new_app_info.app_elf_sha256)) == 0) {
                    logOutput("[OTA] Image matches running firmware.");
                    fclose(updateFile);
                    esp_vfs_fat_rawflash_unmount("/ffat", "ffat");
                    deleteFirmwareFileRaw();
                    delay(800);
                    return;
                }

                logOutput("[OTA] Flashing firmware to partition...", 0xFFE0);

                fseek(updateFile, 0, SEEK_SET);

                if (Update.begin(fileSize, U_FLASH)) {
                    uint8_t* buf = (uint8_t*)malloc(4096);
                    size_t written = 0;
                    int lastPct = -1;

                    while (written < fileSize) {
                        size_t bytesToRead = min((size_t)4096, fileSize - written);
                        size_t len = fread(buf, 1, bytesToRead, updateFile);
                        if (len == 0) break;

                        Update.write(buf, len);
                        written += len;

                        int pct = (int)((written * 100) / fileSize);
                        if (pct != lastPct && pct % 10 == 0) {
                            lastPct = pct;
#if defined(TARGET_LILYGO)
                            displayManager.drawBootProgress(pct);
#endif
                        }
                    }
                    free(buf);

                    if (written == fileSize && Update.end(true)) {
                        logOutput("[OTA] Update complete! Rebooting...", 0x07E0);
                        fclose(updateFile);
                        esp_vfs_fat_rawflash_unmount("/ffat", "ffat");
                        deleteFirmwareFileRaw();
                        delay(1200);
                        ESP.restart();
                    }
                }
            }
            fclose(updateFile);
        } else {
            logOutput("[OTA] No firmware.bin found. Booting...");
            delay(300);
        }

        esp_vfs_fat_rawflash_unmount("/ffat", "ffat");
    }
};