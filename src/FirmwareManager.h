#pragma once
#include <Arduino.h>
#include <Update.h>
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_vfs_fat.h"
#include "esp_app_format.h"

class FirmwareManager {
private:
    // Raw sector-level FAT directory entry deleter
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
                if (firstByte == 0x00) break; // End of directory
                if (firstByte == 0xE5) continue; // Already deleted

                // Match SFN "FIRMWARE" / "FIRMWA~1"
                bool isTargetSFN = (memcmp(&sectorBuf[i], "FIRMWARE", 8) == 0) ||
                                   (memcmp(&sectorBuf[i], "FIRMWA~1", 8) == 0);

                // Match LFN "firmware"
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
                    sectorBuf[i] = 0xE5; // FAT deleted marker
                    sectorModified = true;
                    fileDeleted = true;
                }
            }

            if (sectorModified) {
                // Erase 4KB SPI flash sector before writing
                esp_partition_erase_range(part, offset, 4096);
                esp_partition_write(part, offset, sectorBuf, 4096);
            }
        }

        if (fileDeleted) {
            Serial.println("[OTA] firmware.bin permanently erased from RAW FAT partition.");
            Serial0.println("[OTA] firmware.bin permanently erased from RAW FAT partition.");
        }
    }

public:
    static void checkAndApplyUpdate() {
        Serial.println("[SYSTEM] Mounting FAT file system...");
        Serial0.println("[SYSTEM] Mounting FAT file system...");
        
        esp_vfs_fat_mount_config_t mount_config = {};
        mount_config.max_files = 4;
        mount_config.format_if_mount_failed = false;
        mount_config.allocation_unit_size = 4096;
        
        esp_err_t err = esp_vfs_fat_spiflash_mount_ro("/ffat", "ffat", &mount_config);
        if (err != ESP_OK) {
            Serial.printf("[SYSTEM] Mount Failed: %s\n", esp_err_to_name(err));
            Serial0.printf("[SYSTEM] Mount Failed: %s\n", esp_err_to_name(err));
            return;
        }

        const char* targetPath = "/ffat/firmware.bin";
        FILE* updateFile = fopen(targetPath, "rb");
        if (!updateFile) {
            targetPath = "/ffat/firmware.bin.bin";
            updateFile = fopen(targetPath, "rb");
        }

        if (updateFile) {
            fseek(updateFile, 0, SEEK_END);
            size_t fileSize = ftell(updateFile);
            fseek(updateFile, 0, SEEK_SET);

            Serial.printf("[OTA] Found %s (%d bytes).\n", targetPath, fileSize);
            Serial0.printf("[OTA] Found %s (%d bytes).\n", targetPath, fileSize);

            if (fileSize > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                uint8_t magic = 0;
                fread(&magic, 1, 1, updateFile);
                
                if (magic != ESP_IMAGE_HEADER_MAGIC) {
                    Serial.printf("[OTA] ERROR: Invalid Magic Byte (0x%02X).\n", magic);
                    Serial0.printf("[OTA] ERROR: Invalid Magic Byte (0x%02X).\n", magic);
                    fclose(updateFile);
                } else {
                    esp_app_desc_t new_app_info;
                    fseek(updateFile, sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), SEEK_SET);
                    fread(&new_app_info, 1, sizeof(esp_app_desc_t), updateFile);

                    const esp_partition_t* running = esp_ota_get_running_partition();
                    esp_app_desc_t running_app_info;
                    esp_ota_get_partition_description(running, &running_app_info);
                    
                    if (memcmp(new_app_info.app_elf_sha256, running_app_info.app_elf_sha256, sizeof(new_app_info.app_elf_sha256)) == 0) {
                        Serial.println("[OTA] Binary matches running image. Cleaning up storage...");
                        Serial0.println("[OTA] Binary matches running image. Cleaning up storage...");
                        fclose(updateFile);
                        esp_vfs_fat_spiflash_unmount_ro("/ffat", "ffat");
                        
                        deleteFirmwareFileRaw();
                        return;
                    } else {
                        Serial.println("[OTA] Flashing new firmware to partition...");
                        Serial0.println("[OTA] Flashing new firmware to partition...");
                        fseek(updateFile, 0, SEEK_SET);
                        
                        if (Update.begin(fileSize, U_FLASH)) {
                            uint8_t* buf = (uint8_t*)malloc(4096);
                            size_t written = 0;
                            bool writeError = false;

                            while (written < fileSize) {
                                size_t bytesToRead = min((size_t)4096, fileSize - written);
                                size_t len = fread(buf, 1, bytesToRead, updateFile);
                                if (len == 0) break;

                                size_t w = Update.write(buf, len);
                                if (w != len) {
                                    writeError = true;
                                    break;
                                }
                                written += len;
                            }
                            free(buf);

                            if (!writeError && written == fileSize && Update.end(true)) {
                                Serial.println("[OTA] Update complete! Erasing file and rebooting...");
                                Serial0.println("[OTA] Update complete! Erasing file and rebooting...");
                                fclose(updateFile);
                                esp_vfs_fat_spiflash_unmount_ro("/ffat", "ffat");
                                
                                deleteFirmwareFileRaw();
                                
                                delay(1000);
                                ESP.restart();
                            } else {
                                fclose(updateFile);
                            }
                        } else {
                            fclose(updateFile);
                        }
                    }
                }
            } else {
                fclose(updateFile);
            }
        } else {
            Serial.println("[OTA] No firmware.bin found. Booting normally.");
            Serial0.println("[OTA] No firmware.bin found. Booting normally.");
        }
        
        esp_vfs_fat_spiflash_unmount_ro("/ffat", "ffat");
    }
};