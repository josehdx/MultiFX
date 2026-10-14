#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>
#include <esp_nimble_hci.h> 
#include <esp_bt.h>
#include <esp_mac.h>
#include <NimBLEDevice.h>

inline char bleEventDebug[64] = "BLE: WAITING...";

class BluetoothManager {
public:
    static void initHCI() {
        Serial.println("[BLE] Pre-initializing BT Controller & HCI...");
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t ret = esp_bt_controller_init(&bt_cfg);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            Serial.printf("[BLE] BT Controller Init Failed! Error: %d\n", ret);
            return;
        }
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            Serial.printf("[BLE] BT Controller Enable Failed! Error: %d\n", ret);
            return;
        }
        esp_err_t err_hci = esp_nimble_hci_init();
        if (err_hci != ESP_OK && err_hci != ESP_ERR_INVALID_STATE) {
            Serial.printf("[BLE] HCI Init Failed! Error: %d\n", err_hci);
        } else {
            Serial.println("[BLE] HCI Controller Successfully Initialized.");
        }
    }

    static void configurePowerAndMac() {
        Serial.println("[BLE] Setting TX Power to Maximum (+9dBm)...");
        esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
        esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        Serial.printf("[BLE] NimBLE baseband up! Hardware MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    static void tightenBLETimeout() {
        if (NimBLEDevice::getServer() != nullptr) {
            auto peers = NimBLEDevice::getServer()->getPeerDevices();
            for(auto peer : peers) {
                // Force connection to drop if no packets are received for 1.5 seconds
                NimBLEDevice::getServer()->updateConnParams(
                    peer,
                    24,     // Min interval: 30ms (24 * 1.25ms)
                    40,     // Max interval: 50ms (40 * 1.25ms)
                    0,      // Latency: 0
                    150     // Supervision Timeout: 1.5s (150 * 10ms)
                );
            }
        }
    }
    
    static void forceAdvertising() {
        if (NimBLEDevice::getServer() != nullptr) {
            NimBLEDevice::getServer()->startAdvertising();
        }
    }
};

#endif