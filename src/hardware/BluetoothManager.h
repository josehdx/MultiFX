#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

inline char bleEventDebug[64] = "BLE: IDLE";
inline char bleScanDebug[64]  = "SCAN: LISTENING...";
inline char bleLatchedDiscReason[64] = "DISC: NONE";

extern void dispatchBleMidi(uint8_t header, uint8_t data1, uint8_t data2);

struct BleMidiPacket { uint8_t header, data1, data2; };
static inline QueueHandle_t bleMidiQueue = nullptr;

class BluetoothManager {
public:
    static inline volatile bool isMvaveConnected = false;
    static inline volatile bool isHostConnected = false;
    static inline uint32_t packetCount = 0;
    
    static inline char bleHostStatusStr[16] = "WAIT";
    static inline char blePedalStatusStr[16] = "WAIT";
    static inline char bleTargetStr[32] = "NONE";
    static inline char bleLastSeenStr[32] = "WAITING...";
    static inline char bleInitStep[64] = "0: UNINITIALIZED";

private:
    static inline NimBLEAddress targetAddress;
    static inline volatile bool hasTarget = false;
    static inline volatile bool doConnect = false;
    static inline unsigned long reconnectLockoutUntil = 0;
    static inline unsigned long clientConnectTimestamp = 0;
    
    static inline uint16_t hostConnHandle = 0xFFFF;
    static inline uint16_t pedalServerConnHandle = 0xFFFF;
    
    static inline NimBLEClient* pClient = nullptr;
    static inline NimBLEServer* pServer = nullptr;
    static inline NimBLECharacteristic* pServerChar = nullptr;

    static inline const NimBLEUUID midiServiceUUID{"03B80E5A-EDE8-4B33-A751-6CE34EC4C700"};
    static inline const NimBLEUUID midiCharUUID{"7772E5DB-3868-4112-A1A9-F2669D106BF3"};

    static const char* getHciReasonString(int reason) {
        int code = reason & 0xFF;
        switch (code) {
            case 0x08: return "TIMEOUT (RF Lost/Lag)";
            case 0x13: return "REMOTE TERM (User Disc)";
            case 0x16: return "LOCAL TERM (ESP32 Disc)";
            case 0x3E: return "CONN ESTABLISH FAIL";
            case 0x0D: return "SLOTS FULL (rc:525)";
            case 0x28: return "INSTANT PASSED (Lag)";
            case 0x15: return "POWER OFF";
            case 0x3B: return "BAD CONN PARAMS";
            default:   return "HCI CODE";
        }
    }

    static void dualLog(const char* fmt, ...) {
        char buf[256];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        
        Serial0.print(buf);
    }

    static void handleDiscoveredDevice(const NimBLEAdvertisedDevice* advertisedDevice) {
        if (!advertisedDevice) return;
        packetCount++;
        
        std::string nameStr = advertisedDevice->getName();
        std::string addrStr = advertisedDevice->getAddress().toString();
        int rssi = advertisedDevice->getRSSI();

        if (nameStr.length() > 0) {
            snprintf(bleLastSeenStr, sizeof(bleLastSeenStr), "%s (%ddBm)", nameStr.c_str(), rssi);
            snprintf(bleScanDebug, sizeof(bleScanDebug), "#%lu %s", (unsigned long)packetCount, nameStr.c_str());
        } else {
            snprintf(bleLastSeenStr, sizeof(bleLastSeenStr), "[%s] (%ddBm)", addrStr.c_str(), rssi);
            snprintf(bleScanDebug, sizeof(bleScanDebug), "#%lu %s", (unsigned long)packetCount, addrStr.c_str());
        }

        if (hasTarget || isMvaveConnected || millis() < reconnectLockoutUntil) return;

        bool isMatch = false;
        if (nameStr.length() > 0) {
            String upper = nameStr.c_str();
            upper.toUpperCase();
            if (upper.indexOf("FOOTCTRL") != -1  || upper.indexOf("CHOCOLATE") != -1 ||
                upper.indexOf("M-VAVE") != -1    || upper.indexOf("FOOT") != -1      ||
                upper.indexOf("CTRL") != -1      || upper.indexOf("CUBE") != -1      ||
                upper.indexOf("MIDI") != -1) {
                isMatch = true;
            }
        }

        if (!isMatch) {
            String mac = addrStr.c_str();
            mac.toLowerCase();
            if (mac.indexOf("12:9d:6d") != -1 || 
                mac.indexOf("06:31:6d") != -1 || 
                mac.indexOf("2e:1f:b0") != -1 || 
                mac.indexOf("43:f2:f3") != -1) {
                isMatch = true;
                if (nameStr.length() == 0) nameStr = "M-Vave (Hidden)";
            }
        }

        if (!isMatch && advertisedDevice->haveServiceUUID()) {
            if (advertisedDevice->isAdvertisingService(midiServiceUUID)) {
                isMatch = true;
            }
        }

        if (isMatch) {
            hasTarget = true;
            targetAddress = advertisedDevice->getAddress();
            strlcpy(bleTargetStr, nameStr.length() > 0 ? nameStr.c_str() : addrStr.c_str(), sizeof(bleTargetStr));
            
            snprintf(bleEventDebug, sizeof(bleEventDebug), "TARGET AQUIRED");
            dualLog("[BLE ACTION] Target pedal [%s] found at %s. Yielding 2.5s for Central connection...\n", bleTargetStr, addrStr.c_str());
            
            clientConnectTimestamp = millis() + 2500;
        }
    }

public:
    class ClientCallbacks : public NimBLEClientCallbacks {
        void onConnect(NimBLEClient* client) override {
            isMvaveConnected = true;
            strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
            snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL LINKED!");
            snprintf(bleScanDebug, sizeof(bleScanDebug), "PEDAL: %s", bleTargetStr);
            dualLog("\n[BLE SUCCESS] Connected to M-Vave Pedal (As Client)!\n");
        }

        void onDisconnect(NimBLEClient* client, int reason) override {
            isMvaveConnected = false;
            hasTarget = false;
            reconnectLockoutUntil = millis() + 3000;
            strlcpy(blePedalStatusStr, "WAIT", sizeof(blePedalStatusStr));
            
            int hciCode = reason & 0xFF;
            snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "P-DISC 0x%02X: %s", hciCode, getHciReasonString(hciCode));
            snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL DISC (0x%02X)", hciCode);
            snprintf(bleScanDebug, sizeof(bleScanDebug), "WAITING (3.0s)...");
            dualLog("[BLE WARN] M-Vave Pedal disconnected (0x%02X - %s).\n", hciCode, getHciReasonString(hciCode));
        }
    };
    static inline ClientCallbacks clientCB;

    static void pedalNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
        if (length >= 5) {
            dualLog("[PEDAL -> MIDI] 0x%02X %03d %03d\n", pData[2], pData[3], pData[4]);
            BleMidiPacket pkt = {pData[2], pData[3], pData[4]};
            if (bleMidiQueue) xQueueSend(bleMidiQueue, &pkt, 0);
        }
    }

    class ScanCallbacks : public NimBLEScanCallbacks {
        void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
            handleDiscoveredDevice(advertisedDevice);
        }
    };
    static inline ScanCallbacks scanCB;

    class ServerCallbacks : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
            uint16_t connHandle = connInfo.getConnHandle();
            std::string addrStr = connInfo.getAddress().toString();
            
            if (NimBLEDevice::getScan()->isScanning()) {
                NimBLEDevice::getScan()->stop();
            }
            
            bool isPedal = false;
            if (hasTarget && addrStr == targetAddress.toString()) isPedal = true;
            if (addrStr.find("12:9d:6d") != std::string::npos) isPedal = true;
            if (addrStr.find("06:31:6d") != std::string::npos) isPedal = true;
            if (addrStr.find("2e:1f:b0") != std::string::npos) isPedal = true;
            if (addrStr.find("43:f2:f3") != std::string::npos) isPedal = true;

            if (isPedal) {
                isMvaveConnected = true;
                hasTarget = false; 
                pedalServerConnHandle = connHandle;
                strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL (SERVER)!");
                dualLog("\n[BLE SERVER] M-Vave Pedal connected as Central (Handle: %u)!\n", connHandle);
            } else {
                isHostConnected = true;
                hostConnHandle = connHandle;
                strlcpy(bleHostStatusStr, "CONN", sizeof(bleHostStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "MAC: CONNECTED!");
                dualLog("\n[BLE SERVER] Host (Mac/iOS/PC) connected (Handle: %u)!\n", connHandle);
            }

            if (server->getConnectedCount() < 2) {
                NimBLEDevice::getAdvertising()->start();
            }
        }

        void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
            uint16_t connHandle = connInfo.getConnHandle();
            int hciCode = reason & 0xFF;

            if (connHandle == pedalServerConnHandle) {
                isMvaveConnected = false;
                pedalServerConnHandle = 0xFFFF;
                strlcpy(blePedalStatusStr, "WAIT", sizeof(blePedalStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL DISC (0x%02X)", hciCode);
                dualLog("[BLE SERVER] M-Vave Pedal (Server role) disconnected.\n");
            } else if (connHandle == hostConnHandle) {
                isHostConnected = false;
                hostConnHandle = 0xFFFF;
                strlcpy(bleHostStatusStr, "WAIT", sizeof(bleHostStatusStr));
                snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "M-DISC 0x%02X: %s", hciCode, getHciReasonString(hciCode));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "MAC DISC (0x%02X)", hciCode);
                dualLog("[BLE SERVER] Host disconnected (0x%02X - %s).\n", hciCode, getHciReasonString(hciCode));
            }
            
            NimBLEDevice::getAdvertising()->start();
        }
    };
    static inline ServerCallbacks serverCB;

    class ServerCharCallbacks : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
            NimBLEAttValue val = pChar->getValue();
            if (val.length() >= 5) {
                const uint8_t* pData = val.data();
                uint8_t header = pData[2];
                uint8_t data1  = pData[3];
                uint8_t data2  = pData[4];

                if (!isMvaveConnected && connInfo.getConnHandle() == hostConnHandle) {
                    if (header == 0xB0 || (header & 0xF0) == 0xC0) {
                        isMvaveConnected = true;
                        isHostConnected = false;
                        hasTarget = false;
                        pedalServerConnHandle = hostConnHandle;
                        hostConnHandle = 0xFFFF;
                        strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
                        strlcpy(bleHostStatusStr, "WAIT", sizeof(bleHostStatusStr));
                        dualLog("\n[BLE ARBITER] Reassigned Handle %u from Host to M-Vave Pedal based on MIDI traffic.\n", connInfo.getConnHandle());
                    }
                }

                dualLog("[MIDI RX] 0x%02X %03d %03d\n", header, data1, data2);
                BleMidiPacket pkt = {header, data1, data2};
                if (bleMidiQueue) xQueueSend(bleMidiQueue, &pkt, 0);
            }
        }
    };
    static inline ServerCharCallbacks serverCharCB;

    static void initDualRole() {
        if (bleMidiQueue == nullptr) {
            bleMidiQueue = xQueueCreate(64, sizeof(BleMidiPacket));
        }

        strlcpy(bleInitStep, "1: NimBLE init()", sizeof(bleInitStep));
        dualLog("\n[BLE INIT] Initializing Dual-Role NimBLE Stack on Core 1...\n");
        NimBLEDevice::init("Whammy_S3");
        
        NimBLEDevice::setPower(ESP_PWR_LVL_P9);

        strlcpy(bleInitStep, "2: createServer()", sizeof(bleInitStep));
        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(&serverCB);

        NimBLEService* pService = pServer->createService(midiServiceUUID);
        pServerChar = pService->createCharacteristic(
            midiCharUUID,
            NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY |
            NIMBLE_PROPERTY::WRITE_NR
        );
        pServerChar->setCallbacks(&serverCharCB);

        strlcpy(bleInitStep, "3: getAdvertising()", sizeof(bleInitStep));
        NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
        NimBLEAdvertisementData advData;
        advData.setFlags(0x06);
        advData.addServiceUUID(midiServiceUUID);
        pAdvertising->setAdvertisementData(advData);

        NimBLEAdvertisementData scanRespData;
        scanRespData.setName("Whammy_S3");
        pAdvertising->setScanResponseData(scanRespData);
        pAdvertising->enableScanResponse(true);
        pAdvertising->setMinInterval(400); 
        pAdvertising->setMaxInterval(800); 
        pAdvertising->start();
        dualLog("[BLE SERVER] Advertising as 'Whammy_S3' for PC/Mac.\n");

        strlcpy(bleInitStep, "4: getScan()", sizeof(bleInitStep));
        NimBLEScan* pScan = NimBLEDevice::getScan();
        pScan->setScanCallbacks(&scanCB, false);
        pScan->setActiveScan(false);
        pScan->setMaxResults(0);          
        pScan->setInterval(100); 
        pScan->setWindow(40);             

        snprintf(bleEventDebug, sizeof(bleEventDebug), "DUAL: ADV & SCAN");
        snprintf(bleScanDebug, sizeof(bleScanDebug), "SCAN: LISTENING...");
        
        strlcpy(bleInitStep, "5: pScan->start()", sizeof(bleInitStep));
        if (pScan->start(0, false, true)) {
            strlcpy(bleInitStep, "6: SUCCESS (Scanning)", sizeof(bleInitStep));
        } else {
            strlcpy(bleInitStep, "ERR: SCAN FAILED", sizeof(bleInitStep));
            dualLog("[BLE ERR] Scanner failed to start.\n");
        }
    }

    static void sendMidiToHost(uint8_t header, uint8_t data1, uint8_t data2) {
        if (isHostConnected && pServerChar) {
            uint8_t packet[5] = {0x80, 0x80, header, data1, data2};
            pServerChar->notify(packet, 5, false);
        }
    }

    static void process() {
        if (bleMidiQueue != nullptr) {
            BleMidiPacket pkt;
            while (xQueueReceive(bleMidiQueue, &pkt, 0)) {
                dispatchBleMidi(pkt.header, pkt.data1, pkt.data2);
                
                // Do not blindly echo CC 11 here: dispatchBleMidi routes it to CC 11 or CC 19
                if (!(pkt.header == 0xB0 && pkt.data1 == 11)) {
                    sendMidiToHost(pkt.header, pkt.data1, pkt.data2);
                }
            }
        }

        if (pServer && pServer->getConnectedCount() < 2 && !NimBLEDevice::getAdvertising()->isAdvertising()) {
            NimBLEDevice::getAdvertising()->start();
        }

        if (!hasTarget && !isMvaveConnected && 
            millis() >= reconnectLockoutUntil && !NimBLEDevice::getScan()->isScanning()) {
            NimBLEDevice::getScan()->start(0, false, true);
        }

        if (hasTarget && !isMvaveConnected && millis() >= clientConnectTimestamp) {
            
            if (NimBLEDevice::getScan()->isScanning()) {
                NimBLEDevice::getScan()->stop();
            }
            vTaskDelay(pdMS_TO_TICKS(150));
            if (isMvaveConnected) {
                hasTarget = false;
                return;
            }
            
            snprintf(bleEventDebug, sizeof(bleEventDebug), "CONNECTING...");
            dualLog("[BLE PROCESS] Connecting to pedal [%s]...\n", targetAddress.toString().c_str());

            if (!pClient) {
                pClient = NimBLEDevice::createClient();
                pClient->setClientCallbacks(&clientCB, false); 
                pClient->setConnectionParams(12, 40, 0, 500); 
                pClient->setConnectTimeout(5000);
            }

            if (pClient->connect(targetAddress)) {
                vTaskDelay(pdMS_TO_TICKS(150));
                
                NimBLERemoteService* pSvc = pClient->getService(midiServiceUUID);
                if (pSvc) {
                    NimBLERemoteCharacteristic* pChar = pSvc->getCharacteristic(midiCharUUID);
                    if (pChar && pChar->canNotify()) {
                        pChar->subscribe(true, pedalNotifyCallback);
                        snprintf(bleEventDebug, sizeof(bleEventDebug), "DUAL LINKED!");
                        dualLog("[BLE PROCESS] >>> PEDAL NOTIFICATIONS SUBSCRIBED! <<<\n");
                    } else {
                        snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL: NO CHAR");
                        snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "GATT: NO CHAR 7772");
                        pClient->disconnect();
                    }
                } else {
                    snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL: NO SVC");
                    snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "GATT: NO SVC 03B8");
                    pClient->disconnect();
                }
            } else {
                int err = pClient->getLastError();
                int hciCode = err & 0xFF;
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL FAIL (0x%02X)", hciCode);
                snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "FAIL 0x%02X: %s", hciCode, getHciReasonString(hciCode));
                dualLog("[BLE ERROR] Connection failed (rc: 0x%02X - %s).\n", hciCode, getHciReasonString(hciCode));
                
                hasTarget = false;
                reconnectLockoutUntil = millis() + 1500;
            }
        }
    }
};

#endif