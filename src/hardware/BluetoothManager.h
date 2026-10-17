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

struct BleMidiPacket {
    uint8_t header, data1, data2;
};

static inline QueueHandle_t bleMidiQueue = nullptr;

class BluetoothManager {
public:
    static inline volatile bool isMvaveConnected = false;
    static inline volatile bool isHostConnected = false;
    static inline volatile bool isNuxConnected = false;
    
    static inline uint32_t packetCount = 0;
    
    static inline char bleHostStatusStr[16] = "WAIT";
    static inline char blePedalStatusStr[16] = "WAIT";
    static inline char bleTargetStr[32] = "NONE";
    static inline char bleLastSeenStr[32] = "WAITING...";
    static inline char bleInitStep[64] = "0: UNINITIALIZED";

    // Shared cooldown timer to isolate the radio state machine during connections
    static inline volatile unsigned long lastConnectionEventTime = 0;

private:
    static inline NimBLEAddress targetAddress;
    static inline volatile bool hasTarget = false;
    static inline volatile bool targetIsPedal = false;
    static inline volatile bool clientIsPedal = false;
    
    static inline unsigned long reconnectLockoutUntil = 0;
    static inline unsigned long clientConnectTimestamp = 0;
    
    static inline uint16_t hostServerConnHandle = 0xFFFF;
    static inline uint16_t pedalServerConnHandle = 0xFFFF;
    
    static inline NimBLEClient* pClient = nullptr;
    static inline NimBLERemoteCharacteristic* pClientRemoteChar = nullptr;

    static inline NimBLEServer* pServer = nullptr;
    static inline NimBLECharacteristic* pServerChar = nullptr;

    static inline const NimBLEUUID midiServiceUUID{"03B80E5A-EDE8-4B33-A751-6CE34EC4C700"};
    static inline const NimBLEUUID midiCharUUID{"7772E5DB-3868-4112-A1A9-F2669D106BF3"};

    // Deferred Logging Flags to prevent Serial.print in ISR callbacks
    static inline volatile bool pendingPedalClientConnLog = false;
    static inline volatile bool pendingNuxClientConnLog = false;
    static inline volatile bool pendingPedalServerConnLog = false;
    static inline volatile bool pendingHostServerConnLog = false;
    static inline volatile int pendingClientDiscReason = -1;
    static inline volatile int pendingPedalServerDiscReason = -1;
    static inline volatile int pendingHostServerDiscReason = -1;

    static const char* getHciReasonString(int reason) {
        int code = reason & 0xFF;
        switch (code) {
            case 0x08: return "TIMEOUT (RF Lost/Lag)";
            case 0x13: return "REMOTE TERM (User Disc)";
            case 0x16: return "LOCAL TERM (ESP32 Disc)";
            case 0x3E: return "CONN ESTABLISH FAIL";
            case 0x0E: return "CONN REJECTED";
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

    static bool isPedalAddress(const std::string& addrStr) {
        if (addrStr.find("12:9d:6d") != std::string::npos ||
            addrStr.find("06:31:6d") != std::string::npos ||
            addrStr.find("2e:1f:b0") != std::string::npos ||
            addrStr.find("43:f2:f3") != std::string::npos) {
            return true;
        }
        return false;
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

        // Ignore discovery if cooling down from a connection event or already have a target
        if (hasTarget || (pClient && pClient->isConnected()) || millis() - lastConnectionEventTime < 2000) return;

        // Prevent targeting if at connection capacity
        int activeConnections = (pServer ? pServer->getConnectedCount() : 0) + (pClient && pClient->isConnected() ? 1 : 0);
        if (activeConnections >= 2) return;

        bool isPedal = false;
        bool isAmp = false;
        
        if (nameStr.length() > 0) {
            String upper = nameStr.c_str();
            upper.toUpperCase();
            if (upper.indexOf("CHOCOLATE") != -1 || upper.indexOf("M-VAVE") != -1 ||
                upper.indexOf("FOOTCTRL") != -1  || upper.indexOf("FOOT") != -1   ||
                upper.indexOf("PEDAL") != -1) {
                isPedal = true;
            } else if (upper.indexOf("NUX") != -1    || upper.indexOf("MIGHTY") != -1 ||
                       upper.indexOf("CUBE") != -1   || upper.indexOf("AMP") != -1) {
                isAmp = true;
            } else if (upper.indexOf("MIDI") != -1 || upper.indexOf("CTRL") != -1) {
                if (isPedalAddress(addrStr)) isPedal = true;
                else isAmp = true;
            }
        }
        
        if (!isPedal && !isAmp && isPedalAddress(addrStr)) {
            isPedal = true;
            if (nameStr.length() == 0) nameStr = "M-Vave (Hidden)";
        }
        
        if (!isPedal && !isAmp && advertisedDevice->haveServiceUUID()) {
            if (advertisedDevice->isAdvertisingService(midiServiceUUID)) {
                isAmp = true;
            }
        }

        // CRITICAL FIX: Never target a pedal if M-vave is already connected (e.g. as Central on Server)
        if (isPedal && isMvaveConnected) return;
        // Never target an amp if NUX is already connected
        if (isAmp && isNuxConnected) return;

        if (isPedal || isAmp) {
            hasTarget = true;
            targetIsPedal = isPedal;
            targetAddress = advertisedDevice->getAddress();
            strlcpy(bleTargetStr, nameStr.length() > 0 ? nameStr.c_str() : addrStr.c_str(), sizeof(bleTargetStr));
            snprintf(bleEventDebug, sizeof(bleEventDebug), "TARGET: %s", isPedal ? "PEDAL" : "AMP");
            clientConnectTimestamp = millis() + 500;
            
            if (NimBLEDevice::getScan()->isScanning()) {
                NimBLEDevice::getScan()->stop();
            }
        }
    }

public:
    class ClientCallbacks : public NimBLEClientCallbacks {
        void onConnect(NimBLEClient* client) override {
            std::string addrStr = client->getPeerAddress().toString();
            lastConnectionEventTime = millis();
            
            if (targetIsPedal || isPedalAddress(addrStr)) {
                clientIsPedal = true;
                isMvaveConnected = true;
                strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL LINKED!");
                snprintf(bleScanDebug, sizeof(bleScanDebug), "PEDAL: %s", bleTargetStr);
                pendingPedalClientConnLog = true;
            } else {
                clientIsPedal = false;
                isNuxConnected = true;
                strlcpy(bleHostStatusStr, "CONN (NUX)", sizeof(bleHostStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "NUX LINKED!");
                pendingNuxClientConnLog = true;
            }
        }

        void onDisconnect(NimBLEClient* client, int reason) override {
            pClientRemoteChar = nullptr;
            hasTarget = false;
            lastConnectionEventTime = millis();
            
            NimBLEDevice::whiteListRemove(client->getPeerAddress());
            reconnectLockoutUntil = millis() + 2000;
            
            // CRITICAL FIX: Only clear the specific device that disconnected
            if (clientIsPedal) {
                clientIsPedal = false;
                if (pedalServerConnHandle == 0xFFFF) {
                    isMvaveConnected = false;
                    strlcpy(blePedalStatusStr, "WAIT", sizeof(blePedalStatusStr));
                }
            } else {
                isNuxConnected = false;
                if (hostServerConnHandle == 0xFFFF) {
                    strlcpy(bleHostStatusStr, "WAIT", sizeof(bleHostStatusStr));
                }
            }
            
            int hciCode = reason & 0xFF;
            snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "CLI-DISC 0x%02X: %s", hciCode, getHciReasonString(hciCode));
            snprintf(bleEventDebug, sizeof(bleEventDebug), "CLIENT DISC (0x%02X)", hciCode);
            snprintf(bleScanDebug, sizeof(bleScanDebug), "SCANNING...");
            pendingClientDiscReason = hciCode;
        }
    };
    static inline ClientCallbacks clientCB;

    static void pedalNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
        if (length >= 5) {
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
            
            int activeConnections = (pServer ? pServer->getConnectedCount() : 0) + (pClient && pClient->isConnected() ? 1 : 0);
            if (activeConnections > 2) {
                server->disconnect(connHandle);
                return;
            }

            std::string addrStr = connInfo.getAddress().toString();
            lastConnectionEventTime = millis();
            
            if (isPedalAddress(addrStr) || (hasTarget && targetIsPedal && addrStr == targetAddress.toString())) {
                isMvaveConnected = true;
                hasTarget = false;
                pedalServerConnHandle = connHandle;
                strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL (SERVER)!");
                pendingPedalServerConnLog = true;
            } else {
                isHostConnected = true;
                hostServerConnHandle = connHandle;
                strlcpy(bleHostStatusStr, "CONN (PC)", sizeof(bleHostStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PC CONNECTED!");
                pendingHostServerConnLog = true;
            }
        }

        void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
            uint16_t connHandle = connInfo.getConnHandle();
            int hciCode = reason & 0xFF;
            lastConnectionEventTime = millis();

            if (connHandle == pedalServerConnHandle) {
                pedalServerConnHandle = 0xFFFF;
                if (!clientIsPedal) {
                    isMvaveConnected = false;
                    strlcpy(blePedalStatusStr, "WAIT", sizeof(blePedalStatusStr));
                }
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL DISC (0x%02X)", hciCode);
                pendingPedalServerDiscReason = hciCode;
            } else if (connHandle == hostServerConnHandle) {
                isHostConnected = false;
                hostServerConnHandle = 0xFFFF;
                if (!isNuxConnected) {
                    strlcpy(bleHostStatusStr, "WAIT", sizeof(bleHostStatusStr));
                }
                snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "HOST-DISC 0x%02X: %s", hciCode, getHciReasonString(hciCode));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "HOST DISC (0x%02X)", hciCode);
                pendingHostServerDiscReason = hciCode;
            }
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

                if (!isMvaveConnected && connInfo.getConnHandle() == hostServerConnHandle) {
                    if (header == 0xB0 || (header & 0xF0) == 0xC0) {
                        isMvaveConnected = true;
                        isHostConnected = false;
                        pedalServerConnHandle = hostServerConnHandle;
                        hostServerConnHandle = 0xFFFF;
                        strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
                        if (!isNuxConnected) {
                            strlcpy(bleHostStatusStr, "WAIT", sizeof(bleHostStatusStr));
                        }
                    }
                }

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
        dualLog("\n[BLE INIT] Initializing Dual-Role NimBLE Stack...\n");
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

        lastConnectionEventTime = millis();
        dualLog("[BLE SERVER] Advertising as 'Whammy_S3'.\n");

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
        uint8_t packet[5] = {0x80, 0x80, header, data1, data2};
        
        if (isHostConnected && pServerChar) {
            pServerChar->notify(packet, 5, false);
        }
        
        if (isNuxConnected && pClientRemoteChar) {
            pClientRemoteChar->writeValue(packet, 5, false);
        }
    }

    static void processDeferredLogs() {
        if (pendingPedalClientConnLog) {
            pendingPedalClientConnLog = false;
            dualLog("\n[BLE SUCCESS] Connected to Pedal (As Client)!\n");
        }
        if (pendingNuxClientConnLog) {
            pendingNuxClientConnLog = false;
            dualLog("\n[BLE SUCCESS] Connected to NUX Amp (As Client)!\n");
        }
        if (pendingPedalServerConnLog) {
            pendingPedalServerConnLog = false;
            dualLog("\n[BLE SERVER] Pedal connected as Central (Handle: %u)!\n", pedalServerConnHandle);
        }
        if (pendingHostServerConnLog) {
            pendingHostServerConnLog = false;
            dualLog("\n[BLE SERVER] Host PC/Mac connected (Handle: %u)!\n", hostServerConnHandle);
        }

        if (pendingClientDiscReason != -1) {
            dualLog("[BLE WARN] Client connection lost (0x%02X - %s).\n", pendingClientDiscReason, getHciReasonString(pendingClientDiscReason));
            pendingClientDiscReason = -1;
        }
        if (pendingPedalServerDiscReason != -1) {
            dualLog("[BLE SERVER] Pedal disconnected.\n");
            pendingPedalServerDiscReason = -1;
        }
        if (pendingHostServerDiscReason != -1) {
            dualLog("[BLE SERVER] Host PC disconnected (0x%02X - %s).\n", pendingHostServerDiscReason, getHciReasonString(pendingHostServerDiscReason));
            pendingHostServerDiscReason = -1;
        }
    }

    static void process() {
        processDeferredLogs();

        if (bleMidiQueue != nullptr) {
            BleMidiPacket pkt;
            while (xQueueReceive(bleMidiQueue, &pkt, 0)) {
                dispatchBleMidi(pkt.header, pkt.data1, pkt.data2);
                if (!(pkt.header == 0xB0 && pkt.data1 == 11)) {
                    sendMidiToHost(pkt.header, pkt.data1, pkt.data2);
                }
            }
        }

        int activeConnections = (pServer ? pServer->getConnectedCount() : 0) + (pClient && pClient->isConnected() ? 1 : 0);
        if (activeConnections >= 2) {
            if (NimBLEDevice::getScan()->isScanning()) NimBLEDevice::getScan()->stop();
            if (pServer && NimBLEDevice::getAdvertising()->isAdvertising()) NimBLEDevice::getAdvertising()->stop();
            return;
        }

        if (millis() - lastConnectionEventTime < 2000) return;

        if (((isMvaveConnected && isHostConnected) || (isMvaveConnected && isNuxConnected)) && 
             NimBLEDevice::getScan()->isScanning()) {
            NimBLEDevice::getScan()->stop();
        }

        static unsigned long lastAdvCall = 0;
        if (pServer && pServer->getConnectedCount() < 2) {
            if (!NimBLEDevice::getAdvertising()->isAdvertising() && (millis() - lastAdvCall > 2000)) {
                NimBLEDevice::getAdvertising()->start();
                lastAdvCall = millis();
            }
        } else {
            lastAdvCall = millis(); 
        }

        static unsigned long lastScanCall = 0;
        bool clientConnected = (pClient && pClient->isConnected());
        
        // Scan if client is not connected and we still need a peripheral (pedal or Nux)
        if (!hasTarget && !clientConnected && millis() >= reconnectLockoutUntil && (!isMvaveConnected || !isNuxConnected)) {
            if (!NimBLEDevice::getScan()->isScanning() && (millis() - lastScanCall > 2000)) {
                NimBLEDevice::getScan()->start(0, false, true);
                lastScanCall = millis();
            }
        } else {
            lastScanCall = millis(); 
        }

        if (hasTarget && !clientConnected && millis() >= clientConnectTimestamp) {
            if (NimBLEDevice::getScan()->isScanning()) {
                NimBLEDevice::getScan()->stop();
            }
            vTaskDelay(pdMS_TO_TICKS(150));

            snprintf(bleEventDebug, sizeof(bleEventDebug), "CONNECTING...");
            dualLog("[BLE PROCESS] Connecting to peripheral [%s]...\n", targetAddress.toString().c_str());

            if (!pClient) {
                pClient = NimBLEDevice::createClient();
                pClient->setClientCallbacks(&clientCB, false);
                pClient->setConnectionParams(12, 40, 0, 500);
                pClient->setConnectTimeout(5000);
            }

            hasTarget = false;

            if (pClient->connect(targetAddress)) {
                vTaskDelay(pdMS_TO_TICKS(150));
                
                NimBLERemoteService* pSvc = pClient->getService(midiServiceUUID);
                if (pSvc) {
                    NimBLERemoteCharacteristic* pChar = pSvc->getCharacteristic(midiCharUUID);
                    if (pChar) {
                        pClientRemoteChar = pChar;
                        if (pChar->canNotify()) {
                            pChar->subscribe(true, pedalNotifyCallback);
                        }
                        snprintf(bleEventDebug, sizeof(bleEventDebug), "LINKED!");
                        dualLog("[BLE PROCESS] >>> PERIPHERAL MIDI READY! <<<\n");
                    } else {
                        snprintf(bleEventDebug, sizeof(bleEventDebug), "NO CHAR");
                        pClient->disconnect();
                    }
                } else {
                    snprintf(bleEventDebug, sizeof(bleEventDebug), "NO SVC");
                    pClient->disconnect();
                }
            } else {
                int err = pClient->getLastError();
                int hciCode = err & 0xFF;
                snprintf(bleEventDebug, sizeof(bleEventDebug), "FAIL (0x%02X)", hciCode);
                dualLog("[BLE ERROR] Connection failed (rc: 0x%02X - %s).\n", hciCode, getHciReasonString(hciCode));
                lastConnectionEventTime = millis();
            }
        }
    }
};

#endif // BLUETOOTH_MANAGER_H