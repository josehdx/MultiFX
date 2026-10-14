#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>
#include <NimBLEDevice.h>

inline char bleEventDebug[64] = "BLE: IDLE";
inline char bleScanDebug[64]  = "SCAN: LISTENING...";
inline char bleLatchedDiscReason[64] = "DISC: NONE";

extern void dispatchBleMidi(uint8_t header, uint8_t data1, uint8_t data2);

class BluetoothManager {
public:
    static inline bool isMvaveConnected = false;
    static inline bool isHostConnected = false;
    static inline uint32_t packetCount = 0;

    static inline char bleHostStatusStr[16] = "WAIT";
    static inline char blePedalStatusStr[16] = "WAIT";
    static inline char bleTargetStr[32] = "NONE";
    static inline char bleLastSeenStr[32] = "WAITING...";
    static inline char bleInitStep[64] = "0: UNINITIALIZED";

private:
    static inline NimBLEAddress targetAddress;
    static inline bool hasTarget = false;
    static inline bool doConnect = false;
    static inline unsigned long reconnectLockoutUntil = 0;
    static inline unsigned long ghostConnectionWatchdog = 0;

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
        packetCount++;
        std::string nameStr = advertisedDevice->getName();
        std::string addrStr = advertisedDevice->getAddress().toString();
        int rssi = advertisedDevice->getRSSI();

        bool hasMidiUUID = advertisedDevice->isAdvertisingService(midiServiceUUID);

        if (nameStr.length() > 0) {
            snprintf(bleLastSeenStr, sizeof(bleLastSeenStr), "%s (%ddBm)", nameStr.c_str(), rssi);
            snprintf(bleScanDebug, sizeof(bleScanDebug), "#%lu %s", (unsigned long)packetCount, nameStr.c_str());
        } else {
            snprintf(bleLastSeenStr, sizeof(bleLastSeenStr), "[%s] (%ddBm)", addrStr.c_str(), rssi);
            snprintf(bleScanDebug, sizeof(bleScanDebug), "#%lu %s", (unsigned long)packetCount, addrStr.c_str());
        }

        if (doConnect || hasTarget || isMvaveConnected || millis() < reconnectLockoutUntil) return;

        bool isMatch = false;
        if (hasMidiUUID) {
            isMatch = true;
        } else if (nameStr.length() > 0) {
            String upper = nameStr.c_str();
            upper.toUpperCase();
            if (upper.indexOf("FOOTCTRL") != -1  || upper.indexOf("CHOCOLATE") != -1 ||
                upper.indexOf("M-VAVE") != -1    || upper.indexOf("FOOT") != -1      ||
                upper.indexOf("CTRL") != -1      || upper.indexOf("CUBE") != -1      ||
                upper.indexOf("MIDI") != -1) {
                isMatch = true;
            }
        }

        if (isMatch) {
            doConnect = true;
            hasTarget = true;
            targetAddress = advertisedDevice->getAddress();
            strlcpy(bleTargetStr, nameStr.length() > 0 ? nameStr.c_str() : addrStr.c_str(), sizeof(bleTargetStr));
            
            snprintf(bleEventDebug, sizeof(bleEventDebug), "BLE: PEDAL FOUND!");
            dualLog("[BLE ACTION] Target %s found. Halting scanner...\n", bleTargetStr);
            NimBLEDevice::getScan()->stop();
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
            doConnect = false;
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
            dispatchBleMidi(pData[2], pData[3], pData[4]);
            sendMidiToHost(pData[2], pData[3], pData[4]);
        }
    }

    class ScanCallbacks : public NimBLEScanCallbacks {
        void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override {
            handleDiscoveredDevice(advertisedDevice);
        }
        void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
            handleDiscoveredDevice(advertisedDevice);
        }
        void onScanEnd(const NimBLEScanResults& results, int reason) override {
            if (!doConnect && !hasTarget && !isMvaveConnected && millis() >= reconnectLockoutUntil) {
                // Restart scanner with 40% duty cycle to prevent radio starvation
                NimBLEDevice::getScan()->start(5000, false, true);
            }
        }
    };
    static inline ScanCallbacks scanCB;

    class ServerCallbacks : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
            int count = server->getConnectedCount();
            
            if (count == 1) {
                isHostConnected = true;
                strlcpy(bleHostStatusStr, "CONN", sizeof(bleHostStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "MAC: CONNECTED!");
                dualLog("\n[BLE SERVER] Host 1 (Mac/iOS) connected!\n");
            } else if (count >= 2) {
                // The M-Vave connected to our Server acting as a Central
                isMvaveConnected = true;
                strlcpy(blePedalStatusStr, "CONN", sizeof(blePedalStatusStr));
                snprintf(bleEventDebug, sizeof(bleEventDebug), "PEDAL (REV): CONN!");
                dualLog("\n[BLE SERVER] Host 2 (M-Vave Central) connected!\n");
            }
            
            // Allow Mac/Pedal to negotiate connection interval before fetching data
            server->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
        }

        void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
            int count = server->getConnectedCount();
            
            if (count == 0) {
                isHostConnected = false;
                isMvaveConnected = false;
                strlcpy(bleHostStatusStr, "WAIT", sizeof(bleHostStatusStr));
                strlcpy(blePedalStatusStr, "WAIT", sizeof(blePedalStatusStr));
            } else if (count == 1) {
                // One device dropped, one is still connected. 
                // We drop the second "Pedal" flag but keep the Host flag active.
                isMvaveConnected = false;
                strlcpy(blePedalStatusStr, "WAIT", sizeof(blePedalStatusStr));
            }
            
            int hciCode = reason & 0xFF;
            snprintf(bleLatchedDiscReason, sizeof(bleLatchedDiscReason), "M-DISC 0x%02X: %s", hciCode, getHciReasonString(hciCode));
            snprintf(bleEventDebug, sizeof(bleEventDebug), "MAC DISC (0x%02X)", hciCode);
            dualLog("[BLE SERVER] Device disconnected (0x%02X - %s).\n", hciCode, getHciReasonString(hciCode));
            
            // Explicitly restart advertising immediately
            NimBLEDevice::getAdvertising()->start();
        }
    };
    static inline ServerCallbacks serverCB;

    class ServerCharCallbacks : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
            // If Mac sends data, refresh the ghost connection watchdog
            ghostConnectionWatchdog = millis();
            
            NimBLEAttValue val = pChar->getValue();
            if (val.length() >= 5) {
                const uint8_t* pData = val.data();
                dualLog("[MAC/PEDAL -> MIDI] 0x%02X %03d %03d\n", pData[2], pData[3], pData[4]);
                dispatchBleMidi(pData[2], pData[3], pData[4]);
            }
        }
    };
    static inline ServerCharCallbacks serverCharCB;

    static void initDualRole() {
        strlcpy(bleInitStep, "1: NimBLE init()", sizeof(bleInitStep));
        dualLog("\n[BLE INIT] Initializing Dual-Role NimBLE Stack on Core 1...\n");
        NimBLEDevice::init("Whammy_S3");
        
        // 0 dBm to prevent battery rail voltage drops
        NimBLEDevice::setPower(ESP_PWR_LVL_N0);

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

        dualLog("[BLE SERVER] Advertising as 'Whammy_S3' for macOS.\n");

        strlcpy(bleInitStep, "4: getScan()", sizeof(bleInitStep));
        NimBLEScan* pScan = NimBLEDevice::getScan();
        pScan->setScanCallbacks(&scanCB, true);
        pScan->setActiveScan(true);
        
        // 40% Duty Cycle. Gives the ESP32 radio 60ms out of every 100ms 
        // to maintain the Mac's connection and broadcast the Server Advertisement.
        pScan->setInterval(100); 
        pScan->setWindow(40);   
        
        snprintf(bleEventDebug, sizeof(bleEventDebug), "DUAL: ADV & SCAN");
        snprintf(bleScanDebug, sizeof(bleScanDebug), "SCAN: LISTENING...");
        
        strlcpy(bleInitStep, "5: pScan->start()", sizeof(bleInitStep));
        if (pScan->start(5000, false, true)) {
            strlcpy(bleInitStep, "6: SUCCESS (Scanning)", sizeof(bleInitStep));
        } else {
            strlcpy(bleInitStep, "ERR: SCAN FAILED", sizeof(bleInitStep));
            dualLog("[BLE ERR] Scanner failed to start.\n");
        }
    }

    static void sendMidiToHost(uint8_t header, uint8_t data1, uint8_t data2) {
        if (isHostConnected && pServerChar) {
            uint8_t packet[5] = {0x80, 0x80, header, data1, data2};
            pServerChar->setValue(packet, 5);
            pServerChar->notify();
        }
    }

    static void process() {
        // Failsafe: If Mac is disconnected but advertising stopped, force it back on
        if (!isHostConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) {
            NimBLEDevice::getAdvertising()->start();
        }

        if (!doConnect && !hasTarget && !isMvaveConnected && 
            millis() >= reconnectLockoutUntil && !NimBLEDevice::getScan()->isScanning()) {
            NimBLEDevice::getScan()->start(5000, false, true);
        }

        if (doConnect && hasTarget) {
            doConnect = false;
            if (NimBLEDevice::getScan()->isScanning()) {
                NimBLEDevice::getScan()->stop();
            }
            vTaskDelay(pdMS_TO_TICKS(150));

            snprintf(bleEventDebug, sizeof(bleEventDebug), "CONNECTING...");
            dualLog("[BLE PROCESS] Connecting to pedal [%s]...\n", targetAddress.toString().c_str());

            if (!pClient) {
                pClient = NimBLEDevice::createClient();
                pClient->setClientCallbacks(&clientCB);
                
                // Widen connection parameters to absorb M-Vave initial bursts
                pClient->setConnectionParams(12, 40, 0, 500); 
                pClient->setConnectTimeout(5000);
            }

            if (pClient->connect(targetAddress)) {
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
                
                // Immediately destroy the client so NimBLE reclaims the connection slot
                NimBLEDevice::deleteClient(pClient);
                pClient = nullptr;
                
                hasTarget = false;
                reconnectLockoutUntil = millis() + 1500;
            }
        }
    }
};

#endif