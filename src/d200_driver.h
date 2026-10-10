#pragma once
#include <Arduino.h>
#include "d200_types.h"

// ============================================================
//  d200_driver.h
//  Nízkoúrovňový ovladač pro Waveshare D200 / LD14P
//  - Čtení z UART2 (Serial2) s velkým RX bufferem
//  - Validace rámců: 0x54, délka 0x2C (47 B), CRC8 tabulka
//  - Převod do pravotočivého kartézského systému robota
//  - Akumulace plné 360° otáčky
// ============================================================

class D200Driver {
private:
    HardwareSerial& _serial;
    int _rxPin;
    int _txPin;

    static const int PACKET_SIZE = 47;
    uint8_t _packet[PACKET_SIZE];
    int _packetIndex = 0;

    // CRC8 lookup tabulka pro LD14P / LD06 / D200
    static const uint8_t CrcTable[256];

    // Akumulace bodů pro aktuální a dokončenou otáčku
    LidarPoint _accumPoints[MAX_LIDAR_POINTS];
    int _accumCount = 0;

    LidarPoint _completedScan[MAX_LIDAR_POINTS];
    int _completedCount = 0;
    bool _scanReady = false;

    float _prevStartAngle = -1.0f;
    uint32_t _lastScanTimeMs = 0;
    float _currentScanFps = 0.0f;
    uint32_t _crcErrors = 0;

    uint8_t calCrc8(const uint8_t *ptr, uint16_t len) const {
        uint8_t crc = 0;
        while (len--) {
            crc = CrcTable[(crc ^ *ptr++) & 0xFF];
        }
        return crc;
    }

    void processPacket(const FilterConfig& cfg) {
        // Kontrola CRC8 nad prvními 46 bajty (poslední bajt je samotné CRC)
        uint8_t expectedCrc = _packet[PACKET_SIZE - 1];
        if (calCrc8(_packet, PACKET_SIZE - 1) != expectedCrc) {
            _crcErrors++;
            return;
        }

        uint16_t startAngleX100 = _packet[4] | (_packet[5] << 8);
        uint16_t endAngleX100   = _packet[42] | (_packet[43] << 8);

        float startAngle = startAngleX100 / 100.0f;
        float endAngle   = endAngleX100 / 100.0f;

        // Detekce dokončení jedné plné 360° otáčky
        // Pokud startovací úhel klesl (např. z 350° na 5°), dokončili jsme otáčku
        if (_prevStartAngle > 270.0f && startAngle < 90.0f) {
            if (_accumCount > 200) { // Ochrana proti falešnému přetočení
                for (int i = 0; i < _accumCount; i++) {
                    _completedScan[i] = _accumPoints[i];
                }
                _completedCount = _accumCount;
                _scanReady = true;

                uint32_t now = millis();
                if (_lastScanTimeMs > 0) {
                    uint32_t dt = now - _lastScanTimeMs;
                    if (dt > 0) {
                        _currentScanFps = 1000.0f / (float)dt;
                    }
                }
                _lastScanTimeMs = now;
            }
            _accumCount = 0;
        }
        _prevStartAngle = startAngle;

        // Úhlový krok mezi 12 body v paketu
        float step;
        if (endAngle < startAngle) {
            step = (endAngle + 360.0f - startAngle) / 11.0f;
        } else {
            step = (endAngle - startAngle) / 11.0f;
        }

        // Parsování 12 měření
        for (int i = 0; i < 12; i++) {
            if (_accumCount >= MAX_LIDAR_POINTS) break;

            int baseIdx = 6 + i * 3;
            uint16_t dist = _packet[baseIdx] | (_packet[baseIdx + 1] << 8);
            uint8_t intensity = _packet[baseIdx + 2];

            // Nativní úhel z LiDARu (ve stupních)
            float rawAngle = startAngle + step * i;
            while (rawAngle >= 360.0f) rawAngle -= 360.0f;
            while (rawAngle < 0.0f)   rawAngle += 360.0f;

            // --- Převod souřadnicového systému ---
            // D200 standardně rotuje po směru hodinových ručiček (CW).
            // V robotickém standardu (pravotočivý):
            //   0° = přímo dopředu (+Y)
            //   90° = vpravo (+X)
            //   180° = dozadu (-Y)
            //   270° = vlevo (-X)
            // Aplikujeme kalibrační angle_offset_deg:
            float robotAngle = rawAngle - cfg.angle_offset_deg;
            while (robotAngle >= 360.0f) robotAngle -= 360.0f;
            while (robotAngle < 0.0f)   robotAngle += 360.0f;

            // Kartézský převod:
            // X (vpravo)  = dist * sin(robotAngle)
            // Y (dopředu) = dist * cos(robotAngle)
            float rad = robotAngle * (PI / 180.0f);
            float distF = (float)dist;

            LidarPoint pt;
            pt.angle_deg = robotAngle;
            pt.dist_mm = distF;
            pt.intensity = intensity;
            pt.x = distF * sinf(rad);
            pt.y = distF * cosf(rad);
            pt.valid = true;

            _accumPoints[_accumCount++] = pt;
        }
    }

public:
    D200Driver(HardwareSerial& serial = Serial2, int rxPin = 13, int txPin = 10)
        : _serial(serial), _rxPin(rxPin), _txPin(txPin) {}

    void init() {
        _serial.setRxBufferSize(2048);
        _serial.begin(230400, SERIAL_8N1, _rxPin, _txPin);
        _packetIndex = 0;
        _accumCount = 0;
        _completedCount = 0;
        _scanReady = false;
        _crcErrors = 0;
    }

    // Volat v každém průchodu loop()
    void update(const FilterConfig& cfg) {
        while (_serial.available()) {
            uint8_t c = _serial.read();

            if (_packetIndex == 0) {
                if (c == 0x54) {
                    _packet[_packetIndex++] = c;
                }
            } else if (_packetIndex == 1) {
                if (c == 0x2C) {
                    _packet[_packetIndex++] = c;
                } else if (c == 0x54) {
                    _packetIndex = 1; // Další potenciální start
                } else {
                    _packetIndex = 0;
                }
            } else {
                _packet[_packetIndex++] = c;
                if (_packetIndex == PACKET_SIZE) {
                    processPacket(cfg);
                    _packetIndex = 0;
                }
            }
        }
    }

    bool hasNewScan() const {
        return _scanReady;
    }

    // Načte dokončený sken a resetuje flag
    int getCompletedScan(LidarPoint* outBuffer, int maxPoints) {
        if (!_scanReady) return 0;
        int count = min(_completedCount, maxPoints);
        for (int i = 0; i < count; i++) {
            outBuffer[i] = _completedScan[i];
        }
        _scanReady = false;
        return count;
    }

    float getScanFps() const { return _currentScanFps; }
    uint32_t getCrcErrors() const { return _crcErrors; }
};

// Definice lookup tabulky CRC8
const uint8_t D200Driver::CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x53, 0x1e, 0xc9, 0x84, 0x2a, 0x67, 0xb0, 0xfd, 0xa1, 0xec, 0x3b, 0x76, 0xd8, 0x95, 0x42, 0x0f,
    0xfa, 0xb7, 0x60, 0x2d, 0x83, 0xce, 0x19, 0x54, 0x08, 0x45, 0x92, 0xdf, 0x71, 0x3c, 0xeb, 0xa6,
    0xa6, 0xeb, 0x3c, 0x71, 0xdf, 0x92, 0x45, 0x08, 0x54, 0x19, 0xce, 0x83, 0x2d, 0x60, 0xb7, 0xfa,
    0x0f, 0x42, 0x95, 0xd8, 0x76, 0x3b, 0xec, 0xa1, 0xfd, 0xb0, 0x67, 0x2a, 0x84, 0xc9, 0x1e, 0x53,
    0xf5, 0xb8, 0x6f, 0x22, 0x8c, 0xc1, 0x16, 0x5b, 0x07, 0x4a, 0x9d, 0xd0, 0x7e, 0x33, 0xe4, 0xa9,
    0x5c, 0x11, 0xc6, 0x8b, 0x25, 0x68, 0xbf, 0xf2, 0xae, 0xe3, 0x34, 0x79, 0xd7, 0x9a, 0x4d, 0x00,
    0x4d, 0x00, 0xd7, 0x9a, 0x34, 0x79, 0xae, 0xe3, 0xbf, 0xf2, 0x25, 0x68, 0xc6, 0x8b, 0x5c, 0x11,
    0xe4, 0xa9, 0x7e, 0x33, 0x9d, 0xd0, 0x07, 0x4a, 0x16, 0x5b, 0x8c, 0xc1, 0x6f, 0x22, 0xf5, 0xb8,
    0x1e, 0x53, 0x84, 0xc9, 0x67, 0x2a, 0xfd, 0xb0, 0xec, 0xa1, 0x76, 0x3b, 0x95, 0xd8, 0x0f, 0x42,
    0xb7, 0xfa, 0x2d, 0x60, 0xce, 0x83, 0x54, 0x19, 0x45, 0x08, 0xdf, 0x92, 0x3c, 0x71, 0xa6, 0xeb,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x42, 0x0f, 0x95, 0xd8, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0x84, 0xc9, 0x1e, 0x53,
    0xb8, 0xf5, 0x22, 0x6f, 0xc1, 0x8c, 0x5b, 0x16, 0x4a, 0x07, 0xd0, 0x9d, 0x33, 0x7e, 0xa9, 0xe4,
    0x11, 0x5c, 0x8b, 0xc6, 0x68, 0x25, 0xf2, 0xbf, 0xae, 0xe3, 0x34, 0x79, 0xd7, 0x9a, 0x4d, 0x00
};
