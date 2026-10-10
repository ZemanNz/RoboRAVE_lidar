#pragma once
#include <Arduino.h>
#include "d200_types.h"

// ============================================================
//  comm_protocol.h
//  Komunikace s PC přes USB Serial (921600 baud)
//  - Odesílání telemetrie, bodů, stěn a svíček ve formátu JSON
//  - Příjem jednoduchých příkazů pro ladění za běhu:
//      FOV 0 / FOV 1 (přepínání 360° / 180°)
//      RANGE <min> <max> (např. RANGE 130 4000)
//      INTENSITY <min> (např. INTENSITY 20)
//      OFFSET <deg> (např. OFFSET 7.0)
//      BLIND <enable> <min> <max> (např. BLIND 1 170 190)
// ============================================================

class CommProtocol {
private:
    Stream& _port;
    char _rxLineBuf[128];
    int _rxLineIdx = 0;

public:
    CommProtocol(Stream& port = Serial) : _port(port) {}

    // Odeslání kompletního rámce v kompaktním JSONu
    void sendFrame(const LidarPoint* points, int pointCount,
                   const WallSegment* walls, int wallCount,
                   const Candle* candles, int candleCount,
                   const LidarDiagnostics& diag,
                   int subsampleStep = 2) // Odesíláme každý 2. bod pro plynulost
    {
        _port.print(F("{\"fps\":"));
        _port.print(diag.scan_fps, 1);
        _port.print(F(",\"pts\":"));
        _port.print(diag.valid_points);
        _port.print(F(",\"calc_us\":"));
        _port.print(diag.calc_time_us);
        _port.print(F(",\"heap\":"));
        _port.print(diag.free_heap);
        _port.print(F(",\"crc_err\":"));
        _port.print(diag.crc_errors);

        // Seznam stěn: [[x1, y1, x2, y2, len], ...]
        _port.print(F(",\"walls\":["));
        for (int i = 0; i < wallCount; i++) {
            if (i > 0) _port.print(',');
            _port.print('[');
            _port.print((int)roundf(walls[i].x1));
            _port.print(',');
            _port.print((int)roundf(walls[i].y1));
            _port.print(',');
            _port.print((int)roundf(walls[i].x2));
            _port.print(',');
            _port.print((int)roundf(walls[i].y2));
            _port.print(',');
            _port.print((int)roundf(walls[i].length_mm));
            _port.print(']');
        }
        _port.print(']');

        // Seznam svíček: [[x, y, diam], ...]
        _port.print(F(",\"candles\":["));
        for (int i = 0; i < candleCount; i++) {
            if (i > 0) _port.print(',');
            _port.print('[');
            _port.print((int)roundf(candles[i].x));
            _port.print(',');
            _port.print((int)roundf(candles[i].y));
            _port.print(',');
            _port.print((int)roundf(candles[i].diameter_mm));
            _port.print(']');
        }
        _port.print(']');

        // Podsamplované body: [x1, y1, int1, x2, y2, int2, ...]
        _port.print(F(",\"cloud\":["));
        bool firstPt = true;
        for (int i = 0; i < pointCount; i += subsampleStep) {
            if (!points[i].valid) continue;
            if (!firstPt) _port.print(',');
            firstPt = false;
            _port.print((int)roundf(points[i].x));
            _port.print(',');
            _port.print((int)roundf(points[i].y));
            _port.print(',');
            _port.print((int)points[i].intensity);
        }
        _port.print(']');

        _port.println('}');
    }

    // Zpracování příchozích příkazů z Pythonu
    bool checkCommands(FilterConfig& cfg) {
        bool changed = false;
        while (_port.available()) {
            char c = _port.read();
            if (c == '\r') continue;
            if (c == '\n') {
                _rxLineBuf[_rxLineIdx] = '\0';
                if (_rxLineIdx > 0) {
                    changed = parseCommand(_rxLineBuf, cfg);
                }
                _rxLineIdx = 0;
            } else if (_rxLineIdx < 120) {
                _rxLineBuf[_rxLineIdx++] = c;
            }
        }
        return changed;
    }

private:
    bool parseCommand(char* cmd, FilterConfig& cfg) {
        // Příkaz: FOV <0|1>
        if (strncmp(cmd, "FOV ", 4) == 0) {
            int val = atoi(cmd + 4);
            cfg.fov_limited = (val == 1);
            _port.print(F("# ACK: fov_limited = "));
            _port.println(cfg.fov_limited);
            return true;
        }

        // Příkaz: RANGE <min> <max>
        if (strncmp(cmd, "RANGE ", 6) == 0) {
            float minVal = 0, maxVal = 0;
            if (sscanf(cmd + 6, "%f %f", &minVal, &maxVal) == 2) {
                cfg.min_range_mm = minVal;
                cfg.max_range_mm = maxVal;
                _port.print(F("# ACK: range = "));
                _port.print(cfg.min_range_mm);
                _port.print(F(" - "));
                _port.println(cfg.max_range_mm);
                return true;
            }
        }

        // Příkaz: INTENSITY <min>
        if (strncmp(cmd, "INTENSITY ", 10) == 0) {
            int val = atoi(cmd + 10);
            cfg.min_intensity = (uint8_t)val;
            _port.print(F("# ACK: min_intensity = "));
            _port.println(cfg.min_intensity);
            return true;
        }

        // Příkaz: OFFSET <deg>
        if (strncmp(cmd, "OFFSET ", 7) == 0) {
            float val = atof(cmd + 7);
            cfg.angle_offset_deg = val;
            _port.print(F("# ACK: angle_offset = "));
            _port.println(cfg.angle_offset_deg);
            return true;
        }

        // Příkaz: BLIND <0|1> <min> <max>
        if (strncmp(cmd, "BLIND ", 6) == 0) {
            int en = 0; float bMin = 0, bMax = 0;
            if (sscanf(cmd + 6, "%d %f %f", &en, &bMin, &bMax) == 3) {
                cfg.blind_spot_enabled = (en == 1);
                cfg.blind_min_deg = bMin;
                cfg.blind_max_deg = bMax;
                _port.print(F("# ACK: blind = "));
                _port.print(cfg.blind_spot_enabled);
                _port.print(F(" ("));
                _port.print(cfg.blind_min_deg);
                _port.print(F("-"));
                _port.print(cfg.blind_max_deg);
                _port.println(F(")"));
                return true;
            }
        }

        return false;
    }
};
