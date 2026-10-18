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

    // Odeslání kompletního rámce v kompaktním JSONu (včetně cílů a trasy počítaných na ESP32)
    void sendFrame(const LidarPoint* points, int pointCount,
                   const BoundingBox* boxes, int boxCount,
                   const BoundingBox* staticBoxes, int staticBoxCount,
                   const TargetDetail* targets, int targetCount,
                   const int* route, int routeCount,
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

        // 1. Živé Bounding Boxy (levý panel): [[minX, maxX, minY, maxY, cx, cy, w, h, span, dist, pts, is_solo], ...]
        _port.print(F(",\"boxes\":["));
        for (int i = 0; i < boxCount; i++) {
            if (i > 0) _port.print(',');
            _port.print('[');
            _port.print((int)roundf(boxes[i].min_x)); _port.print(',');
            _port.print((int)roundf(boxes[i].max_x)); _port.print(',');
            _port.print((int)roundf(boxes[i].min_y)); _port.print(',');
            _port.print((int)roundf(boxes[i].max_y)); _port.print(',');
            _port.print((int)roundf(boxes[i].cx)); _port.print(',');
            _port.print((int)roundf(boxes[i].cy)); _port.print(',');
            _port.print((int)roundf(boxes[i].width)); _port.print(',');
            _port.print((int)roundf(boxes[i].height)); _port.print(',');
            _port.print((int)roundf(boxes[i].span_mm)); _port.print(',');
            _port.print((int)roundf(boxes[i].dist_mm)); _port.print(',');
            _port.print(boxes[i].point_count); _port.print(',');
            _port.print(boxes[i].is_solo_candle ? 1 : 0);
            _port.print(']');
        }
        _port.print(']');

        // 2. Dlouhodobá statická mapa (pravý panel): [[minX, maxX, minY, maxY, cx, cy, w, h, span, dist, pts, is_solo], ...]
        _port.print(F(",\"static_boxes\":["));
        for (int i = 0; i < staticBoxCount; i++) {
            if (i > 0) _port.print(',');
            _port.print('[');
            _port.print((int)roundf(staticBoxes[i].min_x)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].max_x)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].min_y)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].max_y)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].cx)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].cy)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].width)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].height)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].span_mm)); _port.print(',');
            _port.print((int)roundf(staticBoxes[i].dist_mm)); _port.print(',');
            _port.print(staticBoxes[i].point_count); _port.print(',');
            _port.print(staticBoxes[i].is_solo_candle ? 1 : 0);
            _port.print(']');
        }
        _port.print(']');

        // 3. Kompletní detailní cíle (počítáno 100% na ESP32: Svíčky, Stěny, Waypointy a Nájezdy)
        _port.print(F(",\"targets\":["));
        for (int i = 0; i < targetCount; i++) {
            if (i > 0) _port.print(',');
            const TargetDetail& td = targets[i];
            _port.print(F("{\"id\":")); _port.print(td.id);
            _port.print(F(",\"type\":"));
            if (td.type == TARGET_CORNER) _port.print(F("\"ROH\""));
            else if (td.type == TARGET_WALL) _port.print(F("\"STĚNA\""));
            else _port.print(F("\"VOLNÁ\""));
            _port.print(F(",\"candle\":["));
            _port.print((int)roundf(td.candle_x)); _port.print(',');
            _port.print((int)roundf(td.candle_y)); _port.print(']');
            _port.print(F(",\"box\":["));
            _port.print((int)roundf(td.box.min_x)); _port.print(',');
            _port.print((int)roundf(td.box.max_x)); _port.print(',');
            _port.print((int)roundf(td.box.min_y)); _port.print(',');
            _port.print((int)roundf(td.box.max_y)); _port.print(',');
            _port.print((int)roundf(td.box.cx)); _port.print(',');
            _port.print((int)roundf(td.box.cy)); _port.print(',');
            _port.print((int)roundf(td.box.width)); _port.print(',');
            _port.print((int)roundf(td.box.height)); _port.print(']');
            _port.print(F(",\"waypoint\":["));
            _port.print((int)roundf(td.waypoint_x)); _port.print(',');
            _port.print((int)roundf(td.waypoint_y)); _port.print(']');
            _port.print(F(",\"approach\":["));
            _port.print(td.approach_dx, 3); _port.print(',');
            _port.print(td.approach_dy, 3); _port.print(']');
            _port.print(F(",\"travel_mm\":")); _port.print((int)roundf(td.travel_dist_mm));
            _port.print(F(",\"walls\":["));
            for (int w = 0; w < td.wall_count; w++) {
                if (w > 0) _port.print(',');
                _port.print(F("[["));
                _port.print((int)roundf(td.walls[w].x1)); _port.print(',');
                _port.print((int)roundf(td.walls[w].y1)); _port.print(F("],["));
                _port.print((int)roundf(td.walls[w].x2)); _port.print(',');
                _port.print((int)roundf(td.walls[w].y2)); _port.print(F("]]"));
            }
            _port.print(']');
            _port.print('}');
        }
        _port.print(']');

        // 4. Optimální trasa hašení (pořadí ID cílů ze startu robota)
        _port.print(F(",\"route\":["));
        for (int i = 0; i < routeCount; i++) {
            if (i > 0) _port.print(',');
            _port.print(route[i]);
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
    bool checkCommands(FilterConfig& cfg, bool* outResetMap = nullptr) {
        bool changed = false;
        while (_port.available()) {
            char c = _port.read();
            if (c == '\r') continue;
            if (c == '\n') {
                _rxLineBuf[_rxLineIdx] = '\0';
                if (_rxLineIdx > 0) {
                    if (strncmp(_rxLineBuf, "RESET_MAP", 9) == 0) {
                        if (outResetMap) *outResetMap = true;
                        _port.println(F("# ACK: RESET_MAP"));
                        changed = true;
                    } else {
                        changed = parseCommand(_rxLineBuf, cfg);
                    }
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
