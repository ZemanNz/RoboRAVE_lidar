#pragma once
#include <Arduino.h>

// ============================================================
//  d200_types.h
//  Základní datové struktury a konstanty pro D200 / LD14P
// ============================================================

// Maximální počet bodů na jednu 360° otáčku (při 4000 Hz a 6 Hz je to cca 667 bodů)
#define MAX_LIDAR_POINTS 750
#define MAX_DETECTED_WALLS 6
#define MAX_DETECTED_CANDLES 4

// Struktura jednoho bodu měření
struct LidarPoint {
    float x;           // Kartézské X v mm (pravotočivý systém robota: X doprava, Y dopředu)
    float y;           // Kartézské Y v mm
    float angle_deg;   // Úhel ve stupních [0, 360)
    float dist_mm;     // Vzdálenost v mm
    uint8_t intensity; // Odrazivost / síla signálu (0 - 255)
    bool valid;        // Zda bod prošel filtry
};

// Detekovaná rovná stěna (úsečka)
struct WallSegment {
    float x1, y1;      // Počáteční bod úsečky v mm
    float x2, y2;      // Koncový bod úsečky v mm
    float length_mm;   // Naměřená délka úsečky v mm
    int point_count;   // Počet bodů tvořících stěnu
};

// Detekovaná svíčka (izolovaný válec o průměru ~50 mm)
struct Candle {
    float x, y;        // Odhadovaný střed svíčky v mm
    float diameter_mm; // Odhadovaný průměr shluku v mm
    int point_count;   // Počet bodů clusteru
};

// Detekovaný Bounding Box (Svíčka + Stěna i samostatná svíčka, max 4 ks)
struct BoundingBox {
    float min_x, max_x;
    float min_y, max_y;
    float cx, cy;
    float width, height;
    float span_mm;
    float dist_mm;
    float aspect_ratio;
    int point_count;
    bool is_solo_candle;
};

// Dlouhodobě sledovaný Bounding Box ve statické mapě
struct TrackedBox {
    BoundingBox box;
    int hits;      // Počet detekcí v posledních skenech
    int missed;    // Počet po sobě jdoucích skenů, kdy nebyl vidět
};

enum TargetType {
    TARGET_SOLO = 0,    // Volná svíčka bez stěn
    TARGET_WALL = 1,    // Svíčka chráněná 1 deskou (300 mm)
    TARGET_CORNER = 2   // Svíčka v rohu mezi 2 deskami (L-box)
};

// Kompletní cíl detekovaný a spočítaný přímo na ESP32 (Svíčka, Stěny, Waypoint 38cm, Nájezd)
struct TargetDetail {
    int id;                        // 1 .. 4
    TargetType type;               // SOLO, WALL, CORNER
    BoundingBox box;               // Ohraničující box
    float candle_x, candle_y;      // Přesný střed plamene svíčky v mm
    int wall_count;                // 0, 1 nebo 2 stěny
    WallSegment walls[2];          // Úsečky stěn v mm
    float waypoint_x, waypoint_y;  // Cílový nájezdový bod (38 cm před svíčkou v mm)
    float approach_dx, approach_dy;// Směrový vektor nájezdu (od waypointu do svíčky)
    float travel_dist_mm;          // Vzdálenost od středu robota k waypointu v mm
    bool is_visible;               // Přímo viditelný v tomto skenu
    int hits;                      // Počet potvrzení v čase
};

#define MAX_DETECTED_BOXES 4
#define MAX_TRACKED_BOXES 8

// Diagnostika a telemetrie
struct LidarDiagnostics {
    float scan_fps;          // Skutečná frekvence otáček LiDARu (Hz)
    uint32_t total_points;   // Počet surových bodů za poslední otáčku
    uint32_t valid_points;   // Počet bodů po filtraci
    uint32_t calc_time_us;   // Doba výpočtu pipeline na ESP32 (mikrosekundy)
    uint32_t free_heap;      // Volná RAM na ESP32 (bajty)
    uint32_t crc_errors;     // Počet zahozených paketů kvůli chybné CRC
};

// Konfigurace filtrů za běhu
struct FilterConfig {
    float min_range_mm;       // Range clamp min (mrtvá zóna, výchozí 130 mm)
    float max_range_mm;       // Range clamp max (místnost, výchozí 4000 mm)
    uint8_t min_intensity;    // Minimální odrazivost (výchozí 15)
    bool fov_limited;         // Omezení zorného pole (false = 360°, true = dopředný kužel)
    float fov_angle_deg;      // Šířka dopředného kuželu (např. 180° nebo 120°)
    float angle_offset_deg;   // Fyzické pootočení uchycení senzoru (např. 7.0°)
    
    // Konstrukční maska (výřez úhlů, kde senzoru stíní sloupky/konstrukce)
    bool blind_spot_enabled;
    float blind_min_deg;      // Např. zadní sloupky 170° - 190°
    float blind_max_deg;
};
