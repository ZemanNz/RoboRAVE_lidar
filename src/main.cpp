#include <Arduino.h>
#include "d200_types.h"
#include "d200_driver.h"
#include "filter_pipeline.h"
#include "detector.h"
#include "comm_protocol.h"

// ============================================================
//  LIDAR_SVICKY_27 - Testovací framework pro Waveshare D200
//  Soutěž Kybernoid 27 (hašení svíček)
//  ESP32 provádí veškeré výpočty, filtrování a geometrickou detekci.
// ============================================================

// Piny UART2 pro LiDAR
#define LIDAR_RX_PIN 13
#define LIDAR_TX_PIN 10

// Instance subsystémů
D200Driver lidar(Serial2, LIDAR_RX_PIN, LIDAR_TX_PIN);
CommProtocol comm(Serial);

// Výchozí konfigurace filtrů
FilterConfig config = {
    .min_range_mm = 130.0f,     // Mrtvá zóna D200
    .max_range_mm = 2400.0f,    // Výchozí rozsah 2.4 m ("2 metry něco")
    .min_intensity = 15,        // Práh odrazivosti
    .fov_limited = true,        // Výchozí 180° dopředný kužel
    .fov_angle_deg = 180.0f,    // Úhel dopředného kuželu
    .angle_offset_deg = 0.0f,   // Fyzické pootočení
    .blind_spot_enabled = false,
    .blind_min_deg = 170.0f,    // Příklad vymaskování sloupku za robotem
    .blind_max_deg = 190.0f
};

// Buffery pro zpracování aktuální otáčky
static LidarPoint scanPoints[MAX_LIDAR_POINTS];
static WallSegment detectedWalls[MAX_DETECTED_WALLS];
static Candle detectedCandles[MAX_DETECTED_CANDLES];

void setup() {
    // USB Serial pro přenos dat do Pythonu (vysoká rychlost 921600 baud)
    Serial.begin(921600);
    delay(100);

    // Inicializace LiDAR driveru (Serial2 @ 230400, RX buffer 2048 B)
    lidar.init();

    Serial.println(F("# [INIT] Kybernoid 27 - D200 LiDAR Test Framework Ready"));
}

void loop() {
    // 1. Čtení přicházejících bajtů z LiDARu
    lidar.update(config);

    // 2. Kontrola příkazů z PC přes USB Serial
    comm.checkCommands(config);

    // 3. Jakmile je dokončena celá 360° otáčka, spustíme výpočet
    if (lidar.hasNewScan()) {
        uint32_t tStart = micros();

        // Převzetí bodů
        int rawCount = lidar.getCompletedScan(scanPoints, MAX_LIDAR_POINTS);

        // Aplikace filtrační pipeline (Range clamp, Intenzita, FoV, Maska)
        int validCount = 0;
        FilterPipeline::apply(scanPoints, rawCount, config, validCount);

        // Geometrická detekce na ESP32 (Hledání 30cm stěn a 50mm svíček)
        int wallCount = 0;
        int candleCount = 0;
        GeometryDetector::process(scanPoints, rawCount,
                                 detectedWalls, wallCount,
                                 detectedCandles, candleCount);

        uint32_t calcTimeUs = micros() - tStart;

        // Příprava diagnostiky
        LidarDiagnostics diag;
        diag.scan_fps = lidar.getScanFps();
        diag.total_points = rawCount;
        diag.valid_points = validCount;
        diag.calc_time_us = calcTimeUs;
        diag.free_heap = ESP.getFreeHeap();
        diag.crc_errors = lidar.getCrcErrors();

        // Odeslání výsledků do Pythonu (podvzorkování mračna 1:2 pro úsporu UART bandwidth)
        comm.sendFrame(scanPoints, rawCount,
                       detectedWalls, wallCount,
                       detectedCandles, candleCount,
                       diag, 2);
    }
}
