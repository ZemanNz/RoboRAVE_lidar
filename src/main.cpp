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
    .max_range_mm = 4000.0f,    // Rozsah hřiště / testovací místnosti
    .min_intensity = 15,        // Práh odrazivosti
    .fov_limited = false,       // false = 360°, true = dopředný kužel
    .fov_angle_deg = 180.0f,    // Úhel dopředného kuželu
    .angle_offset_deg = 0.0f,   // Fyzické pootočení
    .blind_spot_enabled = false,
    .blind_min_deg = 170.0f,    // Příklad vymaskování sloupku za robotem
    .blind_max_deg = 190.0f
};

// Buffery pro zpracování aktuální otáčky
static LidarPoint scanPoints[MAX_LIDAR_POINTS];
static BoundingBox detectedBoxes[MAX_DETECTED_BOXES];
static BoundingBox staticBoxes[MAX_DETECTED_BOXES];
static StaticMapTracker staticMap;

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
    bool resetMapRequested = false;
    bool configChanged = comm.checkCommands(config, &resetMapRequested);
    if (configChanged || resetMapRequested) {
        staticMap.reset();
    }

    // 3. Jakmile je dokončena celá 360° otáčka, spustíme výpočet
    if (lidar.hasNewScan()) {
        uint32_t tStart = micros();

        // Převzetí bodů
        int rawCount = lidar.getCompletedScan(scanPoints, MAX_LIDAR_POINTS);

        // Aplikace filtrační pipeline (Range clamp, Intenzita, FoV, Maska)
        int validCount = 0;
        FilterPipeline::apply(scanPoints, rawCount, config, validCount);

        // 1. Výpočet živých Bounding Boxů na ESP32 (Svíčky + Stěny, max 4 ks)
        int boxCount = 0;
        BoxDetector::process(scanPoints, rawCount, detectedBoxes, boxCount);

        // 2. Dlouhodobá statická mapa: akumulace snímků a vyhodnocení nejstabilnějších cílů
        staticMap.update(detectedBoxes, boxCount);
        int staticCount = 0;
        staticMap.getStaticBoxes(staticBoxes, staticCount, 4); // min 4 potvrzení

        uint32_t calcTimeUs = micros() - tStart;

        // Příprava diagnostiky
        LidarDiagnostics diag;
        diag.scan_fps = lidar.getScanFps();
        diag.total_points = rawCount;
        diag.valid_points = validCount;
        diag.calc_time_us = calcTimeUs;
        diag.free_heap = ESP.getFreeHeap();
        diag.crc_errors = lidar.getCrcErrors();

        // Odeslání výsledků do Pythonu (živé boxy i dlouhodobá statická mapa)
        comm.sendFrame(scanPoints, rawCount,
                       detectedBoxes, boxCount,
                       staticBoxes, staticCount,
                       diag, 2);
    }
}
