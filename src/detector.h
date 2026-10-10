#pragma once
#include <Arduino.h>
#include <math.h>
#include "d200_types.h"

// ============================================================
//  detector.h
//  Geometrická detekce běžící přímo na ESP32 (Heavy Lifting):
//  1. Detekce rovných stěn (úsečky 300 mm ± 40 mm)
//  2. Detekce svíček (izolované shluky průměru 40–80 mm s 2–8 body)
// ============================================================

// Konfigurace pro hledání stěn (Kybernoid překážky = 300 mm bílé desky)
struct WallParams {
    float expected_length_mm = 300.0f;
    float length_tolerance_mm = 60.0f;  // 240 až 360 mm (zohledňuje částečné zakrytí nebo úhlový rozptyl)
    float max_point_dist_mm = 65.0f;    // Max vzdálenost mezi sousedními body jedné stěny
    float max_line_deviation_mm = 22.0f; // Max odchylka bodu od proložené přímky
    int min_points = 5;                 // Minimální počet bodů na stěnu
};

// Konfigurace pro hledání svíček (černé válce průměr 50 mm)
struct CandleParams {
    float min_diameter_mm = 30.0f;      // Min průměr shluku
    float max_diameter_mm = 95.0f;      // Max průměr shluku
    int min_points = 2;                 // Min bodů
    int max_points = 10;                // Max bodů
    float max_neighbor_dist_mm = 50.0f; // Eukleidovská vzdálenost pro clustering
    float min_dist_to_wall_mm = 90.0f;  // Svíčka nesmí být přilepená na stěnu
};

class GeometryDetector {
public:
    static void process(LidarPoint* points, int count,
                        WallSegment* outWalls, int& wallCount,
                        Candle* outCandles, int& candleCount,
                        const WallParams& wParams = WallParams(),
                        const CandleParams& cParams = CandleParams())
    {
        wallCount = 0;
        candleCount = 0;
        if (count < 5) return;

        // Maskovací pole: který bod byl spotřebován do stěny
        // Použijeme statické pole pro úsporu stacku
        static bool usedInWall[MAX_LIDAR_POINTS];
        for (int i = 0; i < count; i++) {
            usedInWall[i] = false;
        }

        // ========================================================
        // 1. KROK: Hledání rovných stěn (Split-and-Merge / Line fitting)
        // Body přicházejí seřazené podle úhlu. Procházíme souvislé sekvence.
        // ========================================================
        int seqStart = -1;
        for (int i = 0; i <= count; i++) {
            bool validPoint = (i < count) && points[i].valid;

            // Zjištění, zda bod navazuje na předchozí (vzdálenost v prostoru)
            bool connected = false;
            if (validPoint && seqStart >= 0) {
                float dx = points[i].x - points[i - 1].x;
                float dy = points[i].y - points[i - 1].y;
                float distSq = dx * dx + dy * dy;
                if (distSq <= (wParams.max_point_dist_mm * wParams.max_point_dist_mm)) {
                    connected = true;
                }
            }

            if (validPoint && (seqStart < 0 || connected)) {
                if (seqStart < 0) seqStart = i;
            } else {
                // Konec souvislé sekvence -> otestujeme, zda tvoří stěnu
                if (seqStart >= 0) {
                    int seqEnd = i - 1;
                    int seqLen = seqEnd - seqStart + 1;

                    if (seqLen >= wParams.min_points && wallCount < MAX_DETECTED_WALLS) {
                        checkAndExtractWall(points, seqStart, seqEnd, wParams,
                                            outWalls, wallCount, usedInWall);
                    }
                    seqStart = validPoint ? i : -1;
                }
            }
        }

        // ========================================================
        // 2. KROK: Hledání svíček (Clustering zbylých volných bodů)
        // ========================================================
        int clStart = -1;
        for (int i = 0; i <= count; i++) {
            bool candidate = (i < count) && points[i].valid && !usedInWall[i];

            bool connected = false;
            if (candidate && clStart >= 0) {
                float dx = points[i].x - points[i - 1].x;
                float dy = points[i].y - points[i - 1].y;
                float distSq = dx * dx + dy * dy;
                if (distSq <= (cParams.max_neighbor_dist_mm * cParams.max_neighbor_dist_mm)) {
                    connected = true;
                }
            }

            if (candidate && (clStart < 0 || connected)) {
                if (clStart < 0) clStart = i;
            } else {
                // Konec clusteru
                if (clStart >= 0) {
                    int clEnd = i - 1;
                    int clLen = clEnd - clStart + 1;

                    if (clLen >= cParams.min_points && clLen <= cParams.max_points && candleCount < MAX_DETECTED_CANDLES) {
                        checkAndExtractCandle(points, clStart, clEnd, cParams,
                                              outWalls, wallCount, outCandles, candleCount);
                    }
                    clStart = candidate ? i : -1;
                }
            }
        }
    }

private:
    // Otestování přímkovosti a rozměru stěny
    static void checkAndExtractWall(const LidarPoint* points, int startIdx, int endIdx,
                                    const WallParams& wp,
                                    WallSegment* outWalls, int& wallCount,
                                    bool* usedInWall)
    {
        float x1 = points[startIdx].x;
        float y1 = points[startIdx].y;
        float x2 = points[endIdx].x;
        float y2 = points[endIdx].y;

        float dx = x2 - x1;
        float dy = y2 - y1;
        float len = sqrtf(dx * dx + dy * dy);

        // Kontrola délky stěny (očekáváme cca 300 mm)
        if (fabsf(len - wp.expected_length_mm) > wp.length_tolerance_mm) {
            return;
        }

        // Přímka daná body (x1, y1) a (x2, y2):
        // Vzdálenost bodu (x0, y0) od přímky = |dy*x0 - dx*y0 + x2*y1 - y2*x1| / len
        float invLen = 1.0f / len;
        float maxDev = 0.0f;
        for (int k = startIdx; k <= endIdx; k++) {
            float distToLine = fabsf(dy * points[k].x - dx * points[k].y + x2 * y1 - y2 * x1) * invLen;
            if (distToLine > maxDev) {
                maxDev = distToLine;
            }
        }

        if (maxDev <= wp.max_line_deviation_mm) {
            // Nalezena platná stěna
            WallSegment seg;
            seg.x1 = x1;
            seg.y1 = y1;
            seg.x2 = x2;
            seg.y2 = y2;
            seg.length_mm = len;
            seg.point_count = endIdx - startIdx + 1;

            outWalls[wallCount++] = seg;

            // Označíme body jako spotřebované
            for (int k = startIdx; k <= endIdx; k++) {
                usedInWall[k] = true;
            }
        }
    }

    // Otestování izolovaného clusteru svíčky
    static void checkAndExtractCandle(const LidarPoint* points, int startIdx, int endIdx,
                                      const CandleParams& cp,
                                      const WallSegment* walls, int wallCount,
                                      Candle* outCandles, int& candleCount)
    {
        // Spočteme střed a rozměr clusteru
        float sumX = 0.0f, sumY = 0.0f;
        int n = endIdx - startIdx + 1;

        for (int k = startIdx; k <= endIdx; k++) {
            sumX += points[k].x;
            sumY += points[k].y;
        }
        float cx = sumX / (float)n;
        float cy = sumY / (float)n;

        // Maximální vzdálenost mezi dvěma body v clusteru (průměr)
        float maxDistSq = 0.0f;
        for (int a = startIdx; a <= endIdx; a++) {
            for (int b = a + 1; b <= endIdx; b++) {
                float ddx = points[a].x - points[b].x;
                float ddy = points[a].y - points[b].y;
                float dsq = ddx * ddx + ddy * ddy;
                if (dsq > maxDistSq) maxDistSq = dsq;
            }
        }
        float diam = sqrtf(maxDistSq);

        // U 2 bodů může být průměr menší než reálný válec (odráží se z jedné strany),
        // proto kontrolujeme i mezní rozsah
        if (diam < cp.min_diameter_mm || diam > cp.max_diameter_mm) {
            // Pokud je to jen 2-3 body na velkou vzdálenost, průměr může být menší, ale omezujeme horní strop
            if (diam > cp.max_diameter_mm) return;
        }

        // Kontrola, zda svíčka neleží příliš blízko některé detekované stěně
        for (int w = 0; w < wallCount; w++) {
            // Vzdálenost bodu (cx, cy) od úsečky (x1, y1)-(x2, y2)
            float vx = walls[w].x2 - walls[w].x1;
            float vy = walls[w].y2 - walls[w].y1;
            float wx = cx - walls[w].x1;
            float wy = cy - walls[w].y1;

            float c1 = wx * vx + wy * vy;
            float c2 = vx * vx + vy * vy;
            float distToWall = 9999.0f;

            if (c2 > 0.001f) {
                float b = c1 / c2;
                if (b < 0.0f) b = 0.0f;
                else if (b > 1.0f) b = 1.0f;
                float projX = walls[w].x1 + b * vx;
                float projY = walls[w].y1 + b * vy;
                float ddx = cx - projX;
                float ddy = cy - projY;
                distToWall = sqrtf(ddx * ddx + ddy * ddy);
            }

            if (distToWall < cp.min_dist_to_wall_mm) {
                return; // Cluster je příliš blízko stěny, ignorujeme
            }
        }

        // Validní svíčka
        Candle cand;
        cand.x = cx;
        cand.y = cy;
        cand.diameter_mm = (diam < 40.0f) ? 50.0f : diam; // Normalizovaný odhad
        cand.point_count = n;

        outCandles[candleCount++] = cand;
    }
};
