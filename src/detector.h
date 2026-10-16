#pragma once
#include <Arduino.h>
#include <math.h>
#include "d200_types.h"

// ============================================================
//  detector.h
//  Geometrická detekce na ESP32 (Kybernoid 27)
//
//  Inovace (Cílené prokládání 30cm stěn & Hypotheses Generation):
//  1. Svíčky se najdou jako první (max 4 ks).
//  2. Pro každou svíčku se prozkoumá okolí (cca 100-380 mm za/kolem svíčky):
//     - Hledají se body tvořící stěnu(y) chránící tuto svíčku.
//     - Proloží se orientovaná úsečka o nominální délce 280-300 mm.
//     - Funguje perfektně i když svíčka v ose pohledu stíní střed stěny!
//  3. Prozkoumání zbylých bodů na volné stěny (Split-and-Merge).
//  4. Přísný strop délky: 240 mm <= délka <= 330 mm.
//  5. Kolmost (90°) pro stěny tvořící rohy a U-boxy.
//  6. Maximálně 6 nejlepších stěn.
// ============================================================

struct WallParams {
    float expected_length_mm = 300.0f;
    float max_length_mm = 330.0f;       // TVRDÝ STROP: nad 330 mm to NENÍ herní stěna!
    float min_length_mm = 160.0f;       // Minimální detekovaná délka pro volnou stěnu
    float max_point_dist_mm = 85.0f;    // Mezera mezi body
    float max_line_deviation_mm = 25.0f; // Max odchylka bodu od přímky
    int min_points = 4;
};

struct CandleParams {
    float min_diameter_mm = 25.0f;
    float max_diameter_mm = 75.0f;
    int min_points = 3;
    int max_points = 8;
    float max_neighbor_dist_mm = 45.0f;
    float min_dist_to_wall_mm = 40.0f;
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

        static bool usedInCandle[MAX_LIDAR_POINTS];
        for (int i = 0; i < count; i++) {
            usedInCandle[i] = false;
        }

        // ========================================================
        // 1. KROK: Hledání svíček (max 4 ks)
        // ========================================================
        int clStart = -1;
        for (int i = 0; i <= count; i++) {
            bool candidate = (i < count) && points[i].valid;

            bool connected = false;
            if (candidate && clStart >= 0) {
                float dx = points[i].x - points[i - 1].x;
                float dy = points[i].y - points[i - 1].y;
                if ((dx * dx + dy * dy) <= (cParams.max_neighbor_dist_mm * cParams.max_neighbor_dist_mm)) {
                    connected = true;
                }
            }

            if (candidate && (clStart < 0 || connected)) {
                if (clStart < 0) clStart = i;
            } else {
                if (clStart >= 0) {
                    int clEnd = i - 1;
                    int clLen = clEnd - clStart + 1;

                    if (clLen >= cParams.min_points && clLen <= cParams.max_points && candleCount < MAX_DETECTED_CANDLES) {
                        checkAndExtractCandle(points, clStart, clEnd, cParams,
                                              outCandles, candleCount, usedInCandle);
                    }
                    clStart = candidate ? i : -1;
                }
            }
        }

        // ========================================================
        // 2. KROK: CÍLENÉ HLEDÁNÍ STĚN OKOLO DETEKOVANÝCH SVÍČEK
        // Svíčka je středem hnízda; stěny jsou ve vzdálenosti 120 až 380 mm od jejího středu
        // ========================================================
        #define MAX_CANDIDATES 24
        static WallSegment candidates[MAX_CANDIDATES];
        int numCandidates = 0;
        static bool usedInWall[MAX_LIDAR_POINTS];
        for (int i = 0; i < count; i++) usedInWall[i] = false;

        for (int c = 0; c < candleCount; c++) {
            float cx = outCandles[c].x;
            float cy = outCandles[c].y;

            // Vybereme body v okolí svíčky (100 až 400 mm od svíčky)
            int localIdxs[60];
            int localCount = 0;
            for (int i = 0; i < count && localCount < 60; i++) {
                if (!points[i].valid || usedInCandle[i]) continue;
                float d = hypotf(points[i].x - cx, points[i].y - cy);
                if (d >= 100.0f && d <= 400.0f) {
                    localIdxs[localCount++] = i;
                }
            }

            if (localCount >= 4) {
                // Hledáme nejlepší 30cm úsečky v tomto okolí pomocí RANSAC / proložení
                findWallsAroundCandle(points, localIdxs, localCount, cx, cy, wParams,
                                      candidates, numCandidates, usedInWall);
            }
        }

        // ========================================================
        // 3. KROK: Hledání zbylých volných stěn na hřišti (např. samostatná stěna)
        // ========================================================
        int seqStart = -1;
        for (int i = 0; i <= count; i++) {
            bool validPoint = (i < count) && points[i].valid && !usedInCandle[i] && !usedInWall[i];

            bool connected = false;
            if (validPoint && seqStart >= 0) {
                float dx = points[i].x - points[i - 1].x;
                float dy = points[i].y - points[i - 1].y;
                if ((dx * dx + dy * dy) <= (wParams.max_point_dist_mm * wParams.max_point_dist_mm)) {
                    connected = true;
                }
            }

            if (validPoint && (seqStart < 0 || connected)) {
                if (seqStart < 0) seqStart = i;
            } else {
                if (seqStart >= 0) {
                    int seqEnd = i - 1;
                    int seqLen = seqEnd - seqStart + 1;
                    if (seqLen >= wParams.min_points && numCandidates < MAX_CANDIDATES) {
                        extractFreeWall(points, seqStart, seqEnd, wParams, candidates, numCandidates);
                    }
                    seqStart = validPoint ? i : -1;
                }
            }
        }

        // ========================================================
        // 4. KROK: Eliminace duplicitních překryvů a výběr TOP 6
        // ========================================================
        // Odstranění stěn, které se navzájem téměř kryjí
        static WallSegment uniqueWalls[MAX_CANDIDATES];
        int numUnique = 0;

        for (int i = 0; i < numCandidates; i++) {
            const WallSegment& w1 = candidates[i];
            if (w1.length_mm > wParams.max_length_mm || w1.length_mm < 160.0f) continue;

            float m1x = (w1.x1 + w1.x2) * 0.5f;
            float m1y = (w1.y1 + w1.y2) * 0.5f;
            bool duplicate = false;

            for (int j = 0; j < numUnique; j++) {
                const WallSegment& w2 = uniqueWalls[j];
                float m2x = (w2.x1 + w2.x2) * 0.5f;
                float m2y = (w2.y1 + w2.y2) * 0.5f;
                if (hypotf(m1x - m2x, m1y - m2y) < 70.0f) {
                    duplicate = true;
                    // Ponecháme tu s délkou bližší 300 mm
                    if (fabsf(w1.length_mm - 300.0f) < fabsf(w2.length_mm - 300.0f)) {
                        uniqueWalls[j] = w1;
                    }
                    break;
                }
            }

            if (!duplicate && numUnique < MAX_CANDIDATES) {
                uniqueWalls[numUnique++] = w1;
            }
        }

        // Skórování a výběr max 6
        if (numUnique <= MAX_DETECTED_WALLS) {
            for (int i = 0; i < numUnique; i++) {
                outWalls[i] = uniqueWalls[i];
            }
            wallCount = numUnique;
        } else {
            float scores[MAX_CANDIDATES];
            for (int i = 0; i < numUnique; i++) {
                float score = 0.0f;
                const WallSegment& w = uniqueWalls[i];

                // 1. Délka blízko 300 mm
                float diff = fabsf(w.length_mm - 300.0f);
                score += fmaxf(0.0f, (60.0f - diff) * 3.0f);

                float mx = (w.x1 + w.x2) * 0.5f;
                float my = (w.y1 + w.y2) * 0.5f;
                float wdx = (w.x2 - w.x1) / w.length_mm;
                float wdy = (w.y2 - w.y1) / w.length_mm;

                // 2. Svíčka před/uprostřed stěny
                for (int c = 0; c < candleCount; c++) {
                    float d = hypotf(outCandles[c].x - mx, outCandles[c].y - my);
                    if (d < 380.0f) {
                        score += (380.0f - d) * 1.2f;
                    }
                }

                // 3. Kolmost (pravý úhel) na jinou stěnu
                for (int j = 0; j < numUnique; j++) {
                    if (i == j) continue;
                    const WallSegment& w2 = uniqueWalls[j];
                    float w2dx = (w2.x2 - w2.x1) / w2.length_mm;
                    float w2dy = (w2.y2 - w2.y1) / w2.length_mm;
                    float dot = fabsf(wdx * w2dx + wdy * w2dy);
                    if (dot < 0.22f) { // cca 90°
                        float d1 = hypotf(w.x1 - w2.x1, w.y1 - w2.y1);
                        float d2 = hypotf(w.x1 - w2.x2, w.y1 - w2.y2);
                        float d3 = hypotf(w.x2 - w2.x1, w.y2 - w2.y1);
                        float d4 = hypotf(w.x2 - w2.x2, w.y2 - w2.y2);
                        if (fminf(fminf(d1, d2), fminf(d3, d4)) < 180.0f) {
                            score += 200.0f;
                        }
                    }
                }
                scores[i] = score;
            }

            bool picked[MAX_CANDIDATES];
            for (int i = 0; i < numUnique; i++) picked[i] = false;

            for (int k = 0; k < MAX_DETECTED_WALLS && k < numUnique; k++) {
                int bestIdx = -1;
                float bestScore = -9999.0f;
                for (int i = 0; i < numUnique; i++) {
                    if (!picked[i] && scores[i] > bestScore) {
                        bestScore = scores[i];
                        bestIdx = i;
                    }
                }
                if (bestIdx >= 0) {
                    picked[bestIdx] = true;
                    outWalls[wallCount++] = uniqueWalls[bestIdx];
                }
            }
        }

        // ========================================================
        // 5. KROK: Validace svíček vůči stěnám
        // Svíčka NESMÍ ležet v dráze stěny (nesmí být v přímce stěny ani na jejím rohu).
        // Skutečná svíčka stojí kolmo před stěnou s dostatečným odstupem (d_perp >= 55 mm).
        // ========================================================
        static Candle validCandles[MAX_DETECTED_CANDLES];
        int validCandleCount = 0;

        for (int c = 0; c < candleCount; c++) {
            const Candle& cand = outCandles[c];
            bool inWallPath = false;

            for (int w = 0; w < wallCount; w++) {
                const WallSegment& wall = outWalls[w];
                float dx = wall.x2 - wall.x1;
                float dy = wall.y2 - wall.y1;
                float wLen = wall.length_mm;
                if (wLen < 10.0f) continue;

                float invL = 1.0f / wLen;
                float ux = dx * invL;
                float uy = dy * invL;
                float nx = -uy;
                float ny = ux;

                float vx = cand.x - wall.x1;
                float vy = cand.y - wall.y1;

                float projT = vx * ux + vy * uy;
                float distToLine = fabsf(vx * nx + vy * ny);

                // Zakázaný koridor stěny (prodloužený o 150 mm před i za stěnu):
                // Svíčka nesmí ležet v přímce stěny ani v mezeře/rohu mezi stěnami (d_perp <= 55 mm)
                if (distToLine <= 55.0f && projT >= -150.0f && projT <= (wLen + 150.0f)) {
                    inWallPath = true;
                    break;
                }
            }

            if (!inWallPath && validCandleCount < MAX_DETECTED_CANDLES) {
                validCandles[validCandleCount++] = cand;
            }
        }

        for (int c = 0; c < validCandleCount; c++) {
            outCandles[c] = validCandles[c];
        }
        candleCount = validCandleCount;
    }

private:
    // Cílené prokládání stěn v okolí svíčky
    // Najde i stěnu přerušenou svíčkou (stínem) na dvě části
    static void findWallsAroundCandle(const LidarPoint* points, const int* idxs, int nIdxs,
                                      float cx, float cy, const WallParams& wp,
                                      WallSegment* outWalls, int& wallCount,
                                      bool* usedInWall)
    {
        // Vyzkoušíme dvojice vzdálenějších bodů jako nosné přímky
        for (int a = 0; a < nIdxs; a++) {
            int ia = idxs[a];
            for (int b = a + 1; b < nIdxs; b++) {
                int ib = idxs[b];
                float dx = points[ib].x - points[ia].x;
                float dy = points[ib].y - points[ia].y;
                float distAB = hypotf(dx, dy);

                // Hledáme úseky, které by mohly tvořit kraje 30cm desky (cca 200 až 320 mm)
                if (distAB < 180.0f || distAB > 325.0f) continue;

                float invD = 1.0f / distAB;
                float nx = -dy * invD;
                float ny = dx * invD;

                // Spočteme body ležící na této přímce (inliery)
                int inliers = 0;
                float minProj = 1e9f, maxProj = -1e9f;
                int inlierIdxs[30];

                for (int k = 0; k < nIdxs; k++) {
                    int ik = idxs[k];
                    float dev = fabsf((points[ik].x - points[ia].x) * nx + (points[ik].y - points[ia].y) * ny);
                    if (dev <= wp.max_line_deviation_mm) {
                        float proj = (points[ik].x - points[ia].x) * (dx * invD) + (points[ik].y - points[ia].y) * (dy * invD);
                        if (proj < minProj) minProj = proj;
                        if (proj > maxProj) maxProj = proj;
                        if (inliers < 30) inlierIdxs[inliers] = ik;
                        inliers++;
                    }
                }

                float wallLen = maxProj - minProj;
                // Pokud inliery pokrývají 220 až 330 mm a je jich dostatek
                if (inliers >= 5 && wallLen >= 200.0f && wallLen <= wp.max_length_mm) {
                    if (wallCount < MAX_CANDIDATES) {
                        WallSegment seg;
                        float ux = dx * invD;
                        float uy = dy * invD;
                        seg.x1 = points[ia].x + minProj * ux;
                        seg.y1 = points[ia].y + minProj * uy;
                        seg.x2 = points[ia].x + maxProj * ux;
                        seg.y2 = points[ia].y + maxProj * uy;
                        seg.length_mm = wallLen;
                        seg.point_count = inliers;

                        // Označíme inliery
                        for (int k = 0; k < inliers; k++) {
                            usedInWall[inlierIdxs[k]] = true;
                        }

                        outWalls[wallCount++] = seg;
                        return; // Nalezena stěna pro toto rameno svíčky
                    }
                }
            }
        }
    }

    // Detekce volné stěny metodou Split-and-Merge
    static void extractFreeWall(const LidarPoint* points, int startIdx, int endIdx,
                                const WallParams& wp,
                                WallSegment* outWalls, int& wallCount)
    {
        if (wallCount >= MAX_CANDIDATES) return;

        float x1 = points[startIdx].x;
        float y1 = points[startIdx].y;
        float x2 = points[endIdx].x;
        float y2 = points[endIdx].y;

        float dx = x2 - x1;
        float dy = y2 - y1;
        float len = hypotf(dx, dy);

        if (len < wp.min_length_mm) return;

        float invLen = 1.0f / len;
        float maxDev = 0.0f;
        int maxDevIdx = -1;
        for (int k = startIdx; k <= endIdx; k++) {
            float distToLine = fabsf(dy * points[k].x - dx * points[k].y + x2 * y1 - y2 * x1) * invLen;
            if (distToLine > maxDev) {
                maxDev = distToLine;
                maxDevIdx = k;
            }
        }

        if (maxDev <= wp.max_line_deviation_mm) {
            if (len <= wp.max_length_mm) {
                WallSegment seg;
                seg.x1 = x1; seg.y1 = y1;
                seg.x2 = x2; seg.y2 = y2;
                seg.length_mm = len;
                seg.point_count = endIdx - startIdx + 1;
                outWalls[wallCount++] = seg;
            }
        } else if (maxDev > 22.0f && maxDevIdx > startIdx + 2 && maxDevIdx < endIdx - 2) {
            extractFreeWall(points, startIdx, maxDevIdx, wp, outWalls, wallCount);
            extractFreeWall(points, maxDevIdx, endIdx, wp, outWalls, wallCount);
        }
    }

    static void checkAndExtractCandle(const LidarPoint* points, int startIdx, int endIdx,
                                      const CandleParams& cp,
                                      Candle* outCandles, int& candleCount,
                                      bool* usedInCandle)
    {
        float sumX = 0.0f, sumY = 0.0f;
        int n = endIdx - startIdx + 1;

        for (int k = startIdx; k <= endIdx; k++) {
            sumX += points[k].x;
            sumY += points[k].y;
        }
        float cx = sumX / (float)n;
        float cy = sumY / (float)n;

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

        if (diam < cp.min_diameter_mm || diam > cp.max_diameter_mm) {
            return;
        }

        Candle cand;
        cand.x = cx;
        cand.y = cy;
        cand.diameter_mm = (diam < 40.0f) ? 50.0f : diam;
        cand.point_count = n;

        outCandles[candleCount++] = cand;

        for (int k = startIdx; k <= endIdx; k++) {
            usedInCandle[k] = true;
        }
    }
};
