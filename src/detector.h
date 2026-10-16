#pragma once
#include <Arduino.h>
#include <math.h>
#include "d200_types.h"

// ============================================================
//  detector.h
//  Geometrická detekce na ESP32 (Kybernoid 27)
//
//  Vylepšení:
//  1. Cílené prokládání stěn okolo svíček (i s málo body / přes stín svíčky).
//  2. Rozšíření stěny na nominální rozměr 300 mm:
//     Pokud je vidět jen část desky za svíčkou (např. 180-260 mm),
//     stěna se symetricky protáhne na 290-300 mm.
//  3. Detekce a vynucení kolmosti (pravé úhly 90°):
//     Pokud jsou dvě stěny blízko sebe (do 180 mm), zarovnají se přesně do 90°
//     podle té jistější/delší z nich.
//  4. Přísný strop: stěna nepřesáhne 330 mm.
//  5. Max 6 stěn a 4 svíčky.
// ============================================================

struct WallParams {
    float expected_length_mm = 300.0f;
    float max_length_mm = 330.0f;       // TVRDÝ STROP
    float min_length_mm = 140.0f;       // Chytí i kratší odrazy
    float max_point_dist_mm = 85.0f;    
    float max_line_deviation_mm = 26.0f; 
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
        // 2. KROK: Hledání kandidátů stěn v okolí svíček
        // ========================================================
        #define MAX_CANDIDATES 24
        static WallSegment candidates[MAX_CANDIDATES];
        int numCandidates = 0;
        static bool usedInWall[MAX_LIDAR_POINTS];
        for (int i = 0; i < count; i++) usedInWall[i] = false;

        for (int c = 0; c < candleCount; c++) {
            float cx = outCandles[c].x;
            float cy = outCandles[c].y;

            int localIdxs[60];
            int localCount = 0;
            for (int i = 0; i < count && localCount < 60; i++) {
                if (!points[i].valid || usedInCandle[i]) continue;
                float d = hypotf(points[i].x - cx, points[i].y - cy);
                if (d >= 90.0f && d <= 420.0f) {
                    localIdxs[localCount++] = i;
                }
            }

            if (localCount >= 4) {
                findWallsAroundCandle(points, localIdxs, localCount, cx, cy, wParams,
                                      candidates, numCandidates, usedInWall);
            }
        }

        // ========================================================
        // 3. KROK: Hledání zbylých volných stěn
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
        // 4. KROK: Eliminace duplicit
        // ========================================================
        static WallSegment uniqueWalls[MAX_CANDIDATES];
        int numUnique = 0;

        for (int i = 0; i < numCandidates; i++) {
            const WallSegment& w1 = candidates[i];
            if (w1.length_mm > wParams.max_length_mm || w1.length_mm < 140.0f) continue;

            float m1x = (w1.x1 + w1.x2) * 0.5f;
            float m1y = (w1.y1 + w1.y2) * 0.5f;
            bool duplicate = false;

            for (int j = 0; j < numUnique; j++) {
                const WallSegment& w2 = uniqueWalls[j];
                float m2x = (w2.x1 + w2.x2) * 0.5f;
                float m2y = (w2.y1 + w2.y2) * 0.5f;
                if (hypotf(m1x - m2x, m1y - m2y) < 70.0f) {
                    duplicate = true;
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

        // ========================================================
        // 5. KROK: Vynucení kolmosti rohových stěn (Orthogonal snapping)
        // Podle delší/jistější stěny zarovnáme druhou stěnu přesně do 90°
        // ========================================================
        for (int i = 0; i < numUnique; i++) {
            for (int j = i + 1; j < numUnique; j++) {
                WallSegment& wA = (uniqueWalls[i].length_mm >= uniqueWalls[j].length_mm) ? uniqueWalls[i] : uniqueWalls[j];
                WallSegment& wB = (uniqueWalls[i].length_mm >= uniqueWalls[j].length_mm) ? uniqueWalls[j] : uniqueWalls[i];

                float d1 = hypotf(wA.x1 - wB.x1, wA.y1 - wB.y1);
                float d2 = hypotf(wA.x1 - wB.x2, wA.y1 - wB.y2);
                float d3 = hypotf(wA.x2 - wB.x1, wA.y2 - wB.y1);
                float d4 = hypotf(wA.x2 - wB.x2, wA.y2 - wB.y2);
                float minEndDist = fminf(fminf(d1, d2), fminf(d3, d4));

                if (minEndDist < 160.0f) { // Stěny se v rohu dotýkají
                    float adx = (wA.x2 - wA.x1) / wA.length_mm;
                    float ady = (wA.y2 - wA.y1) / wA.length_mm;
                    float bdx = (wB.x2 - wB.x1) / wB.length_mm;
                    float bdy = (wB.y2 - wB.y1) / wB.length_mm;

                    float dot = fabsf(adx * bdx + ady * bdy);
                    if (dot < 0.35f) { // úhel cca 70° až 110° -> je to pravý roh!
                        // Vytvoříme ideální kolmý vektor k wA
                        float orthoX = -ady;
                        float orthoY =  adx;
                        if ((orthoX * bdx + orthoY * bdy) < 0.0f) {
                            orthoX = -orthoX;
                            orthoY = -orthoY;
                        }

                        // Vyrovnáme wB do přesné kolmice podle středu
                        float bmx = (wB.x1 + wB.x2) * 0.5f;
                        float bmy = (wB.y1 + wB.y2) * 0.5f;
                        float hlen = wB.length_mm * 0.5f;
                        wB.x1 = bmx - orthoX * hlen;
                        wB.y1 = bmy - orthoY * hlen;
                        wB.x2 = bmx + orthoX * hlen;
                        wB.y2 = bmy + orthoY * hlen;
                    }
                }
            }
        }

        // ========================================================
        // 6. KROK: Skórování a výběr TOP 6
        // ========================================================
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

                float diff = fabsf(w.length_mm - 300.0f);
                score += fmaxf(0.0f, (60.0f - diff) * 3.0f);

                float mx = (w.x1 + w.x2) * 0.5f;
                float my = (w.y1 + w.y2) * 0.5f;
                float wdx = (w.x2 - w.x1) / w.length_mm;
                float wdy = (w.y2 - w.y1) / w.length_mm;

                for (int c = 0; c < candleCount; c++) {
                    float d = hypotf(outCandles[c].x - mx, outCandles[c].y - my);
                    if (d < 380.0f) {
                        score += (380.0f - d) * 1.5f;
                    }
                }

                for (int j = 0; j < numUnique; j++) {
                    if (i == j) continue;
                    const WallSegment& w2 = uniqueWalls[j];
                    float w2dx = (w2.x2 - w2.x1) / w2.length_mm;
                    float w2dy = (w2.y2 - w2.y1) / w2.length_mm;
                    float dot = fabsf(wdx * w2dx + wdy * w2dy);
                    if (dot < 0.22f) {
                        float d1 = hypotf(w.x1 - w2.x1, w.y1 - w2.y1);
                        float d2 = hypotf(w.x1 - w2.x2, w.y1 - w2.y2);
                        float d3 = hypotf(w.x2 - w2.x1, w.y2 - w2.y1);
                        float d4 = hypotf(w.x2 - w2.x2, w.y2 - w2.y2);
                        if (fminf(fminf(d1, d2), fminf(d3, d4)) < 180.0f) {
                            score += 250.0f;
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
    }

private:
    static void findWallsAroundCandle(const LidarPoint* points, const int* idxs, int nIdxs,
                                      float cx, float cy, const WallParams& wp,
                                      WallSegment* outWalls, int& wallCount,
                                      bool* usedInWall)
    {
        for (int a = 0; a < nIdxs; a++) {
            int ia = idxs[a];
            for (int b = a + 1; b < nIdxs; b++) {
                int ib = idxs[b];
                float dx = points[ib].x - points[ia].x;
                float dy = points[ib].y - points[ia].y;
                float distAB = hypotf(dx, dy);

                if (distAB < 120.0f || distAB > 330.0f) continue;

                float invD = 1.0f / distAB;
                float nx = -dy * invD;
                float ny = dx * invD;

                int inliers = 0;
                float minProj = 1e9f, maxProj = -1e9f;
                int inlierIdxs[35];

                for (int k = 0; k < nIdxs; k++) {
                    int ik = idxs[k];
                    float dev = fabsf((points[ik].x - points[ia].x) * nx + (points[ik].y - points[ia].y) * ny);
                    if (dev <= wp.max_line_deviation_mm) {
                        float proj = (points[ik].x - points[ia].x) * (dx * invD) + (points[ik].y - points[ia].y) * (dy * invD);
                        if (proj < minProj) minProj = proj;
                        if (proj > maxProj) maxProj = proj;
                        if (inliers < 35) inlierIdxs[inliers] = ik;
                        inliers++;
                    }
                }

                float wallLen = maxProj - minProj;
                // Stačí i 140 mm naměřené délky, pokud má alespoň 4 body
                if (inliers >= 4 && wallLen >= 140.0f && wallLen <= wp.max_length_mm) {
                    if (wallCount < MAX_CANDIDATES) {
                        WallSegment seg;
                        float ux = dx * invD;
                        float uy = dy * invD;

                        // Pokud je stěna částečně zakrytá svíčkou (např. 180-260 mm),
                        // protáhneme ji symetricky na nominální délku 295 mm
                        if (wallLen < 270.0f) {
                            float midP = (minProj + maxProj) * 0.5f;
                            minProj = midP - 147.5f;
                            maxProj = midP + 147.5f;
                            wallLen = 295.0f;
                        }

                        seg.x1 = points[ia].x + minProj * ux;
                        seg.y1 = points[ia].y + minProj * uy;
                        seg.x2 = points[ia].x + maxProj * ux;
                        seg.y2 = points[ia].y + maxProj * uy;
                        seg.length_mm = wallLen;
                        seg.point_count = inliers;

                        for (int k = 0; k < inliers; k++) {
                            usedInWall[inlierIdxs[k]] = true;
                        }

                        outWalls[wallCount++] = seg;
                        // Pokračujeme dál ať najdeme i druhou/třetí stěnu u stejné svíčky!
                    }
                }
            }
        }
    }

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
