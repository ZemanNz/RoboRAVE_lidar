#pragma once
#include <Arduino.h>
#include <math.h>
#include "d200_types.h"

// ============================================================
//  detector.h
//  Geometrická detekce na ESP32 (Kybernoid 27)
//
//  Přísná pravidla a inteligence:
//  1. Stěny mají tvrdý strop délky max 330 mm (cokoli > 330 mm je stěna místnosti/bordel).
//  2. Překonání stínu svíčky (stitching/spojování kolineárních částí stěny):
//     - Pokud je stěna v jedné přímce přerušena svíčkou na dva kusy,
//       algoritmus je spojí do jedné plné 30cm stěny.
//  3. Svíčky se hledají jako první (max 4 ks).
//  4. Bodování stěn (scoring):
//     - Ideální rozměr 270 až 320 mm
//     - Kolmost (pravé úhly 90° rohových/U-boxů)
//     - Přítomnost svíčky před/uprostřed stěny
//  5. Do výstupu jde maximálně 6 nejlepších stěn.
// ============================================================

struct WallParams {
    float expected_length_mm = 300.0f;
    float max_length_mm = 330.0f;       // TVRDÝ STROP: nad 330 mm to NENÍ herní stěna!
    float min_length_mm = 160.0f;       // Minimální délka pro dílčí úsek (před spojením)
    float max_point_dist_mm = 70.0f;    // Max vzdálenost mezi sousedními body
    float max_line_deviation_mm = 24.0f; // Max odchylka bodu od proložené přímky
    int min_points = 4;                 // Minimální počet bodů
};

struct CandleParams {
    float min_diameter_mm = 25.0f;      // Min průměr clusteru
    float max_diameter_mm = 75.0f;      // Max průměr clusteru
    int min_points = 3;                 // Min 3 body
    int max_points = 8;                 // Max 8 bodů
    float max_neighbor_dist_mm = 45.0f; // Eukleidovská vzdálenost pro clustering
    float min_dist_to_wall_mm = 45.0f;
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
                float distSq = dx * dx + dy * dy;
                if (distSq <= (cParams.max_neighbor_dist_mm * cParams.max_neighbor_dist_mm)) {
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
        // 2. KROK: Hledání primárních segmentů stěn
        // ========================================================
        #define MAX_RAW_SEGMENTS 24
        static WallSegment rawSegs[MAX_RAW_SEGMENTS];
        int numRaw = 0;

        int seqStart = -1;
        for (int i = 0; i <= count; i++) {
            bool validPoint = (i < count) && points[i].valid && !usedInCandle[i];

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
                if (seqStart >= 0) {
                    int seqEnd = i - 1;
                    int seqLen = seqEnd - seqStart + 1;

                    if (seqLen >= wParams.min_points && numRaw < MAX_RAW_SEGMENTS) {
                        extractRawWall(points, seqStart, seqEnd, wParams, rawSegs, numRaw);
                    }
                    seqStart = validPoint ? i : -1;
                }
            }
        }

        // ========================================================
        // 3. KROK: Spojování kolineárních částí stěny přerušených svíčkou
        // ========================================================
        static WallSegment mergedSegs[MAX_RAW_SEGMENTS];
        bool mergedUsed[MAX_RAW_SEGMENTS];
        int numMerged = 0;
        for (int i = 0; i < numRaw; i++) mergedUsed[i] = false;

        for (int i = 0; i < numRaw; i++) {
            if (mergedUsed[i]) continue;
            WallSegment cur = rawSegs[i];

            for (int j = i + 1; j < numRaw; j++) {
                if (mergedUsed[j]) continue;
                const WallSegment& other = rawSegs[j];

                // Zjistíme, zda cur a other leží ve stejné přímce a jejich celková délka by byla <= 330 mm
                // 4 kombinace koncových bodů:
                float p1x = cur.x1, p1y = cur.y1;
                float p2x = other.x2, p2y = other.y2;
                float spanLen = hypotf(p2x - p1x, p2y - p1y);

                if (spanLen >= 240.0f && spanLen <= wParams.max_length_mm) {
                    // Kontrola kolinearity: vzdálenost vnitřních bodů od přímky p1->p2
                    float ldx = p2x - p1x;
                    float ldy = p2y - p1y;
                    float dev1 = fabsf(ldy * cur.x2 - ldx * cur.y2 + p2x * p1y - p2y * p1x) / spanLen;
                    float dev2 = fabsf(ldy * other.x1 - ldx * other.y1 + p2x * p1y - p2y * p1x) / spanLen;

                    if (dev1 < 25.0f && dev2 < 25.0f) {
                        // Kolineární a tvoří jednu 30cm stěnu přerušenou svíčkou!
                        cur.x1 = p1x; cur.y1 = p1y;
                        cur.x2 = p2x; cur.y2 = p2y;
                        cur.length_mm = spanLen;
                        cur.point_count += other.point_count;
                        mergedUsed[j] = true;
                        break;
                    }
                }
            }

            if (cur.length_mm <= wParams.max_length_mm && numMerged < MAX_RAW_SEGMENTS) {
                mergedSegs[numMerged++] = cur;
            }
        }

        // ========================================================
        // 4. KROK: Skórování a výběr maximálně 6 herních stěn
        // ========================================================
        if (numMerged <= MAX_DETECTED_WALLS) {
            for (int i = 0; i < numMerged; i++) {
                outWalls[i] = mergedSegs[i];
            }
            wallCount = numMerged;
        } else {
            float scores[MAX_RAW_SEGMENTS];
            for (int i = 0; i < numMerged; i++) {
                float score = 0.0f;
                const WallSegment& w = mergedSegs[i];

                // 1. Shoda s přesným rozměrem 300 mm (tvrdý strop 330 mm)
                float diff = fabsf(w.length_mm - 300.0f);
                if (w.length_mm <= 330.0f) {
                    score += fmaxf(0.0f, (60.0f - diff) * 3.0f); // až +180 bodů za přesně 300 mm
                }

                float mx = (w.x1 + w.x2) * 0.5f;
                float my = (w.y1 + w.y2) * 0.5f;
                float wdx = w.x2 - w.x1;
                float wdy = w.y2 - w.y1;
                float wlen = w.length_mm;
                if (wlen > 1.0f) { wdx /= wlen; wdy /= wlen; }

                // 2. Bonus za svíčku před / uprostřed stěny (< 380 mm)
                for (int c = 0; c < candleCount; c++) {
                    float cdist = hypotf(outCandles[c].x - mx, outCandles[c].y - my);
                    if (cdist < 380.0f) {
                        score += (380.0f - cdist) * 1.0f; // až +380 bodů!
                    }
                }

                // 3. Bonus za pravý úhel (kolmost 90° na jinou stěnu poblíž konce)
                for (int j = 0; j < numMerged; j++) {
                    if (i == j) continue;
                    const WallSegment& w2 = mergedSegs[j];
                    float w2dx = w2.x2 - w2.x1;
                    float w2dy = w2.y2 - w2.y1;
                    float w2len = w2.length_mm;
                    if (w2len > 1.0f) {
                        w2dx /= w2len; w2dy /= w2len;
                        float dot = fabsf(wdx * w2dx + wdy * w2dy);
                        if (dot < 0.22f) { // úhel cca 77° až 103° (kolmé)
                            float d1 = hypotf(w.x1 - w2.x1, w.y1 - w2.y1);
                            float d2 = hypotf(w.x1 - w2.x2, w.y1 - w2.y2);
                            float d3 = hypotf(w.x2 - w2.x1, w.y2 - w2.y1);
                            float d4 = hypotf(w.x2 - w2.x2, w.y2 - w2.y2);
                            float minEndDist = fminf(fminf(d1, d2), fminf(d3, d4));
                            if (minEndDist < 180.0f) {
                                score += 200.0f; // Velký bonus za pravý roh!
                            }
                        }
                    }
                }

                scores[i] = score;
            }

            // Vybereme TOP 6 nejlepších stěn
            bool picked[MAX_RAW_SEGMENTS];
            for (int i = 0; i < numMerged; i++) picked[i] = false;

            for (int k = 0; k < MAX_DETECTED_WALLS && k < numMerged; k++) {
                int bestIdx = -1;
                float bestScore = -9999.0f;
                for (int i = 0; i < numMerged; i++) {
                    if (!picked[i] && scores[i] > bestScore) {
                        bestScore = scores[i];
                        bestIdx = i;
                    }
                }
                if (bestIdx >= 0) {
                    picked[bestIdx] = true;
                    outWalls[wallCount++] = mergedSegs[bestIdx];
                }
            }
        }
    }

private:
    static void extractRawWall(const LidarPoint* points, int startIdx, int endIdx,
                               const WallParams& wp,
                               WallSegment* outWalls, int& wallCount)
    {
        if (wallCount >= MAX_RAW_SEGMENTS) return;

        float x1 = points[startIdx].x;
        float y1 = points[startIdx].y;
        float x2 = points[endIdx].x;
        float y2 = points[endIdx].y;

        float dx = x2 - x1;
        float dy = y2 - y1;
        float len = sqrtf(dx * dx + dy * dy);

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

        // Pokud je odchylka malá a délka nepřesahuje tvrdý strop 330 mm
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
            // Rozdělení v bodě zlomu (roh 90° nebo překážka)
            extractRawWall(points, startIdx, maxDevIdx, wp, outWalls, wallCount);
            extractRawWall(points, maxDevIdx, endIdx, wp, outWalls, wallCount);
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
