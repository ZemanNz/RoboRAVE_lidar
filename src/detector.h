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
    float max_length_mm = 360.0f;        // Tolerance: deska 300 mm se šikmým odrazem měří až 340-350 mm
    float min_length_mm = 90.0f;         // Zmírněno pro zachycení i stíněného křídla za svíčkou
    float max_point_dist_mm = 125.0f;    // Zvýšeno pro překlenutí stínu svíčky (cca 90-110 mm)
    float max_line_deviation_mm = 25.0f; // Max odchylka bodu od přímky
    int min_points = 3;                  // Stačí 3 body pro křídlo
};

struct CandleParams {
    float min_diameter_mm = 12.0f;       // Zmírněno na 12 mm pro spolehlivé zachycení i 2-bodových clusterů
    float max_diameter_mm = 100.0f;      // Zvýšeno na 100 mm pro spolehlivou detekci zblízka (20-40 cm)
    int min_points = 2;
    int max_points = 24;                 // Zvýšeno na 24 pro spolehlivou detekci zblízka (20-40 cm)
    float max_neighbor_dist_mm = 55.0f;
    float min_dist_to_wall_mm = 40.0f;
    float min_candle_separation_mm = 800.0f; // Minimální vzdálenost mezi dvěma svíčkami
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

            // Vybereme body v okolí svíčky (60 až 420 mm od svíčky)
            int localIdxs[100];
            int localCount = 0;
            for (int i = 0; i < count && localCount < 100; i++) {
                if (!points[i].valid || usedInCandle[i]) continue;
                float d = hypotf(points[i].x - cx, points[i].y - cy);
                if (d >= 60.0f && d <= 420.0f) {
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
        // 3.5 KROK: Protažení stěn a hledání inlierů k dosažení plné délky 300 mm
        // Pro každého kandidáta prozkoumáme nosnou přímku před i za stěnou (do 330 mm)
        // a začleníme veškeré zapadající body ležící na stejné přímce.
        // ========================================================
        for (int i = 0; i < numCandidates; i++) {
            extendWallWithInliers(candidates[i], points, count, usedInCandle, usedInWall, wParams);
        }

        // ========================================================
        // ========================================================
        // 4. KROK: Eliminace duplicitních překryvů a výběr TOP 6
        // ========================================================
        static bool isPerpCandle[MAX_DETECTED_CANDLES];
        for (int c = 0; c < MAX_DETECTED_CANDLES; c++) isPerpCandle[c] = false;

        // Odstranění stěn, které se navzájem téměř kryjí
        static WallSegment uniqueWalls[MAX_CANDIDATES];
        int numUnique = 0;

        for (int i = 0; i < numCandidates; i++) {
            WallSegment w1 = candidates[i];
            float perpDist = 0.0f, projT = 0.0f;
            bool hasPerp = checkCandleOnPerpendicular(w1, points, count,
                                                     outCandles, candleCount,
                                                     usedInCandle, usedInWall,
                                                     isPerpCandle,
                                                     &perpDist, &projT);

            // Pokud stěna má svíčku na své kolmici, postačí délka 75 mm (jinak standardně 130 mm)
            float minAllowedLen = hasPerp ? 75.0f : 130.0f;
            if (w1.length_mm > wParams.max_length_mm || w1.length_mm < minAllowedLen) continue;

            // Pokud má svíčku na kolmici a ještě nedosahuje 300 mm, dotáhneme ji symetricky na plných 300 mm
            if (hasPerp && w1.length_mm < 300.0f) {
                float dx = w1.x2 - w1.x1;
                float dy = w1.y2 - w1.y1;
                float invL = 1.0f / w1.length_mm;
                float ux = dx * invL;
                float uy = dy * invL;
                float pad = (300.0f - w1.length_mm) * 0.5f;
                w1.x1 -= pad * ux;
                w1.y1 -= pad * uy;
                w1.x2 += pad * ux;
                w1.y2 += pad * uy;
                w1.length_mm = 300.0f;
            }

            float m1x = (w1.x1 + w1.x2) * 0.5f;
            float m1y = (w1.y1 + w1.y2) * 0.5f;
            bool duplicate = false;

            for (int j = 0; j < numUnique; j++) {
                const WallSegment& w2 = uniqueWalls[j];
                float m2x = (w2.x1 + w2.x2) * 0.5f;
                float m2y = (w2.y1 + w2.y2) * 0.5f;
                if (hypotf(m1x - m2x, m1y - m2y) < 70.0f) {
                    duplicate = true;
                    // Preferujeme stěnu se svíčkou na kolmici, případně s délkou bližší 300 mm
                    bool w2HasPerp = checkCandleOnPerpendicular(w2, points, count,
                                                                outCandles, candleCount,
                                                                usedInCandle, usedInWall);
                    if ((hasPerp && !w2HasPerp) ||
                        (hasPerp == w2HasPerp && fabsf(w1.length_mm - 300.0f) < fabsf(w2.length_mm - 300.0f))) {
                        uniqueWalls[j] = w1;
                    }
                    break;
                }
            }

            if (!duplicate && numUnique < MAX_CANDIDATES) {
                uniqueWalls[numUnique++] = w1;
            }
        }

        // Finální doladění unikátních stěn (opětovné protažení na 300 mm se všemi inliery)
        for (int i = 0; i < numUnique; i++) {
            extendWallWithInliers(uniqueWalls[i], points, count, usedInCandle, usedInWall, wParams);
        }

        // Skórování a výběr max MAX_DETECTED_WALLS
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

            // 2. Svíčka před/uprostřed stěny (vzdálenost od středu)
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

            // 4. PRAVIDLO KOLMICE: Svíčka stojící přímo na kolmici stěny
            // Pokud má kandidát na stěnu svíčku na své kolmici (např. cca 50-280 mm před tělem stěny),
            // dáváme mu masivní prioritu (+400 b.) před náhodnými zdmi místnosti.
            float perpDist = 0.0f;
            float projT = 0.0f;
            if (checkCandleOnPerpendicular(w, points, count,
                                           outCandles, candleCount,
                                           usedInCandle, usedInWall,
                                           isPerpCandle,
                                           &perpDist, &projT)) {
                score += 400.0f;
                float centerOffset = fabsf(projT - w.length_mm * 0.5f);
                score += fmaxf(0.0f, (120.0f - centerOffset) * 2.0f);
            }

            scores[i] = score;
        }

        // Seřazení podle skóre a výběr nejlepších nekonfliktních stěn (max MAX_DETECTED_WALLS)
        // PRAVIDLO: Stěny se nesmí překrývat! Výjimka: pouze pokud jsou na sebe velice kolmé (70° až 110°).
        bool picked[MAX_CANDIDATES];
        for (int i = 0; i < numUnique; i++) picked[i] = false;

        for (int k = 0; k < numUnique && wallCount < MAX_DETECTED_WALLS; k++) {
            int bestIdx = -1;
            float bestScore = -9999.0f;
            for (int i = 0; i < numUnique; i++) {
                if (!picked[i] && scores[i] > bestScore) {
                    bestScore = scores[i];
                    bestIdx = i;
                }
            }
            if (bestIdx < 0 || bestScore <= 0.0f) break;
            picked[bestIdx] = true;

            const WallSegment& candWall = uniqueWalls[bestIdx];
            bool conflict = false;
            for (int j = 0; j < wallCount; j++) {
                if (areWallsConflicting(candWall, outWalls[j])) {
                    conflict = true;
                    break;
                }
            }
            if (!conflict) {
                outWalls[wallCount++] = candWall;
            }
        }

        // ========================================================
        // 4.5 KROK: DETEKCE ROHŮ ZE DVOU KOLMÝCH STĚN (L-ROHY)
        // A ZÁCHRANA/EXTRAKCE ROHOVÉ SVÍČKY (PRŮSEČÍK KOLMIC 2 STĚN)
        // V pravidlech Kybernoidu stojí svíčka v 90° rohu ze dvou stěn.
        // Hledáme L-rohy, spočteme průsečík nosných přímek a prozkoumáme kapse rohu.
        // ========================================================
        static bool isCornerCandle[MAX_DETECTED_CANDLES];
        for (int c = 0; c < MAX_DETECTED_CANDLES; c++) isCornerCandle[c] = false;

        extractCornerCandles(points, count, outWalls, wallCount,
                              outCandles, candleCount, isCornerCandle);

        // ========================================================
        // 4.6 KROK: DOPLNĚNÍ SVÍČKY PRO KAŽDOU SAMOSTATNOU STĚNU (~300 mm)
        // Každá překážková deska 300 mm na hřišti kryje svíčku. Pokud u sebe dosud žádnou nemá,
        // vyhledáme body nebo syntetizujeme svíčku 130 mm před jejím středem směrem k robotu.
        // ========================================================
        for (int w = 0; w < wallCount && candleCount < MAX_DETECTED_CANDLES; w++) {
            const WallSegment& wall = outWalls[w];
            if (wall.length_mm < 240.0f || wall.length_mm > 350.0f) continue;

            float wmx = (wall.x1 + wall.x2) * 0.5f;
            float wmy = (wall.y1 + wall.y2) * 0.5f;

            bool hasCandle = false;
            for (int c = 0; c < candleCount; c++) {
                if (hypotf(outCandles[c].x - wmx, outCandles[c].y - wmy) <= 260.0f) {
                    hasCandle = true;
                    break;
                }
            }

            if (!hasCandle) {
                float dx = wall.x2 - wall.x1;
                float dy = wall.y2 - wall.y1;
                float invL = 1.0f / wall.length_mm;
                float ux = dx * invL;
                float uy = dy * invL;
                float nx = -uy;
                float ny = ux;
                // Normála směrem k robotovi (0, 0)
                if ((wmx * nx + wmy * ny) > 0.0f) {
                    nx = -nx;
                    ny = -ny;
                }

                float sumX = 0, sumY = 0;
                int inliers = 0;
                for (int i = 0; i < count; i++) {
                    if (!points[i].valid || usedInCandle[i] || usedInWall[i]) continue;
                    float d = hypotf(points[i].x - (wmx + 130.0f * nx), points[i].y - (wmy + 130.0f * ny));
                    if (d < 65.0f) {
                        sumX += points[i].x;
                        sumY += points[i].y;
                        inliers++;
                    }
                }

                Candle sc;
                if (inliers >= 2) {
                    sc.x = sumX / (float)inliers;
                    sc.y = sumY / (float)inliers;
                    sc.point_count = inliers;
                } else {
                    sc.x = wmx + 130.0f * nx;
                    sc.y = wmy + 130.0f * ny;
                    sc.point_count = 0;
                }
                sc.diameter_mm = 50.0f;
                isPerpCandle[candleCount] = true;
                outCandles[candleCount++] = sc;
            }
        }

        // ========================================================
        // 5. KROK: Validace svíček vůči stěnám a ohodnocení vazby ke stěnám
        // PRAVIDLA:
        // 1. Svíčky se stěnou mají masivní přednost před osamocenými shluky bodů.
        // 2. Maximálně JEDNA svíčka bez stěny (ta před námi; druhá bez stěny je vyloučena).
        // 3. Svíčky od sebe musí být vzdáleny alespoň 80 cm (800 mm).
        // ========================================================
        static Candle candPool[MAX_DETECTED_CANDLES * 2];
        static float candScores[MAX_DETECTED_CANDLES * 2];
        static int candNearbyWalls[MAX_DETECTED_CANDLES * 2];
        int poolCount = 0;

        for (int c = 0; c < candleCount && poolCount < (MAX_DETECTED_CANDLES * 2); c++) {
            const Candle& cand = outCandles[c];

            bool inWallPath = false;
            int wallsNearby = 0;
            float closestWallDist = 9999.0f;

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

                // Zakázaný koridor těla stěny (nesmí ležet přímo v materiálu)
                if (!isCornerCandle[c] && !isPerpCandle[c] && distToLine <= 55.0f && projT >= -150.0f && projT <= (wLen + 150.0f)) {
                    inWallPath = true;
                    break;
                }

                // Vzdálenost středu svíčky k úsečce stěny
                float tClamped = fmaxf(0.0f, fminf(wLen, projT));
                float closeX = wall.x1 + tClamped * ux;
                float closeY = wall.y1 + tClamped * uy;
                float distToSeg = hypotf(cand.x - closeX, cand.y - closeY);

                if (distToSeg < closestWallDist) {
                    closestWallDist = distToSeg;
                }

                // Poloměr 25 cm (250 mm)
                if (distToSeg <= 250.0f) {
                    wallsNearby++;
                }
            }

            if (inWallPath) continue;

            if (isCornerCandle[c] && wallsNearby < 2) wallsNearby = 2;
            if (isPerpCandle[c] && wallsNearby < 1) wallsNearby = 1;

            float score = (float)wallsNearby * 2000.0f;
            if (wallsNearby > 0 && closestWallDist < 250.0f) {
                score += (250.0f - closestWallDist);
            }
            score += (float)cand.point_count * 5.0f;

            candPool[poolCount] = cand;
            candScores[poolCount] = score;
            candNearbyWalls[poolCount] = wallsNearby;
            poolCount++;
        }

        // Seřazení podle skóre
        for (int a = 0; a < poolCount - 1; a++) {
            for (int b = a + 1; b < poolCount; b++) {
                if (candScores[b] > candScores[a]) {
                    Candle cTmp = candPool[a]; candPool[a] = candPool[b]; candPool[b] = cTmp;
                    float sTmp = candScores[a]; candScores[a] = candScores[b]; candScores[b] = sTmp;
                    int wTmp = candNearbyWalls[a]; candNearbyWalls[a] = candNearbyWalls[b]; candNearbyWalls[b] = wTmp;
                }
            }
        }

        // Výběr: max 1 svíčka bez stěny a rozestup 80 cm
        static Candle filteredCandles[MAX_DETECTED_CANDLES];
        int filteredCount = 0;
        int isolatedCount = 0;
        float minDistSq = cParams.min_candle_separation_mm * cParams.min_candle_separation_mm;

        for (int i = 0; i < poolCount && filteredCount < MAX_DETECTED_CANDLES; i++) {
            bool isIsolated = (candNearbyWalls[i] == 0);
            if (isIsolated && isolatedCount >= 1) {
                continue; // Další svíčka bez stěny je zakázána!
            }

            bool tooClose = false;
            for (int j = 0; j < filteredCount; j++) {
                float ddx = candPool[i].x - filteredCandles[j].x;
                float ddy = candPool[i].y - filteredCandles[j].y;
                if ((ddx * ddx + ddy * ddy) < minDistSq) {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose) {
                if (isIsolated) isolatedCount++;
                filteredCandles[filteredCount++] = candPool[i];
            }
        }

        for (int c = 0; c < filteredCount; c++) {
            outCandles[c] = filteredCandles[c];
        }
        candleCount = filteredCount;

        // ========================================================
        // 6. KROK: Prioritizace svíčky v dopředném záměrném výseku (+-10° dopředu)
        // Svíčka v tomto 20° výseku je primárním startovním cílem robota -> zařadíme ji jako #1
        // ========================================================
        int targetIdx = -1;
        float minAngleDiff = 999.0f;
        for (int c = 0; c < candleCount; c++) {
            if (outCandles[c].y > 0.0f) {
                float angDeg = fabsf(atan2f(outCandles[c].x, outCandles[c].y) * (180.0f / M_PI));
                if (angDeg <= 10.0f && angDeg < minAngleDiff) {
                    minAngleDiff = angDeg;
                    targetIdx = c;
                }
            }
        }
        if (targetIdx > 0) {
            Candle tmp = outCandles[0];
            outCandles[0] = outCandles[targetIdx];
            outCandles[targetIdx] = tmp;
        }
    }

private:
    // Kontrola kolizí a překryvu dvou stěn:
    // Pravidlo: Stěny se nesmí překrývat ani křížit! Výjimka: pouze pokud jsou na sebe velice kolmé (70° až 110°).
    static bool areWallsConflicting(const WallSegment& a, const WallSegment& b) {
        if (a.length_mm < 10.0f || b.length_mm < 10.0f) return false;

        float adx = (a.x2 - a.x1) / a.length_mm;
        float ady = (a.y2 - a.y1) / a.length_mm;
        float bdx = (b.x2 - b.x1) / b.length_mm;
        float bdy = (b.y2 - b.y1) / b.length_mm;

        float amx = (a.x1 + a.x2) * 0.5f;
        float amy = (a.y1 + a.y2) * 0.5f;
        float bmx = (b.x1 + b.x2) * 0.5f;
        float bmy = (b.y1 + b.y2) * 0.5f;

        float centerDist = hypotf(amx - bmx, amy - bmy);

        // |dot| blízko 0 = cca 90° (kolmé), |dot| blízko 1 = rovnoběžné
        float dot = fabsf(adx * bdx + ady * bdy);

        // Kolmé stěny (cca 70° až 110° -> cos(70°) = 0.342)
        bool isPerpendicular = (dot <= 0.35f);

        if (isPerpendicular) {
            // Kolmé stěny tvoří rohy L nebo U-boxy, dotýkají se v rozích.
            // Konflikt by byl pouze, pokud by jejich středy ležely téměř na sobě (< 70 mm)
            return (centerDist < 70.0f);
        }

        // Pokud stěny NEJSOU kolmé (jsou rovnoběžné, šikmé nebo pod úhlem např. 40-60°):
        // NESMÍ SE PŘEKRÝVAT!
        if (centerDist < 200.0f) return true;

        // Vzdálenost středu B od nosné přímky A a podélný překryv
        float perpDistB = fabsf((bmx - a.x1) * (-ady) + (bmy - a.y1) * (adx));
        float projLongB = (bmx - a.x1) * adx + (bmy - a.y1) * ady;
        if (perpDistB < 150.0f && projLongB >= -60.0f && projLongB <= (a.length_mm + 60.0f)) {
            return true;
        }

        // Vzdálenost středu A od nosné přímky B a podélný překryv
        float perpDistA = fabsf((amx - b.x1) * (-bdy) + (amy - b.y1) * (bdx));
        float projLongA = (amx - b.x1) * bdx + (amy - b.y1) * bdy;
        if (perpDistA < 150.0f && projLongA >= -60.0f && projLongA <= (b.length_mm + 60.0f)) {
            return true;
        }

        return false;
    }

    // Kontrola kolmice: Zda se před kandidátem na stěnu nachází svíčka (bod / shluk bodů)
    // na kolmici ze středu nebo těla stěny ve vzdálenosti cca 50 až 280 mm.
    // Ověřuje jak již detekované svíčky v outCandles, tak případné surové body LiDARu.
    static bool checkCandleOnPerpendicular(const WallSegment& wall,
                                          const LidarPoint* points, int count,
                                          Candle* outCandles, int& candleCount,
                                          const bool* usedInCandle, const bool* usedInWall,
                                          bool* isPerpCandle = nullptr,
                                          float* outPerpDist = nullptr,
                                          float* outProjT = nullptr)
    {
        if (wall.length_mm < 30.0f) return false;

        float dx = wall.x2 - wall.x1;
        float dy = wall.y2 - wall.y1;
        float invL = 1.0f / wall.length_mm;
        float ux = dx * invL;
        float uy = dy * invL;
        float nx = -uy;
        float ny = ux;

        // 1. Zkontrolujeme, zda některá již detekovaná svíčka leží na kolmici této stěny
        for (int c = 0; c < candleCount; c++) {
            float vx = outCandles[c].x - wall.x1;
            float vy = outCandles[c].y - wall.y1;
            float proj = vx * ux + vy * uy;
            float perp = fabsf(vx * nx + vy * ny);

            // Svíčka stojí kolmo před stěnou (50 až 280 mm) v rozmezí délky stěny
            if (perp >= 50.0f && perp <= 280.0f &&
                proj >= -40.0f && proj <= (wall.length_mm + 40.0f) &&
                fabsf(proj - wall.length_mm * 0.5f) <= 140.0f) {
                if (outPerpDist) *outPerpDist = perp;
                if (outProjT) *outProjT = proj;
                if (isPerpCandle) isPerpCandle[c] = true;
                return true;
            }
        }

        // 2. Pokud žádná dosud detekovaná svíčka neleží na kolmici, prohledáme surové body LiDARu.
        // Hledáme malý shluk bodů (2 až 8 bodů) v zóně kolmice.
        float sumX = 0.0f, sumY = 0.0f;
        int rawInliers = 0;
        int rawIndices[32];

        for (int k = 0; k < count && rawInliers < 32; k++) {
            if (!points[k].valid || usedInWall[k]) continue;

            float vx = points[k].x - wall.x1;
            float vy = points[k].y - wall.y1;
            float proj = vx * ux + vy * uy;
            float perp = fabsf(vx * nx + vy * ny);

            if (perp >= 55.0f && perp <= 270.0f &&
                proj >= -30.0f && proj <= (wall.length_mm + 30.0f) &&
                fabsf(proj - wall.length_mm * 0.5f) <= 130.0f) {
                rawIndices[rawInliers++] = k;
                sumX += points[k].x;
                sumY += points[k].y;
            }
        }

        if (rawInliers >= 2 && rawInliers <= 24) {
            float avgX = sumX / (float)rawInliers;
            float avgY = sumY / (float)rawInliers;

            // Kontrola kompaktnosti shluku (svíčka má průměr 50 mm, body do cca 90 mm zblízka)
            float maxDistSq = 0.0f;
            for (int a = 0; a < rawInliers; a++) {
                int ia = rawIndices[a];
                for (int b = a + 1; b < rawInliers; b++) {
                    int ib = rawIndices[b];
                    float ddx = points[ia].x - points[ib].x;
                    float ddy = points[ia].y - points[ib].y;
                    float dsq = ddx * ddx + ddy * ddy;
                    if (dsq > maxDistSq) maxDistSq = dsq;
                }
            }

            if (maxDistSq <= (90.0f * 90.0f)) {
                if (outPerpDist) *outPerpDist = fabsf((avgX - wall.x1) * nx + (avgY - wall.y1) * ny);
                if (outProjT) *outProjT = (avgX - wall.x1) * ux + (avgY - wall.y1) * uy;

                // Zaregistrujeme novou svíčku, pokud ještě neexistuje
                bool alreadyExists = false;
                for (int c = 0; c < candleCount; c++) {
                    if (hypotf(outCandles[c].x - avgX, outCandles[c].y - avgY) < 80.0f) {
                        alreadyExists = true;
                        if (isPerpCandle) isPerpCandle[c] = true;
                        break;
                    }
                }

                if (!alreadyExists && candleCount < MAX_DETECTED_CANDLES) {
                    Candle nc;
                    nc.x = avgX;
                    nc.y = avgY;
                    nc.diameter_mm = 50.0f;
                    nc.point_count = rawInliers;
                    if (isPerpCandle) isPerpCandle[candleCount] = true;
                    outCandles[candleCount++] = nc;
                }
                return true;
            }
        }

        return false;
    }

    // Prodloužení stěny podél její nosné přímky a pohlcení zapadajících bodů
    // Cílem je dosáhnout plné nominální délky 300 mm (max 330 mm) překlenutím mezer a výpadků
    static void extendWallWithInliers(WallSegment& wall, const LidarPoint* points, int count,
                                      const bool* usedInCandle, bool* usedInWall,
                                      const WallParams& wp)
    {
        if (wall.length_mm < 10.0f) return;

        float dx = wall.x2 - wall.x1;
        float dy = wall.y2 - wall.y1;
        float invL = 1.0f / wall.length_mm;
        float ux = dx * invL;
        float uy = dy * invL;
        float nx = -uy;
        float ny = ux;

        struct ProjPoint {
            float proj;
            int idx;
        };
        const int MAX_LINE_PTS = 120;
        ProjPoint linePoints[MAX_LINE_PTS];
        int numLinePoints = 0;

        // Prohledáme všechny platné body v koridoru prodloužené přímky stěny
        for (int i = 0; i < count && numLinePoints < MAX_LINE_PTS; i++) {
            if (!points[i].valid || usedInCandle[i]) continue;

            float vx = points[i].x - wall.x1;
            float vy = points[i].y - wall.y1;

            float dev = fabsf(vx * nx + vy * ny);
            if (dev <= wp.max_line_deviation_mm) {
                float proj = vx * ux + vy * uy;
                // Hledáme v dosahu až 180 mm před začátkem i za koncem stěny
                if (proj >= -180.0f && proj <= (wall.length_mm + 180.0f)) {
                    linePoints[numLinePoints].proj = proj;
                    linePoints[numLinePoints].idx = i;
                    numLinePoints++;
                }
            }
        }

        if (numLinePoints < 2) return;

        // Seřazení podle projekce podél osy stěny (insertion sort)
        for (int a = 0; a < numLinePoints - 1; a++) {
            for (int b = a + 1; b < numLinePoints; b++) {
                if (linePoints[b].proj < linePoints[a].proj) {
                    ProjPoint tmp = linePoints[a];
                    linePoints[a] = linePoints[b];
                    linePoints[b] = tmp;
                }
            }
        }

        // Najdeme body tvořící původní stěnu
        int coreMinIdx = -1, coreMaxIdx = -1;
        for (int k = 0; k < numLinePoints; k++) {
            if (linePoints[k].proj >= -25.0f && linePoints[k].proj <= (wall.length_mm + 25.0f)) {
                if (coreMinIdx < 0) coreMinIdx = k;
                coreMaxIdx = k;
            }
        }

        if (coreMinIdx < 0) return;

        // Rozšiřujeme dozadu (směrem před začátek wall.x1)
        int bestStart = coreMinIdx;
        while (bestStart > 0) {
            float gap = linePoints[bestStart].proj - linePoints[bestStart - 1].proj;
            float totalSpan = linePoints[coreMaxIdx].proj - linePoints[bestStart - 1].proj;
            if (gap <= wp.max_point_dist_mm && totalSpan <= wp.max_length_mm) {
                bestStart--;
            } else {
                break;
            }
        }

        // Rozšiřujeme dopředu (směrem za konec wall.x2)
        int bestEnd = coreMaxIdx;
        while (bestEnd < numLinePoints - 1) {
            float gap = linePoints[bestEnd + 1].proj - linePoints[bestEnd].proj;
            float totalSpan = linePoints[bestEnd + 1].proj - linePoints[bestStart].proj;
            if (gap <= wp.max_point_dist_mm && totalSpan <= wp.max_length_mm) {
                bestEnd++;
            } else {
                break;
            }
        }

        float newMinProj = linePoints[bestStart].proj;
        float newMaxProj = linePoints[bestEnd].proj;
        float newLen = newMaxProj - newMinProj;

        if (newLen >= wall.length_mm && newLen <= wp.max_length_mm) {
            float origX1 = wall.x1;
            float origY1 = wall.y1;
            wall.x1 = origX1 + newMinProj * ux;
            wall.y1 = origY1 + newMinProj * uy;
            wall.x2 = origX1 + newMaxProj * ux;
            wall.y2 = origY1 + newMaxProj * uy;
            wall.length_mm = newLen;
            wall.point_count = (bestEnd - bestStart + 1);

            for (int k = bestStart; k <= bestEnd; k++) {
                usedInWall[linePoints[k].idx] = true;
            }

            // Pokud stěna zachytila body (např. obě křídla kolem stínu svíčky), ale délka je 220-295 mm,
            // protáhneme ji podél osy symetricky na plnou nominální délku 300 mm
            if (wall.length_mm >= 210.0f && wall.length_mm < 300.0f) {
                float pad = (300.0f - wall.length_mm) * 0.5f;
                wall.x1 -= pad * ux;
                wall.y1 -= pad * uy;
                wall.x2 += pad * ux;
                wall.y2 += pad * uy;
                wall.length_mm = 300.0f;
            }
        }
    }

    // Detekce rohů tvořených dvěma na sebe kolmými stěnami (L-rohy) a záchrana rohové svíčky
    // Využívá průsečík kolmic obou stěn pro přesnou lokalizaci svíčky
    static void extractCornerCandles(const LidarPoint* points, int count,
                                     const WallSegment* walls, int wallCount,
                                     Candle* outCandles, int& candleCount,
                                     bool* isCornerCandle)
    {
        for (int i = 0; i < wallCount; i++) {
            for (int j = i + 1; j < wallCount; j++) {
                const WallSegment& w1 = walls[i];
                const WallSegment& w2 = walls[j];
                if (w1.length_mm < 130.0f || w2.length_mm < 130.0f) continue;

                float u1x = (w1.x2 - w1.x1) / w1.length_mm;
                float u1y = (w1.y2 - w1.y1) / w1.length_mm;
                float u2x = (w2.x2 - w2.x1) / w2.length_mm;
                float u2y = (w2.y2 - w2.y1) / w2.length_mm;

                // 1. Kontrola kolmosti (skalární součin u1 a u2 kolem 0, tolerance cca +-25°)
                float dot = fabsf(u1x * u2x + u1y * u2y);
                if (dot > 0.42f) continue;

                // 2. Vzdálenost nejbližších konců obou stěn (musí tvořit roh)
                float d11 = hypotf(w1.x1 - w2.x1, w1.y1 - w2.y1);
                float d12 = hypotf(w1.x1 - w2.x2, w1.y1 - w2.y2);
                float d21 = hypotf(w1.x2 - w2.x1, w1.y2 - w2.y1);
                float d22 = hypotf(w1.x2 - w2.x2, w1.y2 - w2.y2);
                float minEndDist = fminf(fminf(d11, d12), fminf(d21, d22));
                if (minEndDist > 250.0f) continue;

                // 3. Průsečík nosných přímek obou stěn (Corner Point Pc)
                float a1 = -u1y, b1 = u1x, c1 = -(a1 * w1.x1 + b1 * w1.y1);
                float a2 = -u2y, b2 = u2x, c2 = -(a2 * w2.x1 + b2 * w2.y1);
                float det = a1 * b2 - a2 * b1;
                if (fabsf(det) < 0.1f) continue;

                float pcX = (b1 * c2 - b2 * c1) / det;
                float pcY = (c1 * a2 - c2 * a1) / det;

                // 4. Ramena obou stěn směrem OD rohu do pole
                float distW1_1 = hypotf(w1.x1 - pcX, w1.y1 - pcY);
                float distW1_2 = hypotf(w1.x2 - pcX, w1.y2 - pcY);
                float farW1X = (distW1_1 > distW1_2) ? w1.x1 : w1.x2;
                float farW1Y = (distW1_1 > distW1_2) ? w1.y1 : w1.y2;
                float lenV1 = hypotf(farW1X - pcX, farW1Y - pcY);
                if (lenV1 < 30.0f) continue;
                float arm1X = (farW1X - pcX) / lenV1;
                float arm1Y = (farW1Y - pcY) / lenV1;

                float distW2_1 = hypotf(w2.x1 - pcX, w2.y1 - pcY);
                float distW2_2 = hypotf(w2.x2 - pcX, w2.y2 - pcY);
                float farW2X = (distW2_1 > distW2_2) ? w2.x1 : w2.x2;
                float farW2Y = (distW2_1 > distW2_2) ? w2.y1 : w2.y2;
                float lenV2 = hypotf(farW2X - pcX, farW2Y - pcY);
                if (lenV2 < 30.0f) continue;
                float arm2X = (farW2X - pcX) / lenV2;
                float arm2Y = (farW2Y - pcY) / lenV2;

                // 5. Zkontrolujeme, zda v tomto rohu již máme svíčku z 1. kroku
                bool candleFound = false;
                for (int c = 0; c < candleCount; c++) {
                    float cdx = outCandles[c].x - pcX;
                    float cdy = outCandles[c].y - pcY;
                    float cdist = hypotf(cdx, cdy);
                    if (cdist < 350.0f) {
                        float ct1 = cdx * arm1X + cdy * arm1Y;
                        float ct2 = cdx * arm2X + cdy * arm2Y;
                        if (ct1 > 20.0f && ct2 > 20.0f) {
                            isCornerCandle[c] = true;
                            candleFound = true;
                            break;
                        }
                    }
                }

                if (candleFound) continue;

                // 6. Prohledáme body LiDARu ve vnitřní rohové kapse (průsečík kolmic obou stěn)
                float sumX = 0.0f, sumY = 0.0f;
                int inlierCount = 0;

                for (int k = 0; k < count; k++) {
                    if (!points[k].valid) continue;
                    float px = points[k].x;
                    float py = points[k].y;

                    float rdx = px - pcX;
                    float rdy = py - pcY;
                    float rdist = hypotf(rdx, rdy);
                    if (rdist < 60.0f || rdist > 350.0f) continue;

                    // Projekce do vnitřního kvadrantu rohu
                    float t1 = rdx * arm1X + rdy * arm1Y;
                    float t2 = rdx * arm2X + rdy * arm2Y;
                    if (t1 < 20.0f || t2 < 20.0f) continue;

                    // Vzdálenost od nosných přímek stěn
                    float d1 = fabsf((px - w1.x1) * (-u1y) + (py - w1.y1) * (u1x));
                    float d2 = fabsf((px - w2.x1) * (-u2y) + (py - w2.y1) * (u2x));
                    if (d1 < 40.0f || d1 > 280.0f || d2 < 40.0f || d2 > 280.0f) continue;

                    sumX += px;
                    sumY += py;
                    inlierCount++;
                }

                // Pokud máme volné místo pro svíčku (max 4 svíčky na poli)
                if (candleCount < MAX_DETECTED_CANDLES) {
                    Candle nc;
                    if (inlierCount >= 1) {
                        nc.x = sumX / (float)inlierCount;
                        nc.y = sumY / (float)inlierCount;
                        nc.point_count = inlierCount;
                    } else {
                        // Geometrická syntéza z průsečíku kolmic stěn (střed rohu ~130 mm od stěn)
                        nc.x = pcX + 130.0f * arm1X + 130.0f * arm2X;
                        nc.y = pcY + 130.0f * arm1Y + 130.0f * arm2Y;
                        nc.point_count = 0;
                    }
                    nc.diameter_mm = 50.0f;
                    isCornerCandle[candleCount] = true;
                    outCandles[candleCount++] = nc;
                }
            }
        }
    }

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

                // Hledáme úseky, které by mohly tvořit kraje nebo ramena desky (cca 65 až 365 mm)
                if (distAB < 65.0f || distAB > 365.0f) continue;

                float invD = 1.0f / distAB;
                float nx = -dy * invD;
                float ny = dx * invD;
                float ux = dx * invD;
                float uy = dy * invD;

                // Spočteme body ležící na této přímce (inliery)
                int inliers = 0;
                float minProj = 1e9f, maxProj = -1e9f;
                int inlierIdxs[40];

                for (int k = 0; k < nIdxs; k++) {
                    int ik = idxs[k];
                    float dev = fabsf((points[ik].x - points[ia].x) * nx + (points[ik].y - points[ia].y) * ny);
                    if (dev <= wp.max_line_deviation_mm) {
                        float proj = (points[ik].x - points[ia].x) * ux + (points[ik].y - points[ia].y) * uy;
                        if (proj < minProj) minProj = proj;
                        if (proj > maxProj) maxProj = proj;
                        if (inliers < 40) inlierIdxs[inliers] = ik;
                        inliers++;
                    }
                }

                float wallLen = maxProj - minProj;

                // Kontrola polohy svíčky vůči této přímce (kolmice ze středu/těla stěny)
                float c_vx = cx - points[ia].x;
                float c_vy = cy - points[ia].y;
                float c_proj = c_vx * ux + c_vy * uy;
                float c_perp = fabsf(c_vx * nx + c_vy * ny);

                bool candleOnPerp = (c_perp >= 50.0f && c_perp <= 280.0f &&
                                     c_proj >= -40.0f && c_proj <= (wallLen + 40.0f));

                float minLenRequired = candleOnPerp ? 75.0f : 120.0f;
                int minInliersRequired = candleOnPerp ? 3 : 4;

                // Pokud inliery pokrývají požadovanou délku a nepřesahují max_length_mm
                if (inliers >= minInliersRequired && wallLen >= minLenRequired && wallLen <= wp.max_length_mm) {
                    if (wallCount < MAX_CANDIDATES) {
                        WallSegment seg;
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

                        // Pokud má stěna svíčku na kolmici a ještě nedosáhla 300 mm (např. 75-295 mm),
                        // protáhneme ji podél osy symetricky na plnou nominální délku 300 mm
                        if (seg.length_mm >= 75.0f && seg.length_mm < 300.0f) {
                            float pad = (300.0f - seg.length_mm) * 0.5f;
                            seg.x1 -= pad * ux;
                            seg.y1 -= pad * uy;
                            seg.x2 += pad * ux;
                            seg.y2 += pad * uy;
                            seg.length_mm = 300.0f;
                        }

                        outWalls[wallCount++] = seg;
                        if (candleOnPerp) return; // Nalezena ověřená stěna se svíčkou na kolmici
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

// ============================================================
//  BoxDetector - 2D prostorové shlukování na ESP32 (Svíčky + Stěny)
// ============================================================
class BoxDetector {
public:
    static void process(const LidarPoint* points, int count,
                        BoundingBox* outBoxes, int& boxCount,
                        float mergeDistMm = 250.0f,
                        int minPts = 3,
                        float maxAspect = 3.2f)
    {
        boxCount = 0;
        if (count < minPts) return;

        // Disjoint-set (Union-Find) pro propojení platných bodů
        static int parent[MAX_LIDAR_POINTS];
        for (int i = 0; i < count; i++) {
            parent[i] = i;
        }

        auto getRoot = [](int i, int* p) -> int {
            int root = i;
            while (p[root] != root) {
                root = p[root];
            }
            int curr = i;
            while (curr != root) {
                int nxt = p[curr];
                p[curr] = root;
                curr = nxt;
            }
            return root;
        };

        float mergeSq = mergeDistMm * mergeDistMm;

        // Spatial Grid pro rychlé 2D propojení O(N)
        // Vzhledem k rozsahu do 4 m stačí jednoduchý hashing
        for (int i = 0; i < count; i++) {
            if (!points[i].valid) continue;
            float xi = points[i].x;
            float yi = points[i].y;

            // Porovnáme s body vpřed (v úhlové posloupnosti)
            for (int j = i + 1; j < count; j++) {
                if (!points[j].valid) continue;
                float dx = points[j].x - xi;
                float dy = points[j].y - yi;
                if (dx * dx + dy * dy <= mergeSq) {
                    int ri = getRoot(i, parent);
                    int rj = getRoot(j, parent);
                    if (ri != rj) {
                        parent[ri] = rj;
                    }
                }
            }
        }

        // Extrakce shluků
        static BoundingBox candidates[16];
        int candCount = 0;

        static bool rootProcessed[MAX_LIDAR_POINTS];
        for (int i = 0; i < count; i++) rootProcessed[i] = false;

        for (int i = 0; i < count; i++) {
            if (!points[i].valid) continue;
            int r = getRoot(i, parent);
            if (rootProcessed[r]) continue;
            rootProcessed[r] = true;

            // Spočteme vlastnosti shluku s kořenem r
            float minX = 99999.0f, maxX = -99999.0f;
            float minY = 99999.0f, maxY = -99999.0f;
            float sumX = 0.0f, sumY = 0.0f;
            int k = 0;

            for (int j = 0; j < count; j++) {
                if (!points[j].valid) continue;
                if (getRoot(j, parent) == r) {
                    float px = points[j].x;
                    float py = points[j].y;
                    if (px < minX) minX = px;
                    if (px > maxX) maxX = px;
                    if (py < minY) minY = py;
                    if (py > maxY) maxY = py;
                    sumX += px;
                    sumY += py;
                    k++;
                }
            }

            if (k < minPts) continue;

            float w = maxX - minX;
            float h = maxY - minY;
            float maxSide = (w > h) ? w : h;
            float minSide = (w < h) ? w : h;
            float span = sqrtf(w * w + h * h);

            // Filtrace:
            // 1. Zdi arény / místnosti (> 650 mm) ZAHODIT
            if (span > 650.0f || maxSide > 620.0f) continue;
            // 2. Mikroskopický šum (< 20 mm) ZAHODIT
            if (span < 20.0f) continue;
            // 3. Poměr stran (pro k > 7 nesmí být zeď 1:10)
            float denom = (minSide > 18.0f) ? minSide : 18.0f;
            float aspect = maxSide / denom;
            if (k > 7 && aspect > maxAspect) continue;

            if (candCount < 16) {
                BoundingBox b;
                b.min_x = minX; b.max_x = maxX;
                b.min_y = minY; b.max_y = maxY;
                b.cx = sumX / (float)k;
                b.cy = sumY / (float)k;
                b.width = w;
                b.height = h;
                b.span_mm = span;
                b.dist_mm = sqrtf(b.cx * b.cx + b.cy * b.cy);
                b.aspect_ratio = aspect;
                b.point_count = k;
                b.is_solo_candle = (span < 140.0f);
                candidates[candCount++] = b;
            }
        }

        // Seřadit podle vzdálenosti (bližší mají přednost)
        for (int a = 0; a < candCount - 1; a++) {
            for (int b = a + 1; b < candCount; b++) {
                if (candidates[b].dist_mm < candidates[a].dist_mm) {
                    BoundingBox tmp = candidates[a];
                    candidates[a] = candidates[b];
                    candidates[b] = tmp;
                }
            }
        }

        // Vybrat max 4
        int limit = (candCount < MAX_DETECTED_BOXES) ? candCount : MAX_DETECTED_BOXES;
        for (int i = 0; i < limit; i++) {
            outBoxes[boxCount++] = candidates[i];
        }
    }
};

// ============================================================
//  TargetExtractor - Detailní geometrie cíle na ESP32 (Svíčka, Stěny, Nájezdy)
// ============================================================
class TargetExtractor {
public:
    static TargetDetail extract(const LidarPoint* points, int count, const BoundingBox& box) {
        TargetDetail target;
        memset(&target, 0, sizeof(target));
        target.box = box;
        target.is_visible = true;

        // Rezerva boxu výhradně pro hledání detailů uvnitř tohoto boxu (margin 25 mm)
        float search_margin = 25.0f;
        float bMinX = box.min_x - search_margin;
        float bMaxX = box.max_x + search_margin;
        float bMinY = box.min_y - search_margin;
        float bMaxY = box.max_y + search_margin;

        int boxIndices[120];
        int boxPtCount = 0;
        float sumX = 0.0f, sumY = 0.0f;
        for (int i = 0; i < count && boxPtCount < 120; i++) {
            if (!points[i].valid) continue;
            float px = points[i].x;
            float py = points[i].y;
            if (px >= bMinX && px <= bMaxX && py >= bMinY && py <= bMaxY) {
                boxIndices[boxPtCount++] = i;
                sumX += px;
                sumY += py;
            }
        }

        // Malý shluk nebo samostatná svíčka
        if (boxPtCount < 5 || box.span_mm < 160.0f || box.is_solo_candle) {
            target.type = TARGET_SOLO;
            target.candle_x = (boxPtCount > 0) ? (sumX / (float)boxPtCount) : box.cx;
            target.candle_y = (boxPtCount > 0) ? (sumY / (float)boxPtCount) : box.cy;
            target.wall_count = 0;

            float dLen = hypotf(target.candle_x, target.candle_y);
            float ux = (dLen > 10.0f) ? (target.candle_x / dLen) : 0.0f;
            float uy = (dLen > 10.0f) ? (target.candle_y / dLen) : 1.0f;

            target.waypoint_x = target.candle_x - ux * 380.0f;
            target.waypoint_y = target.candle_y - uy * 380.0f;
            target.approach_dx = ux;
            target.approach_dy = uy;
            target.travel_dist_mm = hypotf(target.waypoint_x, target.waypoint_y);
            return target;
        }

        // 1. SEPARACE BODŮ SVÍČKY OD STĚN (v12 princip)
        // Hledáme kompaktní shluk bodů svíčky (průměr <= 40 mm, 2 až 8 bodů)
        int bestCandCluster[20];
        int bestCandCount = 0;
        float bestCandX = 0.0f, bestCandY = 0.0f;

        for (int i = 0; i < boxPtCount; i++) {
            int pIdx1 = boxIndices[i];
            float px = points[pIdx1].x;
            float py = points[pIdx1].y;
            int tempCluster[20];
            int tempCount = 0;
            float cSumX = 0.0f, cSumY = 0.0f;

            for (int j = 0; j < boxPtCount && tempCount < 20; j++) {
                int pIdx2 = boxIndices[j];
                float d = hypotf(points[pIdx2].x - px, points[pIdx2].y - py);
                if (d <= 40.0f) {
                    tempCluster[tempCount++] = pIdx2;
                    cSumX += points[pIdx2].x;
                    cSumY += points[pIdx2].y;
                }
            }

            if (tempCount >= 2 && tempCount <= 8 && tempCount > bestCandCount) {
                bestCandCount = tempCount;
                for (int k = 0; k < tempCount; k++) bestCandCluster[k] = tempCluster[k];
                bestCandX = cSumX / (float)tempCount;
                bestCandY = cSumY / (float)tempCount;
            }
        }

        // Vyloučíme body svíčky z bodů pro hledání stěn!
        int wallIndices[120];
        int wallPtCount = 0;
        for (int i = 0; i < boxPtCount; i++) {
            int idx = boxIndices[i];
            bool isCandle = false;
            for (int k = 0; k < bestCandCount; k++) {
                if (bestCandCluster[k] == idx) { isCandle = true; break; }
            }
            if (!isCandle) wallIndices[wallPtCount++] = idx;
        }
        if (wallPtCount < 4) {
            // Pokud by nezbylo dost bodů pro stěny, použijeme všechny
            wallPtCount = boxPtCount;
            for (int i = 0; i < boxPtCount; i++) wallIndices[i] = boxIndices[i];
        }

        // 2. Hledání stěny 1 pomocí RANSAC POUZE ze stěnových bodů
        int bestW1Inliers[120];
        int bestW1Count = 0;
        int p1_idx = -1, p2_idx = -1;

        for (int i = 0; i < wallPtCount && i < 30; i++) {
            int idx1 = wallIndices[i];
            float x1 = points[idx1].x;
            float y1 = points[idx1].y;
            for (int j = i + 1; j < wallPtCount && j < 30; j++) {
                int idx2 = wallIndices[j];
                float x2 = points[idx2].x;
                float y2 = points[idx2].y;
                float d = hypotf(x2 - x1, y2 - y1);
                if (d < 75.0f) continue;

                float nx = -(y2 - y1) / d;
                float ny = (x2 - x1) / d;

                int inlCount = 0;
                int tempInl[120];
                for (int k = 0; k < wallPtCount; k++) {
                    int pIdx = wallIndices[k];
                    float dist = fabsf((points[pIdx].x - x1) * nx + (points[pIdx].y - y1) * ny);
                    if (dist <= 20.0f) {
                        tempInl[inlCount++] = pIdx;
                    }
                }
                if (inlCount > bestW1Count) {
                    bestW1Count = inlCount;
                    p1_idx = idx1;
                    p2_idx = idx2;
                    for (int k = 0; k < inlCount; k++) bestW1Inliers[k] = tempInl[k];
                }
            }
        }

        if (bestW1Count < 4) {
            target.type = TARGET_SOLO;
            target.candle_x = (bestCandCount > 0) ? bestCandX : ((boxPtCount > 0) ? (sumX / (float)boxPtCount) : box.cx);
            target.candle_y = (bestCandCount > 0) ? bestCandY : ((boxPtCount > 0) ? (sumY / (float)boxPtCount) : box.cy);
            target.candle_x = fmaxf(box.min_x, fminf(box.max_x, target.candle_x));
            target.candle_y = fmaxf(box.min_y, fminf(box.max_y, target.candle_y));
            target.wall_count = 0;
            float dLen = hypotf(target.candle_x, target.candle_y);
            float ux = (dLen > 10.0f) ? (target.candle_x / dLen) : 0.0f;
            float uy = (dLen > 10.0f) ? (target.candle_y / dLen) : 1.0f;
            target.waypoint_x = target.candle_x - ux * 380.0f;
            target.waypoint_y = target.candle_y - uy * 380.0f;
            target.approach_dx = ux;
            target.approach_dy = uy;
            target.travel_dist_mm = hypotf(target.waypoint_x, target.waypoint_y);
            return target;
        }

        float dx1 = points[p2_idx].x - points[p1_idx].x;
        float dy1 = points[p2_idx].y - points[p1_idx].y;
        float L1 = hypotf(dx1, dy1);
        float u1x = dx1 / L1;
        float u1y = dy1 / L1;

        float minT1 = 99999.0f, maxT1 = -99999.0f;
        for (int k = 0; k < bestW1Count; k++) {
            int pIdx = bestW1Inliers[k];
            float proj = (points[pIdx].x - points[p1_idx].x) * u1x + (points[pIdx].y - points[p1_idx].y) * u1y;
            if (proj < minT1) minT1 = proj;
            if (proj > maxT1) maxT1 = proj;
        }

        float w1_sx = points[p1_idx].x + u1x * minT1;
        float w1_sy = points[p1_idx].y + u1y * minT1;
        float w1_ex = points[p1_idx].x + u1x * maxT1;
        float w1_ey = points[p1_idx].y + u1y * maxT1;
        float w1_len = maxT1 - minT1;

        // Body pro druhou stěnu: ze stěnových bodů vyloučíme inliery stěny 1
        int outlierIndices[120];
        int outCount = 0;
        for (int i = 0; i < wallPtCount; i++) {
            int idx = wallIndices[i];
            bool isInl = false;
            for (int k = 0; k < bestW1Count; k++) {
                if (bestW1Inliers[k] == idx) { isInl = true; break; }
            }
            if (!isInl) outlierIndices[outCount++] = idx;
        }

        // Hledání druhé stěny v outlierech kolmé na stěnu 1
        int bestW2Inliers[120];
        int bestW2Count = 0;
        int q1_idx = -1, q2_idx = -1;

        if (outCount >= 3) {
            for (int i = 0; i < outCount && i < 25; i++) {
                int qidx1 = outlierIndices[i];
                float qx1 = points[qidx1].x;
                float qy1 = points[qidx1].y;
                for (int j = i + 1; j < outCount && j < 25; j++) {
                    int qidx2 = outlierIndices[j];
                    float qx2 = points[qidx2].x;
                    float qy2 = points[qidx2].y;
                    float qd = hypotf(qx2 - qx1, qy2 - qy1);
                    if (qd < 60.0f) continue;

                    float qux = (qx2 - qx1) / qd;
                    float quy = (qy2 - qy1) / qd;
                    float dot = fabsf(u1x * qux + u1y * quy);
                    if (dot > 0.60f) continue; // kolmost (53° až 127°)

                    float qnx = -quy;
                    float qny = qux;
                    int inl2 = 0;
                    int tempInl2[120];
                    for (int k = 0; k < outCount; k++) {
                        int pIdx = outlierIndices[k];
                        float dist = fabsf((points[pIdx].x - qx1) * qnx + (points[pIdx].y - qy1) * qny);
                        if (dist <= 20.0f) tempInl2[inl2++] = pIdx;
                    }
                    if (inl2 > bestW2Count) {
                        bestW2Count = inl2;
                        q1_idx = qidx1;
                        q2_idx = qidx2;
                        for (int k = 0; k < inl2; k++) bestW2Inliers[k] = tempInl2[k];
                    }
                }
            }
        }

        // A. ROH (2 STĚNY SBÍHAJÍCÍ SE VE VRCHOLU V - ŽÁDNÝ TROJÚHELNÍK!)
        if (bestW2Count >= 3) {
            target.type = TARGET_CORNER;
            target.wall_count = 2;

            float qdx = points[q2_idx].x - points[q1_idx].x;
            float qdy = points[q2_idx].y - points[q1_idx].y;
            float L2 = hypotf(qdx, qdy);
            float u2x = qdx / L2;
            float u2y = qdy / L2;

            float cross = u1x * u2y - u1y * u2x;
            float Vx = 0.0f, Vy = 0.0f;
            if (fabsf(cross) > 1e-4f) {
                float s = ((points[q1_idx].x - points[p1_idx].x) * u2y - (points[q1_idx].y - points[p1_idx].y) * u2x) / cross;
                Vx = points[p1_idx].x + s * u1x;
                Vy = points[p1_idx].y + s * u1y;
            } else {
                Vx = (w1_sx + w1_ex) * 0.5f;
                Vy = (w1_sy + w1_ey) * 0.5f;
            }

            float d_s1 = hypotf(w1_sx - Vx, w1_sy - Vy);
            float d_e1 = hypotf(w1_ex - Vx, w1_sy - Vy);
            float far1x = (d_e1 > d_s1) ? w1_ex : w1_sx;
            float far1y = (d_e1 > d_s1) ? w1_ey : w1_sy;
            float arm1_len = fmaxf(220.0f, fmaxf(d_s1, d_e1));
            float norm1 = fmaxf(1.0f, hypotf(far1x - Vx, far1y - Vy));
            float arm1_ux = (far1x - Vx) / norm1;
            float arm1_uy = (far1y - Vy) / norm1;

            // Rameno 1: pouze úsečka z vrcholu V podél stěny 1
            target.walls[0].x1 = Vx; target.walls[0].y1 = Vy;
            target.walls[0].x2 = Vx + arm1_ux * arm1_len; target.walls[0].y2 = Vy + arm1_uy * arm1_len;
            target.walls[0].length_mm = arm1_len;
            target.walls[0].point_count = bestW1Count;

            float minT2 = 99999.0f, maxT2 = -99999.0f;
            for (int k = 0; k < bestW2Count; k++) {
                int pIdx = bestW2Inliers[k];
                float proj = (points[pIdx].x - points[q1_idx].x) * u2x + (points[pIdx].y - points[q1_idx].y) * u2y;
                if (proj < minT2) minT2 = proj;
                if (proj > maxT2) maxT2 = proj;
            }
            float w2_sx = points[q1_idx].x + u2x * minT2;
            float w2_sy = points[q1_idx].y + u2y * minT2;
            float w2_ex = points[q1_idx].x + u2x * maxT2;
            float w2_ey = points[q1_idx].y + u2y * maxT2;
            float d_s2 = hypotf(w2_sx - Vx, w2_sy - Vy);
            float d_e2 = hypotf(w2_ex - Vx, w2_ey - Vy);
            float far2x = (d_e2 > d_s2) ? w2_ex : w2_sx;
            float far2y = (d_e2 > d_s2) ? w2_ey : w2_sy;
            float arm2_len = fmaxf(220.0f, fmaxf(d_s2, d_e2));
            float norm2 = fmaxf(1.0f, hypotf(far2x - Vx, far2y - Vy));
            float arm2_ux = (far2x - Vx) / norm2;
            float arm2_uy = (far2y - Vy) / norm2;

            // Rameno 2: druhá samostatná úsečka z vrcholu V podél stěny 2 (NIKDY se nespojuje konec 1 a 2!)
            target.walls[1].x1 = Vx; target.walls[1].y1 = Vy;
            target.walls[1].x2 = Vx + arm2_ux * arm2_len; target.walls[1].y2 = Vy + arm2_uy * arm2_len;
            target.walls[1].length_mm = arm2_len;
            target.walls[1].point_count = bestW2Count;

            // Svíčka: buď předem separovaný shluk svíčky, nebo zbylé body
            if (bestCandCount > 0) {
                target.candle_x = bestCandX;
                target.candle_y = bestCandY;
            } else {
                float candSumX = 0.0f, candSumY = 0.0f;
                int candPts = 0;
                for (int i = 0; i < outCount; i++) {
                    int idx = outlierIndices[i];
                    bool isW2 = false;
                    for (int k = 0; k < bestW2Count; k++) {
                        if (bestW2Inliers[k] == idx) { isW2 = true; break; }
                    }
                    if (!isW2) {
                        candSumX += points[idx].x;
                        candSumY += points[idx].y;
                        candPts++;
                    }
                }
                if (candPts > 0) {
                    target.candle_x = candSumX / (float)candPts;
                    target.candle_y = candSumY / (float)candPts;
                } else {
                    target.candle_x = Vx + 85.0f * arm1_ux + 85.0f * arm2_ux;
                    target.candle_y = Vy + 85.0f * arm1_uy + 85.0f * arm2_uy;
                }
            }

            float bisect_x = arm1_ux + arm2_ux;
            float bisect_y = arm1_uy + arm2_uy;
            float blen = hypotf(bisect_x, bisect_y);
            float u_open_x = (blen > 0.01f) ? (bisect_x / blen) : 0.0f;
            float u_open_y = (blen > 0.01f) ? (bisect_y / blen) : 1.0f;

            // Bezpečnostní pojistka: svíčka se NIKDY nesmí ocitnout mimo svůj Bounding Box!
            target.candle_x = fmaxf(box.min_x, fminf(box.max_x, target.candle_x));
            target.candle_y = fmaxf(box.min_y, fminf(box.max_y, target.candle_y));

            target.waypoint_x = target.candle_x + u_open_x * 380.0f;
            target.waypoint_y = target.candle_y + u_open_y * 380.0f;
            target.approach_dx = -u_open_x;
            target.approach_dy = -u_open_y;
            target.travel_dist_mm = hypotf(target.waypoint_x, target.waypoint_y);
            return target;
        }

        // B. 1 STĚNA
        target.type = TARGET_WALL;
        target.wall_count = 1;

        float pad1 = (300.0f - w1_len) * 0.5f;
        if (pad1 > 0.0f) {
            w1_sx -= pad1 * u1x; w1_sy -= pad1 * u1y;
            w1_ex += pad1 * u1x; w1_ey += pad1 * u1y;
            w1_len = 300.0f;
        }
        target.walls[0].x1 = w1_sx; target.walls[0].y1 = w1_sy;
        target.walls[0].x2 = w1_ex; target.walls[0].y2 = w1_ey;
        target.walls[0].length_mm = w1_len;
        target.walls[0].point_count = bestW1Count;

        if (bestCandCount > 0) {
            target.candle_x = bestCandX;
            target.candle_y = bestCandY;
        } else if (outCount > 0) {
            float oSumX = 0.0f, oSumY = 0.0f;
            for (int i = 0; i < outCount; i++) {
                oSumX += points[outlierIndices[i]].x;
                oSumY += points[outlierIndices[i]].y;
            }
            target.candle_x = oSumX / (float)outCount;
            target.candle_y = oSumY / (float)outCount;
        } else {
            target.candle_x = (w1_sx + w1_ex) * 0.5f;
            target.candle_y = (w1_sy + w1_ey) * 0.5f;
        }

        float t_proj = (target.candle_x - w1_sx) * u1x + (target.candle_y - w1_sy) * u1y;
        float t_clamp = fmaxf(0.0f, fminf(w1_len, t_proj));
        float p_proj_x = w1_sx + t_clamp * u1x;
        float p_proj_y = w1_sy + t_clamp * u1y;

        float perp_dx = target.candle_x - p_proj_x;
        float perp_dy = target.candle_y - p_proj_y;
        float perp_dist = hypotf(perp_dx, perp_dy);
        float norm_x = 0.0f, norm_y = 0.0f;

        if (perp_dist > 15.0f) {
            norm_x = perp_dx / perp_dist;
            norm_y = perp_dy / perp_dist;
        } else {
            float nx = -u1y, ny = u1x;
            if (nx * (-p_proj_x) + ny * (-p_proj_y) < 0) {
                nx = -nx; ny = -ny;
            }
            norm_x = nx; norm_y = ny;
            target.candle_x = p_proj_x + norm_x * 80.0f;
            target.candle_y = p_proj_y + norm_y * 80.0f;
        }

        // Bezpečnostní pojistka: svíčka se NIKDY nesmí ocitnout mimo svůj Bounding Box!
        target.candle_x = fmaxf(box.min_x, fminf(box.max_x, target.candle_x));
        target.candle_y = fmaxf(box.min_y, fminf(box.max_y, target.candle_y));

        target.waypoint_x = target.candle_x + norm_x * 380.0f;
        target.waypoint_y = target.candle_y + norm_y * 380.0f;
        target.approach_dx = -norm_x;
        target.approach_dy = -norm_y;
        target.travel_dist_mm = hypotf(target.waypoint_x, target.waypoint_y);
        return target;
    }
};

struct TrackedTarget {
    TargetDetail target;
    int type_votes[3]; // [0]=SOLO, [1]=WALL, [2]=CORNER
    int hits;
    int missed;
    bool active;
};

// ============================================================
//  StaticMapTracker - Dlouhodobá statická mapa na ESP32
//  Agreguje snímky v čase a udržuje nejstabilnější/nejčastější boxy a cíle.
// ============================================================
class StaticMapTracker {
private:
    TrackedBox _tracked[MAX_TRACKED_BOXES];
    int _trackedCount = 0;
    TrackedTarget _trackedTargets[MAX_TRACKED_BOXES];
    uint32_t _totalScans = 0;

public:
    StaticMapTracker() {
        reset();
    }

    void reset() {
        _trackedCount = 0;
        _totalScans = 0;
        for (int i = 0; i < MAX_TRACKED_BOXES; i++) {
            _trackedTargets[i].active = false;
            _trackedTargets[i].hits = 0;
            _trackedTargets[i].missed = 0;
        }
    }

    // Původní metoda pro aktualizaci boxů
    void update(const BoundingBox* liveBoxes, int liveCount, float matchDistMm = 160.0f) {
        float matchDistSq = matchDistMm * matchDistMm;
        bool matchedTracked[MAX_TRACKED_BOXES];
        for (int i = 0; i < _trackedCount; i++) matchedTracked[i] = false;

        for (int i = 0; i < liveCount; i++) {
            const BoundingBox& lb = liveBoxes[i];
            int bestIdx = -1;
            float bestDistSq = matchDistSq;

            for (int j = 0; j < _trackedCount; j++) {
                if (matchedTracked[j]) continue;
                float dx = _tracked[j].box.cx - lb.cx;
                float dy = _tracked[j].box.cy - lb.cy;
                float dsq = dx * dx + dy * dy;
                if (dsq < bestDistSq) {
                    bestDistSq = dsq;
                    bestIdx = j;
                }
            }

            if (bestIdx >= 0) {
                TrackedBox& tb = _tracked[bestIdx];
                const float alpha = 0.20f;
                tb.box.cx = (1.0f - alpha) * tb.box.cx + alpha * lb.cx;
                tb.box.cy = (1.0f - alpha) * tb.box.cy + alpha * lb.cy;
                tb.box.width = (1.0f - alpha) * tb.box.width + alpha * lb.width;
                tb.box.height = (1.0f - alpha) * tb.box.height + alpha * lb.height;
                tb.box.min_x = tb.box.cx - tb.box.width * 0.5f;
                tb.box.max_x = tb.box.cx + tb.box.width * 0.5f;
                tb.box.min_y = tb.box.cy - tb.box.height * 0.5f;
                tb.box.max_y = tb.box.cy + tb.box.height * 0.5f;
                tb.box.span_mm = sqrtf(tb.box.width * tb.box.width + tb.box.height * tb.box.height);
                tb.box.dist_mm = sqrtf(tb.box.cx * tb.box.cx + tb.box.cy * tb.box.cy);
                tb.box.point_count = (tb.box.point_count + lb.point_count) / 2;
                tb.box.is_solo_candle = lb.is_solo_candle;

                tb.hits++;
                tb.missed = 0;
                matchedTracked[bestIdx] = true;
            } else {
                if (_trackedCount < MAX_TRACKED_BOXES) {
                    TrackedBox& tb = _tracked[_trackedCount++];
                    tb.box = lb;
                    tb.hits = 1;
                    tb.missed = 0;
                }
            }
        }

        for (int j = 0; j < _trackedCount; j++) {
            if (!matchedTracked[j]) {
                _tracked[j].missed++;
            }
        }

        int writeIdx = 0;
        for (int r = 0; r < _trackedCount; r++) {
            bool keep = (_tracked[r].hits >= 10) ? (_tracked[r].missed < 18) : (_tracked[r].missed < 8);
            if (keep) {
                if (writeIdx != r) {
                    _tracked[writeIdx] = _tracked[r];
                }
                writeIdx++;
            }
        }
        _trackedCount = writeIdx;
    }

    // Nová metoda pro kompletní aktualizaci (včetně svíček, stěn a nájezdových waypointů na ESP32)
    void update(const LidarPoint* points, int pointCount,
                const BoundingBox* liveBoxes, int liveCount,
                float matchDistMm = 250.0f)
    {
        _totalScans++;
        // 1. Aktualizace bounding boxů
        update(liveBoxes, liveCount, 160.0f);

        // 2. Aktualizace detailních cílů
        float matchDistSq = matchDistMm * matchDistMm;
        bool matchedTracked[MAX_TRACKED_BOXES];
        for (int i = 0; i < MAX_TRACKED_BOXES; i++) matchedTracked[i] = false;

        for (int i = 0; i < liveCount; i++) {
            TargetDetail liveT = TargetExtractor::extract(points, pointCount, liveBoxes[i]);

            int bestIdx = -1;
            float bestDistSq = matchDistSq;

            for (int j = 0; j < MAX_TRACKED_BOXES; j++) {
                if (!_trackedTargets[j].active || matchedTracked[j]) continue;
                float dx = _trackedTargets[j].target.candle_x - liveT.candle_x;
                float dy = _trackedTargets[j].target.candle_y - liveT.candle_y;
                float dsq = dx * dx + dy * dy;
                if (dsq < bestDistSq) {
                    bestDistSq = dsq;
                    bestIdx = j;
                }
            }

            if (bestIdx >= 0) {
                TrackedTarget& tt = _trackedTargets[bestIdx];
                tt.hits++;
                tt.missed = 0;
                matchedTracked[bestIdx] = true;

                tt.type_votes[liveT.type]++;
                if (tt.hits <= 2) {
                    tt.target.type = liveT.type;
                } else if (tt.type_votes[TARGET_CORNER] >= 4) {
                    tt.target.type = TARGET_CORNER;
                } else if (tt.type_votes[TARGET_WALL] >= 4 && tt.target.type != TARGET_CORNER) {
                    tt.target.type = TARGET_WALL;
                } else if (tt.type_votes[TARGET_SOLO] >= 7 && tt.type_votes[TARGET_CORNER] < 2 && tt.type_votes[TARGET_WALL] < 2) {
                    tt.target.type = TARGET_SOLO;
                }

                const float alpha = 0.15f;
                tt.target.box = liveT.box;
                tt.target.candle_x = (1.0f - alpha) * tt.target.candle_x + alpha * liveT.candle_x;
                tt.target.candle_y = (1.0f - alpha) * tt.target.candle_y + alpha * liveT.candle_y;
                tt.target.waypoint_x = (1.0f - alpha) * tt.target.waypoint_x + alpha * liveT.waypoint_x;
                tt.target.waypoint_y = (1.0f - alpha) * tt.target.waypoint_y + alpha * liveT.waypoint_y;
                tt.target.approach_dx = (1.0f - alpha) * tt.target.approach_dx + alpha * liveT.approach_dx;
                tt.target.approach_dy = (1.0f - alpha) * tt.target.approach_dy + alpha * liveT.approach_dy;
                float aLen = hypotf(tt.target.approach_dx, tt.target.approach_dy);
                if (aLen > 0.01f) {
                    tt.target.approach_dx /= aLen;
                    tt.target.approach_dy /= aLen;
                }
                tt.target.travel_dist_mm = hypotf(tt.target.waypoint_x, tt.target.waypoint_y);
                tt.target.is_visible = true;
                tt.target.hits = tt.hits;

                // Stěny: vyhlazení se zachováním stejné orientace úsečky a eliminace přeblikávání
                if (liveT.wall_count > 0) {
                    if (tt.target.wall_count == liveT.wall_count) {
                        for (int w = 0; w < liveT.wall_count; w++) {
                            float s1x = tt.target.walls[w].x1, s1y = tt.target.walls[w].y1;
                            float e1x = tt.target.walls[w].x2, e1y = tt.target.walls[w].y2;
                            float s2x = liveT.walls[w].x1, s2y = liveT.walls[w].y1;
                            float e2x = liveT.walls[w].x2, e2y = liveT.walls[w].y2;

                            float d_same = hypotf(s1x - s2x, s1y - s2y) + hypotf(e1x - e2x, e1y - e2y);
                            float d_flip = hypotf(s1x - e2x, s1y - e2y) + hypotf(e1x - s2x, e1y - s2y);
                            if (d_flip < d_same) {
                                float tx = s2x; s2x = e2x; e2x = tx;
                                float ty = s2y; s2y = e2y; e2y = ty;
                            }

                            tt.target.walls[w].x1 = (1.0f - alpha) * s1x + alpha * s2x;
                            tt.target.walls[w].y1 = (1.0f - alpha) * s1y + alpha * s2y;
                            tt.target.walls[w].x2 = (1.0f - alpha) * e1x + alpha * e2x;
                            tt.target.walls[w].y2 = (1.0f - alpha) * e1y + alpha * e2y;
                            tt.target.walls[w].length_mm = hypotf(tt.target.walls[w].x2 - tt.target.walls[w].x1,
                                                                 tt.target.walls[w].y2 - tt.target.walls[w].y1);
                        }
                    } else {
                        if (!(tt.hits > 5 && tt.target.wall_count > liveT.wall_count)) {
                            tt.target.wall_count = liveT.wall_count;
                            tt.target.walls[0] = liveT.walls[0];
                            tt.target.walls[1] = liveT.walls[1];
                        }
                    }
                }
            } else {
                int emptyIdx = -1;
                for (int j = 0; j < MAX_TRACKED_BOXES; j++) {
                    if (!_trackedTargets[j].active) { emptyIdx = j; break; }
                }
                if (emptyIdx >= 0) {
                    TrackedTarget& tt = _trackedTargets[emptyIdx];
                    tt.target = liveT;
                    tt.hits = 1;
                    tt.missed = 0;
                    tt.active = true;
                    tt.type_votes[0] = 0; tt.type_votes[1] = 0; tt.type_votes[2] = 0;
                    tt.type_votes[liveT.type] = 1;
                    matchedTracked[emptyIdx] = true;
                }
            }
        }

        for (int j = 0; j < MAX_TRACKED_BOXES; j++) {
            if (_trackedTargets[j].active && !matchedTracked[j]) {
                _trackedTargets[j].missed++;
                _trackedTargets[j].target.is_visible = false;
                if (_trackedTargets[j].missed > 20) {
                    _trackedTargets[j].active = false;
                }
            }
        }
    }

    // Získá potvrzené boxy
    void getStaticBoxes(BoundingBox* outBoxes, int& outCount, int minHits = 4) const {
        outCount = 0;
        TrackedBox pool[MAX_TRACKED_BOXES];
        int poolCount = 0;

        for (int i = 0; i < _trackedCount; i++) {
            if (_tracked[i].hits >= minHits) {
                pool[poolCount++] = _tracked[i];
            }
        }

        for (int a = 0; a < poolCount - 1; a++) {
            for (int b = a + 1; b < poolCount; b++) {
                if (pool[b].hits > pool[a].hits) {
                    TrackedBox tmp = pool[a];
                    pool[a] = pool[b];
                    pool[b] = tmp;
                }
            }
        }

        int limit = (poolCount < MAX_DETECTED_BOXES) ? poolCount : MAX_DETECTED_BOXES;
        for (int i = 0; i < limit; i++) {
            outBoxes[outCount++] = pool[i].box;
        }
    }

    // Získá potvrzené detailní cíle a optimální trasu hašení (počítáno 100% na ESP32)
    void getStaticTargets(TargetDetail* outTargets, int& outCount,
                          int* outRoute, int& routeCount,
                          int minHits = 3) const
    {
        outCount = 0;
        routeCount = 0;

        TargetDetail pool[MAX_TRACKED_BOXES];
        int poolHits[MAX_TRACKED_BOXES];
        int poolCount = 0;

        for (int i = 0; i < MAX_TRACKED_BOXES; i++) {
            if (_trackedTargets[i].active && _trackedTargets[i].hits >= minHits && _trackedTargets[i].missed <= 3) {
                // Svíčka musí ležet před robotem v poli (Y >= 30 mm)
                if (_trackedTargets[i].target.candle_y >= 30.0f) {
                    pool[poolCount] = _trackedTargets[i].target;
                    poolHits[poolCount] = _trackedTargets[i].hits;
                    poolCount++;
                }
            }
        }

        for (int a = 0; a < poolCount - 1; a++) {
            for (int b = a + 1; b < poolCount; b++) {
                if (poolHits[b] > poolHits[a]) {
                    TargetDetail tmpT = pool[a]; pool[a] = pool[b]; pool[b] = tmpT;
                    int tmpH = poolHits[a]; poolHits[a] = poolHits[b]; poolHits[b] = tmpH;
                }
            }
        }

        outCount = (poolCount < MAX_DETECTED_BOXES) ? poolCount : MAX_DETECTED_BOXES;
        for (int i = 0; i < outCount; i++) {
            outTargets[i] = pool[i];
            outTargets[i].id = i + 1;
        }

        // Výpočet optimální trasy hašení (Nearest Neighbor ze startu (0,0))
        if (outCount > 0) {
            bool visited[MAX_DETECTED_BOXES];
            for (int i = 0; i < outCount; i++) visited[i] = false;

            float curX = 0.0f, curY = 0.0f;
            for (int step = 0; step < outCount; step++) {
                int bestNext = -1;
                float bestDist = 999999.0f;
                for (int i = 0; i < outCount; i++) {
                    if (!visited[i]) {
                        float d = hypotf(outTargets[i].waypoint_x - curX, outTargets[i].waypoint_y - curY);
                        if (d < bestDist) {
                            bestDist = d;
                            bestNext = i;
                        }
                    }
                }
                if (bestNext >= 0) {
                    visited[bestNext] = true;
                    outRoute[routeCount++] = outTargets[bestNext].id;
                    curX = outTargets[bestNext].waypoint_x;
                    curY = outTargets[bestNext].waypoint_y;
                }
            }
        }
    }

    uint32_t getTotalScans() const { return _totalScans; }
    int getTrackedCount() const { return _trackedCount; }
};


