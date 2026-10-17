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
    float max_diameter_mm = 80.0f;
    int min_points = 2;
    int max_points = 8;
    float max_neighbor_dist_mm = 55.0f;
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

        // Seřazení podle skóre a výběr nejlepších stěn (max MAX_DETECTED_WALLS)
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
        // 5. KROK: Validace svíček vůči stěnám
        // Svíčka NESMÍ ležet v dráze stěny (nesmí být v přímce stěny ani na jejím rohu).
        // Skutečná svíčka stojí kolmo před stěnou s dostatečným odstupem (d_perp >= 55 mm).
        // Rohová svíčka a svíčka na kolmici stěny jsou před vyřazením chráněny.
        // ========================================================
        static Candle validCandles[MAX_DETECTED_CANDLES];
        int validCandleCount = 0;

        for (int c = 0; c < candleCount; c++) {
            const Candle& cand = outCandles[c];

            // Rohová svíčka a svíčka na kolmici stěny jsou ověřeny a nesmí být vyřazeny chodbou
            if (isCornerCandle[c] || isPerpCandle[c]) {
                if (validCandleCount < MAX_DETECTED_CANDLES) {
                    validCandles[validCandleCount++] = cand;
                }
                continue;
            }

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
        int rawIndices[16];

        for (int k = 0; k < count && rawInliers < 16; k++) {
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

        if (rawInliers >= 2 && rawInliers <= 8) {
            float avgX = sumX / (float)rawInliers;
            float avgY = sumY / (float)rawInliers;

            // Kontrola kompaktnosti shluku (svíčka má průměr 50 mm, body do cca 75 mm)
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

            if (maxDistSq <= (75.0f * 75.0f)) {
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
