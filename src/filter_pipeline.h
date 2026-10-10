#pragma once
#include <Arduino.h>
#include "d200_types.h"

// ============================================================
//  filter_pipeline.h
//  Filtrační pipeline konfigurovatelná za běhu:
//  - Range Clamp: mrtvá zóna (< min_range) a stěny místnosti (> max_range)
//  - Intensity Filter: práh minimální odrazivosti
//  - FoV Masking: 360° vs. dopředný kužel (např. 180° nebo 120°)
//  - Konstrukční maska: vyřazení úhlů, kde senzoru stíní robot
// ============================================================

class FilterPipeline {
public:
    static void apply(LidarPoint* points, int count, const FilterConfig& cfg, int& validCount) {
        validCount = 0;
        float halfFov = cfg.fov_angle_deg / 2.0f;

        for (int i = 0; i < count; i++) {
            LidarPoint& p = points[i];
            p.valid = false;

            // 1. Vzdálenost (Range clamp)
            if (p.dist_mm < cfg.min_range_mm || p.dist_mm > cfg.max_range_mm) {
                continue;
            }

            // 2. Intenzita odrazu
            if (p.intensity < cfg.min_intensity) {
                continue;
            }

            // 3. Konstrukční maska (slepá zóna za robotem nebo sloupky)
            if (cfg.blind_spot_enabled) {
                if (cfg.blind_min_deg <= cfg.blind_max_deg) {
                    if (p.angle_deg >= cfg.blind_min_deg && p.angle_deg <= cfg.blind_max_deg) {
                        continue;
                    }
                } else {
                    // Přechod přes 0°/360°
                    if (p.angle_deg >= cfg.blind_min_deg || p.angle_deg <= cfg.blind_max_deg) {
                        continue;
                    }
                }
            }

            // 4. FoV ořez (dopředný kužel např. ±90° pro 180° nebo ±60° pro 120°)
            if (cfg.fov_limited) {
                // Úhel p.angle_deg je v rozsahu [0, 360) s 0° přímo vpřed.
                // Převod na symetrický rozsah [-180, +180]
                float centeredAngle = p.angle_deg;
                if (centeredAngle > 180.0f) {
                    centeredAngle -= 360.0f;
                }
                if (fabsf(centeredAngle) > halfFov) {
                    continue;
                }
            }

            // Bod prošel všemi filtry
            p.valid = true;
            validCount++;
        }
    }
};
