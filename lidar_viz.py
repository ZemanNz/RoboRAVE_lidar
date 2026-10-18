#!/usr/bin/env python3
"""
LIDAR_SVICKY_27 - Vícepanelový lokalizační a detekční systém (Kybernoid 27)
Filozofie: Shluky bodů (Bounding Boxy), invariantní map-matching a kumulativní mapa hřiště.

4 panely:
  1. LEVÝ PANEL (Nahoře): Živý radar (Live LIDAR scan) v reálném čase + surové Bounding Boxy.
  2. STŘEDNÍ PANEL (Nahoře): Referenční etalon (zamčený na STARTu) s odstupňovanými kotvami a trojúhelníkem.
  3. PRAVÝ PANEL (Nahoře): Porovnání map & LOKALIZACE ROBOTA (paprsky shod, růžový robot se směrem).
  4. SPODNÍ PANEL: Celková obdélníková mapa hřiště (2.4 m x 3.5 m) s kumulativním dosazováním cílů,
     zachováním zastíněných svíček při pohybu, kontrolou pravidel a optimální trasou hašení.

Ovládání:
  [L]         - Zamknout / Odemknout referenční etalon (aktivuje lokalizaci a kumulativní mapu)
  [2 / ě]     - Startovní stěna 2.4 m (Kratší stěna) -> auto-fit pole 2.4 x 3.5 m
  [3 / š]     - Startovní stěna 3.5 m (Delší stěna)  -> auto-fit pole 3.5 x 2.4 m
  [P]         - Přepnout startovní stěnu (2.4 m <-> 3.5 m)
  [F]         - Přepnout FoV: 180° Kužel (start) / 360° Plný rozhled (v poli)
  [MEZERNÍK]  - Zmrazit / Odmrazit obraz (Freeze)
  [C / M]     - Resetovat mapy a začít znovu sbírat etalon na startu
  [N]         - Resetovat novou stabilní mapu na aktuální pozici
  [R / E]     - Min vzdálenost (+50 / -50 mm)
  [T / G]     - Max vzdálenost (+200 / -200 mm)
  [I / K]     - Min intenzita (+5 / -5)
  [+ / -]     - Zoom in / Zoom out (lze použít i kolečko myši)
  Klik myší   - Detailní inspekce souřadnic vybraného bodu / cíle
"""

import sys, os, time, json, math
import pygame
import serial
from serial.tools import list_ports

# ============================================================
#  Rozměry oken a panelů (Optimalizováno pro obrazovky 1920x1080)
# ============================================================
PANEL_W     = 600
PANEL_H     = 560      # Výška horních 3 panelů
FIELD_H     = 300      # Výška spodní mapy hřiště
WIN_W       = PANEL_W * 3
WIN_H       = PANEL_H + FIELD_H

# Rozměry soutěžního hřiště Kybernoid / RoboRAVE (mm)
FIELD_W_MM  = 3500.0   # Delší strana hřiště
FIELD_H_MM  = 2400.0   # Kratší strana hřiště

# Konfigurace startovních stěn (přepínatelné klávesami [2/ě], [3/š] nebo [P])
WALL_CONFIGS = {
    2: {
        "name": "2.4 m (Kratší stěna)",
        "w": 2400.0, "l": 3500.0,
        "key_hint": "[2 / ě]",
        "desc": "Robot u stěny 2.4 m, pole sahá 3.5 m dopředu"
    },
    3: {
        "name": "3.5 m (Delší stěna)",
        "w": 3500.0, "l": 2400.0,
        "key_hint": "[3 / š]",
        "desc": "Robot u stěny 3.5 m, pole sahá 2.4 m dopředu"
    }
}

# Barevná paleta
BG_COLOR     = (18, 20, 24)
GRID_COLOR   = (40, 45, 52)
CIRCLE_COLOR = (55, 62, 75)
TEXT_COLOR   = (220, 225, 230)
ACCENT_COLOR = (0, 210, 255)
ACCENT_MAP   = (255, 200, 60)

PINK_MAIN    = (255, 70, 180)
PINK_BG      = (65, 15, 45)
PINK_LITE    = (255, 150, 215)

def get_serial_port():
    if len(sys.argv) > 1: return sys.argv[1]
    ports = list(list_ports.comports())
    for p in ports:
        if "USB" in p.device or "ACM" in p.device: return p.device
    return "/dev/ttyUSB0"

def get_baudrate():
    if len(sys.argv) > 2: return int(sys.argv[2])
    return 921600

def parse_box_list(raw_boxes):
    """Převede surové pole z ESP32 JSON na strukturované slovníky Bounding Boxů."""
    boxes = []
    for b in raw_boxes:
        if len(b) >= 12:
            is_solo = bool(b[11])
            boxes.append({
                'min_x': float(b[0]), 'max_x': float(b[1]),
                'min_y': float(b[2]), 'max_y': float(b[3]),
                'cx': float(b[4]), 'cy': float(b[5]),
                'width': float(b[6]), 'height': float(b[7]),
                'span_mm': float(b[8]),
                'dist_mm': float(b[9]),
                'pts_count': int(b[10]),
                'is_solo': is_solo,
                'type': "SVÍČKA" if is_solo else "SVÍČKA+STĚNA"
            })
    return boxes


# ============================================================
#  Map-to-Map Lokalizátor robota (Porovnání Bounding Boxů)
# ============================================================
class RobotLocalizer:
    """
    Spolehlivý invariantní Map-to-Map matching na Bounding Boxech:
    1. Při zamknutí [L] se uloží REFERENČNÍ MAPA (Bounding Boxy ze startovní pozice).
       Vypočtou se všechny vzájemné vzdálenosti mezi středy boxů (vzdálenosti jsou absolutně
       invariantní vůči jakékoliv translaci či rotaci robota).
    2. Vytvoří se hierarchické geometrické kotvy:
       - Stupeň 1: Trojúhelníkový vzorec (Kotva 1 + Kotva 2 přes společný pivot).
       - Stupeň 2: Kotva 1 (Primární přímka s největším rozvorem pro maximální úhlovou přesnost).
       - Stupeň 3: Kotva 2 (Sekundární přímka pro případ zastínění jednoho cíle).
       - Stupeň 4: Záložní kotvy (jakákoliv dvojice cílů).
    3. Analytická 2D rigidní transformace (Procrustes / Kabsch) okamžitě spočítá přesnou
       polohu (X, Y) a orientaci (theta) robota vůči výchozímu bodu START = [0,0].
    """

    def __init__(self):
        self.locked           = False
        self.ref_boxes        = []   # Referenční Bounding Boxy v souřadnicích startu
        self.ref_dists        = []   # [(i, j, dist_mm)] mezi referenčními středy boxů
        self.anchors          = []   # Seřazené kotvy podle rozvoru a kvality
        self.triangle_pattern = None # Trojúhelníkový vzorec

        self.robot_x          = 0.0  # mm ve světě (START = [0,0])
        self.robot_y          = 0.0
        self.robot_deg        = 0.0  # orientace ve stupních
        self.pose_confidence  = 0.0  # 0.0 – 1.0
        self.trajectory       = [(0.0, 0.0)]
        self.is_stationary    = True
        self.last_match_info  = None # Detailní informace o shodě pro vykreslení paprsků

    def lock_map(self, boxes):
        """Uloží aktuální potvrzené Bounding Boxy jako referenční etalon světa a sestaví kotvy."""
        self.ref_boxes = [dict(b) for b in boxes]
        self.ref_dists = []
        self.last_match_info = None

        n = len(self.ref_boxes)
        if n < 2:
            self.locked = False
            return False

        # 1. Invariantní párové vzdálenosti mezi všemi středy boxů
        for i in range(n):
            for j in range(i + 1, n):
                d = math.hypot(self.ref_boxes[i]['cx'] - self.ref_boxes[j]['cx'],
                               self.ref_boxes[i]['cy'] - self.ref_boxes[j]['cy'])
                self.ref_dists.append((i, j, d))

        # 2. Sestavení a seřazení geometrických kotev
        raw_anchors = []
        for i in range(n):
            for j in range(i + 1, n):
                b1 = self.ref_boxes[i]
                b2 = self.ref_boxes[j]
                dx = b2['cx'] - b1['cx']
                dy = b2['cy'] - b1['cy']
                dist = math.hypot(dx, dy)
                if dist < 250.0:
                    continue
                # Skóre: větší rozvor = podstatně vyšší přesnost úhlu + bonus za body
                score = dist + (b1.get('pts_count', 0) + b2.get('pts_count', 0)) * 5.0
                raw_anchors.append({
                    'idx1': i, 'idx2': j,
                    'p1': (b1['cx'], b1['cy']),
                    'p2': (b2['cx'], b2['cy']),
                    'dist': dist,
                    'score': score
                })

        raw_anchors.sort(key=lambda a: a['score'], reverse=True)
        self.anchors = []
        for rank, a in enumerate(raw_anchors):
            a['rank'] = rank + 1
            if rank == 0:
                a['name'] = "KOTVA 1 (PRIMÁRNÍ)"
                a['color'] = (255, 215, 0)   # Zlatá
            elif rank == 1:
                a['name'] = "KOTVA 2 (SEKUNDÁRNÍ)"
                a['color'] = (255, 150, 40)  # Oranžová
            else:
                a['name'] = f"KOTVA {rank+1} (ZÁLOŽNÍ)"
                a['color'] = (200, 140, 255) # Fialová
            self.anchors.append(a)

        # 3. Sestavení trojúhelníkového vzorce (pokud máme alespoň 3 boxy)
        self.triangle_pattern = None
        if len(self.anchors) >= 2 and n >= 3:
            a1 = self.anchors[0]
            a2 = self.anchors[1]
            shared = set([a1['idx1'], a1['idx2']]).intersection([a2['idx1'], a2['idx2']])
            if shared:
                pivot_idx = list(shared)[0]
                other1_idx = a1['idx2'] if a1['idx1'] == pivot_idx else a1['idx1']
                other2_idx = a2['idx2'] if a2['idx1'] == pivot_idx else a2['idx1']
                pivot_pt = (self.ref_boxes[pivot_idx]['cx'], self.ref_boxes[pivot_idx]['cy'])
                p1_pt    = (self.ref_boxes[other1_idx]['cx'], self.ref_boxes[other1_idx]['cy'])
                p2_pt    = (self.ref_boxes[other2_idx]['cx'], self.ref_boxes[other2_idx]['cy'])

                v1_x = p1_pt[0] - pivot_pt[0]; v1_y = p1_pt[1] - pivot_pt[1]
                v2_x = p2_pt[0] - pivot_pt[0]; v2_y = p2_pt[1] - pivot_pt[1]
                az1 = math.atan2(v1_x, v1_y)
                az2 = math.atan2(v2_x, v2_y)
                ang_diff = abs(math.atan2(math.sin(az1 - az2), math.cos(az1 - az2)))
                d3 = math.hypot(p1_pt[0] - p2_pt[0], p1_pt[1] - p2_pt[1])

                self.triangle_pattern = {
                    'pivot_idx': pivot_idx,
                    'other1_idx': other1_idx,
                    'other2_idx': other2_idx,
                    'pivot_pt': pivot_pt,
                    'p1_pt': p1_pt,
                    'p2_pt': p2_pt,
                    'dist1': a1['dist'],
                    'dist2': a2['dist'],
                    'dist3': d3,
                    'angle_deg': math.degrees(ang_diff)
                }

        self.locked = True
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 1.0
        self.trajectory = [(0.0, 0.0)]
        self.is_stationary = True
        return True

    def unlock_map(self):
        """Odemkne mapu a vrátí robota do výchozího stavu."""
        self.locked = False
        self.ref_boxes = []
        self.ref_dists = []
        self.anchors = []
        self.triangle_pattern = None
        self.last_match_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 0.0
        self.trajectory = [(0.0, 0.0)]
        self.is_stationary = True

    @staticmethod
    def _rigid_transform_2d(src, dst):
        """
        Metoda nejmenších čtverců (Procrustes / Kabsch) v navigačním systému:
        +Y dopředu, +X doprava, úhel theta orientován doprava (ve směru hodinových ručiček).
        """
        n = len(src)
        if n == 0: return 0.0, 0.0, 0.0
        if n == 1:
            return dst[0][0] - src[0][0], dst[0][1] - src[0][1], 0.0

        cx_s = sum(p[0] for p in src) / n
        cy_s = sum(p[1] for p in src) / n
        cx_d = sum(p[0] for p in dst) / n
        cy_d = sum(p[1] for p in dst) / n

        A = 0.0
        B = 0.0
        for (sx, sy), (dx, dy) in zip(src, dst):
            scx = sx - cx_s; scy = sy - cy_s
            dcx = dx - cx_d; dcy = dy - cy_d
            A += (dcx * scx + dcy * scy)
            B += (dcx * scy - dcy * scx)

        theta = math.atan2(B, A)
        cos_t = math.cos(theta); sin_t = math.sin(theta)

        tx = cx_d - (cx_s * cos_t + cy_s * sin_t)
        ty = cy_d - (-cx_s * sin_t + cy_s * cos_t)
        return tx, ty, theta

    def estimate_pose(self, current_boxes):
        """
        Určí pozici a orientaci robota (x, y, theta) porovnáním aktuálních Bounding Boxů
        s referenčním etalonem ze startu pomocí odstupňovaných kotev.
        """
        if not self.locked or len(self.ref_boxes) < 2 or len(current_boxes) < 2:
            return False

        cands = [(float(b['cx']), float(b['cy'])) for b in current_boxes]

        def evaluate_transformation(tx, ty, th, active_anchors, pattern_name, base_score=0.0):
            cos_t = math.cos(th); sin_t = math.sin(th)
            inliers = 0
            err_sum = 0.0
            inlier_src = []
            inlier_dst = []
            inlier_pairs = []

            for cp in cands:
                # Navigační transformace: X doprava, Y dopředu, th doprava
                wx = tx + cp[0] * cos_t + cp[1] * sin_t
                wy = ty - cp[0] * sin_t + cp[1] * cos_t
                min_d = 9999.0
                best_rb = None
                for rb in self.ref_boxes:
                    d = math.hypot(wx - rb['cx'], wy - rb['cy'])
                    if d < min_d:
                        min_d = d
                        best_rb = rb

                if min_d < 160.0 and best_rb:
                    inliers += 1
                    err_sum += min_d
                    inlier_src.append(cp)
                    inlier_dst.append((best_rb['cx'], best_rb['cy']))
                    inlier_pairs.append(((best_rb['cx'], best_rb['cy']), (wx, wy), min_d))

            avg_err = (err_sum / inliers) if inliers > 0 else 9999.0

            # Zpřesnění metodou Procrustes přes inliers
            if len(inlier_src) >= 2:
                rtx, rty, rth = self._rigid_transform_2d(inlier_src, inlier_dst)
            else:
                rtx, rty, rth = tx, ty, th

            total_score = base_score + inliers * 500.0 - avg_err * 2.0
            return {
                'inliers': inliers,
                'avg_error': avg_err,
                'tx': rtx, 'ty': rty, 'theta': rth,
                'inlier_pairs': inlier_pairs,
                'active_anchors': active_anchors,
                'pattern_name': pattern_name,
                'score': total_score
            }

        best_match_info = None
        best_score = -999999.0

        # ========================================================
        # STUPEŇ 1: SHODA TROJÚHELNÍKOVÉHO VZORCE (Kotva 1 + Kotva 2)
        # ========================================================
        if self.triangle_pattern and len(cands) >= 3:
            tp = self.triangle_pattern
            for p in range(len(cands)):
                for a in range(len(cands)):
                    if a == p: continue
                    d1_cur = math.hypot(cands[a][0] - cands[p][0], cands[a][1] - cands[p][1])
                    if abs(d1_cur - tp['dist1']) > 130.0: continue

                    for b in range(len(cands)):
                        if b == p or b == a: continue
                        d2_cur = math.hypot(cands[b][0] - cands[p][0], cands[b][1] - cands[p][1])
                        if abs(d2_cur - tp['dist2']) > 130.0: continue

                        # Kontrola 3. strany trojúhelníku
                        d3_cur = math.hypot(cands[a][0] - cands[b][0], cands[a][1] - cands[b][1])
                        if abs(d3_cur - tp['dist3']) > 140.0: continue

                        # Azimut Kotvy 1 v referenčním světě
                        del_X = tp['p1_pt'][0] - tp['pivot_pt'][0]
                        del_Y = tp['p1_pt'][1] - tp['pivot_pt'][1]
                        az_ref = math.atan2(del_X, del_Y)

                        # Azimut v aktuálním měření senzoru
                        d_x = cands[a][0] - cands[p][0]
                        d_y = cands[a][1] - cands[p][1]
                        az_cur = math.atan2(d_x, d_y)

                        th = (az_ref - az_cur + math.pi) % (2 * math.pi) - math.pi
                        cos_t = math.cos(th); sin_t = math.sin(th)
                        tx = tp['pivot_pt'][0] - (cands[p][0] * cos_t + cands[p][1] * sin_t)
                        ty = tp['pivot_pt'][1] - (-cands[p][0] * sin_t + cands[p][1] * cos_t)

                        res = evaluate_transformation(
                            tx, ty, th,
                            active_anchors=self.anchors[:2],
                            pattern_name="TROJÚHELNÍK (KOTVA 1 + 2)",
                            base_score=5000.0
                        )

                        if res['inliers'] >= 3 and res['score'] > best_score:
                            best_score = res['score']
                            best_match_info = res

        # ========================================================
        # STUPEŇ 2: SHODA JEDNOTLIVÝCH KOTEVA (Kotva 1 -> Kotva 2 -> Záložní)
        # ========================================================
        if best_match_info is None or best_match_info['inliers'] < 3:
            for anchor in self.anchors:
                d_ref = anchor['dist']
                ref1 = anchor['p1']
                ref2 = anchor['p2']
                del_X = ref2[0] - ref1[0]
                del_Y = ref2[1] - ref1[1]
                az_ref = math.atan2(del_X, del_Y)

                for a in range(len(cands)):
                    for b in range(a + 1, len(cands)):
                        d_cur = math.hypot(cands[a][0] - cands[b][0], cands[a][1] - cands[b][1])
                        if abs(d_cur - d_ref) > 130.0: continue

                        for (p1, p2) in [(cands[a], cands[b]), (cands[b], cands[a])]:
                            d_x = p2[0] - p1[0]
                            d_y = p2[1] - p1[1]
                            az_cur = math.atan2(d_x, d_y)

                            th = (az_ref - az_cur + math.pi) % (2 * math.pi) - math.pi
                            cos_t = math.cos(th); sin_t = math.sin(th)
                            tx = ref1[0] - (p1[0] * cos_t + p1[1] * sin_t)
                            ty = ref1[1] - (-p1[0] * sin_t + p1[1] * cos_t)

                            rank_bonus = max(0.0, 1500.0 - (anchor['rank'] - 1) * 350.0)
                            res = evaluate_transformation(
                                tx, ty, th,
                                active_anchors=[anchor],
                                pattern_name=anchor['name'],
                                base_score=rank_bonus
                            )

                            if res['inliers'] >= 2 and res['score'] > best_score:
                                best_score = res['score']
                                best_match_info = res

        # ========================================================
        # APLIKACE VÝSLEDKU LOKALIZACE
        # ========================================================
        if best_match_info is not None and best_match_info['inliers'] >= 2:
            self.last_match_info = best_match_info
            self.robot_x = best_match_info['tx']
            self.robot_y = best_match_info['ty']
            self.robot_deg = math.degrees(best_match_info['theta'])
            self.is_stationary = True
            quality = min(1.0, best_match_info['inliers'] / max(2, len(self.ref_boxes)))
            self.pose_confidence = quality

            # Záznam trajektorie robota (při posunu alespoň o 20 mm)
            last_pt = self.trajectory[-1]
            if math.hypot(self.robot_x - last_pt[0], self.robot_y - last_pt[1]) > 20.0:
                self.trajectory.append((self.robot_x, self.robot_y))
                if len(self.trajectory) > 500:
                    self.trajectory.pop(0)

            return True
        else:
            self.pose_confidence = max(0.0, self.pose_confidence - 0.05)
            return False


# ============================================================
#  Kumulativní mapa celého hřiště (GlobalFieldMap)
#  S podporou doplňování cílů a zachováním zastíněných překážek
# ============================================================
class GlobalFieldMap:
    """
    Globální model celého hřiště (2.4 m x 3.5 m):
    - Kumulativně shromažďuje Bounding Boxy (až 4 cíle).
    - Zachovává známé cíle, i když je robot v daném úhlu zrovna nevidí (jsou zastíněné).
    - Jakmile se při pohybu objeví dosud neviděný cíl (např. 4. svíčka skrytá za stěnou),
      automaticky ho doplní do mapy.
    - Filtruje falešné odrazy za mantinelem hřiště.
    - Počítá optimální pořadí trasy pro hašení svíček (1 -> 2 -> 3 -> 4).
    """

    def __init__(self):
        self.targets = []               # Seznam globálních cílů v souřadnicích hřiště
        self.robot_start = (0.0, 0.0)   # Výchozí pozice START
        self.robot_pos = (0.0, 0.0)     # Aktuální pozice robota [X, Y]
        self.robot_deg = 0.0            # Aktuální natočení
        self.trajectory = [(0.0, 0.0)]  # Trajektorie robota
        self.total_updates = 0
        self.start_wall_mode = 2        # 2 = 2.4 m (Kratší stěna), 3 = 3.5 m (Delší stěna)
        self.smooth_bounds = None       # [xmin, xmax, ymin, ymax]
        self.rejected_outside_count = 0 # Počet bodů mimo mantinel

    def reset(self):
        self.targets.clear()
        self.robot_pos = (0.0, 0.0)
        self.robot_deg = 0.0
        self.trajectory = [(0.0, 0.0)]
        self.total_updates = 0
        self.smooth_bounds = None
        self.rejected_outside_count = 0

    def set_start_wall(self, mode):
        if mode in (2, 3) and self.start_wall_mode != mode:
            self.start_wall_mode = mode
            self.smooth_bounds = None

    def toggle_start_wall(self):
        new_mode = 3 if self.start_wall_mode == 2 else 2
        self.set_start_wall(new_mode)
        return self.start_wall_mode

    def get_field_dims(self):
        cfg = WALL_CONFIGS.get(self.start_wall_mode, WALL_CONFIGS[2])
        return cfg["w"], cfg["l"]

    def get_auto_field_bounds(self):
        """Vrátí hranice hřiště [xmin, xmax, ymin, ymax] přizpůsobené startu a cílům."""
        W, L = self.get_field_dims()

        x_pts = [0.0, self.robot_pos[0]]
        y_pts = [0.0, self.robot_pos[1]]
        for tp in self.trajectory:
            x_pts.append(tp[0]); y_pts.append(tp[1])
        for t in self.targets:
            if abs(t['x']) <= 4500.0 and -800.0 <= t['y'] <= 5000.0:
                x_pts.append(t['x']); y_pts.append(t['y'])

        # Zadní startovní stěna (robot začíná cca 200 mm od mantinelu)
        min_y = min(y_pts)
        target_ymin = min(-200.0, min_y - 80.0)
        target_ymax = target_ymin + L

        # Vycentrování na šířku W
        min_x = min(x_pts)
        max_x = max(x_pts)
        mid_x = (min_x + max_x) * 0.5
        target_xmin = mid_x - W * 0.5
        target_xmax = target_xmin + W

        if self.smooth_bounds is None:
            self.smooth_bounds = [target_xmin, target_xmax, target_ymin, target_ymax]
        else:
            alpha = 0.10
            self.smooth_bounds[0] = (1.0 - alpha) * self.smooth_bounds[0] + alpha * target_xmin
            self.smooth_bounds[1] = self.smooth_bounds[0] + W
            self.smooth_bounds[2] = (1.0 - alpha) * self.smooth_bounds[2] + alpha * target_ymin
            self.smooth_bounds[3] = self.smooth_bounds[2] + L

        return tuple(self.smooth_bounds)

    def update_robot_pose(self, rx, ry, rdeg):
        self.robot_pos = (rx, ry)
        self.robot_deg = rdeg
        if len(self.trajectory) == 0 or math.hypot(rx - self.trajectory[-1][0], ry - self.trajectory[-1][1]) > 15.0:
            self.trajectory.append((rx, ry))
            if len(self.trajectory) > 600:
                self.trajectory.pop(0)

    def ingest_boxes(self, boxes, rx, ry, rdeg):
        """
        Převezme Bounding Boxy z aktuálního skenu a transformuje je do souřadnic hřiště.
        - Spáruje známé cíle (aktualizuje EMA polohu).
        - Přidá nově objevené cíle (doplňování při pohybu robota).
        - Zastíněné cíle nezahazuje, ale ponechává je s označením [ZASTÍNĚNÝ].
        """
        self.total_updates += 1
        xmin, xmax, ymin, ymax = self.get_auto_field_bounds()
        margin = 40.0 # Ochranný okraj od mantinelu

        heading_rad = math.radians(rdeg)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        # 1. Transformace aktuálních boxů do souřadnic hřiště
        curr_transformed = []
        rejected = 0
        for b in boxes:
            cx, cy = b['cx'], b['cy']
            wx = rx + cx * cos_h + cy * sin_h
            wy = ry - cx * sin_h + cy * cos_h

            # Kontrola hranic pole: co leží vně mantinelu, je falešný odraz
            if (xmin + margin <= wx <= xmax - margin) and (ymin + margin <= wy <= ymax - margin):
                curr_transformed.append({
                    'wx': wx, 'wy': wy,
                    'width': b['width'], 'height': b['height'],
                    'span_mm': b['span_mm'],
                    'is_solo': b['is_solo'],
                    'pts_count': b['pts_count']
                })
            else:
                rejected += 1

        self.rejected_outside_count = rejected

        # 2. Označení všech dosavadních cílů jako prozatím neviděných v tomto snímku
        for t in self.targets:
            t['is_visible'] = False

        # 3. Párování a doplňování cílů
        matched_target_idxs = set()
        for cb in curr_transformed:
            best_idx = -1
            best_d = 280.0 # Tolerance párování v mm

            for idx, t in enumerate(self.targets):
                if idx in matched_target_idxs: continue
                d = math.hypot(t['x'] - cb['wx'], t['y'] - cb['wy'])
                if d < best_d:
                    best_d = d
                    best_idx = idx

            if best_idx >= 0:
                # Nalezena shoda -> vyhlazení polohy a rozměrů
                t = self.targets[best_idx]
                alpha = 0.15
                t['x'] = (1.0 - alpha) * t['x'] + alpha * cb['wx']
                t['y'] = (1.0 - alpha) * t['y'] + alpha * cb['wy']
                t['width'] = (1.0 - alpha) * t['width'] + alpha * cb['width']
                t['height'] = (1.0 - alpha) * t['height'] + alpha * cb['height']
                t['span_mm'] = math.hypot(t['width'], t['height'])
                t['hits'] += 1
                t['is_visible'] = True
                t['is_solo'] = cb['is_solo']
                t['status'] = "VIDITELNÝ"
                matched_target_idxs.add(best_idx)
            else:
                # Nový cíl -> kontrola minimálního rozestupu od stávajících cílů (min 600 mm)
                too_close = any(math.hypot(cb['wx'] - t['x'], cb['wy'] - t['y']) < 600.0 for t in self.targets)
                if not too_close and len(self.targets) < 4:
                    new_target = {
                        'x': cb['wx'], 'y': cb['wy'],
                        'width': cb['width'], 'height': cb['height'],
                        'span_mm': cb['span_mm'],
                        'hits': 1,
                        'is_visible': True,
                        'is_solo': cb['is_solo'],
                        'status': "VIDITELNÝ"
                    }
                    self.targets.append(new_target)
                    matched_target_idxs.add(len(self.targets) - 1)

        # 4. Pro cíle, které nebyly v tomto skenu spatřeny (zastíněné):
        for idx, t in enumerate(self.targets):
            if not t['is_visible']:
                t['status'] = "ZASTÍNĚNÝ / ZNÁMÝ"

            # Vzdálenost od aktuální pozice robota
            t['dist_from_robot'] = math.hypot(t['x'] - rx, t['y'] - ry)
            t['dist_from_start'] = math.hypot(t['x'], t['y'])

    def get_field_targets(self):
        """Vrátí potvrzené cíle hřiště (seřazené podle vzdálenosti od startu)."""
        valid = [t for t in self.targets if t.get('hits', 0) >= 1]
        valid.sort(key=lambda t: t['dist_from_start'])
        return valid[:4]

    def compute_extinguish_route(self):
        """Vypočítá optimální trasu hašení svíček od aktuální polohy robota."""
        targets = self.get_field_targets()
        if len(targets) < 2:
            return targets

        unvisited = list(targets)
        route = []
        curr_x, curr_y = self.robot_pos

        while unvisited:
            best_idx = 0
            best_d = 999999.0
            for i, t in enumerate(unvisited):
                d = math.hypot(t['x'] - curr_x, t['y'] - curr_y)
                if d < best_d:
                    best_d = d
                    best_idx = i
            nxt = unvisited.pop(best_idx)
            route.append(nxt)
            curr_x, curr_y = nxt['x'], nxt['y']

        return route


# ============================================================
#  Hlavní vícepanelový vizualizér
# ============================================================
class DualLidarVisualizer:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.frozen = False

        # Výchozí parametry
        self.fov_limited = True
        self.auto_fov_enabled = True  # 180° start kužel -> 360° plný rozhled po [L] v poli
        self.min_range = 130
        self.max_range = 2400
        self.min_intensity = 15
        self.view_span_mm = 3500.0

        self.selected_object = None

        self.last_frame = {
            "fps": 0.0, "pts": 0, "calc_us": 0, "heap": 0, "crc_err": 0,
            "boxes": [], "static_boxes": [], "cloud": []
        }

        self.localizer = RobotLocalizer()
        self.field_map = GlobalFieldMap()

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption(f"Kybernoid 27 - LiDAR D200 Bounding Box Localizer [{port}]")
        self.clock = pygame.time.Clock()

        try:
            self.font_main = pygame.font.SysFont("DejaVu Sans Mono", 11)
            self.font_bold = pygame.font.SysFont("DejaVu Sans Mono", 12, bold=True)
            self.font_big  = pygame.font.SysFont("DejaVu Sans Mono", 15, bold=True)
        except:
            self.font_main = pygame.font.SysFont(None, 14)
            self.font_bold = pygame.font.SysFont(None, 16)
            self.font_big  = pygame.font.SysFont(None, 20)

        self.connect_serial()
        # Výchozí konfigurace do ESP32
        self.send_cmd("FOV 1")
        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
        self.send_cmd(f"INTENSITY {self.min_intensity}")

    def connect_serial(self):
        try:
            print(f"[SERIAL] Připojuji k {self.port} při {self.baud} baud...")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.ser.reset_input_buffer()
            print("[SERIAL] Připojeno!")
        except Exception as e:
            print(f"[SERIAL CHYBA] {e}")
            self.ser = None

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd_str.strip() + "\n").encode('utf-8'))
                self.ser.flush()
            except: pass

    def on_parameter_changed(self):
        """Vyresetuje stav map a odešle RESET_MAP do ESP32."""
        self.localizer.unlock_map()
        self.field_map.reset()
        self.send_cmd("RESET_MAP")
        print("[MAPA] Mapy vyresetovány -> sběr čistého etalonu.")

    def to_screen(self, x_mm, y_mm, panel_offset_x):
        cx = panel_offset_x + PANEL_W / 2.0
        cy = PANEL_H / 2.0
        scale = (PANEL_W / 2.0) / self.view_span_mm
        sx = int(cx + x_mm * scale)
        sy = int(cy - y_mm * scale)
        return sx, sy

    def read_serial(self):
        if not self.ser or not self.ser.is_open: return
        newest = None
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("{") and line.endswith("}"):
                    newest = json.loads(line)
            except: pass

        if newest is not None and not self.frozen:
            self.last_frame = newest
            live_boxes = parse_box_list(newest.get("boxes", []))
            static_boxes = parse_box_list(newest.get("static_boxes", []))

            # Pokud ještě není zamčeno [L], průběžně aktualizujeme globální model z pozice START=[0,0]
            if not self.localizer.locked:
                active_b = static_boxes if static_boxes else live_boxes
                if active_b:
                    self.field_map.ingest_boxes(active_b, 0.0, 0.0, 0.0)
            else:
                # Po zamknutí provádíme lokalizaci robota podle aktuálních boxů
                active_b = static_boxes if len(static_boxes) >= 2 else live_boxes
                if len(active_b) >= 2:
                    self.localizer.estimate_pose(active_b)

                loc = self.localizer
                if loc.pose_confidence > 0.3 and len(active_b) >= 1:
                    # Kumulativní plnění hřiště a zachování/doplnění cílů
                    self.field_map.ingest_boxes(active_b, loc.robot_x, loc.robot_y, loc.robot_deg)
                    self.field_map.update_robot_pose(loc.robot_x, loc.robot_y, loc.robot_deg)

    def handle_click(self, pos):
        if pos[1] >= PANEL_H:
            return # Klik ve spodním panelu

        panel_idx = pos[0] // PANEL_W
        offset_x = panel_idx * PANEL_W
        cx = offset_x + PANEL_W / 2.0
        cy = PANEL_H / 2.0
        scale = (PANEL_W / 2.0) / self.view_span_mm
        click_x = (pos[0] - cx) / scale
        click_y = -(pos[1] - cy) / scale

        best_dist = 999999.0
        best_obj = None

        boxes_to_check = parse_box_list(self.last_frame.get("static_boxes", []))
        if not boxes_to_check:
            boxes_to_check = parse_box_list(self.last_frame.get("boxes", []))

        for b in boxes_to_check:
            d = math.hypot(b['cx'] - click_x, b['cy'] - click_y)
            if d < best_dist:
                best_dist = d
                best_obj = (b['cx'], b['cy'], b['dist_mm'])

        if best_dist < 400:
            self.selected_object = best_obj
        else:
            self.selected_object = (click_x, click_y, math.hypot(click_x, click_y))

    def draw_panel_decorations(self, offset_x, panel_idx=0):
        cx = offset_x + PANEL_W // 2
        cy = PANEL_H // 2
        scale = (PANEL_W / 2.0) / self.view_span_mm

        # Osy
        pygame.draw.line(self.screen, GRID_COLOR, (offset_x, cy), (offset_x + PANEL_W, cy), 1)
        pygame.draw.line(self.screen, GRID_COLOR, (cx, 0), (cx, PANEL_H), 1)

        # Orientační kruhy (po 0.5 m až do 3.5 m)
        for d_m in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]:
            r_px = int(d_m * 1000.0 * scale)
            c = CIRCLE_COLOR if d_m in [1.0, 2.0, 3.0] else (30, 34, 42)
            pygame.draw.circle(self.screen, c, (cx, cy), r_px, 1)
            lbl = self.font_main.render(f"{d_m:.1f} m", True, (90, 100, 115))
            self.screen.blit(lbl, (cx + 5, cy - r_px - 8))

        if panel_idx == 0:
            # Panel 1: Robot pevně ve středu
            pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 8)
            pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 35), 3)
        elif panel_idx == 1:
            # Panel 2: Bod START (0,0)
            pygame.draw.circle(self.screen, (0, 180, 220), (cx, cy), 7, 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx - 5, cy), (cx + 5, cy), 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx, cy - 5), (cx, cy + 5), 1)
            self.screen.blit(self.font_main.render("START (0,0)", True, (0, 220, 255)), (cx + 9, cy - 7))
        elif panel_idx == 2:
            # Panel 3: Bod START jako výchozí bod referenčního světa
            pygame.draw.circle(self.screen, (0, 220, 255), (cx, cy), 5)
            self.screen.blit(self.font_main.render("START", True, (0, 200, 240)), (cx + 8, cy - 6))

    # ========================================================
    #  PANEL 1: ŽIVÝ RADAR (LIVE SCAN & BOXY)
    # ========================================================
    def draw_left_live(self):
        offset_x = 0
        self.draw_panel_decorations(offset_x, panel_idx=0)

        # Body mračna
        cloud = self.last_frame.get("cloud", [])
        n = len(cloud) // 3
        for i in range(n):
            x = cloud[i * 3]
            y = cloud[i * 3 + 1]
            intensity = cloud[i * 3 + 2]
            sx, sy = self.to_screen(x, y, offset_x)
            if 0 <= sx < PANEL_W and 0 <= sy < PANEL_H:
                v = min(255, max(30, intensity * 2 + 40))
                self.screen.set_at((sx, sy), (v, v, v))

        # Surové Bounding Boxy z ESP32
        live_boxes = parse_box_list(self.last_frame.get("boxes", []))
        for idx, cl in enumerate(live_boxes):
            p_min = self.to_screen(cl['min_x'], cl['max_y'], offset_x)
            p_max = self.to_screen(cl['max_x'], cl['min_y'], offset_x)
            rx = min(p_min[0], p_max[0])
            ry = min(p_min[1], p_max[1])
            rw = max(8, abs(p_max[0] - p_min[0]))
            rh = max(8, abs(p_max[1] - p_min[1]))

            box_col = (0, 240, 160) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 180, 40)
            pygame.draw.rect(self.screen, box_col, (rx, ry, rw, rh), 2)

            scx, scy = self.to_screen(cl['cx'], cl['cy'], offset_x)
            pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 5)
            pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)

            lbl1 = self.font_bold.render(f"BOX #{idx+1} [{cl['type']}]: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
            lbl2 = self.font_main.render(f"STŘED: [{int(cl['cx'])}, {int(cl['cy'])}] | R:{int(cl['dist_mm'])}mm ({cl['pts_count']}b)", True, (255, 240, 130))
            self.screen.blit(lbl1, (rx + 4, max(4, ry - 18)))
            self.screen.blit(lbl2, (scx + 8, scy - 7))

        # Telemetrický panel vlevo nahoře
        panel = pygame.Surface((290, 140))
        panel.set_alpha(220)
        panel.fill((10, 12, 16))
        self.screen.blit(panel, (12, 12))
        pygame.draw.rect(self.screen, (40, 50, 60), (12, 12, 290, 140), 1)

        f = self.last_frame
        y = 18
        self.screen.blit(self.font_big.render("1. ŽIVÝ RADAR (D200)", True, ACCENT_COLOR), (20, y)); y += 22
        def row(label, val, col=(200, 210, 225)):
            nonlocal y
            self.screen.blit(self.font_main.render(label, True, (130, 140, 155)), (20, y))
            self.screen.blit(self.font_bold.render(str(val), True, col), (150, y))
            y += 18

        row("Frekvence:", f"{f.get('fps', 0.0):.1f} Hz", (100, 255, 120))
        row("Filtrováno bodů:", f"{f.get('pts', 0)} bodů")
        row("Výpočet na ESP32:", f"{f.get('calc_us', 0)} µs", (0, 220, 255))
        row("Volná RAM ESP32:", f"{f.get('heap', 0) // 1024} kB")
        row("Živé boxy (max 4):", f"{len(live_boxes)} cílů", (0, 240, 160))

        # Ovládací HUD vlevo dole
        cmd_panel = pygame.Surface((576, 95))
        cmd_panel.set_alpha(220)
        cmd_panel.fill((10, 12, 16))
        self.screen.blit(cmd_panel, (12, PANEL_H - 105))
        pygame.draw.rect(self.screen, (55, 45, 30), (12, PANEL_H - 105, 576, 95), 1)

        cy = PANEL_H - 99
        self.screen.blit(self.font_bold.render("LADĚNÍ ZA BĚHU (PŘÍKAZY DO ESP32):", True, (240, 200, 80)), (22, cy)); cy += 20
        fov_txt = "[F] FoV: 180° Kužel (Start)" if self.fov_limited else "[F] FoV: 360° Celé pole (V poli)"
        auto_badge = "[AUTO]" if self.auto_fov_enabled else ""
        freeze_txt = "[MEZERNÍK] ODMRAZIT" if self.frozen else "[MEZERNÍK] ZMRAZIT"
        c1 = self.font_main.render(f"{fov_txt} {auto_badge} | {freeze_txt}", True, (220, 220, 220))
        c2 = self.font_main.render(f"Rozsah: [R/E] Min: {self.min_range} mm | [T/G] Max: {self.max_range} mm", True, (170, 180, 195))
        c3 = self.font_main.render(f"Odrazivost: [I/K] Min Intenzita: {self.min_intensity} | [C] Resetovat mapy", True, (170, 180, 195))
        self.screen.blit(c1, (22, cy)); cy += 17
        self.screen.blit(c2, (22, cy)); cy += 17
        self.screen.blit(c3, (22, cy))

        if self.selected_object:
            sel_panel = pygame.Surface((250, 60))
            sel_panel.set_alpha(220)
            sel_panel.fill((25, 20, 10))
            self.screen.blit(sel_panel, (PANEL_W - 262, 12))
            pygame.draw.rect(self.screen, (255, 180, 0), (PANEL_W - 262, 12, 250, 60), 1)
            self.screen.blit(self.font_bold.render("INSPEKCE BODU:", True, (255, 200, 50)), (PANEL_W - 254, 18))
            sx_mm, sy_mm, s_dist = self.selected_object
            self.screen.blit(self.font_main.render(f"X:{sx_mm:.0f} Y:{sy_mm:.0f} | R:{s_dist:.0f}mm", True, (255, 255, 255)), (PANEL_W - 254, 40))

    # ========================================================
    #  PANEL 2: REFERENČNÍ ETALON (START & ODSTUPŇOVANÉ KOTVY)
    # ========================================================
    def draw_mid_original(self):
        offset_x = PANEL_W
        self.draw_panel_decorations(offset_x, panel_idx=1)

        loc = self.localizer
        ref_boxes = loc.ref_boxes if loc.locked else parse_box_list(self.last_frame.get("static_boxes", []))

        # Kotvy a trojúhelník
        if loc.locked and loc.anchors:
            # KOTVA 1 (Zlatá)
            if len(loc.anchors) >= 1:
                a1 = loc.anchors[0]
                p1 = self.to_screen(a1['p1'][0], a1['p1'][1], offset_x)
                p2 = self.to_screen(a1['p2'][0], a1['p2'][1], offset_x)
                pygame.draw.line(self.screen, a1['color'], p1, p2, 4)
                mx = (p1[0] + p2[0]) // 2; my = (p1[1] + p2[1]) // 2
                self.screen.blit(self.font_bold.render(f"KOTVA 1: {int(a1['dist'])} mm", True, a1['color']), (mx + 8, my - 8))

            # KOTVA 2 (Oranžová)
            if len(loc.anchors) >= 2:
                a2 = loc.anchors[1]
                p1 = self.to_screen(a2['p1'][0], a2['p1'][1], offset_x)
                p2 = self.to_screen(a2['p2'][0], a2['p2'][1], offset_x)
                pygame.draw.line(self.screen, a2['color'], p1, p2, 3)
                mx = (p1[0] + p2[0]) // 2; my = (p1[1] + p2[1]) // 2
                self.screen.blit(self.font_bold.render(f"KOTVA 2: {int(a2['dist'])} mm", True, a2['color']), (mx + 8, my + 8))

            # Trojúhelníkový vzorec (spojnice)
            if loc.triangle_pattern:
                tp = loc.triangle_pattern
                p1 = self.to_screen(tp['p1_pt'][0], tp['p1_pt'][1], offset_x)
                p2 = self.to_screen(tp['p2_pt'][0], tp['p2_pt'][1], offset_x)
                pygame.draw.line(self.screen, (0, 220, 255), p1, p2, 2)
                mx = (p1[0] + p2[0]) // 2; my = (p1[1] + p2[1]) // 2
                self.screen.blit(self.font_main.render(f"{int(tp['dist3'])} mm ({tp['angle_deg']:.0f}°)", True, (0, 220, 255)), (mx - 20, my - 16))

        # Vykreslení referenčních Bounding Boxů
        for idx, cl in enumerate(ref_boxes):
            p_min = self.to_screen(cl['min_x'], cl['max_y'], offset_x)
            p_max = self.to_screen(cl['max_x'], cl['min_y'], offset_x)
            rx = min(p_min[0], p_max[0])
            ry = min(p_min[1], p_max[1])
            rw = max(10, abs(p_max[0] - p_min[0]))
            rh = max(10, abs(p_max[1] - p_min[1]))

            box_col = (0, 240, 160) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 180, 40)
            s_fill = pygame.Surface((rw, rh), pygame.SRCALPHA)
            fill_col = (0, 200, 140, 30) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 160, 30, 30)
            s_fill.fill(fill_col)
            self.screen.blit(s_fill, (rx, ry))
            pygame.draw.rect(self.screen, box_col, (rx, ry, rw, rh), 2)

            scx, scy = self.to_screen(cl['cx'], cl['cy'], offset_x)
            pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 6)
            pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)

            lbl_t = self.font_bold.render(f"ETALON #{idx+1} [{cl['type']}]: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
            lbl_c = self.font_main.render(f"X:{int(cl['cx']):+d} Y:{int(cl['cy']):+d} | R:{int(cl['dist_mm'])} mm", True, (255, 245, 140))
            self.screen.blit(lbl_t, (rx + 4, max(4, ry - 18)))
            self.screen.blit(lbl_c, (scx + 8, scy - 7))

        # Horní status panel
        stat_box = pygame.Surface((570, 56))
        stat_box.set_alpha(220)
        stat_box.fill((10, 12, 16))
        self.screen.blit(stat_box, (offset_x + 15, 12))
        border_col = (80, 255, 160) if loc.locked else (70, 60, 40)
        pygame.draw.rect(self.screen, border_col, (offset_x + 15, 12, 570, 56), 1)

        lock_badge = " 🔒 ZAMČENO" if loc.locked else " [L] ZAMKNOUT MAPU"
        self.screen.blit(self.font_big.render(f"2. REFERENČNÍ ETALON (START){lock_badge}", True, (80, 255, 160) if loc.locked else ACCENT_MAP), (offset_x + 25, 17))
        tri_txt = " | Trojúhelník: AKTIVNÍ" if (loc.locked and loc.triangle_pattern) else ""
        sub_str = f"Kotvy: {len(loc.anchors)} | Referenční boxy: {len(ref_boxes)}/4{tri_txt}" if loc.locked else f"Sběr etalonu na startu... Boxů: {len(ref_boxes)}/4 (stiskněte [L])"
        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (offset_x + 25, 39))

        # Spodní tip panel
        tip_box = pygame.Surface((570, 75))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (offset_x + 15, PANEL_H - 85))
        pygame.draw.rect(self.screen, (50, 45, 30), (offset_x + 15, PANEL_H - 85, 570, 75), 1)
        self.screen.blit(self.font_bold.render("REFERENČNÍ ETALON (START 180°):", True, (240, 200, 80)), (offset_x + 25, PANEL_H - 79))
        self.screen.blit(self.font_main.render("• Čistý sken 180° dopředu ze startovní zdi eliminuje falešné odrazy za mantinelem.", True, (200, 210, 225)), (offset_x + 25, PANEL_H - 61))
        self.screen.blit(self.font_main.render("• [L] Zamknout etalon -> automaticky aktivuje 360° plný rozhled v poli!", True, (100, 255, 140)), (offset_x + 25, PANEL_H - 43))

    # ========================================================
    #  PANEL 3: POROVNÁNÍ & LOKALIZACE ROBOTA (DRUHÁ STATICKÁ MAPA & KOTVY)
    # ========================================================
    def draw_right_comparison(self):
        offset_x = PANEL_W * 2
        self.draw_panel_decorations(offset_x, panel_idx=2)
        loc = self.localizer

        if not loc.locked:
            msg_box = pygame.Surface((540, 130))
            msg_box.set_alpha(220)
            msg_box.fill((12, 14, 18))
            bx = offset_x + 30; by = PANEL_H // 2 - 65
            self.screen.blit(msg_box, (bx, by))
            pygame.draw.rect(self.screen, (70, 60, 40), (bx, by, 540, 130), 1)
            self.screen.blit(self.font_big.render("3. LOKALIZACE ROBOTA & DRUHÁ STATICKÁ MAPA", True, (200, 200, 200)), (bx + 20, by + 12))
            self.screen.blit(self.font_bold.render("Čekám na zamknutí etalonu na startu...", True, (240, 180, 50)), (bx + 20, by + 36))
            self.screen.blit(self.font_main.render("1. Nechte na startu ustálit boxy v panelu 2.", True, (160, 170, 185)), (bx + 20, by + 60))
            self.screen.blit(self.font_main.render("2. Stiskněte [L] pro zamknutí etalonu a aktivaci lokalizace.", True, (80, 255, 160)), (bx + 20, by + 80))
            self.screen.blit(self.font_main.render("3. Zde se vykreslí druhá statická mapa z nového místa a kotvy.", True, (255, 120, 220)), (bx + 20, by + 100))
            return

        # 1. GHOST STÍNY REFERENČNÍHO ETALONU ZE STARTU (pro vizuální porovnání)
        for idx, rb in enumerate(loc.ref_boxes):
            p_min = self.to_screen(rb['min_x'], rb['max_y'], offset_x)
            p_max = self.to_screen(rb['max_x'], rb['min_y'], offset_x)
            rx = min(p_min[0], p_max[0])
            ry = min(p_min[1], p_max[1])
            rw = max(8, abs(p_max[0] - p_min[0]))
            rh = max(8, abs(p_max[1] - p_min[1]))
            pygame.draw.rect(self.screen, (60, 75, 95), (rx, ry, rw, rh), 1)
            scx, scy = self.to_screen(rb['cx'], rb['cy'], offset_x)
            pygame.draw.circle(self.screen, (70, 85, 105), (scx, scy), 4, 1)
            self.screen.blit(self.font_main.render(f"REF #{idx+1}", True, (90, 105, 125)), (rx + 2, max(4, ry - 14)))

        # 2. VÝPOČET POLOHY AKTUÁLNÍCH BOUNDING BOXŮ Z NOVÉHO MÍSTA
        curr_boxes = parse_box_list(self.last_frame.get("static_boxes", []))
        if not curr_boxes:
            curr_boxes = parse_box_list(self.last_frame.get("boxes", []))

        cos_t = math.cos(math.radians(loc.robot_deg))
        sin_t = math.sin(math.radians(loc.robot_deg))

        transformed_boxes = []
        for cl in curr_boxes:
            cx_local = cl['cx']
            cy_local = cl['cy']
            # Střed boxu ve světě
            wx = loc.robot_x + cx_local * cos_t + cy_local * sin_t
            wy = loc.robot_y - cx_local * sin_t + cy_local * cos_t

            # 4 rohy orientovaného Bounding Boxu
            hw = cl['width'] * 0.5
            hh = cl['height'] * 0.5
            local_corners = [
                (cx_local - hw, cy_local - hh),
                (cx_local + hw, cy_local - hh),
                (cx_local + hw, cy_local + hh),
                (cx_local - hw, cy_local + hh)
            ]
            poly_scr = []
            for lx, ly in local_corners:
                g_cx = loc.robot_x + lx * cos_t + ly * sin_t
                g_cy = loc.robot_y - lx * sin_t + ly * cos_t
                poly_scr.append(self.to_screen(g_cx, g_cy, offset_x))

            transformed_boxes.append({
                'box': cl,
                'wx': wx, 'wy': wy,
                'poly_scr': poly_scr
            })

        # 3. KOTVY A VZDÁLENOSTI MEZI NOVÝMI BOUNDING BOXY (PŘÍMKY V POLI)
        n_tb = len(transformed_boxes)
        for i in range(n_tb):
            for j in range(i + 1, n_tb):
                b1 = transformed_boxes[i]
                b2 = transformed_boxes[j]
                d_cur = math.hypot(b1['wx'] - b2['wx'], b1['wy'] - b2['wy'])
                p1 = self.to_screen(b1['wx'], b1['wy'], offset_x)
                p2 = self.to_screen(b2['wx'], b2['wy'], offset_x)

                # Porovnání s referenčními kotvami
                is_k1 = False
                is_k2 = False
                if len(loc.anchors) >= 1 and abs(d_cur - loc.anchors[0]['dist']) < 130.0:
                    is_k1 = True
                elif len(loc.anchors) >= 2 and abs(d_cur - loc.anchors[1]['dist']) < 130.0:
                    is_k2 = True

                mx = (p1[0] + p2[0]) // 2
                my = (p1[1] + p2[1]) // 2

                if is_k1:
                    # KOTVA 1 (Zlatá)
                    pygame.draw.line(self.screen, (255, 215, 0), p1, p2, 4)
                    lbl = self.font_bold.render(f"KOTVA 1: {int(d_cur)} mm", True, (255, 215, 0))
                    self.screen.blit(lbl, (mx + 8, my - 16))
                elif is_k2:
                    # KOTVA 2 (Oranžová)
                    pygame.draw.line(self.screen, (255, 150, 40), p1, p2, 3)
                    lbl = self.font_bold.render(f"KOTVA 2: {int(d_cur)} mm", True, (255, 150, 40))
                    self.screen.blit(lbl, (mx + 8, my + 6))
                else:
                    # Další spojnice
                    pygame.draw.line(self.screen, (0, 180, 220), p1, p2, 1)
                    lbl = self.font_main.render(f"{int(d_cur)} mm", True, (0, 200, 240))
                    self.screen.blit(lbl, (mx + 6, my - 8))

        # 4. ROZHODOVACÍ PAPRSKY SHOD MEZI INLIERY A REFERENČNÍMI BODY
        match = loc.last_match_info
        if match and match.get('inlier_pairs'):
            for ref_pt, cur_trans_pt, err in match['inlier_pairs']:
                rp = self.to_screen(ref_pt[0], ref_pt[1], offset_x)
                cp = self.to_screen(cur_trans_pt[0], cur_trans_pt[1], offset_x)
                pygame.draw.line(self.screen, (255, 60, 200), rp, cp, 2)
                pygame.draw.circle(self.screen, (255, 200, 240), cp, 4)
                mid_x = (rp[0] + cp[0]) // 2
                mid_y = (rp[1] + cp[1]) // 2
                self.screen.blit(self.font_main.render(f"Δ{err:.0f}mm", True, (255, 130, 220)), (mid_x + 4, mid_y - 6))

        # 5. VYKRESLENÍ PLNÝCH ORIENTOVANÝCH BOUNDING BOXŮ Z NOVÉHO MÍSTA
        for idx, tb in enumerate(transformed_boxes):
            cl = tb['box']
            wx = tb['wx']
            wy = tb['wy']
            poly_scr = tb['poly_scr']

            box_col = (0, 240, 160) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 180, 40)

            # Poloprůhledná výplň orientovaného boxu
            s_poly = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
            fill_col = (0, 200, 140, 35) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 160, 30, 35)
            local_poly = [(pt[0] - offset_x, pt[1]) for pt in poly_scr]
            pygame.draw.polygon(s_poly, fill_col, local_poly)
            pygame.draw.polygon(s_poly, box_col, local_poly, 2)
            self.screen.blit(s_poly, (offset_x, 0))

            # Středový křížek
            scx, scy = self.to_screen(wx, wy, offset_x)
            pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 6)
            pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (scx - 7, scy), (scx + 7, scy), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (scx, scy - 7), (scx, scy + 7), 2)

            # Popisky nového boxu
            lbl1 = self.font_bold.render(f"NOVÝ #{idx+1} [{cl['type']}]: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
            lbl2 = self.font_main.render(f"SVĚT: [{int(wx):+d}, {int(wy):+d}] | R od robota:{int(cl['dist_mm'])} mm", True, (255, 245, 140))
            self.screen.blit(lbl1, (scx + 10, scy - 18))
            self.screen.blit(lbl2, (scx + 10, scy - 4))

        # 6. TRAJEKTORIE ROBOTA
        if len(loc.trajectory) >= 2:
            traj_px = [self.to_screen(tp[0], tp[1], offset_x) for tp in loc.trajectory]
            pygame.draw.lines(self.screen, (220, 50, 140), False, traj_px, 2)
            for tpt in traj_px[::4]:
                pygame.draw.circle(self.screen, (255, 100, 180), tpt, 2)

        # 7. RŮŽOVÝ ROBOT S NATOČENÍM
        rx, ry = self.to_screen(loc.robot_x, loc.robot_y, offset_x)
        h_rad = math.radians(loc.robot_deg)
        cos_h = math.cos(h_rad); sin_h = math.sin(h_rad)

        # Zorný kužel dopředu
        cone_r_mm = 650.0
        cone_pts = [(rx, ry)]
        for deg_off in range(-20, 21, 5):
            a_rad = h_rad + math.radians(deg_off)
            kx = loc.robot_x + cone_r_mm * math.sin(a_rad)
            ky = loc.robot_y + cone_r_mm * math.cos(a_rad)
            cone_pts.append(self.to_screen(kx, ky, offset_x))
        if len(cone_pts) > 2:
            s_cone = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
            cone_local = [(pt[0] - offset_x, pt[1]) for pt in cone_pts]
            pygame.draw.polygon(s_cone, (255, 60, 170, 25), cone_local)
            pygame.draw.lines(s_cone, (255, 80, 190, 80), False, cone_local, 1)
            self.screen.blit(s_cone, (offset_x, 0))

        # Šasi robota (čtverec 240 x 240 mm)
        hw_mm = 120.0
        local_box = [(-hw_mm, -hw_mm), (hw_mm, -hw_mm), (hw_mm, hw_mm), (-hw_mm, hw_mm)]
        box_px = []
        for (bx, by) in local_box:
            gx = loc.robot_x + bx * cos_h + by * sin_h
            gy = loc.robot_y - bx * sin_h + by * cos_h
            box_px.append(self.to_screen(gx, gy, offset_x))
        pygame.draw.polygon(self.screen, PINK_BG, box_px, 0)
        pygame.draw.polygon(self.screen, PINK_MAIN, box_px, 2)

        # Šipka pohledu dopředu
        tip_x = int(rx + 26 * sin_h)
        tip_y = int(ry - 26 * cos_h)
        pygame.draw.line(self.screen, PINK_MAIN, (rx, ry), (tip_x, tip_y), 3)

        # Popisek robota
        conf_pct = int(loc.pose_confidence * 100)
        lbl_rob = self.font_bold.render(f"ROBOT [{loc.robot_x:+.0f}, {loc.robot_y:+.0f}] {loc.robot_deg:+.0f}° ({conf_pct}%)", True, PINK_LITE)
        self.screen.blit(lbl_rob, (rx + 16, ry - 10))

        # 8. HORNÍ STATUS PANEL
        stat_box = pygame.Surface((570, 56))
        stat_box.set_alpha(220)
        stat_box.fill((10, 12, 16))
        self.screen.blit(stat_box, (offset_x + 15, 12))
        pygame.draw.rect(self.screen, (80, 255, 160), (offset_x + 15, 12, 570, 56), 1)

        pat_name = match.get('pattern_name', 'HLEDÁM SHODU...') if match else 'HLEDÁM SHODU...'
        inliers = match.get('inliers', 0) if match else 0
        err = match.get('avg_error', 0.0) if match else 0.0
        dist_from_start = math.hypot(loc.robot_x, loc.robot_y)
        fov_tag = "360° V POLI" if not self.fov_limited else "180° START"

        self.screen.blit(self.font_big.render(f"3. SHODA [{fov_tag}]: {pat_name}", True, (80, 255, 160)), (offset_x + 25, 17))
        sub_str = f"Inliery: {inliers} | Chyba: {err:.1f} mm | Od startu: {dist_from_start:.0f} mm | Jistota: {conf_pct}%"
        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (offset_x + 25, 38))

        # 9. SPODNÍ TIP PANEL
        tip_box = pygame.Surface((570, 75))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (offset_x + 15, PANEL_H - 85))
        pygame.draw.rect(self.screen, (60, 30, 50), (offset_x + 15, PANEL_H - 85, 570, 75), 1)
        self.screen.blit(self.font_bold.render("DRUHÁ STATICKÁ MAPA & KOTVY Z NOVÉHO MÍSTA:", True, (255, 150, 215)), (offset_x + 25, PANEL_H - 79))
        self.screen.blit(self.font_main.render("• Zlatá: KOTVA 1 (nejdelší rozvor) | Oranžová: KOTVA 2 | Růžová: paprsky shod", True, (240, 220, 80)), (offset_x + 25, PANEL_H - 61))
        self.screen.blit(self.font_main.render("• Šedé obdélníky = etalon ze STARTu | Barevné obdélníky = boxy z nového místa", True, (100, 255, 140)), (offset_x + 25, PANEL_H - 43))

    # ========================================================
    #  PANEL 4: OBDÉLNÍKOVÁ MAPA CELÉHO HŘIŠTĚ (KUMULATIVNÍ CÍLE)
    # ========================================================
    def draw_field_map(self):
        fm = self.field_map
        y_offset = PANEL_H
        xmin, xmax, ymin, ymax = fm.get_auto_field_bounds()

        # Pozadí
        pygame.draw.rect(self.screen, (12, 14, 18), pygame.Rect(0, y_offset, WIN_W, FIELD_H))

        # ========================================================
        # 1. LEVÝ ODDÍL: GRAFICKÝ OBDÉLNÍK HŘIŠTĚ
        # ========================================================
        avail_w = 510.0
        avail_h = float(FIELD_H - 58)

        field_w_mm = xmax - xmin
        field_h_mm = ymax - ymin
        scale_x = avail_w / max(100.0, field_w_mm)
        scale_y = avail_h / max(100.0, field_h_mm)
        draw_scale = min(scale_x, scale_y)

        draw_w = field_w_mm * draw_scale
        draw_h = field_h_mm * draw_scale
        field_x0 = 20 + int((avail_w - draw_w) * 0.5)
        field_y0 = y_offset + 10 + int((avail_h - draw_h) * 0.5)

        def f_to_screen(gx, gy):
            sx = int(field_x0 + (gx - xmin) * draw_scale)
            sy = int(field_y0 + draw_h - (gy - ymin) * draw_scale)
            return sx, sy

        # Rám hřiště
        f_rect_pts = [
            f_to_screen(xmin, ymin),
            f_to_screen(xmax, ymin),
            f_to_screen(xmax, ymax),
            f_to_screen(xmin, ymax)
        ]
        pygame.draw.polygon(self.screen, (16, 32, 22), f_rect_pts, 0)
        pygame.draw.polygon(self.screen, (60, 180, 100), f_rect_pts, 2)

        # Síť po 500 mm
        gx = math.ceil(xmin / 500.0) * 500.0
        while gx < xmax:
            p1 = f_to_screen(gx, ymin); p2 = f_to_screen(gx, ymax)
            pygame.draw.line(self.screen, (28, 62, 40), p1, p2, 1)
            gx += 500.0
        gy = math.ceil(ymin / 500.0) * 500.0
        while gy < ymax:
            p1 = f_to_screen(xmin, gy); p2 = f_to_screen(xmax, gy)
            pygame.draw.line(self.screen, (28, 62, 40), p1, p2, 1)
            gy += 500.0

        # Kóty hřiště
        lbl_w = self.font_main.render(f"{field_w_mm:.0f} mm", True, (80, 160, 110))
        self.screen.blit(lbl_w, ((f_rect_pts[0][0] + f_rect_pts[1][0]) // 2 - lbl_w.get_width() // 2, f_rect_pts[0][1] + 2))
        lbl_h = self.font_main.render(f"{field_h_mm:.0f} mm", True, (80, 160, 110))
        self.screen.blit(lbl_h, (f_rect_pts[0][0] - lbl_h.get_width() - 4, (f_rect_pts[0][1] + f_rect_pts[3][1]) // 2 - 6))

        # START pozice [0, 0]
        start_sx, start_sy = f_to_screen(0.0, 0.0)
        pygame.draw.circle(self.screen, (0, 180, 255), (start_sx, start_sy), 6, 1)
        pygame.draw.line(self.screen, (0, 220, 255), (start_sx - 6, start_sy), (start_sx + 6, start_sy), 1)
        pygame.draw.line(self.screen, (0, 220, 255), (start_sx, start_sy - 6), (start_sx, start_sy + 6), 1)
        self.screen.blit(self.font_main.render("START (0,0)", True, (0, 200, 255)), (start_sx + 8, start_sy - 6))

        # Trajektorie robota na mapě hřiště
        if len(fm.trajectory) >= 2:
            traj_px = [f_to_screen(tp[0], tp[1]) for tp in fm.trajectory]
            pygame.draw.lines(self.screen, (230, 60, 150), False, traj_px, 2)

        # Aktuální pozice robota v poli
        rx, ry = fm.robot_pos
        if math.hypot(rx, ry) > 15.0 or abs(fm.robot_deg) > 2.0:
            rob_sx, rob_sy = f_to_screen(rx, ry)
            h_rad = math.radians(fm.robot_deg)
            pygame.draw.circle(self.screen, (255, 80, 180), (rob_sx, rob_sy), 5)
            tip_x = int(rob_sx + 14 * math.sin(h_rad))
            tip_y = int(rob_sy - 14 * math.cos(h_rad))
            pygame.draw.line(self.screen, (255, 120, 220), (rob_sx, rob_sy), (tip_x, tip_y), 2)

        # Trasa hašení svíček (optimalizované pořadí)
        route = fm.compute_extinguish_route()
        if len(route) >= 2:
            r_pts = [f_to_screen(rx, ry)] + [f_to_screen(t['x'], t['y']) for t in route]
            pygame.draw.lines(self.screen, (255, 215, 0), False, r_pts, 2)

        # Bounding Boxy uvnitř pole (kumulativní cíle)
        field_targets = fm.get_field_targets()
        for idx, t in enumerate(field_targets):
            csx, csy = f_to_screen(t['x'], t['y'])
            is_vis = t.get('is_visible', True)

            # Rámeček boxu v měřítku hřiště
            bw = max(8, int(t.get('width', 300) * draw_scale))
            bh = max(8, int(t.get('height', 300) * draw_scale))
            box_rect = pygame.Rect(csx - bw // 2, csy - bh // 2, bw, bh)

            if is_vis:
                box_color = (0, 240, 160) if not t['is_solo'] else (255, 180, 40)
                pygame.draw.rect(self.screen, box_color, box_rect, 2)
                pygame.draw.circle(self.screen, (255, 230, 20), (csx, csy), 5)
            else:
                # ZASTÍNĚNÝ CÍL (doplněný z paměti, aktuálně skrytý)
                pygame.draw.rect(self.screen, (160, 140, 60), box_rect, 1)
                pygame.draw.circle(self.screen, (180, 120, 40), (csx, csy), 4)

            lbl_tag = f"C{idx+1}" if is_vis else f"C{idx+1} [Z]"
            lbl_col = (255, 230, 180) if is_vis else (180, 170, 120)
            self.screen.blit(self.font_bold.render(lbl_tag, True, lbl_col), (csx + bw // 2 + 3, csy - 7))

        # Tlačítka startovní stěny pod grafickým obdélníkem
        w_active = fm.start_wall_mode
        info_y = field_y0 + draw_h + 8
        self.screen.blit(self.font_bold.render("START. STĚNA:", True, (200, 210, 225)), (field_x0, info_y))
        tx = field_x0 + 105

        is_2 = (w_active == 2)
        r2 = pygame.Rect(tx, info_y - 2, 140, 18)
        pygame.draw.rect(self.screen, (30, 75, 45) if is_2 else (22, 26, 32), r2, 0, 3)
        pygame.draw.rect(self.screen, (100, 255, 140) if is_2 else (45, 52, 65), r2, 1, 3)
        self.screen.blit(self.font_bold.render("[2/ě] 2.4 m KRATŠÍ" + (" ✓" if is_2 else ""), True, (100, 255, 140) if is_2 else (130, 140, 155)), (tx + 4, info_y))
        tx += 148

        is_3 = (w_active == 3)
        r3 = pygame.Rect(tx, info_y - 2, 130, 18)
        pygame.draw.rect(self.screen, (30, 75, 45) if is_3 else (22, 26, 32), r3, 0, 3)
        pygame.draw.rect(self.screen, (100, 255, 140) if is_3 else (45, 52, 65), r3, 1, 3)
        self.screen.blit(self.font_bold.render("[3/š] 3.5 m DELŠÍ" + (" ✓" if is_3 else ""), True, (100, 255, 140) if is_3 else (130, 140, 155)), (tx + 4, info_y))

        # ========================================================
        # 2. STŘEDNÍ ODDÍL: KONTROLA PRAVIDEL SOUTĚŽE
        # ========================================================
        mid_x = 550
        pygame.draw.line(self.screen, (40, 48, 60), (mid_x - 12, y_offset), (mid_x - 12, y_offset + FIELD_H), 1)

        t_y = y_offset + 10
        self.screen.blit(self.font_big.render("GEOMETRICKÁ PRAVIDLA SOUTĚŽE", True, (255, 215, 80)), (mid_x, t_y))
        t_y += 24

        n_t = len(field_targets)

        # 1. Počet cílů (max 4)
        c_ok = (n_t == 4)
        sym1 = "✓" if c_ok else ("..." if n_t < 4 else "✗")
        col1 = (100, 255, 120) if c_ok else ((240, 180, 50) if n_t < 4 else (255, 70, 70))
        self.screen.blit(self.font_bold.render(f"{sym1} POČET CÍLŮ: {n_t}/4 ({'SPLNĚNO' if c_ok else f'nalezeno {n_t}'})", True, col1), (mid_x, t_y)); t_y += 18

        # 2. Hranice hřiště (100% uvnitř pole)
        rej_cnt = fm.rejected_outside_count
        sym_in = "✓" if (rej_cnt == 0) else "✗"
        col_in = (100, 255, 120) if (rej_cnt == 0) else (255, 70, 70)
        msg_in = "vše uvnitř pole" if rej_cnt == 0 else f"{rej_cnt} bodů vně mantinelu odfiltrováno"
        self.screen.blit(self.font_bold.render(f"{sym_in} HRANICE HŘIŠTĚ: 100% UVNITŘ ({msg_in})", True, col_in), (mid_x, t_y)); t_y += 18

        # 3. Minimální rozestup 600-800 mm
        dist_ok = True
        min_pair_d = 9999.0
        if n_t >= 2:
            for i in range(n_t):
                for j in range(i + 1, n_t):
                    d = math.hypot(field_targets[i]['x'] - field_targets[j]['x'],
                                   field_targets[i]['y'] - field_targets[j]['y'])
                    if d < min_pair_d: min_pair_d = d
                    if d < 600.0: dist_ok = False
        sym2 = "✓" if dist_ok else "✗"
        col2 = (100, 255, 120) if dist_ok else (255, 70, 70)
        d_str = f"(min {min_pair_d:.0f} mm)" if n_t >= 2 else "(čekám na další cíle)"
        self.screen.blit(self.font_bold.render(f"{sym2} MIN. ROZESTUP 600 mm: {'SPLNĚNO' if dist_ok else 'PORUŠENO'} {d_str}", True, col2), (mid_x, t_y)); t_y += 18

        # 4. Stav doplňování cílů a zastínění
        n_vis = sum(1 for t in field_targets if t.get('is_visible', True))
        n_occ = n_t - n_vis
        sym4 = "✓" if n_occ == 0 else "ℹ"
        col4 = (100, 255, 120) if n_occ == 0 else (240, 200, 80)
        occ_txt = f"{n_vis} viditelných, {n_occ} zastíněných (uchováno v paměti)" if n_occ > 0 else f"všech {n_t} cílů přímo viditelných"
        self.screen.blit(self.font_bold.render(f"{sym4} STAV ZASTÍNĚNÍ: {occ_txt}", True, col4), (mid_x, t_y)); t_y += 24

        # Tipy pod pravidly
        self.screen.blit(self.font_bold.render("NÁPOVĚDA K SOUTĚŽI:", True, (200, 210, 225)), (mid_x, t_y)); t_y += 18
        self.screen.blit(self.font_main.render("• [2] Kratší stěna 2.4 m | [3] Delší stěna 3.5 m | [P] Přepnout", True, (160, 175, 195)), (mid_x, t_y)); t_y += 16
        self.screen.blit(self.font_main.render("• Zastíněný cíl zůstává v mapě: robot o něm ví a neztratí ho!", True, (100, 255, 140)), (mid_x, t_y)); t_y += 16
        self.screen.blit(self.font_main.render("• Žlutá lomená čára ukazuje optimální trasu hašení svíček.", True, (255, 215, 80)), (mid_x, t_y))

        # ========================================================
        # 3. PRAVÝ ODDÍL: SEZNAM CÍLŮ V POLI & TRASA HAŠENÍ
        # ========================================================
        right_x = 1180
        pygame.draw.line(self.screen, (40, 48, 60), (right_x - 12, y_offset), (right_x - 12, y_offset + FIELD_H), 1)

        r_y = y_offset + 10
        self.screen.blit(self.font_big.render("SEZNAM CÍLŮ V POLI & OPTIMÁLNÍ TRASA", True, (0, 220, 255)), (right_x, r_y))
        r_y += 24

        if not field_targets:
            self.screen.blit(self.font_main.render("Zatím nebyly detekovány žádné cíle uvnitř hřiště.", True, (150, 160, 175)), (right_x, r_y))
        else:
            for idx, t in enumerate(route):
                is_vis = t.get('is_visible', True)
                status_txt = "VIDITELNÝ" if is_vis else "ZASTÍNĚNÝ (ZNÁMÝ)"
                col_st = (100, 255, 120) if is_vis else (240, 180, 50)

                t_name = f"CÍL #{idx+1} [POŘADÍ {idx+1}]"
                self.screen.blit(self.font_bold.render(t_name, True, (255, 215, 0)), (right_x, r_y))
                self.screen.blit(self.font_bold.render(f"[{status_txt}]", True, col_st), (right_x + 160, r_y))
                r_y += 17

                pos_str = f"Pozice: X={t['x']:+.0f} mm, Y={t['y']:+.0f} mm | R od robota: {t.get('dist_from_robot', 0):.0f} mm"
                self.screen.blit(self.font_main.render(pos_str, True, (210, 215, 225)), (right_x + 10, r_y))
                r_y += 18

    # ========================================================
    #  Hlavní smyčka
    # ========================================================
    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.handle_click(event.pos)
                    elif event.button == 4:
                        self.view_span_mm = max(1200.0, self.view_span_mm - 300.0)
                    elif event.button == 5:
                        self.view_span_mm = min(6000.0, self.view_span_mm + 300.0)

                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        self.frozen = not self.frozen
                    elif event.key in [pygame.K_c, pygame.K_m]:
                        self.on_parameter_changed()
                    elif event.key == pygame.K_f:
                        # Ruční přepnutí FoV 180° <-> 360°
                        self.fov_limited = not self.fov_limited
                        self.send_cmd(f"FOV {1 if self.fov_limited else 0}")
                        print(f"[FoV] Přepnuto na: {'180° Kužel (Start)' if self.fov_limited else '360° Celé pole'}")
                    elif event.key == pygame.K_l:
                        # ZAMKNUTÍ / ODEMKNUTÍ ETALONU
                        if not self.localizer.locked:
                            boxes = parse_box_list(self.last_frame.get("static_boxes", []))
                            if not boxes:
                                boxes = parse_box_list(self.last_frame.get("boxes", []))

                            ok = self.localizer.lock_map(boxes)
                            if ok:
                                self.field_map.reset()
                                self.field_map.ingest_boxes(boxes, 0.0, 0.0, 0.0)
                                print(f"[LOC] Etalon uzamčen ({len(boxes)} cílů ze startu).")

                                # Automatické přepnutí na 360° rozhled v poli
                                if self.auto_fov_enabled and self.fov_limited:
                                    self.fov_limited = False
                                    self.send_cmd("FOV 0")
                                    print("[FoV AUTO] Vyjíždíme do pole -> Automaticky aktivuji 360° PLNÝ ROZHLED!")
                            else:
                                print(f"[LOC] Nedostatek cílů ({len(boxes)}) – nechte stabilizovat boxy na startu.")
                        else:
                            self.localizer.unlock_map()
                            self.field_map.reset()
                            if self.auto_fov_enabled and not self.fov_limited:
                                self.fov_limited = True
                                self.send_cmd("FOV 1")
                                print("[FoV AUTO] Návrat na start -> Automaticky aktivuji 180° DOPŘEDNÝ KUŽEL!")
                            print("[LOC] Etalon odemčen. Můžete znovu nasbírat výchozí mapu.")

                    elif event.key == pygame.K_n:
                        self.field_map.reset()
                        print("[MAPA] Nová mapa na aktuální pozici robota resetována.")
                    elif event.key in [pygame.K_2, pygame.K_AT]:
                        self.field_map.set_start_wall(2)
                        print("[HŘIŠTĚ] Nastavena startovní stěna 2.4 m (Kratší)")
                    elif event.key in [pygame.K_3, pygame.K_HASH]:
                        self.field_map.set_start_wall(3)
                        print("[HŘIŠTĚ] Nastavena startovní stěna 3.5 m (Delší)")
                    elif event.key == pygame.K_p:
                        m = self.field_map.toggle_start_wall()
                        print(f"[HŘIŠTĚ] Startovní stěna přepnuta na: {m}.{4 if m==2 else 5} m")

                    elif event.key == pygame.K_r:
                        self.min_range = min(1000, self.min_range + 50)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                        self.on_parameter_changed()
                    elif event.key == pygame.K_e:
                        self.min_range = max(50, self.min_range - 50)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                        self.on_parameter_changed()
                    elif event.key == pygame.K_t:
                        self.max_range = min(6000, self.max_range + 200)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                        self.on_parameter_changed()
                    elif event.key == pygame.K_g:
                        self.max_range = max(500, self.max_range - 200)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                        self.on_parameter_changed()
                    elif event.key == pygame.K_i:
                        self.min_intensity = min(200, self.min_intensity + 5)
                        self.send_cmd(f"INTENSITY {self.min_intensity}")
                        self.on_parameter_changed()
                    elif event.key == pygame.K_k:
                        self.min_intensity = max(0, self.min_intensity - 5)
                        self.send_cmd(f"INTENSITY {self.min_intensity}")
                        self.on_parameter_changed()
                    elif event.key in [pygame.K_PLUS, pygame.K_EQUALS]:
                        self.view_span_mm = max(1200.0, self.view_span_mm - 300.0)
                    elif event.key in [pygame.K_MINUS]:
                        self.view_span_mm = min(6000.0, self.view_span_mm + 300.0)

            self.read_serial()

            self.screen.fill(BG_COLOR)
            self.draw_left_live()
            self.draw_mid_original()
            self.draw_right_comparison()

            # Vertikální předěly mezi horními panely
            pygame.draw.line(self.screen, (40, 48, 60), (PANEL_W, 0), (PANEL_W, PANEL_H), 2)
            pygame.draw.line(self.screen, (40, 48, 60), (PANEL_W * 2, 0), (PANEL_W * 2, PANEL_H), 2)

            # Horizontální předěl nad spodní mapou hřiště
            pygame.draw.line(self.screen, (60, 75, 95), (0, PANEL_H), (WIN_W, PANEL_H), 3)

            # Spodní mapa hřiště
            self.draw_field_map()

            pygame.display.flip()
            self.clock.tick(60)

        if self.ser: self.ser.close()
        pygame.quit()


if __name__ == "__main__":
    port = get_serial_port()
    baud = get_baudrate()
    app = DualLidarVisualizer(port, baud)
    app.run()
