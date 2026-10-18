#!/usr/bin/env python3
"""
LIDAR_SVICKY_27 - Dvoupanelový vizualizační terminál (Kybernoid 27)
Autor: Antigravity & Nik

Dva panely vedle sebe:
  1. LEVÝ PANEL: Živý radar (Live LIDAR scan) v reálném čase + přesný HUD.
  2. PRAVÝ PANEL: Stabilní herní mapa (World Model) + LOKALIZACE ROBOTA.

Klávesové zkratky a ovládání:
  [2 / ě]     - Startovní stěna 2.4 m (Kratší stěna) -> auto-fit pole 2.4 x 3.5 m
  [3 / š]     - Startovní stěna 3.5 m (Delší stěna)  -> auto-fit pole 3.5 x 2.4 m
  [P]         - Přepnout startovní stěnu (2.4 m <-> 3.5 m)
  [F]         - Přepnout režim: 180° Kužel / 360° Celé pole (výchozí 180°)
  [MEZERNÍK]  - Zmrazit / Odmrazit obraz (Freeze)
  [R / E]     - Min vzdálenost (+50 / -50 mm)
  [T / G]     - Max vzdálenost (+200 / -200 mm)
  [I / K]     - Min intenzita (+5 / -5)
  [C]         - Vymazat a přepočítat stabilní mapu hřiště
  [L]         - ZAMKNOUT mapu jako referenční svět (spustí lokalizaci)
  [N]         - Resetovat novou stabilní mapu na aktuální pozici
  [+ / -]     - Zoom in / Zoom out (funguje také kolečko myši)
  Klik myší   - Detailní inspekce souřadnic vybraného bodu / objektu
"""

import sys, os, time, json, math
import pygame
import serial
from serial.tools import list_ports

# Rozměry (Třípanelový vizualizér + mapa hřiště pro monitor 1920x1080)
PANEL_W     = 600
PANEL_H     = 560      # Výška radarových panelů (zmenšená, aby se vešla mapa hřiště)
FIELD_H     = 300      # Výška obdélníkové mapy hřiště dole
WIN_W       = PANEL_W * 3
WIN_H       = PANEL_H + FIELD_H

# Rozměry soutěžního hřiště Kybernoid (mm)
FIELD_W_MM  = 3500.0   # Delší strana hřiště
FIELD_H_MM  = 2400.0   # Kratší strana hřiště

# Konfigurace startovních stěn (přepínatelné klávesami [2/ě] a [3/š] nebo [P])
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

# Barvy
BG_COLOR     = (18, 20, 24)
GRID_COLOR   = (40, 45, 52)
CIRCLE_COLOR = (55, 62, 75)
TEXT_COLOR   = (220, 225, 230)
ACCENT_COLOR = (0, 210, 255)
ACCENT_MAP   = (255, 200, 60)

def get_serial_port():
    if len(sys.argv) > 1: return sys.argv[1]
    ports = list(list_ports.comports())
    for p in ports:
        if "USB" in p.device or "ACM" in p.device: return p.device
    return "/dev/ttyUSB0"

def get_baudrate():
    if len(sys.argv) > 2: return int(sys.argv[2])
    return 921600


# ============================================================
#  Map-to-Map Lokalizátor robota (Porovnání stabilních map hřiště)
# ============================================================
class RobotLocalizer:
    """
    Čistý a spolehlivý Map-to-Map Matching:
    1. Při zamknutí [L] se uloží REFERENČNÍ MAPA (vyfiltrované svíčky a stěny z počátku).
       Změří se všechny vzájemné vzdálenosti mezi body (tyto vzdálenosti jsou absolutně
       neměnné bez ohledu na to, kam robot popojede a jak se otočí).
    2. Při pohybu robota se vezme AKTUÁLNÍ STABILNÍ MAPA (vyfiltrovaná z cca 15-20 měření)
       a porovná se s referenční mapou:
       - Změří se vzdálenosti mezi aktuálními body.
       - Najde se shoda se vzdálenostmi v referenční mapě -> jednoznačná identita bodů.
       - Z rozdílu směrů se spočítá přesný úhel natočení theta.
       - Z posunu bodů se spočítá přesná poloha (X, Y) robota od startu.
    3. Žádné umělé blokování podle vzdálenosti skoku – robot se může posunout o kolik chce!
    4. Výsledek je díky porovnávání dvou stabilních map naprosto klidný, přesný a bez šumu.
    """

    def __init__(self):
        self.locked           = False
        self.ref_candles      = []   # Referenční svíčky ve světových souřadnicích
        self.ref_walls        = []   # Referenční stěny ve světových souřadnicích
        self.ref_candle_dists = []   # [(i, j, dist_mm)] mezi referenčními svíčkami
        self.ref_cw_dists     = []   # [(c_idx, w_idx, dist_mm)] mezi svíčkami a středy stěn

        self.robot_x          = 0.0  # mm ve světě (START = [0,0])
        self.robot_y          = 0.0
        self.robot_deg        = 0.0  # orientace ve stupních
        self.pose_confidence  = 0.0  # 0.0 – 1.0
        self.trajectory       = [(0.0, 0.0)]
        self.is_stationary    = True
        self.candle_tracker   = []   # Krátkodobý tracker svíček (vyhlazení chvění, rozpad za 4 snímky)
        self.last_match_info  = None # Informace o geometrickém rozhodování pro 3. panel

    def lock_map(self, conf_candles, conf_walls):
        """Uloží aktuální potvrzené svíčky a stěny jako fixní referenční mapu světa a vytvoří odstupňované kotvy."""
        self.ref_candles = [dict(c) for c in conf_candles]
        self.ref_walls   = [dict(w) for w in conf_walls]
        self.ref_candle_dists = []
        self.ref_cw_dists     = []
        self.last_match_info  = None

        # 1. Invariantní vzdálenosti mezi svíčkami
        for i in range(len(self.ref_candles)):
            for j in range(i + 1, len(self.ref_candles)):
                d = math.hypot(self.ref_candles[i]['x'] - self.ref_candles[j]['x'],
                               self.ref_candles[i]['y'] - self.ref_candles[j]['y'])
                self.ref_candle_dists.append((i, j, d))

        # 2. Invariantní vzdálenosti mezi svíčkami a středy stěn
        for ci, c in enumerate(self.ref_candles):
            for wi, w in enumerate(self.ref_walls):
                d = math.hypot(c['x'] - w['mx'], c['y'] - w['my'])
                self.ref_cw_dists.append((ci, wi, d))

        # 3. ODSTUPŇOVANÉ GEOMETRICKÉ KOTVY (Přímky mezi svíčkami seřazené podle kvality a rozvoru)
        raw_anchors = []
        for i in range(len(self.ref_candles)):
            for j in range(i + 1, len(self.ref_candles)):
                c1 = self.ref_candles[i]
                c2 = self.ref_candles[j]
                dx = c2['x'] - c1['x']
                dy = c2['y'] - c1['y']
                dist = math.hypot(dx, dy)
                if dist < 300.0: continue
                # Skóre: větší rozvor = mnohem vyšší přesnost úhlu + bonus za oporu o stěny
                score = dist + (c1.get('walls_count', 0) + c2.get('walls_count', 0)) * 400.0
                raw_anchors.append({
                    'idx1': i, 'idx2': j,
                    'p1': (c1['x'], c1['y']),
                    'p2': (c2['x'], c2['y']),
                    'dist': dist,
                    'score': score
                })

        raw_anchors.sort(key=lambda a: a['score'], reverse=True)
        self.anchors = []
        for rank, a in enumerate(raw_anchors):
            a['rank'] = rank + 1
            if rank == 0:
                a['name'] = "KOTVA 1 (PRIMÁRNÍ)"
                a['color'] = (255, 215, 0) # Zlatá
            elif rank == 1:
                a['name'] = "KOTVA 2 (SEKUNDÁRNÍ)"
                a['color'] = (255, 150, 40) # Oranžová
            else:
                a['name'] = f"KOTVA {rank+1} (ZÁLOŽNÍ)"
                a['color'] = (200, 140, 255) # Fialová
            self.anchors.append(a)

        # 4. TROJÚHELNÍKOVÝ VZOREC (spojení Kotvy 1 a Kotvy 2 přes společnou pivotní svíčku)
        self.triangle_pattern = None
        if len(self.anchors) >= 2:
            a1 = self.anchors[0]
            a2 = self.anchors[1]
            shared = set([a1['idx1'], a1['idx2']]).intersection([a2['idx1'], a2['idx2']])
            if shared:
                pivot_idx = list(shared)[0]
                other1_idx = a1['idx2'] if a1['idx1'] == pivot_idx else a1['idx1']
                other2_idx = a2['idx2'] if a2['idx1'] == pivot_idx else a2['idx1']
                pivot_pt = (self.ref_candles[pivot_idx]['x'], self.ref_candles[pivot_idx]['y'])
                p1_pt    = (self.ref_candles[other1_idx]['x'], self.ref_candles[other1_idx]['y'])
                p2_pt    = (self.ref_candles[other2_idx]['x'], self.ref_candles[other2_idx]['y'])

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

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 1.0
        self.trajectory = [(0.0, 0.0)]
        self.is_stationary = True

        self.locked = (len(self.ref_candles) + len(self.ref_walls)) >= 2
        return self.locked

    def unlock_map(self):
        """Odemkne mapu a vrátí robota na start."""
        self.locked = False
        self.ref_candles = []
        self.ref_walls = []
        self.ref_candle_dists = []
        self.ref_cw_dists = []
        self.anchors = []
        self.triangle_pattern = None
        self.candle_tracker = []
        self.last_match_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 0.0
        self.trajectory = [(0.0, 0.0)]
        self.is_stationary = True

    @staticmethod
    def _rigid_transform_2d(src, dst):
        """Metoda nejmenších čtverců (Procrustes) v navigačním systému (+Y dopředu, +X doprava, rotace doprava)."""
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

    def estimate_pose(self, new_candles, new_walls):
        """
        Určí pozici a orientaci robota (x, y, theta) porovnáním DVOU STABILNÍCH MAP.
        Využívá hierarchii kotev:
        1. Stupeň: Trojúhelníkový vzorec (Kotva 1 + Kotva 2) - nejvyšší jistota
        2. Stupeň: Kotva 1 (Primární přímka)
        3. Stupeň: Kotva 2 (Sekundární přímka pro případ zakrytí jedné svíčky)
        4. Stupeň: Záložní kotvy a stěny
        """
        if not self.locked or (len(self.ref_candles) + len(self.ref_walls)) < 2:
            return False

        # Převod na standardní formát (x, y) pro svíčky a (mx, my) pro stěny
        cands = []
        for c in new_candles:
            if isinstance(c, (list, tuple)):
                cands.append((float(c[0]), float(c[1])))
            elif isinstance(c, dict):
                cands.append((float(c['x']), float(c['y'])))

        walls = []
        for w in new_walls:
            if isinstance(w, (list, tuple)):
                mx = (float(w[0]) + float(w[2])) * 0.5
                my = (float(w[1]) + float(w[3])) * 0.5
                walls.append((mx, my))
            elif isinstance(w, dict):
                walls.append((float(w['mx']), float(w['my'])))

        if (len(cands) + len(walls)) < 2:
            return False

        # Pomocná funkce pro ohodnocení shody z transformace (tx, ty, th)
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
                best_rc = None
                for rc in self.ref_candles:
                    d = math.hypot(wx - rc['x'], wy - rc['y'])
                    if d < min_d: min_d = d; best_rc = rc
                if min_d < 160.0 and best_rc:
                    inliers += 2
                    err_sum += min_d
                    inlier_src.append(cp)
                    inlier_dst.append((best_rc['x'], best_rc['y']))
                    inlier_pairs.append(((best_rc['x'], best_rc['y']), (wx, wy), min_d))

            matched_walls = []
            for wp in walls:
                wx = tx + wp[0] * cos_t + wp[1] * sin_t
                wy = ty - wp[0] * sin_t + wp[1] * cos_t
                min_d = 9999.0
                best_rw = None
                for rw in self.ref_walls:
                    d = math.hypot(wx - rw['mx'], wy - rw['my'])
                    if d < min_d: min_d = d; best_rw = rw
                if min_d < 220.0 and best_rw:
                    inliers += 1
                    err_sum += min_d
                    matched_walls.append(((best_rw['mx'], best_rw['my']), (wx, wy), min_d))

            avg_err = (err_sum / inliers) if inliers > 0 else 9999.0

            # Zpřesnění metodou Procrustes přes inliers
            if len(inlier_src) >= 2:
                rtx, rty, rth = self._rigid_transform_2d(inlier_src, inlier_dst)
            else:
                rtx, rty, rth = tx, ty, th

            total_score = base_score + inliers * 400.0 - avg_err * 2.0
            return {
                'inliers': inliers,
                'avg_error': avg_err,
                'tx': rtx, 'ty': rty, 'theta': rth,
                'inlier_pairs': inlier_pairs,
                'matched_walls': matched_walls,
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
                    if abs(d1_cur - tp['dist1']) > 120.0: continue

                    for b in range(len(cands)):
                        if b == p or b == a: continue
                        d2_cur = math.hypot(cands[b][0] - cands[p][0], cands[b][1] - cands[p][1])
                        if abs(d2_cur - tp['dist2']) > 120.0: continue

                        # Kontrola třetí strany trojúhelníku
                        d3_cur = math.hypot(cands[a][0] - cands[b][0], cands[a][1] - cands[b][1])
                        if abs(d3_cur - tp['dist3']) > 140.0: continue

                        # Výpočet azimutu Kotvy 1
                        del_X = tp['p1_pt'][0] - tp['pivot_pt'][0]
                        del_Y = tp['p1_pt'][1] - tp['pivot_pt'][1]
                        az_ref = math.atan2(del_X, del_Y)

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
                            pattern_name="KOTVA 1 + KOTVA 2 (PLNÝ TROJÚHELNÍK)",
                            base_score=5000.0
                        )

                        if res['inliers'] >= 4 and res['score'] > best_score:
                            best_score = res['score']
                            best_match_info = res

        # ========================================================
        # STUPEŇ 2: SHODA JEDNOTLIVÝCH KOTEVA (Kotva 1 -> Kotva 2 -> Záložní)
        # ========================================================
        if best_match_info is None or best_match_info['inliers'] < 4:
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

                            rank_bonus = max(0.0, 1000.0 - (anchor['rank'] - 1) * 300.0)
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
        # STUPEŇ 3: ZÁLOŽNÍ METODA (SVÍČKA-STĚNA NEBO STĚNA-STĚNA)
        # ========================================================
        if best_match_info is None or best_match_info['inliers'] < 2:
            all_cur = [(c[0], c[1], 'c') for c in cands] + [(w[0], w[1], 'w') for w in walls]
            all_ref = [(c['x'], c['y'], 'c') for c in self.ref_candles] + [(w['mx'], w['my'], 'w') for w in self.ref_walls]

            for i in range(len(all_cur)):
                for j in range(i + 1, len(all_cur)):
                    d_cur = math.hypot(all_cur[i][0] - all_cur[j][0], all_cur[i][1] - all_cur[j][1])
                    if d_cur < 150.0: continue

                    for ri in range(len(all_ref)):
                        if all_cur[i][2] != all_ref[ri][2]: continue
                        for rj in range(len(all_ref)):
                            if rj == ri or all_cur[j][2] != all_ref[rj][2]: continue

                            d_ref = math.hypot(all_ref[ri][0] - all_ref[rj][0], all_ref[ri][1] - all_ref[rj][1])
                            if abs(d_cur - d_ref) < 130.0:
                                p1 = (all_cur[i][0], all_cur[i][1])
                                p2 = (all_cur[j][0], all_cur[j][1])
                                r1 = (all_ref[ri][0], all_ref[ri][1])
                                r2 = (all_ref[rj][0], all_ref[rj][1])

                                del_X = r2[0] - r1[0]; del_Y = r2[1] - r1[1]
                                d_x = p2[0] - p1[0];   d_y = p2[1] - p1[1]
                                th = (math.atan2(del_X, del_Y) - math.atan2(d_x, d_y) + math.pi) % (2 * math.pi) - math.pi
                                cos_t = math.cos(th); sin_t = math.sin(th)
                                tx = r1[0] - (p1[0] * cos_t + p1[1] * sin_t)
                                ty = r1[1] - (-p1[0] * sin_t + p1[1] * cos_t)

                                res = evaluate_transformation(
                                    tx, ty, th,
                                    active_anchors=[],
                                    pattern_name="ZÁLOHA: SVÍČKA-STĚNA",
                                    base_score=0.0
                                )

                                if res['inliers'] >= 2 and res['score'] > best_score:
                                    best_score = res['score']
                                    best_match_info = res

        # ========================================================
        # APLIKACE VÝSLEDKU: STATICKÁ POLOHA POROVNÁNÍM DVOU STABILNÍCH MAP
        # ========================================================
        if best_match_info is not None and best_match_info['inliers'] >= 2:
            self.last_match_info = best_match_info
            self.robot_x = best_match_info['tx']
            self.robot_y = best_match_info['ty']
            self.robot_deg = math.degrees(best_match_info['theta'])
            self.is_stationary = True
            quality = min(1.0, best_match_info['inliers'] / max(2, len(self.ref_candles) + len(self.ref_walls)))
            self.pose_confidence = quality

            # Záznam do projeté trajektorie (každých 25 mm)
            last_pt = self.trajectory[-1]
            if math.hypot(self.robot_x - last_pt[0], self.robot_y - last_pt[1]) > 25.0:
                self.trajectory.append((self.robot_x, self.robot_y))
                if len(self.trajectory) > 300:
                    self.trajectory.pop(0)

            return True
        else:
            self.pose_confidence = max(0.0, self.pose_confidence - 0.05)
            return False

    update_pose = estimate_pose


# ============================================================
#  Třída pro stabilní herní mapu (World Model)
# ============================================================

class StableFieldModel:
    def __init__(self):
        self.candles = []
        self.walls = []
        self.total_scans = 0

    def reset(self):
        self.candles.clear()
        self.walls.clear()
        self.total_scans = 0

    def update(self, live_candles, live_walls):
        self.total_scans += 1

        # 1. SVÍČKY (párování do 150 mm)
        matched_c = set()
        for lc in live_candles:
            cx, cy = lc[0], lc[1]
            best_idx = -1
            best_dist = 150.0
            for idx, sc in enumerate(self.candles):
                if idx in matched_c: continue
                d = math.hypot(sc['x'] - cx, sc['y'] - cy)
                if d < best_dist:
                    best_dist = d
                    best_idx = idx

            if best_idx >= 0:
                sc = self.candles[best_idx]
                sc['x'] = 0.85 * sc['x'] + 0.15 * cx
                sc['y'] = 0.85 * sc['y'] + 0.15 * cy
                sc['hits'] += 1
                sc['missed'] = 0
                matched_c.add(best_idx)
            else:
                if len(self.candles) < 8:
                    new_idx = len(self.candles)
                    self.candles.append({'x': cx, 'y': cy, 'hits': 1, 'missed': 0})
                    matched_c.add(new_idx)

        for idx, sc in enumerate(self.candles):
            if idx not in matched_c:
                sc['missed'] += 1

        self.candles = [c for c in self.candles if c['missed'] < 16 or (c['hits'] > 10 and c['missed'] < 25)]

        # 2. STĚNY (párování do 140 mm)
        matched_w = set()
        for lw in live_walls:
            x1, y1, x2, y2 = lw[0], lw[1], lw[2], lw[3]
            mx = (x1 + x2) * 0.5
            my = (y1 + y2) * 0.5
            wlen = lw[4]

            best_idx = -1
            best_dist = 140.0
            for idx, sw in enumerate(self.walls):
                if idx in matched_w: continue
                d = math.hypot(sw['mx'] - mx, sw['my'] - my)
                if d < best_dist:
                    best_dist = d
                    best_idx = idx

            if best_idx >= 0:
                sw = self.walls[best_idx]
                sw['x1'] = 0.8 * sw['x1'] + 0.2 * x1
                sw['y1'] = 0.8 * sw['y1'] + 0.2 * y1
                sw['x2'] = 0.8 * sw['x2'] + 0.2 * x2
                sw['y2'] = 0.8 * sw['y2'] + 0.2 * y2
                sw['mx'] = (sw['x1'] + sw['x2']) * 0.5
                sw['my'] = (sw['y1'] + sw['y2']) * 0.5
                sw['len'] = math.hypot(sw['x2'] - sw['x1'], sw['y2'] - sw['y1'])
                sw['hits'] += 1
                sw['missed'] = 0
                matched_w.add(best_idx)
            else:
                if len(self.walls) < 12:
                    new_w_idx = len(self.walls)
                    self.walls.append({
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'mx': mx, 'my': my, 'len': wlen,
                        'hits': 1, 'missed': 0
                    })
                    matched_w.add(new_w_idx)

        for idx, sw in enumerate(self.walls):
            if idx not in matched_w:
                sw['missed'] += 1

        self.walls = [w for w in self.walls if w['missed'] < 14 or (w['hits'] > 8 and w['missed'] < 22)]

    def get_confirmed_candles(self):
        valid = [c for c in self.candles if c['hits'] >= 4]
        conf_walls = self.get_confirmed_walls()

        candidates = []
        for c in valid:
            in_wall = False
            walls_within_25cm = 0
            closest_dist = 9999.0

            for w in conf_walls:
                dx = w['x2'] - w['x1']
                dy = w['y2'] - w['y1']
                L = math.hypot(dx, dy)
                if L < 10.0: continue
                ux = dx / L
                uy = dy / L
                nx = -uy
                ny = ux

                vx = c['x'] - w['x1']
                vy = c['y'] - w['y1']
                proj_t = vx * ux + vy * uy
                d_perp = abs(vx * nx + vy * ny)

                # Fialový koridor: svíčka nesmí ležet v dráze stěny ani v prodloužení o 15 cm před/za
                if d_perp <= 55.0 and -150.0 <= proj_t <= L + 150.0:
                    in_wall = True
                    break

                # Nejbližší bod na úsečce stěny
                t_clamped = max(0.0, min(L, proj_t))
                cx_wall = w['x1'] + t_clamped * ux
                cy_wall = w['y1'] + t_clamped * uy
                dist_to_seg = math.hypot(c['x'] - cx_wall, c['y'] - cy_wall)

                if dist_to_seg < closest_dist:
                    closest_dist = dist_to_seg

                # Poloměr 25 cm (250 mm): počítáme stěny u svíčky
                if dist_to_seg <= 250.0:
                    walls_within_25cm += 1

            if not in_wall:
                c_copy = dict(c)
                c_copy['walls_count'] = walls_within_25cm
                c_copy['closest_wall_dist'] = closest_dist
                # Skóre: přednost mají svíčky se stěnou (2 stěny > 1 stěna > 0 stěn)
                score = walls_within_25cm * 2000.0 + c['hits'] * 10.0
                if walls_within_25cm > 0:
                    score += max(0.0, 250.0 - closest_dist)
                c_copy['score'] = score
                candidates.append(c_copy)

        # Doplnění svíčky pro každou samostatnou 30cm stěnu, která zatím nemá svíčku v dosahu 25 cm:
        for w in conf_walls:
            if w['len'] >= 250.0 and w['len'] <= 350.0:
                has_candle = False
                for cand in candidates:
                    d = math.hypot(cand['x'] - w['mx'], cand['y'] - w['my'])
                    if d <= 260.0:
                        has_candle = True
                        break

                if not has_candle:
                    # Normála směřující k robotu (do středu pole)
                    dx = w['x2'] - w['x1']
                    dy = w['y2'] - w['y1']
                    L = math.hypot(dx, dy)
                    if L >= 10.0:
                        ux = dx / L
                        uy = dy / L
                        nx = -uy
                        ny = ux
                        if (w['mx'] * nx + w['my'] * ny) > 0:
                            nx = -nx
                            ny = -ny

                        synth_x = w['mx'] + 130.0 * nx
                        synth_y = w['my'] + 130.0 * ny
                        candidates.append({
                            'x': synth_x,
                            'y': synth_y,
                            'hits': w['hits'],
                            'walls_count': 1,
                            'closest_wall_dist': 130.0,
                            'score': 2500.0 + w['hits'] * 10.0
                        })

        # Seřadíme sestupně podle skóre
        candidates.sort(key=lambda x: x['score'], reverse=True)

        # Výběr s pravidly:
        # 1. Rozestup svíček alespoň 80 cm (800 mm)
        # 2. Maximálně JEDNA svíčka bez stěny (dvě svíčky bez stěny nemohou být)
        filtered = []
        isolated_count = 0

        for cand in candidates:
            if len(filtered) >= 4:
                break

            is_isolated = (cand['walls_count'] == 0)
            if is_isolated and isolated_count >= 1:
                # Už máme jednu samostatnou svíčku bez stěny, další bez stěny je vyloučena!
                continue

            # Kontrola vzdálenosti 80 cm od už vybraných
            too_close = False
            for f in filtered:
                if math.hypot(cand['x'] - f['x'], cand['y'] - f['y']) < 800.0:
                    too_close = True
                    break

            if not too_close:
                if is_isolated:
                    isolated_count += 1
                filtered.append(cand)

        return filtered

    def get_confirmed_walls(self):
        valid = [w for w in self.walls if w['hits'] >= 4]
        valid.sort(key=lambda w: w['hits'], reverse=True)

        selected = []
        for w in valid:
            if len(selected) >= 6:
                break

            conflict = False
            w_dx = (w['x2'] - w['x1']) / w['len'] if w['len'] > 0 else 0
            w_dy = (w['y2'] - w['y1']) / w['len'] if w['len'] > 0 else 0

            for s in selected:
                s_dx = (s['x2'] - s['x1']) / s['len'] if s['len'] > 0 else 0
                s_dy = (s['y2'] - s['y1']) / s['len'] if s['len'] > 0 else 0

                center_dist = math.hypot(w['mx'] - s['mx'], w['my'] - s['my'])
                dot = abs(w_dx * s_dx + w_dy * s_dy)

                # Kolmé stěny (cca 70° až 110° -> |dot| <= 0.35)
                is_perp = (dot <= 0.35)

                if is_perp:
                    # Kolmé stěny tvoří rohy L nebo U-boxy, dotýkají se v rozích.
                    if center_dist < 70.0:
                        conflict = True
                        break
                else:
                    # Nekolmé stěny (rovnoběžné nebo šikmé např. pod 45°) se nesmí překrývat
                    if center_dist < 200.0:
                        conflict = True
                        break

                    # Kolmá vzdálenost nosné přímky a podélný překryv
                    perp_dist = abs((w['mx'] - s['x1']) * (-s_dy) + (w['my'] - s['y1']) * s_dx)
                    proj_long = (w['mx'] - s['x1']) * s_dx + (w['my'] - s['y1']) * s_dy
                    if perp_dist < 150.0 and -60.0 <= proj_long <= (s['len'] + 60.0):
                        conflict = True
                        break

            if not conflict:
                selected.append(w)

        return selected



# ============================================================
#  Globální mapa hřiště (akumuluje svíčky a stěny z různých poloh robota)
# ============================================================
class GlobalFieldMap:
    """
    Kumulativní mapa celého hřiště v absolutních souřadnicích (START = [0,0]).
    Sbírá svíčky a stěny postupně, jak robot objíždí hřiště.
    Dynamicky napasovává obdélník pole podle zvolené startovní stěny (2.4 m nebo 3.5 m)
    a polohy detekovaných svíček tak, aby všechny platné svíčky ležely uvnitř pole.
    Vynucuje pravidla soutěže:
      - Pouze objekty UVNITŘ napasovaného pole (co je vně, není svíčka/stěna)
      - Max 4 svíčky (rozestup min 800 mm, max 1 bez stěny)
      - Max 6 stěn (nepřekrývající se)
      - Svíčka nesmí ležet v koridoru stěny
    """

    def __init__(self):
        self.candles = []      # [{'x': float, 'y': float, 'hits': int, 'walls_count': int, ...}]
        self.walls = []        # [{'x1','y1','x2','y2','mx','my','len','hits': int}]
        self.robot_start = (0.0, 0.0)   # Výchozí pozice START
        self.robot_pos = (0.0, 0.0)     # Aktuální pozice
        self.robot_deg = 0.0            # Aktuální natočení
        self.trajectory = [(0.0, 0.0)]  # Celková trajektorie
        self.total_updates = 0
        self.start_wall_mode = 2        # 2 = 2.4 m (Kratší stěna), 3 = 3.5 m (Delší stěna)
        self.smooth_bounds = None       # [xmin, xmax, ymin, ymax]
        self.rejected_outside_count = 0 # Počet bodů vyřazených mimo hranice hřiště

    def reset(self):
        self.candles.clear()
        self.walls.clear()
        self.robot_pos = (0.0, 0.0)
        self.robot_deg = 0.0
        self.trajectory = [(0.0, 0.0)]
        self.total_updates = 0
        self.smooth_bounds = None
        self.rejected_outside_count = 0

    def set_start_wall(self, mode):
        """Nastaví startovní stěnu: 2 pro 2.4 m, 3 pro 3.5 m."""
        if mode in (2, 3) and self.start_wall_mode != mode:
            self.start_wall_mode = mode
            self.smooth_bounds = None  # Okamžitý skok na nové rozměry

    def toggle_start_wall(self):
        """Přepne mezi startovní stěnou 2.4 m a 3.5 m."""
        new_mode = 3 if self.start_wall_mode == 2 else 2
        self.set_start_wall(new_mode)
        return self.start_wall_mode

    def get_field_dims(self):
        """Vrátí nominální šířku W (podél X) a délku L (podél Y)."""
        cfg = WALL_CONFIGS.get(self.start_wall_mode, WALL_CONFIGS[2])
        return cfg["w"], cfg["l"]

    def get_auto_field_bounds(self):
        """
        Automaticky spočítá a vyhladí hranice hřiště [xmin, xmax, ymin, ymax].
        Napasuje pole tak, aby:
          1. Robot u startovní stěny [0,0] byl bezpečně uvnitř.
          2. Všechny kvalitní svíčky a stěny byly rovnoměrně vycentrovány v šířce W.
          3. Celá délka L sahala dopředu od zadní stěny.
        """
        W, L = self.get_field_dims()

        # Body pro usazení hřiště: start robota [0,0], aktuální robot [rx, ry] a trajektorie
        x_pts = [0.0, self.robot_pos[0]]
        y_pts = [0.0, self.robot_pos[1]]

        # Zahrneme body trajektorie
        for tp in self.trajectory:
            x_pts.append(tp[0])
            y_pts.append(tp[1])

        # Svíčky s alespoň 1 hitem v rozumném dosahu
        cand_x = []
        cand_y = []
        for c in self.candles:
            if c.get('hits', 0) >= 1 and abs(c['x']) <= 4500.0 and -800.0 <= c['y'] <= 5000.0:
                cand_x.append(c['x'])
                cand_y.append(c['y'])
                x_pts.append(c['x'])
                y_pts.append(c['y'])

        # 1. OSA Y (Hloubka od zadní startovní stěny):
        # Robot začíná cca 20 cm (200 mm) od zadní stěny -> stěna na Y = -200 mm
        min_y = min(y_pts)
        target_ymin = min(-200.0, min_y - 80.0)
        target_ymax = target_ymin + L

        # 2. OSA X (Šířka hřiště):
        # Všechny spolehlivé objekty vycentrujeme do šířky W
        min_x = min(x_pts)
        max_x = max(x_pts)
        span_x = max_x - min_x

        if span_x <= (W - 80.0):
            # Všechny body se krásně vejdou do šířky W -> vycentrujeme je symetricky
            mid_x = (min_x + max_x) * 0.5
            target_xmin = mid_x - W * 0.5
        else:
            # Pokud je rozpětí bodů větší než W (např. zbloudilý odraz),
            # upřednostníme robota a nejstabilnější svíčky (hits >= 2)
            reliable_x = [0.0, self.robot_pos[0]]
            for c in self.candles:
                if c.get('hits', 0) >= 2 and abs(c['x']) <= W:
                    reliable_x.append(c['x'])
            mid_x = (min(reliable_x) + max(reliable_x)) * 0.5
            target_xmin = mid_x - W * 0.5

        # Robot na startu [0, 0] musí ležet uvnitř pole s rezervou min 100 mm od levé i pravé zdi:
        # target_xmin <= -100  a  target_xmin + W >= +100 (tj. target_xmin >= 100 - W)
        target_xmin = max(100.0 - W, min(-100.0, target_xmin))
        target_xmax = target_xmin + W

        target_bounds = [target_xmin, target_xmax, target_ymin, target_ymax]

        # 3. Vyhlazení (EMA - Exponential Moving Average):
        if self.smooth_bounds is None:
            self.smooth_bounds = list(target_bounds)
        else:
            alpha = 0.15
            self.smooth_bounds[0] = (1.0 - alpha) * self.smooth_bounds[0] + alpha * target_xmin
            self.smooth_bounds[1] = self.smooth_bounds[0] + W
            self.smooth_bounds[2] = (1.0 - alpha) * self.smooth_bounds[2] + alpha * target_ymin
            self.smooth_bounds[3] = self.smooth_bounds[2] + L

        return tuple(self.smooth_bounds)

    def update_robot_pose(self, rx, ry, rdeg):
        """Aktualizuje polohu robota a trajektorii."""
        self.robot_pos = (rx, ry)
        self.robot_deg = rdeg
        if len(self.trajectory) == 0 or math.hypot(rx - self.trajectory[-1][0], ry - self.trajectory[-1][1]) > 15.0:
            self.trajectory.append((rx, ry))
            if len(self.trajectory) > 600:
                self.trajectory.pop(0)

    def ingest_candles(self, candles_abs):
        """
        Přijme seznam svíček v absolutních souřadnicích hřiště (START=[0,0]).
        Páruje do 200 mm s existujícími a přidává nové.
        """
        self.total_updates += 1
        matched = set()
        for c in candles_abs:
            cx, cy = c['x'], c['y']
            best_idx = -1
            best_d = 200.0
            for idx, gc in enumerate(self.candles):
                if idx in matched:
                    continue
                d = math.hypot(gc['x'] - cx, gc['y'] - cy)
                if d < best_d:
                    best_d = d
                    best_idx = idx

            if best_idx >= 0:
                gc = self.candles[best_idx]
                alpha = 0.15
                gc['x'] = (1.0 - alpha) * gc['x'] + alpha * cx
                gc['y'] = (1.0 - alpha) * gc['y'] + alpha * cy
                gc['hits'] += 1
                gc['walls_count'] = max(gc.get('walls_count', 0), c.get('walls_count', 0))
                matched.add(best_idx)
            else:
                if len(self.candles) < 8:
                    self.candles.append({
                        'x': cx, 'y': cy,
                        'hits': c.get('hits', 1),
                        'walls_count': c.get('walls_count', 0),
                        'source': 'sensor'
                    })

    def ingest_walls(self, walls_abs):
        """
        Přijme seznam stěn v absolutních souřadnicích hřiště.
        Páruje středy do 250 mm s existujícími a přidává nové.
        """
        matched = set()
        for w in walls_abs:
            mx = (w['x1'] + w['x2']) * 0.5
            my = (w['y1'] + w['y2']) * 0.5
            best_idx = -1
            best_d = 250.0
            for idx, gw in enumerate(self.walls):
                if idx in matched:
                    continue
                d = math.hypot(gw['mx'] - mx, gw['my'] - my)
                if d < best_d:
                    best_d = d
                    best_idx = idx

            if best_idx >= 0:
                gw = self.walls[best_idx]
                alpha = 0.15
                gw['x1'] = (1 - alpha) * gw['x1'] + alpha * w['x1']
                gw['y1'] = (1 - alpha) * gw['y1'] + alpha * w['y1']
                gw['x2'] = (1 - alpha) * gw['x2'] + alpha * w['x2']
                gw['y2'] = (1 - alpha) * gw['y2'] + alpha * w['y2']
                gw['mx'] = (gw['x1'] + gw['x2']) * 0.5
                gw['my'] = (gw['y1'] + gw['y2']) * 0.5
                gw['len'] = math.hypot(gw['x2'] - gw['x1'], gw['y2'] - gw['y1'])
                gw['hits'] += 1
                matched.add(best_idx)
            else:
                if len(self.walls) < 12:
                    self.walls.append({
                        'x1': w['x1'], 'y1': w['y1'],
                        'x2': w['x2'], 'y2': w['y2'],
                        'mx': mx, 'my': my,
                        'len': w.get('len', math.hypot(w['x2'] - w['x1'], w['y2'] - w['y1'])),
                        'hits': w.get('hits', 1)
                    })

    def get_field_candles(self, max_count=4):
        """
        Vrátí nejvýše max_count svíček po aplikaci:
          1. Přísného prostorového filtru (pouze svíčky UVNITŘ hřiště).
          2. Fialového koridoru stěn (svíčka nesmí ležet v ose stěny).
          3. Pravidel soutěže (rozestup min 800 mm, max 1 volná svíčka bez stěny).
        """
        xmin, xmax, ymin, ymax = self.get_auto_field_bounds()
        margin = 35.0  # Bezpečnostní okraj od obvodových stěn

        good_walls = self.get_field_walls()
        candidates = []
        rejected_outside = 0

        for c in self.candles:
            if c.get('hits', 0) < 1:
                continue

            # 1. PŘÍSNÝ FILTR HRANIC POLE: Co je mimo pole, NENÍ svíčka!
            if not (xmin + margin <= c['x'] <= xmax - margin and ymin + margin <= c['y'] <= ymax - margin):
                rejected_outside += 1
                continue

            # 2. FILTR FIALOVÉHO KORIDORU STĚNY
            in_wall = False
            walls_near = 0
            for w in good_walls:
                dx = w['x2'] - w['x1']
                dy = w['y2'] - w['y1']
                L = w['len']
                if L < 10.0:
                    continue
                ux = dx / L
                uy = dy / L
                nx = -uy
                ny = ux
                vx = c['x'] - w['x1']
                vy = c['y'] - w['y1']
                proj_t = vx * ux + vy * uy
                d_perp = abs(vx * nx + vy * ny)

                if d_perp <= 55.0 and -150.0 <= proj_t <= L + 150.0:
                    in_wall = True
                    break

                t_cl = max(0.0, min(L, proj_t))
                wx = w['x1'] + t_cl * ux
                wy = w['y1'] + t_cl * uy
                if math.hypot(c['x'] - wx, c['y'] - wy) <= 250.0:
                    walls_near += 1

            if not in_wall:
                candidates.append({
                    'x': c['x'], 'y': c['y'],
                    'hits': c['hits'],
                    'walls_count': max(c.get('walls_count', 0), walls_near),
                    'score': c['hits'] * 10.0 + walls_near * 2000.0
                })

        self.rejected_outside_count = rejected_outside
        candidates.sort(key=lambda x: x['score'], reverse=True)

        # 3. VÝBĚR S PRAVIDLY: rozestup min 800 mm, max 1 volná svíčka bez stěny
        selected = []
        isolated_count = 0
        for cand in candidates:
            if len(selected) >= max_count:
                break
            is_iso = (cand['walls_count'] == 0)
            if is_iso and isolated_count >= 1:
                continue
            too_close = any(math.hypot(cand['x'] - s['x'], cand['y'] - s['y']) < 800.0 for s in selected)
            if not too_close:
                if is_iso:
                    isolated_count += 1
                selected.append(cand)

        return selected

    def get_field_walls(self, max_count=6):
        """Vrátí nejvýše max_count stěn bez překryvů, které leží uvnitř hřiště."""
        xmin, xmax, ymin, ymax = self.get_auto_field_bounds()
        valid = [w for w in self.walls if w.get('hits', 0) >= 2]

        # Filtrujeme stěny, které by ležely zcela mimo pole
        in_bounds_walls = []
        for w in valid:
            if (xmin - 80.0 <= w['mx'] <= xmax + 80.0 and ymin - 80.0 <= w['my'] <= ymax + 80.0):
                in_bounds_walls.append(w)

        in_bounds_walls.sort(key=lambda w: w['hits'], reverse=True)

        selected = []
        for w in in_bounds_walls:
            if len(selected) >= max_count:
                break
            conflict = False
            w_dx = (w['x2'] - w['x1']) / w['len'] if w['len'] > 0 else 0
            w_dy = (w['y2'] - w['y1']) / w['len'] if w['len'] > 0 else 0
            for s in selected:
                s_dx = (s['x2'] - s['x1']) / s['len'] if s['len'] > 0 else 0
                s_dy = (s['y2'] - s['y1']) / s['len'] if s['len'] > 0 else 0
                center_dist = math.hypot(w['mx'] - s['mx'], w['my'] - s['my'])
                dot = abs(w_dx * s_dx + w_dy * s_dy)
                is_perp = (dot <= 0.35)
                if is_perp:
                    if center_dist < 70.0:
                        conflict = True
                        break
                else:
                    if center_dist < 200.0:
                        conflict = True
                        break
            if not conflict:
                selected.append(w)
        return selected


# ============================================================
#  Dvoupanelový vizualizér
# ============================================================
class DualLidarVisualizer:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.frozen = False

        # Výchozí nastavení: 180° dopředný kužel a rozsah 2400 mm ("2 metry něco")
        self.fov_limited = True
        self.min_range = 130
        self.max_range = 2400
        self.min_intensity = 15
        self.view_span_mm = 3500.0
        self.last_sync_time = 0.0

        self.selected_object = None

        self.last_frame = {
            "fps": 0.0, "pts": 0, "calc_us": 0, "heap": 0, "crc_err": 0,
            "walls": [], "candles": [], "cloud": []
        }

        self.ref_model = StableFieldModel()   # Stará stabilní mapa (na výchozím místě START)
        self.curr_model = StableFieldModel()  # Nová stabilní mapa (na aktuálním místě robota)
        self.localizer = RobotLocalizer()
        self.field_map = GlobalFieldMap()   # Kumulativní mapa celého hřiště
        self.field_mode_idx = 0             # Výchozí startovní orientace (Kratší stěna střed)

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption(f"Kybernoid 27 - LiDAR D200 Detector [{port}]")
        self.clock = pygame.time.Clock()

        try:
            self.font_main = pygame.font.SysFont("DejaVu Sans Mono", 12)
            self.font_bold = pygame.font.SysFont("DejaVu Sans Mono", 13, bold=True)
            self.font_big  = pygame.font.SysFont("DejaVu Sans Mono", 16, bold=True)
        except:
            self.font_main = pygame.font.SysFont(None, 15)
            self.font_bold = pygame.font.SysFont(None, 17)
            self.font_big  = pygame.font.SysFont(None, 21)

        self.connect_serial()
        self.sync_esp32_config()

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

    def sync_esp32_config(self):
        """Vynutí odeslání nastavení FoV (180°), rozsahu a odrazivosti do ESP32."""
        self.send_cmd(f"FOV {1 if self.fov_limited else 0}")
        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
        self.send_cmd(f"INTENSITY {self.min_intensity}")

    def on_parameter_changed(self):
        """Vždy při změně parametrů (FoV/dosah/intenzita) se vygeneruje čistá nová mapa."""
        self.sync_esp32_config()
        self.ref_model.reset()
        self.curr_model.reset()
        self.field_map.reset()
        self.localizer.unlock_map()
        print("[MAPA] Parametry upraveny -> stabilní mapy, mapa hřiště i lokalizace resetovány.")

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
                elif line.startswith("#"):
                    # Detekce restartu ESP32 při pohybu nebo výpadku napájení
                    if any(w in line for w in ["INIT", "Ready", "rst:", "boot"]):
                        print(f"[SERIAL] Detekován restart ESP32 ({line}) -> Obnovuji konfiguraci...")
                        self.sync_esp32_config()
            except: pass

        if newest is not None and not self.frozen:
            # 1. Automatická kontrola a obnova parametrů ESP32 (detekce zbloudilých 360° nebo příliš vzdálených bodů)
            cloud = newest.get("cloud", [])
            n_pts = len(cloud) // 3
            has_rear = False
            has_overrange = False
            for i in range(min(40, n_pts)):
                px = cloud[i * 3]
                py = cloud[i * 3 + 1]
                if py < -100 and (px*px + py*py) > 40000:
                    has_rear = True
                if (px*px + py*py) > (self.max_range + 200)**2:
                    has_overrange = True

            now = time.time()
            if ((self.fov_limited and has_rear) or has_overrange) and (now - self.last_sync_time > 1.0):
                self.last_sync_time = now
                print(f"[AUTO-SYNC] ESP32 ztratil nastavení (360°={has_rear}, nadlimit={has_overrange}) -> Vynucuji synchronizaci...")
                self.sync_esp32_config()

            # 2. Ochranné klientské vyčištění dat (pro případ, že ESP32 stihne poslat nefiltrovaný snímek)
            max_r_sq = (self.max_range + 50) ** 2
            filtered_cloud = []
            for i in range(n_pts):
                px = cloud[i * 3]
                py = cloud[i * 3 + 1]
                pi = cloud[i * 3 + 2]
                if self.fov_limited and py < 0:
                    continue
                if (px*px + py*py) > max_r_sq:
                    continue
                filtered_cloud.extend([px, py, pi])
            newest["cloud"] = filtered_cloud

            filtered_walls = []
            for w in newest.get("walls", []):
                my = (w[1] + w[3]) * 0.5
                mx = (w[0] + w[2]) * 0.5
                if self.fov_limited and my < 0:
                    continue
                if (mx*mx + my*my) > max_r_sq:
                    continue
                filtered_walls.append(w)
            newest["walls"] = filtered_walls

            filtered_cands = []
            for c in newest.get("candles", []):
                cx, cy = c[0], c[1]
                if self.fov_limited and cy < 0:
                    continue
                if (cx*cx + cy*cy) > max_r_sq:
                    continue
                filtered_cands.append(c)
            newest["candles"] = filtered_cands

            self.last_frame = newest

            if not self.localizer.locked:
                # 1. PŘED ZAMKNUTÍM: Sbíráme Starou stabilní mapu na výchozím místě (START)
                self.ref_model.update(newest.get("candles", []), newest.get("walls", []))

                # Průběžně plníme globální mapu hřiště z výchozí pozice (poloha = [0,0], natočení = 0°)
                ref_c = self.ref_model.get_confirmed_candles()
                ref_w = self.ref_model.get_confirmed_walls()
                if ref_c or ref_w:
                    self.field_map.ingest_candles(ref_c)
                    self.field_map.ingest_walls(ref_w)
            else:
                # 2. PO ZAMKNUTÍ: Stará stabilní mapa je neměnná reference.
                # Sbíráme Novou stabilní mapu na aktuálním stanovišti robota!
                self.curr_model.update(newest.get("candles", []), newest.get("walls", []))

                # Porovnáváme DVĚ STABILNÍ MAPY (potvrzené objekty z nového modelu proti starému etalonu)
                new_cands = self.curr_model.get_confirmed_candles()
                new_walls = self.curr_model.get_confirmed_walls()
                if (len(new_cands) + len(new_walls)) >= 2:
                    self.localizer.estimate_pose(new_cands, new_walls)

                # Aktualizace globální mapy hřiště transformovanými daty z nové pozice robota
                loc = self.localizer
                if loc.pose_confidence > 0.3 and (len(new_cands) + len(new_walls)) >= 1:
                    heading_rad = math.radians(loc.robot_deg)
                    cos_h = math.cos(heading_rad)
                    sin_h = math.sin(heading_rad)

                    # Transformace svíček do absolutních souřadnic hřiště
                    abs_candles = []
                    for c in new_cands:
                        ax = loc.robot_x + c['x'] * cos_h + c['y'] * sin_h
                        ay = loc.robot_y - c['x'] * sin_h + c['y'] * cos_h
                        abs_candles.append({'x': ax, 'y': ay, 'walls_count': c.get('walls_count', 0)})

                    # Transformace stěn do absolutních souřadnic hřiště
                    abs_walls = []
                    for w in new_walls:
                        ax1 = loc.robot_x + w['x1'] * cos_h + w['y1'] * sin_h
                        ay1 = loc.robot_y - w['x1'] * sin_h + w['y1'] * cos_h
                        ax2 = loc.robot_x + w['x2'] * cos_h + w['y2'] * sin_h
                        ay2 = loc.robot_y - w['x2'] * sin_h + w['y2'] * cos_h
                        abs_walls.append({
                            'x1': ax1, 'y1': ay1, 'x2': ax2, 'y2': ay2,
                            'len': w.get('len', math.hypot(ax2 - ax1, ay2 - ay1))
                        })

                    self.field_map.ingest_candles(abs_candles)
                    self.field_map.ingest_walls(abs_walls)
                    self.field_map.update_robot_pose(loc.robot_x, loc.robot_y, loc.robot_deg)

    def handle_click(self, pos):
        if pos[0] >= PANEL_W or pos[1] >= PANEL_H:
            return  # Klik mimo 1. panel neprovádí inspekci bodů radaru
        cx = PANEL_W / 2.0
        cy = PANEL_H / 2.0
        scale = (PANEL_W / 2.0) / self.view_span_mm
        click_x_mm = (pos[0] - cx) / scale
        click_y_mm = -(pos[1] - cy) / scale

        best_dist = 999999.0
        best_obj = None

        # Svíčky
        for c in self.last_frame.get("candles", []):
            dx = c[0] - click_x_mm
            dy = c[1] - click_y_mm
            d = math.hypot(dx, dy)
            if d < best_dist:
                best_dist = d
                best_obj = (c[0], c[1], math.hypot(c[0], c[1]))

        # Stěny
        for w in self.last_frame.get("walls", []):
            mx = (w[0] + w[2]) / 2.0
            my = (w[1] + w[3]) / 2.0
            dx = mx - click_x_mm
            dy = my - click_y_mm
            d = math.hypot(dx, dy)
            if d < best_dist:
                best_dist = d
                best_obj = (mx, my, math.hypot(mx, my))

        if best_dist < 400: # V dosahu kliku
            self.selected_object = best_obj
        else:
            self.selected_object = (click_x_mm, click_y_mm, math.hypot(click_x_mm, click_y_mm))

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

        # Obrys hřiště podle aktivní startovní stěny (2.4x3.5 m nebo 3.5x2.4 m)
        fw, fl = self.field_map.get_field_dims()
        hw = fw / 2.0
        p1 = self.to_screen(-hw, 0, offset_x)
        p2 = self.to_screen( hw, 0, offset_x)
        p3 = self.to_screen( hw, fl, offset_x)
        p4 = self.to_screen(-hw, fl, offset_x)
        pygame.draw.polygon(self.screen, (28, 44, 34), [p1, p2, p3, p4], 1)

        if panel_idx == 0:
            # 1. PANEL (ŽIVÝ RADAR): Robot pevně ve středu se směrovou šipkou dopředu
            pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 8)
            pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 35), 3)
        elif panel_idx == 1:
            # 2. PANEL (PŮVODNÍ STATICKÁ MAPA): Výchozí bod START (0,0) + orientační šipka
            pygame.draw.circle(self.screen, (0, 180, 220), (cx, cy), 7, 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx - 5, cy), (cx + 5, cy), 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx, cy - 5), (cx, cy + 5), 1)
            pygame.draw.line(self.screen, (0, 220, 150), (cx, cy), (cx, cy - 25), 2)
            lbl_start = self.font_main.render("START (0,0)", True, (0, 220, 255))
            self.screen.blit(lbl_start, (cx + 9, cy - 7))
        else:
            # 3. PANEL (NOVÁ MAPA & POROVNÁNÍ): Původní počátek START (0,0)
            pygame.draw.circle(self.screen, (0, 180, 220), (cx, cy), 7, 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx - 5, cy), (cx + 5, cy), 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx, cy - 5), (cx, cy + 5), 1)
            lbl_start = self.font_main.render("START (0,0)", True, (0, 190, 220))
            self.screen.blit(lbl_start, (cx + 9, cy - 7))


    def draw_wall_corridor(self, x1, y1, x2, y2, wlen, offset_x):
        if wlen < 10.0: return
        dx = (x2 - x1) / wlen
        dy = (y2 - y1) / wlen
        nx = -dy
        ny = dx

        ext = 150.0  # 15 cm prodloužení přes stěnu
        hw = 55.0    # Tloušťka koridoru (+-55 mm od přímky)

        ax1 = x1 - ext * dx
        ay1 = y1 - ext * dy
        ax2 = x2 + ext * dx
        ay2 = y2 + ext * dy

        c1 = self.to_screen(ax1 + hw * nx, ay1 + hw * ny, offset_x)
        c2 = self.to_screen(ax2 + hw * nx, ay2 + hw * ny, offset_x)
        c3 = self.to_screen(ax2 - hw * nx, ay2 - hw * ny, offset_x)
        c4 = self.to_screen(ax1 - hw * nx, ay1 - hw * ny, offset_x)

        # Fialový koridor (zakázaná zóna pro svíčky)
        pygame.draw.polygon(self.screen, (45, 12, 55), [c1, c2, c3, c4], 0)
        pygame.draw.polygon(self.screen, (160, 45, 200), [c1, c2, c3, c4], 1)

        # Prodloužení osy fialovou čarou
        ep1 = self.to_screen(ax1, ay1, offset_x)
        ep2 = self.to_screen(ax2, ay2, offset_x)
        p1 = self.to_screen(x1, y1, offset_x)
        p2 = self.to_screen(x2, y2, offset_x)
        pygame.draw.line(self.screen, (200, 60, 240), ep1, p1, 1)
        pygame.draw.line(self.screen, (200, 60, 240), p2, ep2, 1)

    def draw_aiming_sector(self, offset_x):
        r_mm = min(self.max_range, self.view_span_mm)
        p_center = self.to_screen(0, 0, offset_x)

        arc_pts = []
        for deg in range(-10, 11, 2):
            rad = math.radians(deg)
            px, py = self.to_screen(r_mm * math.sin(rad), r_mm * math.cos(rad), offset_x)
            arc_pts.append((px, py))

        poly_pts = [p_center] + arc_pts + [p_center]

        s = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
        pygame.draw.polygon(s, (0, 150, 255, 30), [(pt[0] - offset_x, pt[1]) for pt in poly_pts])
        pygame.draw.lines(s, (0, 210, 255, 150), False, [(pt[0] - offset_x, pt[1]) for pt in poly_pts], 1)

        top_center = self.to_screen(0, r_mm, offset_x)
        pygame.draw.line(s, (0, 230, 255, 90), (p_center[0] - offset_x, p_center[1]), (top_center[0] - offset_x, top_center[1]), 1)
        self.screen.blit(s, (offset_x, 0))

        lbl = self.font_main.render("ZÁMĚR ±10° (1. CÍL)", True, (0, 220, 255))
        lx = top_center[0] - lbl.get_width() // 2
        ly = max(180, top_center[1] - 18)
        self.screen.blit(lbl, (lx, ly))

    def draw_left_live(self):
        self.draw_panel_decorations(0, panel_idx=0)
        scale = (PANEL_W / 2.0) / self.view_span_mm

        # Záhlaví panelu 1
        lbl_p1 = self.font_big.render("1. ŽIVÝ RADAR (SENZOR)", True, (0, 220, 255))
        self.screen.blit(lbl_p1, (295, 18))

        # Modrý záměrný výsek 20° (+-10° dopředu)
        self.draw_aiming_sector(0)

        # Body mračna
        cloud = self.last_frame.get("cloud", [])
        n = len(cloud) // 3
        for i in range(n):
            x = cloud[i * 3]
            y = cloud[i * 3 + 1]
            intensity = cloud[i * 3 + 2]
            sx, sy = self.to_screen(x, y, 0)
            if 0 <= sx < PANEL_W and 0 <= sy < PANEL_H:
                v = min(255, max(30, intensity * 2 + 40))
                self.screen.set_at((sx, sy), (v, v, v))

        # Živé stěny a fialový koridor
        for w in self.last_frame.get("walls", []):
            self.draw_wall_corridor(w[0], w[1], w[2], w[3], w[4], 0)
            p1 = self.to_screen(w[0], w[1], 0)
            p2 = self.to_screen(w[2], w[3], 0)
            pygame.draw.line(self.screen, (0, 255, 120), p1, p2, 4)
            pygame.draw.circle(self.screen, (100, 255, 160), p1, 3)
            pygame.draw.circle(self.screen, (100, 255, 160), p2, 3)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            lbl = self.font_main.render(f"STĚNA {int(w[4])}mm", True, (0, 255, 120))
            self.screen.blit(lbl, (mx + 5, my - 6))

        # Živé svíčky
        for c in self.last_frame.get("candles", []):
            scx, scy = self.to_screen(c[0], c[1], 0)
            r_ring = int(200.0 * scale)
            d = int(math.hypot(c[0], c[1]))
            ang_deg = math.degrees(math.atan2(c[0], c[1])) if c[1] != 0 else 0.0
            in_sector = (abs(ang_deg) <= 10.0 and c[1] > 0)

            if in_sector:
                # Zvýraznění prioritního 1. cíle v záměrném výseku
                pygame.draw.circle(self.screen, (0, 220, 255), (scx, scy), max(4, r_ring), 2)
                pygame.draw.circle(self.screen, (0, 255, 255), (scx, scy), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (scx, scy), 3)
                pygame.draw.line(self.screen, (0, 255, 255), (scx - 14, scy), (scx + 14, scy), 1)
                pygame.draw.line(self.screen, (0, 255, 255), (scx, scy - 14), (scx, scy + 14), 1)
                lbl = self.font_bold.render(f"★ 1. CÍL ({d}mm, {ang_deg:+.1f}°)", True, (0, 240, 255))
                self.screen.blit(lbl, (scx + 14, scy - 12))
            else:
                pygame.draw.circle(self.screen, (255, 80, 80, 50), (scx, scy), max(4, r_ring), 1)
                pygame.draw.circle(self.screen, (255, 40, 40), (scx, scy), 6)
                pygame.draw.circle(self.screen, (255, 220, 220), (scx, scy), 2)
                lbl = self.font_bold.render(f"SVÍČKA ({d}mm)", True, (255, 80, 80))
                self.screen.blit(lbl, (scx + 10, scy - 10))

        # ============================================================
        # 1. TELEMETRICKÝ PANEL (V LEVÉM HORNÍM ROHU)
        # ============================================================
        panel = pygame.Surface((260, 200))
        panel.set_alpha(215)
        panel.fill((10, 12, 16))
        self.screen.blit(panel, (12, 12))
        pygame.draw.rect(self.screen, (30, 45, 60), (12, 12, 260, 200), 1)

        f = self.last_frame
        fps      = f.get("fps", 0.0)
        pts      = f.get("pts", 0)
        calc_us  = f.get("calc_us", 0)
        heap     = f.get("heap", 0)
        crc_err  = f.get("crc_err", 0)
        n_walls  = len(f.get("walls", []))
        n_cands  = len(f.get("candles", []))

        y = 20
        title = self.font_big.render("ESP32 D200 DETEKTOR", True, ACCENT_COLOR)
        self.screen.blit(title, (22, y)); y += 24

        def row(label, val, col=(200, 210, 225)):
            nonlocal y
            t1 = self.font_main.render(label, True, (130, 140, 155))
            t2 = self.font_bold.render(str(val), True, col)
            self.screen.blit(t1, (22, y))
            self.screen.blit(t2, (155, y))
            y += 18

        row("Frekvence skenu:", f"{fps:.1f} Hz", (100, 255, 120) if fps > 4.5 else (255, 150, 0))
        row("Filtrované body:", f"{pts} bodů")
        row("Výpočet ESP32:", f"{calc_us/1000.0:.2f} ms", (0, 220, 255))
        row("Volná RAM ESP32:", f"{heap // 1024} kB")
        row("CRC8 Chyby:", f"{crc_err}", (255, 80, 80) if crc_err > 0 else (120, 200, 120))
        row("Nalezené stěny:", f"{n_walls} (zeleně)", (0, 255, 100))
        row("Nalezené svíčky:", f"{n_cands} (červeně)", (255, 80, 80))

        # ============================================================
        # 2. PANEL OVLÁDÁNÍ (V LEVÉM DOLNÍM ROHU)
        # ============================================================
        cmd_panel = pygame.Surface((576, 95))
        cmd_panel.set_alpha(210)
        cmd_panel.fill((10, 12, 16))
        self.screen.blit(cmd_panel, (12, PANEL_H - 105))
        pygame.draw.rect(self.screen, (55, 45, 30), (12, PANEL_H - 105, 576, 95), 1)

        cy = PANEL_H - 99
        h_title = self.font_bold.render("LADĚNÍ ZA BĚHU (PŘÍKAZY DO ESP32):", True, (240, 200, 80))
        self.screen.blit(h_title, (22, cy)); cy += 20

        fov_txt = "[F] Režim: 180° Kužel" if self.fov_limited else "[F] Režim: 360° Celé pole"
        freeze_txt = "[MEZERNÍK] ODMRAZIT" if self.frozen else "[MEZERNÍK] ZMRAZIT SKEN"
        
        c1 = self.font_main.render(f"{fov_txt} | {freeze_txt}", True, (220, 220, 220))
        c2 = self.font_main.render(f"Rozsah: [R/E] Min: {self.min_range} mm | [T/G] Max: {self.max_range} mm", True, (170, 180, 195))
        c3 = self.font_main.render(f"Odrazivost: [I/K] Min Intenzita: {self.min_intensity}", True, (170, 180, 195))
        self.screen.blit(c1, (22, cy)); cy += 17
        self.screen.blit(c2, (22, cy)); cy += 17
        self.screen.blit(c3, (22, cy))

        # Info o vybraném bodě po kliknutí
        if self.selected_object:
            sel_panel = pygame.Surface((250, 60))
            sel_panel.set_alpha(220)
            sel_panel.fill((25, 20, 10))
            self.screen.blit(sel_panel, (PANEL_W - 262, 12))
            pygame.draw.rect(self.screen, (255, 180, 0), (PANEL_W - 262, 12, 250, 60), 1)

            st = self.font_bold.render("INSPEKCE BODU:", True, (255, 200, 50))
            self.screen.blit(st, (PANEL_W - 254, 18))
            sx_mm, sy_mm, s_dist = self.selected_object
            s_info = self.font_main.render(f"X:{sx_mm:.0f} Y:{sy_mm:.0f} | R:{s_dist:.0f}mm", True, (255, 255, 255))
            self.screen.blit(s_info, (PANEL_W - 254, 40))

    def draw_mid_original(self):
        """2. PANEL: Původní statická mapa (referenční etalon hřiště nasbíraný z výchozího bodu)."""
        offset_x = PANEL_W
        self.draw_panel_decorations(offset_x, panel_idx=1)
        scale = (PANEL_W / 2.0) / self.view_span_mm

        loc = self.localizer
        if loc.locked:
            conf_candles = loc.ref_candles
            conf_walls   = loc.ref_walls
        else:
            conf_candles = self.ref_model.get_confirmed_candles()
            conf_walls   = self.ref_model.get_confirmed_walls()

        # 1. Stěny a koridory původní referenční mapy
        for w in conf_walls:
            self.draw_wall_corridor(w['x1'], w['y1'], w['x2'], w['y2'], w['len'], offset_x)
            p1 = self.to_screen(w['x1'], w['y1'], offset_x)
            p2 = self.to_screen(w['x2'], w['y2'], offset_x)
            pygame.draw.line(self.screen, (60, 230, 120), p1, p2, 5)
            pygame.draw.circle(self.screen, (130, 255, 170), p1, 4)
            pygame.draw.circle(self.screen, (130, 255, 170), p2, 4)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            lbl = self.font_bold.render(f"STĚNA {int(w['len'])}mm", True, (60, 230, 120))
            self.screen.blit(lbl, (mx + 6, my - 7))

        # 2. Invariantní geometrické kotvy a trojúhelníkový vzorec
        if loc.locked and loc.anchors:
            # Vykreslení kotev podle hierarchie (od méně významných k nejdůležitějším)
            for a in reversed(loc.anchors):
                p1 = self.to_screen(a['p1'][0], a['p1'][1], offset_x)
                p2 = self.to_screen(a['p2'][0], a['p2'][1], offset_x)
                w = 3 if a['rank'] <= 2 else 1
                pygame.draw.line(self.screen, a['color'], p1, p2, w)
                pygame.draw.circle(self.screen, a['color'], p1, 5, 1)
                pygame.draw.circle(self.screen, a['color'], p2, 5, 1)
                mid_x = (p1[0] + p2[0]) // 2
                mid_y = (p1[1] + p2[1]) // 2
                lbl_a = self.font_bold.render(f"{a['name']}: {a['dist']:.0f} mm", True, a['color'])
                self.screen.blit(lbl_a, (mid_x + 5, mid_y - 8))

            # Pokud máme trojúhelníkový vzorec, vykreslíme i třetí stranu a úhel u pivotu
            if loc.triangle_pattern:
                tp = loc.triangle_pattern
                p_piv = self.to_screen(tp['pivot_pt'][0], tp['pivot_pt'][1], offset_x)
                p_p1  = self.to_screen(tp['p1_pt'][0], tp['p1_pt'][1], offset_x)
                p_p2  = self.to_screen(tp['p2_pt'][0], tp['p2_pt'][1], offset_x)
                # Třetí strana trojúhelníku (tyrkysově)
                pygame.draw.line(self.screen, (0, 220, 200), p_p1, p_p2, 2)
                t_mid_x = (p_p1[0] + p_p2[0]) // 2
                t_mid_y = (p_p1[1] + p_p2[1]) // 2
                self.screen.blit(self.font_main.render(f"L3: {tp['dist3']:.0f} mm", True, (0, 220, 200)), (t_mid_x + 4, t_mid_y - 6))

                # Úhel u pivotu
                lbl_ang = self.font_bold.render(f"∠ {tp['angle_deg']:.1f}°", True, (255, 255, 255))
                self.screen.blit(lbl_ang, (p_piv[0] + 12, p_piv[1] + 10))
        elif len(conf_candles) >= 2:
            # Před zamknutím: orientační vzdálenosti mezi nalezenými svíčkami
            for i in range(len(conf_candles)):
                for j in range(i + 1, len(conf_candles)):
                    c1 = conf_candles[i]
                    c2 = conf_candles[j]
                    d_mm = math.hypot(c1['x'] - c2['x'], c1['y'] - c2['y'])
                    p1 = self.to_screen(c1['x'], c1['y'], offset_x)
                    p2 = self.to_screen(c2['x'], c2['y'], offset_x)
                    pygame.draw.line(self.screen, (130, 110, 35), p1, p2, 1)
                    mid_x = (p1[0] + p2[0]) // 2
                    mid_y = (p1[1] + p2[1]) // 2
                    lbl_d = self.font_main.render(f"{d_mm:.0f} mm", True, (215, 185, 60))
                    self.screen.blit(lbl_d, (mid_x + 3, mid_y - 6))

        # 3. Svíčky původní referenční mapy
        target_cand = None
        for idx, c in enumerate(conf_candles):
            scx, scy = self.to_screen(c['x'], c['y'], offset_x)
            r_ring = int(220.0 * scale)
            d = int(math.hypot(c['x'], c['y']))
            ang_deg = math.degrees(math.atan2(c['x'], c['y'])) if c['y'] != 0 else 0.0
            in_sector = (abs(ang_deg) <= 10.0 and c['y'] > 0)
            walls_cnt = c.get('walls_count', 0)
            wall_info = f"[{walls_cnt} stěn]" if walls_cnt > 0 else "[BEZ STĚNY]"

            # Role v hierarchii kotev
            role_tag = ""
            if loc.locked and loc.triangle_pattern:
                tp = loc.triangle_pattern
                if idx == tp['pivot_idx']:
                    role_tag = " [PIVOT K1+K2]"
                elif idx == tp['other1_idx']:
                    role_tag = " [K1]"
                elif idx == tp['other2_idx']:
                    role_tag = " [K2]"
            elif loc.locked and loc.anchors:
                a1 = loc.anchors[0]
                if idx in (a1['idx1'], a1['idx2']):
                    role_tag = " [K1]"

            if in_sector and target_cand is None:
                target_cand = c
                pygame.draw.circle(self.screen, (0, 180, 220), (scx, scy), max(4, r_ring), 1)
                pygame.draw.circle(self.screen, (0, 255, 255), (scx, scy), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (scx, scy), 3)
                pygame.draw.line(self.screen, (0, 255, 255), (scx - 12, scy), (scx + 12, scy), 1)
                pygame.draw.line(self.screen, (0, 255, 255), (scx, scy - 12), (scx, scy + 12), 1)
                lbl = self.font_bold.render(f"★ 1. CÍL ({d}mm, {ang_deg:+.1f}°) {wall_info}{role_tag}", True, (0, 240, 255))
                self.screen.blit(lbl, (scx + 14, scy - 12))
            else:
                ring_col = (255, 140, 80) if walls_cnt > 0 else (255, 60, 60)
                pygame.draw.circle(self.screen, ring_col, (scx, scy), max(4, r_ring), 1)
                pygame.draw.circle(self.screen, (255, 40, 40), (scx, scy), 7)
                pygame.draw.circle(self.screen, (255, 240, 180), (scx, scy), 3)
                lbl_color = (255, 130, 130) if walls_cnt > 0 else (255, 80, 80)
                lbl = self.font_bold.render(f"SVÍČKA #{idx+1} ({d}mm) {wall_info}{role_tag}", True, lbl_color)
                self.screen.blit(lbl, (scx + 12, scy - 12))

        # 4. Trasa mezi potvrzenými svíčkami
        if len(conf_candles) >= 2:
            unvisited = list(conf_candles)
            route = [(0.0, 0.0)]
            curr = (0.0, 0.0)
            if target_cand and target_cand in unvisited:
                unvisited.remove(target_cand)
                route.append((target_cand['x'], target_cand['y']))
                curr = (target_cand['x'], target_cand['y'])
            while unvisited:
                best_i = 0
                best_d = 999999.0
                for i, c in enumerate(unvisited):
                    d = math.hypot(c['x'] - curr[0], c['y'] - curr[1])
                    if d < best_d:
                        best_d = d
                        best_i = i
                next_c = unvisited.pop(best_i)
                route.append((next_c['x'], next_c['y']))
                curr = (next_c['x'], next_c['y'])
            for i in range(1, len(route)):
                p_from = self.to_screen(route[i-1][0], route[i-1][1], offset_x)
                p_to   = self.to_screen(route[i][0], route[i][1], offset_x)
                pygame.draw.line(self.screen, (255, 215, 0), p_from, p_to, 2)
                mid_x = (p_from[0] + p_to[0]) // 2
                mid_y = (p_from[1] + p_to[1]) // 2
                self.screen.blit(self.font_bold.render(f"-> {i}", True, (255, 230, 80)), (mid_x + 4, mid_y - 6))

        # 5. Horní stavový box Panelu 2
        stat_box = pygame.Surface((570, 56))
        stat_box.set_alpha(220)
        stat_box.fill((10, 12, 16))
        self.screen.blit(stat_box, (offset_x + 15, 12))
        border_col = (80, 255, 160) if loc.locked else (240, 180, 40)
        pygame.draw.rect(self.screen, border_col, (offset_x + 15, 12, 570, 56), 1)

        if loc.locked:
            t_stat = self.font_big.render("2. PŮVODNÍ STATICKÁ MAPA 🔒 ZAMČENA", True, (80, 255, 160))
            sub_str = f"Referenční etalon: {len(conf_candles)} svíček, {len(conf_walls)} stěn | Pevná geometrie uložena"
        else:
            t_stat = self.font_big.render("2. PŮVODNÍ STATICKÁ MAPA [SBĚR]", True, (240, 180, 40))
            sub_str = f"Svíčky: {len(conf_candles)}/4 | Stěny: {len(conf_walls)}/6 | Skenů: {self.ref_model.total_scans} (stiskni [L] pro zamknutí)"
        self.screen.blit(t_stat, (offset_x + 25, 17))
        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (offset_x + 25, 40))

        # 6. Spodní nápověda Panelu 2
        tip_box = pygame.Surface((570, 75))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (offset_x + 15, PANEL_H - 85))
        pygame.draw.rect(self.screen, (50, 45, 30), (offset_x + 15, PANEL_H - 85, 570, 75), 1)
        self.screen.blit(self.font_bold.render("REFERENČNÍ MAPA HŘIŠTĚ:", True, (240, 200, 80)), (offset_x + 25, PANEL_H - 79))
        self.screen.blit(self.font_main.render("• Slouží jako pevný souřadnicový systém vygenerovaný na výchozím místě robota.", True, (200, 210, 225)), (offset_x + 25, PANEL_H - 61))
        self.screen.blit(self.font_main.render("• [L] Zamknout mapu jako etalon | [C] Resetovat a sbírat znovu", True, (150, 160, 180)), (offset_x + 25, PANEL_H - 43))

    def draw_right_comparison(self):
        """3. PANEL: Nová statická mapa & Porovnání s referenční mapou a rozhodovací prvky."""
        offset_x = PANEL_W * 2
        self.draw_panel_decorations(offset_x, panel_idx=2)
        scale = (PANEL_W / 2.0) / self.view_span_mm
        loc = self.localizer

        if not loc.locked:
            # Informační panel o neaktivní lokalizaci (čeká na [L])
            msg_box = pygame.Surface((540, 130))
            msg_box.set_alpha(220)
            msg_box.fill((12, 14, 18))
            bx = offset_x + 30
            by = PANEL_H // 2 - 65
            self.screen.blit(msg_box, (bx, by))
            pygame.draw.rect(self.screen, (70, 60, 40), (bx, by, 540, 130), 1)
            
            t1 = self.font_big.render("3. NOVÁ MAPA & POROVNÁNÍ", True, (200, 200, 200))
            t2 = self.font_bold.render("Čekám na zamknutí výchozí mapy...", True, (240, 180, 50))
            t3 = self.font_main.render("1. Nechte na výchozím místě stabilizovat mapu v panelu 2.", True, (160, 170, 185))
            t4 = self.font_main.render("2. Stiskněte [L] pro zamknutí etalonu a aktivaci lokalizace robota.", True, (80, 255, 160))
            t5 = self.font_main.render("3. Po posunu robota se zde zobrazí poloha a rozhodovací přímky.", True, (255, 120, 220))
            self.screen.blit(t1, (bx + 20, by + 12))
            self.screen.blit(t2, (bx + 20, by + 36))
            self.screen.blit(t3, (bx + 20, by + 60))
            self.screen.blit(t4, (bx + 20, by + 80))
            self.screen.blit(t5, (bx + 20, by + 100))
            return

        # ========================================================
        # LOKALIZACE JE AKTIVNÍ: ZOBRAZENÍ SROVNÁNÍ
        # ========================================================

        # 1. Tlumená referenční mapa na pozadí (ground truth etalon)
        for rw in loc.ref_walls:
            p1 = self.to_screen(rw['x1'], rw['y1'], offset_x)
            p2 = self.to_screen(rw['x2'], rw['y2'], offset_x)
            pygame.draw.line(self.screen, (25, 75, 45), p1, p2, 3)
            pygame.draw.circle(self.screen, (40, 110, 65), p1, 3)
            pygame.draw.circle(self.screen, (40, 110, 65), p2, 3)

        for rc in loc.ref_candles:
            scx, scy = self.to_screen(rc['x'], rc['y'], offset_x)
            pygame.draw.circle(self.screen, (100, 50, 50), (scx, scy), 8, 1)
            pygame.draw.line(self.screen, (100, 50, 50), (scx - 4, scy), (scx + 4, scy), 1)
            pygame.draw.line(self.screen, (100, 50, 50), (scx, scy - 4), (scx, scy + 4), 1)

        # 2. Vektor posunu od počátku START (0,0) k robotu
        p_start = self.to_screen(0, 0, offset_x)
        p_rob   = self.to_screen(loc.robot_x, loc.robot_y, offset_x)
        dist_from_start = math.hypot(loc.robot_x, loc.robot_y)

        if dist_from_start > 15.0:
            pygame.draw.line(self.screen, (150, 70, 170), p_start, p_rob, 2)
            vmx = (p_start[0] + p_rob[0]) // 2
            vmy = (p_start[1] + p_rob[1]) // 2
            lbl_posun = self.font_bold.render(f"Δ {dist_from_start:.0f} mm", True, (220, 120, 255))
            self.screen.blit(lbl_posun, (vmx + 6, vmy - 8))

        # 3. Trajektorie pohybu robota
        if len(loc.trajectory) >= 2:
            traj_px = [self.to_screen(tp[0], tp[1], offset_x) for tp in loc.trajectory]
            pygame.draw.lines(self.screen, (220, 50, 140), False, traj_px, 2)
            for tpt in traj_px[::4]:
                pygame.draw.circle(self.screen, (255, 100, 180), tpt, 2)

        # 4. Potvrzené objekty z Nové stabilní mapy přetransformované do souřadnic hřiště
        # Navigační systém: +X doprava, +Y dopředu, heading po směru hodinových ručiček
        heading_rad = math.radians(loc.robot_deg)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        curr_walls = self.curr_model.get_confirmed_walls()
        curr_cands = self.curr_model.get_confirmed_candles()

        # Transformované stěny z Nové stabilní mapy
        for w in curr_walls:
            tw_x1 = loc.robot_x + w['x1'] * cos_h + w['y1'] * sin_h
            tw_y1 = loc.robot_y - w['x1'] * sin_h + w['y1'] * cos_h
            tw_x2 = loc.robot_x + w['x2'] * cos_h + w['y2'] * sin_h
            tw_y2 = loc.robot_y - w['x2'] * sin_h + w['y2'] * cos_h

            self.draw_wall_corridor(tw_x1, tw_y1, tw_x2, tw_y2, w['len'], offset_x)
            p1 = self.to_screen(tw_x1, tw_y1, offset_x)
            p2 = self.to_screen(tw_x2, tw_y2, offset_x)
            pygame.draw.line(self.screen, (0, 255, 120), p1, p2, 4)
            pygame.draw.circle(self.screen, (100, 255, 160), p1, 3)
            pygame.draw.circle(self.screen, (100, 255, 160), p2, 3)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            lbl = self.font_bold.render(f"STĚNA {int(w['len'])}mm", True, (0, 255, 120))
            self.screen.blit(lbl, (mx + 6, my - 7))

        # Transformované svíčky z Nové stabilní mapy
        for c in curr_cands:
            tc_x = loc.robot_x + c['x'] * cos_h + c['y'] * sin_h
            tc_y = loc.robot_y - c['x'] * sin_h + c['y'] * cos_h
            scx, scy = self.to_screen(tc_x, tc_y, offset_x)
            pygame.draw.circle(self.screen, (255, 50, 120), (scx, scy), 8)
            pygame.draw.circle(self.screen, (255, 230, 255), (scx, scy), 3)
            w_info = f"[{c.get('walls_count', 0)} stěn]" if c.get('walls_count', 0) > 0 else "[BEZ STĚNY]"
            lbl = self.font_bold.render(f"NOVÁ SVÍČKA {w_info}", True, (255, 90, 160))
            self.screen.blit(lbl, (scx + 10, scy - 10))

        # ========================================================
        # 5. BAREVNÉ ROZHODOVACÍ PŘÍMKY A KORESPONDENCE (MATCHING)
        # ========================================================
        match = loc.last_match_info or {}
        active_anchors = match.get('active_anchors', [])
        pattern_name = match.get('pattern_name', "HLEDÁM KOTVY...")

        # A) Aktivní kotevní přímky použité pro výpočet pozice
        for anc in active_anchors:
            ap1 = self.to_screen(anc['p1'][0], anc['p1'][1], offset_x)
            ap2 = self.to_screen(anc['p2'][0], anc['p2'][1], offset_x)
            col = anc.get('color', (255, 215, 0))
            pygame.draw.line(self.screen, col, ap1, ap2, 3)
            pygame.draw.circle(self.screen, col, ap1, 7, 2)
            pygame.draw.circle(self.screen, col, ap2, 7, 2)
            amx = (ap1[0] + ap2[0]) // 2
            amy = (ap1[1] + ap2[1]) // 2
            lbl_k = self.font_bold.render(f"★ {anc['name']} ({anc['dist']:.0f} mm)", True, col)
            self.screen.blit(lbl_k, (amx + 6, amy - 8))

        # Pokud se shoduje plný trojúhelník, vykreslíme spojnici třetí strany L3
        if "TROJÚHELNÍK" in pattern_name and loc.triangle_pattern:
            tp = loc.triangle_pattern
            p_p1 = self.to_screen(tp['p1_pt'][0], tp['p1_pt'][1], offset_x)
            p_p2 = self.to_screen(tp['p2_pt'][0], tp['p2_pt'][1], offset_x)
            pygame.draw.line(self.screen, (0, 220, 200), p_p1, p_p2, 2)
            t_mid_x = (p_p1[0] + p_p2[0]) // 2
            t_mid_y = (p_p1[1] + p_p2[1]) // 2
            self.screen.blit(self.font_bold.render(f"L3: {tp['dist3']:.0f} mm (TROJÚHELNÍK OK)", True, (0, 220, 200)), (t_mid_x + 5, t_mid_y - 6))

        # B) Tyrkysové/azurové paprsky korespondence mezi referencí a novou pozicí
        for (ref_pt, cur_world_pt, err_mm) in match.get('inlier_pairs', []):
            rp = self.to_screen(ref_pt[0], ref_pt[1], offset_x)
            cp = self.to_screen(cur_world_pt[0], cur_world_pt[1], offset_x)
            pygame.draw.line(self.screen, (0, 240, 255), rp, cp, 2)
            pygame.draw.circle(self.screen, (0, 255, 255), cp, 4)
            if err_mm > 5.0:
                mid_x = (rp[0] + cp[0]) // 2
                mid_y = (rp[1] + cp[1]) // 2
                lbl_e = self.font_main.render(f"±{err_mm:.0f}mm", True, (0, 240, 255))
                self.screen.blit(lbl_e, (mid_x + 4, mid_y - 6))

        # C) Zelené paprsky korespondence pro stěny
        for (ref_pt, cur_world_pt, err_mm) in match.get('matched_walls', []):
            rp = self.to_screen(ref_pt[0], ref_pt[1], offset_x)
            cp = self.to_screen(cur_world_pt[0], cur_world_pt[1], offset_x)
            pygame.draw.line(self.screen, (80, 255, 140), rp, cp, 1)

        # ========================================================
        # 6. POHYBLIVÝ RŮŽOVÝ ROBOT
        # ========================================================
        PINK_MAIN = (255, 70, 180)
        PINK_BG   = (65, 15, 45)
        PINK_LITE = (255, 150, 215)
        rx, ry = p_rob

        # Zorný kužel LiDARu dopředu
        cone_r_mm = 650.0
        cone_pts = [(rx, ry)]
        for deg_off in range(-20, 21, 5):
            a_rad = heading_rad + math.radians(deg_off)
            kx = loc.robot_x + cone_r_mm * math.sin(a_rad)
            ky = loc.robot_y + cone_r_mm * math.cos(a_rad)
            cone_pts.append(self.to_screen(kx, ky, offset_x))
        if len(cone_pts) > 2:
            s_cone = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
            cone_local = [(pt[0] - offset_x, pt[1]) for pt in cone_pts]
            pygame.draw.polygon(s_cone, (255, 60, 170, 25), cone_local)
            pygame.draw.lines(s_cone, (255, 80, 190, 80), False, cone_local, 1)
            self.screen.blit(s_cone, (offset_x, 0))

        # Tělo robota: Růžový čtverec / šasi (240 x 240 mm)
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
        arrow_len = 26
        tip_x = int(rx + arrow_len * sin_h)
        tip_y = int(ry - arrow_len * cos_h)
        pygame.draw.line(self.screen, PINK_MAIN, (rx, ry), (tip_x, tip_y), 3)
        wing_len = 8
        wing_rad = math.radians(145)
        for sgn in [-1, 1]:
            w_ang = heading_rad + sgn * wing_rad
            wx2 = int(tip_x + wing_len * math.sin(w_ang))
            wy2 = int(tip_y - wing_len * math.cos(w_ang))
            pygame.draw.line(self.screen, PINK_MAIN, (tip_x, tip_y), (wx2, wy2), 2)

        pygame.draw.circle(self.screen, PINK_LITE, (rx, ry), 5)
        pygame.draw.circle(self.screen, (255, 255, 255), (rx, ry), 2)

        # Text u robota
        conf_pct = int(loc.pose_confidence * 100)
        stat_tag = " [KLID]" if loc.is_stationary else " [POHYB]"
        lbl_rob = self.font_bold.render(
            f"ROBOT ({loc.robot_x:+.0f}, {loc.robot_y:+.0f}mm) {loc.robot_deg:+.0f}°{stat_tag}",
            True, PINK_LITE)
        lbl_x = min(rx + 16, offset_x + PANEL_W - lbl_rob.get_width() - 8)
        self.screen.blit(lbl_rob, (lbl_x, ry - 10))

        # ========================================================
        # 7. HORNÍ STAVOVÝ PANEL PANELU 3
        # ========================================================
        stat_box = pygame.Surface((570, 56))
        stat_box.set_alpha(220)
        stat_box.fill((10, 12, 16))
        self.screen.blit(stat_box, (offset_x + 15, 12))

        is_ready = (self.curr_model.total_scans >= 4 and (len(curr_cands) + len(curr_walls)) >= 2 and match.get('inliers', 0) >= 2)
        border_col = (80, 255, 160) if is_ready else (240, 200, 60)
        pygame.draw.rect(self.screen, border_col, (offset_x + 15, 12, 570, 56), 1)

        inliers_cnt = match.get('inliers', 0)
        avg_err_val = match.get('avg_error', 0.0)

        if is_ready:
            t_stat = self.font_big.render(f"3. SHODA: {pattern_name}", True, (80, 255, 160))
            sub_str = f"Inliery: {inliers_cnt} | Chyba: {avg_err_val:.1f} mm | Od startu: {dist_from_start:.0f} mm (skenů: {self.curr_model.total_scans})"
        else:
            t_stat = self.font_big.render("3. NOVÁ MAPA: SBÍRÁM A POROVNÁVÁM...", True, (240, 200, 60))
            sub_str = f"Skeny: {self.curr_model.total_scans}/6 | Svíčky: {len(curr_cands)} | Stěny: {len(curr_walls)} | Hledám kotvy..."

        self.screen.blit(t_stat, (offset_x + 25, 17))
        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (offset_x + 25, 40))

        # Barevný pruh jistoty
        bar_w = 540
        fill_w = int(bar_w * loc.pose_confidence)
        pygame.draw.rect(self.screen, (35, 20, 30), (offset_x + 25, 54, bar_w, 6))
        bar_col = (min(255, int(255 * (1.0 - loc.pose_confidence))),
                   min(255, int(80 + 175 * loc.pose_confidence)), 120)
        pygame.draw.rect(self.screen, bar_col, (offset_x + 25, 54, fill_w, 6))

        # ========================================================
        # 8. SPODNÍ NÁPOVĚDA A LEGENDA ROZHODOVÁNÍ
        # ========================================================
        tip_box = pygame.Surface((570, 75))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (offset_x + 15, PANEL_H - 85))
        pygame.draw.rect(self.screen, (60, 30, 50), (offset_x + 15, PANEL_H - 85, 570, 75), 1)
        self.screen.blit(self.font_bold.render("HIERARCHICKÉ KOTVY A POROVNÁNÍ MAP:", True, (255, 150, 215)), (offset_x + 25, PANEL_H - 79))
        self.screen.blit(self.font_main.render("• Zlatá: KOTVA 1 (Primární) | Oranžová: KOTVA 2 (Záloha) | Tyrkys: Trojúhelník", True, (240, 220, 80)), (offset_x + 25, PANEL_H - 61))
        self.screen.blit(self.font_main.render("• [N] Reset a nový sběr na této pozici | [L] Odemknout/Zamknout výchozí start", True, (160, 200, 225)), (offset_x + 25, PANEL_H - 43))

    def draw_field_map(self):
        """4. PANEL (DOLE): Obdélníková mapa celého hřiště s dynamickým auto-fitem podle stěny 2.4/3.5 m."""
        fm = self.field_map
        y_offset = PANEL_H  # Y = 560 px
        xmin, xmax, ymin, ymax = fm.get_auto_field_bounds()

        # Pozadí spodního panelu
        panel_rect = pygame.Rect(0, y_offset, WIN_W, FIELD_H)
        pygame.draw.rect(self.screen, (12, 14, 18), panel_rect)

        # ========================================================
        # 1. LEVÝ ODDÍL: GRAFICKÝ OBDÉLNÍK HŘIŠTĚ (Auto-fit)
        # ========================================================
        avail_w = 510.0
        avail_h = float(FIELD_H - 58)  # ~242 px

        field_w_mm = xmax - xmin
        field_h_mm = ymax - ymin
        scale = min(avail_w / max(100.0, field_w_mm), avail_h / max(100.0, field_h_mm))

        draw_w = int(field_w_mm * scale)
        draw_h = int(field_h_mm * scale)

        field_x0 = 20 + int((avail_w - draw_w) / 2.0)
        field_y0 = y_offset + 14 + int((avail_h - draw_h) / 2.0)

        def f_to_screen(fx, fy):
            sx = field_x0 + int((fx - xmin) * scale)
            sy = field_y0 + draw_h - int((fy - ymin) * scale)
            return sx, sy

        # Plocha hřiště (tmavě zelená)
        f_rect_pts = [
            f_to_screen(xmin, ymin),
            f_to_screen(xmax, ymin),
            f_to_screen(xmax, ymax),
            f_to_screen(xmin, ymax)
        ]
        pygame.draw.polygon(self.screen, (22, 48, 32), f_rect_pts, 0)
        pygame.draw.polygon(self.screen, (50, 180, 90), f_rect_pts, 2)

        # Vnitřní mřížka po 500 mm
        gx = math.ceil(xmin / 500.0) * 500.0
        while gx <= xmax:
            p1 = f_to_screen(gx, ymin)
            p2 = f_to_screen(gx, ymax)
            pygame.draw.line(self.screen, (28, 62, 40), p1, p2, 1)
            gx += 500.0

        gy = math.ceil(ymin / 500.0) * 500.0
        while gy <= ymax:
            p1 = f_to_screen(xmin, gy)
            p2 = f_to_screen(xmax, gy)
            pygame.draw.line(self.screen, (28, 62, 40), p1, p2, 1)
            gy += 500.0

        # Rozměry u stěn hřiště
        lbl_w = self.font_main.render(f"{field_w_mm:.0f} mm", True, (80, 160, 110))
        self.screen.blit(lbl_w, ((f_rect_pts[0][0] + f_rect_pts[1][0]) // 2 - lbl_w.get_width() // 2, f_rect_pts[0][1] + 2))
        lbl_h = self.font_main.render(f"{field_h_mm:.0f} mm", True, (80, 160, 110))
        self.screen.blit(lbl_h, (f_rect_pts[0][0] - lbl_h.get_width() - 4, (f_rect_pts[0][1] + f_rect_pts[3][1]) // 2 - 6))

        # Stěny uvnitř pole
        field_walls = fm.get_field_walls()
        for w in field_walls:
            wp1 = f_to_screen(w['x1'], w['y1'])
            wp2 = f_to_screen(w['x2'], w['y2'])
            pygame.draw.line(self.screen, (40, 255, 120), wp1, wp2, 3)
            pygame.draw.circle(self.screen, (100, 255, 160), wp1, 3)
            pygame.draw.circle(self.screen, (100, 255, 160), wp2, 3)

        # Svíčky uvnitř pole
        field_candles = fm.get_field_candles()
        for idx, c in enumerate(field_candles):
            csx, csy = f_to_screen(c['x'], c['y'])
            has_wall = c.get('walls_count', 0) > 0
            ring_col = (255, 150, 60) if has_wall else (255, 60, 60)
            pygame.draw.circle(self.screen, (40, 10, 10), (csx, csy), 10)
            pygame.draw.circle(self.screen, ring_col, (csx, csy), 9, 2)
            pygame.draw.circle(self.screen, (255, 40, 40), (csx, csy), 5)
            pygame.draw.circle(self.screen, (255, 255, 200), (csx, csy), 2)
            lbl_c = self.font_bold.render(f"S{idx+1}", True, (255, 230, 180))
            self.screen.blit(lbl_c, (csx + 10, csy - 7))

        # START pozice [0, 0]
        start_sx, start_sy = f_to_screen(0.0, 0.0)
        pygame.draw.circle(self.screen, (0, 180, 255), (start_sx, start_sy), 6, 1)
        pygame.draw.line(self.screen, (0, 220, 255), (start_sx - 6, start_sy), (start_sx + 6, start_sy), 1)
        pygame.draw.line(self.screen, (0, 220, 255), (start_sx, start_sy - 6), (start_sx, start_sy + 6), 1)
        self.screen.blit(self.font_main.render("START (0,0)", True, (0, 200, 255)), (start_sx + 8, start_sy - 6))

        # Trajektorie robota
        if len(fm.trajectory) >= 2:
            traj_px = [f_to_screen(tp[0], tp[1]) for tp in fm.trajectory]
            pygame.draw.lines(self.screen, (230, 60, 150), False, traj_px, 2)

        # Aktuální pozice robota (pokud se pohnul)
        rx, ry = fm.robot_pos
        if math.hypot(rx, ry) > 15.0 or abs(fm.robot_deg) > 2.0:
            rob_sx, rob_sy = f_to_screen(rx, ry)
            h_rad = math.radians(fm.robot_deg)
            pygame.draw.circle(self.screen, (255, 80, 180), (rob_sx, rob_sy), 5)
            tip_x = int(rob_sx + 14 * math.sin(h_rad))
            tip_y = int(rob_sy - 14 * math.cos(h_rad))
            pygame.draw.line(self.screen, (255, 120, 220), (rob_sx, rob_sy), (tip_x, tip_y), 2)

        # Ovládací indikátory startovní stěny pod hřištěm
        w_active = fm.start_wall_mode
        info_y = field_y0 + draw_h + 8
        t_st = self.font_bold.render("START. STĚNA:", True, (200, 210, 225))
        self.screen.blit(t_st, (field_x0, info_y))
        tx = field_x0 + t_st.get_width() + 8

        # Tlačítko [2/ě] 2.4 m
        is_2 = (w_active == 2)
        col_2_bg = (30, 75, 45) if is_2 else (22, 26, 32)
        col_2_txt = (100, 255, 140) if is_2 else (130, 140, 155)
        tag_2 = "[2/ě] 2.4 m KRATŠÍ [AKTIVNÍ]" if is_2 else "[2/ě] 2.4 m"
        lbl_2 = self.font_bold.render(tag_2, True, col_2_txt)
        r2 = pygame.Rect(tx, info_y - 2, lbl_2.get_width() + 8, 18)
        pygame.draw.rect(self.screen, col_2_bg, r2, 0, 3)
        pygame.draw.rect(self.screen, col_2_txt if is_2 else (45, 52, 65), r2, 1, 3)
        self.screen.blit(lbl_2, (tx + 4, info_y))
        tx += r2.width + 6

        # Tlačítko [3/š] 3.5 m
        is_3 = (w_active == 3)
        col_3_bg = (30, 75, 45) if is_3 else (22, 26, 32)
        col_3_txt = (100, 255, 140) if is_3 else (130, 140, 155)
        tag_3 = "[3/š] 3.5 m DELŠÍ [AKTIVNÍ]" if is_3 else "[3/š] 3.5 m"
        lbl_3 = self.font_bold.render(tag_3, True, col_3_txt)
        r3 = pygame.Rect(tx, info_y - 2, lbl_3.get_width() + 8, 18)
        pygame.draw.rect(self.screen, col_3_bg, r3, 0, 3)
        pygame.draw.rect(self.screen, col_3_txt if is_3 else (45, 52, 65), r3, 1, 3)
        self.screen.blit(lbl_3, (tx + 4, info_y))

        # Auto-fit rozsah text
        fit_str = f"Auto-fit: X ∈ [{xmin:+.0f}, {xmax:+.0f}] mm | Y ∈ [{ymin:+.0f}, {ymax:+.0f}] mm"
        self.screen.blit(self.font_main.render(fit_str, True, (130, 175, 150)), (field_x0, info_y + 20))

        # ========================================================
        # 2. STŘEDNÍ ODDÍL: KONTROLA PRAVIDEL & DETAIL SVÍČEK
        # ========================================================
        mid_x = 550
        pygame.draw.line(self.screen, (40, 48, 60), (mid_x - 10, y_offset), (mid_x - 10, y_offset + FIELD_H), 1)

        t_y = y_offset + 10
        self.screen.blit(self.font_big.render("GEOMETRICKÁ PRAVIDLA SOUTĚŽE", True, (255, 215, 80)), (mid_x, t_y))
        t_y += 22

        n_c = len(field_candles)
        n_w = len(field_walls)

        # Pravidlo 1: Počet svíček (max 4)
        c_ok = (n_c == 4)
        sym1 = "✓" if c_ok else ("..." if n_c < 4 else "✗")
        col1 = (100, 255, 120) if c_ok else ((240, 180, 50) if n_c < 4 else (255, 70, 70))
        self.screen.blit(self.font_bold.render(f"{sym1} POČET SVÍČEK: {n_c}/4 ({'SPLNĚNO' if c_ok else f'nalezeno {n_c}'})", True, col1), (mid_x, t_y))
        t_y += 17

        # Pravidlo 2: Všechny svíčky v poli (100% UVNITŘ, žádná vně)
        rej_cnt = fm.rejected_outside_count
        all_in = (rej_cnt == 0 and n_c > 0)
        sym_in = "✓" if (rej_cnt == 0) else "✗"
        col_in = (100, 255, 120) if (rej_cnt == 0) else (255, 70, 70)
        msg_in = "všechny uvnitř pole" if rej_cnt == 0 else f"{rej_cnt} bodů vně vyřazeno"
        self.screen.blit(self.font_bold.render(f"{sym_in} HRANICE HŘIŠTĚ: 100% UVNITŘ ({n_c}/{n_c} v poli, {msg_in})", True, col_in), (mid_x, t_y))
        t_y += 17

        # Pravidlo 3: Rozestup min 800 mm
        dist_ok = True
        min_pair_d = 9999.0
        if n_c >= 2:
            for i in range(n_c):
                for j in range(i + 1, n_c):
                    d = math.hypot(field_candles[i]['x'] - field_candles[j]['x'],
                                   field_candles[i]['y'] - field_candles[j]['y'])
                    if d < min_pair_d: min_pair_d = d
                    if d < 780.0: dist_ok = False
        sym2 = "✓" if dist_ok else "✗"
        col2 = (100, 255, 120) if dist_ok else (255, 70, 70)
        d_str = f"(min {min_pair_d:.0f} mm)" if n_c >= 2 else "(čekám na další svíčky)"
        self.screen.blit(self.font_bold.render(f"{sym2} MIN. ROZESTUP 800 mm: {'SPLNĚNO' if dist_ok else 'PORUŠENO'} {d_str}", True, col2), (mid_x, t_y))
        t_y += 17

        # Pravidlo 4: Max 1 volná svíčka bez stěny
        iso_cnt = sum(1 for c in field_candles if c.get('walls_count', 0) == 0)
        iso_ok = (iso_cnt <= 1)
        sym3 = "✓" if iso_ok else "✗"
        col3 = (100, 255, 120) if iso_ok else (255, 70, 70)
        self.screen.blit(self.font_bold.render(f"{sym3} MAX 1 VOLNÁ SVÍČKA: {'SPLNĚNO' if iso_ok else 'PŘEKROČENO'} ({iso_cnt} volná)", True, col3), (mid_x, t_y))
        t_y += 17

        # Pravidlo 5: Stěny (max 6 bez překryvů)
        w_ok = (n_w <= 6)
        sym4 = "✓" if w_ok else "✗"
        col4 = (100, 255, 120) if w_ok else (255, 70, 70)
        self.screen.blit(self.font_bold.render(f"{sym4} STĚNY HŘIŠTĚ: {n_w}/6 (bez kolizí a uvnitř pole)", True, col4), (mid_x, t_y))
        t_y += 22

        # Detailní rozpis jednotlivých svíček
        self.screen.blit(self.font_bold.render("DOSAZENÉ SVÍČKY V POLI [X, Y]:", True, (200, 210, 225)), (mid_x, t_y))
        t_y += 17

        if n_c == 0:
            self.screen.blit(self.font_main.render("Zatím nepotvrzeny žádné stabilní svíčky...", True, (140, 145, 155)), (mid_x, t_y))
        else:
            for i, c in enumerate(field_candles):
                wc = c.get('walls_count', 0)
                wt = f"[U STĚNY: {wc}]" if wc > 0 else "[VOLNÁ SVÍČKA]"
                w_col = (255, 170, 70) if wc > 0 else (255, 100, 100)
                txt = f"S{i+1}: X={c['x']:+.0f} mm, Y={c['y']:+.0f} mm  {wt}  (hitů: {c['hits']})"
                self.screen.blit(self.font_main.render(txt, True, w_col), (mid_x, t_y))
                t_y += 15

        # ========================================================
        # 3. PRAVÝ ODDÍL: CELKOVÝ OBRAZ, LOKALIZACE & POKRYTÍ
        # ========================================================
        right_x = 1200
        pygame.draw.line(self.screen, (40, 48, 60), (right_x - 10, y_offset), (right_x - 10, y_offset + FIELD_H), 1)

        r_y = y_offset + 10
        self.screen.blit(self.font_big.render("CELKOVÝ OBRAZ HŘIŠTĚ", True, (0, 220, 255)), (right_x, r_y))
        r_y += 22

        # Stav mapování
        loc = self.localizer
        if not loc.locked:
            st_txt = "⏳ VÝCHOZÍ SBĚR (stiskni [L] pro zamknutí etalonu)"
            st_col = (240, 180, 50)
        else:
            p_name = loc.last_match_info.get('pattern_name', 'Hledám kotvy') if loc.last_match_info else 'Klid'
            st_txt = f"🔒 MAPOVÁNÍ AKTIVNÍ ({p_name})"
            st_col = (80, 255, 160)
        self.screen.blit(self.font_bold.render(st_txt, True, st_col), (right_x, r_y))
        r_y += 20

        # Pokrytí hřiště bar
        total_items = n_c + n_w
        cov_pct = min(100, int((n_c / 4.0 * 60.0) + (min(6, n_w) / 6.0 * 40.0)))
        cov_col = (80, 255, 160) if cov_pct >= 85 else ((240, 200, 60) if cov_pct >= 50 else (255, 90, 90))
        self.screen.blit(self.font_bold.render(f"POKRYTÍ HŘIŠTĚ: {cov_pct}%", True, cov_col), (right_x, r_y))
        r_y += 16

        bar_w = 340
        pygame.draw.rect(self.screen, (30, 35, 45), (right_x, r_y, bar_w, 8))
        pygame.draw.rect(self.screen, cov_col, (right_x, r_y, int(bar_w * cov_pct / 100.0), 8))
        r_y += 17

        # Poloha robota
        dist_s = math.hypot(rx, ry)
        self.screen.blit(self.font_bold.render(f"POZICE ROBOTA OD STARTU:", True, (255, 130, 200)), (right_x, r_y))
        r_y += 17
        self.screen.blit(self.font_main.render(f"• X: {rx:+.0f} mm | Y: {ry:+.0f} mm | Vzdálenost: {dist_s:.0f} mm", True, (220, 220, 225)), (right_x, r_y))
        r_y += 15
        self.screen.blit(self.font_main.render(f"• Natočení: {fm.robot_deg:+.1f}° | Jistota: {int(loc.pose_confidence * 100)}%", True, (200, 200, 210)), (right_x, r_y))
        r_y += 20

        # Rychlá nápověda kláves
        self.screen.blit(self.font_bold.render("OVLÁDÁNÍ STARTOVNÍCH STĚN A MAPY:", True, (240, 200, 80)), (right_x, r_y))
        r_y += 17
        self.screen.blit(self.font_main.render("• [2] nebo [ě]: Start u KRATŠÍ stěny 2.4 m", True, (100, 255, 140)), (right_x, r_y))
        r_y += 15
        self.screen.blit(self.font_main.render("• [3] nebo [š]: Start u DELŠÍ stěny 3.5 m", True, (100, 255, 140)), (right_x, r_y))
        r_y += 15
        self.screen.blit(self.font_main.render("• [P]: Přepnout stěnu 2.4 m <-> 3.5 m | [L]: Zamknout etalon", True, (200, 215, 230)), (right_x, r_y))
        r_y += 15
        self.screen.blit(self.font_main.render("• [N]: Reset nové mapy na pozici | [C]: Reset všeho", True, (160, 175, 195)), (right_x, r_y))

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.handle_click(event.pos)
                    elif event.button == 4: # Kolečko myši nahoru (zoom in)
                        self.view_span_mm = max(1200.0, self.view_span_mm - 300.0)
                    elif event.button == 5: # Kolečko myši dolů (zoom out)
                        self.view_span_mm = min(6000.0, self.view_span_mm + 300.0)

                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        self.frozen = not self.frozen
                    elif event.key in [pygame.K_c, pygame.K_m]:
                        self.on_parameter_changed()
                    elif event.key == pygame.K_f:
                        self.fov_limited = not self.fov_limited
                        self.send_cmd(f"FOV {1 if self.fov_limited else 0}")
                        self.on_parameter_changed()
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

                    # VOLBA STARTOVNÍ STĚNY: [2 / ě] pro 2.4 m, [3 / š] pro 3.5 m, [P] pro přepínání
                    elif event.key in [pygame.K_2, pygame.K_KP2] or getattr(event, 'unicode', '') in ['2', 'ě', 'Ě']:
                        self.field_map.set_start_wall(2)
                        print("[POLE] Aktivována startovní stěna: 2.4 m (Kratší stěna) [Klávesa 2 / ě]")
                    elif event.key in [pygame.K_3, pygame.K_KP3] or getattr(event, 'unicode', '') in ['3', 'š', 'Š']:
                        self.field_map.set_start_wall(3)
                        print("[POLE] Aktivována startovní stěna: 3.5 m (Delší stěna) [Klávesa 3 / š]")
                    elif event.key == pygame.K_p:
                        new_mode = self.field_map.toggle_start_wall()
                        w_str = "2.4 m (Kratší stěna)" if new_mode == 2 else "3.5 m (Delší stěna)"
                        print(f"[POLE] Startovní stěna přepnuta: {w_str} [Klávesa P]")

                    elif event.key == pygame.K_l:
                        # ZAMKNUTÍ NEBO ODEMKNUTÍ REFERENČNÍHO ETALONU
                        if not self.localizer.locked:
                            cc = self.ref_model.get_confirmed_candles()
                            cw = self.ref_model.get_confirmed_walls()
                            ok = self.localizer.lock_map(cc, cw)
                            n_lm = len(cc) + len(cw)
                            if ok:
                                self.curr_model.reset()
                                # Globální mapa začíná s potvrzeným etalonem
                                self.field_map.reset()
                                self.field_map.ingest_candles(cc)
                                self.field_map.ingest_walls(cw)
                                print(f"[LOC] Stará mapa zamčena jako referenční etalon ({n_lm} landmarků).")
                                print(f"[LOC] Zahajuji sběr Nové stabilní mapy a kumulativní mapování hřiště...")
                            else:
                                print(f"[LOC] Nedostatek landmarků ({n_lm}) – nechte stabilizovat starou mapu na startu.")
                        else:
                            self.localizer.unlock_map()
                            self.ref_model.reset()
                            self.curr_model.reset()
                            self.field_map.reset()
                            print("[LOC] Mapa odemčena. Můžete znovu nasbírat původní etalon na startu.")

                    elif event.key == pygame.K_n:
                        if self.localizer.locked:
                            self.curr_model.reset()
                            self.localizer.last_match_info = None
                            print("[LOC] Nová stabilní mapa resetována. Sbírám čistou novou mapu na aktuální pozici...")

            self.read_serial()

            self.screen.fill(BG_COLOR)
            self.draw_left_live()
            self.draw_mid_original()
            self.draw_right_comparison()

            # Vertikální předěly mezi 3 horními panely
            pygame.draw.line(self.screen, (40, 48, 60), (PANEL_W, 0), (PANEL_W, PANEL_H), 2)
            pygame.draw.line(self.screen, (40, 48, 60), (PANEL_W * 2, 0), (PANEL_W * 2, PANEL_H), 2)

            # Horizontální předěl nad celkovou mapou hřiště
            pygame.draw.line(self.screen, (60, 75, 95), (0, PANEL_H), (WIN_W, PANEL_H), 3)

            # 4. PANEL: Celková mapa hřiště (kumulativní model se svíčkami a stěnami)
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
