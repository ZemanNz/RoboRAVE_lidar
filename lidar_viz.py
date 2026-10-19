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

import sys, os, time, json, math, itertools, collections
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


def parse_target_list(raw_targets):
    """Převede cíle s detekovanými stěnami a nájezdy (počítané 100% na ESP32) na strukturované slovníky."""
    targets = []
    for t in raw_targets:
        box_arr = t.get("box", [])
        box_dict = {}
        if len(box_arr) >= 8:
            box_dict = {
                'min_x': float(box_arr[0]), 'max_x': float(box_arr[1]),
                'min_y': float(box_arr[2]), 'max_y': float(box_arr[3]),
                'cx': float(box_arr[4]), 'cy': float(box_arr[5]),
                'width': float(box_arr[6]), 'height': float(box_arr[7]),
                'span_mm': math.hypot(float(box_arr[6]), float(box_arr[7])),
                'dist_mm': math.hypot(float(box_arr[4]), float(box_arr[5])),
                'type': t.get("type", "VOLNÁ")
            }

        c = t.get("candle", [0, 0])
        wp = t.get("waypoint", [0, 0])
        app = t.get("approach", [0, 0])
        walls_raw = t.get("walls", [])
        walls = []
        for w in walls_raw:
            if len(w) == 2:
                walls.append(((float(w[0][0]), float(w[0][1])), (float(w[1][0]), float(w[1][1]))))

        t_type = t.get("type", "VOLNÁ")
        targets.append({
            'id': int(t.get("id", len(targets) + 1)),
            'type': t_type,
            'is_corner': (t_type == "ROH"),
            'is_solo': (t_type == "VOLNÁ"),
            'candle': (float(c[0]), float(c[1])),
            'box': box_dict,
            'waypoint': (float(wp[0]), float(wp[1])),
            'approach': (float(app[0]), float(app[1])),
            'travel_mm': float(t.get("travel_mm", 0.0)),
            'walls': walls
        })
    return targets


def transform_esp_target_to_world(target, rx, ry, th_rad):
    """Transformuje tvrdá data cíle z ESP32 do souřadnic hřiště podle aktuální polohy robota."""
    cos_t = math.cos(th_rad)
    sin_t = math.sin(th_rad)
    def to_w(lx, ly):
        wx = rx + lx * cos_t + ly * sin_t
        wy = ry - lx * sin_t + ly * cos_t
        return wx, wy

    # Svíčka
    cw = to_w(target['candle'][0], target['candle'][1])
    # Nájezdový waypoint (38 cm před svíčkou)
    wpw = to_w(target['waypoint'][0], target['waypoint'][1])
    # Směr nájezdu ve světě
    adx = target['approach'][0] * cos_t + target['approach'][1] * sin_t
    ady = -target['approach'][0] * sin_t + target['approach'][1] * cos_t

    # Stěny ve světě
    walls_w = []
    for w1, w2 in target['walls']:
        ww1 = to_w(w1[0], w1[1])
        ww2 = to_w(w2[0], w2[1])
        walls_w.append((ww1, ww2))

    # Box ve světě (střed)
    b = dict(target['box'])
    if b:
        b_cx_w, b_cy_w = to_w(b['cx'], b['cy'])
        b['world_cx'] = b_cx_w
        b['world_cy'] = b_cy_w

    travel_dist = math.hypot(wpw[0] - rx, wpw[1] - ry)

    return {
        'id': target['id'],
        'type': target['type'],
        'is_corner': target['is_corner'],
        'is_solo': target['is_solo'],
        'box': b,
        'candle_world': cw,
        'waypoint_world': wpw,
        'approach_dir': (adx, ady),
        'travel_dist_mm': travel_dist,
        'walls_world': walls_w,
        'from_esp32': True
    }



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
        """Uloží aktuální potvrzené Bounding Boxy jako referenční etalon světa a sestaví všech až 6 kotev."""
        self.ref_boxes = [dict(b) for b in boxes]
        self.ref_dists = []
        self.last_match_info = None

        n = len(self.ref_boxes)
        if n < 2:
            self.locked = False
            return False

        # Invariantní párové vzdálenosti a kotvy pro všech n*(n-1)/2 dvojic (pro 4 svíčky = přesně 6 kotev)
        raw_anchors = []
        for i in range(n):
            for j in range(i + 1, n):
                b1 = self.ref_boxes[i]
                b2 = self.ref_boxes[j]
                dx = b2['cx'] - b1['cx']
                dy = b2['cy'] - b1['cy']
                dist = math.hypot(dx, dy)
                raw_anchors.append({
                    'idx1': i, 'idx2': j,
                    'p1': (b1['cx'], b1['cy']),
                    'p2': (b2['cx'], b2['cy']),
                    'dist': dist
                })

        # Seřadíme kotvy od nejdelší po nejkratší rozvor
        raw_anchors.sort(key=lambda a: a['dist'], reverse=True)

        ANCHOR_PALETTE = [
            {"name": "KOTVA 1", "color": (255, 215, 0)},    # Zlatá (nejdelší)
            {"name": "KOTVA 2", "color": (255, 140, 30)},   # Oranžová
            {"name": "KOTVA 3", "color": (0, 230, 200)},    # Tyrkysová
            {"name": "KOTVA 4", "color": (200, 110, 255)},  # Fialová
            {"name": "KOTVA 5", "color": (60, 180, 255)},   # Modrá
            {"name": "KOTVA 6", "color": (160, 240, 60)},   # Limetková
        ]

        self.anchors = []
        for rank, a in enumerate(raw_anchors):
            a['rank'] = rank + 1
            pal = ANCHOR_PALETTE[rank % len(ANCHOR_PALETTE)]
            a['name'] = f"{pal['name']} ({int(a['dist'])} mm)"
            a['color'] = pal['color']
            self.anchors.append(a)

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
        s referenčním etalonem ze startu pomocí multi-anchor grafu a vzájemných vnitřních úhlů.
        """
        if not self.locked or len(self.ref_boxes) < 2 or len(current_boxes) < 2:
            return False

        cands = [(float(b['cx']), float(b['cy'])) for b in current_boxes]
        n_cands = len(cands)
        n_refs = len(self.ref_boxes)

        best_match_info = None
        best_score = -999999.0

        # Zkoumáme shody o velikosti K od min(4, n_refs, n_cands) dolů k 2
        max_k = min(4, n_refs, n_cands)

        for k in range(max_k, 1, -1):
            # Pro každou podmnožinu k referenčních bodů
            for ref_comb in itertools.combinations(range(n_refs), k):
                # Pro každou uspořádanou k-tici kandidátů
                for cand_perm in itertools.permutations(range(n_cands), k):
                    # 1. KONTROLA DÉLEK VŠECH KOTEVA (párové vzdálenosti)
                    edges_ok = True
                    sum_dist_err = 0.0
                    for i in range(k):
                        for j in range(i + 1, k):
                            ri = ref_comb[i]; rj = ref_comb[j]
                            ci = cand_perm[i]; cj = cand_perm[j]
                            d_ref = math.hypot(self.ref_boxes[ri]['cx'] - self.ref_boxes[rj]['cx'],
                                               self.ref_boxes[ri]['cy'] - self.ref_boxes[rj]['cy'])
                            d_cur = math.hypot(cands[ci][0] - cands[cj][0],
                                               cands[ci][1] - cands[cj][1])
                            err = abs(d_cur - d_ref)
                            if err > 130.0:
                                edges_ok = False
                                break
                            sum_dist_err += err
                        if not edges_ok:
                            break
                    if not edges_ok:
                        continue

                    # 2. KONTROLA VNITŘNÍCH ORIENTOVANÝCH ÚHLŮ (pro k >= 3)
                    # Zaručuje, že konstelace není zrcadlená a žádná kotva není otočená o 180°!
                    angles_ok = True
                    if k >= 3:
                        for p in range(k):
                            rp = ref_comb[p]; cp = cand_perm[p]
                            for a in range(k):
                                if a == p: continue
                                ra = ref_comb[a]; ca = cand_perm[a]
                                for b in range(a + 1, k):
                                    if b == p: continue
                                    rb = ref_comb[b]; cb = cand_perm[b]

                                    # Orientovaný úhel v etalonu
                                    ur_x = self.ref_boxes[ra]['cx'] - self.ref_boxes[rp]['cx']
                                    ur_y = self.ref_boxes[ra]['cy'] - self.ref_boxes[rp]['cy']
                                    vr_x = self.ref_boxes[rb]['cx'] - self.ref_boxes[rp]['cx']
                                    vr_y = self.ref_boxes[rb]['cy'] - self.ref_boxes[rp]['cy']
                                    ang_ref = math.atan2(ur_x * vr_y - ur_y * vr_x, ur_x * vr_x + ur_y * vr_y)

                                    # Orientovaný úhel v měření senzoru robota (+X doprava, +Y dopředu)
                                    uc_x = cands[ca][0] - cands[cp][0]
                                    uc_y = cands[ca][1] - cands[cp][1]
                                    vc_x = cands[cb][0] - cands[cp][0]
                                    vc_y = cands[cb][1] - cands[cp][1]
                                    ang_cur = math.atan2(uc_x * vc_y - uc_y * vc_x, uc_x * vc_x + uc_y * vc_y)

                                    # Rozdíl orientovaných úhlů
                                    ang_diff = abs(math.atan2(math.sin(ang_cur - ang_ref), math.cos(ang_cur - ang_ref)))
                                    if ang_diff > math.radians(14.0):
                                        angles_ok = False
                                        break
                                if not angles_ok:
                                    break
                            if not angles_ok:
                                break
                    if not angles_ok:
                        continue

                    # 3. VÝPOČET RIGIDNÍ TRANSFORMACE (Kabsch / Procrustes)
                    src = [cands[cand_perm[i]] for i in range(k)]
                    dst = [(self.ref_boxes[ref_comb[i]]['cx'], self.ref_boxes[ref_comb[i]]['cy']) for i in range(k)]
                    tx, ty, th = self._rigid_transform_2d(src, dst)

                    # 4. KONTROLA PRO k = 2 (výběr správné orientace, ochrana proti otočení o 180°)
                    penalty = 0.0
                    if k == 2:
                        # Pokud je pozice robota mimo mantinely hřiště, silně penalizovat
                        if not (-1400.0 <= tx <= 1400.0 and -400.0 <= ty <= 3800.0):
                            penalty += 4000.0

                        # Kontinuita orientace a polohy (ochrana proti 180° skokům, pokud už robot byl lokalizován)
                        if self.pose_confidence > 0.2:
                            deg_cur = math.degrees(th)
                            deg_diff = abs((deg_cur - self.robot_deg + 180.0) % 360.0 - 180.0)
                            dist_jump = math.hypot(tx - self.robot_x, ty - self.robot_y)
                            if deg_diff > 90.0:
                                penalty += 3000.0 + deg_diff * 20.0
                            if dist_jump > 600.0:
                                penalty += dist_jump * 3.0

                    # 5. VYHODNOCENÍ VŠECH KANDIDÁTŮ S TOUTO TRANSFORMACÍ (Inliers)
                    cos_t = math.cos(th); sin_t = math.sin(th)
                    inliers = 0
                    err_sum = 0.0
                    inlier_pairs = []

                    for cp in cands:
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
                            inlier_pairs.append(((best_rb['cx'], best_rb['cy']), (wx, wy), min_d))

                    avg_err = (err_sum / inliers) if inliers > 0 else 9999.0
                    base_bonus = 6000.0 if k >= 4 else (3500.0 if k == 3 else 1000.0)
                    score = base_bonus + inliers * 500.0 - avg_err * 2.0 - penalty

                    if score > best_score and inliers >= 2:
                        best_score = score
                        # Sestavení spárovaných kotev ze všech referenčních kotev
                        # Mapování z referenčního indexu na index kandidáta
                        ref_to_cand = {ref_comb[i]: cand_perm[i] for i in range(k)}
                        matched_anchors = []
                        for anc in self.anchors:
                            i1 = anc['idx1']; i2 = anc['idx2']
                            if i1 in ref_to_cand and i2 in ref_to_cand:
                                c1 = ref_to_cand[i1]; c2 = ref_to_cand[i2]
                                p1_cur = cands[c1]; p2_cur = cands[c2]
                                w1x = tx + p1_cur[0] * cos_t + p1_cur[1] * sin_t
                                w1y = ty - p1_cur[0] * sin_t + p1_cur[1] * cos_t
                                w2x = tx + p2_cur[0] * cos_t + p2_cur[1] * sin_t
                                w2y = ty - p2_cur[0] * sin_t + p2_cur[1] * cos_t
                                cur_d = math.hypot(p2_cur[0] - p1_cur[0], p2_cur[1] - p1_cur[1])
                                matched_anchors.append({
                                    'anchor': anc,
                                    'p1_world': (w1x, w1y),
                                    'p2_world': (w2x, w2y),
                                    'cur_dist': cur_d,
                                    'ref_dist': anc['dist']
                                })

                        pat_name = f"PLNÁ SHODA ({k} CÍLE, {len(matched_anchors)} KOTEVA)" if k >= 4 else \
                                  (f"TROJÚHELNÍK (3 CÍLE, {len(matched_anchors)} KOTVY)" if k == 3 else \
                                   f"DVĚ SVÍČKY ({matched_anchors[0]['anchor']['name'] if matched_anchors else 'KOTVA'})")

                        best_match_info = {
                            'inliers': inliers,
                            'k_matched': k,
                            'avg_error': avg_err,
                            'tx': tx, 'ty': ty, 'theta': th,
                            'inlier_pairs': inlier_pairs,
                            'matched_anchors': matched_anchors,
                            'pattern_name': pat_name,
                            'score': score
                        }

            # Pokud jsme našli platnou shodu s k >= 3, preferujeme ji
            if best_match_info is not None and best_match_info['k_matched'] >= 3:
                break

        # Aplikace výsledku lokalizace
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


def draw_arrow(screen, color, p_start, p_end, width=3, tip_len=14, tip_width=8):
    """Vykreslí orientovanou nájezdovou šipku s trojúhelníkovým hrotem."""
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    dist = math.hypot(dx, dy)
    if dist < 6.0:
        return
    pygame.draw.line(screen, color, p_start, p_end, width)
    ux = dx / dist
    uy = dy / dist
    nx = -uy
    ny = ux
    tip_base_x = p_end[0] - ux * tip_len
    tip_base_y = p_end[1] - uy * tip_len
    p1 = (int(tip_base_x + nx * tip_width), int(tip_base_y + ny * tip_width))
    p2 = (int(tip_base_x - nx * tip_width), int(tip_base_y - ny * tip_width))
    pygame.draw.polygon(screen, color, [p_end, p1, p2])


# ============================================================
#  BoxDetailStabilizer - Dlouhodobá stabilizace stěn a svíček v čase
#  Zabraňuje přeblikávání, stabilizuje orientace stěn a nájezdové vektory.
# ============================================================
class TrackedBoxDetail:
    def __init__(self, track_id, init_meas):
        self.track_id = track_id
        self.type_history = collections.deque(maxlen=14)
        self.type_history.append(init_meas['type'])
        self.confirmed_type = init_meas['type']
        self.candle_world = init_meas['candle_world']
        self.walls_world = list(init_meas['walls_world'])
        self.approach_dir = init_meas['approach_dir']
        self.waypoint_world = init_meas['waypoint_world']
        self.hits = 1
        self.missed = 0
        self.travel_dist_mm = 0.0

    def update(self, meas, rx, ry):
        self.hits += 1
        self.missed = 0
        self.type_history.append(meas['type'])

        # Stabilní typ podle historie (většinový konsensus):
        if self.hits <= 2:
            self.confirmed_type = meas['type']
        else:
            c_corner = self.type_history.count('CORNER')
            c_wall   = self.type_history.count('WALL')
            if c_corner >= 3:
                self.confirmed_type = 'CORNER'
            elif c_wall >= 4:
                self.confirmed_type = 'WALL'
            else:
                self.confirmed_type = 'SOLO'

        alpha = 0.22
        # Vyhlazení polohy svíčky (EMA)
        mc = meas['candle_world']
        self.candle_world = (
            (1.0 - alpha) * self.candle_world[0] + alpha * mc[0],
            (1.0 - alpha) * self.candle_world[1] + alpha * mc[1]
        )

        # Vyhlazení stěn (pokud v tomto snímku byly detekovány, vyhladíme; pokud ne, zachováme předchozí!)
        mw = meas['walls_world']
        if mw:
            if len(mw) == len(self.walls_world):
                new_w = []
                for (s1, e1), (s2, e2) in zip(self.walls_world, mw):
                    ns = ((1.0 - alpha) * s1[0] + alpha * s2[0], (1.0 - alpha) * s1[1] + alpha * s2[1])
                    ne = ((1.0 - alpha) * e1[0] + alpha * e2[0], (1.0 - alpha) * e1[1] + alpha * e2[1])
                    new_w.append((ns, ne))
                self.walls_world = new_w
            else:
                self.walls_world = list(mw)

        # Vyhlazení směru nájezdu a cílového waypointu
        mdir = meas['approach_dir']
        adx = (1.0 - alpha) * self.approach_dir[0] + alpha * mdir[0]
        ady = (1.0 - alpha) * self.approach_dir[1] + alpha * mdir[1]
        alen = math.hypot(adx, ady)
        if alen > 1e-4:
            self.approach_dir = (adx / alen, ady / alen)

        mwp = meas['waypoint_world']
        self.waypoint_world = (
            (1.0 - alpha) * self.waypoint_world[0] + alpha * mwp[0],
            (1.0 - alpha) * self.waypoint_world[1] + alpha * mwp[1]
        )
        self.travel_dist_mm = math.hypot(self.waypoint_world[0] - rx, self.waypoint_world[1] - ry)


class BoxDetailStabilizer:
    def __init__(self):
        self.tracks = {}
        self.next_id = 1

    def reset(self):
        self.tracks.clear()
        self.next_id = 1

    def update_all(self, raw_details, rx, ry):
        matched_ids = set()
        out_list = []

        for rd in raw_details:
            cw = rd['candle_world']
            best_id = None
            best_d = 280.0

            for tid, tr in self.tracks.items():
                if tid in matched_ids: continue
                d = math.hypot(tr.candle_world[0] - cw[0], tr.candle_world[1] - cw[1])
                if d < best_d:
                    best_d = d
                    best_id = tid

            if best_id is not None:
                tr = self.tracks[best_id]
                tr.update(rd, rx, ry)
                matched_ids.add(best_id)
            else:
                tid = self.next_id
                self.next_id += 1
                tr = TrackedBoxDetail(tid, rd)
                tr.travel_dist_mm = math.hypot(tr.waypoint_world[0] - rx, tr.waypoint_world[1] - ry)
                self.tracks[tid] = tr
                matched_ids.add(tid)

            is_cor = (tr.confirmed_type == 'CORNER')
            is_sol = (tr.confirmed_type == 'SOLO')
            res = {
                'box': rd.get('box', {}),
                'type': tr.confirmed_type,
                'is_corner': is_cor,
                'is_solo': is_sol,
                'candle_world': tr.candle_world,
                'walls_world': tr.walls_world,
                'waypoint_world': tr.waypoint_world,
                'arrow_start': tr.waypoint_world,
                'arrow_end': tr.candle_world,
                'travel_dist_mm': tr.travel_dist_mm,
                'approach_dir': tr.approach_dir,
                'hits': tr.hits
            }
            out_list.append(res)

        # Čištění ztracených cílů (až po 20 snímcích absence)
        to_del = []
        for tid, tr in self.tracks.items():
            if tid not in matched_ids:
                tr.missed += 1
                if tr.missed > 20:
                    to_del.append(tid)
        for tid in to_del:
            del self.tracks[tid]

        return out_list


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
            "boxes": [], "static_boxes": [], "targets": [], "route": [], "cloud": []
        }

        self.localizer = RobotLocalizer()
        self.field_map = GlobalFieldMap()
        self.processed_boxes = []
        self.esp_targets = []  # Tvrdá data detailních cílů přímo z ESP32 (svíčky, stěny, 38cm nájezdy)
        self.esp_route = []    # Tvrdé optimální pořadí hašení přímo z ESP32
        self.detect_details_enabled = True  # Klávesa [O] zapíná/vypíná zobrazení stěn a nájezdových cest
        self.current_box_details = []
        self.detail_stabilizer = BoxDetailStabilizer()

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
        self.processed_boxes = []
        self.esp_targets = []
        self.esp_route = []
        self.current_box_details = []
        self.detail_stabilizer.reset()
        self.send_cmd("RESET_MAP")
        print("[MAPA] Mapy vyresetovány -> sběr čistého etalonu.")

    def to_screen(self, x_mm, y_mm, panel_offset_x):
        cx = panel_offset_x + PANEL_W / 2.0
        cy = PANEL_H / 2.0
        scale = (PANEL_W / 2.0) / self.view_span_mm
        sx = int(cx + x_mm * scale)
        sy = int(cy - y_mm * scale)
        return sx, sy

    def prioritize_and_recover_expected_boxes(self, raw_boxes, cloud_data):
        """
        Zvýhodnění reálných svíček a odlehčení náročnosti detekce v očekávaných místech:
        1. Vyloučí/odsunue boxy, které leží mimo mantinely hřiště (nohy diváků, zdi místnosti).
        2. Pro každou referenční svíčku ze STARTU vypočítá očekávanou polohu v lokálním souřadném systému robota.
        3. Pokud je v očekávané zóně (< 250 mm) přítomen detekovaný box, potvrdí ho s nejvyšší prioritou.
        4. Pokud svíčka chybí (problikává, je dál nebo má méně bodů), sníží náročnost:
           prohledá mračno bodů v predikovaném místě a i při pouhých 2-3 bodech vytvoří zachráněný Bounding Box.
        """
        loc = self.localizer
        if not loc.locked or len(loc.ref_boxes) == 0:
            return list(raw_boxes)

        rx = loc.robot_x
        ry = loc.robot_y
        r_deg = loc.robot_deg
        th = math.radians(r_deg)
        cos_t = math.cos(th)
        sin_t = math.sin(th)

        # Hranice hřiště pro detekci objektů za mantinelem
        xmin, xmax, ymin, ymax = self.field_map.get_auto_field_bounds()
        out_margin = 150.0  # mm tolerance za mantinelem

        # Rozdělení na boxy uvnitř a vně hřiště
        inside_boxes = []
        outside_boxes = []

        for b in raw_boxes:
            b_copy = dict(b)
            # Světové souřadnice středu boxu
            wx = rx + b['cx'] * cos_t + b['cy'] * sin_t
            wy = ry - b['cx'] * sin_t + b['cy'] * cos_t
            b_copy['world_x'] = wx
            b_copy['world_y'] = wy

            is_outside = (wx < xmin - out_margin or wx > xmax + out_margin or
                          wy < ymin - out_margin or wy > ymax + out_margin)
            b_copy['is_outside'] = is_outside
            if is_outside:
                outside_boxes.append(b_copy)
            else:
                inside_boxes.append(b_copy)

        # 2. Predikce lokální polohy každé referenční svíčky v senzoru robota
        matched_inside_indices = set()
        enhanced_boxes = []

        for ref_idx, rb in enumerate(loc.ref_boxes):
            # Globální poloha referenční svíčky
            r_wx = rb['cx']
            r_wy = rb['cy']

            # Relativní vektor vůči robotovi
            del_x = r_wx - rx
            del_y = r_wy - ry

            # Inverzní rotace do lokálního systému robota (+X doprava, +Y dopředu)
            pred_lx = del_x * cos_t - del_y * sin_t
            pred_ly = del_x * sin_t + del_y * cos_t
            pred_dist = math.hypot(pred_lx, pred_ly)

            # Hledáme, zda v okruhu 240 mm od predikované polohy máme box uvnitř hřiště
            best_box_idx = -1
            best_dist = 240.0

            for idx, b in enumerate(inside_boxes):
                if idx in matched_inside_indices:
                    continue
                d = math.hypot(b['cx'] - pred_lx, b['cy'] - pred_ly)
                if d < best_dist:
                    best_dist = d
                    best_box_idx = idx

            if best_box_idx >= 0:
                # Nalezena očekávaná svíčka -> potvrdíme a zvýhodníme
                b = inside_boxes[best_box_idx]
                b['is_expected'] = True
                b['expected_ref_idx'] = ref_idx + 1
                b['pred_dist'] = pred_dist
                enhanced_boxes.append(b)
                matched_inside_indices.add(best_box_idx)
            else:
                # Svíčka v detekovaných boxech chybí (problikává nebo má málo bodů).
                # Odlehčení náročnosti: prohledáme surové mračno bodů (cloud) v predikovaném místě!
                if cloud_data and len(cloud_data) >= 3:
                    pts_found = []
                    n_cloud = len(cloud_data) // 3
                    search_radius = 180.0
                    search_radius_sq = search_radius * search_radius

                    for i in range(n_cloud):
                        px = cloud_data[i * 3]
                        py = cloud_data[i * 3 + 1]
                        dx = px - pred_lx
                        dy = py - pred_ly
                        if dx * dx + dy * dy <= search_radius_sq:
                            pts_found.append((px, py))

                    # Stačí pouhé 2 body (odlehčený práh pro očekávanou svíčku ze startu)
                    if len(pts_found) >= 2:
                        sum_x = sum(p[0] for p in pts_found)
                        sum_y = sum(p[1] for p in pts_found)
                        k = len(pts_found)
                        c_x = sum_x / k
                        c_y = sum_y / k
                        min_x = min(p[0] for p in pts_found)
                        max_x = max(p[0] for p in pts_found)
                        min_y = min(p[1] for p in pts_found)
                        max_y = max(p[1] for p in pts_found)
                        w = max(35.0, max_x - min_x)
                        h = max(35.0, max_y - min_y)
                        span = math.hypot(w, h)
                        dist = math.hypot(c_x, c_y)

                        recovered_b = {
                            'min_x': min_x, 'max_x': max_x,
                            'min_y': min_y, 'max_y': max_y,
                            'cx': c_x, 'cy': c_y,
                            'width': w, 'height': h,
                            'span_mm': span,
                            'dist_mm': dist,
                            'pts_count': k,
                            'is_solo': (span < 160.0),
                            'type': 'SVÍČKA' if span < 160.0 else 'SVÍČKA+STĚNA',
                            'is_expected': True,
                            'is_recovered': True,
                            'expected_ref_idx': ref_idx + 1,
                            'world_x': r_wx,
                            'world_y': r_wy,
                            'is_outside': False
                        }
                        enhanced_boxes.append(recovered_b)

        # Přidáme zbylé nezařazené boxy uvnitř hřiště (které nepatří k žádné očekávané svíčce)
        for idx, b in enumerate(inside_boxes):
            if idx not in matched_inside_indices:
                b['is_expected'] = False
                enhanced_boxes.append(b)

        # Seřazení:
        # 1. Očekávané svíčky v hřišti (mají absolutní přednost)
        # 2. Ostatní boxy uvnitř hřiště seřazené podle vzdálenosti
        enhanced_boxes.sort(key=lambda x: (not x.get('is_expected', False), x.get('dist_mm', 9999.0)))

        # Výsledný výběr: max 4 nejlepší boxy
        final_boxes = enhanced_boxes[:4]

        # Pokud by zbývalo místo (< 4) a neměli bychom nic jiného, zařadíme i box z outside (pro diagnostiku)
        if len(final_boxes) < 4:
            for ob in outside_boxes:
                if len(final_boxes) >= 4:
                    break
                final_boxes.append(ob)

        return final_boxes

    def compute_raw_box_details(self, box, cloud_data, rx, ry, th_rad):
        """
        Geometrická analýza stěn a svíčky STRIKTNĚ UVNITŘ BOUNDING BOXU (ve světových souřadnicích):
        1. Vyfiltruje body z cloud_data uvnitř hranic boxu a převede do světa.
        2. Proloží stěnu 1 (RANSAC).
        3. Pokud existují kolmé body pro stěnu 2 -> ROH (určí vrchol V, obě ramena a osu otevřeného prostoru).
        4. Pokud pouze 1 stěna -> určí kolmici od stěny přes svíčku dál do volného prostoru.
        5. Šipky nikdy nekříží stěny: u rohu vedou středem otevřeného úhlu, u 1 stěny po kolmici z otevřené strany.
        """
        cos_t = math.cos(th_rad)
        sin_t = math.sin(th_rad)

        margin = 35.0
        bx_min = box['min_x'] - margin
        bx_max = box['max_x'] + margin
        by_min = box['min_y'] - margin
        by_max = box['max_y'] + margin

        # Výběr bodů striktně uvnitř boxu převedených přímo do světových souřadnic
        world_pts = []
        if cloud_data and len(cloud_data) >= 3:
            n_cloud = len(cloud_data) // 3
            for i in range(n_cloud):
                lx = cloud_data[i * 3]
                ly = cloud_data[i * 3 + 1]
                if bx_min <= lx <= bx_max and by_min <= ly <= by_max:
                    wx = rx + lx * cos_t + ly * sin_t
                    wy = ry - lx * sin_t + ly * cos_t
                    world_pts.append((wx, wy))

        w_box_cx = rx + box['cx'] * cos_t + box['cy'] * sin_t
        w_box_cy = ry - box['cx'] * sin_t + box['cy'] * cos_t

        # 1. Malý box nebo málo bodů -> SAMOSTATNÁ VOLNÁ SVÍČKA
        if len(world_pts) < 5 or box.get('span_mm', 0) < 160.0 or box.get('is_solo', False):
            cand_x = sum(p[0] for p in world_pts) / len(world_pts) if world_pts else w_box_cx
            cand_y = sum(p[1] for p in world_pts) / len(world_pts) if world_pts else w_box_cy

            d_vec_x = cand_x - rx
            d_vec_y = cand_y - ry
            d_len = math.hypot(d_vec_x, d_vec_y)
            if d_len > 10.0:
                ux = d_vec_x / d_len; uy = d_vec_y / d_len
            else:
                ux, uy = 0.0, 1.0

            wp_x = cand_x - ux * 380.0
            wp_y = cand_y - uy * 380.0

            return {
                'box': box,
                'type': 'SOLO',
                'candle_world': (cand_x, cand_y),
                'walls_world': [],
                'waypoint_world': (wp_x, wp_y),
                'approach_dir': (ux, uy)
            }

        # 2. Hledáme první stěnu (Wall 1) pomocí RANSAC ve světovém prostoru
        best_w1_inliers = []
        best_w1_line = None
        n_p = len(world_pts)
        step = 1 if n_p < 40 else 2
        for i in range(0, min(n_p, 30), step):
            p1 = world_pts[i]
            for j in range(i + 1, min(n_p, 30), step):
                p2 = world_pts[j]
                d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                if d < 75.0: continue
                nx = -(p2[1] - p1[1]) / d
                ny = (p2[0] - p1[0]) / d
                inliers = [p for p in world_pts if abs((p[0] - p1[0]) * nx + (p[1] - p1[1]) * ny) <= 22.0]
                if len(inliers) > len(best_w1_inliers):
                    best_w1_inliers = inliers
                    best_w1_line = (p1, p2)

        if not best_w1_line or len(best_w1_inliers) < 4:
            cand_x = sum(p[0] for p in world_pts) / len(world_pts)
            cand_y = sum(p[1] for p in world_pts) / len(world_pts)
            d_vec_x = cand_x - rx; d_vec_y = cand_y - ry
            d_len = max(1.0, math.hypot(d_vec_x, d_vec_y))
            ux = d_vec_x / d_len; uy = d_vec_y / d_len
            return {
                'box': box,
                'type': 'SOLO',
                'candle_world': (cand_x, cand_y),
                'walls_world': [],
                'waypoint_world': (cand_x - ux * 380.0, cand_y - uy * 380.0),
                'approach_dir': (ux, uy)
            }

        # Geometrie stěny 1
        p1, p2 = best_w1_line
        dx = p2[0] - p1[0]; dy = p2[1] - p1[1]
        L1 = math.hypot(dx, dy)
        u1x = dx / L1; u1y = dy / L1

        projs1 = [((p[0] - p1[0]) * u1x + (p[1] - p1[1]) * u1y) for p in best_w1_inliers]
        min_t1 = min(projs1); max_t1 = max(projs1)
        w1_s = (p1[0] + u1x * min_t1, p1[1] + u1y * min_t1)
        w1_e = (p1[0] + u1x * max_t1, p1[1] + u1y * max_t1)
        w1_len = max_t1 - min_t1

        outliers1 = [p for p in world_pts if p not in best_w1_inliers]

        # 3. Hledáme druhou stěnu (Wall 2) kolmou k Wall 1
        best_w2_inliers = []
        best_w2_line = None
        if len(outliers1) >= 4:
            n_out = len(outliers1)
            for i in range(min(n_out, 25)):
                q1 = outliers1[i]
                for j in range(i + 1, min(n_out, 25)):
                    q2 = outliers1[j]
                    qd = math.hypot(q2[0] - q1[0], q2[1] - q1[1])
                    if qd < 65.0: continue
                    qux = (q2[0] - q1[0]) / qd
                    quy = (q2[1] - q1[1]) / qd
                    dot = abs(u1x * qux + u1y * quy)
                    if dot > 0.65: continue # požadavek kolmosti (50° až 130°)
                    q_nx = -quy; q_ny = qux
                    in2 = [p for p in outliers1 if abs((p[0] - q1[0]) * q_nx + (p[1] - q1[1]) * q_ny) <= 22.0]
                    if len(in2) > len(best_w2_inliers):
                        best_w2_inliers = in2
                        best_w2_line = (q1, q2)

        # ========================================================
        # VĚTEV A: ROH (2 STĚNY KOLEM SVÍČKY)
        # ========================================================
        if best_w2_line and len(best_w2_inliers) >= 3:
            q1, q2 = best_w2_line
            qdx = q2[0] - q1[0]; qdy = q2[1] - q1[1]
            L2 = math.hypot(qdx, qdy)
            u2x = qdx / L2; u2y = qdy / L2

            # Průsečík obou ramen stěn = Vrchol rohu V
            cross = u1x * u2y - u1y * u2x
            if abs(cross) > 1e-4:
                s_param = ((q1[0] - p1[0]) * u2y - (q1[1] - p1[1]) * u2x) / cross
                Vx = p1[0] + s_param * u1x
                Vy = p1[1] + s_param * u1y
            else:
                Vx = (w1_s[0] + w1_e[0]) * 0.5
                Vy = (w1_s[1] + w1_e[1]) * 0.5
            V = (Vx, Vy)

            # Orientujeme rameno 1 směrem OD vrcholu V podél stěny
            d_s1 = math.hypot(w1_s[0] - Vx, w1_s[1] - Vy)
            d_e1 = math.hypot(w1_e[0] - Vx, w1_e[1] - Vy)
            far1 = w1_e if d_e1 > d_s1 else w1_s
            len1 = max(140.0, max(d_s1, d_e1))
            norm_arm1 = max(1.0, math.hypot(far1[0] - Vx, far1[1] - Vy))
            arm1_x = (far1[0] - Vx) / norm_arm1
            arm1_y = (far1[1] - Vy) / norm_arm1
            wall1_seg = (V, (Vx + arm1_x * len1, Vy + arm1_y * len1))

            # Orientujeme rameno 2 směrem OD vrcholu V podél stěny
            projs2 = [((p[0] - q1[0]) * u2x + (p[1] - q1[1]) * u2y) for p in best_w2_inliers]
            w2_s = (q1[0] + u2x * min(projs2), q1[1] + u2y * min(projs2))
            w2_e = (q1[0] + u2x * max(projs2), q1[1] + u2y * max(projs2))
            d_s2 = math.hypot(w2_s[0] - Vx, w2_s[1] - Vy)
            d_e2 = math.hypot(w2_e[0] - Vx, w2_e[1] - Vy)
            far2 = w2_e if d_e2 > d_s2 else w2_s
            len2 = max(140.0, max(d_s2, d_e2))
            norm_arm2 = max(1.0, math.hypot(far2[0] - Vx, far2[1] - Vy))
            arm2_x = (far2[0] - Vx) / norm_arm2
            arm2_y = (far2[1] - Vy) / norm_arm2
            wall2_seg = (V, (Vx + arm2_x * len2, Vy + arm2_y * len2))

            # Svíčka C: zbylé body uvnitř rohu
            cand_pts = [p for p in outliers1 if p not in best_w2_inliers]
            if cand_pts:
                cand_x = sum(p[0] for p in cand_pts) / len(cand_pts)
                cand_y = sum(p[1] for p in cand_pts) / len(cand_pts)
            else:
                cand_x = Vx + 85.0 * arm1_x + 85.0 * arm2_x
                cand_y = Vy + 85.0 * arm1_y + 85.0 * arm2_y

            # Otevřený prostor: osa rohu (bisektor) mířící ven z rohu do volného hřiště
            bisect_x = arm1_x + arm2_x
            bisect_y = arm1_y + arm2_y
            blen = math.hypot(bisect_x, bisect_y)
            if blen > 0.01:
                u_open_x = bisect_x / blen
                u_open_y = bisect_y / blen
            else:
                u_open_x = 0.0; u_open_y = 1.0

            # Waypoint je 380 mm v otevřeném prostoru před svíčkou
            wp_x = cand_x + u_open_x * 380.0
            wp_y = cand_y + u_open_y * 380.0

            # Šipka míří Z otevřeného prostoru (od Waypointu) DO svíčky
            app_dir = (-u_open_x, -u_open_y)

            return {
                'box': box,
                'type': 'CORNER',
                'candle_world': (cand_x, cand_y),
                'walls_world': [wall1_seg, wall2_seg],
                'waypoint_world': (wp_x, wp_y),
                'approach_dir': app_dir
            }

        # ========================================================
        # VĚTEV B: 1 STĚNA (SVÍČKA CHRÁNĚNÁ JEDNOU DESKOU)
        # ========================================================
        wall1_seg = (w1_s, w1_e)

        # Svíčka C z outliers
        if outliers1:
            cand_x = sum(p[0] for p in outliers1) / len(outliers1)
            cand_y = sum(p[1] for p in outliers1) / len(outliers1)
        else:
            mid_x = (w1_s[0] + w1_e[0]) * 0.5
            mid_y = (w1_s[1] + w1_e[1]) * 0.5
            cand_x = mid_x
            cand_y = mid_y

        # Pata kolmice P_proj na přímce stěny 1
        t_proj = (cand_x - w1_s[0]) * u1x + (cand_y - w1_s[1]) * u1y
        t_clamped = max(0.0, min(w1_len, t_proj))
        p_proj_x = w1_s[0] + t_clamped * u1x
        p_proj_y = w1_s[1] + t_clamped * u1y

        # Kolmice: vektor ze stěny (pata kolmice) přes svíčku dál do otevřeného prostoru
        perp_dx = cand_x - p_proj_x
        perp_dy = cand_y - p_proj_y
        perp_dist = math.hypot(perp_dx, perp_dy)

        if perp_dist > 15.0:
            norm_x = perp_dx / perp_dist
            norm_y = perp_dy / perp_dist
        else:
            # Pokud svíčka leží přímo na přímce stěny, vezmeme normálu ke stěně mířící k robotovi
            nx = -u1y; ny = u1x
            mid_x = (w1_s[0] + w1_e[0]) * 0.5
            mid_y = (w1_s[1] + w1_e[1]) * 0.5
            if nx * (rx - mid_x) + ny * (ry - mid_y) < 0:
                nx = -nx; ny = -ny
            norm_x = nx; norm_y = ny
            cand_x = p_proj_x + norm_x * 80.0
            cand_y = p_proj_y + norm_y * 80.0

        # Waypoint leží na kolmici od stěny přes svíčku a od svíčky dál o 380 mm
        wp_x = cand_x + norm_x * 380.0
        wp_y = cand_y + norm_y * 380.0

        # Šipka míří od waypointu do svíčky (po kolmici ze strany otevřeného prostoru)
        app_dir = (-norm_x, -norm_y)

        return {
            'box': box,
            'type': 'WALL',
            'candle_world': (cand_x, cand_y),
            'walls_world': [wall1_seg],
            'waypoint_world': (wp_x, wp_y),
            'approach_dir': app_dir
        }

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
            cloud_data = newest.get("cloud", [])
            self.esp_targets = parse_target_list(newest.get("targets", []))
            self.esp_route = newest.get("route", [])

            # Zpracování a zvýhodnění očekávaných svíček v hřišti
            raw_active = static_boxes if len(static_boxes) >= 2 else live_boxes
            if self.localizer.locked:
                self.processed_boxes = self.prioritize_and_recover_expected_boxes(raw_active, cloud_data)
            else:
                self.processed_boxes = static_boxes if static_boxes else live_boxes

            active_b = self.processed_boxes

            # Pokud ještě není zamčeno [L], průběžně aktualizujeme globální model z pozice START=[0,0]
            if not self.localizer.locked:
                if active_b:
                    self.field_map.ingest_boxes(active_b, 0.0, 0.0, 0.0)
            else:
                # Po zamknutí provádíme lokalizaci robota podle aktuálních boxů
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

        # Bounding Boxy: živé z ESP32 nebo obohacené v očekávaných zónách po zamknutí
        boxes_to_draw = self.processed_boxes if (self.localizer.locked and self.processed_boxes) else parse_box_list(self.last_frame.get("boxes", []))
        for idx, cl in enumerate(boxes_to_draw):
            p_min = self.to_screen(cl['min_x'], cl['max_y'], offset_x)
            p_max = self.to_screen(cl['max_x'], cl['min_y'], offset_x)
            rx = min(p_min[0], p_max[0])
            ry = min(p_min[1], p_max[1])
            rw = max(8, abs(p_max[0] - p_min[0]))
            rh = max(8, abs(p_max[1] - p_min[1]))

            is_rec = cl.get('is_recovered', False)
            is_exp = cl.get('is_expected', False)
            is_out = cl.get('is_outside', False)

            if is_rec:
                box_col = (0, 230, 255)
            elif is_exp:
                box_col = (0, 255, 170) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 205, 40)
            elif is_out:
                box_col = (130, 130, 140)
            else:
                box_col = (0, 240, 160) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 180, 40)

            pygame.draw.rect(self.screen, box_col, (rx, ry, rw, rh), 2)

            scx, scy = self.to_screen(cl['cx'], cl['cy'], offset_x)
            pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 5)
            pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)

            tag = " [ZACHRÁNĚNÁ]" if is_rec else (f" [★ ETALON #{cl.get('expected_ref_idx', '')}]" if is_exp else (" [VNĚ POLE]" if is_out else ""))
            lbl1 = self.font_bold.render(f"BOX #{idx+1} [{cl['type']}]{tag}: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
            lbl2 = self.font_main.render(f"STŘED: [{int(cl['cx'])}, {int(cl['cy'])}] | R:{int(cl['dist_mm'])}mm ({cl['pts_count']}b)", True, (255, 240, 130))
            self.screen.blit(lbl1, (rx + 4, max(4, ry - 18)))
            self.screen.blit(lbl2, (scx + 8, scy - 7))

        # Jemné stěny a nájezdy v Panelu 1 při stisknutí [O]
        if self.detect_details_enabled and boxes_to_draw:
            cloud_data = self.last_frame.get("cloud", [])
            for b_idx, cl in enumerate(boxes_to_draw):
                raw_d = self.compute_raw_box_details(cl, cloud_data, 0.0, 0.0, 0.0)
                # Stěny uvnitř boxu
                for w_seg in raw_d.get('walls_world', []):
                    wp1 = self.to_screen(w_seg[0][0], w_seg[0][1], offset_x)
                    wp2 = self.to_screen(w_seg[1][0], w_seg[1][1], offset_x)
                    pygame.draw.line(self.screen, (255, 250, 130), wp1, wp2, 3)
                # Svíčka
                cw = raw_d.get('candle_world', (cl['cx'], cl['cy']))
                cs = self.to_screen(cw[0], cw[1], offset_x)
                pygame.draw.circle(self.screen, (255, 50, 30), cs, 6)
                pygame.draw.circle(self.screen, (255, 220, 0), cs, 3)
                # Šipka nájezdu
                wp_p = raw_d.get('waypoint_world', (cw[0], cw[1]))
                wp_s = self.to_screen(wp_p[0], wp_p[1], offset_x)
                draw_arrow(self.screen, (255, 75, 40), wp_s, cs, width=3, tip_len=12, tip_width=6)

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
        row("Živé boxy (max 4):", f"{len(boxes_to_draw)} cílů", (0, 240, 160))

        # Ovládací HUD vlevo dole
        cmd_panel = pygame.Surface((576, 110))
        cmd_panel.set_alpha(220)
        cmd_panel.fill((10, 12, 16))
        self.screen.blit(cmd_panel, (12, PANEL_H - 120))
        pygame.draw.rect(self.screen, (55, 45, 30), (12, PANEL_H - 120, 576, 110), 1)

        cy = PANEL_H - 114
        self.screen.blit(self.font_bold.render("LADĚNÍ ZA BĚHU (PŘÍKAZY DO ESP32):", True, (240, 200, 80)), (22, cy)); cy += 19
        fov_txt = "[F] FoV: 180° Kužel (Start)" if self.fov_limited else "[F] FoV: 360° Celé pole (V poli)"
        auto_badge = "[AUTO]" if self.auto_fov_enabled else ""
        freeze_txt = "[MEZERNÍK] ODMRAZIT" if self.frozen else "[MEZERNÍK] ZMRAZIT"
        c1 = self.font_main.render(f"{fov_txt} {auto_badge} | {freeze_txt}", True, (220, 220, 220))
        c2 = self.font_main.render(f"Rozsah: [R/E] Min: {self.min_range} mm | [T/G] Max: {self.max_range} mm", True, (170, 180, 195))
        c3 = self.font_main.render(f"Odrazivost: [I/K] Min Intenzita: {self.min_intensity} | [C] Resetovat mapy", True, (170, 180, 195))
        det_col = (100, 255, 140) if self.detect_details_enabled else (180, 185, 195)
        det_txt = "[ZAPNUTO - Stěny + Šipky 38cm]" if self.detect_details_enabled else "[VYPNUTO]"
        c4 = self.font_bold.render(f"[O] Stěny & Nájezdy do svíček: {det_txt}", True, det_col)
        self.screen.blit(c1, (22, cy)); cy += 16
        self.screen.blit(c2, (22, cy)); cy += 16
        self.screen.blit(c3, (22, cy)); cy += 16
        self.screen.blit(c4, (22, cy))

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

        # Kotvy (všech až 6 kotev seřazených a odlišených barvami)
        if loc.locked and loc.anchors:
            for a in loc.anchors:
                p1 = self.to_screen(a['p1'][0], a['p1'][1], offset_x)
                p2 = self.to_screen(a['p2'][0], a['p2'][1], offset_x)
                thick = 4 if a['rank'] == 1 else (3 if a['rank'] <= 2 else 2)
                pygame.draw.line(self.screen, a['color'], p1, p2, thick)
                mx = (p1[0] + p2[0]) // 2; my = (p1[1] + p2[1]) // 2
                off_y = -14 if (a['rank'] % 2 == 1) else 6
                lbl = self.font_bold.render(f"K{a['rank']}: {int(a['dist'])} mm", True, a['color'])
                self.screen.blit(lbl, (mx + 8, my + off_y))

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

        # Jemné stěny a nájezdy v Panelu 2 (Etalon) při stisknutí [O]
        if self.detect_details_enabled and ref_boxes:
            cloud_data = self.last_frame.get("cloud", [])
            for b_idx, cl in enumerate(ref_boxes):
                raw_d = self.compute_raw_box_details(cl, cloud_data, 0.0, 0.0, 0.0)
                # Stěny
                for w_seg in raw_d.get('walls_world', []):
                    wp1 = self.to_screen(w_seg[0][0], w_seg[0][1], offset_x)
                    wp2 = self.to_screen(w_seg[1][0], w_seg[1][1], offset_x)
                    pygame.draw.line(self.screen, (255, 250, 130), wp1, wp2, 3)
                # Svíčka
                cw = raw_d.get('candle_world', (cl['cx'], cl['cy']))
                cs = self.to_screen(cw[0], cw[1], offset_x)
                pygame.draw.circle(self.screen, (255, 50, 30), cs, 6)
                pygame.draw.circle(self.screen, (255, 220, 0), cs, 3)
                # Šipka
                wp_p = raw_d.get('waypoint_world', (cw[0], cw[1]))
                wp_s = self.to_screen(wp_p[0], wp_p[1], offset_x)
                draw_arrow(self.screen, (255, 75, 40), wp_s, cs, width=3, tip_len=12, tip_width=6)

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
        curr_boxes = self.processed_boxes if self.processed_boxes else parse_box_list(self.last_frame.get("static_boxes", []))
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
            transformed_boxes.append({
                'box': cl,
                'wx': wx, 'wy': wy
            })

        # 3. KOTVY MEZI NOVÝMI BOUNDING BOXY (VŠECHNY SPÁROVANÉ KOTVY Z ETALONU)
        match = loc.last_match_info
        matched_anchors = match.get('matched_anchors', []) if match else []

        # Vykreslení všech spárovaných kotev z grafu
        for ma in matched_anchors:
            anc = ma['anchor']
            p1 = self.to_screen(ma['p1_world'][0], ma['p1_world'][1], offset_x)
            p2 = self.to_screen(ma['p2_world'][0], ma['p2_world'][1], offset_x)
            thick = 4 if anc['rank'] == 1 else (3 if anc['rank'] <= 2 else 2)
            pygame.draw.line(self.screen, anc['color'], p1, p2, thick)
            mx = (p1[0] + p2[0]) // 2; my = (p1[1] + p2[1]) // 2
            off_y = -14 if (anc['rank'] % 2 == 1) else 6
            lbl = self.font_bold.render(f"K{anc['rank']}: {int(ma['cur_dist'])} mm", True, anc['color'])
            self.screen.blit(lbl, (mx + 8, my + off_y))

        # Pokud ještě nemáme spárované kotvy, zobrazíme surové vzdálenosti mezi detekovanými boxy
        if not matched_anchors:
            n_tb = len(transformed_boxes)
            for i in range(n_tb):
                for j in range(i + 1, n_tb):
                    b1 = transformed_boxes[i]
                    b2 = transformed_boxes[j]
                    d_cur = math.hypot(b1['wx'] - b2['wx'], b1['wy'] - b2['wy'])
                    p1 = self.to_screen(b1['wx'], b1['wy'], offset_x)
                    p2 = self.to_screen(b2['wx'], b2['wy'], offset_x)
                    pygame.draw.line(self.screen, (0, 180, 220), p1, p2, 1)
                    mx = (p1[0] + p2[0]) // 2; my = (p1[1] + p2[1]) // 2
                    self.screen.blit(self.font_main.render(f"{int(d_cur)} mm", True, (0, 200, 240)), (mx + 6, my - 8))

        # 4. ROZHODOVACÍ PAPRSKY SHOD MEZI INLIERY A REFERENČNÍMI BODY
        if match and match.get('inlier_pairs'):
            for ref_pt, cur_trans_pt, err in match['inlier_pairs']:
                rp = self.to_screen(ref_pt[0], ref_pt[1], offset_x)
                cp = self.to_screen(cur_trans_pt[0], cur_trans_pt[1], offset_x)
                pygame.draw.line(self.screen, (255, 60, 200), rp, cp, 2)
                pygame.draw.circle(self.screen, (255, 200, 240), cp, 4)
                mid_x = (rp[0] + cp[0]) // 2
                mid_y = (rp[1] + cp[1]) // 2
                self.screen.blit(self.font_main.render(f"Δ{err:.0f}mm", True, (255, 130, 220)), (mid_x + 4, mid_y - 6))

        # 5. VYKRESLENÍ PLNÝCH NENATOČENÝCH BOUNDING BOXŮ Z NOVÉHO MÍSTA (AABB)
        for idx, tb in enumerate(transformed_boxes):
            cl = tb['box']
            wx = tb['wx']
            wy = tb['wy']
            hw = cl['width'] * 0.5
            hh = cl['height'] * 0.5

            p_min = self.to_screen(wx - hw, wy + hh, offset_x)
            p_max = self.to_screen(wx + hw, wy - hh, offset_x)
            rx = min(p_min[0], p_max[0])
            ry = min(p_min[1], p_max[1])
            rw = max(10, abs(p_max[0] - p_min[0]))
            rh = max(10, abs(p_max[1] - p_min[1]))

            is_rec = cl.get('is_recovered', False)
            is_exp = cl.get('is_expected', False)
            is_out = cl.get('is_outside', False)

            if is_rec:
                box_col = (0, 230, 255)
                fill_col = (0, 200, 240, 45)
            elif is_exp:
                box_col = (0, 255, 170) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 200, 40)
                fill_col = (0, 200, 140, 35) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 170, 30, 35)
            elif is_out:
                box_col = (130, 130, 140)
                fill_col = (100, 100, 110, 20)
            else:
                box_col = (0, 240, 160) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 180, 40)
                fill_col = (0, 200, 140, 30) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 160, 30, 30)

            # Poloprůhledná výplň nenatočeného pravoúhlého boxu
            s_fill = pygame.Surface((rw, rh), pygame.SRCALPHA)
            s_fill.fill(fill_col)
            self.screen.blit(s_fill, (rx, ry))
            pygame.draw.rect(self.screen, box_col, (rx, ry, rw, rh), 2)

            # Středový křížek
            scx, scy = self.to_screen(wx, wy, offset_x)
            pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 6)
            pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (scx - 7, scy), (scx + 7, scy), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (scx, scy - 7), (scx, scy + 7), 2)

            # Popisky nového boxu
            tag = " [ZACHRÁNĚNÁ]" if is_rec else (f" [★ ETALON #{cl.get('expected_ref_idx', '')}]" if is_exp else (" [VNĚ POLE]" if is_out else ""))
            lbl1 = self.font_bold.render(f"NOVÝ #{idx+1} [{cl['type']}]{tag}: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
            lbl2 = self.font_main.render(f"SVĚT: [{int(wx):+d}, {int(wy):+d}] | R:{int(cl['dist_mm'])} mm ({cl.get('pts_count', 0)}b)", True, (255, 245, 140))
            self.screen.blit(lbl1, (scx + 10, scy - 18))
            self.screen.blit(lbl2, (scx + 10, scy - 4))

        # 5b. JEMNÁ DETEKCE STĚN, SVÍČEK A NÁJEZDOVÝCH ŠIPEK STRIKTNĚ UVNITŘ BOUNDING BOXŮ (KLÁVESA [O])
        self.current_box_details = []
        if self.detect_details_enabled and transformed_boxes:
            cloud_data = self.last_frame.get("cloud", [])
            f_xmin, f_xmax, f_ymin, f_ymax = self.field_map.get_auto_field_bounds()

            # Převedeme cíle z ESP32 do světových souřadnic (pokud existují)
            esp_world_targets = []
            if self.esp_targets:
                th_rad = math.radians(loc.robot_deg)
                for et in self.esp_targets:
                    esp_world_targets.append(transform_esp_target_to_world(et, loc.robot_x, loc.robot_y, th_rad))

            matched_esp_indices = set()
            raw_details = []

            for tb in transformed_boxes:
                cl = tb['box']
                wx, wy = tb['wx'], tb['wy']
                hw = max(20.0, cl['width'] * 0.5)
                hh = max(20.0, cl['height'] * 0.5)

                # 1. Kontrola hranic hřiště: Box musí ležet uvnitř hřiště (nikdy za startovní stěnou)
                if wy < (f_ymin - 80.0) or wy > (f_ymax + 80.0) or wx < (f_xmin - 100.0) or wx > (f_xmax + 100.0):
                    continue

                # 2. Hledáme odpovídající cíl z ESP32, který leží UVNITŘ TOHOTO KONKRÉTNÍHO BOXU
                matched_esp = None
                best_esp_dist = max(180.0, math.hypot(hw, hh) + 40.0)
                best_esp_idx = -1

                for e_idx, e_tgt in enumerate(esp_world_targets):
                    if e_idx in matched_esp_indices:
                        continue
                    c_wx, c_wy = e_tgt['candle_world']
                    d = math.hypot(c_wx - wx, c_wy - wy)
                    if d < best_esp_dist:
                        # Ověříme, že cíl z ESP32 leží v hřišti (Y >= 20 mm)
                        if c_wy >= 20.0 and (f_xmin - 60.0) <= c_wx <= (f_xmax + 60.0):
                            best_esp_dist = d
                            best_esp_idx = e_idx
                            matched_esp = dict(e_tgt)

                if matched_esp is not None:
                    matched_esp_indices.add(best_esp_idx)
                    # Bezpečnostní pojistka: svíčka leží STRIKTNĚ uvnitř tohoto Bounding Boxu
                    c_x = max(wx - hw, min(wx + hw, matched_esp['candle_world'][0]))
                    c_y = max(wy - hh, min(wy + hh, matched_esp['candle_world'][1]))
                    c_y = max(30.0, min(f_ymax - 30.0, c_y))
                    c_x = max(f_xmin + 30.0, min(f_xmax - 30.0, c_x))
                    matched_esp['candle_world'] = (c_x, c_y)
                    matched_esp['box'] = cl
                    raw_details.append(matched_esp)
                else:
                    # Pokud z ESP32 cíl chybí (např. zachráněná svíčka), spočítáme detail STRIKTNĚ uvnitř tohoto boxu
                    raw_d = self.compute_raw_box_details(
                        cl, cloud_data, loc.robot_x, loc.robot_y, math.radians(loc.robot_deg)
                    )
                    # Pojistka: svíčka leží STRIKTNĚ uvnitř tohoto boxu a uvnitř pole
                    c_x = max(wx - hw, min(wx + hw, raw_d['candle_world'][0]))
                    c_y = max(wy - hh, min(wy + hh, raw_d['candle_world'][1]))
                    c_y = max(30.0, min(f_ymax - 30.0, c_y))
                    c_x = max(f_xmin + 30.0, min(f_xmax - 30.0, c_x))
                    raw_d['candle_world'] = (c_x, c_y)
                    raw_details.append(raw_d)

            # Časová stabilizace pro stěny a svíčky v potvrzených boxech
            self.current_box_details = self.detail_stabilizer.update_all(
                raw_details, loc.robot_x, loc.robot_y
            )

            # Finální bezpečnostní filtr: NIKDY nepustit žádný cíl mimo mantinely hřiště!
            self.current_box_details = [
                d for d in self.current_box_details
                if (d['candle_world'][1] >= 20.0 and d['candle_world'][1] <= (f_ymax + 50.0) and
                    (f_xmin - 80.0) <= d['candle_world'][0] <= (f_xmax + 80.0))
            ]

        if self.detect_details_enabled and self.current_box_details:
            r_pos_scr = self.to_screen(loc.robot_x, loc.robot_y, offset_x)

            for idx, details in enumerate(self.current_box_details):
                wp_world = details['waypoint_world']
                cand_world = details['candle_world']
                wp_scr = self.to_screen(wp_world[0], wp_world[1], offset_x)
                cand_scr = self.to_screen(cand_world[0], cand_world[1], offset_x)

                # 1. Trasa od středu lidaru k Waypointu (nájezdovému bodu)
                pygame.draw.line(self.screen, (0, 240, 255), r_pos_scr, wp_scr, 2)
                mid_tr_x = (r_pos_scr[0] + wp_scr[0]) // 2
                mid_tr_y = (r_pos_scr[1] + wp_scr[1]) // 2
                lbl_path = self.font_bold.render(f"CESTA #{idx+1}: {int(details['travel_dist_mm'])} mm", True, (0, 255, 240))
                self.screen.blit(lbl_path, (mid_tr_x + 6, mid_tr_y - 8))

                # 2. Cílový bod nájezdu (Approach Waypoint - terčík)
                pygame.draw.circle(self.screen, (0, 255, 200), wp_scr, 9, 2)
                pygame.draw.circle(self.screen, (255, 230, 0), wp_scr, 4)
                pygame.draw.line(self.screen, (0, 255, 200), (wp_scr[0] - 12, wp_scr[1]), (wp_scr[0] + 12, wp_scr[1]), 1)
                pygame.draw.line(self.screen, (0, 255, 200), (wp_scr[0], wp_scr[1] - 12), (wp_scr[0], wp_scr[1] + 12), 1)
                self.screen.blit(self.font_bold.render(f"NÁJEZD #{idx+1} [38 cm]", True, (0, 255, 220)), (wp_scr[0] + 12, wp_scr[1] - 8))

                # 3. Nájezdová šipka 38 cm mířící do plamene svíčky (středem otevřeného úhlu nebo po kolmici)
                draw_arrow(self.screen, (255, 75, 40), wp_scr, cand_scr, width=4, tip_len=18, tip_width=9)

                # 4. Detekované stabilizované stěny uvnitř boxu
                for w_seg in details['walls_world']:
                    wp1 = self.to_screen(w_seg[0][0], w_seg[0][1], offset_x)
                    wp2 = self.to_screen(w_seg[1][0], w_seg[1][1], offset_x)
                    pygame.draw.line(self.screen, (255, 250, 130), wp1, wp2, 5)
                    pygame.draw.circle(self.screen, (255, 255, 255), wp1, 3)
                    pygame.draw.circle(self.screen, (255, 255, 255), wp2, 3)

                # 5. Detekovaná svíčka (plamínek / kruh)
                pygame.draw.circle(self.screen, (255, 50, 30), cand_scr, 8)
                pygame.draw.circle(self.screen, (255, 220, 0), cand_scr, 5)
                pygame.draw.circle(self.screen, (255, 255, 255), cand_scr, 2)
                type_tag = "ROH (2 STĚNY)" if details['is_corner'] else ("1 STĚNA" if not details['is_solo'] else "VOLNÁ")
                src_tag = " [ESP32]" if details.get('from_esp32') else ""
                self.screen.blit(self.font_bold.render(f"🔥 SVÍČKA [{type_tag}] #{idx+1}{src_tag}", True, (255, 215, 60)), (cand_scr[0] + 10, cand_scr[1] + 8))

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
        opt_tag = " [O: 100% ESP32 NÁJEZDY ZAPNUTY]" if self.detect_details_enabled else " [O: NÁJEZDY Vypnuty]"

        self.screen.blit(self.font_big.render(f"3. SHODA [{fov_tag}]{opt_tag}: {pat_name}", True, (255, 200, 60) if self.detect_details_enabled else (80, 255, 160)), (offset_x + 25, 17))
        sub_str = f"Inliery: {inliers} | Kotvy: {len(matched_anchors)}/6 | Chyba: {err:.1f} mm | Od startu: {dist_from_start:.0f} mm | Jistota: {conf_pct}%"
        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (offset_x + 25, 38))

        # 9. SPODNÍ TIP PANEL
        tip_box = pygame.Surface((570, 75))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (offset_x + 15, PANEL_H - 85))
        pygame.draw.rect(self.screen, (60, 30, 50), (offset_x + 15, PANEL_H - 85, 570, 75), 1)
        self.screen.blit(self.font_bold.render("DRUHÁ STATICKÁ MAPA & NÁJEZDOVÉ ŠIPKY (100% Z ESP32):", True, (255, 150, 215)), (offset_x + 25, PANEL_H - 79))
        self.screen.blit(self.font_main.render("• Výpočet 100% na ESP32: stěny, svíčky, 38 cm nájezdy i trasa počítány přímo ve firmware!", True, (100, 255, 140)), (offset_x + 25, PANEL_H - 61))
        self.screen.blit(self.font_main.render("• Geometrie: bisektor u rohu, kolmice u 1 stěny. Žádné křížení stěn!", True, (0, 240, 255)), (offset_x + 25, PANEL_H - 43))

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
        route = []
        if self.esp_route and self.current_box_details:
            id_map = {det['id']: det for det in self.current_box_details}
            for rid in self.esp_route:
                if rid in id_map:
                    det = id_map[rid]
                    route.append({
                        'id': det['id'],
                        'type': det['type'],
                        'x': det['candle_world'][0],
                        'y': det['candle_world'][1],
                        'waypoint_x': det['waypoint_world'][0],
                        'waypoint_y': det['waypoint_world'][1],
                        'dist_from_robot': det['travel_dist_mm'],
                        'is_visible': True,
                        'is_solo': det['is_solo']
                    })
        # Doplníme zbývající potvrzené cíle z pole
        if self.current_box_details and len(route) < len(self.current_box_details):
            existing_ids = {r['id'] for r in route}
            for det in self.current_box_details:
                if det['id'] not in existing_ids:
                    route.append({
                        'id': det['id'],
                        'type': det['type'],
                        'x': det['candle_world'][0],
                        'y': det['candle_world'][1],
                        'waypoint_x': det['waypoint_world'][0],
                        'waypoint_y': det['waypoint_world'][1],
                        'dist_from_robot': det['travel_dist_mm'],
                        'is_visible': True,
                        'is_solo': det['is_solo']
                    })
        if not route:
            route = fm.compute_extinguish_route()

        # Bezpečnostní filtr: Trasa vede POUZE přes cíle uvnitř hřiště (Y >= 20 mm)
        route = [t for t in route if t['y'] >= 20.0 and (ymin - 50.0) <= t['y'] <= (ymax + 50.0) and (xmin - 50.0) <= t['x'] <= (xmax + 50.0)]

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

        # Vykreslení stěn, svíček a nájezdových šipek přímo v poli (pokud je [O] aktivní)
        if self.detect_details_enabled and self.current_box_details:
            r_pos = fm.robot_pos
            r_sx, r_sy = f_to_screen(r_pos[0], r_pos[1])
            for idx, det in enumerate(self.current_box_details):
                wp_w = det['waypoint_world']
                cand_w = det['candle_world']
                wp_s = f_to_screen(wp_w[0], wp_w[1])
                cand_s = f_to_screen(cand_w[0], cand_w[1])

                # Trasa od robota k Waypointu
                pygame.draw.line(self.screen, (0, 240, 255), (r_sx, r_sy), wp_s, 2)

                # Terčík waypointu
                pygame.draw.circle(self.screen, (0, 255, 200), wp_s, 5, 1)
                pygame.draw.circle(self.screen, (255, 230, 0), wp_s, 2)

                # Nájezdová šipka
                draw_arrow(self.screen, (255, 75, 40), wp_s, cand_s, width=2, tip_len=8, tip_width=4)

                # Stěny uvnitř boxu
                for w_seg in det['walls_world']:
                    ws1 = f_to_screen(w_seg[0][0], w_seg[0][1])
                    ws2 = f_to_screen(w_seg[1][0], w_seg[1][1])
                    pygame.draw.line(self.screen, (255, 250, 130), ws1, ws2, 3)

                # Svíčka (střed)
                pygame.draw.circle(self.screen, (255, 50, 30), cand_s, 4)
                pygame.draw.circle(self.screen, (255, 220, 0), cand_s, 2)

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
        hw_tag = " [100% ESP32 FIRMWARE]" if (self.esp_route and self.current_box_details) else ""
        self.screen.blit(self.font_big.render(f"SEZNAM CÍLŮ V POLI & OPTIMÁLNÍ TRASA{hw_tag}", True, (0, 220, 255)), (right_x, r_y))
        r_y += 24

        display_targets = route if route else field_targets
        if not display_targets:
            self.screen.blit(self.font_main.render("Zatím nebyly detekovány žádné cíle uvnitř hřiště.", True, (150, 160, 175)), (right_x, r_y))
        else:
            for idx, t in enumerate(display_targets):
                is_vis = t.get('is_visible', True)
                t_type = t.get('type', "SVÍČKA" if t.get('is_solo') else "ROH/STĚNA")
                status_txt = f"[{t_type}]" if 'type' in t else ("VIDITELNÝ" if is_vis else "ZASTÍNĚNÝ")
                col_st = (100, 255, 120) if is_vis else (240, 180, 50)

                t_id = t.get('id', idx + 1)
                t_name = f"#{idx+1} CÍL ID {t_id} [POŘADÍ HAŠENÍ {idx+1}]"
                self.screen.blit(self.font_bold.render(t_name, True, (255, 215, 0)), (right_x, r_y))
                self.screen.blit(self.font_bold.render(status_txt, True, col_st), (right_x + 220, r_y))
                r_y += 17

                d_rob = t.get('dist_from_robot', math.hypot(t['x'] - rx, t['y'] - ry))
                pos_str = f"Svíčka: X={t['x']:+.0f}, Y={t['y']:+.0f} mm | R dojezdu: {d_rob:.0f} mm"
                self.screen.blit(self.font_main.render(pos_str, True, (210, 215, 225)), (right_x + 10, r_y))
                r_y += 18

            # Stavový blok výpočtu pod seznamem
            r_y += 6
            self.screen.blit(self.font_bold.render("ARCHITEKTURA VÝPOČTU:", True, (200, 210, 225)), (right_x, r_y))
            r_y += 18
            if self.esp_targets:
                self.screen.blit(self.font_main.render("✓ 100% ESP32: Geometrie, stěny, svíčky, 38cm nájezdy i trasa", True, (100, 255, 140)), (right_x, r_y))
                r_y += 16
                self.screen.blit(self.font_main.render("✓ Robot může po odpojení PC hasit zcela autonomně!", True, (0, 240, 255)), (right_x, r_y))
            else:
                self.screen.blit(self.font_main.render("• Python emulace (připojte ESP32 s novým firmwarem)", True, (240, 180, 50)), (right_x, r_y))

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
                            self.current_box_details = []
                            self.detail_stabilizer.reset()
                            if self.auto_fov_enabled and not self.fov_limited:
                                self.fov_limited = True
                                self.send_cmd("FOV 1")
                                print("[FoV AUTO] Návrat na start -> Automaticky aktivuji 180° DOPŘEDNÝ KUŽEL!")
                            print("[LOC] Etalon odemčen. Můžete znovu nasbírat výchozí mapu.")

                    elif event.key == pygame.K_n:
                        self.field_map.reset()
                        self.current_box_details = []
                        self.detail_stabilizer.reset()
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
                    elif event.key == pygame.K_o:
                        self.detect_details_enabled = not self.detect_details_enabled
                        st = "ZAPNUTA" if self.detect_details_enabled else "VYPNUTA"
                        print(f"[DETAIL] Jemná detekce svíček, stěn a nájezdových cest: {st}")

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
