#!/usr/bin/env python3
"""
LIDAR_SVICKY_27 - Dvoupanelový vizualizační terminál (Kybernoid 27)
Autor: Antigravity & Nik

Dva panely vedle sebe:
  1. LEVÝ PANEL: Živý radar (Live LIDAR scan) v reálném čase + přesný HUD.
  2. PRAVÝ PANEL: Stabilní herní mapa (World Model) + LOKALIZACE ROBOTA.

Klávesové zkratky a ovládání:
  [F]         - Přepnout režim: 180° Kužel / 360° Celé pole (výchozí 180°)
  [MEZERNÍK]  - Zmrazit / Odmrazit obraz (Freeze)
  [R / E]     - Min vzdálenost (+50 / -50 mm)
  [T / G]     - Max vzdálenost (+200 / -200 mm)
  [I / K]     - Min intenzita (+5 / -5)
  [C]         - Vymazat a přepočítat stabilní mapu hřiště
  [L]         - ZAMKNOUT mapu jako referenční svět (spustí lokalizaci)
  [+ / -]     - Zoom in / Zoom out (funguje také kolečko myši)
  Klik myší   - Detailní inspekce souřadnic vybraného bodu / objektu
"""

import sys, os, time, json, math
import pygame
import serial
from serial.tools import list_ports

# Rozměry
PANEL_W     = 840
PANEL_H     = 850
WIN_W       = PANEL_W * 2
WIN_H       = PANEL_H

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

    def lock_map(self, conf_candles, conf_walls):
        """Uloží aktuální potvrzené svíčky a stěny jako fixní referenční mapu světa."""
        self.ref_candles = [dict(c) for c in conf_candles]
        self.ref_walls   = [dict(w) for w in conf_walls]
        self.ref_candle_dists = []
        self.ref_cw_dists     = []
        self.candle_tracker   = []

        # Invariantní vzdálenosti mezi všemi svíčkami
        for i in range(len(self.ref_candles)):
            for j in range(i + 1, len(self.ref_candles)):
                d = math.hypot(self.ref_candles[i]['x'] - self.ref_candles[j]['x'],
                               self.ref_candles[i]['y'] - self.ref_candles[j]['y'])
                self.ref_candle_dists.append((i, j, d))

        # Invariantní vzdálenosti mezi svíčkami a středy stěn
        for ci, c in enumerate(self.ref_candles):
            for wi, w in enumerate(self.ref_walls):
                d = math.hypot(c['x'] - w['mx'], c['y'] - w['my'])
                self.ref_cw_dists.append((ci, wi, d))

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
        self.candle_tracker = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 0.0
        self.trajectory = [(0.0, 0.0)]
        self.is_stationary = True

    @staticmethod
    def _rigid_transform_2d(src, dst):
        """Metoda nejmenších čtverců (Procrustes) pro analytický výpočet (tx, ty, theta)."""
        n = len(src)
        if n == 0: return 0.0, 0.0, 0.0
        if n == 1:
            return dst[0][0] - src[0][0], dst[0][1] - src[0][1], 0.0

        cx_s = sum(p[0] for p in src) / n
        cy_s = sum(p[1] for p in src) / n
        cx_d = sum(p[0] for p in dst) / n
        cy_d = sum(p[1] for p in dst) / n

        h00 = h01 = h10 = h11 = 0.0
        for (sx, sy), (dx, dy) in zip(src, dst):
            scx = sx - cx_s; scy = sy - cy_s
            dcx = dx - cx_d; dcy = dy - cy_d
            h00 += scx * dcx; h01 += scx * dcy
            h10 += scy * dcx; h11 += scy * dcy

        theta = math.atan2(h10 - h01, h00 + h11)
        cos_t = math.cos(theta); sin_t = math.sin(theta)

        tx = cx_d - (cx_s * cos_t - cy_s * sin_t)
        ty = cy_d - (cx_s * sin_t + cy_s * cos_t)
        return tx, ty, theta

    def estimate_pose(self, live_candles, live_walls):
        """
        Určí pozici a orientaci robota (x, y, theta) vůči referenční mapě
        pomocí invariantních vzdáleností mezi svíčkami a stěnami.
        Funguje při jakémkoliv pohybu i přemístění robota.
        """
        if not self.locked or (len(self.ref_candles) + len(self.ref_walls)) < 2:
            return False

        # Převod na standardní formát (x, y) pro svíčky a (mx, my) pro stěny
        raw_cands = []
        for c in live_candles:
            if isinstance(c, (list, tuple)):
                raw_cands.append((float(c[0]), float(c[1])))
            elif isinstance(c, dict):
                raw_cands.append((float(c['x']), float(c['y'])))

        raw_walls = []
        for w in live_walls:
            if isinstance(w, (list, tuple)):
                mx = (float(w[0]) + float(w[2])) * 0.5
                my = (float(w[1]) + float(w[3])) * 0.5
                raw_walls.append((mx, my))
            elif isinstance(w, dict):
                raw_walls.append((float(w['mx']), float(w['my'])))

        # Krátkodobý tracker svíček (vyhlazuje chvění měření a překlenuje krátké výpadky na 1-2 snímky)
        matched_tracker = set()
        for cx, cy in raw_cands:
            best_idx = -1
            best_d = 100.0
            for idx, tc in enumerate(self.candle_tracker):
                if idx in matched_tracker: continue
                d = math.hypot(tc['x'] - cx, tc['y'] - cy)
                if d < best_d:
                    best_d = d
                    best_idx = idx
            if best_idx >= 0:
                self.candle_tracker[best_idx]['x'] = 0.7 * self.candle_tracker[best_idx]['x'] + 0.3 * cx
                self.candle_tracker[best_idx]['y'] = 0.7 * self.candle_tracker[best_idx]['y'] + 0.3 * cy
                self.candle_tracker[best_idx]['hits'] += 1
                self.candle_tracker[best_idx]['missed'] = 0
                matched_tracker.add(best_idx)
            else:
                self.candle_tracker.append({'x': cx, 'y': cy, 'hits': 1, 'missed': 0})
                matched_tracker.add(len(self.candle_tracker) - 1)

        for idx, tc in enumerate(self.candle_tracker):
            if idx not in matched_tracker:
                tc['missed'] += 1

        # Rychlý rozpad (max 4 missed snímky = 0.6 s) -> při posunu robota nezůstávají duchové!
        self.candle_tracker = [c for c in self.candle_tracker if c['missed'] <= 4]

        # Výběr kandidátů na svíčky:
        # Preferujeme stabilizované body z trackeru, případně přímo živé ze snímku
        tracked_cands = [(c['x'], c['y']) for c in self.candle_tracker if c['hits'] >= 2]
        cands = tracked_cands if len(tracked_cands) >= 2 else raw_cands
        walls = raw_walls

        best_tx = None
        best_ty = None
        best_theta = None
        best_inliers = 0
        best_error = 9999.0

        # ========================================================
        # 1. PÁROVÁNÍ PODLE INVARIANTNÍCH VZDÁLENOSTÍ MEZI SVÍČKAMI
        # ========================================================
        if len(cands) >= 2 and len(self.ref_candles) >= 2:
            for a in range(len(cands)):
                for b in range(a + 1, len(cands)):
                    d_cur = math.hypot(cands[a][0] - cands[b][0], cands[a][1] - cands[b][1])

                    # Hledáme odpovídající dvojici svíček v referenční mapě
                    for (ri, rj, d_ref) in self.ref_candle_dists:
                        if abs(d_cur - d_ref) < 130.0:  # tolerance 13 cm na rozestup
                            rI = (self.ref_candles[ri]['x'], self.ref_candles[ri]['y'])
                            rJ = (self.ref_candles[rj]['x'], self.ref_candles[rj]['y'])

                            # Dvě možné orientace přiřazení: (a->ri, b->rj) a (a->rj, b->ri)
                            for (p1, p2, ref1, ref2) in [(cands[a], cands[b], rI, rJ), (cands[a], cands[b], rJ, rI)]:
                                v_ref_x = ref2[0] - ref1[0]; v_ref_y = ref2[1] - ref1[1]
                                v_cur_x = p2[0] - p1[0];     v_cur_y = p2[1] - p1[1]
                                th = math.atan2(v_ref_y, v_ref_x) - math.atan2(v_cur_y, v_cur_x)
                                cos_t = math.cos(th); sin_t = math.sin(th)

                                tx = ref1[0] - (p1[0]*cos_t - p1[1]*sin_t)
                                ty = ref1[1] - (p1[0]*sin_t + p1[1]*cos_t)

                                # Ověříme shodu se všemi svíčkami a stěnami v mapě
                                inliers = 0
                                err_sum = 0.0
                                inlier_src = []
                                inlier_dst = []

                                for cp in cands:
                                    wx = cp[0]*cos_t - cp[1]*sin_t + tx
                                    wy = cp[0]*sin_t + cp[1]*cos_t + ty
                                    min_d = 9999.0
                                    best_rc = None
                                    for rc in self.ref_candles:
                                        d = math.hypot(wx - rc['x'], wy - rc['y'])
                                        if d < min_d: min_d = d; best_rc = rc
                                    if min_d < 180.0 and best_rc:
                                        inliers += 2 # Svíčka má váhu 2
                                        err_sum += min_d
                                        inlier_src.append(cp)
                                        inlier_dst.append((best_rc['x'], best_rc['y']))

                                for wp in walls:
                                    wx = wp[0]*cos_t - wp[1]*sin_t + tx
                                    wy = wp[0]*sin_t + wp[1]*cos_t + ty
                                    min_d = min((math.hypot(wx - rw['mx'], wy - rw['my']) for rw in self.ref_walls), default=9999.0)
                                    if min_d < 220.0:
                                        inliers += 1
                                        err_sum += min_d

                                avg_err = (err_sum / inliers) if inliers > 0 else 9999.0

                                if inliers > best_inliers or (inliers == best_inliers and avg_err < best_error):
                                    best_inliers = inliers
                                    best_error = avg_err
                                    if len(inlier_src) >= 2:
                                        rtx, rty, rth = self._rigid_transform_2d(inlier_src, inlier_dst)
                                        best_tx = rtx; best_ty = rty; best_theta = rth
                                    else:
                                        best_tx = tx; best_ty = ty; best_theta = th

        # ========================================================
        # 2. ZÁLOŽNÍ METODA: PÁROVÁNÍ SVÍČKA-STĚNA / STĚNA-STĚNA
        # ========================================================
        if best_inliers < 2:
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

                                tx, ty, th = self._rigid_transform_2d([p1, p2], [r1, r2])
                                cos_t = math.cos(th); sin_t = math.sin(th)

                                inliers = 0
                                err_sum = 0.0
                                for p in all_cur:
                                    wx = p[0]*cos_t - p[1]*sin_t + tx
                                    wy = p[0]*sin_t + p[1]*cos_t + ty
                                    min_d = min((math.hypot(wx - rp[0], wy - rp[1]) for rp in all_ref if rp[2] == p[2]), default=9999.0)
                                    if min_d < 200.0:
                                        inliers += 1
                                        err_sum += min_d

                                avg_err = (err_sum / inliers) if inliers > 0 else 9999.0
                                if inliers > best_inliers or (inliers == best_inliers and avg_err < best_error):
                                    best_inliers = inliers
                                    best_error = avg_err
                                    best_tx = tx; best_ty = ty; best_theta = th

        # ========================================================
        # 3. APLIKACE VÝSLEDKU: STATICKÁ POLOHA, SETRVAČNOST A TRAJEKTORIE
        # ========================================================
        if best_inliers >= 2 and best_tx is not None:
            cur_rad = math.radians(self.robot_deg)
            dist_diff = math.hypot(best_tx - self.robot_x, best_ty - self.robot_y)
            ang_diff = abs(math.atan2(math.sin(best_theta - cur_rad), math.cos(best_theta - cur_rad)))

            # Klidový zámek: pokud se poloha liší o méně než 20 mm a 1.5°, robot stojí na místě
            # (Absolutně statická poloha lidaru bez sebemenšího třesu!)
            if dist_diff < 20.0 and ang_diff < math.radians(1.5):
                self.is_stationary = True
                self.pose_confidence = min(1.0, self.pose_confidence + 0.05)
            else:
                self.is_stationary = False

                # Pokud došlo k velkému přemístění (např. uživatel zvedl a posunul LiDAR o > 300 mm),
                # reagujeme rychleji (alpha = 0.50), při normálním pohybu plynule setrvačně (alpha = 0.22)
                alpha = 0.50 if dist_diff > 300.0 else 0.22

                self.robot_x = (1.0 - alpha) * self.robot_x + alpha * best_tx
                self.robot_y = (1.0 - alpha) * self.robot_y + alpha * best_ty

                # Hladké vyhlazení úhlu na jednotkové kružnici
                sin_b = (1.0 - alpha) * math.sin(cur_rad) + alpha * math.sin(best_theta)
                cos_b = (1.0 - alpha) * math.cos(cur_rad) + alpha * math.cos(best_theta)
                self.robot_deg = math.degrees(math.atan2(sin_b, cos_b))

                quality = min(1.0, best_inliers / max(2, len(self.ref_candles) + len(self.ref_walls)))
                self.pose_confidence = min(1.0, 0.7 * self.pose_confidence + 0.3 * quality)

                # Záznam do projeté trajektorie (každých 25 mm)
                last_pt = self.trajectory[-1]
                if math.hypot(self.robot_x - last_pt[0], self.robot_y - last_pt[1]) > 25.0:
                    self.trajectory.append((self.robot_x, self.robot_y))
                    if len(self.trajectory) > 300:
                        self.trajectory.pop(0)

            return True
        else:
            # Krátkodobý výpadek měření – držíme pozici, jistota jemně klesá
            self.pose_confidence = max(0.0, self.pose_confidence - 0.02)
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
#  Detektor makro-shluků (Svíčka + Stěna i samostatná svíčka, max 4 boxy)
# ============================================================
class ClusterDetector:
    """
    Detekce makro-shluků překážek (Svíčka + Stěna i samostatná svíčka).
    Princip:
    1. 2D prostorové shlukování (Spatial Grid):
       Body vzdálené do 250 mm (25 cm) se spojí do jednoho shluku.
    2. Zachytí i samostatnou svíčku bez stěny:
       Stačí alespoň 3-4 body a rozměr od 20 mm.
    3. Poměr stran (Aspect Ratio) zhruba čtverec až obdélník 1:3:
       max(W, H) / max(min(W, H), 18.0) <= 3.2.
       Spolehlivě odfiltruje táhlé zdi místnosti/arény (poměry 5:1 až 20:1)!
    4. Maximální rozměr: do 650 mm (vyřadí velké zdi).
    5. Striktní limit: MAXIMÁLNĚ 4 Bounding Boxy.
    """
    def __init__(self, merge_dist_mm=250.0, min_pts=3, max_clusters=4, max_aspect_ratio=3.2):
        self.merge_dist_mm = merge_dist_mm
        self.min_pts = min_pts
        self.max_clusters = max_clusters
        self.max_aspect_ratio = max_aspect_ratio

    def detect(self, raw_cloud):
        if not raw_cloud or len(raw_cloud) < 3 * self.min_pts:
            return []

        # Extrakce bodů (x, y)
        n = len(raw_cloud) // 3
        pts = [(float(raw_cloud[i * 3]), float(raw_cloud[i * 3 + 1])) for i in range(n)]

        # 1. Prostorová mřížka (Grid Hashing s buňkami o velikosti 250 mm)
        cell_size = self.merge_dist_mm
        grid = {}
        for i, (x, y) in enumerate(pts):
            gx = int(math.floor(x / cell_size))
            gy = int(math.floor(y / cell_size))
            cell = (gx, gy)
            if cell not in grid:
                grid[cell] = []
            grid[cell].append(i)

        # Disjoint-Set (Union-Find) pro propojení bodů v 2D do 250 mm
        parent = list(range(n))
        def find(i):
            path = []
            while parent[i] != i:
                path.append(i)
                i = parent[i]
            for node in path:
                parent[node] = i
            return i

        def union(i, j):
            root_i = find(i)
            root_j = find(j)
            if root_i != root_j:
                parent[root_i] = root_j

        merge_sq = self.merge_dist_mm * self.merge_dist_mm

        # Propojení uvnitř buněk a se sousedními buňkami
        for (gx, gy), idxs in grid.items():
            for a_idx in range(len(idxs)):
                i = idxs[a_idx]
                xi, yi = pts[i]
                for b_idx in range(a_idx + 1, len(idxs)):
                    j = idxs[b_idx]
                    if (xi - pts[j][0])**2 + (yi - pts[j][1])**2 <= merge_sq:
                        union(i, j)

            for dx, dy in [(1, 0), (0, 1), (1, 1), (1, -1)]:
                nbr_cell = (gx + dx, gy + dy)
                if nbr_cell in grid:
                    nbr_idxs = grid[nbr_cell]
                    for i in idxs:
                        xi, yi = pts[i]
                        for j in nbr_idxs:
                            if (xi - pts[j][0])**2 + (yi - pts[j][1])**2 <= merge_sq:
                                union(i, j)

        clusters_dict = {}
        for i in range(n):
            root = find(i)
            if root not in clusters_dict:
                clusters_dict[root] = []
            clusters_dict[root].append(pts[i])

        candidates = []
        for root, c in clusters_dict.items():
            k = len(c)
            if k < self.min_pts:
                continue

            # Rozměry pravoúhlého boxu
            min_x = min(p[0] for p in c)
            max_x = max(p[0] for p in c)
            min_y = min(p[1] for p in c)
            max_y = max(p[1] for p in c)
            width = max_x - min_x
            height = max_y - min_y

            max_side = max(width, height)
            min_side = min(width, height)

            # Maximální rozpětí (průměr shluku)
            max_d = 0.0
            sample_c = c if k <= 40 else c[::max(1, k // 30)]
            for i in range(len(sample_c)):
                for j in range(i + 1, len(sample_c)):
                    d_ij = math.hypot(sample_c[i][0] - sample_c[j][0], sample_c[i][1] - sample_c[j][1])
                    if d_ij > max_d:
                        max_d = d_ij

            span_mm = max(max_d, max_side)

            # 1. Zahození zdí (> 650 mm)
            if span_mm > 650.0 or max_side > 620.0:
                continue

            # 2. Zahození mikroskopického šumu (< 20 mm)
            if span_mm < 20.0:
                continue

            # 3. Kontrola poměru stran (Aspect Ratio):
            # Pro malé shluky (do 7 bodů, např. osamocená svíčka) na poměru stran nezáleží.
            # Pro větší objekty musí mít tvar zhruba čtverce až obdélníku cca 1:3 (odfiltruje zdi 1:10)
            aspect_ratio = max_side / max(min_side, 18.0)
            if k > 7 and aspect_ratio > self.max_aspect_ratio:
                continue

            # Středový bod (těžiště shluku)
            cx = sum(p[0] for p in c) / k
            cy = sum(p[1] for p in c) / k
            dist_mm = math.hypot(cx, cy)

            is_solo_candle = (span_mm < 140.0)
            target_type = "SVÍČKA" if is_solo_candle else "SVÍČKA+STĚNA"

            candidates.append({
                'points': c,
                'pts_count': k,
                'cx': cx, 'cy': cy,
                'min_x': min_x, 'max_x': max_x,
                'min_y': min_y, 'max_y': max_y,
                'width': width,
                'height': height,
                'span_mm': span_mm,
                'dist_mm': dist_mm,
                'aspect_ratio': aspect_ratio,
                'type': target_type
            })

        # Výběr MAXIMÁLNĚ 4 kandidátů:
        # Bližší bounding boxy mají přednost (řadíme podle vzdálenosti vzestupně)
        candidates.sort(key=lambda item: item['dist_mm'])
        return candidates[:self.max_clusters]


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

        # Výchozí nastavení přesně dle specifikace a screenshotu
        self.fov_limited = True
        self.min_range = 130
        self.max_range = 1800
        self.min_intensity = 15
        self.view_span_mm = 3500.0

        self.selected_object = None

        self.last_frame = {
            "fps": 0.0, "pts": 0, "calc_us": 0, "heap": 0, "crc_err": 0,
            "walls": [], "candles": [], "cloud": []
        }

        self.model = StableFieldModel()
        self.localizer = RobotLocalizer()
        self.cluster_detector = ClusterDetector(merge_dist_mm=250.0, min_pts=4, max_clusters=4)
        self.show_boxes = True
        self.show_legacy_detection = False

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
        # Nastavení výchozího 180° režimu do ESP32
        self.send_cmd(f"FOV 1")
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
        """Vždy při změně parametrů (FoV/dosah/intenzita) se vygeneruje čistá nová mapa."""
        self.model.reset()
        self.localizer.unlock_map()
        self.send_cmd("RESET_MAP")
        print("[MAPA] Parametry upraveny -> stabilní mapa i lokalizace resetovány.")

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
            self.model.update(newest.get("candles", []), newest.get("walls", []))
            # Lokalizace: odhad polohy robota v reálném čase z čerstvých dat senzoru
            if self.localizer.locked:
                self.localizer.estimate_pose(newest.get("candles", []), newest.get("walls", []))

    def handle_click(self, pos):
        if pos[0] >= PANEL_W:
            return  # Klik v pravém panelu neprovádí inspekci bodů radaru
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

    def draw_panel_decorations(self, offset_x, is_right_panel=False):
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

        # Obrys hřiště 2400 x 3500 mm
        hw = 2400 / 2.0
        p1 = self.to_screen(-hw, 0, offset_x)
        p2 = self.to_screen( hw, 0, offset_x)
        p3 = self.to_screen( hw, 3500, offset_x)
        p4 = self.to_screen(-hw, 3500, offset_x)
        pygame.draw.polygon(self.screen, (28, 44, 34), [p1, p2, p3, p4], 1)

        if not is_right_panel:
            # LEVÝ RADAR: Robot pevně ve středu se směrovou šipkou dopředu
            pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 8)
            pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 35), 3)
        else:
            # PRAVÁ GLOBÁLNÍ MAPA: Bod START (0,0) – výchozí pozice robota
            pygame.draw.circle(self.screen, (0, 180, 220), (cx, cy), 7, 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx - 5, cy), (cx + 5, cy), 1)
            pygame.draw.line(self.screen, (0, 220, 255), (cx, cy - 5), (cx, cy + 5), 1)
            lbl_start = self.font_main.render("START (0,0)", True, (0, 220, 255))
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
        self.draw_panel_decorations(0, is_right_panel=False)
        scale = (PANEL_W / 2.0) / self.view_span_mm

        # (Záměrný sektor zakomentován - fokus pouze na bounding boxy)
        # self.draw_aiming_sector(0)

        # Body mračna (čistý živý radar)
        cloud = self.last_frame.get("cloud", [])
        n = len(cloud) // 3
        for i in range(n):
            x = cloud[i * 3]
            y = cloud[i * 3 + 1]
            intensity = cloud[i * 3 + 2]
            sx, sy = self.to_screen(x, y, 0)
            if 0 <= sx < PANEL_W and 0 <= sy < PANEL_H:
                v = min(255, max(40, intensity * 2 + 50))
                self.screen.set_at((sx, sy), (v, v, v))

        # Bounding Boxy přijaté přímo z ESP32 (výpočet probíhá na mikrokontroléru)
        raw_boxes = self.last_frame.get("boxes", [])
        targets = []
        for b in raw_boxes:
            if len(b) >= 12:
                targets.append({
                    'min_x': b[0], 'max_x': b[1],
                    'min_y': b[2], 'max_y': b[3],
                    'cx': b[4], 'cy': b[5],
                    'width': b[6], 'height': b[7],
                    'span_mm': b[8],
                    'dist_mm': b[9],
                    'pts_count': b[10],
                    'is_solo': bool(b[11]),
                    'type': "SVÍČKA" if b[11] else "SVÍČKA+STĚNA"
                })

        if self.show_boxes:
            for idx, cl in enumerate(targets):
                p_min = self.to_screen(cl['min_x'], cl['max_y'], 0)
                p_max = self.to_screen(cl['max_x'], cl['min_y'], 0)
                rx = min(p_min[0], p_max[0])
                ry = min(p_min[1], p_max[1])
                rw = max(8, abs(p_max[0] - p_min[0]))
                rh = max(8, abs(p_max[1] - p_min[1]))

                # Rámeček Bounding Boxu
                box_col = (0, 240, 160) if cl.get('type') == 'SVÍČKA+STĚNA' else (255, 180, 40)
                pygame.draw.rect(self.screen, box_col, (rx, ry, rw, rh), 2)

                # STŘEDOVÝ BOD (těžiště)
                scx, scy = self.to_screen(cl['cx'], cl['cy'], 0)
                pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 5)
                pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)
                pygame.draw.line(self.screen, (255, 255, 255), (scx - 7, scy), (scx + 7, scy), 2)
                pygame.draw.line(self.screen, (255, 255, 255), (scx, scy - 7), (scx, scy + 7), 2)

                # Popisky u boxu
                target_type = cl.get('type', 'CÍL')
                lbl1 = self.font_bold.render(f"BOX #{idx+1} [{target_type}]: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
                lbl2 = self.font_main.render(f"STŘED: [{int(cl['cx'])}, {int(cl['cy'])}] | R:{int(cl['dist_mm'])}mm ({cl['pts_count']}b)", True, (255, 240, 130))
                self.screen.blit(lbl1, (rx + 4, max(4, ry - 18)))
                self.screen.blit(lbl2, (scx + 9, scy - 8))

        # ============================================================
        # STARÁ DETEKCE STĚN A SVÍČEK KOMPLETNĚ ZAKOMENTOVÁNA:
        # ============================================================
        # for w in self.last_frame.get("walls", []):
        #     self.draw_wall_corridor(w[0], w[1], w[2], w[3], w[4], 0)
        #     p1 = self.to_screen(w[0], w[1], 0)
        #     p2 = self.to_screen(w[2], w[3], 0)
        #     pygame.draw.line(self.screen, (0, 255, 120), p1, p2, 4)
        #     ...
        # for c in self.last_frame.get("candles", []):
        #     ...

        # ============================================================
        # 1. TELEMETRICKÝ PANEL (V LEVÉM HORNÍM ROHU)
        # ============================================================
        panel = pygame.Surface((340, 220))
        panel.set_alpha(210)
        panel.fill((10, 12, 16))
        self.screen.blit(panel, (15, 15))

        f = self.last_frame
        fps      = f.get("fps", 0.0)
        pts      = f.get("pts", 0)
        calc_us  = f.get("calc_us", 0)
        heap     = f.get("heap", 0)
        crc_err  = f.get("crc_err", 0)

        y = 25
        title = self.font_big.render("ESP32 D200 RADAR & BOXY", True, ACCENT_COLOR)
        self.screen.blit(title, (25, y)); y += 26

        def row(label, val, col=(200, 210, 225)):
            nonlocal y
            t1 = self.font_main.render(label, True, (130, 140, 155))
            t2 = self.font_bold.render(str(val), True, col)
            self.screen.blit(t1, (25, y))
            self.screen.blit(t2, (180, y))
            y += 20

        row("Frekvence skenu:", f"{fps:.1f} Hz", (100, 255, 120) if fps > 4.5 else (255, 150, 0))
        row("Filtrované body:", f"{pts} bodů")
        row("Výpočet na ESP32:", f"{calc_us} µs ({calc_us/1000.0:.2f} ms)", (0, 220, 255))
        row("Volná RAM ESP32:", f"{heap // 1024} kB")
        row("CRC8 Chyby:", f"{crc_err}", (255, 80, 80) if crc_err > 0 else (120, 200, 120))
        row("Detekované cíle:", f"{len(targets)} / 4 (max 4)", (0, 240, 160) if len(targets) > 0 else (130, 140, 155))

        # ============================================================
        # 2. PANEL OVLÁDÁNÍ (V LEVÉM DOLNÍM ROHU)
        # ============================================================
        cmd_panel = pygame.Surface((440, 120))
        cmd_panel.set_alpha(200)
        cmd_panel.fill((10, 12, 16))
        self.screen.blit(cmd_panel, (15, PANEL_H - 135))

        cy = PANEL_H - 128
        h_title = self.font_bold.render("LADĚNÍ ZA BĚHU (PŘÍKAZY DO ESP32):", True, (240, 200, 80))
        self.screen.blit(h_title, (25, cy)); cy += 22

        fov_txt = "[F] Režim: 180° Kužel" if self.fov_limited else "[F] Režim: 360° Celé pole"
        freeze_txt = "[MEZERNÍK] ODMRAZIT" if self.frozen else "[MEZERNÍK] ZMRAZIT SKEN"
        box_txt = "[B] BOXY: ZAP" if self.show_boxes else "[B] BOXY: VYP"
        
        c1 = self.font_main.render(f"{fov_txt} | {freeze_txt} | {box_txt}", True, (220, 220, 220))
        c2 = self.font_main.render(f"Rozsah: [R/E] Min: {self.min_range} mm | [T/G] Max: {self.max_range} mm", True, (170, 180, 195))
        c3 = self.font_main.render(f"Odrazivost: [I/K] Min Intenzita: {self.min_intensity}", True, (170, 180, 195))
        self.screen.blit(c1, (25, cy)); cy += 18
        self.screen.blit(c2, (25, cy)); cy += 18
        self.screen.blit(c3, (25, cy))

        # Info o vybraném bodě po kliknutí
        if self.selected_object:
            sel_panel = pygame.Surface((320, 70))
            sel_panel.set_alpha(220)
            sel_panel.fill((25, 20, 10))
            self.screen.blit(sel_panel, (PANEL_W - 335, 15))
            pygame.draw.rect(self.screen, (255, 180, 0), (PANEL_W - 335, 15, 320, 70), 1)

            st = self.font_bold.render("INSPEKCE VYBRANÉHO BODU:", True, (255, 200, 50))
            self.screen.blit(st, (PANEL_W - 325, 22))
            sx_mm, sy_mm, s_dist = self.selected_object
            s_info = self.font_main.render(f"X: {sx_mm:.0f} mm, Y: {sy_mm:.0f} mm | R: {s_dist:.0f} mm", True, (255, 255, 255))
            self.screen.blit(s_info, (PANEL_W - 325, 45))

    def draw_right_stable(self):
        self.draw_panel_decorations(PANEL_W, is_right_panel=True)
        scale = (PANEL_W / 2.0) / self.view_span_mm
        loc = self.localizer

        # Dlouhodobé statické Bounding Boxy přijaté z ESP32:
        raw_static = self.last_frame.get("static_boxes", [])
        static_boxes = []
        for b in raw_static:
            if len(b) >= 12:
                static_boxes.append({
                    'min_x': b[0], 'max_x': b[1],
                    'min_y': b[2], 'max_y': b[3],
                    'cx': b[4], 'cy': b[5],
                    'width': b[6], 'height': b[7],
                    'span_mm': b[8],
                    'dist_mm': b[9],
                    'pts_count': b[10],
                    'is_solo': bool(b[11]),
                    'type': "SVÍČKA" if b[11] else "SVÍČKA+STĚNA"
                })

        # Vykreslení potvrzených dlouhodobých Bounding Boxů
        for idx, cl in enumerate(static_boxes):
            p_min = self.to_screen(cl['min_x'], cl['max_y'], PANEL_W)
            p_max = self.to_screen(cl['max_x'], cl['min_y'], PANEL_W)
            rx = min(p_min[0], p_max[0])
            ry = min(p_min[1], p_max[1])
            rw = max(10, abs(p_max[0] - p_min[0]))
            rh = max(10, abs(p_max[1] - p_min[1]))

            # Barva podle typu (oranžová svíčka, tyrkysová svíčka+stěna)
            box_col = (0, 240, 160) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 180, 40)
            
            # Poloprůhledná výplň boxu pro lepší viditelnost
            s_fill = pygame.Surface((rw, rh), pygame.SRCALPHA)
            fill_col = (0, 200, 140, 35) if cl['type'] == 'SVÍČKA+STĚNA' else (255, 160, 30, 35)
            s_fill.fill(fill_col)
            self.screen.blit(s_fill, (rx, ry))

            # Rámeček
            pygame.draw.rect(self.screen, box_col, (rx, ry, rw, rh), 2)

            # Spojnice od výchozího bodu LiDARu (0,0) ke středu boxu
            scx, scy = self.to_screen(cl['cx'], cl['cy'], PANEL_W)
            origin_x, origin_y = self.to_screen(0, 0, PANEL_W)
            pygame.draw.line(self.screen, (70, 90, 115), (origin_x, origin_y), (scx, scy), 1)

            # STŘEDOVÝ BOD [X, Y]
            pygame.draw.circle(self.screen, (255, 230, 20), (scx, scy), 6)
            pygame.draw.circle(self.screen, (20, 20, 20), (scx, scy), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (scx - 8, scy), (scx + 8, scy), 2)
            pygame.draw.line(self.screen, (255, 255, 255), (scx, scy - 8), (scx, scy + 8), 2)

            # Zvýrazněné popisy s přesnými souřadnicemi [X, Y] a vzdáleností od startu
            lbl_type = self.font_bold.render(f"BOX #{idx+1} [{cl['type']}]: {int(cl['width'])}x{int(cl['height'])} mm", True, box_col)
            lbl_pos = self.font_bold.render(f"POZICE: X={int(cl['cx']):+d} mm, Y={int(cl['cy']):+d} mm | R={int(cl['dist_mm'])} mm", True, (255, 245, 140))
            self.screen.blit(lbl_type, (rx + 4, max(4, ry - 20)))
            self.screen.blit(lbl_pos, (scx + 10, scy - 8))

        # Živé detekce přetransformované do souřadnic hřiště (fialovo-růžové body potvrzující shodu)
        if loc.locked and self.last_frame:
            h_rad = math.radians(loc.robot_deg)
            c_h = math.cos(h_rad)
            s_h = math.sin(h_rad)
            for lc in self.last_frame.get("candles", []):
                cx, cy = float(lc[0]), float(lc[1])
                wx = loc.robot_x + cx * c_h - cy * s_h
                wy = loc.robot_y + cx * s_h + cy * c_h
                lsx, lsy = self.to_screen(wx, wy, PANEL_W)
                pygame.draw.circle(self.screen, (255, 120, 220), (lsx, lsy), 6, 1)
                pygame.draw.circle(self.screen, (255, 240, 255), (lsx, lsy), 2)

        # ============================================================
        # POHYBLIVÝ RŮŽOVÝ ROBOT (ZOBRAZEN HNED OD ZAČÁTKU)
        # ============================================================
        PINK_MAIN = (255, 70, 180)
        PINK_BG   = (65, 15, 45)
        PINK_LITE = (255, 150, 215)

        # 1. Trajektorie pohybu (projetá trasa od startu)
        if len(loc.trajectory) >= 2:
            traj_px = [self.to_screen(tp[0], tp[1], PANEL_W) for tp in loc.trajectory]
            pygame.draw.lines(self.screen, (220, 50, 140), False, traj_px, 2)
            for tpt in traj_px[::4]:
                pygame.draw.circle(self.screen, (255, 100, 180), tpt, 2)

        # 2. Souřadnice a rotace robota
        rx, ry = self.to_screen(loc.robot_x, loc.robot_y, PANEL_W)
        heading_rad = math.radians(loc.robot_deg)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        # 3. Zorný vějíř LiDARu dopředu (průhledný kužel ukazující kam se dívá senzor)
        cone_r_mm = 650.0
        cone_pts = [(rx, ry)]
        for deg_off in range(-20, 21, 5):
            a_rad = heading_rad + math.radians(deg_off)
            kx = loc.robot_x + cone_r_mm * math.sin(a_rad)
            ky = loc.robot_y + cone_r_mm * math.cos(a_rad)
            cone_pts.append(self.to_screen(kx, ky, PANEL_W))
        if len(cone_pts) > 2:
            s_cone = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
            cone_local = [(pt[0] - PANEL_W, pt[1]) for pt in cone_pts]
            pygame.draw.polygon(s_cone, (255, 60, 170, 25), cone_local)
            pygame.draw.lines(s_cone, (255, 80, 190, 80), False, cone_local, 1)
            self.screen.blit(s_cone, (PANEL_W, 0))

        # 4. Tělo robota: Růžový čtverec / šasi (240 x 240 mm v reálném měřítku hřiště)
        hw_mm = 120.0
        local_box = [(-hw_mm, -hw_mm), (hw_mm, -hw_mm), (hw_mm, hw_mm), (-hw_mm, hw_mm)]
        box_px = []
        for (bx, by) in local_box:
            # Otočení v rovině
            gx = loc.robot_x + bx * cos_h + by * sin_h
            gy = loc.robot_y - bx * sin_h + by * cos_h
            box_px.append(self.to_screen(gx, gy, PANEL_W))

        pygame.draw.polygon(self.screen, PINK_BG, box_px, 0)
        pygame.draw.polygon(self.screen, PINK_MAIN, box_px, 2)

        # 5. Výrazná růžová šipka pohledu dopředu
        arrow_len = 26
        tip_x = int(rx + arrow_len * sin_h)
        tip_y = int(ry - arrow_len * cos_h)
        pygame.draw.line(self.screen, PINK_MAIN, (rx, ry), (tip_x, tip_y), 3)

        # Křídla šipky
        wing_len = 8
        wing_rad = math.radians(145)
        for sgn in [-1, 1]:
            w_ang = heading_rad + sgn * wing_rad
            wx2 = int(tip_x + wing_len * math.sin(w_ang))
            wy2 = int(tip_y - wing_len * math.cos(w_ang))
            pygame.draw.line(self.screen, PINK_MAIN, (tip_x, tip_y), (wx2, wy2), 2)

        # 6. Malý LiDAR senzor uprostřed těla
        pygame.draw.circle(self.screen, PINK_LITE, (rx, ry), 5)
        pygame.draw.circle(self.screen, (255, 255, 255), (rx, ry), 2)

        # 7. Textový popisek u robota
        conf_pct = int(loc.pose_confidence * 100)
        d_from_start = math.hypot(loc.robot_x, loc.robot_y)
        stat_tag = " [KLID]" if loc.is_stationary else " [POHYB]"
        lbl_rob = self.font_bold.render(
            f"ROBOT ({loc.robot_x:+.0f}, {loc.robot_y:+.0f}mm) {loc.robot_deg:+.0f}°{stat_tag} [{conf_pct}%]",
            True, PINK_LITE if loc.locked else (190, 150, 180))
        lbl_x = min(rx + 18, PANEL_W * 2 - lbl_rob.get_width() - 8)
        self.screen.blit(lbl_rob, (lbl_x, ry - 10))

        # ============================================================
        # HORNÍ STAVOVÝ PANEL
        # ============================================================
        stat_box = pygame.Surface((490, 56))
        stat_box.set_alpha(220)
        stat_box.fill((10, 12, 16))
        self.screen.blit(stat_box, (PANEL_W + 15, 15))
        pygame.draw.rect(self.screen, (80, 255, 160), (PANEL_W + 15, 15, 490, 56), 1)

        t_stat = self.font_big.render("DLOUHODOBÁ STATICKÁ MAPA (ESP32)", True, (80, 255, 160))
        self.screen.blit(t_stat, (PANEL_W + 25, 20))
        sub_str = f"Potvrzené cíle: {len(static_boxes)} / 4 | Výpočet: ESP32 EMA akumulace"
        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (PANEL_W + 25, 38))

        # ============================================================
        # SPODNÍ NÁPOVĚDA
        # ============================================================
        tip_box = pygame.Surface((440, 95))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (PANEL_W + 15, PANEL_H - 110))
        pygame.draw.rect(self.screen, (60, 50, 30), (PANEL_W + 15, PANEL_H - 110, 440, 95), 1)
        self.screen.blit(self.font_bold.render("STATICKÁ MAPA & SOUŘADNÝ SYSTÉM:", True, ACCENT_MAP), (PANEL_W + 25, PANEL_H - 102))
        self.screen.blit(self.font_main.render("• Výchozí bod START (0,0) = pozice robota a senzoru", True, (0, 220, 255)), (PANEL_W + 25, PANEL_H - 82))
        self.screen.blit(self.font_main.render("• Osy X (vpravo/vlevo) a Y (dopředu) v reálných mm", True, (240, 230, 130)), (PANEL_W + 25, PANEL_H - 64))
        self.screen.blit(self.font_main.render("• [C] / [M] Vymazat a znovu akumulovat mapu na ESP32", True, (160, 170, 185)), (PANEL_W + 25, PANEL_H - 46))



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
                    elif event.key == pygame.K_b:
                        self.show_boxes = not self.show_boxes
                        print(f"[BOX] Zobrazení Bounding Boxů: {'ZAPNUTO' if self.show_boxes else 'VYPNUTO'}")
                    elif event.key == pygame.K_v:
                        self.show_legacy_detection = not self.show_legacy_detection
                        print(f"[DET] Stará detekce stěn a svíček: {'ZAPNUTA' if self.show_legacy_detection else 'VYPNUTA'}")
                    elif event.key == pygame.K_l:
                        # ZAMKNUTÍ MAPY → spuštění lokalizace
                        cc = self.model.get_confirmed_candles()
                        cw = self.model.get_confirmed_walls()
                        ok = self.localizer.lock_map(cc, cw)
                        n_lm = len(cc) + len(cw)
                        if ok:
                            print(f"[LOC] Mapa zamčena! {n_lm} landmarků uloženo jako reference.")
                        else:
                            print(f"[LOC] Nedostatek landmarků ({n_lm}) – nejdřív nechte stabilizovat mapu.")

            self.read_serial()

            self.screen.fill(BG_COLOR)
            self.draw_left_live()
            self.draw_right_stable()

            # Vertikální předěl mezi panely
            pygame.draw.line(self.screen, (40, 48, 60), (PANEL_W, 0), (PANEL_W, WIN_H), 2)

            pygame.display.flip()
            self.clock.tick(60)

        if self.ser: self.ser.close()
        pygame.quit()

if __name__ == "__main__":
    port = get_serial_port()
    baud = get_baudrate()
    app = DualLidarVisualizer(port, baud)
    app.run()
