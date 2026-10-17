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
#  Pokročilý lokalizátor robota (Hybrid Candle Constellation + RANSAC)
# ============================================================
class RobotLocalizer:
    """
    Vysoce robustní lokalizátor robota na hřišti:
    1. Primární kotvy = SVÍČKY (Candle Constellation Matching):
       Svíčky jsou válce o průměru 5 cm – jejich střed je neměnný nezávisle na úhlu pohledu.
       Vzdálenosti mezi všemi dvojicemi svíček tvoří unikátní geometrický otisk (fingerprint).
       Jakmile robot vidí alespoň 2 svíčky, jejich vzájemná vzdálenost okamžitě a jednoznačně
       určí jejich identitu a z nich analyticky polohu (x, y) i orientaci (theta).
    2. Sekundární kotvy = STĚNY (Type-Aware RANSAC + Procrustes):
       Pokud robot vidí méně než 2 svíčky, použije se osvědčený RANSAC, ale s typovou kontrolou
       (svíčka se páruje výhradně se svíčkou, stěna se stěnou) a kontrolou geometrické vzdálenosti.
    3. Analytické zpřesnění (Procrustes Least-Squares):
       Ze všech potvrzených inlierů se spočítá optimální tuhý 2D transform bez jakéhokoliv driftu.
    4. Časový filtr (EMA + Hold) a záznam ujeté trajektorie robota.
    """

    def __init__(self):
        self.locked           = False
        self.ref_candles      = []   # Uložené referenční svíčky ve světových souřadnicích
        self.ref_walls        = []   # Uložené referenční stěny ve světových souřadnicích
        self.candle_dists     = []   # [(i, j, dist_mm)] pro všechny dvojice referenčních svíček
        self.wall_dists       = []   # [(i, j, dist_mm)] pro středy stěn

        self.robot_x          = 0.0  # mm ve světových souřadnicích (start = [0,0])
        self.robot_y          = 0.0
        self.robot_deg        = 0.0  # 0° = robot se dívá ve směru světového Y+
        self.pose_confidence  = 0.0  # 0.0 – 1.0
        self.trajectory       = [(0.0, 0.0)] # Ujetá trasa od startu

    def lock_map(self, conf_candles, conf_walls):
        """Uloží aktuální potvrzené svíčky a stěny jako fixní referenční svět."""
        self.ref_candles = [dict(c) for c in conf_candles]
        self.ref_walls   = [dict(w) for w in conf_walls]
        self.candle_dists = []
        self.wall_dists   = []

        # Vzdálenosti mezi všemi dvojicemi svíček
        for i in range(len(self.ref_candles)):
            for j in range(i + 1, len(self.ref_candles)):
                d = math.hypot(self.ref_candles[i]['x'] - self.ref_candles[j]['x'],
                               self.ref_candles[i]['y'] - self.ref_candles[j]['y'])
                self.candle_dists.append((i, j, d))

        # Vzdálenosti mezi středy stěn
        for i in range(len(self.ref_walls)):
            for j in range(i + 1, len(self.ref_walls)):
                d = math.hypot(self.ref_walls[i]['mx'] - self.ref_walls[j]['mx'],
                               self.ref_walls[i]['my'] - self.ref_walls[j]['my'])
                self.wall_dists.append((i, j, d))

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 1.0
        self.trajectory = [(0.0, 0.0)]

        # Vyžadujeme alespoň 1 svíčku + 1 stěnu nebo 2 stěny
        self.locked = (len(self.ref_candles) + len(self.ref_walls)) >= 2
        return self.locked

    def unlock_map(self):
        """Odemkne mapu a vrátí robota na start."""
        self.locked = False
        self.ref_candles = []
        self.ref_walls = []
        self.candle_dists = []
        self.wall_dists = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_deg = 0.0
        self.pose_confidence = 0.0
        self.trajectory = [(0.0, 0.0)]

    @staticmethod
    def _rigid_transform_2d(src, dst):
        """Analytický 2D Procrustes fit (rotace + posun) ze seznamu dvojic bodů."""
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
        Odhad polohy robota v reálném čase.
        live_candles: pole svíček ze senzoru [ [x, y, ...], ... ]
        live_walls:   pole stěn ze senzoru   [ [x1, y1, x2, y2, len, ...], ... ]
        """
        if not self.locked or (len(self.ref_candles) + len(self.ref_walls)) < 2:
            return False

        # Získání bodů ze senzoru (sensor frame)
        cur_cands = [(c[0], c[1]) for c in live_candles]
        cur_walls = [((w[0]+w[2])*0.5, (w[1]+w[3])*0.5) for w in live_walls]

        best_tx = None
        best_ty = None
        best_theta = None
        best_quality = 0.0

        # ========================================================
        # METODA 1: Geometrická konstelace svíček (přesnost na mm)
        # ========================================================
        if len(cur_cands) >= 2 and len(self.ref_candles) >= 2:
            cand_matches = []
            for a in range(len(cur_cands)):
                for b in range(a + 1, len(cur_cands)):
                    d_cur = math.hypot(cur_cands[a][0] - cur_cands[b][0],
                                       cur_cands[a][1] - cur_cands[b][1])

                    # Hledáme odpovídající dvojici v referenci
                    for (ri, rj, d_ref) in self.candle_dists:
                        if abs(d_cur - d_ref) < 85.0:  # Tolerance do 85 mm na rozteč svíček
                            pA = cur_cands[a]; pB = cur_cands[b]
                            rI = (self.ref_candles[ri]['x'], self.ref_candles[ri]['y'])
                            rJ = (self.ref_candles[rj]['x'], self.ref_candles[rj]['y'])

                            # Vyzkoušíme obě orientace (A->I, B->J) i (A->J, B->I)
                            for (c1, c2, ref1, ref2) in [(pA, pB, rI, rJ), (pA, pB, rJ, rI)]:
                                tx, ty, th = self._rigid_transform_2d([c1, c2], [ref1, ref2])
                                cos_t = math.cos(th); sin_t = math.sin(th)

                                # Ověříme počet inlierů napříč všemi svíčkami
                                inliers = 0
                                err_sum = 0.0
                                for cp in cur_cands:
                                    wx = cp[0]*cos_t - cp[1]*sin_t + tx
                                    wy = cp[0]*sin_t + cp[1]*cos_t + ty
                                    # Nejbližší ref svíčka
                                    min_d = min(math.hypot(wx - rc['x'], wy - rc['y']) for rc in self.ref_candles)
                                    if min_d < 160.0:
                                        inliers += 1
                                        err_sum += min_d

                                if inliers >= 2:
                                    quality = (inliers / len(self.ref_candles)) * max(0.1, 1.0 - (err_sum / inliers) / 160.0)
                                    if quality > best_quality:
                                        best_quality = quality
                                        best_tx = tx
                                        best_ty = ty
                                        best_theta = th

        # ========================================================
        # METODA 2: Typově uvědomělý RANSAC + Procrustes zpřesnění
        # (Pokud metoda 1 nenašla shodu – např. vidíme jen 1 svíčku a stěny)
        # ========================================================
        if best_quality < 0.35:
            # Sestavíme typované body
            cur_pts = [(c[0], c[1], 'c') for c in cur_cands]
            cur_pts += [(w[0], w[1], 'w') for w in cur_walls]

            ref_pts = [(rc['x'], rc['y'], 'c') for rc in self.ref_candles]
            ref_pts += [(rw['mx'], rw['my'], 'w') for rw in self.ref_walls]

            n_cur = len(cur_pts)
            n_ref = len(ref_pts)

            if n_cur >= 2 and n_ref >= 2:
                MAX_C = min(n_cur, 8)
                for i in range(MAX_C):
                    for j in range(i + 1, MAX_C):
                        d_cur = math.hypot(cur_pts[i][0] - cur_pts[j][0],
                                           cur_pts[i][1] - cur_pts[j][1])
                        if d_cur < 200.0: continue # Příliš blízko u sebe

                        for ri in range(n_ref):
                            if cur_pts[i][2] != ref_pts[ri][2]: continue # Typ musí sedět (svíčka-svíčka, stěna-stěna)
                            for rj in range(n_ref):
                                if rj == ri: continue
                                if cur_pts[j][2] != ref_pts[rj][2]: continue

                                d_ref = math.hypot(ref_pts[ri][0] - ref_pts[rj][0],
                                                   ref_pts[ri][1] - ref_pts[rj][1])
                                if abs(d_cur - d_ref) > 130.0: continue

                                # Spočítáme počáteční transformaci
                                p1 = (cur_pts[i][0], cur_pts[i][1])
                                p2 = (cur_pts[j][0], cur_pts[j][1])
                                r1 = (ref_pts[ri][0], ref_pts[ri][1])
                                r2 = (ref_pts[rj][0], ref_pts[rj][1])
                                tx, ty, th = self._rigid_transform_2d([p1, p2], [r1, r2])
                                cos_t = math.cos(th); sin_t = math.sin(th)

                                inlier_src = []
                                inlier_dst = []
                                total_err = 0.0

                                for cp in cur_pts:
                                    wx = cp[0]*cos_t - cp[1]*sin_t + tx
                                    wy = cp[0]*sin_t + cp[1]*cos_t + ty
                                    best_match_d = 9999.0
                                    best_rp = None
                                    for rp in ref_pts:
                                        if cp[2] == rp[2]: # stejný typ
                                            d = math.hypot(wx - rp[0], wy - rp[1])
                                            if d < best_match_d:
                                                best_match_d = d
                                                best_rp = rp

                                    if best_match_d < 180.0 and best_rp is not None:
                                        inlier_src.append((cp[0], cp[1]))
                                        inlier_dst.append((best_rp[0], best_rp[1]))
                                        total_err += best_match_d

                                if len(inlier_src) >= 2:
                                    quality = (len(inlier_src) / max(n_cur, n_ref)) * max(0.1, 1.0 - (total_err / len(inlier_src)) / 180.0)
                                    if quality > best_quality:
                                        # Procrustes zpřesnění ze VŠECH inlierů!
                                        ref_tx, ref_ty, ref_th = self._rigid_transform_2d(inlier_src, inlier_dst)
                                        best_quality = quality
                                        best_tx = ref_tx
                                        best_ty = ref_ty
                                        best_theta = ref_th

        # ========================================================
        # METODA 3: Časový filtr (EMA + Hold) a plynulý update
        # ========================================================
        if best_quality >= 0.25 and best_tx is not None:
            # Odhad byl úspěšný – hladce vyhladíme
            alpha = 0.40  # 40% čerstvý odhad, 60% historie pro eliminaci třesu

            self.robot_x = (1.0 - alpha) * self.robot_x + alpha * best_tx
            self.robot_y = (1.0 - alpha) * self.robot_y + alpha * best_ty

            # Hladké vyhlazení orientace přes sin/cos
            cur_th = math.radians(self.robot_deg)
            sin_blend = (1.0 - alpha) * math.sin(cur_th) + alpha * math.sin(best_theta)
            cos_blend = (1.0 - alpha) * math.cos(cur_th) + alpha * math.cos(best_theta)
            self.robot_deg = math.degrees(math.atan2(sin_blend, cos_blend))

            self.pose_confidence = min(1.0, 0.65 * self.pose_confidence + 0.35 * (best_quality + 0.3))

            # Ukládání bodů trajektorie (pokud se robot posunul aspoň o 25 mm)
            last_pt = self.trajectory[-1]
            if math.hypot(self.robot_x - last_pt[0], self.robot_y - last_pt[1]) > 25.0:
                self.trajectory.append((self.robot_x, self.robot_y))
                if len(self.trajectory) > 250:
                    self.trajectory.pop(0)

            return True
        else:
            # Ztráta signálu / zakrytí – robot drží pozici, jistota jemně klesá
            self.pose_confidence = max(0.0, self.pose_confidence - 0.04)
            return False


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

        self.candles = [c for c in self.candles if c['missed'] < 25 or (c['hits'] > 15 and c['missed'] < 80)]

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

        self.walls = [w for w in self.walls if w['missed'] < 20 or (w['hits'] > 12 and w['missed'] < 60)]

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
        # 1. PŮVODNÍ TELEMETRICKÝ PANEL (V LEVÉM HORNÍM ROHU)
        # ============================================================
        panel = pygame.Surface((340, 240))
        panel.set_alpha(210)
        panel.fill((10, 12, 16))
        self.screen.blit(panel, (15, 15))

        f = self.last_frame
        fps      = f.get("fps", 0.0)
        pts      = f.get("pts", 0)
        calc_us  = f.get("calc_us", 0)
        heap     = f.get("heap", 0)
        crc_err  = f.get("crc_err", 0)
        n_walls  = len(f.get("walls", []))
        n_cands  = len(f.get("candles", []))

        y = 25
        title = self.font_big.render("ESP32 D200 DETEKTOR", True, ACCENT_COLOR)
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
        row("Nalezené stěny:", f"{n_walls} (zeleně)", (0, 255, 100))
        row("Nalezené svíčky:", f"{n_cands} (červeně)", (255, 80, 80))

        # ============================================================
        # 2. PŮVODNÍ PANEL OVLÁDÁNÍ (V LEVÉM DOLNÍM ROHU) - PŘESNĚ DLE SCREENSHOTU
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
        
        c1 = self.font_main.render(f"{fov_txt} | {freeze_txt}", True, (220, 220, 220))
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

        # Získání objektů: po uzamčení bereme PEVNÝ referenční svět, před uzamčením živý model
        loc = self.localizer
        if loc.locked:
            conf_candles = loc.ref_candles
            conf_walls   = loc.ref_walls
        else:
            conf_candles = self.model.get_confirmed_candles()
            conf_walls   = self.model.get_confirmed_walls()

        # Stabilní stěny a fialový koridor
        for w in conf_walls:
            self.draw_wall_corridor(w['x1'], w['y1'], w['x2'], w['y2'], w['len'], PANEL_W)
            p1 = self.to_screen(w['x1'], w['y1'], PANEL_W)
            p2 = self.to_screen(w['x2'], w['y2'], PANEL_W)
            pygame.draw.line(self.screen, (80, 255, 140), p1, p2, 6)
            pygame.draw.circle(self.screen, (150, 255, 180), p1, 5)
            pygame.draw.circle(self.screen, (150, 255, 180), p2, 5)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            lbl = self.font_bold.render(f"STĚNA {int(w['len'])}mm", True, (80, 255, 140))
            self.screen.blit(lbl, (mx + 8, my - 8))

        # Stabilní svíčky
        target_cand = None
        for idx, c in enumerate(conf_candles):
            scx, scy = self.to_screen(c['x'], c['y'], PANEL_W)
            r_ring = int(250.0 * scale) # Zóna dosahu 25 cm kolem svíčky
            d = int(math.hypot(c['x'], c['y']))
            ang_deg = math.degrees(math.atan2(c['x'], c['y'])) if c['y'] != 0 else 0.0
            in_sector = (abs(ang_deg) <= 10.0 and c['y'] > 0)
            walls_cnt = c.get('walls_count', 0)
            wall_info = f"[{walls_cnt} stěn]" if walls_cnt > 0 else "[BEZ STĚNY]"

            if in_sector:
                if target_cand is None: target_cand = c
                pygame.draw.circle(self.screen, (0, 180, 220), (scx, scy), max(4, r_ring), 1)
                pygame.draw.circle(self.screen, (0, 255, 255), (scx, scy), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (scx, scy), 3)
                pygame.draw.line(self.screen, (0, 255, 255), (scx - 14, scy), (scx + 14, scy), 1)
                pygame.draw.line(self.screen, (0, 255, 255), (scx, scy - 14), (scx, scy + 14), 1)
                lbl = self.font_bold.render(f"★ 1. CÍL ({d}mm, {ang_deg:+.1f}°) {wall_info}", True, (0, 240, 255))
                self.screen.blit(lbl, (scx + 14, scy - 12))
            else:
                ring_color = (255, 140, 80) if walls_cnt > 0 else (255, 60, 60)
                pygame.draw.circle(self.screen, ring_color, (scx, scy), max(4, r_ring), 1)
                pygame.draw.circle(self.screen, (255, 40, 40), (scx, scy), 8)
                pygame.draw.circle(self.screen, (255, 240, 180), (scx, scy), 3)
                lbl_color = (255, 130, 130) if walls_cnt > 0 else (255, 80, 80)
                lbl = self.font_bold.render(f"SVÍČKA #{idx+1} ({d}mm) {wall_info}", True, lbl_color)
                self.screen.blit(lbl, (scx + 12, scy - 12))

        # Trasa mezi potvrzenými svíčkami (1. cíl je svíčka v záměrném výseku)
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
                p_from = self.to_screen(route[i-1][0], route[i-1][1], PANEL_W)
                p_to   = self.to_screen(route[i][0], route[i][1], PANEL_W)
                pygame.draw.line(self.screen, (255, 215, 0), p_from, p_to, 2)
                mid_x = (p_from[0] + p_to[0]) // 2
                mid_y = (p_from[1] + p_to[1]) // 2
                self.screen.blit(self.font_bold.render(f"-> {i}", True, (255, 230, 80)), (mid_x + 4, mid_y - 6))

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
        lbl_rob = self.font_bold.render(
            f"ROBOT ({loc.robot_x:+.0f}, {loc.robot_y:+.0f}mm) {loc.robot_deg:+.0f}° [{conf_pct}%]",
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
        border_col = (80, 255, 160) if loc.locked else (60, 50, 30)
        pygame.draw.rect(self.screen, border_col, (PANEL_W + 15, 15, 490, 56), 1)

        loc_status = " 🔒 LOKALIZACE AKTIVNÍ" if loc.locked else " [L] ZAMKNOUT MAPU"
        t_stat = self.font_big.render(f"MAPA HŘIŠTĚ{loc_status}", True,
                                      (80, 255, 160) if loc.locked else ACCENT_MAP)
        self.screen.blit(t_stat, (PANEL_W + 25, 20))

        if loc.locked:
            sub_str = f"Kotvy: {len(conf_candles)} svíček, {len(conf_walls)} stěn | Posun: {d_from_start:.0f} mm | Jistota: {conf_pct}%"
            # Barevný pruh jistoty
            bar_w = 460
            fill_w = int(bar_w * loc.pose_confidence)
            pygame.draw.rect(self.screen, (35, 20, 30), (PANEL_W + 25, 54, bar_w, 6))
            bar_col = (min(255, int(255 * (1.0 - loc.pose_confidence))),
                       min(255, int(80 + 175 * loc.pose_confidence)), 120)
            pygame.draw.rect(self.screen, bar_col, (PANEL_W + 25, 54, fill_w, 6))
        else:
            sub_str = f"Svíčky: {len(conf_candles)}/4 | Stěny: {len(conf_walls)}/6 | Skenů: {self.model.total_scans}"

        self.screen.blit(self.font_main.render(sub_str, True, (210, 215, 225)), (PANEL_W + 25, 38))

        # ============================================================
        # SPODNÍ NÁPOVĚDA
        # ============================================================
        tip_box = pygame.Surface((440, 95))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (PANEL_W + 15, PANEL_H - 110))
        pygame.draw.rect(self.screen, (60, 50, 30), (PANEL_W + 15, PANEL_H - 110, 440, 95), 1)
        self.screen.blit(self.font_bold.render("ORIENTACE A LOKALIZACE ROBOTA:", True, ACCENT_MAP), (PANEL_W + 25, PANEL_H - 102))
        self.screen.blit(self.font_main.render("• [L] Zamknout mapu: uloží svíčky jako pevné body hřiště", True, (80, 255, 160)), (PANEL_W + 25, PANEL_H - 82))
        self.screen.blit(self.font_main.render("• Růžový robot ukazuje aktuální polohu [X, Y] a směr senzoru", True, PINK_LITE), (PANEL_W + 25, PANEL_H - 64))
        self.screen.blit(self.font_main.render("• [C] Resetovat mapu | Bod START zůstává v počátku (0,0)", True, (160, 170, 185)), (PANEL_W + 25, PANEL_H - 46))



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
