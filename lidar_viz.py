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
#  Lokalizátor robota – 3 pevné kotvy (trilateration + Procrustes)
# ============================================================
class RobotLocalizer:
    """
    Vybere 3 nejstabilnější stěny jako pevné kotvy A, B, C.
    Uloží jejich vzájemné vzdálenosti d_AB, d_AC, d_BC jako "otisk" mapy.
    Při pohybu robota hledá v aktuálním skenu trojici stěn, jejichž vzájemné
    vzdálenosti nejlépe souhlasí s uloženým otiskem, a z korespondence
    A↔A', B↔B', C↔C' vypočítá tuhý transform (posun + rotace).

    Proč stěny (ne svíčky)?
    - Střed stěny (midpoint) je určen z mnoha LiDAR bodů → nižší šum
    - Stěny jsou velké objekty, nejsou přerušeny překážkami
    - Svíčky mohou být občas zakryty nebo zaměněny
    """

    def __init__(self):
        self.locked          = False
        # 3 referenční kotvy ve světovém souřadnicovém systému
        self.anchors         = []   # [{'x','y','label','hits'}]  – max 3
        # Vzdálenosti mezi kotvami: [(i,j,dist_mm), ...]
        self.anchor_dists    = []
        # Odhadovaná pozice robota ve světovém rámci
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.robot_deg       = 0.0   # 0° = robot se dívá ve směru Y+
        self.pose_confidence = 0.0   # 0.0 – 1.0
        self._history        = []    # EMA buffer posledních 8 odhadů

    # ------------------------------------------------------------------
    def lock_map(self, conf_candles, conf_walls):
        """
        Vybere 3 nejlepší kotvy z potvrzených stěn (fallback na svíčky)
        a uloží jejich světové souřadnice + vzájemné vzdálenosti.
        """
        self.locked = False
        self.anchors = []
        self.anchor_dists = []
        self._history = []
        self.robot_x = self.robot_y = self.robot_deg = 0.0
        self.pose_confidence = 0.0

        # Kotvy = top-3 stěny řazené dle počtu hitů (nejjistější stěny)
        sorted_walls = sorted(conf_walls, key=lambda w: w.get('hits', 0), reverse=True)
        labels = ['A', 'B', 'C']
        for w in sorted_walls[:3]:
            lbl = labels[len(self.anchors)]
            self.anchors.append({'x': w['mx'], 'y': w['my'],
                                 'label': lbl, 'hits': w.get('hits', 0)})

        # Pokud máme méně než 3 stěny, doplníme svíčkami
        if len(self.anchors) < 3:
            sorted_c = sorted(conf_candles, key=lambda c: c.get('hits', 0), reverse=True)
            for c in sorted_c:
                if len(self.anchors) >= 3: break
                lbl = labels[len(self.anchors)]
                self.anchors.append({'x': c['x'], 'y': c['y'],
                                     'label': lbl, 'hits': c.get('hits', 0)})

        if len(self.anchors) < 2:
            return False

        # Uložení vzájemných vzdáleností (fingerprint mapy)
        for i in range(len(self.anchors)):
            for j in range(i + 1, len(self.anchors)):
                d = math.hypot(self.anchors[i]['x'] - self.anchors[j]['x'],
                               self.anchors[i]['y'] - self.anchors[j]['y'])
                self.anchor_dists.append((i, j, d))

        self.locked = True
        return True

    # ------------------------------------------------------------------
    @staticmethod
    def _rigid_transform_2d(src, dst):
        """
        Vypočítá nejlepší tuhý 2D transform (rotace + translace) z N korespondencí
        src[i] → dst[i] pomocí Procrustes analýzy (uzavřený tvar pro 2D).
        Vrací (dx, dy, theta_rad).
        """
        n = len(src)
        if n == 0: return 0.0, 0.0, 0.0

        # Centroidy
        cx_s = sum(p[0] for p in src) / n
        cy_s = sum(p[1] for p in src) / n
        cx_d = sum(p[0] for p in dst) / n
        cy_d = sum(p[1] for p in dst) / n

        # Kovarianční matice H = Σ (src_c)^T (dst_c)  (2×2 → skaláry)
        h00 = h01 = h10 = h11 = 0.0
        for (sx, sy), (dx, dy) in zip(src, dst):
            scx = sx - cx_s; scy = sy - cy_s
            dcx = dx - cx_d; dcy = dy - cy_d
            h00 += scx * dcx; h01 += scx * dcy
            h10 += scy * dcx; h11 += scy * dcy

        # Pro 2D: sin(θ) = h10-h01, cos(θ) = h00+h11  (analytická SVD)
        theta = math.atan2(h10 - h01, h00 + h11)
        cos_t = math.cos(theta); sin_t = math.sin(theta)

        # Translace: dst_centroid = R * src_centroid + t
        tx = cx_d - (cx_s * cos_t - cy_s * sin_t)
        ty = cy_d - (cx_s * sin_t + cy_s * cos_t)
        return tx, ty, theta

    # ------------------------------------------------------------------
    def _match_to_anchors(self, cur_walls, cur_candles):
        """
        Hledá trojici aktuálních landmarků, jejichž vzájemné vzdálenosti
        nejlépe souhlasí s uloženým otiskem kotev.
        Vrací seznam korespondencí [(cur_pt, ref_anchor), ...] nebo None.
        """
        import itertools

        # Sestavíme seznam aktuálních landmarků (stěny mají přednost)
        cur_lm = [(w['mx'], w['my']) for w in sorted(cur_walls,
                   key=lambda w: w.get('hits', 0), reverse=True)]
        cur_lm += [(c['x'], c['y']) for c in sorted(cur_candles,
                    key=lambda c: c.get('hits', 0), reverse=True)]

        n_anc = len(self.anchors)
        if len(cur_lm) < n_anc: return None

        ref_pts = [(a['x'], a['y']) for a in self.anchors]

        best_err = 9999.0
        best_corr = None

        # Zkusíme všechny permutace n_anc aktuálních bodů mapovaných na kotvy
        for combo in itertools.combinations(range(len(cur_lm)), n_anc):
            for perm in itertools.permutations(combo):
                # Zkontrolujeme shodu vzdáleností (fingerprint matching)
                err = 0.0
                for (ai, aj, ref_d) in self.anchor_dists:
                    pi = cur_lm[perm[ai]]
                    pj = cur_lm[perm[aj]]
                    cur_d = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
                    err += abs(cur_d - ref_d)
                err /= max(1, len(self.anchor_dists))

                if err < best_err:
                    best_err = err
                    best_corr = [(cur_lm[perm[i]], ref_pts[i]) for i in range(n_anc)]

        # Přijmeme jen pokud průměrná vzdálenostní chyba < 300 mm
        if best_corr is None or best_err > 300.0:
            return None
        return best_corr, best_err

    # ------------------------------------------------------------------
    def estimate_pose(self, conf_candles, conf_walls):
        """Odhad polohy robota. Volá se každý frame po zamknutí mapy."""
        if not self.locked or len(self.anchors) < 2:
            return False

        result = self._match_to_anchors(conf_walls, conf_candles)
        if result is None:
            self.pose_confidence = max(0.0, self.pose_confidence - 0.05)
            return False

        corr, match_err = result
        src = [c[0] for c in corr]   # aktuální body (sensor frame)
        dst = [c[1] for c in corr]   # referenční kotvy (world frame)

        tx, ty, theta = self._rigid_transform_2d(src, dst)

        # Confidence: klesá s chybou fingerprint matchingu
        conf = max(0.0, min(1.0, 1.0 - match_err / 300.0))

        # EMA buffer pro hladký pohyb ikony robota
        self._history.append((tx, ty, theta, conf))
        if len(self._history) > 8:
            self._history.pop(0)

        w_sum = 0.0; wx = 0.0; wy = 0.0; wt_s = 0.0; wt_c = 0.0; wc = 0.0
        for (hx, hy, ht, hc) in self._history:
            w = max(0.01, hc)
            wx += hx * w; wy += hy * w; w_sum += w
            wt_s += math.sin(ht) * w; wt_c += math.cos(ht) * w
            wc += hc
        if w_sum > 0:
            self.robot_x   = wx / w_sum
            self.robot_y   = wy / w_sum
            self.robot_deg = math.degrees(math.atan2(wt_s / w_sum, wt_c / w_sum))
            self.pose_confidence = min(1.0, wc / len(self._history))
        return True


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
        print("[MAPA] Parametry upraveny -> stabilní mapa vyčištěna a generuje se nanovo.")

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
            # Lokalizace: odhad polohy robota na základě pevné referenční mapy
            if self.localizer.locked:
                cc = self.model.get_confirmed_candles()
                cw = self.model.get_confirmed_walls()
                self.localizer.estimate_pose(cc, cw)

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

    def draw_panel_decorations(self, offset_x):
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

        # Robot ve středu (modrý kruh se směrovou šipkou dopředu)
        pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 8)
        pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 35), 3)

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
        self.draw_panel_decorations(0)
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
        self.draw_panel_decorations(PANEL_W)
        scale = (PANEL_W / 2.0) / self.view_span_mm

        # Modrý záměrný výsek 20° (+-10° dopředu)
        self.draw_aiming_sector(PANEL_W)

        conf_candles = self.model.get_confirmed_candles()
        conf_walls = self.model.get_confirmed_walls()

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

            # Pokud je detekována svíčka v záměrném výseku, navštívíme ji VŽDY jako první
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
        # ============================================================
        # LOKALIZACE: Kotvy A/B/C + růžová šipka robota
        # ============================================================
        loc = self.localizer
        if loc.locked:
            conf = loc.pose_confidence
            PINK   = (255, 80, 180)
            CYAN   = (0, 220, 255)
            YELLOW = (255, 215, 0)
            ANCHOR_COLORS = [(255, 100, 60), (60, 200, 255), (120, 255, 100)]  # A=oranžová, B=modrá, C=zelená

            # ---- Kotvy A, B, C: zvýrazněné čtverce + popisek + vzdálenosti ----
            anchor_screen = []
            for i, anc in enumerate(loc.anchors):
                px, py = self.to_screen(anc['x'], anc['y'], PANEL_W)
                anchor_screen.append((px, py))
                ac = ANCHOR_COLORS[i % len(ANCHOR_COLORS)]
                # Čtverec kotvy
                pygame.draw.rect(self.screen, (10, 10, 20), (px-9, py-9, 18, 18))
                pygame.draw.rect(self.screen, ac, (px-9, py-9, 18, 18), 2)
                # Popisek A / B / C
                lbl = self.font_bold.render(anc['label'], True, ac)
                self.screen.blit(lbl, (px + 11, py - 8))

            # Vzdálenostní čáry mezi kotvami (trojúhelník)
            for (ai, aj, ref_d) in loc.anchor_dists:
                if ai < len(anchor_screen) and aj < len(anchor_screen):
                    pa = anchor_screen[ai]; pb = anchor_screen[aj]
                    pygame.draw.line(self.screen, (80, 80, 50), pa, pb, 1)
                    mid_x = (pa[0] + pb[0]) // 2
                    mid_y = (pa[1] + pb[1]) // 2
                    lbl_d = self.font_main.render(
                        f"{loc.anchors[ai]['label']}-{loc.anchors[aj]['label']}: {ref_d:.0f}mm",
                        True, (160, 155, 80))
                    self.screen.blit(lbl_d, (mid_x + 4, mid_y - 6))

            # ---- START: výchozí pozice robota = počátek světového souřadnicového systému ----
            ox, oy = self.to_screen(0.0, 0.0, PANEL_W)
            pygame.draw.circle(self.screen, (0, 60, 80), (ox, oy), 10)
            pygame.draw.circle(self.screen, CYAN, (ox, oy), 10, 2)
            pygame.draw.line(self.screen, CYAN, (ox-8, oy), (ox+8, oy), 1)
            pygame.draw.line(self.screen, CYAN, (ox, oy-8), (ox, oy+8), 1)
            lbl_start = self.font_main.render("START", True, CYAN)
            self.screen.blit(lbl_start, (ox + 12, oy - 7))

            # ---- Robot: růžová šipka na aktuální odhadované pozici ----
            rx, ry = self.to_screen(loc.robot_x, loc.robot_y, PANEL_W)
            heading_rad = math.radians(loc.robot_deg)

            # Šipka = plný trojúhelník (hlava vpředu)
            tip_len = 22    # délka od středu k hrotu
            side_len = 11   # polovina šířky základny
            back_len = 10   # vzdálenost od středu k základně

            def rot(x, y):
                c = math.cos(heading_rad); s = math.sin(heading_rad)
                return int(rx + x*s - y*c), int(ry - x*c - y*s)

            tip  = rot(0, tip_len)
            bl   = rot(-side_len, -back_len)
            br   = rot( side_len, -back_len)

            # Vyplněný trojúhelník s tmavou výplní
            pygame.draw.polygon(self.screen, (60, 10, 40), [tip, bl, br])
            pygame.draw.polygon(self.screen, PINK, [tip, bl, br], 2)

            # Malá tečka ve středu (středový bod robota)
            pygame.draw.circle(self.screen, PINK, (rx, ry), 3)

            # Tenká přímka od počátku k robotu (trajektorie pohybu)
            if math.hypot(loc.robot_x, loc.robot_y) > 50.0:
                pygame.draw.line(self.screen, (80, 30, 60), (ox, oy), (rx, ry), 1)

            # Popisek robota
            d_from_origin = math.hypot(loc.robot_x, loc.robot_y)
            conf_pct = int(conf * 100)
            lbl_rob = self.font_bold.render(
                f"ROBOT  ({loc.robot_x:+.0f}, {loc.robot_y:+.0f}) mm  {loc.robot_deg:+.0f}°  [{conf_pct}%]",
                True, PINK)
            lbl_rx = min(rx + 16, PANEL_W * 2 - lbl_rob.get_width() - 5)
            self.screen.blit(lbl_rob, (lbl_rx, ry - 10))

            # ---- Pruh jistoty lokalizace ----
            bar_x = PANEL_W + 15; bar_y = 68; bar_w = 490
            pygame.draw.rect(self.screen, (30, 15, 25), (bar_x, bar_y, bar_w, 7))
            fill_w = min(bar_w, max(0, int(bar_w * conf)))
            bar_col = (min(255, int(255*(1-conf))), min(255, int(55 + 200*conf)), 100)
            pygame.draw.rect(self.screen, bar_col, (bar_x, bar_y, fill_w, 7))
            anc_lbl = f"{len(loc.anchors)} kotvy | vzdálenost od startu: {d_from_origin:.0f} mm | jistota: {conf_pct}%"
            self.screen.blit(self.font_main.render(anc_lbl, True, (200, 200, 180)), (bar_x, bar_y + 9))

        # Horní panel stavu pravého okna
        stat_box = pygame.Surface((490, 52))
        stat_box.set_alpha(215)
        stat_box.fill((10, 12, 16))
        self.screen.blit(stat_box, (PANEL_W + 15, 15))
        pygame.draw.rect(self.screen, (60, 50, 30), (PANEL_W + 15, 15, 490, 52), 1)

        loc_status = "  🔒 LOKALIZACE AKTIVNÍ" if loc.locked else "  [L] Zamknout mapu"
        t_stat = self.font_big.render(f"STABILNÍ REKONSTRUKCE HŘIŠTĚ{loc_status}", True,
                                      (80, 255, 160) if loc.locked else ACCENT_MAP)
        self.screen.blit(t_stat, (PANEL_W + 25, 21))
        stat_str = f"Potvrzeno: {len(conf_candles)}/4 svíček | {len(conf_walls)}/6 stěn | Skenů: {self.model.total_scans}"
        self.screen.blit(self.font_main.render(stat_str, True, (210, 215, 225)), (PANEL_W + 25, 43))

        # Nápověda vpravo dole
        tip_box = pygame.Surface((440, 95))
        tip_box.set_alpha(210)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (PANEL_W + 15, PANEL_H - 110))
        pygame.draw.rect(self.screen, (60, 50, 30), (PANEL_W + 15, PANEL_H - 110, 440, 95), 1)
        self.screen.blit(self.font_bold.render("PRINCIP DLOUHODOBÉ MAPY:", True, ACCENT_MAP), (PANEL_W + 25, PANEL_H - 102))
        self.screen.blit(self.font_main.render("• Fialový koridor: Zóna stěny (+15 cm za roh), svíčka zde nesmí být", True, (210, 130, 255)), (PANEL_W + 25, PANEL_H - 82))
        self.screen.blit(self.font_main.render("• [L] Zamknout mapu → robot se lokalizuje vůči ní při pohybu", True, (80, 255, 160)), (PANEL_W + 25, PANEL_H - 64))
        self.screen.blit(self.font_main.render("• Při změně parametrů se mapa vyčistí | [C] Manuální reset", True, (160, 170, 185)), (PANEL_W + 25, PANEL_H - 46))


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
