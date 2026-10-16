#!/usr/bin/env python3
"""
LIDAR_SVICKY_27 - Dvoupanelový vizualizér (Živý radar + Stabilní perzistentní mapa)
Autor: Antigravity & Nik

Dva panely vedle sebe:
  1. LEVÝ PANEL: Živý radar (Live LIDAR scan) v reálném čase (6.7 Hz).
  2. PRAVÝ PANEL: Stabilní herní mapa (World Model).
     - Filtruje šum a pohybující se lidi.
     - Drží stabilní pozice svíček (max 4) a stěn (max 6).
     - Vyhlazuje a uzamyká neměnné herní pole.

Klávesy:
  C / M       - Resetovat stabilní mapu
  SPACE       - Zmrazit živý obraz
  F           - Přepnout FoV 360° / 180°
  R / E       - Min dosah (±50 mm)
  T / G       - Max dosah (±200 mm)
  I / K       - Min intenzita (±5)
  Q / ESC     - Konec
"""

import sys, os, time, json, math
import pygame
import serial
from serial.tools import list_ports

# Rozměry
PANEL_W     = 800
PANEL_H     = 820
WIN_W       = PANEL_W * 2
WIN_H       = PANEL_H
VIEW_SPAN   = 3200.0  # mm od středu

# Barvy
BG_COLOR    = (14, 16, 20)
GRID_COLOR  = (32, 36, 44)
TEXT_COLOR  = (220, 225, 230)
ACCENT_LIVE = (0, 210, 255)
ACCENT_MAP  = (255, 200, 60)

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
#  Trida pro akumulaci a stabilitu herniho pole (World Model)
# ============================================================
class StableFieldModel:
    def __init__(self):
        self.candles = []  # [{x, y, hits, missed, confirmed}]
        self.walls = []    # [{x1, y1, x2, y2, mx, my, len, hits, missed}]
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
            # lc = [x, y, diam]
            cx, cy = lc[0], lc[1]
            best_idx = -1
            best_dist = 150.0  # max posun svíčky
            for idx, sc in enumerate(self.candles):
                if idx in matched_c:
                    continue
                d = math.hypot(sc['x'] - cx, sc['y'] - cy)
                if d < best_dist:
                    best_dist = d
                    best_idx = idx

            if best_idx >= 0:
                sc = self.candles[best_idx]
                # EMA vyhlazení
                sc['x'] = 0.85 * sc['x'] + 0.15 * cx
                sc['y'] = 0.85 * sc['y'] + 0.15 * cy
                sc['hits'] += 1
                sc['missed'] = 0
                matched_c.add(best_idx)
            else:
                if len(self.candles) < 8:
                    new_idx = len(self.candles)
                    self.candles.append({
                        'x': cx, 'y': cy,
                        'hits': 1, 'missed': 0
                    })
                    matched_c.add(new_idx)

        for idx, sc in enumerate(self.candles):
            if idx not in matched_c:
                sc['missed'] += 1

        # Mazání svíček s velkým missed
        self.candles = [c for c in self.candles if c['missed'] < 25 or (c['hits'] > 15 and c['missed'] < 80)]

        # 2. STĚNY (párování podle středu a úhlu)
        matched_w = set()
        for lw in live_walls:
            # lw = [x1, y1, x2, y2, len]
            x1, y1, x2, y2 = lw[0], lw[1], lw[2], lw[3]
            mx = (x1 + x2) * 0.5
            my = (y1 + y2) * 0.5
            wlen = lw[4]

            best_idx = -1
            best_dist = 140.0
            for idx, sw in enumerate(self.walls):
                if idx in matched_w:
                    continue
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
        # Vrátí max 4 svíčky seřazené podle stability (hits)
        valid = [c for c in self.candles if c['hits'] >= 6]
        valid.sort(key=lambda c: c['hits'], reverse=True)
        return valid[:4]

    def get_confirmed_walls(self):
        # Vrátí max 6 stěn seřazených podle stability
        valid = [w for w in self.walls if w['hits'] >= 5]
        valid.sort(key=lambda w: w['hits'], reverse=True)
        return valid[:6]


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

        self.fov_limited = False
        self.min_range = 130
        self.max_range = 2600
        self.min_intensity = 15

        self.last_frame = {
            "fps": 0.0, "pts": 0, "calc_us": 0, "heap": 0, "crc_err": 0,
            "walls": [], "candles": [], "cloud": []
        }

        self.model = StableFieldModel()

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption(f"Kybernoid 27 - D200 Live Radar & Stable Field Map [{port}]")
        self.clock = pygame.time.Clock()

        try:
            self.font_main = pygame.font.SysFont("DejaVu Sans Mono", 13)
            self.font_bold = pygame.font.SysFont("DejaVu Sans Mono", 14, bold=True)
            self.font_big  = pygame.font.SysFont("DejaVu Sans Mono", 17, bold=True)
        except:
            self.font_main = pygame.font.SysFont(None, 16)
            self.font_bold = pygame.font.SysFont(None, 18)
            self.font_big  = pygame.font.SysFont(None, 22)

        self.connect_serial()

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

    def to_screen(self, x_mm, y_mm, panel_offset_x):
        cx = panel_offset_x + PANEL_W / 2.0
        cy = PANEL_H / 2.0 + 20
        scale = (PANEL_W / 2.0) / VIEW_SPAN
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

    def draw_panel_decorations(self, offset_x, title, accent_col):
        cx = offset_x + PANEL_W // 2
        cy = PANEL_H // 2 + 20
        scale = (PANEL_W / 2.0) / VIEW_SPAN

        # Rámeček panelu
        pygame.draw.rect(self.screen, (24, 28, 36), (offset_x + 2, 2, PANEL_W - 4, PANEL_H - 4), 1)

        # Mřížka a kruhy
        pygame.draw.line(self.screen, GRID_COLOR, (offset_x, cy), (offset_x + PANEL_W, cy), 1)
        pygame.draw.line(self.screen, GRID_COLOR, (cx, 0), (cx, PANEL_H), 1)

        for d_m in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
            r_px = int(d_m * 1000.0 * scale)
            c = (48, 56, 68) if d_m in [1.0, 2.0] else (26, 30, 38)
            pygame.draw.circle(self.screen, c, (cx, cy), r_px, 1)
            lbl = self.font_main.render(f"{d_m:.1f} m", True, (100, 110, 125))
            self.screen.blit(lbl, (cx + 5, cy - r_px - 8))

        # Obrys hřiště 2400 x 3500 mm
        hw = 2400 / 2.0
        p1 = self.to_screen(-hw, 0, offset_x)
        p2 = self.to_screen( hw, 0, offset_x)
        p3 = self.to_screen( hw, 3500, offset_x)
        p4 = self.to_screen(-hw, 3500, offset_x)
        pygame.draw.polygon(self.screen, (24, 40, 32), [p1, p2, p3, p4], 1)

        # Robot
        pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 7)
        pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 30), 3)

        # Nadpis panelu
        t_surf = self.font_big.render(title, True, accent_col)
        self.screen.blit(t_surf, (offset_x + 20, 18))

    def draw_left_live(self):
        self.draw_panel_decorations(0, "1. ZIVY LIDAR SKEN (REAL-TIME)", ACCENT_LIVE)
        scale = (PANEL_W / 2.0) / VIEW_SPAN

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

        # Živé stěny
        for w in self.last_frame.get("walls", []):
            p1 = self.to_screen(w[0], w[1], 0)
            p2 = self.to_screen(w[2], w[3], 0)
            pygame.draw.line(self.screen, (0, 255, 120), p1, p2, 4)
            pygame.draw.circle(self.screen, (100, 255, 160), p1, 3)
            pygame.draw.circle(self.screen, (100, 255, 160), p2, 3)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            lbl = self.font_main.render(f"{int(w[4])}mm", True, (0, 255, 120))
            self.screen.blit(lbl, (mx + 5, my - 6))

        # Živé svíčky
        for c in self.last_frame.get("candles", []):
            scx, scy = self.to_screen(c[0], c[1], 0)
            r_ring = int(200.0 * scale)
            pygame.draw.circle(self.screen, (255, 60, 60, 50), (scx, scy), max(4, r_ring), 1)
            pygame.draw.circle(self.screen, (255, 50, 50), (scx, scy), 6)
            pygame.draw.circle(self.screen, (255, 230, 200), (scx, scy), 2)
            d = int(math.hypot(c[0], c[1]))
            lbl = self.font_bold.render(f"SVICKA ({d}mm)", True, (255, 80, 80))
            self.screen.blit(lbl, (scx + 10, scy - 10))

        # Telemetrie panel
        f = self.last_frame
        fps = f.get("fps", 0.0)
        calc_us = f.get("calc_us", 0)
        pts = f.get("pts", 0)
        nw = len(f.get("walls", []))
        nc = len(f.get("candles", []))

        info_box = pygame.Surface((250, 95))
        info_box.set_alpha(200)
        info_box.fill((10, 12, 16))
        self.screen.blit(info_box, (15, PANEL_H - 110))
        pygame.draw.rect(self.screen, (40, 50, 65), (15, PANEL_H - 110, 250, 95), 1)

        self.screen.blit(self.font_main.render(f"FPS: {fps:.1f} Hz | Calc: {calc_us} µs", True, (0, 220, 255)), (25, PANEL_H - 102))
        self.screen.blit(self.font_main.render(f"Filtrovano: {pts} bodu", True, (180, 190, 205)), (25, PANEL_H - 84))
        self.screen.blit(self.font_bold.render(f"Zive steny: {nw} | Svicky: {nc}", True, (255, 220, 80)), (25, PANEL_H - 66))
        self.screen.blit(self.font_main.render("[SPACE] Freeze | [C] Reset Mapy", True, (140, 150, 165)), (25, PANEL_H - 48))

    def draw_right_stable(self):
        self.draw_panel_decorations(PANEL_W, "2. STABILNI MAPA HRISTE (CONFIRMED)", ACCENT_MAP)
        scale = (PANEL_W / 2.0) / VIEW_SPAN

        conf_candles = self.model.get_confirmed_candles()
        conf_walls = self.model.get_confirmed_walls()

        # Stabilní stěny (výrazně tlusté zlatavě-zelené)
        for w in conf_walls:
            p1 = self.to_screen(w['x1'], w['y1'], PANEL_W)
            p2 = self.to_screen(w['x2'], w['y2'], PANEL_W)
            pygame.draw.line(self.screen, (80, 255, 140), p1, p2, 6)
            pygame.draw.circle(self.screen, (150, 255, 180), p1, 5)
            pygame.draw.circle(self.screen, (150, 255, 180), p2, 5)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            lbl = self.font_bold.render(f"STENA {int(w['len'])}mm", True, (80, 255, 140))
            self.screen.blit(lbl, (mx + 8, my - 8))

        # Stabilní svíčky (jasně červené terče)
        for idx, c in enumerate(conf_candles):
            scx, scy = self.to_screen(c['x'], c['y'], PANEL_W)
            r_ring = int(200.0 * scale)
            pygame.draw.circle(self.screen, (255, 80, 80), (scx, scy), max(4, r_ring), 2)
            pygame.draw.circle(self.screen, (255, 40, 40), (scx, scy), 8)
            pygame.draw.circle(self.screen, (255, 240, 180), (scx, scy), 3)

            d = int(math.hypot(c['x'], c['y']))
            lbl = self.font_bold.render(f"SVICKA #{idx+1} ({d}mm)", True, (255, 100, 100))
            self.screen.blit(lbl, (scx + 12, scy - 12))

        # Plánovaná optimální trasa mezi potvrzenými svíčkami (žlutá čára)
        if len(conf_candles) >= 2:
            unvisited = list(conf_candles)
            route = [(0.0, 0.0)]
            curr = (0.0, 0.0)
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

        # Info box pravého panelu
        map_box = pygame.Surface((280, 95))
        map_box.set_alpha(200)
        map_box.fill((10, 12, 16))
        self.screen.blit(map_box, (PANEL_W + 15, PANEL_H - 110))
        pygame.draw.rect(self.screen, (60, 50, 40), (PANEL_W + 15, PANEL_H - 110, 280, 95), 1)

        self.screen.blit(self.font_bold.render(f"POTVRZENE SVICKY: {len(conf_candles)} / 4", True, (255, 90, 90)), (PANEL_W + 25, PANEL_H - 102))
        self.screen.blit(self.font_bold.render(f"POTVRZENE STENY:  {len(conf_walls)} / 6", True, (80, 255, 140)), (PANEL_W + 25, PANEL_H - 84))
        self.screen.blit(self.font_main.render(f"Celkem skenu: {self.model.total_scans} (nehybna mapa)", True, (180, 190, 205)), (PANEL_W + 25, PANEL_H - 66))
        self.screen.blit(self.font_main.render("Filtruje lidi & drzi stale pozice", True, (255, 200, 60)), (PANEL_W + 25, PANEL_H - 48))

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        self.frozen = not self.frozen
                    elif event.key in [pygame.K_c, pygame.K_m]:
                        self.model.reset()
                        print("[MAPA] Stabilní mapa byla resetována.")
                    elif event.key == pygame.K_f:
                        self.fov_limited = not self.fov_limited
                        self.send_cmd(f"FOV {1 if self.fov_limited else 0}")
                    elif event.key == pygame.K_r:
                        self.min_range = min(1000, self.min_range + 50)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                    elif event.key == pygame.K_e:
                        self.min_range = max(50, self.min_range - 50)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                    elif event.key == pygame.K_t:
                        self.max_range = min(6000, self.max_range + 200)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                    elif event.key == pygame.K_g:
                        self.max_range = max(500, self.max_range - 200)
                        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
                    elif event.key == pygame.K_i:
                        self.min_intensity = min(200, self.min_intensity + 5)
                        self.send_cmd(f"INTENSITY {self.min_intensity}")
                    elif event.key == pygame.K_k:
                        self.min_intensity = max(0, self.min_intensity - 5)
                        self.send_cmd(f"INTENSITY {self.min_intensity}")

            self.read_serial()

            self.screen.fill(BG_COLOR)
            self.draw_left_live()
            self.draw_right_stable()

            # Vertikální předěl mezi panely
            pygame.draw.line(self.screen, (50, 60, 75), (PANEL_W, 0), (PANEL_W, WIN_H), 2)

            pygame.display.flip()
            self.clock.tick(60)

        if self.ser: self.ser.close()
        pygame.quit()

if __name__ == "__main__":
    port = get_serial_port()
    baud = get_baudrate()
    app = DualLidarVisualizer(port, baud)
    app.run()
