#!/usr/bin/env python3
"""
LIDAR_SVICKY_27 - Dvoupanelový vizualizér s interaktivními tlačítky (GUI)
Autor: Antigravity & Nik

Dva panely vedle sebe:
  1. LEVÝ PANEL: Živý radar (Live LIDAR scan) v reálném čase.
  2. PRAVÝ PANEL: Stabilní herní mapa (World Model - uzamčené 4 svíčky a 6 stěn).

Interaktivní tlačítka (klikací myší i přes klávesnici):
  - [FOV]: Přepínání 180° dopředný kužel / 360° celé pole (výchozí 180°)
  - [ZOOM +/-]: Zvětšení / zmenšení zobrazení (zoom in / zoom out)
  - [MIN DIST +/-]: Ořez mrtvé zóny senzoru
  - [MAX DIST +/-]: Ořez stěn místnosti
  - [INTENSITY +/-]: Práh odrazivosti
  - [RESET MAPY]: Okamžité vymazání stabilní mapy (také se resetuje automaticky při každé změně parametrů!)
  - [FREEZE]: Zmrazení obrazu
"""

import sys, os, time, json, math
import pygame
import serial
from serial.tools import list_ports

# Rozměry
PANEL_W     = 820
PANEL_H     = 840
WIN_W       = PANEL_W * 2
WIN_H       = PANEL_H

# Barvy
BG_COLOR    = (13, 15, 19)
GRID_COLOR  = (30, 34, 42)
TEXT_COLOR  = (220, 225, 230)
ACCENT_LIVE = (0, 210, 255)
ACCENT_MAP  = (255, 200, 60)
BTN_BG      = (22, 26, 34)
BTN_BORDER  = (50, 60, 75)
BTN_HOVER   = (35, 42, 55)

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
#  Trida pro tlacitko v GUI
# ============================================================
class Button:
    def __init__(self, rect, label, action_tag, col_accent=ACCENT_LIVE):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.action_tag = action_tag
        self.col_accent = col_accent
        self.hovered = False

    def draw(self, screen, font):
        bg = BTN_HOVER if self.hovered else BTN_BG
        pygame.draw.rect(screen, bg, self.rect, border_radius=4)
        border_col = self.col_accent if self.hovered else BTN_BORDER
        pygame.draw.rect(screen, border_col, self.rect, 1, border_radius=4)

        lbl = font.render(self.label, True, (240, 240, 245) if self.hovered else (180, 190, 205))
        lx = self.rect.centerx - lbl.get_width() // 2
        ly = self.rect.centery - lbl.get_height() // 2
        screen.blit(lbl, (lx, ly))

    def handle_mouse(self, mouse_pos):
        self.hovered = self.rect.collidepoint(mouse_pos)

    def is_clicked(self, mouse_pos):
        return self.rect.collidepoint(mouse_pos)


# ============================================================
#  Trida pro akumulaci a stabilitu herniho pole (World Model)
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

        # 2. STĚNY
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
        valid = [c for c in self.candles if c['hits'] >= 5]
        valid.sort(key=lambda c: c['hits'], reverse=True)
        return valid[:4]

    def get_confirmed_walls(self):
        valid = [w for w in self.walls if w['hits'] >= 4]
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

        # Výchozí nastavení dle požadavku: 180° dopředný kužel
        self.fov_limited = True
        self.min_range = 130
        self.max_range = 2400
        self.min_intensity = 15
        self.view_span_mm = 2800.0  # Měřítko zoomu

        self.last_frame = {
            "fps": 0.0, "pts": 0, "calc_us": 0, "heap": 0, "crc_err": 0,
            "walls": [], "candles": [], "cloud": []
        }

        self.model = StableFieldModel()

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption(f"Kybernoid 27 - Interactive Dual Lidar Radar & Field Map [{port}]")
        self.clock = pygame.time.Clock()

        try:
            self.font_main = pygame.font.SysFont("DejaVu Sans Mono", 12)
            self.font_bold = pygame.font.SysFont("DejaVu Sans Mono", 13, bold=True)
            self.font_big  = pygame.font.SysFont("DejaVu Sans Mono", 16, bold=True)
        except:
            self.font_main = pygame.font.SysFont(None, 15)
            self.font_bold = pygame.font.SysFont(None, 17)
            self.font_big  = pygame.font.SysFont(None, 21)

        # Vytvoření tlačítek GUI
        self.buttons = []
        self.init_buttons()

        self.connect_serial()
        # Pošleme výchozí 180° režim do ESP32
        self.send_cmd(f"FOV 1")
        self.send_cmd(f"RANGE {self.min_range} {self.max_range}")

    def init_buttons(self):
        # Spodní lišta tlačítek pod levým panelem
        y_row1 = PANEL_H - 105
        y_row2 = PANEL_H - 65

        # Řádek 1: FOV, Zoom, Freeze, Reset Mapy
        self.buttons.append(Button((20,  y_row1, 130, 32), "FoV: 180°", "TOGGLE_FOV", ACCENT_LIVE))
        self.buttons.append(Button((160, y_row1, 95, 32), "Zoom +", "ZOOM_IN", ACCENT_LIVE))
        self.buttons.append(Button((265, y_row1, 95, 32), "Zoom -", "ZOOM_OUT", ACCENT_LIVE))
        self.buttons.append(Button((370, y_row1, 110, 32), "Zmrazit", "FREEZE", ACCENT_LIVE))
        self.buttons.append(Button((490, y_row1, 160, 32), "RESET MAPY (C)", "RESET_MAP", ACCENT_MAP))

        # Řádek 2: Min Range, Max Range, Intenzita
        self.buttons.append(Button((20,  y_row2, 100, 32), "MinR -50", "MINR_DEC", ACCENT_LIVE))
        self.buttons.append(Button((125, y_row2, 100, 32), "MinR +50", "MINR_INC", ACCENT_LIVE))
        self.buttons.append(Button((235, y_row2, 105, 32), "MaxR -200", "MAXR_DEC", ACCENT_LIVE))
        self.buttons.append(Button((345, y_row2, 105, 32), "MaxR +200", "MAXR_INC", ACCENT_LIVE))
        self.buttons.append(Button((460, y_row2, 90, 32), "Int -5", "INT_DEC", ACCENT_LIVE))
        self.buttons.append(Button((555, y_row2, 90, 32), "Int +5", "INT_INC", ACCENT_LIVE))

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
        """Vždy když se změní nastavení filtrů/dosahu/FoV, vygeneruje se nová mapa."""
        self.model.reset()
        print("[MAPA] Parametry změněny -> vygenerována nová čistá mapa.")

    def to_screen(self, x_mm, y_mm, panel_offset_x):
        cx = panel_offset_x + PANEL_W / 2.0
        cy = PANEL_H / 2.0 - 20
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

    def handle_action(self, tag):
        if tag == "TOGGLE_FOV":
            self.fov_limited = not self.fov_limited
            self.send_cmd(f"FOV {1 if self.fov_limited else 0}")
            self.buttons[0].label = "FoV: 180°" if self.fov_limited else "FoV: 360°"
            self.on_parameter_changed()
        elif tag == "ZOOM_IN":
            self.view_span_mm = max(1200.0, self.view_span_mm - 300.0)
        elif tag == "ZOOM_OUT":
            self.view_span_mm = min(5000.0, self.view_span_mm + 300.0)
        elif tag == "FREEZE":
            self.frozen = not self.frozen
            self.buttons[3].label = "Odmrazit" if self.frozen else "Zmrazit"
        elif tag == "RESET_MAP":
            self.on_parameter_changed()
        elif tag == "MINR_DEC":
            self.min_range = max(50, self.min_range - 50)
            self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
            self.on_parameter_changed()
        elif tag == "MINR_INC":
            self.min_range = min(1000, self.min_range + 50)
            self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
            self.on_parameter_changed()
        elif tag == "MAXR_DEC":
            self.max_range = max(600, self.max_range - 200)
            self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
            self.on_parameter_changed()
        elif tag == "MAXR_INC":
            self.max_range = min(5000, self.max_range + 200)
            self.send_cmd(f"RANGE {self.min_range} {self.max_range}")
            self.on_parameter_changed()
        elif tag == "INT_DEC":
            self.min_intensity = max(0, self.min_intensity - 5)
            self.send_cmd(f"INTENSITY {self.min_intensity}")
            self.on_parameter_changed()
        elif tag == "INT_INC":
            self.min_intensity = min(200, self.min_intensity + 5)
            self.send_cmd(f"INTENSITY {self.min_intensity}")
            self.on_parameter_changed()

    def draw_panel_decorations(self, offset_x, title, accent_col):
        cx = offset_x + PANEL_W // 2
        cy = PANEL_H // 2 - 20
        scale = (PANEL_W / 2.0) / self.view_span_mm

        pygame.draw.rect(self.screen, (22, 26, 32), (offset_x + 2, 2, PANEL_W - 4, PANEL_H - 4), 1)

        # Osy
        pygame.draw.line(self.screen, GRID_COLOR, (offset_x, cy), (offset_x + PANEL_W, cy), 1)
        pygame.draw.line(self.screen, GRID_COLOR, (cx, 0), (cx, PANEL_H), 1)

        # Orientační kruhy
        for d_m in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
            r_px = int(d_m * 1000.0 * scale)
            c = (42, 50, 62) if d_m in [1.0, 2.0] else (24, 28, 35)
            pygame.draw.circle(self.screen, c, (cx, cy), r_px, 1)
            lbl = self.font_main.render(f"{d_m:.1f} m", True, (90, 100, 115))
            self.screen.blit(lbl, (cx + 5, cy - r_px - 8))

        # Obrys hřiště 2400 x 3500 mm
        hw = 2400 / 2.0
        p1 = self.to_screen(-hw, 0, offset_x)
        p2 = self.to_screen( hw, 0, offset_x)
        p3 = self.to_screen( hw, 3500, offset_x)
        p4 = self.to_screen(-hw, 3500, offset_x)
        pygame.draw.polygon(self.screen, (22, 38, 30), [p1, p2, p3, p4], 1)

        # Robot
        pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 7)
        pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 30), 3)

        # Nadpis panelu
        t_surf = self.font_big.render(title, True, accent_col)
        self.screen.blit(t_surf, (offset_x + 20, 16))

    def draw_left_live(self):
        self.draw_panel_decorations(0, "1. ZIVY RADAR (REAL-TIME)", ACCENT_LIVE)
        scale = (PANEL_W / 2.0) / self.view_span_mm

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

        # Parametry a telemetrie nahoře
        f = self.last_frame
        fps = f.get("fps", 0.0)
        calc_us = f.get("calc_us", 0)
        pts = f.get("pts", 0)
        nw = len(f.get("walls", []))
        nc = len(f.get("candles", []))

        # Stavový pruh filtrů vlevo nahoře
        p_str = f"FPS: {fps:.1f} Hz | Calc: {calc_us}µs | Pts: {pts} | Dosah: {self.min_range}-{self.max_range}mm | Int: {self.min_intensity}"
        self.screen.blit(self.font_main.render(p_str, True, (0, 210, 255)), (20, 42))

        # Vykreslení tlačítek
        for btn in self.buttons:
            btn.draw(self.screen, self.font_bold)

    def draw_right_stable(self):
        self.draw_panel_decorations(PANEL_W, "2. STABILNI MAPA HRISTE (WORLD MODEL)", ACCENT_MAP)
        scale = (PANEL_W / 2.0) / self.view_span_mm

        conf_candles = self.model.get_confirmed_candles()
        conf_walls = self.model.get_confirmed_walls()

        # Stabilní stěny
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

        # Stabilní svíčky
        for idx, c in enumerate(conf_candles):
            scx, scy = self.to_screen(c['x'], c['y'], PANEL_W)
            r_ring = int(200.0 * scale)
            pygame.draw.circle(self.screen, (255, 80, 80), (scx, scy), max(4, r_ring), 2)
            pygame.draw.circle(self.screen, (255, 40, 40), (scx, scy), 8)
            pygame.draw.circle(self.screen, (255, 240, 180), (scx, scy), 3)

            d = int(math.hypot(c['x'], c['y']))
            lbl = self.font_bold.render(f"SVICKA #{idx+1} ({d}mm)", True, (255, 100, 100))
            self.screen.blit(lbl, (scx + 12, scy - 12))

        # Trasa
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

        # Info pruh pravého panelu
        stat_str = f"Potvrzeno: {len(conf_candles)}/4 svicek | {len(conf_walls)}/6 sten | Skenu v modelovem poli: {self.model.total_scans}"
        self.screen.blit(self.font_bold.render(stat_str, True, ACCENT_MAP), (PANEL_W + 20, 42))

        # Nápověda vpravo dole
        tip_box = pygame.Surface((340, 60))
        tip_box.set_alpha(180)
        tip_box.fill((10, 12, 16))
        self.screen.blit(tip_box, (PANEL_W + 20, PANEL_H - 80))
        pygame.draw.rect(self.screen, (50, 45, 30), (PANEL_W + 20, PANEL_H - 80, 340, 60), 1)
        self.screen.blit(self.font_main.render("• Mapa se generuje ze souhlasnych mereni", True, (200, 205, 215)), (PANEL_W + 30, PANEL_H - 72))
        self.screen.blit(self.font_main.render("• Pri zmene parametru se vycisti a prepocita", True, (240, 200, 80)), (PANEL_W + 30, PANEL_H - 52))
        self.screen.blit(self.font_main.render("• Klavesa [C] pro okamzity manualni reset", True, (160, 170, 185)), (PANEL_W + 30, PANEL_H - 34))

    def run(self):
        while self.running:
            mouse_pos = pygame.mouse.get_pos()
            for btn in self.buttons:
                btn.handle_mouse(mouse_pos)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        for btn in self.buttons:
                            if btn.is_clicked(event.pos):
                                self.handle_action(btn.action_tag)
                                break
                    elif event.button == 4: # Kolečko myši nahoru (zoom in)
                        self.handle_action("ZOOM_IN")
                    elif event.button == 5: # Kolečko myši dolů (zoom out)
                        self.handle_action("ZOOM_OUT")

                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        self.handle_action("FREEZE")
                    elif event.key in [pygame.K_c, pygame.K_m]:
                        self.handle_action("RESET_MAP")
                    elif event.key == pygame.K_f:
                        self.handle_action("TOGGLE_FOV")
                    elif event.key == pygame.K_r:
                        self.handle_action("MINR_INC")
                    elif event.key == pygame.K_e:
                        self.handle_action("MINR_DEC")
                    elif event.key == pygame.K_t:
                        self.handle_action("MAXR_INC")
                    elif event.key == pygame.K_g:
                        self.handle_action("MAXR_DEC")
                    elif event.key == pygame.K_i:
                        self.handle_action("INT_INC")
                    elif event.key == pygame.K_k:
                        self.handle_action("INT_DEC")
                    elif event.key in [pygame.K_PLUS, pygame.K_EQUALS]:
                        self.handle_action("ZOOM_IN")
                    elif event.key in [pygame.K_MINUS]:
                        self.handle_action("ZOOM_OUT")

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
