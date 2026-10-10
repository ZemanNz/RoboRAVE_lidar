#!/usr/bin/env python3
"""
LIDAR_SVICKY_27 - Vizualizační a řídicí terminál pro testování LiDARu D200 (Kybernoid 27)
Autor: Antigravity & Nik

Použití:
    python3 lidar_viz.py [/dev/ttyUSB0] [921600]

Klávesové zkratky a ovládání:
    F           - Přepnout režim 360° / 180° (FoV ořez)
    SPACE       - Zmrazit / Odmrazit obraz (Freeze)
    R           - Resetovat / Zvýšit práh minimální vzdálenosti (+50 mm)
    E           - Snížit práh minimální vzdálenosti (-50 mm)
    T           - Zvýšit práh max vzdálenosti (+200 mm)
    G           - Snížit práh max vzdálenosti (-200 mm)
    I / K       - Zvýšit / Snížit práh intenzity (±5)
    Klik myší   - Zobrazit detailní souřadnice nejbližšího objektu
    ESC / Q     - Ukončit
"""

import sys, os, time, json, math
import pygame
import serial
from serial.tools import list_ports

# Výchozí konfigurace
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 921600
WIN_SIZE     = 950
BG_COLOR     = (18, 20, 24)
GRID_COLOR   = (40, 45, 52)
CIRCLE_COLOR = (55, 62, 75)
TEXT_COLOR   = (220, 225, 230)
ACCENT_COLOR = (0, 210, 255)

# Rozsah zobrazení (mm od středu robota)
VIEW_SPAN_MM = 3500.0  # Poloměr okna odpovídá ~3.5 metru

def get_serial_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = list(list_ports.comports())
    for p in ports:
        if "USB" in p.device or "ACM" in p.device:
            return p.device
    return DEFAULT_PORT

def get_baudrate():
    if len(sys.argv) > 2:
        return int(sys.argv[2])
    return DEFAULT_BAUD

class LidarVisualizer:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.frozen = False
        
        # Lokální stav parametrů
        self.fov_limited = False
        self.min_range = 130
        self.max_range = 4000
        self.min_intensity = 15
        
        # Poslední přijatá data
        self.last_frame = {
            "fps": 0.0,
            "pts": 0,
            "calc_us": 0,
            "heap": 0,
            "crc_err": 0,
            "walls": [],
            "candles": [],
            "cloud": []
        }
        
        self.selected_object = None

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_SIZE, WIN_SIZE))
        pygame.display.set_caption(f"Kybernoid 27 - LiDAR D200 Detector [{port}]")
        self.clock = pygame.time.Clock()
        
        try:
            self.font_main = pygame.font.SysFont("DejaVu Sans Mono", 14)
            self.font_bold = pygame.font.SysFont("DejaVu Sans Mono", 15, bold=True)
            self.font_big  = pygame.font.SysFont("DejaVu Sans Mono", 18, bold=True)
        except:
            self.font_main = pygame.font.SysFont(None, 18)
            self.font_bold = pygame.font.SysFont(None, 20)
            self.font_big  = pygame.font.SysFont(None, 24)

        self.connect_serial()

    def connect_serial(self):
        try:
            print(f"[SERIAL] Připojuji k {self.port} při {self.baud} baud...")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.ser.reset_input_buffer()
            print("[SERIAL] Připojeno úspěšně!")
        except Exception as e:
            print(f"[SERIAL CHYBA] Nelze otevřít {self.port}: {e}")
            self.ser = None

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                line = (cmd_str.strip() + "\n").encode('utf-8')
                self.ser.write(line)
                self.ser.flush()
                print(f"[CMD POSLÁNO] {cmd_str}")
            except Exception as e:
                print(f"[CMD CHYBA] {e}")

    def to_screen(self, x_mm, y_mm):
        """
        Převod kartézských mm na pixely okna.
        Střed senzoru (0,0) je uprostřed okna (WIN_SIZE/2, WIN_SIZE/2).
        Y směřuje nahoru (dopředu pro robota), X vpravo.
        """
        cx = WIN_SIZE / 2.0
        cy = WIN_SIZE / 2.0
        scale = (WIN_SIZE / 2.0) / VIEW_SPAN_MM
        
        sx = int(cx + x_mm * scale)
        sy = int(cy - y_mm * scale)  # v Pygame je osa Y dolů
        return sx, sy

    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            return

        while self.ser.in_waiting > 0:
            try:
                raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw_line:
                    continue
                
                # Zprávy začínající # jsou logy z firmware
                if raw_line.startswith("#"):
                    print(raw_line)
                    continue

                if raw_line.startswith("{") and raw_line.endswith("}"):
                    if not self.frozen:
                        data = json.loads(raw_line)
                        self.last_frame = data
            except Exception as e:
                # Ošetření přerušeného JSONu na lince
                pass

    def draw_grid_and_rings(self):
        cx = WIN_SIZE // 2
        cy = WIN_SIZE // 2
        scale = (WIN_SIZE / 2.0) / VIEW_SPAN_MM

        # Osy X a Y
        pygame.draw.line(self.screen, GRID_COLOR, (0, cy), (WIN_SIZE, cy), 1)
        pygame.draw.line(self.screen, GRID_COLOR, (cx, 0), (cx, WIN_SIZE), 1)

        # Orientační kruhy vzdálenosti (1 m, 2 m, 3 m)
        for dist_m in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
            r_px = int(dist_m * 1000.0 * scale)
            col = (65, 75, 90) if dist_m in [1.0, 2.0, 3.0] else (35, 40, 48)
            pygame.draw.circle(self.screen, col, (cx, cy), r_px, 1)
            
            # Popisek vzdálenosti
            lbl = self.font_main.render(f"{dist_m:.1f} m", True, (120, 130, 145))
            self.screen.blit(lbl, (cx + 5, cy - r_px - 8))

        # Obrys hřiště Kybernoid (2400 × 3500 mm) jako orientační reference
        # Střed senzoru uprostřed hřiště
        hw = 2400 / 2.0
        hh = 3500 / 2.0
        p1 = self.to_screen(-hw, -hh)
        p2 = self.to_screen( hw, -hh)
        p3 = self.to_screen( hw,  hh)
        p4 = self.to_screen(-hw,  hh)
        pygame.draw.polygon(self.screen, (38, 50, 42), [p1, p2, p3, p4], 1)

        # Tělo senzoru / robota
        pygame.draw.circle(self.screen, (0, 180, 255), (cx, cy), 7)
        # Šipka dopředu (Y+)
        pygame.draw.line(self.screen, (0, 255, 180), (cx, cy), (cx, cy - 35), 3)

    def draw_lidar_cloud(self):
        cloud = self.last_frame.get("cloud", [])
        # cloud je ve formátu [x1, y1, int1, x2, y2, int2, ...]
        n = len(cloud) // 3
        for i in range(n):
            x = cloud[i * 3]
            y = cloud[i * 3 + 1]
            intensity = cloud[i * 3 + 2]

            sx, sy = self.to_screen(x, y)
            if 0 <= sx < WIN_SIZE and 0 <= sy < WIN_SIZE:
                # Barevné tónování podle intenzity odrazivosti
                val = min(255, max(40, intensity * 2 + 50))
                color = (val, val, min(255, val + 20))
                self.screen.set_at((sx, sy), color)
                # Pro výraznější zobrazení: malý čtvereček 2x2
                self.screen.set_at((sx+1, sy), color)
                self.screen.set_at((sx, sy+1), color)

    def draw_detections(self):
        scale = (WIN_SIZE / 2.0) / VIEW_SPAN_MM

        # 1. Zelené stěny (rovné bílé desky 300 mm)
        walls = self.last_frame.get("walls", [])
        for w in walls:
            # w = [x1, y1, x2, y2, len]
            x1, y1, x2, y2, l_mm = w
            p1 = self.to_screen(x1, y1)
            p2 = self.to_screen(x2, y2)
            
            # Tlustá zelená čára
            pygame.draw.line(self.screen, (0, 255, 100), p1, p2, 5)
            # Koncové body
            pygame.draw.circle(self.screen, (50, 255, 150), p1, 4)
            pygame.draw.circle(self.screen, (50, 255, 150), p2, 4)

            # Popisek délky uprostřed stěny
            mid_x = (x1 + x2) / 2.0
            mid_y = (y1 + y2) / 2.0
            smx, smy = self.to_screen(mid_x, mid_y)
            lbl = self.font_bold.render(f"STĚNA {l_mm}mm", True, (0, 255, 100))
            self.screen.blit(lbl, (smx + 8, smy - 8))

        # 2. Červené svíčky (černé válce průměr 50 mm + bezpečnostní zóna 400 mm)
        candles = self.last_frame.get("candles", [])
        for c in candles:
            # c = [x, y, diam]
            cx_mm, cy_mm, diam = c
            scx, scy = self.to_screen(cx_mm, cy_mm)

            # Bezpečnostní zóna hašení (kruh o průměru 400 mm kolem svíčky)
            r_ring_px = int(200.0 * scale)
            pygame.draw.circle(self.screen, (255, 80, 80, 50), (scx, scy), max(4, r_ring_px), 1)

            # Samotný válec svíčky (průměr 50 mm)
            r_px = max(5, int((diam / 2.0) * scale))
            pygame.draw.circle(self.screen, (255, 40, 40), (scx, scy), r_px)
            pygame.draw.circle(self.screen, (255, 220, 220), (scx, scy), 2)

            dist_to_robot = math.sqrt(cx_mm * cx_mm + cy_mm * cy_mm)
            lbl = self.font_bold.render(f"SVÍČKA ({int(dist_to_robot)}mm)", True, (255, 80, 80))
            self.screen.blit(lbl, (scx + 12, scy - 10))

    def draw_hud(self):
        # Panel telemetrie v levém horním rohu
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

        # Panel nápovědy a ovládání v levém dolním rohu
        cmd_panel = pygame.Surface((440, 120))
        cmd_panel.set_alpha(200)
        cmd_panel.fill((10, 12, 16))
        self.screen.blit(cmd_panel, (15, WIN_SIZE - 135))

        cy = WIN_SIZE - 128
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
            self.screen.blit(sel_panel, (WIN_SIZE - 335, 15))
            pygame.draw.rect(self.screen, (255, 180, 0), (WIN_SIZE - 335, 15, 320, 70), 1)

            st = self.font_bold.render("INSPEKCE VYBRANÉHO BODU:", True, (255, 200, 50))
            self.screen.blit(st, (WIN_SIZE - 325, 22))
            sx_mm, sy_mm, s_dist = self.selected_object
            s_info = self.font_main.render(f"X: {sx_mm:.0f} mm, Y: {sy_mm:.0f} mm | R: {s_dist:.0f} mm", True, (255, 255, 255))
            self.screen.blit(s_info, (WIN_SIZE - 325, 45))

    def handle_click(self, pos):
        # Převod kliku v okně na kartézské mm
        cx = WIN_SIZE / 2.0
        cy = WIN_SIZE / 2.0
        scale = (WIN_SIZE / 2.0) / VIEW_SPAN_MM
        click_x_mm = (pos[0] - cx) / scale
        click_y_mm = -(pos[1] - cy) / scale

        # Hledáme nejbližší objekt (svíčku, stěnu nebo bod)
        best_dist = 999999.0
        best_obj = None

        # Svíčky
        for c in self.last_frame.get("candles", []):
            dx = c[0] - click_x_mm
            dy = c[1] - click_y_mm
            d = math.sqrt(dx*dx + dy*dy)
            if d < best_dist:
                best_dist = d
                best_obj = (c[0], c[1], math.sqrt(c[0]**2 + c[1]**2))

        # Stěny
        for w in self.last_frame.get("walls", []):
            mx = (w[0] + w[2]) / 2.0
            my = (w[1] + w[3]) / 2.0
            dx = mx - click_x_mm
            dy = my - click_y_mm
            d = math.sqrt(dx*dx + dy*dy)
            if d < best_dist:
                best_dist = d
                best_obj = (mx, my, math.sqrt(mx**2 + my**2))

        if best_dist < 400: # v dosahu kliku
            self.selected_object = best_obj
        else:
            self.selected_object = (click_x_mm, click_y_mm, math.sqrt(click_x_mm**2 + click_y_mm**2))

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.handle_click(event.pos)
                elif event.type == pygame.KEYDOWN:
                    if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        self.frozen = not self.frozen
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
            self.draw_grid_and_rings()
            self.draw_lidar_cloud()
            self.draw_detections()
            self.draw_hud()

            pygame.display.flip()
            self.clock.tick(60)

        if self.ser:
            self.ser.close()
        pygame.quit()

if __name__ == "__main__":
    port = get_serial_port()
    baud = get_baudrate()
    app = LidarVisualizer(port, baud)
    app.run()
