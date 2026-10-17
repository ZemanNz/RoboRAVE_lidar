# LiDAR D200 / LD14P Detector & Test Framework (Kybernoid 27)

Modulární testovací a detekční framework pro 360° triangulační laserový skener **Waveshare D200 (jádro LD14P)** přímo na mikrokontroleru **ESP32** (Arduino / PlatformIO) s interaktivním PC vizualizérem v Pythonu (Pygame).

Framework je navržen pro soutěž **Kybernoid 27 (hašení svíček)**:
- Plocha pole: $2400 \times 3500\text{ mm}$ (otevřené hřiště bez pevných obvodových mantinelů)
- Překážky (stěny): rovné bílé desky šířky **$300\text{ mm} \pm 40\text{ mm}$**
- Cíle (svíčky): černé válce o průměru **$50\text{ mm}$** s bezpečnostní zónou hašení $400\text{ mm}$

Veškerá filtrace a detekce běží **přímo na ESP32**. Počítač slouží pouze jako pasivní monitor a terminál pro ladění parametrů za běhu.

---

## 1. Architektura projektu

```
LIDAR_SVICKY_27/
├── platformio.ini         # Konfigurace ESP32 (baud 921600)
├── lidar_viz.py           # Pygame vizualizér s interaktivním řízením za běhu
├── README.md              # Tento návod k oživení
└── src/
    ├── main.cpp           # Hlavní smyčka integrující pipeline
    ├── d200_types.h       # Datové struktury (body, stěny, svíčky, telemetrie)
    ├── d200_driver.h      # UART2 ovladač (230400 baud, CRC8, kartézský převod)
    ├── filter_pipeline.h  # Range clamp, odrazivost, FoV ořez, konstrukční maska
    ├── detector.h         # Extrakce 30cm stěn a clustering 50mm svíček
    └── comm_protocol.h    # JSON streaming a obousměrné příkazy pro ladění
```

---

## 2. Zapojení hardwaru

| LiDAR D200 (LD14P) | ESP32 Pin | Popis |
| :--- | :--- | :--- |
| **TX** | **GPIO 13** | UART2 RX (230 400 baud) |
| **RX / PWM** | **GPIO 10** | Řízení otáček / TX (volitelně) |
| **VCC (5V)** | **5V (VIN)** | Napájení motoru i laseru (odběr cca 250–400 mA) |
| **GND** | **GND** | Společná zem |

> [!IMPORTANT]
> Pro komunikaci s PC použij standardní USB kabel do ESP32. Rychlost USB sériové linky je nastavena na **921 600 baud**.

---

## 3. Rychlé zprovoznění a flash firmware

### A. Překlad a nahrání do ESP32
Projekt je připraven pro PlatformIO:

```bash
# Sestavení projektu
~/.platformio/penv/bin/pio run

# Nahrání do připojeného ESP32
~/.platformio/penv/bin/pio run --target upload
```

### B. Spuštění Python vizualizéru
Potřebné knihovny:
```bash
pip install pygame pyserial
```

Spuštění vizualizéru:
```bash
python3 lidar_viz.py /dev/ttyUSB0 921600
```
*(Pokud nezadáš port, skript se pokusí automaticky detekovat první dostupné USB sériové zařízení).*

---

## 4. Ovládání a ladění za běhu v `lidar_viz.py`

| Klávesa | Akce | Popis |
| :--- | :--- | :--- |
| **`F`** | Přepnout 360° / 180° | Omezí zorné pole pouze na dopředný kužel |
| **`MEZERNÍK`** | Freeze | Zmrazí aktuální snímek pro detailní prohlídku |
| **`R` / `E`** | Min Range $\pm 50\text{ mm}$ | Zvýší / sníží mrtvou zónu senzoru |
| **`T` / `G`** | Max Range $\pm 200\text{ mm}$ | Zvýší / sníží maximální dosah (potlačení stěn místnosti) |
| **`I` / `K`** | Min Intenzita $\pm 5$ | Nastavení prahu odrazivosti signálu |
| **Klik myši** | Inspekce objektu | Zobrazí přesné souřadnice $[X, Y]$ a vzdálenost |
| **`ESC` / `Q`** | Konec | Ukončení aplikace |

---

## 5. Doporučené prahy pro první test na koberci

Při testování v pokoji / na koberci mimo soutěžní arénu doporučujeme následující nastavení:

1. **Range Clamp:**
   - `min_range`: **$130\text{ mm}$** (D200 má optickou slepou zónu cca $100\text{ mm}$).
   - `max_range`: **$2500\text{ mm}$** (pokud jsi v menším pokoji, omezí se nábytek a vzdálené zdi).
2. **Minimální intenzita (`min_intensity`):**
   - **$15$ až $25$**: Dobře odfiltruje zrcadlové odlesky z lesklých podlah a lakovaných povrchů, ale spolehlivě zachytí bílé dřevotřískové desky i černý matný válec svíčky.
3. **Tolerance stěny (`detector.h`):**
   - Nominální šířka stěny: **$300\text{ mm}$**
   - Tolerance: **$\pm 60\text{ mm}$** (při zešikmeném úhlu pohledu LiDAR vidí jen část stěny nebo je okraj zanořený).
4. **Rozpoznání svíčky:**
   - Shluk 2 až 8 bodů o průměru **$30\text{ až }90\text{ mm}$** ve vzdálenosti alespoň **$90\text{ mm}$** od nejbližší stěny.

---

## 6. Historie verzí a osvědčené commity

Detailní rozpis všech verzí, vlastností, změn parametrů a verzí výslovně pochválených uživatelem nalezneš v samostatném průvodci:
👉 **[COMMITS_GUIDE.md](file:///home/nik/Plocha/Programing/C++/Platformio/LIDAR_SVICKY_27/COMMITS_GUIDE.md)**

- **`v1.2` (HEAD)**: Nové pravidlo kontroly kolmice na svíčku (+400 b. priorita pro stěny se svíčkou před zdmi pokoje), záchrana a 300mm dotažení stíněných stěn od 75 mm
- **`v1.1-latest-backup` (`11fbd60`)**: Plné protažení 300mm stěn přes stín svíčky, detekce rohů a ±10° zaměřovač
- **`3a030e5`**: Verze se záměrným modrým výsekem 20° a prioritou 1. cíle
- **`v1.0-user-approved` (`e501734`)**: Zlatá stabilní verze (klasický HUD, stabilní mapa hřiště)
- **`890ef73`**: Původní osvědčená detekce stěn prokládáním kolem svíček
