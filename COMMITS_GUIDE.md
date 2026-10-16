# Průvodce verzemi a commity (LIDAR_SVICKY_27)

Tento dokument slouží jako rychlý přehled historie vývoje, vlastností jednotlivých commitů, verzí **výslovně pochválených uživatelem** a návodu, jak se k nim kdykoliv vrátit.

---

## 🌟 Rychlý rozcestník doporučených verzí

| Verze / Git Tag | Commit Hash | Uživatelské hodnocení | Klíčové vlastnosti |
| :--- | :--- | :--- | :--- |
| **`v1.1-latest-backup`** (HEAD) | `11fbd60` | 🔥 *Nejnovější plná verze* | Překlenutí stínů svíček, detekce rohové svíčky průsečíkem, 300mm stěny, modrý záměrný kužel ±10°. |
| **`3a030e5`** | `3a030e5` | ⭐ *"Tohle už je taky dost dobrý"* | Přidán 20° dopředný záměrný výsek (modrý kužel) a priorita 1. cíle. |
| **`v1.0-user-approved`** | `e501734` | ⭐⭐⭐ *"Už to byla ta verze, kde jsem ti říkal, že je to fakt super / v Gitu se mi to velice líbilo"* | **Zlatá stabilní verze**: čistý původní textový HUD, stabilní World Model (pravý panel), osvědčená detekce. |
| **`890ef73`** | `890ef73` | ⭐⭐⭐ *"Už to vypadá mnohem lépe, vypadá to fakt dobře... tam ta detekce stěn byla lepší"* | Původní průlomová verze s candle-centric prokládáním stěn za svíčkou. |

---

## 📋 Detailní přehled commitů (od nejnovějšího k nejstaršímu)

### 1. Commit `11fbd60` (Tag: `v1.1-latest-backup`, Větev: `backup_v1.1_latest`)
- **Popis:** *Detector: Bridge candle shadows to detect full 300 mm walls and pad to 300 mm*
- **V čem je dobrý:**
  - Vyřešil situaci, kdy svíčka stojící před stěnou vrže stín (~94 mm) a rozdělí stěnu na dvě křídla.
  - Zvýšil parametr `max_point_dist_mm` na 125 mm a snížil `min_length_mm` na 90 mm, takže stěna za svíčkou není roztržena ani zahozena.
  - Zvýšil strop délky na 360 mm (naměřená deska pod úhlem dosahuje ~340 mm).
  - Automaticky dotahuje stěny s dostatkem bodů symetricky na plnou nominální délku **300 mm**.

### 2. Commit `f5fec4e`
- **Popis:** *Detector: Add line extension to recover full 300 mm wall length from inliers*
- **V čem je dobrý:**
  - Zavedl Region Growing / Line Extension: prohledává osu stěny dopředu i dozadu (až ±160 mm) a napojuje zapadající body na stejné přímce.
  - Zabraňuje předčasnému useknutí stěny na 190–240 mm kvůli drobným úhlovým výpadkům.

### 3. Commit `cdc0408`
- **Popis:** *Detector: Add L-corner candle extraction via perpendicular walls intersection*
- **V čem je dobrý:**
  - Vyřešil problém, proč nebyla detekována 4. svíčka v pravém horním rohu.
  - Detekuje L-rohy ze 2 kolmých stěn (90°), spočte průsečík nosných přímek a prohledá kapsu rohu.
  - Chrání rohové svíčky před vyřazením fialovým koridorem stěn.

### 4. Commit `3a030e5` ⭐ *(Pochváleno: „Tohle už je taky dost dobrý“)*
- **Popis:** *Aiming sector: Add 20 deg forward aiming sector (+-10 deg) with blue cone, target reticle, and route priority*
- **V čem je dobrý:**
  - Vizuální modrý výsek ±10° dopředu pro zamíření robota před startem na 1. svíčku.
  - Svíčka ve výseku získá tyrkysový záměrný kříž `★ 1. CÍL` a prioritní první pozici v plánované trase.

### 5. Commit `0ecc434`
- **Popis:** *Corridor: Extend forbidden wall corridor to 150 mm beyond wall ends and draw purple corridor in GUI*
- **V čem je dobrý:**
  - Prodloužil zakázaný koridor stěny o 15 cm za oba její konce a vizualizoval jej fialově v Pygame.
  - Zabraňuje falešné detekci svíčky v přímce stěny a v mezerách mezi deskami.

### 6. Commit `4758ea9`
- **Popis:** *Detector: Eliminate false candle detections lying collinear with wall paths or on corners*
- **V čem je dobrý:**
  - Základní kontrola kolinearity – svíčka nesmí ležet přímo v ose stěny.

### 7. Commit `e501734` (Tag: `v1.0-user-approved`) ⭐⭐⭐ *(Pochváleno: „Zlatá stabilní verze“)*
- **Popis:** *Rollback detector: Revert detector.h to exact working version from commit 890ef73*
- **V čem je dobrý:**
  - Základní prověřená verze detektoru ze zářijového/říjnového ladění.
  - Obnoven původní textový HUD vlevo dole (bez barevných tlačítek, čisté klávesové zkratky).
  - Plně stabilní dvoupanelový visualizér s pravým panelem (World Model / stabilní akumulace pole).

### 8. Commit `890ef73` ⭐⭐⭐ *(Pochváleno: „Už to vypadá mnohem lépe, vypadá to fakt dobře“)*
- **Popis:** *Detector v1.3: Candle-centric wall hypotheses fitting*
- **V čem je dobrý:**
  - První verze, která začala stěny hledat cíleně v okolí svíček i přes zástin.

---

## 🛠️ Jak se přepínat mezi verzemi v Gitu

Kdykoliv se můžeš přepnout na libovolný commit nebo tag bez obav ze ztráty práce:

### A. Návrat k nejnovější záloze (se všemi vylepšeními):
```bash
git checkout main_v1
# nebo přímo na tag:
git checkout v1.1-latest-backup
```

### B. Přepnutí na verzi se záměrným kuželem (`3a030e5`):
```bash
git checkout 3a030e5
```

### C. Návrat ke zlaté stabilní verzi (`v1.0-user-approved`):
```bash
git checkout v1.0-user-approved
```

### D. Návrat k původní verzi s detekcí stěn (`890ef73`):
```bash
git checkout 890ef73
```

---

## ⚡ Rychlé přeložení a nahrání po přepnutí verze

Po jakémkoliv přepnutí větve/commitu stačí nahrát kód do připojeného ESP32:

```bash
# Sestavení a upload do ESP32
~/.platformio/penv/bin/pio run --target upload --upload-port /dev/ttyUSB0

# Spuštění vizualizéru
python3 lidar_viz.py /dev/ttyUSB0 921600
```
