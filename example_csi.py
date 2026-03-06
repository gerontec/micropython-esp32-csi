"""
ESP32-S3 CSI example – läuft direkt auf dem Gerät als main.py

Workflow:
  1. WiFi verbinden (Station-Mode)
  2. CSI initialisieren + konfigurieren
  3. Daten lesen (polling ODER callback)
"""

import network
import time
import csi

# ─── WiFi ────────────────────────────────────────────────────────────────────

SSID     = "MeinNetz"
PASSWORD = "MeinPasswort"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

print("Verbinde mit", SSID, "...", end="")
for _ in range(20):
    if wlan.isconnected():
        break
    time.sleep(0.5)
    print(".", end="")
if not wlan.isconnected():
    raise RuntimeError("WiFi-Verbindung fehlgeschlagen")
print(" OK –", wlan.ifconfig()[0])

# ─── CSI initialisieren ───────────────────────────────────────────────────────

csi.init()

# Konfiguration:
#   lltf           – Legacy LTF (52 Subträger für 20 MHz)
#   htltf          – HT LTF    (56 Subträger für 20 MHz, 114 für 40 MHz)
#   stbc_htltf2    – STBC HT LTF2 (nur bei STBC-Paketen relevant)
#   ltf_merge      – lltf + htltf mitteln → glättere Schätzung
#   channel_filter – Tiefpass über Subträger (False = rohste Daten)
#   manu_scale     – False = Auto-Skalierung
#   shift          – manuelle Linksverschiebung (0–15), nur wenn manu_scale=True

csi.config(
    lltf=True,
    htltf=True,
    stbc_htltf2=True,
    ltf_merge=False,      # False = unabhängige LLTF + HTLTF Rohdaten
    channel_filter=False, # False = keine Glättung → volle Subträger-Auflösung
    manu_scale=False,
)

csi.enable(True)

# ─── Variante A: Polling ──────────────────────────────────────────────────────

print("\n=== Polling-Modus (10 Frames) ===")

for i in range(10):
    frame = csi.read(timeout_ms=2000)
    if frame is None:
        print("Timeout – kein Frame empfangen")
        continue

    # Rohdaten als I/Q-Paare aufschlüsseln
    iq = csi.parse_iq(frame["data"])

    print(f"Frame {i+1:2d}  "
          f"MAC={frame['mac'].hex(':'):<17}  "
          f"RSSI={frame['rssi']:4d} dBm  "
          f"CH={frame['channel']:2d}  "
          f"BW={frame['bandwidth']} MHz  "
          f"sig_mode={frame['sig_mode']}  "
          f"MCS={frame['mcs']}  "
          f"len={frame['len']}B  "
          f"IQ[0]={iq[0] if iq else 'n/a'}")

csi.enable(False)

# ─── Variante B: Callback ─────────────────────────────────────────────────────

print("\n=== Callback-Modus (5 Sekunden) ===")

frame_count = [0]

def on_csi(frame):
    frame_count[0] += 1
    iq = csi.parse_iq(frame["data"])
    n  = len(iq)
    # Amplitude = sqrt(I² + Q²) für jeden Subträger
    amps = [((iq[k][0]**2 + iq[k][1]**2) ** 0.5) for k in range(n)]
    avg_amp = sum(amps) / n if n else 0
    print(f"CB #{frame_count[0]:4d}  "
          f"RSSI={frame['rssi']:4d} dBm  "
          f"Subträger={n}  "
          f"ø|H|={avg_amp:.2f}  "
          f"first_invalid={frame['first_word_invalid']}")

csi.enable(True)
csi.set_callback(on_csi)

time.sleep(5)

csi.set_callback(None)
csi.enable(False)

print(f"\nEmpfangen: {frame_count[0]} Frames in 5 Sekunden")

# ─── Erweiterte Analyse: Subträger-Struktur ───────────────────────────────────
# Bei 802.11n HT20 (LLTF + HTLTF + STBC-HTLTF2):
#   LLTF:       52 Subträger × 2 Bytes (I+Q) =  104 Bytes
#   HTLTF:      56 Subträger × 2 Bytes (I+Q) =  112 Bytes
#   STBC-HTLTF2:56 Subträger × 2 Bytes (I+Q) =  112 Bytes
#   Gesamt:     328 Bytes (bei first_word_invalid=True: ersten 4 Bytes ignorieren)
