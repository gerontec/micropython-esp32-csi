"""
Herzschlag- und Atemerkennung via WiFi-CSI auf dem ESP32-S3

Voraussetzungen:
  - Person befindet sich im selben Raum, 0.5–3 m vom Router entfernt
  - WiFi-Traffic vorhanden (ggf. kontinuierlichen Ping starten)
  - Erste Schätzung nach ~25 Sekunden

Auf dem Gerät als main.py speichern oder im REPL eingeben.
"""

import network, time, csi, heartbeat

# ─── WiFi ─────────────────────────────────────────────────────────────────
SSID     = "MeinNetz"
PASSWORD = "MeinPasswort"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
print("Verbinde...", end="")
for _ in range(30):
    if wlan.isconnected():
        break
    time.sleep(0.5)
    print(".", end="")
print(" OK –", wlan.ifconfig()[0])

# ─── CSI initialisieren ────────────────────────────────────────────────────
csi.init()
csi.config(
    lltf=True,
    htltf=False,       # für Vitaldaten reicht LLTF (52 Subträger)
    stbc_htltf2=False,
    channel_filter=False,  # ungefiltert = maximale Sensitivität
    ltf_merge=False,
)
csi.enable(True)

print("\nSammle CSI-Daten... (Person im Raum?)")
print("Erste Herzschlagschätzung nach ~25 Sekunden\n")

# ─── Hauptschleife ─────────────────────────────────────────────────────────
frame_count = 0
last_print  = time.ticks_ms()

while True:
    frame = csi.read(timeout_ms=500)
    if frame is None:
        # Kein Traffic → künstlichen erzeugen
        continue

    # Herzschlagmodul befüttern
    heartbeat.feed(frame)
    frame_count += 1

    # Alle 2 Sekunden Status ausgeben
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= 2000:
        last_print = now
        info = heartbeat.info()

        if heartbeat.ready():
            bpm = heartbeat.get_bpm()
            rpm = heartbeat.get_rpm()
            print(f"Herzschlag: {bpm:5.1f} BPM  |  "
                  f"Atmung: {rpm:4.1f} /min  |  "
                  f"Subträger #{info['best_subcarrier']}  |  "
                  f"fs≈{info['fs']:.1f} Hz  |  "
                  f"Frames: {frame_count}")
        else:
            pct = info['samples'] * 100 // 256
            print(f"Warte auf genug Daten... {pct}%  "
                  f"({info['samples']}/256 Samples, "
                  f"fs≈{info['fs']:.1f} Hz)")

# ─── Spektrum ausgeben (Debug) ─────────────────────────────────────────────
# spec = heartbeat.spectrum()
# fs   = heartbeat.info()['fs']
# freq_res = fs / 256
# for i, p in enumerate(spec[:64]):
#     freq = i * freq_res
#     bar  = int(p / max(spec) * 40) if max(spec) > 0 else 0
#     if 0.5 < freq < 3.0:
#         print(f"{freq:.2f} Hz | {'#' * bar} {p:.1f}")
