"""
MR60BHA1 – Herzschlag & Atemfrequenz direkt per Radar
Auf dem ESP32-S3 als main.py speichern.

Verkabelung:
    MR60BHA1 VCC  → 3.3 V
    MR60BHA1 GND  → GND
    MR60BHA1 TX   → GPIO 18  (ESP32 RX)
    MR60BHA1 RX   → GPIO 17  (ESP32 TX)
"""

import time
from mr60bha1 import MR60BHA1

# ── Initialisierung ────────────────────────────────────────────────────────
radar = MR60BHA1(uart_id=2, tx=17, rx=18)

print("MR60BHA1 gestartet – warte auf Kalibrierung (~10 Sek.)...")

# ── Variante A: Polling ────────────────────────────────────────────────────
last = time.ticks_ms()

while True:
    radar.update()

    if time.ticks_diff(time.ticks_ms(), last) >= 1000:
        last = time.ticks_ms()
        i = radar.info()

        if radar.status < 2:
            print(f"Status: {i['status']}")
        else:
            bpm_str = f"{i['bpm']:3d} BPM ({i['bpm_category']})" if i['bpm'] > 0 else "---"
            rpm_str = f"{i['rpm']:3d}/min ({i['rpm_category']})" if i['rpm'] > 0 else "---"
            print(f"Herz: {bpm_str:<25}  Atmung: {rpm_str}")

    time.sleep_ms(20)


# ── Variante B: Callback ───────────────────────────────────────────────────
# def on_frame(f):
#     if "bpm" in f:
#         print(f"Herzschlag: {f['bpm']} BPM")
#     if "rpm" in f:
#         print(f"Atmung:     {f['rpm']} /min")
#
# radar = MR60BHA1(uart_id=2, tx=17, rx=18, callback=on_frame)
# while True:
#     radar.update()
#     time.sleep_ms(20)


# ── Variante C: Kombination mit WiFi-CSI (Vergleich/Fusion) ───────────────
# import csi, heartbeat
#
# csi.init()
# csi.config(lltf=True, htltf=False, channel_filter=False)
# csi.enable(True)
#
# while True:
#     # CSI-basiert
#     frame = csi.read(timeout_ms=0)
#     if frame:
#         heartbeat.feed(frame)
#
#     # Radar-basiert
#     radar.update()
#
#     if heartbeat.ready():
#         print(f"CSI-BPM:   {heartbeat.get_bpm():.1f}  "
#               f"Radar-BPM: {radar.bpm}  "
#               f"CSI-Atm:   {heartbeat.get_rpm():.1f}  "
#               f"Radar-Atm: {radar.rpm}")
#
#     time.sleep_ms(50)
