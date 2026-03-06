"""
MR60BHA1 MicroPython driver
Seeed Studio 60 GHz mmWave FMCW Radar – Herzschlag & Atemfrequenz

Protokoll: UART 115200 8N1
Frame:  [0x53][0x59][TYPE][CMD][LEN_L][LEN_H][DATA...][CHECKSUM][0x54][0x43]
Prüfsum: sum(TYPE, CMD, LEN_L, LEN_H, DATA...) & 0xFF

Verwendung:
    from mr60bha1 import MR60BHA1
    import machine, time

    radar = MR60BHA1(uart_id=2, tx=17, rx=18)
    while True:
        radar.update()          # UART-Puffer verarbeiten
        print(radar.bpm, radar.rpm, radar.status_str)
        time.sleep_ms(50)
"""

import machine
import time
import struct

# ── Frame-Konstanten ──────────────────────────────────────────────────────────
_SOF1, _SOF2 = 0x53, 0x59     # Start of Frame
_EOF1, _EOF2 = 0x54, 0x43     # End of Frame

# TYPE-Bytes
_T_SYSTEM    = 0x01
_T_PRODUCT   = 0x02
_T_STATUS    = 0x05
_T_BREATH    = 0x80
_T_HEART     = 0x81

# CMD-Bytes
_CMD_INFO    = 0x01     # Kategorie (normal/rapid/slow)
_CMD_WAVE    = 0x02     # Rohwellenform
_CMD_RATE    = 0x05     # Frequenzwert (BPM / Atemzüge/min)

# Status-Codes (TYPE=0x05, CMD=0x01)
_STATUS = {0: "initialisierung", 1: "kalibrierung", 2: "messung"}

# Kategorie-Codes
_BREATH_CAT  = {0: "keine", 1: "normal", 2: "zu schnell", 3: "zu langsam"}
_HEART_CAT   = {0: "keine", 1: "normal", 2: "zu schnell", 3: "zu langsam"}

# ── Parser-Zustände ───────────────────────────────────────────────────────────
_S_SOF1, _S_SOF2 = 0, 1
_S_TYPE, _S_CMD  = 2, 3
_S_LENL, _S_LENH = 4, 5
_S_DATA          = 6
_S_CKSUM         = 7
_S_EOF1, _S_EOF2 = 8, 9


class MR60BHA1:
    """
    Treiber für Seeed Studio MR60BHA1 60-GHz-mmWave-Radar.

    Parameter:
        uart_id  – UART-Nummer (0–2)
        tx, rx   – GPIO-Pins
        baudrate – default 115200
        callback – optionale Funktion(frame_dict) die bei jedem Frame aufgerufen wird
    """

    def __init__(self, uart_id=2, tx=17, rx=18, baudrate=115200, callback=None):
        self._uart = machine.UART(uart_id, baudrate=baudrate,
                                   tx=tx, rx=rx,
                                   bits=8, parity=None, stop=1,
                                   rxbuf=512)
        self._cb = callback

        # Messwerte
        self.bpm           = -1      # Herzschlag (Schläge/min), -1 = noch unbekannt
        self.rpm           = -1      # Atemfrequenz (Atemzüge/min)
        self.bpm_category  = "keine" # "normal" | "zu schnell" | "zu langsam"
        self.rpm_category  = "keine"
        self.status        = -1      # 0=init, 1=kalibrierung, 2=messung
        self.status_str    = "unbekannt"
        self.bpm_wave      = 0       # letzter Rohwellenform-Wert Herzschlag
        self.rpm_wave      = 0       # letzter Rohwellenform-Wert Atmung

        # Statistik
        self.frames_ok     = 0
        self.frames_err    = 0

        # Parser-Zustand
        self._state   = _S_SOF1
        self._type    = 0
        self._cmd     = 0
        self._len     = 0
        self._data    = bytearray()
        self._cksum_acc = 0          # laufende Prüfsumme

    # ── Öffentliche API ───────────────────────────────────────────────────────

    def update(self):
        """
        UART-Puffer lesen und verarbeiten.
        Muss regelmäßig aufgerufen werden (z.B. alle 50 ms).
        """
        available = self._uart.any()
        if not available:
            return
        chunk = self._uart.read(min(available, 256))
        if chunk:
            for byte in chunk:
                self._parse_byte(byte)

    def read_blocking(self, timeout_ms=2000):
        """
        Blockierend warten bis mindestens ein vollständiger Frame empfangen wurde.
        Gibt die Anzahl verarbeiteter Frames zurück oder 0 bei Timeout.
        """
        start = time.ticks_ms()
        before = self.frames_ok
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            self.update()
            if self.frames_ok > before:
                return self.frames_ok - before
            time.sleep_ms(10)
        return 0

    def set_callback(self, func):
        """Callback-Funktion setzen: func(frame_dict) oder None."""
        self._cb = func

    def query_status(self):
        """Arbeitsstatus vom Sensor abfragen."""
        self._send(_T_STATUS, 0x81, b'')

    def query_product_info(self):
        """Produktinformationen abfragen (erscheinen als print-Ausgabe)."""
        self._send(_T_PRODUCT, 0xA1, b'')

    def ready(self):
        """True wenn Sensor im Messmodus und erste Werte vorliegen."""
        return self.status == 2 and self.bpm > 0 and self.rpm > 0

    def info(self):
        """Debug-Informationsdict."""
        return {
            "bpm":          self.bpm,
            "rpm":          self.rpm,
            "bpm_category": self.bpm_category,
            "rpm_category": self.rpm_category,
            "status":       self.status_str,
            "bpm_wave":     self.bpm_wave,
            "rpm_wave":     self.rpm_wave,
            "frames_ok":    self.frames_ok,
            "frames_err":   self.frames_err,
        }

    # ── Frame-Parser (Zustandsmaschine) ──────────────────────────────────────

    def _parse_byte(self, b):
        s = self._state

        if s == _S_SOF1:
            if b == _SOF1:
                self._state = _S_SOF2

        elif s == _S_SOF2:
            if b == _SOF2:
                self._state   = _S_TYPE
                self._cksum_acc = 0
            else:
                self._state = _S_SOF1

        elif s == _S_TYPE:
            self._type      = b
            self._cksum_acc = b
            self._state     = _S_CMD

        elif s == _S_CMD:
            self._cmd        = b
            self._cksum_acc += b
            self._state      = _S_LENL

        elif s == _S_LENL:
            self._len        = b
            self._cksum_acc += b
            self._state      = _S_LENH

        elif s == _S_LENH:
            self._len       |= b << 8
            self._cksum_acc += b
            self._data       = bytearray()
            if self._len == 0:
                self._state  = _S_CKSUM
            elif self._len > 128:
                # Unsinnige Länge → Sync-Verlust
                self.frames_err += 1
                self._state = _S_SOF1
            else:
                self._state  = _S_DATA

        elif s == _S_DATA:
            self._data.append(b)
            self._cksum_acc += b
            if len(self._data) == self._len:
                self._state = _S_CKSUM

        elif s == _S_CKSUM:
            expected = self._cksum_acc & 0xFF
            if b == expected:
                self._state = _S_EOF1
            else:
                self.frames_err += 1
                self._state = _S_SOF1

        elif s == _S_EOF1:
            if b == _EOF1:
                self._state = _S_EOF2
            else:
                self.frames_err += 1
                self._state = _S_SOF1

        elif s == _S_EOF2:
            self._state = _S_SOF1
            if b == _EOF2:
                self.frames_ok += 1
                self._dispatch(self._type, self._cmd, bytes(self._data))
            else:
                self.frames_err += 1

    # ── Frame-Dispatch ────────────────────────────────────────────────────────

    def _dispatch(self, t, cmd, data):
        frame = {"type": t, "cmd": cmd, "data": data}

        if t == _T_STATUS and cmd == 0x01 and len(data) >= 1:
            self.status     = data[0]
            self.status_str = _STATUS.get(data[0], f"unbekannt({data[0]})")
            frame["status"] = self.status_str

        elif t == _T_BREATH:
            if cmd == _CMD_INFO and len(data) >= 1:
                self.rpm_category = _BREATH_CAT.get(data[0], "unbekannt")
                frame["rpm_category"] = self.rpm_category
            elif cmd == _CMD_RATE and len(data) >= 1:
                self.rpm = data[0]
                frame["rpm"] = self.rpm
            elif cmd == _CMD_WAVE and len(data) >= 4:
                self.rpm_wave = struct.unpack("<f", data[:4])[0]
                frame["rpm_wave"] = self.rpm_wave

        elif t == _T_HEART:
            if cmd == _CMD_INFO and len(data) >= 1:
                self.bpm_category = _HEART_CAT.get(data[0], "unbekannt")
                frame["bpm_category"] = self.bpm_category
            elif cmd == _CMD_RATE and len(data) >= 1:
                self.bpm = data[0]
                frame["bpm"] = self.bpm
            elif cmd == _CMD_WAVE and len(data) >= 4:
                self.bpm_wave = struct.unpack("<f", data[:4])[0]
                frame["bpm_wave"] = self.bpm_wave

        if self._cb:
            try:
                self._cb(frame)
            except Exception as e:
                print("MR60BHA1 callback error:", e)

    # ── Sende-Hilfsfunktion ───────────────────────────────────────────────────

    def _send(self, t, cmd, data: bytes):
        length = len(data)
        cksum  = (t + cmd + (length & 0xFF) + (length >> 8) +
                  sum(data)) & 0xFF
        frame  = bytes([_SOF1, _SOF2, t, cmd,
                        length & 0xFF, length >> 8]) + \
                 data + bytes([cksum, _EOF1, _EOF2])
        self._uart.write(frame)

    # ── Kontextmanager ────────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self._uart.deinit()
