# micropython-esp32-csi

A MicroPython user C module that provides full access to **WiFi Channel State Information (CSI)** on the **ESP32-S3**.

CSI data contains the complex channel frequency response (I/Q values per subcarrier) for every received WiFi frame — useful for indoor positioning, gesture recognition, presence detection, and RF channel analysis.

## Features

- Full raw CSI data access: LLTF, HTLTF, STBC-HTLTF2
- All rx_ctrl metadata: RSSI, noise floor, channel, MCS, bandwidth, timestamp, antenna
- Thread-safe: polling with GIL release, async callback via `mp_sched_schedule`
- 16-frame ring buffer (drop-oldest strategy)
- Helper to parse raw bytes into I/Q tuples
- Works alongside all standard MicroPython WiFi/network APIs

## Compatibility

| Component      | Version         |
|---------------|-----------------|
| MicroPython   | v1.24.1         |
| ESP-IDF       | v5.2.x          |
| Target        | ESP32-S3        |

## Build

**Prerequisites:** ESP-IDF 5.2 installed and activated, MicroPython v1.24.1 source.

```bash
git clone https://github.com/gerontec/micropython-esp32-csi.git
cd micropython-esp32-csi

# Edit build.sh to point to your MicroPython and ESP-IDF paths, then:
chmod +x build.sh
./build.sh              # build only
./build.sh /dev/ttyUSB0 # build + flash
```

Or manually with CMake:

```bash
source /path/to/esp-idf/export.sh

cmake -S /path/to/micropython/ports/esp32 \
      -B build \
      -DMICROPY_BOARD=ESP32_GENERIC_S3 \
      -DUSER_C_MODULES=/path/to/micropython-esp32-csi/micropython.cmake \
      -DCMAKE_BUILD_TYPE=Release

cmake --build build -- -j$(nproc)
```

## Python API

```python
import network, csi, time

# WiFi must be connected in station mode before using CSI
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("SSID", "password")
while not wlan.isconnected():
    time.sleep(0.5)

# Initialize CSI subsystem (registers rx callback with ESP-IDF)
csi.init()

# Configure which LTF fields to capture
csi.config(
    lltf=True,           # Legacy LTF:       52 subcarriers @ 20 MHz
    htltf=True,          # HT LTF:           56/114 subcarriers @ 20/40 MHz
    stbc_htltf2=True,    # STBC HT LTF2:     only relevant for STBC packets
    ltf_merge=False,     # False = independent raw LLTF + HTLTF data
    channel_filter=False,# False = no smoothing -> full per-subcarrier resolution
    manu_scale=False,    # False = automatic scaling
    shift=0,             # left-shift bits (0-15), only if manu_scale=True
    dump_ack=False,      # also capture ACK frames
)

csi.enable(True)

# --- Polling mode ---
frame = csi.read(timeout_ms=2000)  # returns dict or None on timeout
if frame:
    iq = csi.parse_iq(frame["data"])  # list of (I, Q) tuples
    print(f"RSSI={frame['rssi']} dBm  CH={frame['channel']}  "
          f"BW={frame['bandwidth']} MHz  subcarriers={len(iq)}")
    print(f"IQ[0]={iq[0]}  IQ[-1]={iq[-1]}")

# --- Callback mode ---
def on_csi(frame):
    iq = csi.parse_iq(frame["data"])
    amps = [(iq[i][0]**2 + iq[i][1]**2)**0.5 for i in range(len(iq))]
    print(f"RSSI={frame['rssi']:4d} dBm  "
          f"subcarriers={len(iq)}  "
          f"mean|H|={sum(amps)/len(amps):.2f}")

csi.set_callback(on_csi)
time.sleep(10)
csi.set_callback(None)

csi.enable(False)
```

## Frame Dictionary

`csi.read()` and the callback both deliver a dict with the following keys:

| Key                  | Type      | Description                                      |
|---------------------|-----------|--------------------------------------------------|
| `data`              | bytearray | Raw CSI bytes: `[I0, Q0, I1, Q1, ...]` signed   |
| `len`               | int       | Length of `data` in bytes                        |
| `mac`               | bytes(6)  | Source MAC address                               |
| `dmac`              | bytes(6)  | Destination MAC address                          |
| `rssi`              | int       | Received signal strength (dBm)                   |
| `noise_floor`       | int       | Noise floor (dBm)                                |
| `channel`           | int       | Primary WiFi channel                             |
| `secondary_channel` | int       | Secondary channel (0=none, 1=above, 2=below)     |
| `bandwidth`         | int       | Channel bandwidth: 20 or 40 (MHz)                |
| `sig_mode`          | int       | `csi.SIG_11BG`=0, `csi.SIG_HT`=1, `csi.SIG_VHT`=3 |
| `mcs`               | int       | Modulation and Coding Scheme index               |
| `sgi`               | bool      | Short Guard Interval                             |
| `stbc`              | int       | Space Time Block Coding streams                  |
| `fec_coding`        | bool      | LDPC forward error correction                    |
| `timestamp`         | int       | Local timestamp (µs)                             |
| `ant`               | int       | Receive antenna (0 or 1)                         |
| `first_word_invalid`| bool      | If True, discard `iq[0]` and `iq[1]`             |
| `aggregation`       | bool      | AMPDU aggregated packet                          |
| `ampdu_cnt`         | int       | Number of subframes in AMPDU                     |
| `sig_len`           | int       | Packet length including FCS (bytes)              |
| `rx_state`          | int       | 0 = no error                                     |

## CSI Buffer Layout

The raw `data` bytearray contains interleaved signed 8-bit I and Q values per subcarrier:

```
[I₀, Q₀, I₁, Q₁, ..., Iₙ, Qₙ]
```

For 802.11n HT20 with all fields enabled:

| Field       | Subcarriers | Bytes |
|-------------|-------------|-------|
| LLTF        | 52          | 104   |
| HTLTF       | 56          | 112   |
| STBC-HTLTF2 | 56          | 112   |
| **Total**   | **164**     | **328** |

For HT40, HTLTF and STBC-HTLTF2 use 114 subcarriers each (total 684 bytes).

Use `csi.parse_iq(frame["data"])` to get a list of `(I, Q)` tuples, or work with the bytearray directly using `struct.unpack` for performance.

## Additional API

```python
csi.available()      # number of frames waiting in the internal queue (int)
csi.clear()          # flush the internal queue
csi.parse_iq(data)   # bytearray -> list of (I, Q) tuples

# Constants
csi.SIG_11BG  # 0
csi.SIG_HT    # 1  (802.11n)
csi.SIG_VHT   # 3  (802.11ac)
```

## Heartbeat & Breathing Detection

The `heartbeat` module uses the CSI amplitude time series to detect human vital signs in the same room (0.5–3 m from the access point).

### Algorithm

1. **Amplitude**: compute `|H_k| = sqrt(I² + Q²)` for each subcarrier per frame
2. **Subcarrier selection**: pick the subcarrier with the highest amplitude variance over a sliding window (most motion-sensitive)
3. **IIR bandpass filters** (4th-order Butterworth, precomputed for 10 Hz sample rate):
   - Heartbeat: 0.8–2.5 Hz (48–150 BPM)
   - Breathing: 0.1–0.5 Hz (6–30 breaths/min)
4. **FFT** (Cooley-Tukey radix-2, N=256) with Hann window → find spectral peak
5. **First estimate** available after ~25 seconds of data at 10 Hz

### Usage

```python
import csi, heartbeat

csi.init()
csi.config(lltf=True, htltf=False, channel_filter=False)
csi.enable(True)

while True:
    frame = csi.read(timeout_ms=500)
    if frame:
        heartbeat.feed(frame)

    if heartbeat.ready():
        print(f"Heart: {heartbeat.get_bpm():.1f} BPM  "
              f"Breath: {heartbeat.get_rpm():.1f} /min")
```

### Heartbeat API

| Function               | Returns     | Description                                        |
|------------------------|-------------|----------------------------------------------------|
| `heartbeat.feed(frame)`| None        | Feed a CSI frame dict from `csi.read()`            |
| `heartbeat.get_bpm()`  | float       | Latest heart rate estimate in BPM, or -1.0         |
| `heartbeat.get_rpm()`  | float       | Latest breathing rate (breaths/min), or -1.0       |
| `heartbeat.ready()`    | bool        | True once enough data has been collected           |
| `heartbeat.reset()`    | None        | Clear all buffers and state                        |
| `heartbeat.spectrum()` | list[float] | Power spectrum of heartbeat-filtered signal (debug)|
| `heartbeat.info()`     | dict        | Debug info: samples, fs, best_subcarrier, bpm, rpm |

### Practical notes

- Person should be within 0.5–3 m, line of sight preferred
- More WiFi traffic → more CSI frames → better temporal resolution
- Use `csi.config(channel_filter=False)` for maximum subcarrier independence
- Results improve significantly after the first minute of continuous data
- The algorithm is based on: Wang et al., "Understanding and Modeling of WiFi Signal Based Human Activity Recognition", MobiCom 2015

## License

MIT
