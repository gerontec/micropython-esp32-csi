/*
 * heartbeat.c – WiFi-CSI-basierte Herzschlag- und Atemfrequenzerkennung
 *
 * Algorithmus (nach Wang et al., IEEE INFOCOM 2017):
 *   1. CSI-Amplitude berechnen: |H_k| = sqrt(I_k² + Q_k²) pro Subträger
 *   2. Aktivsten Subträger wählen (höchste Varianz über die Zeit)
 *   3. Bandpassfilter:  0.8–2.5 Hz  → Herzschlag
 *                       0.1–0.5 Hz  → Atmung
 *   4. FFT → Dominante Frequenz → BPM / Atemzüge/min
 *
 * Python-API:
 *   import heartbeat
 *   heartbeat.feed(frame)        # CSI-Frame von csi.read() einspeisen
 *   bpm   = heartbeat.get_bpm()  # letzte Herzschlagschätzung (float) oder -1.0
 *   rpm   = heartbeat.get_rpm()  # letzte Atemfrequenz (float) oder -1.0
 *   ready = heartbeat.ready()    # True sobald genug Daten für erste Schätzung
 *   spec  = heartbeat.spectrum() # Leistungsspektrum als Liste (Debug)
 *   heartbeat.reset()            # Puffer leeren
 *
 * Anforderungen:
 *   - MicroPython CSI-Modul muss initialisiert sein (csi.init(), csi.enable(True))
 *   - Person sollte 0.5–3 m vom AP entfernt sein, selber Raum
 *   - Mindestens 25 Sekunden Daten für erste Schätzung nötig
 *   - Genug WiFi-Traffic im Raum (ggf. kontinuierlichen Ping starten)
 */

#include <math.h>
#include <string.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "py/objlist.h"

/* ── Konfiguration ───────────────────────────────────────────────────────── */

#define HB_N            256     /* FFT-Fenstergröße (Potenz von 2)           */
#define HB_FS_DEFAULT   10.0f   /* Standard-Abtastrate (Hz) – geschätzt      */
#define HB_BPM_MIN      40.0f   /* min. erkannter Herzschlag (BPM)           */
#define HB_BPM_MAX      180.0f  /* max. erkannter Herzschlag (BPM)           */
#define HB_BREATH_MIN   6.0f    /* min. Atemfrequenz (Atemzüge/min)          */
#define HB_BREATH_MAX   30.0f   /* max. Atemfrequenz (Atemzüge/min)          */
#define HB_SUBCARRIER_N 52      /* LLTF-Subträger @ 20 MHz                  */
#define HB_VAR_WIN      64      /* Fenster zur Subträgerauswahl (Samples)    */
#define HB_ANALYSIS_STEP 10     /* Neue Analyse alle N neuen Samples          */

/* ── 4th-order Butterworth Bandpass-Koeffizienten ────────────────────────── */
/* scipy.signal.butter(2, [0.8/5, 2.5/5], 'bandpass') bei fs=10 Hz           */
/* Implementiert als zwei kaskadierende Biquad-Sektionen (Direct Form II)     */

/* Herzschlag-Filter: 0.8–2.5 Hz */
static const float HB_B[5] = {
     0.15998789f,  0.0f, -0.31997577f,  0.0f,  0.15998789f
};
static const float HB_A[5] = {
     1.0f, -1.53461304f,  1.23484048f, -0.62958489f,  0.23484048f
};

/* Atem-Filter: 0.1–0.5 Hz */
static const float BR_B[5] = {
     0.01335920f,  0.0f, -0.02671840f,  0.0f,  0.01335920f
};
static const float BR_A[5] = {
     1.0f, -3.61132985f,  4.92981059f, -3.01904898f,  0.70089678f
};

/* ── Modulzustand ────────────────────────────────────────────────────────── */

static float  s_amp_buf[HB_N];         /* Amplituden-Ringpuffer              */
static float  s_hb_buf[HB_N];          /* nach Herzschlagfilter              */
static float  s_br_buf[HB_N];          /* nach Atemfilter                    */
static int    s_head      = 0;          /* Schreibzeiger                      */
static int    s_count     = 0;          /* gesammelte Samples                 */
static int    s_since_analysis = 0;     /* Samples seit letzter Analyse       */

static float  s_fs        = HB_FS_DEFAULT;
static uint32_t s_last_ts = 0;          /* letzter Zeitstempel (µs)           */

static float  s_bpm       = -1.0f;
static float  s_rpm       = -1.0f;

/* IIR-Filterzustand (Direct Form II, 4 Verzögerungen pro Filter) */
static float  s_hb_w[4]  = {0};
static float  s_br_w[4]  = {0};

/* Subträger-Varianz für Auswahl */
static float  s_var_acc[HB_SUBCARRIER_N];
static float  s_var_mean[HB_SUBCARRIER_N];
static int    s_var_n     = 0;
static int    s_best_sub  = 0;          /* gewählter Subträger-Index          */

/* FFT-Arbeitspuffer */
static float  s_fft_re[HB_N];
static float  s_fft_im[HB_N];

/* ── Interner FFT-Algorithmus (Cooley-Tukey Radix-2 DIT) ─────────────────── */

static void fft_bit_reverse(float *re, float *im, int n) {
    int bits = 0;
    for (int tmp = n >> 1; tmp; tmp >>= 1) bits++;
    for (int i = 0; i < n; i++) {
        int j = 0;
        for (int b = 0; b < bits; b++) {
            j = (j << 1) | ((i >> b) & 1);
        }
        if (j > i) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
}

static void fft_compute(float *re, float *im, int n) {
    fft_bit_reverse(re, im, n);
    for (int len = 2; len <= n; len <<= 1) {
        float angle = -2.0f * (float)M_PI / (float)len;
        float wr = cosf(angle), wi = sinf(angle);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                float ur = re[i + j],         ui = im[i + j];
                float vr = re[i + j + len/2], vi = im[i + j + len/2];
                float tvr = vr * cr - vi * ci;
                float tvi = vr * ci + vi * cr;
                re[i + j]         = ur + tvr;
                im[i + j]         = ui + tvi;
                re[i + j + len/2] = ur - tvr;
                im[i + j + len/2] = ui - tvi;
                float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

/* ── IIR-Bandpassfilter (4th order, Direct Form II transposed) ───────────── */
/* b[5], a[5]: Filterkoeffizienten                                            */
/* w[4]:       Zustandsvariablen (persistent)                                 */

static float iir4_step(float x, const float *b, const float *a, float *w) {
    /* Direct Form II Transposed – numerisch stabiler als Direct Form I       */
    float y = b[0] * x + w[0];
    w[0] = b[1] * x - a[1] * y + w[1];
    w[1] = b[2] * x - a[2] * y + w[2];
    w[2] = b[3] * x - a[3] * y + w[3];
    w[3] = b[4] * x - a[4] * y;
    return y;
}

/* ── Hilfsfunktion: Amplitude aus CSI-Bytes berechnen ───────────────────── */

static float compute_amplitude(const int8_t *buf, int len, int sub_idx) {
    /* CSI-Layout: [I0,Q0, I1,Q1, ...] als int8_t                           */
    int byte_offset = sub_idx * 2;
    if (byte_offset + 1 >= len) return 0.0f;
    float I = (float)buf[byte_offset];
    float Q = (float)buf[byte_offset + 1];
    return sqrtf(I * I + Q * Q);
}

/* ── Analyse: FFT über gefilterte Puffer, Peaks finden ──────────────────── */

static float find_peak_freq(const float *signal, int n, float fs,
                             float freq_min, float freq_max) {
    /* Signal → FFT-Puffer mit Hann-Fensterfunktion */
    for (int i = 0; i < n; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (n - 1)));
        int idx = (s_head + i) % n;  /* Ringpuffer korrekt auslesen */
        s_fft_re[i] = signal[idx] * w;
        s_fft_im[i] = 0.0f;
    }
    fft_compute(s_fft_re, s_fft_im, n);

    /* Leistungsspektrum: nur positive Frequenzen (0..n/2) */
    float freq_res = fs / (float)n;
    int bin_lo = (int)(freq_min / freq_res);
    int bin_hi = (int)(freq_max / freq_res) + 1;
    if (bin_lo < 1)   bin_lo = 1;
    if (bin_hi > n/2) bin_hi = n/2;

    float peak_power = -1.0f;
    int   peak_bin   = bin_lo;
    for (int b = bin_lo; b <= bin_hi; b++) {
        float p = s_fft_re[b] * s_fft_re[b] + s_fft_im[b] * s_fft_im[b];
        if (p > peak_power) {
            peak_power = p;
            peak_bin   = b;
        }
    }
    return (float)peak_bin * freq_res;
}

static void run_analysis(void) {
    if (s_count < HB_N) return;

    /* Herzschlag */
    float hb_freq = find_peak_freq(s_hb_buf, HB_N, s_fs,
                                   HB_BPM_MIN / 60.0f, HB_BPM_MAX / 60.0f);
    s_bpm = hb_freq * 60.0f;

    /* Atemfrequenz (Atmung langsamer, separater Filter) */
    float br_freq = find_peak_freq(s_br_buf, HB_N, s_fs,
                                   HB_BREATH_MIN / 60.0f, HB_BREATH_MAX / 60.0f);
    s_rpm = br_freq * 60.0f;
}

/* ── Subträgerauswahl: Varianzmaximierung ────────────────────────────────── */

static void update_best_subcarrier(const int8_t *buf, int len) {
    float amps[HB_SUBCARRIER_N];
    int n_sub = len / 2;
    if (n_sub > HB_SUBCARRIER_N) n_sub = HB_SUBCARRIER_N;

    for (int k = 0; k < n_sub; k++) {
        amps[k] = compute_amplitude(buf, len, k);
    }

    if (s_var_n == 0) {
        for (int k = 0; k < n_sub; k++) {
            s_var_mean[k] = amps[k];
            s_var_acc[k]  = 0.0f;
        }
    } else {
        for (int k = 0; k < n_sub; k++) {
            float delta = amps[k] - s_var_mean[k];
            s_var_mean[k] += delta / (float)(s_var_n + 1);
            s_var_acc[k]  += delta * (amps[k] - s_var_mean[k]);
        }
    }
    s_var_n++;

    if (s_var_n >= HB_VAR_WIN) {
        float best_var = -1.0f;
        for (int k = 0; k < n_sub; k++) {
            if (s_var_acc[k] > best_var) {
                best_var   = s_var_acc[k];
                s_best_sub = k;
            }
        }
        /* Neustart des Varianzfensters */
        s_var_n = 0;
        memset(s_var_acc,  0, sizeof(s_var_acc));
        memset(s_var_mean, 0, sizeof(s_var_mean));
    }
}

/* ── Python: heartbeat.feed(frame) ──────────────────────────────────────── */

static mp_obj_t hb_feed(mp_obj_t frame_obj) {
    /* frame ist das dict von csi.read() */
    mp_obj_t data_obj = mp_obj_dict_get(frame_obj, MP_OBJ_NEW_QSTR(MP_QSTR_data));
    mp_obj_t ts_obj   = mp_obj_dict_get(frame_obj, MP_OBJ_NEW_QSTR(MP_QSTR_timestamp));

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    const int8_t *raw = (const int8_t *)bufinfo.buf;
    int raw_len = (int)bufinfo.len;

    /* Abtastrate aus Zeitstempeln schätzen */
    uint32_t ts = (uint32_t)mp_obj_get_int(ts_obj);
    if (s_last_ts != 0 && ts > s_last_ts) {
        uint32_t dt_us = ts - s_last_ts;
        if (dt_us > 1000 && dt_us < 5000000) {
            float fs_new = 1e6f / (float)dt_us;
            /* Exponentiell gemittelter Schätzer */
            s_fs = 0.95f * s_fs + 0.05f * fs_new;
        }
    }
    s_last_ts = ts;

    /* Aktivsten Subträger aktualisieren */
    update_best_subcarrier(raw, raw_len);

    /* Amplitude des gewählten Subträgers */
    float amp = compute_amplitude(raw, raw_len, s_best_sub);

    /* Beide Filter anwenden */
    float hb_val = iir4_step(amp, HB_B, HB_A, s_hb_w);
    float br_val = iir4_step(amp, BR_B, BR_A, s_br_w);

    /* In Ringpuffer schreiben */
    s_amp_buf[s_head] = amp;
    s_hb_buf[s_head]  = hb_val;
    s_br_buf[s_head]  = br_val;
    s_head = (s_head + 1) % HB_N;
    if (s_count < HB_N) s_count++;

    /* Analyse periodisch ausführen */
    s_since_analysis++;
    if (s_count == HB_N && s_since_analysis >= HB_ANALYSIS_STEP) {
        s_since_analysis = 0;
        run_analysis();
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(hb_feed_obj, hb_feed);

/* ── Python: heartbeat.get_bpm() → float | -1.0 ─────────────────────────── */

static mp_obj_t hb_get_bpm(void) {
    return mp_obj_new_float(s_bpm);
}
static MP_DEFINE_CONST_FUN_OBJ_0(hb_get_bpm_obj, hb_get_bpm);

/* ── Python: heartbeat.get_rpm() → float | -1.0 ─────────────────────────── */

static mp_obj_t hb_get_rpm(void) {
    return mp_obj_new_float(s_rpm);
}
static MP_DEFINE_CONST_FUN_OBJ_0(hb_get_rpm_obj, hb_get_rpm);

/* ── Python: heartbeat.ready() → bool ───────────────────────────────────── */

static mp_obj_t hb_ready(void) {
    return mp_obj_new_bool(s_count == HB_N && s_bpm > 0.0f);
}
static MP_DEFINE_CONST_FUN_OBJ_0(hb_ready_obj, hb_ready);

/* ── Python: heartbeat.reset() ──────────────────────────────────────────── */

static mp_obj_t hb_reset(void) {
    s_head = s_count = s_since_analysis = s_var_n = 0;
    s_bpm = s_rpm = -1.0f;
    s_fs  = HB_FS_DEFAULT;
    s_last_ts = 0;
    s_best_sub = 0;
    memset(s_amp_buf,  0, sizeof(s_amp_buf));
    memset(s_hb_buf,   0, sizeof(s_hb_buf));
    memset(s_br_buf,   0, sizeof(s_br_buf));
    memset(s_hb_w,     0, sizeof(s_hb_w));
    memset(s_br_w,     0, sizeof(s_br_w));
    memset(s_var_acc,  0, sizeof(s_var_acc));
    memset(s_var_mean, 0, sizeof(s_var_mean));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(hb_reset_obj, hb_reset);

/* ── Python: heartbeat.spectrum() → list[float] (Leistungsspektrum) ─────── */

static mp_obj_t hb_spectrum(void) {
    if (s_count < HB_N) {
        return mp_obj_new_list(0, NULL);
    }

    /* Herzschlag-Signal mit Hann-Fenster in FFT-Puffer */
    for (int i = 0; i < HB_N; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (HB_N - 1)));
        int idx = (s_head + i) % HB_N;
        s_fft_re[i] = s_hb_buf[idx] * w;
        s_fft_im[i] = 0.0f;
    }
    fft_compute(s_fft_re, s_fft_im, HB_N);

    /* Nur positive Frequenzen zurückgeben (HB_N/2 Bins) */
    int n_out = HB_N / 2;
    mp_obj_t lst = mp_obj_new_list(n_out, NULL);
    mp_obj_list_t *lp = (mp_obj_list_t *)lst;
    for (int i = 0; i < n_out; i++) {
        float p = s_fft_re[i] * s_fft_re[i] + s_fft_im[i] * s_fft_im[i];
        lp->items[i] = mp_obj_new_float(p);
    }
    return lst;
}
static MP_DEFINE_CONST_FUN_OBJ_0(hb_spectrum_obj, hb_spectrum);

/* ── Python: heartbeat.info() → dict (Debug-Infos) ──────────────────────── */

static mp_obj_t hb_info(void) {
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_samples),
                      MP_OBJ_NEW_SMALL_INT(s_count));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_fs),
                      mp_obj_new_float(s_fs));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_best_subcarrier),
                      MP_OBJ_NEW_SMALL_INT(s_best_sub));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_bpm),
                      mp_obj_new_float(s_bpm));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rpm),
                      mp_obj_new_float(s_rpm));
    float freq_res = s_fs / (float)HB_N;
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_freq_resolution),
                      mp_obj_new_float(freq_res));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(hb_info_obj, hb_info);

/* ── Modultabelle ────────────────────────────────────────────────────────── */

static const mp_rom_map_elem_t hb_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_heartbeat) },
    { MP_ROM_QSTR(MP_QSTR_feed),      MP_ROM_PTR(&hb_feed_obj)       },
    { MP_ROM_QSTR(MP_QSTR_get_bpm),   MP_ROM_PTR(&hb_get_bpm_obj)    },
    { MP_ROM_QSTR(MP_QSTR_get_rpm),   MP_ROM_PTR(&hb_get_rpm_obj)    },
    { MP_ROM_QSTR(MP_QSTR_ready),     MP_ROM_PTR(&hb_ready_obj)      },
    { MP_ROM_QSTR(MP_QSTR_reset),     MP_ROM_PTR(&hb_reset_obj)      },
    { MP_ROM_QSTR(MP_QSTR_spectrum),  MP_ROM_PTR(&hb_spectrum_obj)   },
    { MP_ROM_QSTR(MP_QSTR_info),      MP_ROM_PTR(&hb_info_obj)       },
};
static MP_DEFINE_CONST_DICT(hb_module_globals, hb_module_globals_table);

const mp_obj_module_t heartbeat_user_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&hb_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_heartbeat, heartbeat_user_module);
