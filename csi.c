/*
 * MicroPython CSI (Channel State Information) module for ESP32-S3
 *
 * Bietet vollen Zugriff auf WiFi-CSI-Rohdaten:
 *   - LLTF  (Legacy Long Training Field,    52 Subträger @ 20 MHz)
 *   - HTLTF (HT Long Training Field,        56 Subträger @ 20 MHz / 114 @ 40 MHz)
 *   - STBC-HTLTF2 (Space Time Block Code)
 *
 * Python-API:
 *   import csi
 *   csi.init()
 *   csi.config(lltf=True, htltf=True, stbc_htltf2=True,
 *              ltf_merge=False, channel_filter=False,
 *              manu_scale=False, shift=0, dump_ack=False)
 *   csi.enable(True)
 *   frame = csi.read(timeout_ms=1000)   # blockierendes Lesen -> dict oder None
 *   csi.set_callback(func)              # asynchroner Callback
 *   n = csi.available()                 # Anzahl gepufferter Frames
 *   csi.clear()                         # Queue leeren
 *   csi.enable(False)
 *
 *   # CSI-Rohdaten als I/Q-Paare:
 *   iq = csi.parse_iq(frame["data"])    # -> list of (I, Q) tuples
 */

#include <string.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/objarray.h"
#include "py/mperrno.h"
#include "py/mpthread.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_wifi.h"
#include "esp_log.h"

#define TAG "csi_mod"

/* Max CSI-Puffer: 802.11n HT40
 *   LLTF:       114 Subträger × 2 Bytes =  228 B
 *   HTLTF:      114 Subträger × 2 Bytes =  228 B
 *   STBC-HTLTF2:114 Subträger × 2 Bytes =  228 B
 *   Gesamt:     684 B  → runden auf 768 */
#define CSI_BUF_MAX     768
#define CSI_QUEUE_SIZE  16

/* ── interne Datentypen ───────────────────────────────────────────────────── */

typedef struct {
    /* rx_ctrl – kompatibel mit ESP-IDF 5.2 (ESP32-S3 Pfad) */
    int8_t   rssi;
    uint8_t  rate;
    uint8_t  sig_mode;          /* 0=11bg, 1=HT(11n), 3=VHT(11ac) */
    uint8_t  mcs;
    uint8_t  cwb;               /* Channel width: 0=20 MHz, 1=40 MHz */
    uint8_t  smoothing;
    uint8_t  not_sounding;
    uint8_t  aggregation;
    uint8_t  stbc;
    uint8_t  fec_coding;
    uint8_t  sgi;
    int8_t   noise_floor;
    uint8_t  ampdu_cnt;
    uint8_t  channel;
    uint8_t  secondary_channel;
    uint32_t timestamp;
    uint8_t  ant;
    uint16_t sig_len;
    uint8_t  rx_state;
    /* CSI-spezifisch */
    uint8_t  mac[6];
    uint8_t  dmac[6];
    bool     first_word_invalid;
    uint16_t len;
    int8_t   buf[CSI_BUF_MAX];
} csi_frame_t;

/* ── Modulzustand ────────────────────────────────────────────────────────── */

static QueueHandle_t    s_csi_queue   = NULL;
static SemaphoreHandle_t s_mutex      = NULL;
static mp_obj_t         s_callback    = MP_OBJ_NULL;
static TaskHandle_t     s_cb_task     = NULL;
static bool             s_initialized = false;

/* ── CSI-Empfangs-Callback (läuft im WiFi-Task) ──────────────────────────── */

/* CSI-Callback läuft im WiFi-Task-Kontext (kein ISR) – normale Queue-APIs verwenden */
static void csi_rx_cb(void *ctx, wifi_csi_info_t *data) {
    if (!data || !s_csi_queue) {
        return;
    }

    csi_frame_t frame = {0};

    frame.rssi              = data->rx_ctrl.rssi;
    frame.rate              = data->rx_ctrl.rate;
    frame.sig_mode          = data->rx_ctrl.sig_mode;
    frame.mcs               = data->rx_ctrl.mcs;
    frame.cwb               = data->rx_ctrl.cwb;
    frame.smoothing         = data->rx_ctrl.smoothing;
    frame.not_sounding      = data->rx_ctrl.not_sounding;
    frame.aggregation       = data->rx_ctrl.aggregation;
    frame.stbc              = data->rx_ctrl.stbc;
    frame.fec_coding        = data->rx_ctrl.fec_coding;
    frame.sgi               = data->rx_ctrl.sgi;
    frame.noise_floor       = data->rx_ctrl.noise_floor;
    frame.ampdu_cnt         = data->rx_ctrl.ampdu_cnt;
    frame.channel           = data->rx_ctrl.channel;
    frame.secondary_channel = data->rx_ctrl.secondary_channel;
    frame.timestamp         = data->rx_ctrl.timestamp;
    frame.ant               = data->rx_ctrl.ant;
    frame.sig_len           = data->rx_ctrl.sig_len;
    frame.rx_state          = data->rx_ctrl.rx_state;

    memcpy(frame.mac,  data->mac,  6);
    memcpy(frame.dmac, data->dmac, 6);
    frame.first_word_invalid = data->first_word_invalid;

    uint16_t copy_len = (data->len < CSI_BUF_MAX) ? data->len : CSI_BUF_MAX;
    frame.len = copy_len;
    if (data->buf && copy_len > 0) {
        memcpy(frame.buf, data->buf, copy_len);
    }

    /* Bei voller Queue: ältesten Frame verwerfen, neuesten einreihen */
    if (uxQueueSpacesAvailable(s_csi_queue) == 0) {
        csi_frame_t dummy;
        xQueueReceive(s_csi_queue, &dummy, 0);
    }
    xQueueSend(s_csi_queue, &frame, 0);
}

/* ── Hilfsfunktion: csi_frame_t → Python-dict ─────────────────────────────── */

static mp_obj_t csi_frame_to_dict(const csi_frame_t *f) {
    mp_obj_t d = mp_obj_new_dict(0);

    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_mac),
                      mp_obj_new_bytes(f->mac, 6));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_dmac),
                      mp_obj_new_bytes(f->dmac, 6));

    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rssi),
                      MP_OBJ_NEW_SMALL_INT(f->rssi));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rate),
                      MP_OBJ_NEW_SMALL_INT(f->rate));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sig_mode),
                      MP_OBJ_NEW_SMALL_INT(f->sig_mode));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_mcs),
                      MP_OBJ_NEW_SMALL_INT(f->mcs));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_bandwidth),
                      MP_OBJ_NEW_SMALL_INT(f->cwb ? 40 : 20));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_smoothing),
                      mp_obj_new_bool(f->smoothing));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_not_sounding),
                      mp_obj_new_bool(f->not_sounding));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_aggregation),
                      mp_obj_new_bool(f->aggregation));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_stbc),
                      MP_OBJ_NEW_SMALL_INT(f->stbc));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_fec_coding),
                      mp_obj_new_bool(f->fec_coding));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sgi),
                      mp_obj_new_bool(f->sgi));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_noise_floor),
                      MP_OBJ_NEW_SMALL_INT(f->noise_floor));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_ampdu_cnt),
                      MP_OBJ_NEW_SMALL_INT(f->ampdu_cnt));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_channel),
                      MP_OBJ_NEW_SMALL_INT(f->channel));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_secondary_channel),
                      MP_OBJ_NEW_SMALL_INT(f->secondary_channel));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_timestamp),
                      mp_obj_new_int_from_uint(f->timestamp));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_ant),
                      MP_OBJ_NEW_SMALL_INT(f->ant));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sig_len),
                      MP_OBJ_NEW_SMALL_INT(f->sig_len));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rx_state),
                      MP_OBJ_NEW_SMALL_INT(f->rx_state));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_first_word_invalid),
                      mp_obj_new_bool(f->first_word_invalid));

    /* Rohdaten als bytearray: Python kann direkt indizieren */
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_data),
                      mp_obj_new_bytearray(f->len, (const byte *)f->buf));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_len),
                      MP_OBJ_NEW_SMALL_INT(f->len));

    return d;
}

/* ── Python: csi.init() ──────────────────────────────────────────────────── */

static mp_obj_t csi_init(void) {
    if (s_initialized) {
        return mp_const_none;
    }
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
    }
    if (!s_csi_queue) {
        s_csi_queue = xQueueCreate(CSI_QUEUE_SIZE, sizeof(csi_frame_t));
        if (!s_csi_queue) {
            mp_raise_OSError(MP_ENOMEM);
        }
    }
    esp_err_t err = esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_OSError,
            MP_ERROR_TEXT("esp_wifi_set_csi_rx_cb: 0x%x"), err);
    }
    s_initialized = true;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(csi_init_obj, csi_init);

/* ── Python: csi.config(...) ─────────────────────────────────────────────── */

static mp_obj_t csi_config(size_t n_args, const mp_obj_t *pos_args,
                            mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_lltf,           MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}  },
        { MP_QSTR_htltf,          MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}  },
        { MP_QSTR_stbc_htltf2,    MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}  },
        { MP_QSTR_ltf_merge,      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_channel_filter, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_manu_scale,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_shift,          MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int  = 0}     },
        { MP_QSTR_dump_ack,       MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    wifi_csi_config_t cfg = {
        .lltf_en           = args[0].u_bool,
        .htltf_en          = args[1].u_bool,
        .stbc_htltf2_en    = args[2].u_bool,
        .ltf_merge_en      = args[3].u_bool,
        .channel_filter_en = args[4].u_bool,
        .manu_scale        = args[5].u_bool,
        .shift             = (uint8_t)args[6].u_int,
        .dump_ack_en       = args[7].u_bool,
    };

    esp_err_t err = esp_wifi_set_csi_config(&cfg);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_OSError,
            MP_ERROR_TEXT("esp_wifi_set_csi_config: 0x%x"), err);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(csi_config_obj, 0, csi_config);

/* ── Python: csi.enable(True/False) ─────────────────────────────────────── */

static mp_obj_t csi_enable(mp_obj_t enable_obj) {
    bool enable = mp_obj_is_true(enable_obj);
    esp_err_t err = esp_wifi_set_csi(enable);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_OSError,
            MP_ERROR_TEXT("esp_wifi_set_csi: 0x%x"), err);
    }
    if (!enable && s_csi_queue) {
        xQueueReset(s_csi_queue);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(csi_enable_obj, csi_enable);

/* ── Python: csi.read(timeout_ms=1000) → dict | None ────────────────────── */

static mp_obj_t csi_read(size_t n_args, const mp_obj_t *args) {
    if (!s_csi_queue) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("csi.init() nicht aufgerufen"));
    }
    uint32_t timeout_ms = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 1000;
    TickType_t ticks = (timeout_ms == 0)          ? 0 :
                       (timeout_ms == 0xFFFFFFFF) ? portMAX_DELAY :
                                                    pdMS_TO_TICKS(timeout_ms);
    csi_frame_t frame;
    /* GIL freigeben damit andere MicroPython-Threads laufen können */
    MP_THREAD_GIL_EXIT();
    BaseType_t got = xQueueReceive(s_csi_queue, &frame, ticks);
    MP_THREAD_GIL_ENTER();

    if (got != pdTRUE) {
        return mp_const_none;   /* Timeout */
    }
    return csi_frame_to_dict(&frame);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(csi_read_obj, 0, 1, csi_read);

/* ── Callback-Task ───────────────────────────────────────────────────────── */
/*
 * Dieser FreeRTOS-Task wartet auf Frames in der Queue und ruft den
 * Python-Callback über mp_sched_schedule() auf.
 * mp_sched_schedule() ist thread-safe und kann aus beliebigem Kontext
 * aufgerufen werden (es setzt nur ein Flag im MicroPython-Scheduler).
 */

/* Wrapper-Objekt: wird von mp_sched_schedule als Callable aufgerufen */
static mp_obj_t csi_do_callback(mp_obj_t frame_dict) {
    if (s_callback != MP_OBJ_NULL && s_callback != mp_const_none) {
        mp_call_function_1(s_callback, frame_dict);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(csi_do_callback_obj, csi_do_callback);

/*
 * Der Callback-Task muss den GIL erwerben, um Python-Objekte zu erzeugen.
 * Wir nutzen MP_THREAD_GIL_ENTER / EXIT, die auf den esp32-Port
 * korrekt definiert sind (auch ohne MICROPY_PY_THREAD aktiv).
 */
static void csi_callback_task(void *arg) {
    csi_frame_t frame;
    for (;;) {
        if (xQueueReceive(s_csi_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (s_callback == MP_OBJ_NULL || s_callback == mp_const_none) {
            continue;
        }

        /* GIL erwerben, Python-Objekt bauen, schedulen, GIL freigeben */
        MP_THREAD_GIL_ENTER();
        mp_obj_t data = csi_frame_to_dict(&frame);
        mp_sched_schedule(MP_OBJ_FROM_PTR(&csi_do_callback_obj), data);
        MP_THREAD_GIL_EXIT();
    }
}

/* ── Python: csi.set_callback(func | None) ───────────────────────────────── */

static mp_obj_t csi_set_callback(mp_obj_t cb) {
    if (!s_csi_queue) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("csi.init() nicht aufgerufen"));
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (cb == mp_const_none) {
        s_callback = MP_OBJ_NULL;
        if (s_cb_task) {
            vTaskDelete(s_cb_task);
            s_cb_task = NULL;
        }
    } else {
        s_callback = cb;
        if (!s_cb_task) {
            xTaskCreate(csi_callback_task, "csi_cb",
                        4096, NULL,
                        configMAX_PRIORITIES - 5,
                        &s_cb_task);
        }
    }

    xSemaphoreGive(s_mutex);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(csi_set_callback_obj, csi_set_callback);

/* ── Python: csi.available() → int ──────────────────────────────────────── */

static mp_obj_t csi_available(void) {
    if (!s_csi_queue) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }
    return MP_OBJ_NEW_SMALL_INT((int)uxQueueMessagesWaiting(s_csi_queue));
}
static MP_DEFINE_CONST_FUN_OBJ_0(csi_available_obj, csi_available);

/* ── Python: csi.clear() ─────────────────────────────────────────────────── */

static mp_obj_t csi_clear(void) {
    if (s_csi_queue) {
        xQueueReset(s_csi_queue);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(csi_clear_obj, csi_clear);

/* ── Python: csi.parse_iq(data) → list of (I, Q) ────────────────────────── */
/*
 * CSI-Buffer-Layout: [I0, Q0, I1, Q1, ...] als vorzeichenbehaftete Bytes.
 * Gibt eine Liste von (I, Q)-Tupeln zurück.
 * Bei first_word_invalid=True sind iq[0] und iq[1] zu ignorieren.
 */
static mp_obj_t csi_parse_iq(mp_obj_t data_obj) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);

    const int8_t *raw = (const int8_t *)bufinfo.buf;
    size_t n_pairs = bufinfo.len / 2;

    mp_obj_list_t *lst = (mp_obj_list_t *)mp_obj_new_list(n_pairs, NULL);
    for (size_t i = 0; i < n_pairs; i++) {
        mp_obj_t iq[2] = {
            MP_OBJ_NEW_SMALL_INT(raw[2 * i]),
            MP_OBJ_NEW_SMALL_INT(raw[2 * i + 1]),
        };
        lst->items[i] = mp_obj_new_tuple(2, iq);
    }
    return MP_OBJ_FROM_PTR(lst);
}
static MP_DEFINE_CONST_FUN_OBJ_1(csi_parse_iq_obj, csi_parse_iq);

/* ── Modultabelle ────────────────────────────────────────────────────────── */

static const mp_rom_map_elem_t csi_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_csi)          },
    { MP_ROM_QSTR(MP_QSTR_init),         MP_ROM_PTR(&csi_init_obj)         },
    { MP_ROM_QSTR(MP_QSTR_config),       MP_ROM_PTR(&csi_config_obj)       },
    { MP_ROM_QSTR(MP_QSTR_enable),       MP_ROM_PTR(&csi_enable_obj)       },
    { MP_ROM_QSTR(MP_QSTR_read),         MP_ROM_PTR(&csi_read_obj)         },
    { MP_ROM_QSTR(MP_QSTR_set_callback), MP_ROM_PTR(&csi_set_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_available),    MP_ROM_PTR(&csi_available_obj)    },
    { MP_ROM_QSTR(MP_QSTR_clear),        MP_ROM_PTR(&csi_clear_obj)        },
    { MP_ROM_QSTR(MP_QSTR_parse_iq),     MP_ROM_PTR(&csi_parse_iq_obj)     },
    /* Konstanten für sig_mode */
    { MP_ROM_QSTR(MP_QSTR_SIG_11BG),     MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_SIG_HT),       MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_SIG_VHT),      MP_ROM_INT(3) },
};
static MP_DEFINE_CONST_DICT(csi_module_globals, csi_module_globals_table);

const mp_obj_module_t csi_user_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&csi_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_csi, csi_user_module);
