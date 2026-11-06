#define LOGF_ENABLE
#include "action.h"
#include "asm/thread.h"
#include "bluetooth/codecs.h"
#include "bt_hci.h"
#include "btc_av.h"
#include "button.h"
#include "esp_a2dp_api.h"
#include "audio.h"
#include "esp_avrc_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "list.h"
#include "logf.h"
#include "panic.h"
#include "pcm-internal.h"
#include "pcm.h"
#include "pcm_sink.h"
#include "queue.h"
#include "semaphore.h"
#include "splash.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "system.h"
#include "thread.h"
#include "tick.h"
#include "tlsf.h"
#include "misc.h"
#ifdef ROCKBOX_HAS_LOGF
#include "logfdisp.h"
#endif
#include "settings.h"
#include "lang.h"
#include "yesno.h"
#include "fixedpoint.h"
#include "pcm_sw_volume.h"
#include "mutex.h"
#include "file.h"
#include "string-extra.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BT_MAX_NAME_LEN 64
#define BT_MAX_ENTRIES 64

typedef struct {
    bool valid;
    esp_bd_addr_t addr;
    int8_t rssi;
    char name[BT_MAX_NAME_LEN];
} bt_list_entry_t;

static bt_list_entry_t bt_entries[BT_MAX_ENTRIES + 1];

static struct a2dp_codec *current_codec = NULL;

#define NUM_CODECS 4

static struct a2dp_codec *codec_list[NUM_CODECS] = {
    &a2dp_codec_ldac,
    &a2dp_codec_aptx_hd,
    &a2dp_codec_aptx,
    &a2dp_codec_sbc,
};

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

/* Remembered state: present while Bluetooth is on; holds the last connected
 * device's address once there is one. Removed when the user turns BT off. */
#define BT_STATE_FILE ROCKBOX_DIR "/bt_autoconnect.txt"

static struct mutex bt_enable_mtx;
static bool bt_ctrl_running = false;
static esp_bd_addr_t bt_last_addr;
static bool bt_have_last_addr = false;
static volatile bool bt_connected = false;
static volatile bool bt_rc_connected = false; /* AVRCP controller connected */
/* New pairings are only accepted while the device list is on screen; bonded
 * devices reconnect with their stored link key and never get here. */
static volatile bool bt_pairing_allowed = false;

static void bt_state_save(void)
{
    int fd = open(BT_STATE_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
        return;
    fdprintf(fd, "on\n");
    if (bt_have_last_addr)
        fdprintf(fd, ESP_BD_ADDR_STR "\n", ESP_BD_ADDR_HEX(bt_last_addr));
    close(fd);
}

static void bt_state_clear(void)
{
    remove(BT_STATE_FILE);
}

/* Returns true if Bluetooth was left on; fills bt_last_addr if known. */
static bool bt_state_load(void)
{
    char line[32];
    int fd = open(BT_STATE_FILE, O_RDONLY);
    if (fd < 0)
        return false;
    read_line(fd, line, sizeof(line));
    if (read_line(fd, line, sizeof(line)) > 0) {
        unsigned int b[ESP_BD_ADDR_LEN];
        if (sscanf(line, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2],
                   &b[3], &b[4], &b[5]) == ESP_BD_ADDR_LEN) {
            for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
                bt_last_addr[i] = b[i];
            bt_have_last_addr = true;
        }
    }
    close(fd);
    return true;
}

/* Fills bt_entries with the bonded (paired) devices */
static void bt_load_bonded(void)
{
    esp_bd_addr_t bonded_addrs[BT_MAX_ENTRIES];
    int num_bonded_addrs = BT_MAX_ENTRIES;
    esp_bt_gap_get_bond_device_list(&num_bonded_addrs, bonded_addrs);
    for (int i = 0; i < num_bonded_addrs; i++) {
        memcpy(bt_entries[i].addr, (bonded_addrs + i), ESP_BD_ADDR_LEN);
        size_t len = BT_MAX_NAME_LEN - 1;
        if (esp_bt_gap_get_bond_device_property(bt_entries[i].addr, "RbDeviceName", bt_entries[i].name, &len) == ESP_OK) {
            bt_entries[i].name[len] = 0;
        } else {
            strncpy(bt_entries[i].name, "Unknown Paired Device", BT_MAX_NAME_LEN);
        }
        bt_entries[i].rssi = -128;
        bt_entries[i].valid = true;
    }
    bt_entries[num_bonded_addrs].valid = false;
}

/* Brings up the controller and Bluedroid if they aren't already. Safe to call
 * from the UI and from the autostart thread at the same time. */
static bool bluetooth_enable(bool show_splash)
{
    esp_err_t ret;
    if (show_splash &&
        esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        splashf(0, "Enabling bluetooth...");
    }
    mutex_lock(&bt_enable_mtx);
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        if (!bt_hci_enable()) {
            mutex_unlock(&bt_enable_mtx);
            if (show_splash)
                splash(2*HZ, "Bluetooth failed to start");
            return false;
        }
        esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        cfg.sc_en = true;
        if ((ret = esp_bluedroid_init_with_cfg(&cfg)) != ESP_OK) {
            panicf("%s initialize bluedroid failed: %d", __func__, ret);
        }
        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
            panicf("%s enable bluedroid failed: %d", __func__, ret);
        }
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        if ((ret = esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t))) != ESP_OK) {
            panicf("%s set security param failed: %d", __func__, ret);
        }
        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
        esp_bt_pin_code_t pin_code;
        esp_bt_gap_set_pin(pin_type, 0, pin_code);
        if ((ret = esp_bt_gap_register_callback(bt_app_gap_cb)) != ESP_OK) {
            panicf("%s set gap cb failed: %d", __func__, ret);
        }
        char device_name[32] = "Rockbox";
        int fd = open_utf8(ROCKBOX_DIR "/playername.txt", O_RDONLY);
        if (fd >= 0) {
            read_line(fd, device_name, sizeof(device_name));
            close(fd);
            device_name[sizeof(device_name)-1] = 0;
        }
        if ((ret = esp_bt_gap_set_device_name(device_name)) != ESP_OK) {
            panicf("%s set device name failed: %d", __func__, ret);
        }
        if ((ret = esp_avrc_tg_init()) != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }
        if ((ret = esp_avrc_tg_register_callback(bt_app_avrc_tg_cb)) != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }

        esp_avrc_psth_bit_mask_t psth = {0};
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_PLAY);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_PAUSE);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_STOP);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_FORWARD);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_BACKWARD);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_FAST_FORWARD);
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &psth, ESP_AVRC_PT_CMD_REWIND);
        ret = esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD,
                                                &psth);
        if (ret != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                            ESP_AVRC_RN_PLAY_STATUS_CHANGE);
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                            ESP_AVRC_RN_TRACK_CHANGE);
        ret = esp_avrc_tg_set_rn_evt_cap(&evt_set);
        if (ret != ESP_OK) {
            panicf("%s avrcp tg init failed: %d", __func__, ret);
        }

        if ((ret = esp_a2d_source_init()) != ESP_OK) {
            panicf("%s a2dp source init failed: %d", __func__, ret);
        }
        if ((ret = esp_a2d_register_callback(bt_app_a2d_cb)) != ESP_OK) {
            panicf("%s a2dp register callback failed: %d", __func__, ret);
        }
        esp_a2d_mcc_t mcc;
        mcc.losc = A2D_SBC_INFO_LEN;
        mcc.media_type = A2D_MEDIA_TYPE_AUDIO;
        mcc.codec_type = ESP_A2D_MCT_SBC;
        mcc.cie.sbc_info.samp_freq    = ESP_A2D_SBC_CIE_SF_44K | ESP_A2D_SBC_CIE_SF_48K;
        mcc.cie.sbc_info.ch_mode      = ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
        mcc.cie.sbc_info.block_len    = ESP_A2D_SBC_CIE_BLOCK_LEN_16;
        mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
        mcc.cie.sbc_info.alloc_mthd   = ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
        mcc.cie.sbc_info.min_bitpool  = 3;
        mcc.cie.sbc_info.max_bitpool  = 53;
        if ((ret = esp_a2d_source_register_stream_endpoint(0, &mcc)) != ESP_OK) {
            panicf("%s a2dp register stream endpoint failed: %d", __func__, ret);
        }
        /* Connectable (but not discoverable) so bonded headphones can reconnect
         * on their own, e.g. when taken out of their case. */
        ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                       ESP_BT_NON_DISCOVERABLE);
        if (ret != ESP_OK) {
            panicf("%s set scan mode failed: %d", __func__, ret);
        }
        bt_load_bonded();
        bt_state_save();
    }
    mutex_unlock(&bt_enable_mtx);
    return true;
}

static struct semaphore bt_a2dp_deinit_sem;

/* Connection result for bt_connect_wait() */
static struct semaphore bt_connect_sem;
static esp_bd_addr_t bt_connect_addr;
static volatile bool bt_connect_waiting = false;

/* Shuts Bluetooth down (the sequence the debug device list used). A2DP deinit
 * completes asynchronously; wait for ESP_A2D_PROF_STATE_EVT before disabling
 * Bluedroid, or enabling again soon after fails with ESP_ERR_INVALID_STATE. */
static void bluetooth_disable(void)
{
    mutex_lock(&bt_enable_mtx);
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_bt_gap_cancel_discovery();
        semaphore_wait(&bt_a2dp_deinit_sem, TIMEOUT_NOBLOCK); /* drop stale */
        if (esp_a2d_source_deinit() == ESP_OK) {
            if (semaphore_wait(&bt_a2dp_deinit_sem, 2*HZ) != OBJ_WAIT_SUCCEEDED)
                logf("bt: a2dp deinit event timed out");
        }
        esp_avrc_tg_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        bt_hci_disable();
        bt_connected = false;
        bt_rc_connected = false;
    }
    mutex_unlock(&bt_enable_mtx);
}

void bluetooth_enable_discover(void)
{
    esp_err_t ret;
    if (!bluetooth_enable(true))
        return;
    if ((ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0)) != ESP_OK) {
        panicf("%s start discovery failed: %d", __func__, ret);
    }
}

#define BT_AUTOCONNECT_TRIES    12
#define BT_AUTOCONNECT_INTERVAL (10*HZ)

/* Bluetooth control thread: does the background start/reconnect at boot and
 * issues A2DP media control on behalf of the PCM sink. bt_sink_play/stop can be
 * called from interrupt context (e.g. USB audio starts the mixer from the USB
 * ISR), where calling into Bluedroid is not allowed, so they only queue_post()
 * here. */
enum {
    BT_CTRL_AUTOSTART = 1,
    BT_CTRL_MEDIA,          /* data: esp_a2d_media_ctrl_t */
    BT_CTRL_STOPPED,        /* PCM stopped; suspend the stream if it stays so */
    BT_CTRL_CODEC_PENDING,  /* codec setup waits for the stack to settle */
};

/* Fallback for a codec setup deferred by the A2DP callback: normally it runs
 * on ESP_A2D_AUDIO_CFG_EVT once the stack's own reconfiguration has finished,
 * but when the stack doesn't reconfigure at all no such event follows. */
#define BT_CODEC_SETUP_FALLBACK HZ
static void bt_codec_setup_if_pending(void);

/* Keep the A2DP stream running (sending silence) this long after PCM stops.
 * Restarting it takes a few hundred ms of AVDTP round trips, so suspending on
 * every brief stop (track change, USB audio underrun) loses audio. */
#define BT_SUSPEND_DELAY (3*HZ)

static volatile bool bt_pcm_playing = false;
static volatile bool bt_stop_pending = false;  /* stopped, stream still held */
bool bt_sink_suspend(void);

static struct event_queue bt_ctrl_queue;
static uint32_t bt_ctrl_stack[0x3000 / sizeof(uint32_t)];

/* What is playing, for the display of an AVRCP controller such as a car.
 * Polled on bt_ctrl, as nothing announces pause or resume. Read from the
 * Bluedroid callback: threads only switch when they yield, and nothing here
 * does, so it is never seen half written. */
#define BT_AVRC_POLL_INTERVAL (HZ/2)
#define BT_AVRC_STR_LEN 128

static struct {
    esp_avrc_playback_stat_t play_status;
    char path[MAX_PATH];
    char title[BT_AVRC_STR_LEN];
    char artist[BT_AVRC_STR_LEN];
    char album[BT_AVRC_STR_LEN];
    char genre[BT_AVRC_STR_LEN];
    char tracknum[12];
    char length[12];
} bt_avrc_np;

static esp_avrc_playback_stat_t bt_avrc_play_status(void)
{
    int status = audio_status();
    if (!(status & AUDIO_STATUS_PLAY))
        return ESP_AVRC_PLAYBACK_STOPPED;
    return (status & AUDIO_STATUS_PAUSE) ? ESP_AVRC_PLAYBACK_PAUSED
                                         : ESP_AVRC_PLAYBACK_PLAYING;
}

static struct mp3entry *bt_avrc_track(void)
{
    if (bt_avrc_play_status() == ESP_AVRC_PLAYBACK_STOPPED)
        return NULL;
    struct mp3entry *id3 = audio_current_track();
    return (id3 && id3->path[0]) ? id3 : NULL;
}

/* Copy a tag, dropping a UTF-8 sequence that didn't fit whole */
static void bt_avrc_copy_str(char *dst, size_t size, const char *src)
{
    size_t len = strlcpy(dst, src ? src : "", size);
    if (len >= size) {
        len = size - 1;
        while (len > 0 && ((unsigned char)dst[len - 1] & 0xC0) == 0x80)
            len--;
        if (len > 0)
            len--; /* the lead byte, or the last plain character */
        dst[len] = '\0';
    }
}

/* Returns true if the track changed */
static bool bt_avrc_update_track(void)
{
    struct mp3entry *id3 = bt_avrc_track();
    const char *path = id3 ? id3->path : "";
    if (!strcmp(bt_avrc_np.path, path))
        return false;

    strlcpy(bt_avrc_np.path, path, sizeof(bt_avrc_np.path));
    const char *title = id3 ? id3->title : NULL;
    if (id3 && !title) {
        title = strrchr(path, '/');
        title = title ? title + 1 : path;
    }
    bt_avrc_copy_str(bt_avrc_np.title, BT_AVRC_STR_LEN, title);
    bt_avrc_copy_str(bt_avrc_np.artist, BT_AVRC_STR_LEN, id3 ? id3->artist : NULL);
    bt_avrc_copy_str(bt_avrc_np.album, BT_AVRC_STR_LEN, id3 ? id3->album : NULL);
    bt_avrc_copy_str(bt_avrc_np.genre, BT_AVRC_STR_LEN, id3 ? id3->genre_string : NULL);
    bt_avrc_np.tracknum[0] = '\0';
    bt_avrc_np.length[0] = '\0';
    if (id3 && id3->tracknum > 0)
        snprintf(bt_avrc_np.tracknum, sizeof(bt_avrc_np.tracknum), "%d", id3->tracknum);
    if (id3)
        snprintf(bt_avrc_np.length, sizeof(bt_avrc_np.length), "%lu", id3->length);
    return true;
}

/* Without browsing, AVRCP identifies the selected track as all zeros and no
 * track as all ones. */
static void bt_avrc_send_track(esp_avrc_rn_rsp_t rsp)
{
    esp_avrc_rn_param_t rn;
    memset(rn.elm_id, bt_avrc_np.path[0] ? 0x00 : 0xFF, sizeof(rn.elm_id));
    esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_TRACK_CHANGE, rsp, &rn);
}

static void bt_avrc_send_play_status(esp_avrc_rn_rsp_t rsp)
{
    esp_avrc_rn_param_t rn;
    rn.playback = bt_avrc_np.play_status;
    esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE, rsp, &rn);
}

/* A CHANGED response is dropped unless the controller has registered for it */
static void bt_avrc_poll(void)
{
    esp_avrc_playback_stat_t play_status = bt_avrc_play_status();
    if (play_status != bt_avrc_np.play_status) {
        bt_avrc_np.play_status = play_status;
        bt_avrc_send_play_status(ESP_AVRC_RN_RSP_CHANGED);
    }
    if (bt_avrc_update_track())
        bt_avrc_send_track(ESP_AVRC_RN_RSP_CHANGED);
}

static void bt_sink_media_ctrl(esp_a2d_media_ctrl_t ctrl);

static void bt_ctrl_task(void)
{
    struct queue_event ev;
    int autoconnect_tries = 0;
    long suspend_at = 0;
    long codec_setup_at = 0;
    long avrc_poll_at = current_tick;

    while (true) {
        int tmo = autoconnect_tries > 0 ? BT_AUTOCONNECT_INTERVAL
                                        : TIMEOUT_BLOCK;
        if (suspend_at) {
            int left = suspend_at - current_tick;
            if (left < 1)
                left = 1;
            if (tmo == TIMEOUT_BLOCK || left < tmo)
                tmo = left;
        }
        if (codec_setup_at) {
            int left = codec_setup_at - current_tick;
            if (left < 1)
                left = 1;
            if (tmo == TIMEOUT_BLOCK || left < tmo)
                tmo = left;
        }
        /* bt_connected too: a timeout also drives the autoconnect retries */
        bool avrc_active = bt_connected && bt_rc_connected;
        if (avrc_active) {
            int left = avrc_poll_at - current_tick;
            if (left < 1)
                left = 1;
            if (tmo == TIMEOUT_BLOCK || left < tmo)
                tmo = left;
        }
        queue_wait_w_tmo(&bt_ctrl_queue, &ev, tmo);

        if (avrc_active && TIME_AFTER(current_tick, avrc_poll_at)) {
            avrc_poll_at = current_tick + BT_AVRC_POLL_INTERVAL;
            bt_avrc_poll();
        }

        if (suspend_at && TIME_AFTER(current_tick, suspend_at)) {
            suspend_at = 0;
            int oldlevel = disable_irq_save();
            if (bt_stop_pending && !bt_pcm_playing) {
                bt_stop_pending = false;
                bt_sink_suspend();
            }
            restore_irq(oldlevel);
        }

        if (codec_setup_at && TIME_AFTER(current_tick, codec_setup_at)) {
            codec_setup_at = 0;
            bt_codec_setup_if_pending();
        }

        switch (ev.id) {
            case BT_CTRL_CODEC_PENDING:
                codec_setup_at = current_tick + BT_CODEC_SETUP_FALLBACK;
                break;
            case BT_CTRL_STOPPED:
                suspend_at = current_tick + BT_SUSPEND_DELAY;
                break;
            case BT_CTRL_AUTOSTART:
                if (bluetooth_enable(false) && bt_have_last_addr)
                    autoconnect_tries = BT_AUTOCONNECT_TRIES;
                /* fall through to try right away */
            case SYS_TIMEOUT:
                if (autoconnect_tries > 0) {
                    if (bt_connected || !bt_have_last_addr ||
                        esp_bluedroid_get_status() !=
                            ESP_BLUEDROID_STATUS_ENABLED) {
                        autoconnect_tries = 0;
                        break;
                    }
                    autoconnect_tries--;
                    esp_a2d_source_connect(bt_last_addr);
                }
                break;
            case BT_CTRL_MEDIA:
                if (bt_connected && esp_bluedroid_get_status() ==
                                        ESP_BLUEDROID_STATUS_ENABLED)
                    bt_sink_media_ctrl((esp_a2d_media_ctrl_t)ev.data);
                break;
        }
    }
}

/* Called once at boot: starts the control thread and, if Bluetooth was on,
 * brings it up in the background and reconnects to the last device. */
void bt_autostart(void)
{
    if (!bt_hci_available())
        return;
    mutex_init(&bt_enable_mtx);
    semaphore_init(&bt_a2dp_deinit_sem, 1, 0);
    semaphore_init(&bt_connect_sem, 1, 0);
    queue_init(&bt_ctrl_queue, false);
    create_thread(bt_ctrl_task, bt_ctrl_stack, sizeof(bt_ctrl_stack), 0,
                  "bt_ctrl" IF_PRIO(, PRIORITY_BLUETOOTH));
    bt_ctrl_running = true;
    bt_state_load(); /* last device, for reconnecting */
    if (global_settings.bluetooth)
        queue_post(&bt_ctrl_queue, BT_CTRL_AUTOSTART, 0);
}

/* "Enable Bluetooth" setting callback, run once the user selects a value.
 * Also called while settings load at boot, before bt_autostart(); that case
 * is left to bt_autostart(). */
void bt_set_enabled(bool enabled)
{
    if (!bt_ctrl_running)
        return;
    if (enabled) {
        if (bluetooth_enable(true))
            queue_post(&bt_ctrl_queue, BT_CTRL_AUTOSTART, 0); /* reconnect */
        else
            global_settings.bluetooth = false;
    } else {
        splash(0, ID2P(LANG_BLUETOOTH_DISABLING));
        bluetooth_disable();
    }
}

bool bt_is_available(void)
{
    return bt_hci_available();
}

int bt_action_callback(int action, struct gui_synclist *lists)
{
    switch (action) {
        case ACTION_REDRAW: {
            int size = 0;
            while (bt_entries[size].valid && size < BT_MAX_ENTRIES) {
                size++;
            }
            gui_synclist_set_nb_items(lists, size * 2);
            return ACTION_REDRAW;
        }
        case ACTION_STD_OK: {
            bt_list_entry_t *entry = (bt_entries + (lists->selected_item / 2));
            esp_err_t ret;
            if ((ret = esp_a2d_source_connect(entry->addr)) != ESP_OK) {
                panicf("%s a2dp connect failed: %d", __func__, ret);
            }
            return ACTION_NONE;
        }
        case ACTION_STD_CONTEXT: {
            bt_list_entry_t *entry = (bt_entries + (lists->selected_item / 2));
            esp_err_t ret;
            if ((ret = esp_a2d_source_disconnect(entry->addr)) != ESP_OK) {
                panicf("%s a2dp disconnect failed: %d", __func__, ret);
            }
            return ACTION_NONE;
        }
        case ACTION_STD_MENU: {
            bt_state_clear();
            bt_have_last_addr = false;
            bluetooth_disable();
            return ACTION_STD_CANCEL;
        }
        default:
            return action;
    }
}

const char * bt_getname(int selected_item, void * data, char * buffer, size_t buffer_len)
{
    bt_list_entry_t *entry = (bt_entries + (selected_item / 2));
    if (selected_item % 2 == 0) {
        return entry->name;
    } else {
        if (entry->rssi == -128) {
            snprintf(buffer, buffer_len, ESP_BD_ADDR_STR " (Saved)", ESP_BD_ADDR_HEX(entry->addr));
        } else {
            snprintf(buffer, buffer_len, "%d dBm " ESP_BD_ADDR_STR, entry->rssi, ESP_BD_ADDR_HEX(entry->addr));
        }
        return buffer;
    }
}

bool bt_scan(void)
{
    struct simplelist_info info;


    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED &&
        !bluetooth_enable(true))
        return false;
    bluetooth_enable_discover();
    int size = 0;
    while (bt_entries[size].valid && size < BT_MAX_ENTRIES) {
        size++;
    }

    simplelist_info_init(&info, "Bluetooth devices:", size, NULL);
    info.scroll_all = false;
    info.selection_size = 2;
    info.action_callback = bt_action_callback;
    info.get_name = bt_getname;
    bt_pairing_allowed = true;
    bool ret = simplelist_show_list(&info);
    bt_pairing_allowed = false;
    return ret;
}

static int bt_entry_count(void)
{
    int size = 0;
    while (bt_entries[size].valid && size < BT_MAX_ENTRIES) {
        size++;
    }
    return size;
}

/* Set while a device list is shown, so connection changes redraw it */
static volatile bool bt_device_list_shown = false;

/* Pair New Device: lists only devices that are not paired yet */
static volatile bool bt_discover_unpaired_only = false;
static esp_bd_addr_t bt_bonded_addrs[BT_MAX_ENTRIES];
static int bt_num_bonded = 0;

#define BT_CONNECT_TIMEOUT (20*HZ) /* paging, pairing and AVDTP setup */

static void bt_snapshot_bonded(void)
{
    bt_num_bonded = BT_MAX_ENTRIES;
    esp_bt_gap_get_bond_device_list(&bt_num_bonded, bt_bonded_addrs);
}

static bool bt_is_bonded(const uint8_t *addr)
{
    for (int i = 0; i < bt_num_bonded; i++) {
        if (memcmp(bt_bonded_addrs[i], addr, ESP_BD_ADDR_LEN) == 0)
            return true;
    }
    return false;
}

static bool bt_is_connected_to(const uint8_t *addr)
{
    return bt_connected &&
           memcmp(bt_last_addr, addr, ESP_BD_ADDR_LEN) == 0;
}

static void bt_remove_entry(int idx)
{
    while (idx < BT_MAX_ENTRIES && bt_entries[idx].valid) {
        bt_entries[idx] = bt_entries[idx + 1];
        idx++;
    }
}

/* Requests a connect or disconnect and waits until addr ends up connected or
 * disconnected */
static void bt_request_and_wait(uint8_t *addr,
                                esp_err_t (*request)(esp_bd_addr_t))
{
    semaphore_wait(&bt_connect_sem, TIMEOUT_NOBLOCK); /* drop stale */
    memcpy(bt_connect_addr, addr, ESP_BD_ADDR_LEN);
    bt_connect_waiting = true;
    if (request(addr) == ESP_OK)
        semaphore_wait(&bt_connect_sem, BT_CONNECT_TIMEOUT);
    bt_connect_waiting = false;
}

/* Connects to addr (pairing first if needed) with a popup until it succeeds
 * or fails */
static bool bt_connect_wait(uint8_t *addr)
{
    splash(0, ID2P(LANG_BLUETOOTH_CONNECTING));
    bt_request_and_wait(addr, esp_a2d_source_connect);
    if (!bt_is_connected_to(addr)) {
        splash(2*HZ, ID2P(LANG_BLUETOOTH_CONNECT_FAILED));
        return false;
    }
    return true;
}

static void bt_disconnect_wait(uint8_t *addr)
{
    splash(0, ID2P(LANG_BLUETOOTH_DISCONNECTING));
    bt_request_and_wait(addr, esp_a2d_source_disconnect);
}

enum {
    BT_DEVICE_ACTION_CONNECTION, /* Connect or Disconnect */
    BT_DEVICE_ACTION_FORGET,
    BT_NUM_DEVICE_ACTIONS
};

static const char *bt_device_action_get_name(int selected_item, void *data,
                                             char *buffer, size_t buffer_len)
{
    (void)buffer;
    (void)buffer_len;
    if (selected_item == BT_DEVICE_ACTION_FORGET)
        return str(LANG_BLUETOOTH_FORGET);
    return bt_is_connected_to(data) ? str(LANG_BLUETOOTH_DISCONNECT)
                                    : str(LANG_BLUETOOTH_CONNECT);
}

/* Paired Devices > device: Connect or Disconnect, and Forget */
static void bt_device_actions(int idx)
{
    esp_bd_addr_t addr;
    struct simplelist_info info;
    memcpy(addr, bt_entries[idx].addr, ESP_BD_ADDR_LEN);
    simplelist_info_init(&info, bt_entries[idx].name, BT_NUM_DEVICE_ACTIONS,
                         addr);
    info.get_name = bt_device_action_get_name;
    simplelist_show_list(&info);

    switch (info.selection) {
        case BT_DEVICE_ACTION_CONNECTION:
            if (bt_is_connected_to(addr))
                bt_disconnect_wait(addr);
            else
                bt_connect_wait(addr);
            break;
        case BT_DEVICE_ACTION_FORGET:
            if (!yesno_pop(ID2P(LANG_BLUETOOTH_FORGET_QUERY)))
                break;
            if (bt_is_connected_to(addr))
                bt_disconnect_wait(addr);
            esp_bt_gap_remove_bond_device(addr);
            if (bt_have_last_addr &&
                memcmp(bt_last_addr, addr, ESP_BD_ADDR_LEN) == 0) {
                bt_have_last_addr = false; /* don't reconnect to it */
                bt_state_save();
            }
            bt_remove_entry(idx);
            break;
    }
}

static int bt_device_list_cb(int action, struct gui_synclist *lists)
{
    switch (action) {
        case ACTION_REDRAW: /* device found, or a connection changed */
            gui_synclist_set_nb_items(lists, bt_entry_count());
            return ACTION_REDRAW;
        case ACTION_STD_OK:
            return ACTION_STD_CANCEL; /* hand the selection to the caller */
        default:
            return action;
    }
}

static const char *bt_device_get_name(int selected_item, void *data,
                                      char *buffer, size_t buffer_len)
{
    (void)data;
    bt_list_entry_t *entry = &bt_entries[selected_item];
    if (bt_is_connected_to(entry->addr)) {
        snprintf(buffer, buffer_len, "%s (%s)", entry->name,
                 str(LANG_BLUETOOTH_CONNECTED));
        return buffer;
    }
    return entry->name;
}

/* Shows bt_entries by name. Selecting a paired device opens its actions; with
 * pairing set, it connects (pairing first): discovery is stopped before that
 * and the device is removed from the list once paired. */
static void bt_device_list(char *title, bool pairing)
{
    struct simplelist_info info;
    simplelist_info_init(&info, title, bt_entry_count(), NULL);
    info.action_callback = bt_device_list_cb;
    info.get_name = bt_device_get_name;

    bt_device_list_shown = true;
    while (true) {
        simplelist_show_list(&info);
        if (info.selection < 0 || info.selection >= bt_entry_count())
            break;
        uint8_t *addr = bt_entries[info.selection].addr;
        if (!pairing) {
            bt_device_actions(info.selection);
        } else if (!bt_is_connected_to(addr)) {
            esp_bt_gap_cancel_discovery(); /* it slows down connecting */
            if (bt_connect_wait(addr)) {
                bt_snapshot_bonded();
                if (bt_is_bonded(addr))
                    bt_remove_entry(info.selection);
            }
        }
        info.count = bt_entry_count();
    }
    bt_device_list_shown = false;
}

/* Settings > Bluetooth Settings > Paired Devices */
int bt_paired_devices(void)
{
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED)
        return 0;
    mutex_lock(&bt_enable_mtx);
    bt_load_bonded();
    mutex_unlock(&bt_enable_mtx);
    bt_device_list(str(LANG_BLUETOOTH_PAIRED_DEVICES), false);
    return 0;
}

/* Settings > Bluetooth Settings > Pair New Device */
int bt_pair_new_device(void)
{
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED)
        return 0;
    bt_entries[0].valid = false;
    bt_snapshot_bonded();
    bt_discover_unpaired_only = true;
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0);
    bt_pairing_allowed = true;
    bt_device_list(str(LANG_BLUETOOTH_PAIR_NEW_DEVICE), true);
    bt_pairing_allowed = false;
    esp_bt_gap_cancel_discovery();
    bt_discover_unpaired_only = false;
    mutex_lock(&bt_enable_mtx);
    bt_load_bonded(); /* what the debug device list expects to start from */
    mutex_unlock(&bt_enable_mtx);
    return 0;
}

static void process_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    uint32_t cod = 0;     /* class of device */
    uint8_t *eir = NULL;
    esp_bt_gap_dev_prop_t *p;
    int8_t rssi;

    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(uint8_t *)(p->val);
        default:
            break;
        }
    }

    // /* search for device with MAJOR service class as "rendering" and "audio" in COD */
    // if (!esp_bt_gap_is_valid_cod(cod) ||
    //         !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING) ||
    //         !(esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_AUDIO)) {
    //     return;
    // }

    if (!esp_bt_gap_is_valid_cod(cod) || eir == NULL) {
        return;
    }

    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    /* get complete or short local name from eir data */
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }
    if (!rmt_bdname) {
        return; // Don't show nameless devices
    }

    if (bt_discover_unpaired_only && bt_is_bonded(param->disc_res.bda)) {
        return;
    }

    int bdname_len = MIN(rmt_bdname_len, BT_MAX_NAME_LEN - 1);

    int i = 0;
    while (i < BT_MAX_ENTRIES && (bt_entries[i].valid && (memcmp(bt_entries[i].addr, param->disc_res.bda, ESP_BD_ADDR_LEN) != 0))) {
        i++;
    }
    if (i < BT_MAX_ENTRIES) {
        memcpy(bt_entries[i].addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
        memcpy(bt_entries[i].name, rmt_bdname, bdname_len);
        bt_entries[i].name[bdname_len] = '\0';
        bt_entries[i].rssi = rssi;
        bt_entries[i].valid = true;
        button_queue_post(BUTTON_REDRAW, 0);
    }
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            logf("gap %d " ESP_BD_ADDR_STR, event, ESP_BD_ADDR_HEX(param->disc_res.bda));
            process_inquiry_scan_result(param);
            return;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            logf("gap %d %d", event, param->disc_st_chg.state);
            return;
        case ESP_BT_GAP_CFM_REQ_EVT:
            logf("gap %d %d", event, param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda,
                                         bt_pairing_allowed);
            break;
        case ESP_BT_GAP_PIN_REQ_EVT: {
            logf("gap %d %d", event, param->pin_req.min_16_digit);
            if (!bt_pairing_allowed) {
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, pin_code);
            } else if (param->pin_req.min_16_digit) {
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '0';
                pin_code[1] = '0';
                pin_code[2] = '0';
                pin_code[3] = '0';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
            logf("gap %d %d", event, param->read_rmt_name.stat);
            if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
                param->read_rmt_name.rmt_name[BT_MAX_NAME_LEN - 1] = 0;
                esp_bt_gap_set_bond_device_property(param->read_rmt_name.bda, "RbDeviceName", param->read_rmt_name.rmt_name, strlen(param->read_rmt_name.rmt_name));
            }
            break;
        }
        default:
            logf("gap %d", event);
            return;
    }
}

extern unsigned char btbuf[];

int bt_heap_info_action_callback(int action, struct gui_synclist *lists)
{
    switch (action) {
        case ACTION_NONE:
            return ACTION_REDRAW;
        default:
            return action;
    }
}

const char * bt_heap_info_getname(int selected_item, void * data, char * buffer, size_t buffer_len)
{
    switch (selected_item) {
        case 0:
#ifndef SIMULATOR
            snprintf(buffer, buffer_len, "heap usage: %d/%d", get_used_size(btbuf), get_max_size(btbuf));
            return buffer;
#else
            return "this is sim build you dummy";
#endif
        case 1: {
            const uint8_t *addr = esp_bt_dev_get_address();
            if (addr) {
                snprintf(buffer, buffer_len, "MAC addr: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(addr));
                return buffer;
            } else {
                return "MAC addr: unknown";
            }
        }
        case 2:
            if (current_codec == &a2dp_codec_ldac) {
                return "current codec: LDAC";
            } else if (current_codec == &a2dp_codec_aptx_hd) {
                return "current codec: aptX HD";
            } else if (current_codec == &a2dp_codec_aptx) {
                return "current codec: aptX";
            } else if (current_codec == &a2dp_codec_sbc) {
                return "current codec: SBC";
            } else {
                return "current codec: unknown";
            }
        case 3:
            if (current_codec != NULL && current_codec->debug_info != NULL) {
                strcpy(buffer, "codec info: ");
                current_codec->debug_info(buffer + 12, buffer_len - 12);
                return buffer;
            } else {
                return "codec info: N/A";
            }
        case 4:
            snprintf(buffer, buffer_len, "real sink sampr: %lu", pcm_current_sink_caps()->samprs[pcm_get_current_sink()->configured_freq]);
            return buffer;
        default:
            return "Unknown item!!";
    }
}

bool bt_heap_info(void)
{
    struct simplelist_info info;

    simplelist_info_init(&info, "Bluetooth debug info:", 5, NULL);
    info.scroll_all = false;
    info.action_callback = bt_heap_info_action_callback;
    info.get_name = bt_heap_info_getname;
    return simplelist_show_list(&info);
}

static esp_a2d_audio_state_t bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;

/* Encoders (LDAC especially) need far more than DEFAULT_STACK_SIZE, which is
 * only 1 KiB on MIPS; overflowing it corrupts whatever is linked next to
 * it (the tagcache stack, in practice). */
#define BT_CODEC_STACK_SIZE 0x4000
uint32_t bt_codec_stack[BT_CODEC_STACK_SIZE / sizeof(uint32_t)];


static void bt_codec_task(void)
{
    int64_t next_ms = (((int64_t)current_tick) * 1000) / HZ;
    while (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED && bt_current_state == ESP_A2D_AUDIO_STATE_STARTED) {
        uint64_t ticked = 0;
        do {
            if (ticked > (1000 / HZ)) {
                ticked %= (1000 / HZ);
                yield();
            }
            if (current_codec == NULL) {
                thread_exit();
                return;
            }
            current_codec->tick();
            next_ms += current_codec->get_period();
            ticked += current_codec->get_period();
        } while ((next_ms * HZ) / 1000 <= current_tick);
        sleep(((next_ms * HZ) / 1000) - current_tick);
    }
    if (current_codec == NULL) {
        thread_exit();
        return;
    }
    current_codec->feeding_flush();
    thread_exit();
}

static esp_a2d_conn_hdl_t conn_hdl;
static bool sampr_switch_ongoing = false;
static struct semaphore sink_control_sem;
static struct event_queue sink_control_queue;
static esp_a2d_mcc_t new_pref_mcc;
struct pcm_sink bt_pcm_sink;
static esp_a2d_mcc_t peer_caps;

/* When the sink sets up the stream itself, the first capability report only
 * covers the endpoint it chose (usually SBC). Once the stack has discovered
 * all endpoints, switch to the best codec through the same init +
 * set_pref_mcc path used for source-initiated connections, after the stack's
 * own reconfiguration has settled (signalled by SRC_SET_PREF_MCC_EVT). */
static bool codec_ready = false;
static bool codec_setup_pending = false;
static uint16_t codec_peer_mtu;
static struct a2dp_codec *upgrade_codec = NULL;
static esp_a2d_mcc_t upgrade_caps;

static void bt_codec_setup(void)
{
    esp_a2d_mcc_t pref_mcc;
    struct a2dp_peer_info peer_info = {
        .peer_mtu = codec_peer_mtu,
        .peer_caps = &peer_caps,
        .conn_hdl = conn_hdl,
        .is_edr = btc_av_is_peer_edr(),
    };
    current_codec->init(&bt_pcm_sink.caps, &peer_info, &pref_mcc);

    /* The codecs list their rates highest first and default to the first.
     * Make the default 44.1 kHz, the rate of most music, when the sink has
     * it: for SBC that is also where the bitrate target gets bitpool 53. */
    for (int i = 0; i < (int)bt_pcm_sink.caps.num_samprs; i++) {
        if (bt_pcm_sink.caps.samprs[i] == SAMPR_44) {
            bt_pcm_sink.caps.default_freq = i;
            break;
        }
    }
    current_codec->set_freq(bt_pcm_sink.caps.samprs[bt_pcm_sink.caps.default_freq],
                            &pref_mcc);

    /* The rate index PCM last set is into the previous connection's rate
     * list, and the encoder was just reset: have PCM set the rate again when
     * it switches to us, or the encoder runs at its own default while PCM
     * feeds it something else. */
    bt_pcm_sink.configured_freq = -1U;
    codec_ready = true;
    esp_err_t ret = esp_a2d_source_set_pref_mcc(conn_hdl, &pref_mcc);
    if (ret != ESP_OK) {
        panicf("%s a2dp source set pref mcc failed: %d", __func__, ret);
    }
}

/* Runs a codec setup deferred from REPORT_SNK_ALL_CODEC_CAPS_EVT. That happens
 * when the sink set up the stream itself: the stack then reports the sink's
 * codecs only after the stream is open, so REPORT_SNK_CODEC_CAPS_EVT never
 * comes. Called on ESP_A2D_AUDIO_CFG_EVT (stack reconfiguration done) or from
 * the bt_ctrl fallback, whichever is first. */
static void bt_codec_setup_if_pending(void)
{
    if (codec_setup_pending && !codec_ready && bt_connected) {
        codec_setup_pending = false;
        logf("bt: codec setup after full discovery");
        bt_codec_setup();
    }
}

/* Sample rate changes can arrive faster than the sink confirms them (USB
 * audio sets 44.1k then the host's rate within milliseconds). One suspend is
 * held for the whole switch; each request bumps req_gen, and the switch only
 * completes once the latest request has been applied. */
static bool sampr_mcc_pending = false;
static unsigned int sampr_req_gen = 0;
static unsigned int sampr_applied_gen = 0;

static void bt_sink_apply_pref_mcc(void)
{
    sampr_applied_gen = sampr_req_gen;
    sampr_mcc_pending = true;
    esp_a2d_source_set_pref_mcc(conn_hdl, &new_pref_mcc);
}

static int sink_suspended = 1;

/* Runs on bt_ctrl: send now, or queue behind the command still awaiting ack. */
static void bt_sink_media_ctrl(esp_a2d_media_ctrl_t ctrl) {
    if (semaphore_wait(&sink_control_sem, TIMEOUT_NOBLOCK) ==
            OBJ_WAIT_SUCCEEDED) {
        esp_a2d_media_ctrl(ctrl);
    } else {
        queue_post(&sink_control_queue, 1, ctrl);
    }
}

/* May be called from interrupt context; see bt_ctrl_task. */
bool bt_sink_suspend(void) {
    if (++sink_suspended == 1) {
        logf("suspending playback: sink locked");
        queue_post(&bt_ctrl_queue, BT_CTRL_MEDIA, ESP_A2D_MEDIA_CTRL_SUSPEND);
        return true;
    }
    return false;
}
bool bt_sink_resume(void) {
    if (--sink_suspended == 0) {
        logf("starting playback: sink unlocked");
        queue_post(&bt_ctrl_queue, BT_CTRL_MEDIA, ESP_A2D_MEDIA_CTRL_START);
        return true;
    }
    return false;
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            logf("a2d %d %d", event, param->conn_stat.state);
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                bt_connected = true;
                conn_hdl = param->conn_stat.conn_hdl;
                codec_peer_mtu = param->conn_stat.audio_mtu;
                bt_pcm_playing = false;
                bt_stop_pending = false;
                memcpy(bt_last_addr, param->conn_stat.remote_bda,
                       ESP_BD_ADDR_LEN);
                bt_have_last_addr = true;
                bt_state_save();
                semaphore_init(&sink_control_sem, 1, 0);
                queue_init(&sink_control_queue, false);
                sink_suspended = 1;
                sampr_switch_ongoing = false;
                sampr_mcc_pending = false;
                codec_ready = false;
                codec_setup_pending = false;
                upgrade_codec = NULL;
                bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
                esp_bt_gap_read_remote_name(param->conn_stat.remote_bda);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED || param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
                bt_connected = false;
                codec_ready = false;
                upgrade_codec = NULL;
                if (current_codec != NULL) {
                    bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
                    current_codec->deinit();
                    current_codec = NULL;
                    pcm_switch_sink(PCM_SINK_BUILTIN);
                }
            }
            /* after the state above is updated */
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED ||
                param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                if (bt_connect_waiting &&
                    memcmp(bt_connect_addr, param->conn_stat.remote_bda,
                           ESP_BD_ADDR_LEN) == 0) {
                    bt_connect_waiting = false;
                    semaphore_release(&bt_connect_sem);
                }
                if (bt_device_list_shown)
                    button_queue_post(BUTTON_REDRAW, 0);
            }
            return;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            logf("a2d %d %d %d", event, param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
            if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
                pcm_switch_sink(PCM_SINK_BLUETOOTH);
            } else if (sampr_switch_ongoing && !sampr_mcc_pending &&
                       param->media_ctrl_stat.cmd ==
                           ESP_A2D_MEDIA_CTRL_SUSPEND) {
                bt_sink_apply_pref_mcc();
            }
            if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_SUSPEND && param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                bt_current_state = ESP_A2D_AUDIO_STATE_SUSPEND;
            } else if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START && param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                bt_current_state = ESP_A2D_AUDIO_STATE_STARTED;
                current_codec->feeding_reset();
                create_thread(bt_codec_task, bt_codec_stack,
                              sizeof(bt_codec_stack), 0, "bt_codec"
                              IF_PRIO(, PRIORITY_BLUETOOTH));
            }
            struct queue_event ev;
            queue_wait_w_tmo(&sink_control_queue, &ev, TIMEOUT_NOBLOCK);
            if (ev.id == 1) {
                esp_a2d_media_ctrl(ev.data);
            } else if (ev.id == SYS_TIMEOUT) {
                semaphore_release(&sink_control_sem);
            }
            return;
        case ESP_A2D_REPORT_SNK_CODEC_CAPS_EVT:
            codec_setup_pending = false;
            conn_hdl = param->a2d_report_snk_codec_caps_stat.conn_hdl;
            codec_peer_mtu = param->a2d_report_snk_codec_caps_stat.mtu;
            bt_codec_setup();
            break;
        case ESP_A2D_REPORT_SNK_ALL_CODEC_CAPS_EVT: {
            logf("a2d %d %d", event, param->a2d_report_snk_all_codec_caps_stat.sep_num);
            struct a2dp_codec *chosen = NULL;
            esp_a2d_mcc_t *chosen_caps = NULL;
            for (int codec_idx = 0; codec_idx < NUM_CODECS && !chosen;
                 codec_idx++) {
                for (uint32_t i = 0;
                     i < param->a2d_report_snk_all_codec_caps_stat.sep_num;
                     i++) {
                    esp_a2d_mcc_t *sink_mcc =
                        &param->a2d_report_snk_all_codec_caps_stat.sep_mcc[i].mcc;
                    if (codec_list[codec_idx]->is_acceptable(sink_mcc)) {
                        chosen = codec_list[codec_idx];
                        chosen_caps = sink_mcc;
                        break;
                    }
                }
            }
            if (!chosen) {
                panicf("Unable to initialize suitable codec");
            }
            if (!codec_ready) {
                current_codec = chosen;
                memcpy(&peer_caps, chosen_caps, sizeof(esp_a2d_mcc_t));
                if (bt_connected) {
                    /* stream already open: see bt_codec_setup_if_pending() */
                    codec_setup_pending = true;
                    queue_post(&bt_ctrl_queue, BT_CTRL_CODEC_PENDING, 0);
                }
            } else if (chosen != current_codec) {
                upgrade_codec = chosen;
                memcpy(&upgrade_caps, chosen_caps, sizeof(esp_a2d_mcc_t));
            }
            break;
        }
        case ESP_A2D_SRC_SET_PREF_MCC_EVT:
            logf("a2d %d %d", event, param->a2d_set_pref_mcc_stat.set_status);
            if (upgrade_codec != NULL && !sampr_switch_ongoing) {
                logf("bt: switching codec after full discovery");
                current_codec->deinit();
                current_codec = upgrade_codec;
                upgrade_codec = NULL;
                memcpy(&peer_caps, &upgrade_caps, sizeof(esp_a2d_mcc_t));
                bt_codec_setup();
                return;
            }
            if (sampr_switch_ongoing) {
                sampr_mcc_pending = false;
                if (sampr_applied_gen != sampr_req_gen) {
                    /* a newer rate was requested meanwhile; apply it too */
                    bt_sink_apply_pref_mcc();
                    return;
                }
                sampr_switch_ongoing = false;
                logf("starting playback: sampr switch done");
                bt_sink_resume();
            } else {
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
            }
            return;
        case ESP_A2D_PROF_STATE_EVT:
            logf("a2d %d %d", event, param->a2d_prof_stat.init_state);
            if (param->a2d_prof_stat.init_state == ESP_A2D_DEINIT_SUCCESS)
                semaphore_release(&bt_a2dp_deinit_sem);
            return;
        case ESP_A2D_AUDIO_CFG_EVT:
            logf("a2d %d", event);
            bt_codec_setup_if_pending();
            return;
        case ESP_A2D_AUDIO_STATE_EVT:
            logf("a2d %d %d\n", event, param->audio_stat.state);
        default:
            logf("a2d %d\n", event);
        return;
    }
}

static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    switch (event) {
        case ESP_AVRC_TG_CONNECTION_STATE_EVT:
            logf("avrc-tg %d %d", event, param->conn_stat.connected);
            bt_rc_connected = param->conn_stat.connected;
            break;
        case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
            logf("avrc-tg %d %d %d", event, param->rmt_feats.ct_feat_flag, param->rmt_feats.feat_mask);
            break;
        case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
            logf("avrc-tg %d %d %d", event, param->psth_cmd.key_code, param->psth_cmd.key_state);
            if (param->psth_cmd.key_state == 1) {
                switch (param->psth_cmd.key_code) {
                    /* Separate commands, not a toggle: a headset may send
                     * play on connecting, to playback already resumed */
                    case ESP_AVRC_PT_CMD_PLAY:
                        button_queue_post(BUTTON_MULTIMEDIA_PLAY, 0);
                        break;
                    case ESP_AVRC_PT_CMD_PAUSE:
                        button_queue_post(BUTTON_MULTIMEDIA_PAUSE, 0);
                        break;
                    case ESP_AVRC_PT_CMD_STOP:
                        button_queue_post(BUTTON_MULTIMEDIA_STOP, 0);
                        break;
                    case ESP_AVRC_PT_CMD_FORWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_NEXT, 0);
                        break;
                    case ESP_AVRC_PT_CMD_BACKWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_PREV, 0);
                        break;
                    case ESP_AVRC_PT_CMD_REWIND:
                        button_queue_post(BUTTON_MULTIMEDIA_REW, 0);
                        break;
                    case ESP_AVRC_PT_CMD_FAST_FORWARD:
                        button_queue_post(BUTTON_MULTIMEDIA_FFWD, 0);
                        break;
                }
            }
            break;
        case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
            logf("avrc-tg %d %d", event, param->reg_ntf.event_id);
            switch (param->reg_ntf.event_id) {
                case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
                    bt_avrc_np.play_status = bt_avrc_play_status();
                    bt_avrc_send_play_status(ESP_AVRC_RN_RSP_INTERIM);
                    break;
                case ESP_AVRC_RN_TRACK_CHANGE:
                    bt_avrc_update_track();
                    bt_avrc_send_track(ESP_AVRC_RN_RSP_INTERIM);
                    break;
            }
            break;
        case ESP_AVRC_TG_GET_PLAY_STATUS_EVT: {
            struct mp3entry *id3 = bt_avrc_track();
            param->get_play_status.play_status = bt_avrc_play_status();
            param->get_play_status.song_length = id3 ? id3->length : 0;
            param->get_play_status.song_position = id3 ? id3->elapsed : 0;
            break;
        }
        case ESP_AVRC_TG_GET_ELEM_ATTRS_EVT:
            logf("avrc-tg %d", event);
            bt_avrc_update_track();
            param->get_elem_attrs.value[0] = bt_avrc_np.title;
            param->get_elem_attrs.value[1] = bt_avrc_np.artist;
            param->get_elem_attrs.value[2] = bt_avrc_np.album;
            param->get_elem_attrs.value[3] = bt_avrc_np.tracknum;
            param->get_elem_attrs.value[5] = bt_avrc_np.genre;
            param->get_elem_attrs.value[6] = bt_avrc_np.length;
            break;
        default:
            logf("avrc-tg %d", event);
            return;
    }
}

static const void *pcm_data_start = NULL;
static size_t  pcm_data_size = 0;
static int     audio_locked = 0;
struct pcm_sink bt_pcm_sink;

void bt_sink_init(void) {
}

void bt_sink_postinit(void) {
}

void bt_sink_lock(void) {
    audio_locked++;
}

void bt_sink_unlock(void) {
    audio_locked--;
}

void bt_sink_set_freq(uint16_t freq) {
    logf("suspending playback: sampr switch start, to %d", freq);

    sampr_req_gen++;
    if (!sampr_switch_ongoing) {
        sampr_switch_ongoing = true;
        bt_sink_suspend();
    }
    current_codec->set_freq(bt_pcm_sink.caps.samprs[freq], &new_pref_mcc);

    if (bt_current_state == ESP_A2D_AUDIO_STATE_SUSPEND && !sampr_mcc_pending) {
        bt_sink_apply_pref_mcc();
    }
}

void bt_sink_play(const void* addr, size_t size) {
    pcm_data_start = addr;
    pcm_data_size = size;
    logf("starting playback: starting to play");
    bt_pcm_playing = true;
    if (bt_stop_pending) {
        bt_stop_pending = false; /* stream was held open; just keep going */
    } else {
        bt_sink_resume();
    }
    pcm_play_dma_status_callback(PCM_DMAST_STARTED);
}

void bt_sink_stop(void) {
    pcm_data_start = NULL;
    pcm_data_size = 0;
    if (!bt_pcm_playing)
        return;
    logf("suspending playback: stopped playing");
    bt_pcm_playing = false;
    bt_stop_pending = true;
    queue_post(&bt_ctrl_queue, BT_CTRL_STOPPED, 0);
}

static bool get_new_buf_maybe(void) {
    bool ret = pcm_play_dma_complete_callback(PCM_DMAST_OK, &pcm_data_start, &pcm_data_size);
    if (ret) {
        pcm_play_dma_status_callback(PCM_DMAST_STARTED);
    }
    return ret;
}

/* Volume for the Bluetooth sink, as a Q16 factor per channel. The A2DP
 * stream is 16-bit, so it can't go through the >16-bit software-volume path;
 * scale here instead. Set through the sink's set_volume op. */
#define BT_VOL_UNITY (1 << 16)
static volatile int32_t bt_vol_factor_l = BT_VOL_UNITY;
static volatile int32_t bt_vol_factor_r = BT_VOL_UNITY;

static int32_t bt_centibels_to_factor(int cb)
{
    if (cb == PCM_MUTE_LEVEL)
        return 0;
    if (cb >= 0)
        return BT_VOL_UNITY;
    return fp_factor(fp_div(cb, 10, 16), 16);
}

static void bt_sink_set_volume(int vol_l, int vol_r)
{
    bt_vol_factor_l = bt_centibels_to_factor(vol_l);
    bt_vol_factor_r = bt_centibels_to_factor(vol_r);
}

static void bt_apply_volume(int16_t *s, size_t bytes)
{
    int32_t fl = bt_vol_factor_l, fr = bt_vol_factor_r;
    if (fl == BT_VOL_UNITY && fr == BT_VOL_UNITY)
        return;
    for (size_t n = bytes / (2 * sizeof(int16_t)); n > 0; n--, s += 2) {
        s[0] = (int16_t)(((int32_t)s[0] * fl) >> 16);
        s[1] = (int16_t)(((int32_t)s[1] * fr) >> 16);
    }
}

size_t bt_read_pcm(uint8_t *buf, size_t len) {
    if (len <= 0 || buf == NULL) {
        return 0;
    }
    size_t written = 0;
    if (pcm_data_start != NULL && audio_locked == 0) {
        while (len > 0 && (pcm_data_size > 0 || get_new_buf_maybe())) {
            size_t sent = MIN(pcm_data_size, len);
            memcpy(buf, pcm_data_start, sent);
            bt_apply_volume((int16_t *)buf, sent);
            pcm_data_start += sent;
            pcm_data_size -= sent;
            buf += sent;
            len -= sent;
            written += sent;
        }
    } else {
        // Transmit digital silence
        written = len;
        memset(buf, 0, len);
    }
    return written;
}

struct pcm_sink bt_pcm_sink = {
    .caps = {
        .samprs       = NULL,
        .num_samprs   = -1,
        .default_freq = -1,
        .volume_type  = -1,
    },
    .ops = {
        .init     = bt_sink_init,
        .postinit = bt_sink_postinit,
        .set_freq = bt_sink_set_freq,
        .lock     = bt_sink_lock,
        .unlock   = bt_sink_unlock,
        .play     = bt_sink_play,
        .stop     = bt_sink_stop,
        .set_volume = bt_sink_set_volume,
    },
};