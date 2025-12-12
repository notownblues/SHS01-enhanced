/*
 * SPDX-FileCopyrightText: 2021-2025
 * SPDX-License-Identifier: CC0-1.0
 *
 * SHS01 Presence Sensor
 * Based on SmartHomeScene firmware with distance and energy data
 *
 * Features:
 * - Occupancy/Moving/Static target detection
 * - Distance measurements (moving, static, detection)
 * - Energy values (confidence 0-100)
 * - Configurable: cooldown, delay, sensitivity, detection range
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "shs01.h"
#include "ld2410_enhanced.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "light_driver.h"

#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"

#if !defined CONFIG_ZB_ZCZR
#error "Enable Router: set CONFIG_ZB_ZCZR=y (menuconfig)"
#endif

static const char *SHS_TAG = "SHS01";

/* ============================================================================
 * NVS KEYS
 * ============================================================================ */

#define SHS_NVS_NAMESPACE       "cfg"
#define SHS_NVS_KEY_MV_CD       "mv_cd"
#define SHS_NVS_KEY_OCC_CD      "occ_cd"
#define SHS_NVS_KEY_MV_SENS     "mv_sens"
#define SHS_NVS_KEY_ST_SENS     "st_sens"
#define SHS_NVS_KEY_MV_GATE     "mv_gate"
#define SHS_NVS_KEY_ST_GATE     "st_gate"

/* ============================================================================
 * CONFIGURATION STORAGE
 * ============================================================================ */

/* Basic config (original) */
static uint16_t shs_movement_cooldown_sec = 0;
static uint16_t shs_occupancy_clear_sec   = 0;
static uint8_t  shs_moving_sens_0_100     = 60;
static uint8_t  shs_static_sens_0_100     = 50;
static uint16_t shs_moving_max_gate       = 8;
static uint16_t shs_static_max_gate       = 8;
static uint16_t shs_sens_mv_0_10          = 6;
static uint16_t shs_sens_st_0_10          = 5;

/* Firmware version (read from LD2410) */
static char     shs_firmware_version[20]  = "Unknown";

/* ============================================================================
 * LIVE DATA (updated from LD2410)
 * ============================================================================ */

/* Occupancy states */
static bool shs_moving_state    = false;
static bool shs_static_state    = false;
static bool shs_occupancy_state = false;

/* NEW: Distance measurements (cm) */
static uint16_t shs_moving_distance     = 0;
static uint16_t shs_static_distance     = 0;
static uint16_t shs_detection_distance  = 0;

/* NEW: Energy values (0-100) */
static uint8_t shs_moving_energy        = 0;
static uint8_t shs_static_energy        = 0;

/* Track if first update has happened */
static bool shs_first_distance_update   = true;
static bool shs_first_energy_update     = true;

/* Zigbee ready flag */
static volatile bool shs_zb_ready = false;

/* ============================================================================
 * NVS SAVE WORKER
 * ============================================================================ */

typedef enum {
    SHS_SAVE_IMMEDIATE_U16,
    SHS_SAVE_DEBOUNCE_SENS_MOVE,
    SHS_SAVE_DEBOUNCE_SENS_STATIC,
    SHS_SAVE_DEBOUNCE_GATE_MOVE,
    SHS_SAVE_DEBOUNCE_GATE_STATIC,
} shs_save_evt_t;

typedef struct {
    shs_save_evt_t type;
    uint16_t       u16;
} shs_save_msg_t;

static QueueHandle_t shs_save_q;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

static inline bool shs_time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static void shs_cfg_save_u16(const char *key, uint16_t v) {
    nvs_handle_t h;
    if (nvs_open(SHS_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u16(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

static void shs_cfg_save_u8(const char *key, uint8_t v) {
    nvs_handle_t h;
    if (nvs_open(SHS_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

static inline void shs_save_enqueue(shs_save_evt_t t, uint16_t v) {
    if (!shs_save_q) return;
    shs_save_msg_t m = {.type = t, .u16 = v};
    (void)xQueueSend(shs_save_q, &m, 0);
}

static void shs_cfg_sync_sens_proxies(void) {
    shs_sens_mv_0_10 = (uint16_t)((shs_moving_sens_0_100 + 5) / 10);
    if (shs_sens_mv_0_10 > 10) shs_sens_mv_0_10 = 10;
    shs_sens_st_0_10 = (uint16_t)((shs_static_sens_0_100 + 5) / 10);
    if (shs_sens_st_0_10 > 10) shs_sens_st_0_10 = 10;
}

static void shs_cfg_load_from_nvs(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(SHS_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(SHS_TAG, "NVS open RO failed (%s), using defaults", esp_err_to_name(err));
        return;
    }

    uint16_t u16tmp; uint8_t u8tmp;

    if (nvs_get_u16(h, SHS_NVS_KEY_MV_CD, &u16tmp) == ESP_OK)
        shs_movement_cooldown_sec = (u16tmp > SHS_COOLDOWN_MAX_SEC) ? SHS_COOLDOWN_MAX_SEC : u16tmp;
    if (nvs_get_u16(h, SHS_NVS_KEY_OCC_CD, &u16tmp) == ESP_OK)
        shs_occupancy_clear_sec = u16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_MV_SENS, &u8tmp) == ESP_OK)
        shs_moving_sens_0_100 = (u8tmp > 100) ? 100 : u8tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_ST_SENS, &u8tmp) == ESP_OK)
        shs_static_sens_0_100 = (u8tmp > 100) ? 100 : u8tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_MV_GATE, &u8tmp) == ESP_OK)
        shs_moving_max_gate = (u8tmp > 8) ? 8 : u8tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_ST_GATE, &u8tmp) == ESP_OK) {
        if (u8tmp < 2) u8tmp = 2; else if (u8tmp > 8) u8tmp = 8;
        shs_static_max_gate = u8tmp;
    }

    nvs_close(h);
    shs_cfg_sync_sens_proxies();

    ESP_LOGI(SHS_TAG, "NVS loaded: mv_cd=%us, occ_cd=%us, mv_sens=%u, st_sens=%u, mv_gate=%u, st_gate=%u",
             (unsigned)shs_movement_cooldown_sec, (unsigned)shs_occupancy_clear_sec,
             (unsigned)shs_moving_sens_0_100, (unsigned)shs_static_sens_0_100,
             (unsigned)shs_moving_max_gate, (unsigned)shs_static_max_gate);
}

/* ============================================================================
 * ZIGBEE ATTRIBUTE HELPERS
 * ============================================================================ */

static void shs_zb_set_occ_bitmap(uint8_t endpoint, bool occupied) {
    if (!shs_zb_ready) return;
    uint8_t v = occupied ? 1 : 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint,
                                 ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                                 &v, false);
    esp_zb_lock_release();
}

static void shs_zb_set_bool_attr(uint8_t endpoint, uint16_t cluster, uint16_t attr_id, bool value) {
    if (!shs_zb_ready) return;
    bool v = value;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, &v, false);
    esp_zb_lock_release();
}

static void shs_zb_set_u16_attr(uint8_t endpoint, uint16_t cluster, uint16_t attr_id, uint16_t value) {
    if (!shs_zb_ready) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, &value, false);
    esp_zb_lock_release();
}

static void shs_zb_set_u8_attr(uint8_t endpoint, uint16_t cluster, uint16_t attr_id, uint8_t value) {
    if (!shs_zb_ready) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, &value, false);
    esp_zb_lock_release();
}


static void shs_zb_set_ou_delay_ep2(uint16_t seconds) {
    if (!shs_zb_ready) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(SHS_EP_OCC,
                                 ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 SHS_ZCL_ATTR_OCC_PIR_OU_DELAY,
                                 &seconds, false);
    esp_zb_lock_release();
}

/* ============================================================================
 * LD2410 CALLBACKS - Update Zigbee attributes on sensor changes
 * ============================================================================ */

/* Cooldown tracking */
static uint32_t shs_moving_cooldown_until = 0;

static void shs_on_state_change(const ld2410_state_t *state) {
    /* Extract RAW states directly from sensor */
    bool raw_moving = (state->target.target_state & 0x01) != 0;
    bool raw_static = (state->target.target_state & 0x02) != 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* MOVING TARGET with optional cooldown */
    bool report_moving;
    if (shs_movement_cooldown_sec == 0) {
        report_moving = raw_moving;
    } else {
        if (raw_moving) {
            report_moving = true;
            shs_moving_cooldown_until = now_ms + (shs_movement_cooldown_sec * 1000);
        } else if (now_ms < shs_moving_cooldown_until) {
            report_moving = true;
        } else {
            report_moving = false;
        }
    }

    if (report_moving != shs_moving_state) {
        shs_moving_state = report_moving;
        ESP_LOGI(SHS_TAG, "Moving Target -> %s", shs_moving_state ? "DETECTED" : "CLEAR");
        shs_zb_set_bool_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            SHS_ATTR_OCC_MOVING_TARGET, shs_moving_state);
    }

    /* STATIC TARGET - always RAW */
    if (raw_static != shs_static_state) {
        shs_static_state = raw_static;
        ESP_LOGI(SHS_TAG, "Static Target -> %s", shs_static_state ? "DETECTED" : "CLEAR");
        shs_zb_set_bool_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            SHS_ATTR_OCC_STATIC_TARGET, shs_static_state);
    }

    /* OCCUPANCY = Moving OR Static */
    bool report_occupancy = shs_moving_state || shs_static_state;

    if (report_occupancy != shs_occupancy_state) {
        shs_occupancy_state = report_occupancy;
        ESP_LOGI(SHS_TAG, "Occupancy -> %s", shs_occupancy_state ? "DETECTED" : "CLEAR");
        shs_zb_set_occ_bitmap(SHS_EP_OCC, shs_occupancy_state);
    }

    /* Update distance/energy - set to 0 when no target detected */
    uint16_t new_moving_dist = raw_moving ? state->target.moving_distance : 0;
    uint8_t new_moving_energy = raw_moving ? state->target.moving_energy : 0;
    uint16_t new_static_dist = raw_static ? state->target.static_distance : 0;
    uint8_t new_static_energy = raw_static ? state->target.static_energy : 0;
    uint16_t new_detection_dist = (raw_moving || raw_static) ? state->target.detection_distance : 0;

    bool dist_changed = (new_moving_dist != shs_moving_distance) ||
                        (new_static_dist != shs_static_distance) ||
                        (new_detection_dist != shs_detection_distance);
    bool energy_changed = (new_moving_energy != shs_moving_energy) ||
                          (new_static_energy != shs_static_energy);

    shs_moving_distance = new_moving_dist;
    shs_moving_energy = new_moving_energy;
    shs_static_distance = new_static_dist;
    shs_static_energy = new_static_energy;
    shs_detection_distance = new_detection_dist;

    /* Push distance values to Zigbee attributes when changed */
    if (dist_changed || shs_first_distance_update) {
        /* Update distance cluster (0xFDCE) on EP2 */
        shs_zb_set_u16_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_MOVING_DISTANCE, shs_moving_distance);
        shs_zb_set_u16_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_STATIC_DISTANCE, shs_static_distance);
        shs_zb_set_u16_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_DETECTION_DISTANCE, shs_detection_distance);

        /* Also update occupancy cluster manufacturer-specific attrs */
        shs_zb_set_u16_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_MOVING_DISTANCE, shs_moving_distance);
        shs_zb_set_u16_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_STATIC_DISTANCE, shs_static_distance);
        shs_zb_set_u16_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_DETECTION_DISTANCE, shs_detection_distance);
    }

    /* Push energy values to Zigbee attributes when changed */
    if (energy_changed || shs_first_energy_update) {
        /* Update distance cluster (0xFDCE) on EP2 */
        shs_zb_set_u8_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_MOVING_ENERGY, shs_moving_energy);
        shs_zb_set_u8_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_STATIC_ENERGY, shs_static_energy);

        /* Also update occupancy cluster manufacturer-specific attrs */
        shs_zb_set_u8_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_MOVING_ENERGY, shs_moving_energy);
        shs_zb_set_u8_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_STATIC_ENERGY, shs_static_energy);
    }

    shs_first_distance_update = false;
    shs_first_energy_update = false;
}

static void shs_on_distance_update(uint16_t moving_dist, uint16_t static_dist, uint16_t detection_dist) {
    /* Check if values actually changed */
    bool changed = (moving_dist != shs_moving_distance) ||
                   (static_dist != shs_static_distance) ||
                   (detection_dist != shs_detection_distance);

    shs_moving_distance = moving_dist;
    shs_static_distance = static_dist;
    shs_detection_distance = detection_dist;

    /* Push to Zigbee when changed */
    if (changed) {
        shs_zb_set_u16_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_MOVING_DISTANCE, moving_dist);
        shs_zb_set_u16_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_STATIC_DISTANCE, static_dist);
        shs_zb_set_u16_attr(SHS_EP_OCC, SHS_CL_DISTANCE_ID, SHS_ATTR_DETECTION_DISTANCE, detection_dist);

        shs_zb_set_u16_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_MOVING_DISTANCE, moving_dist);
        shs_zb_set_u16_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_STATIC_DISTANCE, static_dist);
        shs_zb_set_u16_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, SHS_ATTR_OCC_DETECTION_DISTANCE, detection_dist);
    }
}

/* ============================================================================
 * APPLY LD2410 CONFIGURATION
 * ============================================================================ */

static void shs_apply_ld2410_config(void) {
    /* Set max gates and timeout */
    ld2410_set_max_gate_timeout(
        (uint8_t)shs_moving_max_gate,
        (uint8_t)shs_static_max_gate,
        shs_occupancy_clear_sec
    );

    /* Set global sensitivity */
    ld2410_set_all_sensitivity(shs_moving_sens_0_100, shs_static_sens_0_100);

    /* Set cooldowns */
    ld2410_set_moving_cooldown(shs_movement_cooldown_sec);
    ld2410_set_occupancy_delay(shs_occupancy_clear_sec);

    /* Read firmware version */
    ld2410_read_firmware_version();
    const ld2410_state_t *state = ld2410_get_state();
    if (state->firmware.valid) {
        snprintf(shs_firmware_version, sizeof(shs_firmware_version),
                 "V%d.%02d", state->firmware.major, state->firmware.minor);
    }

    /* Read current config from sensor */
    ld2410_read_config();

    ESP_LOGI(SHS_TAG, "LD2410 configuration applied");
}

/* ============================================================================
 * ZIGBEE ATTRIBUTE WRITE HANDLER
 * ============================================================================ */

static esp_err_t shs_zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (!message) return ESP_OK;

    /* EP1: genOnOff (light) */
    if (message->info.dst_endpoint == SHS_EP_LIGHT &&
        message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            bool light_state = *(bool *)message->attribute.data.value;
            ESP_LOGI(SHS_TAG, "Light -> %s", light_state ? "ON" : "OFF");
            light_driver_set_power(light_state);
            return ESP_OK;
        }
    }

    /* EP1: Config cluster (0xFDCD) */
    if (message->info.dst_endpoint == SHS_EP_LIGHT &&
        message->info.cluster == SHS_CL_CFG_ID) {

        uint16_t v = 0;
        uint8_t v8 = 0;

        if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
            v = *(uint16_t *)message->attribute.data.value;
        } else if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 ||
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            v8 = *(uint8_t *)message->attribute.data.value;
            v = v8;
        }

        switch (message->attribute.id) {
            case SHS_ATTR_MOVEMENT_COOLDOWN:
                if (v > SHS_COOLDOWN_MAX_SEC) v = SHS_COOLDOWN_MAX_SEC;
                shs_movement_cooldown_sec = v;
                ld2410_set_moving_cooldown(v);
                shs_save_enqueue(SHS_SAVE_IMMEDIATE_U16, (SHS_ATTR_MOVEMENT_COOLDOWN << 8));
                ESP_LOGI(SHS_TAG, "Set Movement Cooldown = %us", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_OCC_CLEAR_COOLDOWN:
                shs_occupancy_clear_sec = v;
                ld2410_set_max_gate_timeout((uint8_t)shs_moving_max_gate,
                                           (uint8_t)shs_static_max_gate, v);
                shs_zb_set_ou_delay_ep2(v);
                shs_save_enqueue(SHS_SAVE_IMMEDIATE_U16, (SHS_ATTR_OCC_CLEAR_COOLDOWN << 8));
                ESP_LOGI(SHS_TAG, "Set Occupancy Cooldown = %us", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_MOVING_SENS_0_10:
                if (v > 10) v = 10;
                shs_sens_mv_0_10 = v;
                shs_moving_sens_0_100 = (uint8_t)(v * 10);
                ld2410_set_all_sensitivity(shs_moving_sens_0_100, shs_static_sens_0_100);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_SENS_MOVE, shs_moving_sens_0_100);
                ESP_LOGI(SHS_TAG, "Set Moving Sensitivity = %u/100", (unsigned)shs_moving_sens_0_100);
                return ESP_OK;

            case SHS_ATTR_STATIC_SENS_0_10:
                if (v > 10) v = 10;
                shs_sens_st_0_10 = v;
                shs_static_sens_0_100 = (uint8_t)(v * 10);
                ld2410_set_all_sensitivity(shs_moving_sens_0_100, shs_static_sens_0_100);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_SENS_STATIC, shs_static_sens_0_100);
                ESP_LOGI(SHS_TAG, "Set Static Sensitivity = %u/100", (unsigned)shs_static_sens_0_100);
                return ESP_OK;

            case SHS_ATTR_MOVING_MAX_GATE:
                if (v > 8) v = 8;
                shs_moving_max_gate = v;
                ld2410_set_max_gate_timeout((uint8_t)v, (uint8_t)shs_static_max_gate,
                                           shs_occupancy_clear_sec);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_GATE_MOVE, v);
                ESP_LOGI(SHS_TAG, "Set Movement Detection Range = %u", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_STATIC_MAX_GATE:
                if (v < 2) v = 2; else if (v > 8) v = 8;
                shs_static_max_gate = v;
                ld2410_set_max_gate_timeout((uint8_t)shs_moving_max_gate, (uint8_t)v,
                                           shs_occupancy_clear_sec);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_GATE_STATIC, v);
                ESP_LOGI(SHS_TAG, "Set Static Detection Range = %u", (unsigned)v);
                return ESP_OK;

            default:
                break;
        }
    }

    return ESP_OK;
}

static esp_err_t shs_zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        return shs_zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    }
    return ESP_OK;
}

/* ============================================================================
 * BOOT BUTTON (FACTORY RESET)
 * ============================================================================ */

static void shs_boot_button_task(void *pv) {
    const TickType_t poll = pdMS_TO_TICKS(25);
    const uint32_t required_ticks = SHS_FACTORY_RESET_LONGPRESS_MS / 25;
    uint32_t held = 0;
    bool armed = false;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << SHS_BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    ESP_LOGI(SHS_TAG, "BOOT long-press enabled on GPIO%d (hold %u ms to factory reset)",
             SHS_BOOT_BUTTON_GPIO, (unsigned)SHS_FACTORY_RESET_LONGPRESS_MS);

    while (1) {
        int level = gpio_get_level(SHS_BOOT_BUTTON_GPIO);
        if (level == 0) {
            if (held < required_ticks) held++;
            if (!armed && held > 4) {
                armed = true;
                ESP_LOGI(SHS_TAG, "BOOT press detected, hold to confirm...");
            }
            if (held >= required_ticks) {
                ESP_LOGW(SHS_TAG, "BOOT long-press confirmed: factory reset...");
                esp_zb_factory_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        } else {
            held = 0;
            armed = false;
        }
        vTaskDelay(poll);
    }
}

/* ============================================================================
 * LD2410 PROCESSING TASK
 * ============================================================================ */

static void shs_ld2410_task(void *pvParameters) {
    ESP_LOGI(SHS_TAG, "LD2410 processing task started");

    while (1) {
        ld2410_process();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================================
 * ZIGBEE SIGNAL HANDLER
 * ============================================================================ */

static void shs_bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    if (esp_zb_bdb_start_top_level_commissioning(mode_mask) != ESP_OK) {
        ESP_LOGW(SHS_TAG, "Failed to start Zigbee commissioning");
    }
}

static void shs_basic_publish_metadata_ep1(void) {
    if (!shs_zb_ready) return;

    const char *date_code = SHS_BASIC_DATE_CODE;
    const char *sw_build = SHS_BASIC_SW_BUILD_ID;
    uint8_t power_src = 0x01;

    esp_zb_lock_acquire(portMAX_DELAY);

    esp_zb_zcl_set_attribute_val(SHS_EP_LIGHT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID,
                                 &power_src, false);

    esp_zb_zcl_set_attribute_val(SHS_EP_LIGHT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
                                 (void *)date_code, false);
    esp_zb_zcl_set_attribute_val(SHS_EP_LIGHT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,
                                 (void *)sw_build, false);

    esp_zb_lock_release();
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(SHS_TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            shs_zb_ready = true;

            ESP_LOGI(SHS_TAG, "Device started up in%s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(SHS_TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(SHS_TAG, "Device rebooted");
            }

            /* Publish metadata (doesn't require network) */
            shs_basic_publish_metadata_ep1();
            shs_zb_set_ou_delay_ep2(shs_occupancy_clear_sec);
        } else {
            ESP_LOGW(SHS_TAG, "Failed to initialize Zigbee stack (%s)", esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(SHS_TAG, "Joined network (PAN:0x%04hx, Ch:%d)",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            ESP_LOGW(SHS_TAG, "Network steering not successful (%s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)shs_bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    default:
        ESP_LOGI(SHS_TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* ============================================================================
 * ZIGBEE TASK - ENDPOINT & CLUSTER CREATION
 * ============================================================================ */

static void shs_zigbee_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = SHS_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = SHS_MANUFACTURER_NAME,
        .model_identifier = SHS_MODEL_IDENTIFIER,
    };

    esp_zb_ep_list_t *dev_ep_list = esp_zb_ep_list_create();

    /* ========== EP1: Light + Config Cluster ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_on_off_cluster_cfg_t on_off_cfg = {.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE};
        esp_zb_attribute_list_t *onoff = esp_zb_on_off_cluster_create(&on_off_cfg);

        esp_zb_cluster_list_add_basic_cluster(cl, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_cluster_list_add_on_off_cluster(cl, onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* Custom Config Cluster (0xFDCD) */
        esp_zb_attribute_list_t *cfg_cl = esp_zb_zcl_attr_list_create(SHS_CL_CFG_ID);

        /* Original attributes */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MOVEMENT_COOLDOWN,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_movement_cooldown_sec);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_OCC_CLEAR_COOLDOWN,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_occupancy_clear_sec);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MOVING_SENS_0_10,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_sens_mv_0_10);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_STATIC_SENS_0_10,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_sens_st_0_10);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MOVING_MAX_GATE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_moving_max_gate);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_STATIC_MAX_GATE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_static_max_gate);

        esp_zb_cluster_list_add_custom_cluster(cl, cfg_cl, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LIGHT,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);

        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LIGHT, &info);
    }

    /* ========== EP2: Occupancy + Distance + Gates ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Standard Occupancy Sensing cluster */
        esp_zb_attribute_list_t *occ = esp_zb_occupancy_sensing_cluster_create(NULL);

        /* Add manufacturer-specific boolean attrs (these work!) */
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_MOVING_TARGET,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_moving_state);
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_STATIC_TARGET,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_static_state);

        /* Add distance and energy to occupancy cluster as manufacturer-specific attrs */
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_MOVING_DISTANCE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_moving_distance);
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_STATIC_DISTANCE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_static_distance);
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_DETECTION_DISTANCE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_detection_distance);
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_MOVING_ENERGY,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_moving_energy);
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_STATIC_ENERGY,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_static_energy);

        esp_zb_cluster_list_add_occupancy_sensing_cluster(cl, occ, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* Keep custom distance cluster for compatibility but also add to occupancy */
        esp_zb_attribute_list_t *dist_cl = esp_zb_zcl_attr_list_create(SHS_CL_DISTANCE_ID);

        esp_zb_custom_cluster_add_custom_attr(dist_cl, SHS_ATTR_MOVING_DISTANCE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_moving_distance);
        esp_zb_custom_cluster_add_custom_attr(dist_cl, SHS_ATTR_STATIC_DISTANCE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_static_distance);
        esp_zb_custom_cluster_add_custom_attr(dist_cl, SHS_ATTR_DETECTION_DISTANCE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_detection_distance);
        esp_zb_custom_cluster_add_custom_attr(dist_cl, SHS_ATTR_MOVING_ENERGY,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_moving_energy);
        esp_zb_custom_cluster_add_custom_attr(dist_cl, SHS_ATTR_STATIC_ENERGY,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_static_energy);

        esp_zb_cluster_list_add_custom_cluster(cl, dist_cl, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_OCC,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = 0x0107, /* Occupancy Sensor */
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
    }

    /* Register device and start */
    esp_zb_device_register(dev_ep_list);
    esp_zb_core_action_handler_register(shs_zb_action_handler);
    esp_zb_set_primary_network_channel_set(SHS_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ============================================================================
 * NVS SAVE WORKER TASK
 * ============================================================================ */

static void shs_save_worker(void *pv) {
    TickType_t last_mv_sens = 0, last_st_sens = 0, last_mv_gate = 0, last_st_gate = 0;
    bool pend_mv_sens = false, pend_st_sens = false, pend_mv_gate = false, pend_st_gate = false;
    uint8_t mv_sens_val = shs_moving_sens_0_100, st_sens_val = shs_static_sens_0_100;
    uint8_t mv_gate_val = (uint8_t)shs_moving_max_gate, st_gate_val = (uint8_t)shs_static_max_gate;

    shs_save_msg_t m;
    for (;;) {
        if (xQueueReceive(shs_save_q, &m, pdMS_TO_TICKS(50))) {
            switch (m.type) {
                case SHS_SAVE_IMMEDIATE_U16:
                    if ((m.u16 >> 8) == SHS_ATTR_MOVEMENT_COOLDOWN) {
                        shs_cfg_save_u16(SHS_NVS_KEY_MV_CD, shs_movement_cooldown_sec);
                    } else if ((m.u16 >> 8) == SHS_ATTR_OCC_CLEAR_COOLDOWN) {
                        shs_cfg_save_u16(SHS_NVS_KEY_OCC_CD, shs_occupancy_clear_sec);
                    }
                    break;
                case SHS_SAVE_DEBOUNCE_SENS_MOVE:
                    mv_sens_val = (uint8_t)m.u16; pend_mv_sens = true; last_mv_sens = xTaskGetTickCount();
                    break;
                case SHS_SAVE_DEBOUNCE_SENS_STATIC:
                    st_sens_val = (uint8_t)m.u16; pend_st_sens = true; last_st_sens = xTaskGetTickCount();
                    break;
                case SHS_SAVE_DEBOUNCE_GATE_MOVE:
                    mv_gate_val = (uint8_t)m.u16; pend_mv_gate = true; last_mv_gate = xTaskGetTickCount();
                    break;
                case SHS_SAVE_DEBOUNCE_GATE_STATIC:
                    st_gate_val = (uint8_t)m.u16; pend_st_gate = true; last_st_gate = xTaskGetTickCount();
                    break;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (pend_mv_sens && (now - last_mv_sens) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_MV_SENS, mv_sens_val);
            pend_mv_sens = false;
        }
        if (pend_st_sens && (now - last_st_sens) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_ST_SENS, st_sens_val);
            pend_st_sens = false;
        }
        if (pend_mv_gate && (now - last_mv_gate) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_MV_GATE, mv_gate_val);
            pend_mv_gate = false;
        }
        if (pend_st_gate && (now - last_st_gate) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_ST_GATE, st_gate_val);
            pend_st_gate = false;
        }
    }
}

/* ============================================================================
 * APP_MAIN
 * ============================================================================ */

void app_main(void) {
    /* Initialize NVS */
    esp_err_t nvs_rc = nvs_flash_init();
    if (nvs_rc == ESP_ERR_NVS_NO_FREE_PAGES || nvs_rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_rc);

    /* Load configuration from NVS */
    shs_cfg_load_from_nvs();

    /* Initialize light driver */
    light_driver_init(LIGHT_DEFAULT_OFF);

    /* Initialize LD2410 enhanced driver */
    ESP_ERROR_CHECK(ld2410_init());

    /* Register LD2410 callbacks */
    ld2410_register_state_callback(shs_on_state_change);
    ld2410_register_distance_callback(shs_on_distance_update);

    /* Apply initial configuration to LD2410 */
    vTaskDelay(pdMS_TO_TICKS(100));  /* Let sensor stabilize */
    shs_apply_ld2410_config();

    /* Create NVS save worker queue and task */
    shs_save_q = xQueueCreate(16, sizeof(shs_save_msg_t));
    xTaskCreate(shs_save_worker, "shs_save_worker", 3072, NULL, 3, NULL);

    /* Create tasks */
    xTaskCreate(shs_ld2410_task, "shs_ld2410_task", 4096, NULL, 6, NULL);
    xTaskCreate(shs_boot_button_task, "shs_boot_button", 2048, NULL, 4, NULL);
    xTaskCreate(shs_zigbee_task, "shs_zigbee_main", 4096, NULL, 5, NULL);

    ESP_LOGI(SHS_TAG, "SHS01 firmware started");
}
