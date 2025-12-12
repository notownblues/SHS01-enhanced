/*
 * SPDX-FileCopyrightText: 2021-2025
 * SPDX-License-Identifier: CC0-1.0
 *
 * SHS01 Presence Sensor Header
 * Based on SmartHomeScene firmware with distance and energy data
 */

#ifndef SHS01_H
#define SHS01_H

#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* ---------------- Zigbee device & endpoints ---------------- */
#define SHS_MAX_CHILDREN                10
#define SHS_INSTALLCODE_POLICY_ENABLE   false

/* Endpoints */
#define SHS_EP_LIGHT                    1   /* genOnOff Light + Config Cluster */
#define SHS_EP_OCC                      2   /* Occupancy Sensing + Distance */

/* Router device config */
#define SHS_ZR_CONFIG()                                         \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,               \
        .install_code_policy = SHS_INSTALLCODE_POLICY_ENABLE,   \
        .nwk_cfg.zczr_cfg = {                                   \
            .max_children = SHS_MAX_CHILDREN,                   \
        },                                                      \
    }

/* Channels */
#define SHS_PRIMARY_CHANNEL_MASK        ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Manufacturer / Model strings for Basic cluster (length-prefixed ASCII) */
#define SHS_MANUFACTURER_NAME           "\x0E""SmartHomeScene"
#define SHS_MODEL_IDENTIFIER            "\x0D""SHS01_Enhanced"

/* Optional Basic metadata (length-prefixed ZCL char strings) */
#define SHS_BASIC_DATE_CODE             "\x0A""2025-12-12"
#define SHS_BASIC_SW_BUILD_ID           "\x0B""SHS01-2.0.0"

/* ============================================================================
 * CLUSTER IDS
 * ============================================================================ */

/* Config cluster */
#define SHS_CL_CFG_ID                   0xFDCD

/* Distance measurements cluster */
#define SHS_CL_DISTANCE_ID              0xFDCE

/* ============================================================================
 * CONFIGURATION CLUSTER ATTRIBUTES (0xFDCD on EP1)
 * ============================================================================ */

#define SHS_ATTR_MOVEMENT_COOLDOWN      0x0001  /* uint16, seconds (moving target cooldown) */
#define SHS_ATTR_OCC_CLEAR_COOLDOWN     0x0002  /* uint16, seconds (occupancy delay) */
#define SHS_ATTR_MOVING_SENS_0_10       0x0003  /* uint16, 0-10 scale */
#define SHS_ATTR_STATIC_SENS_0_10       0x0004  /* uint16, 0-10 scale */
#define SHS_ATTR_MOVING_MAX_GATE        0x0005  /* uint16, 0-8 (movement detection range) */
#define SHS_ATTR_STATIC_MAX_GATE        0x0006  /* uint16, 2-8 (static detection range) */

/* ============================================================================
 * DISTANCE CLUSTER ATTRIBUTES (0xFDCE on EP2)
 * ============================================================================ */

#define SHS_ATTR_MOVING_DISTANCE        0x0001  /* uint16, cm */
#define SHS_ATTR_STATIC_DISTANCE        0x0002  /* uint16, cm */
#define SHS_ATTR_DETECTION_DISTANCE     0x0003  /* uint16, cm */
#define SHS_ATTR_MOVING_ENERGY          0x0004  /* uint8, 0-100 */
#define SHS_ATTR_STATIC_ENERGY          0x0005  /* uint8, 0-100 */

/* ============================================================================
 * OCCUPANCY CLUSTER CUSTOM ATTRIBUTES (on EP2)
 * ============================================================================ */

#define SHS_ATTR_OCC_MOVING_TARGET      0xF001  /* bool, manufacturer-specific */
#define SHS_ATTR_OCC_STATIC_TARGET      0xF002  /* bool, manufacturer-specific */
#define SHS_ATTR_OCC_MOVING_DISTANCE    0xF003  /* uint16, cm */
#define SHS_ATTR_OCC_STATIC_DISTANCE    0xF004  /* uint16, cm */
#define SHS_ATTR_OCC_DETECTION_DISTANCE 0xF005  /* uint16, cm */
#define SHS_ATTR_OCC_MOVING_ENERGY      0xF006  /* uint8, 0-100 */
#define SHS_ATTR_OCC_STATIC_ENERGY      0xF007  /* uint8, 0-100 */

/* Standard occupancy cluster attribute */
#define SHS_ZCL_ATTR_OCC_PIR_OU_DELAY   0x0010

/* ============================================================================
 * BOOT BUTTON & MISC
 * ============================================================================ */

#define SHS_BOOT_BUTTON_GPIO            GPIO_NUM_9
#define SHS_FACTORY_RESET_LONGPRESS_MS  6000

/* NVS debounce */
#define SHS_NVS_DEBOUNCE_MS             500
#define SHS_COOLDOWN_MAX_SEC            300

#endif /* SHS01_H */
