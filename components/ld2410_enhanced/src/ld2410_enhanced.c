/**
 * @file ld2410_enhanced.c
 * @brief Enhanced LD2410C mmWave Radar Driver Implementation
 *
 * Complete implementation extracting ALL available sensor data.
 */

#include "ld2410_enhanced.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LD2410";

/* ============================================================================
 * PRIVATE DATA
 * ============================================================================ */

static ld2410_state_t s_state = {0};
static uint8_t s_rx_buffer[LD2410_UART_BUF_SIZE];
static uint8_t s_frame_buffer[64];
static int s_frame_pos = 0;
static bool s_in_frame = false;
static uint8_t s_frame_type = 0;  // 0=data, 1=cmd response

/* Callbacks */
static ld2410_state_callback_t s_state_callback = NULL;
static ld2410_distance_callback_t s_distance_callback = NULL;
static ld2410_energy_callback_t s_energy_callback = NULL;
static ld2410_gate_callback_t s_gate_callback = NULL;

/* Command response handling */
static bool s_waiting_response = false;
static uint16_t s_expected_cmd = 0;
static uint8_t s_response_buffer[64];
static int s_response_len = 0;
static bool s_response_received = false;

/* ============================================================================
 * FRAME PARSING
 * ============================================================================ */

/**
 * @brief Parse basic target data frame (normal mode)
 */
static void parse_basic_frame(const uint8_t *data, int len) {
    if (len < 13) {
        ESP_LOGW(TAG, "Basic frame too short: %d bytes", len);
        return;
    }

    // Frame structure:
    // [0] = data type (0x02 for basic)
    // [1] = 0xAA (header)
    // [2] = target state
    // [3-4] = moving distance (little-endian)
    // [5] = moving energy
    // [6-7] = static distance (little-endian)
    // [8] = static energy
    // [9-10] = detection distance (little-endian)
    // [11] = 0x55 (tail)
    // [12] = 0x00 (check)

    if (data[0] != LD2410_DATA_TYPE_BASIC || data[1] != 0xAA) {
        ESP_LOGW(TAG, "Invalid basic frame header");
        return;
    }

    // Store previous values for change detection
    uint8_t prev_state = s_state.target.target_state;
    uint16_t prev_moving_dist = s_state.target.moving_distance;
    uint16_t prev_static_dist = s_state.target.static_distance;
    uint16_t prev_detection_dist = s_state.target.detection_distance;
    uint8_t prev_moving_energy = s_state.target.moving_energy;
    uint8_t prev_static_energy = s_state.target.static_energy;
    bool first_frame = (s_state.frame_count == 0);

    // Parse target data
    s_state.target.target_state = data[2];
    s_state.target.moving_distance = data[3] | (data[4] << 8);
    s_state.target.moving_energy = data[5];
    s_state.target.static_distance = data[6] | (data[7] << 8);
    s_state.target.static_energy = data[8];
    s_state.target.detection_distance = data[9] | (data[10] << 8);

    // Update derived states with cooldown logic
    uint32_t now = esp_timer_get_time() / 1000;  // ms

    // Moving target detection with cooldown
    bool raw_moving = (s_state.target.target_state & LD2410_STATE_MOVING) != 0;
    if (raw_moving) {
        s_state.moving_detected = true;
        s_state.moving_cooldown_until = now + (s_state.moving_cooldown_seconds * 1000);
    } else if (now >= s_state.moving_cooldown_until) {
        s_state.moving_detected = false;
    }

    // Static target detection (no software cooldown - uses sensor's built-in occupancy timeout)
    s_state.static_detected = (s_state.target.target_state & LD2410_STATE_STATIC) != 0;

    // Occupancy = moving OR static
    s_state.occupancy_detected = s_state.moving_detected || s_state.static_detected;

    // Store previous derived states for change detection
    static bool prev_moving_detected = false;
    static bool prev_static_detected = false;

    // Update connection status
    s_state.connected = true;
    s_state.last_frame_time = now;
    s_state.frame_count++;

    // Fire state callback if derived states or energy changed (or first frame)
    // NOTE: We check derived states (with cooldown), not raw target_state
    bool state_changed = (s_state.moving_detected != prev_moving_detected) ||
                         (s_state.static_detected != prev_static_detected) ||
                         (s_state.target.moving_energy != prev_moving_energy) ||
                         (s_state.target.static_energy != prev_static_energy) ||
                         first_frame;

    prev_moving_detected = s_state.moving_detected;
    prev_static_detected = s_state.static_detected;

    if (state_changed && s_state_callback) {
        s_state_callback(&s_state);
    }

    // Fire distance callback if any distance changed (or first frame)
    bool distance_changed = (s_state.target.moving_distance != prev_moving_dist) ||
                           (s_state.target.static_distance != prev_static_dist) ||
                           (s_state.target.detection_distance != prev_detection_dist) ||
                           first_frame;
    if (distance_changed && s_distance_callback) {
        s_distance_callback(
            s_state.target.moving_distance,
            s_state.target.static_distance,
            s_state.target.detection_distance
        );
    }

    if (s_energy_callback) {
        s_energy_callback(
            s_state.target.moving_energy,
            s_state.target.static_energy
        );
    }
}

/**
 * @brief Parse engineering mode frame (includes per-gate data)
 */
static void parse_engineering_frame(const uint8_t *data, int len) {
    if (len < 35) {  // Minimum engineering frame size
        ESP_LOGW(TAG, "Engineering frame too short: %d bytes", len);
        return;
    }

    // Engineering frame structure:
    // [0] = data type (0x01 for engineering)
    // [1] = 0xAA (header)
    // [2] = target state
    // [3-4] = moving distance
    // [5] = moving energy
    // [6-7] = static distance
    // [8] = static energy
    // [9-10] = detection distance
    // [11] = max moving gate config
    // [12] = max static gate config
    // [13-21] = gate 0-8 moving energy (9 bytes)
    // [22-30] = gate 0-8 static energy (9 bytes)
    // [31+] = reserved data, tail, check

    if (data[0] != LD2410_DATA_TYPE_ENGINEERING || data[1] != 0xAA) {
        ESP_LOGW(TAG, "Invalid engineering frame header");
        return;
    }

    // Store previous values for change detection
    uint8_t prev_state = s_state.target.target_state;
    uint16_t prev_moving_dist = s_state.target.moving_distance;
    uint16_t prev_static_dist = s_state.target.static_distance;
    uint16_t prev_detection_dist = s_state.target.detection_distance;
    uint8_t prev_moving_energy = s_state.target.moving_energy;
    uint8_t prev_static_energy = s_state.target.static_energy;
    bool first_frame = (s_state.frame_count == 0);

    // Parse basic data first (same as normal mode)
    s_state.target.target_state = data[2];
    s_state.target.moving_distance = data[3] | (data[4] << 8);
    s_state.target.moving_energy = data[5];
    s_state.target.static_distance = data[6] | (data[7] << 8);
    s_state.target.static_energy = data[8];
    s_state.target.detection_distance = data[9] | (data[10] << 8);

    // Parse engineering-specific data
    s_state.engineering.max_moving_gate = data[11];
    s_state.engineering.max_static_gate = data[12];

    // Parse per-gate energy values
    for (int i = 0; i < LD2410_MAX_GATES; i++) {
        s_state.engineering.gates[i].move_energy = data[13 + i];
        s_state.engineering.gates[i].still_energy = data[22 + i];
    }
    s_state.engineering.valid = true;

    // Update derived states (same logic as basic)
    uint32_t now = esp_timer_get_time() / 1000;

    bool raw_moving = (s_state.target.target_state & LD2410_STATE_MOVING) != 0;
    if (raw_moving) {
        s_state.moving_detected = true;
        s_state.moving_cooldown_until = now + (s_state.moving_cooldown_seconds * 1000);
    } else if (now >= s_state.moving_cooldown_until) {
        s_state.moving_detected = false;
    }

    // Static target detection (no software cooldown - uses sensor's built-in occupancy timeout)
    s_state.static_detected = (s_state.target.target_state & LD2410_STATE_STATIC) != 0;

    s_state.occupancy_detected = s_state.moving_detected || s_state.static_detected;

    // Store previous derived states for change detection (engineering mode uses same statics)
    static bool prev_eng_moving_detected = false;
    static bool prev_eng_static_detected = false;

    s_state.connected = true;
    s_state.last_frame_time = now;
    s_state.frame_count++;

    // Fire state callback if derived states or energy changed (or first frame)
    bool state_changed = (s_state.moving_detected != prev_eng_moving_detected) ||
                         (s_state.static_detected != prev_eng_static_detected) ||
                         (s_state.target.moving_energy != prev_moving_energy) ||
                         (s_state.target.static_energy != prev_static_energy) ||
                         first_frame;

    prev_eng_moving_detected = s_state.moving_detected;
    prev_eng_static_detected = s_state.static_detected;

    if (state_changed && s_state_callback) {
        s_state_callback(&s_state);
    }

    // Fire distance callback if any distance changed (or first frame)
    bool distance_changed = (s_state.target.moving_distance != prev_moving_dist) ||
                           (s_state.target.static_distance != prev_static_dist) ||
                           (s_state.target.detection_distance != prev_detection_dist) ||
                           first_frame;
    if (distance_changed && s_distance_callback) {
        s_distance_callback(
            s_state.target.moving_distance,
            s_state.target.static_distance,
            s_state.target.detection_distance
        );
    }

    if (s_energy_callback) {
        s_energy_callback(
            s_state.target.moving_energy,
            s_state.target.static_energy
        );
    }

    if (s_gate_callback) {
        s_gate_callback(&s_state.engineering);
    }
}

/**
 * @brief Parse command response frame
 */
static void parse_response_frame(const uint8_t *data, int len) {
    if (len < 4) {
        return;
    }

    // Response format:
    // [0-1] = command word | 0x0100 (ACK)
    // [2-3] = status (0 = success)
    // [4+] = return data

    uint16_t cmd = data[0] | (data[1] << 8);
    uint16_t ack_cmd = s_expected_cmd | 0x0100;

    if (cmd == ack_cmd) {
        memcpy(s_response_buffer, data, len);
        s_response_len = len;
        s_response_received = true;
    }
}

/**
 * @brief Process a complete data frame
 */
static void process_data_frame(const uint8_t *data, int len) {
    if (len < 1) return;

    uint8_t data_type = data[0];

    if (data_type == LD2410_DATA_TYPE_BASIC) {
        parse_basic_frame(data, len);
    } else if (data_type == LD2410_DATA_TYPE_ENGINEERING) {
        parse_engineering_frame(data, len);
    } else {
        ESP_LOGW(TAG, "Unknown data type: 0x%02X", data_type);
    }
}

/**
 * @brief Process incoming bytes and detect frames
 */
static void process_byte(uint8_t byte) {
    static uint8_t header_buf[4] = {0};
    static uint16_t expected_len = 0;

    // Shift header buffer
    header_buf[0] = header_buf[1];
    header_buf[1] = header_buf[2];
    header_buf[2] = header_buf[3];
    header_buf[3] = byte;

    if (!s_in_frame) {
        // Check for data frame header (F4 F3 F2 F1)
        if (header_buf[0] == 0xF4 && header_buf[1] == 0xF3 &&
            header_buf[2] == 0xF2 && header_buf[3] == 0xF1) {
            s_in_frame = true;
            s_frame_type = 0;  // Data frame
            s_frame_pos = 0;
            expected_len = 0;
            return;
        }

        // Check for command response header (FD FC FB FA)
        if (header_buf[0] == 0xFD && header_buf[1] == 0xFC &&
            header_buf[2] == 0xFB && header_buf[3] == 0xFA) {
            s_in_frame = true;
            s_frame_type = 1;  // Command response
            s_frame_pos = 0;
            expected_len = 0;
            return;
        }
    } else {
        // Collect frame data
        if (s_frame_pos < (int)sizeof(s_frame_buffer)) {
            s_frame_buffer[s_frame_pos++] = byte;
        }

        // First two bytes are length (little-endian)
        if (s_frame_pos == 2) {
            expected_len = s_frame_buffer[0] | (s_frame_buffer[1] << 8);
            if (expected_len > sizeof(s_frame_buffer) - 6) {
                // Invalid length, reset
                s_in_frame = false;
                s_state.error_count++;
                return;
            }
        }

        // Check if we have complete frame (length + 4 bytes footer)
        if (expected_len > 0 && s_frame_pos >= (int)(expected_len + 6)) {
            // Verify footer
            int footer_pos = expected_len + 2;
            bool valid_footer = false;

            if (s_frame_type == 0) {
                // Data frame footer: F8 F7 F6 F5
                valid_footer = (s_frame_buffer[footer_pos] == 0xF8 &&
                               s_frame_buffer[footer_pos+1] == 0xF7 &&
                               s_frame_buffer[footer_pos+2] == 0xF6 &&
                               s_frame_buffer[footer_pos+3] == 0xF5);
            } else {
                // Command response footer: 04 03 02 01
                valid_footer = (s_frame_buffer[footer_pos] == 0x04 &&
                               s_frame_buffer[footer_pos+1] == 0x03 &&
                               s_frame_buffer[footer_pos+2] == 0x02 &&
                               s_frame_buffer[footer_pos+3] == 0x01);
            }

            if (valid_footer) {
                // Process complete frame (skip length bytes)
                if (s_frame_type == 0) {
                    process_data_frame(&s_frame_buffer[2], expected_len);
                } else {
                    parse_response_frame(&s_frame_buffer[2], expected_len);
                }
            } else {
                s_state.error_count++;
                ESP_LOGW(TAG, "Invalid frame footer");
            }

            s_in_frame = false;
        }
    }
}

/* ============================================================================
 * COMMAND SENDING
 * ============================================================================ */

/**
 * @brief Send a command frame to the sensor
 */
static esp_err_t send_command(uint16_t cmd, const uint8_t *data, int data_len) {
    uint8_t frame[64];
    int pos = 0;

    // Header
    frame[pos++] = 0xFD;
    frame[pos++] = 0xFC;
    frame[pos++] = 0xFB;
    frame[pos++] = 0xFA;

    // Length (2 bytes command + data)
    uint16_t len = 2 + data_len;
    frame[pos++] = len & 0xFF;
    frame[pos++] = (len >> 8) & 0xFF;

    // Command word
    frame[pos++] = cmd & 0xFF;
    frame[pos++] = (cmd >> 8) & 0xFF;

    // Data
    if (data && data_len > 0) {
        memcpy(&frame[pos], data, data_len);
        pos += data_len;
    }

    // Footer
    frame[pos++] = 0x04;
    frame[pos++] = 0x03;
    frame[pos++] = 0x02;
    frame[pos++] = 0x01;

    // Send
    int written = uart_write_bytes(LD2410_UART_NUM, frame, pos);
    if (written != pos) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Send command and wait for response
 */
static esp_err_t send_command_wait(uint16_t cmd, const uint8_t *data, int data_len, int timeout_ms) {
    s_waiting_response = true;
    s_expected_cmd = cmd;
    s_response_received = false;
    s_response_len = 0;

    esp_err_t err = send_command(cmd, data, data_len);
    if (err != ESP_OK) {
        s_waiting_response = false;
        return err;
    }

    // Wait for response
    int64_t start = esp_timer_get_time();
    while (!s_response_received) {
        ld2410_process();
        if ((esp_timer_get_time() - start) > (timeout_ms * 1000)) {
            s_waiting_response = false;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    s_waiting_response = false;

    // Check ACK status
    if (s_response_len >= 4) {
        uint16_t status = s_response_buffer[2] | (s_response_buffer[3] << 8);
        if (status != 0) {
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

/**
 * @brief Enable configuration mode
 */
static esp_err_t enable_config(void) {
    uint8_t data[] = {0x01, 0x00};  // 0x0001
    return send_command_wait(LD2410_CMD_ENABLE_CONFIG, data, sizeof(data), 100);
}

/**
 * @brief End configuration mode
 */
static esp_err_t end_config(void) {
    return send_command_wait(LD2410_CMD_END_CONFIG, NULL, 0, 100);
}

/* ============================================================================
 * PUBLIC API IMPLEMENTATION
 * ============================================================================ */

esp_err_t ld2410_init(void) {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = LD2410_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(LD2410_UART_NUM, LD2410_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(LD2410_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(LD2410_UART_NUM, LD2410_UART_TX_PIN, LD2410_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize state
    memset(&s_state, 0, sizeof(s_state));
    s_state.moving_cooldown_seconds = 5;  // Default 5 seconds
    s_state.occupancy_clear_delay = 0;

    ESP_LOGI(TAG, "LD2410 driver initialized");
    return ESP_OK;
}

void ld2410_deinit(void) {
    uart_driver_delete(LD2410_UART_NUM);
    ESP_LOGI(TAG, "LD2410 driver deinitialized");
}

const ld2410_state_t* ld2410_get_state(void) {
    return &s_state;
}

void ld2410_process(void) {
    int len = uart_read_bytes(LD2410_UART_NUM, s_rx_buffer, sizeof(s_rx_buffer), 0);
    for (int i = 0; i < len; i++) {
        process_byte(s_rx_buffer[i]);
    }

    // Check for connection timeout (no data for 3 seconds)
    uint32_t now = esp_timer_get_time() / 1000;
    if (s_state.connected && (now - s_state.last_frame_time) > 3000) {
        s_state.connected = false;
        ESP_LOGW(TAG, "Sensor connection lost");
    }
}

esp_err_t ld2410_enable_engineering_mode(void) {
    ESP_LOGI(TAG, "Enabling engineering mode...");

    esp_err_t err = enable_config();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter config mode: %s", esp_err_to_name(err));
        return err;
    }

    err = send_command_wait(LD2410_CMD_ENABLE_ENGINEERING, NULL, 0, 200);
    if (err == ESP_OK) {
        s_state.engineering_mode_enabled = true;
        ESP_LOGI(TAG, "Engineering mode ENABLED successfully");
    } else {
        ESP_LOGE(TAG, "Engineering mode command FAILED: %s", esp_err_to_name(err));
    }

    end_config();
    return err;
}

esp_err_t ld2410_disable_engineering_mode(void) {
    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_DISABLE_ENGINEERING, NULL, 0, 100);
    if (err == ESP_OK) {
        s_state.engineering_mode_enabled = false;
        s_state.engineering.valid = false;
        ESP_LOGI(TAG, "Engineering mode disabled");
    }

    end_config();
    return err;
}

esp_err_t ld2410_set_max_gate_timeout(uint8_t max_moving_gate, uint8_t max_static_gate, uint16_t timeout_seconds) {
    if (max_moving_gate > 8) max_moving_gate = 8;
    if (max_static_gate < 2) max_static_gate = 2;
    if (max_static_gate > 8) max_static_gate = 8;

    // Command format:
    // 2 bytes param word + 4 bytes value for each of 3 parameters
    uint8_t data[18] = {
        // Max moving gate (word 0x0000)
        0x00, 0x00,
        max_moving_gate, 0x00, 0x00, 0x00,
        // Max static gate (word 0x0001)
        0x01, 0x00,
        max_static_gate, 0x00, 0x00, 0x00,
        // Timeout (word 0x0002)
        0x02, 0x00,
        (uint8_t)(timeout_seconds & 0xFF), (uint8_t)((timeout_seconds >> 8) & 0xFF), 0x00, 0x00
    };

    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_SET_MAX_GATE_TIMEOUT, data, sizeof(data), 100);
    if (err == ESP_OK) {
        s_state.config.max_moving_gate = max_moving_gate;
        s_state.config.max_static_gate = max_static_gate;
        s_state.config.timeout_seconds = timeout_seconds;
        ESP_LOGI(TAG, "Set max gates: moving=%d, static=%d, timeout=%ds",
                 max_moving_gate, max_static_gate, timeout_seconds);
    }

    end_config();
    return err;
}

esp_err_t ld2410_set_gate_sensitivity(uint8_t gate, uint8_t move_sensitivity, uint8_t still_sensitivity) {
    if (gate > 8) return ESP_ERR_INVALID_ARG;
    if (move_sensitivity > 100) move_sensitivity = 100;
    if (still_sensitivity > 100) still_sensitivity = 100;

    // Command format:
    // 2 bytes param word + 4 bytes value for each of 3 parameters
    uint8_t data[18] = {
        // Gate number (word 0x0000)
        0x00, 0x00,
        gate, 0x00, 0x00, 0x00,
        // Move sensitivity (word 0x0001)
        0x01, 0x00,
        move_sensitivity, 0x00, 0x00, 0x00,
        // Still sensitivity (word 0x0002)
        0x02, 0x00,
        still_sensitivity, 0x00, 0x00, 0x00
    };

    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_SET_GATE_SENSITIVITY, data, sizeof(data), 100);
    if (err == ESP_OK) {
        s_state.config.gates[gate].move_sensitivity = move_sensitivity;
        s_state.config.gates[gate].still_sensitivity = still_sensitivity;
        ESP_LOGI(TAG, "Set gate %d sensitivity: move=%d, still=%d",
                 gate, move_sensitivity, still_sensitivity);
    }

    end_config();
    return err;
}

esp_err_t ld2410_set_all_sensitivity(uint8_t move_sensitivity, uint8_t still_sensitivity) {
    if (move_sensitivity > 100) move_sensitivity = 100;
    if (still_sensitivity > 100) still_sensitivity = 100;

    // Use 0xFFFF as gate value to set all gates
    uint8_t data[18] = {
        // Gate number (word 0x0000) - 0xFFFF = all gates
        0x00, 0x00,
        0xFF, 0xFF, 0x00, 0x00,
        // Move sensitivity (word 0x0001)
        0x01, 0x00,
        move_sensitivity, 0x00, 0x00, 0x00,
        // Still sensitivity (word 0x0002)
        0x02, 0x00,
        still_sensitivity, 0x00, 0x00, 0x00
    };

    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_SET_GATE_SENSITIVITY, data, sizeof(data), 100);
    if (err == ESP_OK) {
        for (int i = 0; i < LD2410_MAX_GATES; i++) {
            s_state.config.gates[i].move_sensitivity = move_sensitivity;
            s_state.config.gates[i].still_sensitivity = still_sensitivity;
        }
        ESP_LOGI(TAG, "Set all gates sensitivity: move=%d, still=%d",
                 move_sensitivity, still_sensitivity);
    }

    end_config();
    return err;
}

esp_err_t ld2410_read_config(void) {
    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_READ_PARAMS, NULL, 0, 100);
    if (err == ESP_OK && s_response_len >= 28) {
        // Parse response:
        // [4] = 0xAA header
        // [5] = max gates (N, typically 8)
        // [6] = max moving gate config
        // [7] = max static gate config
        // [8-16] = gate 0-8 motion sensitivity
        // [17-25] = gate 0-8 static sensitivity
        // [26-27] = timeout (little-endian)

        s_state.config.max_moving_gate = s_response_buffer[6];
        s_state.config.max_static_gate = s_response_buffer[7];

        for (int i = 0; i < LD2410_MAX_GATES; i++) {
            s_state.config.gates[i].move_sensitivity = s_response_buffer[8 + i];
            s_state.config.gates[i].still_sensitivity = s_response_buffer[17 + i];
        }

        s_state.config.timeout_seconds = s_response_buffer[26] | (s_response_buffer[27] << 8);
        s_state.config.valid = true;

        ESP_LOGI(TAG, "Config read: max_move=%d, max_static=%d, timeout=%d",
                 s_state.config.max_moving_gate, s_state.config.max_static_gate,
                 s_state.config.timeout_seconds);
    }

    end_config();
    return err;
}

esp_err_t ld2410_read_firmware_version(void) {
    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_READ_FIRMWARE, NULL, 0, 100);
    if (err == ESP_OK && s_response_len >= 12) {
        // Response format:
        // [4-5] = firmware type (0x0000)
        // [6] = major version high byte
        // [7] = major version low byte / minor
        // [8-11] = build number

        s_state.firmware.major = s_response_buffer[7];
        s_state.firmware.minor = s_response_buffer[6];
        s_state.firmware.build = s_response_buffer[8] |
                                (s_response_buffer[9] << 8) |
                                (s_response_buffer[10] << 16) |
                                (s_response_buffer[11] << 24);
        s_state.firmware.valid = true;

        ESP_LOGI(TAG, "Firmware: V%d.%02d.%08lX",
                 s_state.firmware.major, s_state.firmware.minor,
                 (unsigned long)s_state.firmware.build);
    }

    end_config();
    return err;
}

esp_err_t ld2410_factory_reset(void) {
    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_FACTORY_RESET, NULL, 0, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Factory reset successful");
        s_state.config.valid = false;
        s_state.engineering.valid = false;
    }

    end_config();
    return err;
}

esp_err_t ld2410_restart(void) {
    esp_err_t err = enable_config();
    if (err != ESP_OK) return err;

    err = send_command_wait(LD2410_CMD_RESTART, NULL, 0, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Restart command sent");
        s_state.connected = false;
    }

    // Don't call end_config - sensor is restarting
    return err;
}

void ld2410_set_moving_cooldown(uint16_t seconds) {
    s_state.moving_cooldown_seconds = seconds;
    ESP_LOGI(TAG, "Moving cooldown set to %d seconds", seconds);
}

void ld2410_set_occupancy_delay(uint16_t seconds) {
    s_state.occupancy_clear_delay = seconds;
    ESP_LOGI(TAG, "Occupancy clear delay set to %d seconds", seconds);
}

void ld2410_register_state_callback(ld2410_state_callback_t callback) {
    s_state_callback = callback;
}

void ld2410_register_distance_callback(ld2410_distance_callback_t callback) {
    s_distance_callback = callback;
}

void ld2410_register_energy_callback(ld2410_energy_callback_t callback) {
    s_energy_callback = callback;
}

void ld2410_register_gate_callback(ld2410_gate_callback_t callback) {
    s_gate_callback = callback;
}

bool ld2410_is_connected(void) {
    return s_state.connected;
}

const char* ld2410_state_to_string(uint8_t state) {
    switch (state) {
        case LD2410_STATE_NO_TARGET: return "No target";
        case LD2410_STATE_MOVING: return "Moving target";
        case LD2410_STATE_STATIC: return "Static target";
        case LD2410_STATE_MOVING_AND_STATIC: return "Moving + Static";
        default: return "Unknown";
    }
}
