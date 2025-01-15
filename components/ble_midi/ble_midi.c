#include <string.h>
#include "ble_midi.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_gatt_common_api.h"
#include "led_strip.h"

static const char *TAG = "BLE_MIDI";

#define MIDI_PROFILE_APP_ID 0
#define DEVICE_NAME "USB2BLE MIDI Bridge"

#define MIDI_PROFILE_NUM     1
#define MIDI_PROFILE_APP_IDX 0
#define MIDI_APP_ID          0x55

#define BLEMIDI_NUM_PORTS 1
#define GATTS_MIDI_CHAR_VAL_LEN_MAX 100

// GATT service table index
enum {
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,
    MIDI_IDX_NB
};

#define CHAR_DECLARATION_SIZE     (sizeof(uint8_t))
#define SVC_INST_ID              0

// GATT service table handles
static uint16_t midi_handle_table[MIDI_IDX_NB];

// MIDI BLE standard UUID
// MIDI Service: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
static const uint8_t midi_service_uuid[] = {
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};

// MIDI I/O Characteristic: 7772E5DB-3868-4112-A1A9-F2669D106BF3
static const uint8_t midi_char_uuid[] = {
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// Add MIDI appearance definition
#define BLE_APPEARANCE_MIDI    0x0877    // Standard appearance value for MIDI device

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t char_uuid;
    esp_bt_uuid_t descr_uuid;
    esp_bd_addr_t remote_bda;
};
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst midi_profile_tab[MIDI_PROFILE_NUM] = {
    [MIDI_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// Add buffer related variables
static uint8_t blemidi_outbuffer[BLEMIDI_NUM_PORTS][GATTS_MIDI_CHAR_VAL_LEN_MAX];
static uint32_t blemidi_outbuffer_len[BLEMIDI_NUM_PORTS];
static size_t blemidi_mtu = GATTS_MIDI_CHAR_VAL_LEN_MAX - 3;

// Get millisecond timestamp
static uint32_t get_ms() {
    return esp_timer_get_time() / 1000;
}

// Generate timestamp bytes
static uint8_t blemidi_timestamp_high(uint32_t timestamp) {
    return (0x80 | ((timestamp >> 7) & 0x3f));
}

static uint8_t blemidi_timestamp_low(uint32_t timestamp) {
    return (0x80 | (timestamp & 0x7f));
}

// Flush output buffer
static int32_t blemidi_outbuffer_flush(uint8_t blemidi_port) {
    if (blemidi_outbuffer_len[blemidi_port] > 0) {
        esp_err_t ret = esp_ble_gatts_send_indicate(
            midi_profile_tab[MIDI_PROFILE_APP_IDX].gatts_if,
            midi_profile_tab[MIDI_PROFILE_APP_IDX].conn_id,
            midi_handle_table[IDX_CHAR_VAL_A],  // Use correct handle
            blemidi_outbuffer_len[blemidi_port],
            blemidi_outbuffer[blemidi_port],
            false);
            
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send MIDI data: %d", ret);
            return -1;
        }
        
        blemidi_outbuffer_len[blemidi_port] = 0;
    }
    return 0;
}

// Add data to output buffer
static int32_t blemidi_outbuffer_push(uint8_t blemidi_port, uint8_t *stream, size_t len) {
    const uint32_t timestamp = get_ms();
    
    // If buffer is about to overflow, flush first
    if ((blemidi_outbuffer_len[blemidi_port] + len + 2) >= blemidi_mtu) {
        blemidi_outbuffer_flush(blemidi_port);
    }

    // Add new message
    if (blemidi_outbuffer_len[blemidi_port] == 0) {
        // New packet: add timestamp
        blemidi_outbuffer[blemidi_port][blemidi_outbuffer_len[blemidi_port]++] = 
            blemidi_timestamp_high(timestamp);
        if (stream[0] >= 0x80) {
            blemidi_outbuffer[blemidi_port][blemidi_outbuffer_len[blemidi_port]++] = 
                blemidi_timestamp_low(timestamp);
        }
    } else {
        blemidi_outbuffer[blemidi_port][blemidi_outbuffer_len[blemidi_port]++] = 
            blemidi_timestamp_low(timestamp);
    }

    // Copy MIDI data
    memcpy(&blemidi_outbuffer[blemidi_port][blemidi_outbuffer_len[blemidi_port]], 
           stream, len);
    blemidi_outbuffer_len[blemidi_port] += len;

    return 0;
}

// Broadcast data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0012,
    .appearance = BLE_APPEARANCE_MIDI,   // Set to MIDI device
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(midi_service_uuid),
    .p_service_uuid = (uint8_t *)midi_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0012,
    .appearance = BLE_APPEARANCE_MIDI,   // Scan response also set to MIDI device
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(midi_service_uuid),
    .p_service_uuid = (uint8_t *)midi_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Broadcast parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(TAG, "GAP event: %d", event);
    
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT, status: %d", 
                 param->adv_start_cmpl.status);
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Broadcast start failed");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT, status: %d", 
                 param->adv_stop_cmpl.status);
        break;

    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:  // Event 21
        ESP_LOGI(TAG, "ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT, status = %d", 
                 param->pkt_data_length_cmpl.status);
        if (param->pkt_data_length_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Packet length set successfully");
        } else {
            ESP_LOGE(TAG, "Packet length set failed");
        }
        break;

    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:  // Event 55
        ESP_LOGI(TAG, "ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT, status = %d", 
                 param->phy_update.status);
        ESP_LOGI(TAG, "TX PHY: %d", param->phy_update.tx_phy);
        ESP_LOGI(TAG, "RX PHY: %d", param->phy_update.rx_phy);
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Connection parameters updated: interval=%d, latency=%d, timeout=%d",
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        ESP_LOGI(TAG, "Unhandled GAP event: %d", event);
        break;
    }
}

// Add necessary UUID definitions
static uint16_t midi_handle_table[MIDI_IDX_NB];

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | 
                                                  ESP_GATT_CHAR_PROP_BIT_READ |
                                                  ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                                                  ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

static const uint8_t midi_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t gatt_db[MIDI_IDX_NB] = {
    // MIDI Service Declaration
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_16, 
        (uint8_t *)&primary_service_uuid, 
        ESP_GATT_PERM_READ,
        sizeof(midi_service_uuid), 
        sizeof(midi_service_uuid), 
        (uint8_t *)midi_service_uuid
    }},

    // MIDI Characteristic Declaration
    [IDX_CHAR_A] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_16, 
        (uint8_t *)&character_declaration_uuid,
        ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, 
        CHAR_DECLARATION_SIZE, 
        (uint8_t *)&char_prop_read_write_notify
    }},

    // MIDI Characteristic Value
    [IDX_CHAR_VAL_A] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_128, 
        (uint8_t *)midi_char_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        GATTS_MIDI_CHAR_VAL_LEN_MAX, 
        0, 
        NULL
    }},

    // MIDI Client Characteristic Configuration Descriptor
    [IDX_CHAR_CFG_A] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_16,
        (uint8_t *)&character_client_config_uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t),
        sizeof(midi_ccc),
        (uint8_t *)midi_ccc
    }},
};

static midi_callback_t midi_callback = NULL;

static void parse_sysex(const uint8_t* data, uint16_t len) {
    if (len < 5) return;  // Message too short
    
    // Skip timestamp bytes
    const uint8_t* sysex = data + 2;
    
    if (sysex[0] == 0xF0 && sysex[1] == 0x7E) {  // Universal Non-realtime
        ESP_LOGI(TAG, "Received universal system information:");
        ESP_LOGI(TAG, "   Device ID: 0x%02x", sysex[4]);
        if (sysex[3] == 0x0D) {  // General MIDI
            if (sysex[4] == 0x70 && sysex[5] == 0x02) {
                ESP_LOGI(TAG, "   Request to switch to General MIDI 2 mode");
            }
        }
    }
}

static bool notifications_enabled = false;

#define LED_GPIO    48
#define LED_NUM     1

static led_strip_handle_t led_strip;

// LED control function
static void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

// LED initialization function
static void init_led(void) {
    // LED configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    // RMT configuration
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    set_led_color(16, 0, 0);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "GATTS event: %d, gatts_if: %d", event, gatts_if);

    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT, status: %d, app_id: %d", param->reg.status, param->reg.app_id);
        if (param->reg.status == ESP_GATT_OK) {
            midi_profile_tab[MIDI_PROFILE_APP_IDX].gatts_if = gatts_if;
            ESP_LOGI(TAG, "Starting to create MIDI service table...");
            esp_err_t ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, MIDI_IDX_NB, SVC_INST_ID);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to create attribute table: %d", ret);
            }
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT, status: %d, svc_inst_id: %d", 
                 param->add_attr_tab.status, param->add_attr_tab.svc_inst_id);
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "Attribute table created successfully, starting service...");
            memcpy(midi_handle_table, param->add_attr_tab.handles, 
                   sizeof(midi_handle_table));
            esp_err_t ret = esp_ble_gatts_start_service(midi_handle_table[IDX_SVC]);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start service: %d", ret);
            }
        } else {
            ESP_LOGE(TAG, "Failed to create attribute table");
        }
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_START_EVT, status: %d, service_handle: %d",
                 param->start.status, param->start.service_handle);
        if (param->start.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "MIDI service started successfully");
        } else {
            ESP_LOGE(TAG, "Failed to start MIDI service");
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT");
        midi_profile_tab[MIDI_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
        midi_profile_tab[MIDI_PROFILE_APP_IDX].gatts_if = gatts_if;
        // Set LED color when connected
        set_led_color(0, 16, 0);  // RGB value, green intensity set to 64
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_READ_EVT, handle: %d", param->read.handle);
        if (param->read.handle == midi_handle_table[IDX_CHAR_VAL_A]) {
            ESP_LOGI(TAG, "Reading MIDI characteristic value");
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT, handle: %d, write_len: %d, is_prep: %d",
                 param->write.handle, param->write.len, param->write.is_prep);
        
        if (!param->write.is_prep) {
            if (param->write.handle == midi_handle_table[IDX_CHAR_VAL_A]) {
                ESP_LOGI(TAG, "Received MIDI data write request");
                ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
                
                // Check if it's a SysEx message
                if (param->write.len > 3 && 
                    param->write.value[2] == 0xF0 && 
                    param->write.value[param->write.len-1] == 0xF7) {
                    parse_sysex(param->write.value, param->write.len);
                }
                
                if (midi_callback && param->write.len > 0) {
                    midi_callback(param->write.value, param->write.len);
                }
            } else if (param->write.handle == midi_handle_table[IDX_CHAR_CFG_A]) {
                if (param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    ESP_LOGI(TAG, "CCCD value updated to: 0x%04x", descr_value);
                    notifications_enabled = (descr_value == 0x0001);
                    if (notifications_enabled) {
                        ESP_LOGI(TAG, "Client enabled MIDI notifications");
                    } else {
                        ESP_LOGI(TAG, "Client disabled MIDI notifications");
                    }
                }
            }
        }
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU: %d", param->mtu.mtu);
        blemidi_mtu = param->mtu.mtu - 3;
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status: %d", param->conf.status);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason: 0x%x", param->disconnect.reason);
        // Turn off LED when disconnected
        set_led_color(16, 0, 0);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_DELETE_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_DELETE_EVT");
        break;

    default:
        ESP_LOGI(TAG, "Unhandled GATTS event: %d", event);
        break;
    }
} 

// Modify send function
esp_err_t ble_midi_send_message(uint8_t *data, size_t len) {
    if (!midi_profile_tab[MIDI_PROFILE_APP_IDX].conn_id) {
        return ESP_ERR_INVALID_STATE;
    }

    // Add message to buffer
    blemidi_outbuffer_push(0, data, len);

    // If message is large or buffer is almost full, flush immediately
    if (len >= (blemidi_mtu - 3) || blemidi_outbuffer_len[0] >= (blemidi_mtu - 3)) {
        blemidi_outbuffer_flush(0);
    }

    return ESP_OK;
} 

// Add callback function setting interface
void ble_midi_set_callback(midi_callback_t callback) {
    midi_callback = callback;
} 

esp_err_t ble_midi_init(void)
{
    esp_err_t ret;

    // Initialize Bluetooth controller and stack
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth controller: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable Bluetooth controller: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth stack: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable Bluetooth stack: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set local MTU size
    ret = esp_ble_gatt_set_local_mtu(GATTS_MIDI_CHAR_VAL_LEN_MAX);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set local MTU: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GATTS callback
    ret = esp_ble_gatts_register_callback(gatts_profile_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GATTS callback: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GAP callback
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set device name
    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure broadcast data
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure broadcast data: %s", esp_err_to_name(ret));
        return ret;
    }
    adv_config_done |= adv_config_flag;

    // Configure scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure scan response data: %s", esp_err_to_name(ret));
        return ret;
    }
    adv_config_done |= scan_rsp_config_flag;

    // Register application
    ret = esp_ble_gatts_app_register(MIDI_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GATT application: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize LED
    init_led();

    ESP_LOGI(TAG, "BLE MIDI initialization completed");
    return ESP_OK;
}

esp_err_t ble_midi_send_data(uint8_t* data, uint16_t length) {
    if (!notifications_enabled) {
        ESP_LOGW(TAG, "Notifications not enabled, cannot send MIDI data");
        return ESP_FAIL;
    }
    
    esp_err_t ret = esp_ble_gatts_send_indicate(
        midi_profile_tab[MIDI_PROFILE_APP_IDX].gatts_if,
        midi_profile_tab[MIDI_PROFILE_APP_IDX].conn_id,
        midi_handle_table[IDX_CHAR_VAL_A],
        length,
        data,
        false);
        
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send MIDI data: %d", ret);
    }
    
    return ret;
}