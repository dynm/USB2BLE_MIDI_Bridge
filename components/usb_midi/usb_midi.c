#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb_midi.h"
#include "usb_midi_descriptor.h"

static const char *TAG = "USB_MIDI";
#define HOST_TASK_PRIORITY 2
#define TX_QUEUE_EVENTS 256

static usb_midi_callback_t midi_callback;
static TaskHandle_t usb_host_task_handle;
static TaskHandle_t usb_midi_task_handle;
static atomic_bool usb_midi_running;
static SemaphoreHandle_t shutdown_done;

// All USB transfers and device lifecycle actions belong to the client task.
// Other tasks only copy events into the queue while holding mutex.
static struct {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t dev_hdl;
    usb_midi_interface_t intf;
    bool device_connected;
    bool interface_claimed;
    bool closing;
    atomic_bool rx_inflight;
    atomic_bool tx_inflight;
    atomic_bool rx_failed;
    atomic_bool tx_failed;
    atomic_bool reconnect_requested;
    atomic_uint rx_transfers, rx_bytes, tx_transfers, tx_bytes, rx_errors, tx_errors;
    uint8_t address;
    uint32_t session;
    uint8_t pending_address;
    usb_transfer_t *rx;
    usb_transfer_t *tx;
    SemaphoreHandle_t mutex;
    QueueHandle_t tx_queue;
} midi_dev;

static void handle_rx_data(usb_transfer_t *transfer)
{
    midi_dev.rx_inflight = false;
    if (midi_dev.closing || !usb_midi_running) {
        return;
    }
    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGW(TAG, "USB MIDI IN failed: %d", transfer->status);
        midi_dev.rx_failed = true;
        atomic_fetch_add(&midi_dev.rx_errors, 1);
        return;
    }
    atomic_fetch_add(&midi_dev.rx_transfers, 1);
    atomic_fetch_add(&midi_dev.rx_bytes, transfer->actual_num_bytes);
    if (midi_callback && transfer->actual_num_bytes > 0) {
        midi_callback(transfer->data_buffer, transfer->actual_num_bytes);
    }
    esp_err_t err = usb_host_transfer_submit(transfer);
    if (err == ESP_OK) {
        midi_dev.rx_inflight = true;
    } else {
        ESP_LOGW(TAG, "USB MIDI IN submit failed: %s", esp_err_to_name(err));
        midi_dev.rx_failed = true;
        atomic_fetch_add(&midi_dev.rx_errors, 1);
    }
}

static void handle_tx_done(usb_transfer_t *transfer)
{
    midi_dev.tx_inflight = false;
    if (!midi_dev.closing && usb_midi_running &&
        (transfer->status != USB_TRANSFER_STATUS_COMPLETED ||
         transfer->actual_num_bytes != transfer->num_bytes)) {
        ESP_LOGW(TAG, "USB MIDI OUT failed: status=%d, bytes=%d/%d; USB IN remains enabled",
                 transfer->status, transfer->actual_num_bytes, transfer->num_bytes);
        midi_dev.tx_failed = true;
        atomic_fetch_add(&midi_dev.tx_errors, 1);
    }
}

static void stop_endpoint(uint8_t address)
{
    if (!address || !midi_dev.interface_claimed) {
        return;
    }
    esp_err_t err = usb_host_endpoint_halt(midi_dev.dev_hdl, address);
    if (err == ESP_OK) {
        err = usb_host_endpoint_flush(midi_dev.dev_hdl, address);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Endpoint 0x%02x cleanup: %s", address, esp_err_to_name(err));
    }
}

static void begin_close(void)
{
    if (!midi_dev.dev_hdl || midi_dev.closing) {
        return;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    midi_dev.device_connected = false;
    xQueueReset(midi_dev.tx_queue);
    xSemaphoreGive(midi_dev.mutex);
    midi_dev.closing = true;
    stop_endpoint(midi_dev.intf.in_addr);
    stop_endpoint(midi_dev.intf.out_addr);
}

// Called only after handle_events returns: ESP-IDF retires its endpoint URBs
// after callbacks return, so releasing an interface inside a callback is unsafe.
static void finish_close(void)
{
    if (!midi_dev.closing || midi_dev.rx_inflight || midi_dev.tx_inflight) {
        return;
    }
    if (midi_dev.interface_claimed) {
        esp_err_t err = usb_host_interface_release(midi_dev.client_hdl, midi_dev.dev_hdl,
                                                   midi_dev.intf.interface_num);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "MIDI interface release: %s", esp_err_to_name(err));
            return;
        }
        midi_dev.interface_claimed = false;
    }
    esp_err_t err = usb_host_device_close(midi_dev.client_hdl, midi_dev.dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MIDI device close: %s", esp_err_to_name(err));
        return;
    }
    usb_host_transfer_free(midi_dev.rx);
    usb_host_transfer_free(midi_dev.tx);
    midi_dev.rx = NULL;
    midi_dev.tx = NULL;
    midi_dev.dev_hdl = NULL;
    memset(&midi_dev.intf, 0, sizeof(midi_dev.intf));
    midi_dev.closing = false;
    midi_dev.rx_failed = false;
    midi_dev.tx_failed = false;
}

static esp_err_t allocate_transfer(uint8_t address, uint16_t size,
                                    usb_transfer_cb_t callback, usb_transfer_t **transfer)
{
    if (!address) {
        return ESP_OK;
    }
    esp_err_t err = usb_host_transfer_alloc(size, 0, transfer);
    if (err == ESP_OK) {
        (*transfer)->device_handle = midi_dev.dev_hdl;
        (*transfer)->bEndpointAddress = address;
        (*transfer)->callback = callback;
        (*transfer)->num_bytes = size;
    }
    return err;
}

static void open_device(uint8_t address)
{
    esp_err_t err = usb_host_device_open(midi_dev.client_hdl, address, &midi_dev.dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Device open failed: %s", esp_err_to_name(err));
        return;
    }
    midi_dev.address = address;
    const usb_device_desc_t *device_desc;
    if (usb_host_get_device_descriptor(midi_dev.dev_hdl, &device_desc) == ESP_OK) {
        ESP_LOGI(TAG, "USB device VID=0x%04x PID=0x%04x",
                 device_desc->idVendor, device_desc->idProduct);
    }
    const usb_config_desc_t *config_desc;
    err = usb_host_get_active_config_descriptor(midi_dev.dev_hdl, &config_desc);
    if (err != ESP_OK) {
        goto failed;
    }
    if (!usb_midi_find_interface((const uint8_t *)config_desc,
                                 config_desc->wTotalLength, &midi_dev.intf)) {
        ESP_LOGW(TAG, "No supported MIDI 1.0 streaming interface");
        err = ESP_ERR_NOT_FOUND;
        goto failed;
    }
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, config_desc, config_desc->wTotalLength, ESP_LOG_DEBUG);
    err = usb_host_interface_claim(midi_dev.client_hdl, midi_dev.dev_hdl,
                                   midi_dev.intf.interface_num, midi_dev.intf.interface_alt);
    if (err != ESP_OK) {
        goto failed;
    }
    midi_dev.interface_claimed = true;
    err = allocate_transfer(midi_dev.intf.in_addr, midi_dev.intf.in_packet_size,
                            handle_rx_data, &midi_dev.rx);
    if (err != ESP_OK) {
        goto failed;
    }
    err = allocate_transfer(midi_dev.intf.out_addr, midi_dev.intf.out_packet_size,
                            handle_tx_done, &midi_dev.tx);
    if (err != ESP_OK) {
        goto failed;
    }
    if (midi_dev.rx) {
        err = usb_host_transfer_submit(midi_dev.rx);
        if (err != ESP_OK) {
            goto failed;
        }
        midi_dev.rx_inflight = true;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    if (++midi_dev.session == 0) {
        midi_dev.session = 1;
    }
    midi_dev.device_connected = true;
    xSemaphoreGive(midi_dev.mutex);
    ESP_LOGI(TAG, "MIDI interface %u alt %u: IN=0x%02x (%u), OUT=0x%02x (%u)",
             midi_dev.intf.interface_num, midi_dev.intf.interface_alt,
             midi_dev.intf.in_addr, midi_dev.intf.in_packet_size,
             midi_dev.intf.out_addr, midi_dev.intf.out_packet_size);
    if (!midi_dev.intf.out_addr) {
        ESP_LOGW(TAG, "No USB MIDI OUT endpoint; BLE to piano playback unavailable");
    }
    return;

failed:
    ESP_LOGW(TAG, "MIDI device setup failed: %s", esp_err_to_name(err));
    begin_close();
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    (void)arg;
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV && usb_midi_running) {
        if (!midi_dev.dev_hdl || midi_dev.closing) {
            midi_dev.pending_address = event_msg->new_dev.address;
        }
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE &&
               event_msg->dev_gone.dev_hdl == midi_dev.dev_hdl) {
        ESP_LOGI(TAG, "USB MIDI device disconnected");
        begin_close();
    }
}

static void submit_output(void)
{
    if (!midi_dev.device_connected || !midi_dev.tx || midi_dev.tx_inflight ||
        midi_dev.closing || midi_dev.rx_failed || midi_dev.tx_failed || !usb_midi_running) {
        return;
    }
    size_t len = 0;
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    while (len + 4 <= midi_dev.intf.out_packet_size &&
           xQueueReceive(midi_dev.tx_queue, midi_dev.tx->data_buffer + len, 0) == pdTRUE) {
        len += 4;
    }
    xSemaphoreGive(midi_dev.mutex);
    if (len == 0) {
        return;
    }
    midi_dev.tx->num_bytes = len;
    esp_err_t err = usb_host_transfer_submit(midi_dev.tx);
    if (err == ESP_OK) {
        midi_dev.tx_inflight = true;
        atomic_fetch_add(&midi_dev.tx_transfers, 1);
        atomic_fetch_add(&midi_dev.tx_bytes, len);
    } else {
        ESP_LOGW(TAG, "USB MIDI OUT submit failed: %s; USB IN remains enabled", esp_err_to_name(err));
        midi_dev.tx_failed = true;
        atomic_fetch_add(&midi_dev.tx_errors, 1);
    }
}

static void usb_host_task(void *arg)
{
    (void)arg;
    bool no_clients = false;
    bool all_free = false;
    while (!no_clients || !all_free) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            no_clients = true;
            esp_err_t err = usb_host_device_free_all();
            all_free = err == ESP_OK;
            if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
                ESP_ERROR_CHECK(err);
            }
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            all_free = true;
        }
    }
    ESP_ERROR_CHECK(usb_host_uninstall());
    xSemaphoreGive(shutdown_done);
    vTaskDelete(NULL);
}

static void usb_midi_task(void *arg)
{
    (void)arg;
    while (usb_midi_running || midi_dev.dev_hdl) {
        if (atomic_exchange(&midi_dev.reconnect_requested, false)) {
            midi_dev.pending_address = midi_dev.address;
            begin_close();
        }
        if (!usb_midi_running || midi_dev.rx_failed) {
            begin_close();
        }
        usb_host_client_handle_events(midi_dev.client_hdl, pdMS_TO_TICKS(10));
        finish_close();
        if (usb_midi_running && !midi_dev.dev_hdl && midi_dev.pending_address) {
            uint8_t address = midi_dev.pending_address;
            midi_dev.pending_address = 0;
            open_device(address);
        }
        submit_output();
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    ESP_ERROR_CHECK(usb_host_client_deregister(midi_dev.client_hdl));
    midi_dev.client_hdl = NULL;
    xSemaphoreGive(midi_dev.mutex);
    vTaskDelete(NULL);
}

esp_err_t usb_midi_init(usb_midi_config_t *config)
{
    if (!config || !config->data_callback) {
        return ESP_ERR_INVALID_ARG;
    }
    if (usb_host_task_handle || usb_midi_running) {
        return ESP_ERR_INVALID_STATE;
    }
    // Retain the public API mutex across reinitialization so a disconnected BLE
    // callback can safely query the driver even while its USB tasks shut down.
    if (!midi_dev.mutex) {
        midi_dev.mutex = xSemaphoreCreateMutex();
    }
    if (!midi_dev.mutex) {
        return ESP_ERR_NO_MEM;
    }
    midi_dev.tx_queue = xQueueCreate(TX_QUEUE_EVENTS, 4);
    shutdown_done = xSemaphoreCreateBinary();
    if (!midi_dev.tx_queue || !shutdown_done) {
        if (midi_dev.tx_queue) vQueueDelete(midi_dev.tx_queue);
        if (shutdown_done) vSemaphoreDelete(shutdown_done);
        midi_dev.tx_queue = NULL;
        shutdown_done = NULL;
        return ESP_ERR_NO_MEM;
    }
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        goto free_resources;
    }
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {.client_event_callback = client_event_cb},
    };
    err = usb_host_client_register(&client_config, &midi_dev.client_hdl);
    if (err != ESP_OK) {
        usb_host_uninstall();
        goto free_resources;
    }
    midi_callback = config->data_callback;
    midi_dev.pending_address = 0;
    usb_midi_running = true;
    if (xTaskCreatePinnedToCore(usb_host_task, "usb_host", 4096, NULL,
                               HOST_TASK_PRIORITY, &usb_host_task_handle, 0) != pdPASS) {
        usb_midi_running = false;
        usb_host_client_deregister(midi_dev.client_hdl);
        midi_dev.client_hdl = NULL;
        // Drain the no-clients event before uninstalling the host library.
        uint32_t flags;
        usb_host_lib_handle_events(0, &flags);
        usb_host_uninstall();
        err = ESP_ERR_NO_MEM;
        goto free_resources;
    }
    if (xTaskCreatePinnedToCore(usb_midi_task, "usb_midi", 4096, NULL,
                               config->task_priority, &usb_midi_task_handle, 0) != pdPASS) {
        usb_midi_running = false;
        usb_host_client_deregister(midi_dev.client_hdl);
        midi_dev.client_hdl = NULL;
        xSemaphoreTake(shutdown_done, portMAX_DELAY);
        usb_host_task_handle = NULL;
        err = ESP_ERR_NO_MEM;
        goto free_resources;
    }
    return ESP_OK;

free_resources:
    vQueueDelete(midi_dev.tx_queue);
    midi_dev.tx_queue = NULL;
    vSemaphoreDelete(shutdown_done);
    shutdown_done = NULL;
    return err;
}

esp_err_t usb_midi_deinit(void)
{
    if (!usb_host_task_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    usb_midi_running = false;
    if (midi_dev.client_hdl) {
        usb_host_client_unblock(midi_dev.client_hdl);
    }
    xSemaphoreGive(midi_dev.mutex);
    if (xSemaphoreTake(shutdown_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    vQueueDelete(midi_dev.tx_queue);
    midi_dev.tx_queue = NULL;
    vSemaphoreDelete(shutdown_done);
    shutdown_done = NULL;
    usb_host_task_handle = NULL;
    usb_midi_task_handle = NULL;
    return ESP_OK;
}

bool usb_midi_device_connected(void)
{
    if (!midi_dev.mutex) {
        return false;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    bool connected = usb_midi_running && midi_dev.device_connected;
    xSemaphoreGive(midi_dev.mutex);
    return connected;
}

uint32_t usb_midi_output_session(void)
{
    if (!midi_dev.mutex) {
        return 0;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    uint32_t session = usb_midi_running && midi_dev.device_connected && midi_dev.intf.out_addr && !midi_dev.tx_failed
                       ? midi_dev.session : 0;
    xSemaphoreGive(midi_dev.mutex);
    return session;
}

esp_err_t usb_midi_send_data(const uint8_t *data, size_t len, uint32_t session)
{
    if (!data || !len || len % 4) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!midi_dev.mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!usb_midi_running || !midi_dev.device_connected || !midi_dev.intf.out_addr ||
        midi_dev.tx_failed || session == 0 || session != midi_dev.session) {
        err = ESP_ERR_INVALID_STATE;
    } else if (len / 4 > uxQueueSpacesAvailable(midi_dev.tx_queue)) {
        err = ESP_ERR_NO_MEM;
    } else {
        for (size_t pos = 0; pos < len; pos += 4) {
            xQueueSend(midi_dev.tx_queue, data + pos, 0);
        }
        usb_host_client_unblock(midi_dev.client_hdl);
    }
    xSemaphoreGive(midi_dev.mutex);
    return err;
}

void usb_midi_reset_output(void)
{
    if (!midi_dev.mutex) {
        return;
    }
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    if (usb_midi_running && midi_dev.device_connected && midi_dev.intf.out_addr && !midi_dev.tx_failed) {
        xQueueReset(midi_dev.tx_queue);
        // Terminate any SysEx already delivered before the connection was lost.
        const uint8_t eox[4] = {0x5, 0xF7, 0, 0};
        xQueueSend(midi_dev.tx_queue, eox, 0);
        const uint8_t controllers[] = {64, 120, 123};
        for (uint8_t channel = 0; channel < 16; channel++) {
            for (size_t i = 0; i < sizeof(controllers); i++) {
                uint8_t event[4] = {0xB, 0xB0 | channel, controllers[i], 0};
                xQueueSend(midi_dev.tx_queue, event, 0);
            }
        }
        usb_host_client_unblock(midi_dev.client_hdl);
    }
    xSemaphoreGive(midi_dev.mutex);
}

void usb_midi_get_status(usb_midi_status_t *status)
{
    memset(status, 0, sizeof(*status));
    if (!midi_dev.mutex) return;
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    status->connected = usb_midi_running && midi_dev.device_connected;
    if (status->connected) {
        status->in_endpoint = midi_dev.intf.in_addr;
        status->out_endpoint = midi_dev.intf.out_addr;
        status->session = midi_dev.session;
        status->queued_events = uxQueueMessagesWaiting(midi_dev.tx_queue);
    }
    status->rx_active = midi_dev.rx_inflight;
    status->tx_active = midi_dev.tx_inflight;
    status->tx_failed = midi_dev.tx_failed;
    status->rx_transfers = midi_dev.rx_transfers;
    status->rx_bytes = midi_dev.rx_bytes;
    status->tx_transfers = midi_dev.tx_transfers;
    status->tx_bytes = midi_dev.tx_bytes;
    status->rx_errors = midi_dev.rx_errors;
    status->tx_errors = midi_dev.tx_errors;
    xSemaphoreGive(midi_dev.mutex);
}

esp_err_t usb_midi_reconnect(void)
{
    if (!midi_dev.mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    if (!usb_midi_running || !midi_dev.client_hdl) {
        xSemaphoreGive(midi_dev.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    midi_dev.reconnect_requested = true;
    usb_host_client_unblock(midi_dev.client_hdl);
    xSemaphoreGive(midi_dev.mutex);
    return ESP_OK;
}
