#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb_midi.h"

static const char *TAG = "USB_MIDI";
static usb_midi_callback_t midi_callback = NULL;
static TaskHandle_t usb_host_task_handle = NULL;
static TaskHandle_t usb_midi_task_handle = NULL;
static bool usb_midi_running = false;

#define HOST_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY   3

// USB MIDI device information
typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    bool device_connected;
    SemaphoreHandle_t mutex;
} usb_midi_dev_t;

static usb_midi_dev_t midi_dev = {
    .client_hdl = NULL,
    .dev_addr = 0,
    .dev_hdl = NULL,
    .device_connected = false,
    .mutex = NULL
};

// MIDI data processing callback
static void handle_rx_data(usb_transfer_t *transfer)
{
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        if (midi_callback && transfer->actual_num_bytes > 0) {
            // Call user registered callback function
            midi_callback(transfer->data_buffer, transfer->actual_num_bytes);
        }

        // Continue submitting next transfer request
        esp_err_t err = usb_host_transfer_submit(transfer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Transfer submission failed: %s", esp_err_to_name(err));
            usb_host_transfer_free(transfer);
        }
    } else {
        ESP_LOGE(TAG, "Transfer error: %d", transfer->status);
        usb_host_transfer_free(transfer);
    }
}

// Start EP2 IN transfer
static void start_midi_transfer(usb_device_handle_t dev_hdl)
{
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(dev_hdl, &config_desc));

    // Find MIDI interface (Interface 3)
    int offset = 0;
    const usb_intf_desc_t *intf_desc = usb_parse_interface_descriptor(config_desc, 3, 0, &offset);
    if (intf_desc == NULL) {
        ESP_LOGE(TAG, "MIDI interface not found");
        return;
    }

    ESP_LOGI(TAG, "Found MIDI interface %d (Class: 0x%02x, SubClass: 0x%02x)",
             intf_desc->bInterfaceNumber,
             intf_desc->bInterfaceClass,
             intf_desc->bInterfaceSubClass);

    // Find EP2 IN endpoint
    offset = 0;
    const usb_ep_desc_t *ep_desc = usb_parse_endpoint_descriptor_by_address(
        config_desc, 3, 0, 0x82, &offset);

    if (ep_desc == NULL) {
        ESP_LOGE(TAG, "Endpoint 0x82 not found");
        return;
    }

    ESP_LOGI(TAG, "Found endpoint 0x82, max packet size: %d", ep_desc->wMaxPacketSize);

    usb_transfer_t *transfer = NULL;
    ESP_ERROR_CHECK(usb_host_transfer_alloc(ep_desc->wMaxPacketSize, 0, &transfer));

    transfer->device_handle = dev_hdl;
    transfer->bEndpointAddress = 0x82;  // EP2 IN
    transfer->callback = handle_rx_data;
    transfer->num_bytes = ep_desc->wMaxPacketSize;

    esp_err_t err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Transfer submission failed: %s", esp_err_to_name(err));
        usb_host_transfer_free(transfer);
    }
}

// Get device information
static void get_device_info(usb_device_handle_t dev_hdl)
{
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(dev_hdl, &dev_info));
    
    ESP_LOGI(TAG, "Device information:");
    ESP_LOGI(TAG, "  Speed: %s", (char *[]){"Low", "Full", "High"}[dev_info.speed]);
    ESP_LOGI(TAG, "  Configuration value: %d", dev_info.bConfigurationValue);
}

// Get device descriptor
static void get_device_desc(usb_device_handle_t dev_hdl)
{
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(dev_hdl, &dev_desc));
    
    ESP_LOGI(TAG, "Device descriptor:");
    ESP_LOGI(TAG, "  VID: 0x%04x", dev_desc->idVendor);
    ESP_LOGI(TAG, "  PID: 0x%04x", dev_desc->idProduct);
}

// USB client event callback
static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    usb_midi_dev_t *dev = (usb_midi_dev_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            xSemaphoreTake(dev->mutex, portMAX_DELAY);
            dev->dev_addr = event_msg->new_dev.address;
            ESP_LOGI(TAG, "New device found, address: %d", dev->dev_addr);
            
            ESP_ERROR_CHECK(usb_host_device_open(dev->client_hdl, dev->dev_addr, &dev->dev_hdl));
            
            // Get device information and descriptor
            get_device_info(dev->dev_hdl);
            get_device_desc(dev->dev_hdl);
            
            // Claim interface
            ESP_ERROR_CHECK(usb_host_interface_claim(dev->client_hdl, dev->dev_hdl, 3, 0));
            
            start_midi_transfer(dev->dev_hdl);
            dev->device_connected = true;
            xSemaphoreGive(dev->mutex);
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            xSemaphoreTake(dev->mutex, portMAX_DELAY);
            if (dev->dev_hdl != NULL) {
                ESP_LOGI(TAG, "Device disconnected");
                ESP_ERROR_CHECK(usb_host_device_close(dev->client_hdl, dev->dev_hdl));
                dev->dev_hdl = NULL;
                dev->dev_addr = 0;
                dev->device_connected = false;
            }
            xSemaphoreGive(dev->mutex);
            break;

        default:
            break;
    }
}

// USB host library task
static void usb_host_task(void *arg)
{
    ESP_LOGI(TAG, "USB host task started");
    
    // Configure and install USB host driver
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Notify main task that USB host library installation is complete
    xTaskNotifyGive(arg);

    bool has_clients = true;

    while (has_clients && usb_midi_running) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "USB host task cleanup...");
    ESP_ERROR_CHECK(usb_host_uninstall());
    
    vTaskDelete(NULL);
}

// USB MIDI task
static void usb_midi_task(void *arg)
{
    ESP_LOGI(TAG, "USB MIDI task started");

    // Create mutex
    midi_dev.mutex = xSemaphoreCreateMutex();
    if (midi_dev.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        vTaskDelete(NULL);
        return;
    }

    // Register USB host client
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = &midi_dev,
        },
    };

    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &midi_dev.client_hdl));

    while (usb_midi_running) {
        // Handle USB host events
        usb_host_client_handle_events(midi_dev.client_hdl, portMAX_DELAY);
    }

    // Cleanup resources
    if (midi_dev.dev_hdl != NULL) {
        ESP_ERROR_CHECK(usb_host_device_close(midi_dev.client_hdl, midi_dev.dev_hdl));
    }
    ESP_ERROR_CHECK(usb_host_client_deregister(midi_dev.client_hdl));
    
    vSemaphoreDelete(midi_dev.mutex);
    vTaskDelete(NULL);
}

esp_err_t usb_midi_init(usb_midi_config_t *config)
{
    if (usb_midi_running) {
        return ESP_ERR_INVALID_STATE;
    }

    // Save callback function
    midi_callback = config->data_callback;

    // Start USB host task
    usb_midi_running = true;
    BaseType_t task_created = xTaskCreatePinnedToCore(
        usb_host_task,
        "usb_host",
        4096,
        xTaskGetCurrentTaskHandle(),  // Pass current task handle for notification
        HOST_TASK_PRIORITY,
        &usb_host_task_handle,
        0
    );

    if (task_created != pdPASS) {
        usb_midi_running = false;
        return ESP_ERR_NO_MEM;
    }

    // Wait for USB host library installation to complete
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
        usb_midi_running = false;
        vTaskDelete(usb_host_task_handle);
        return ESP_ERR_TIMEOUT;
    }

    // Create USB MIDI task
    task_created = xTaskCreatePinnedToCore(
        usb_midi_task,
        "usb_midi",
        4096,
        NULL,
        config->task_priority,
        &usb_midi_task_handle,
        0
    );

    if (task_created != pdPASS) {
        usb_midi_running = false;
        ESP_ERROR_CHECK(usb_host_uninstall());
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t usb_midi_deinit(void)
{
    if (!usb_midi_running) {
        return ESP_ERR_INVALID_STATE;
    }

    usb_midi_running = false;
    
    // Wait for tasks to finish
    vTaskDelay(pdMS_TO_TICKS(100));

    // Tasks will clean up and delete themselves
    usb_host_task_handle = NULL;
    usb_midi_task_handle = NULL;
    
    return ESP_OK;
}

bool usb_midi_device_connected(void)
{
    if (!midi_dev.mutex) {
        return false;
    }
    
    bool connected = false;
    xSemaphoreTake(midi_dev.mutex, portMAX_DELAY);
    connected = midi_dev.device_connected;
    xSemaphoreGive(midi_dev.mutex);
    
    return connected;
}
