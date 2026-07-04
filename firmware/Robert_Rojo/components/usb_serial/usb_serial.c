#include "usb_serial.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

static bool s_driver_installed = false;

esp_err_t usb_serial_init_driver(void)
{
    if (s_driver_installed) {
        return ESP_OK;
    }

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg),
                        "USB_SERIAL",
                        "tinyusb_driver_install failed");

    s_driver_installed = true;
    return ESP_OK;
}

esp_err_t usb_serial_init(usb_serial_t *dev,
                          tinyusb_cdcacm_itf_t port,
                          const char *name)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(usb_serial_init_driver(),
                        "USB_SERIAL",
                        "driver init failed");

    dev->port = port;
    dev->name = name;

    const tinyusb_config_cdcacm_t cdc_cfg = {
        .cdc_port = port,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };

    return tinyusb_cdcacm_init(&cdc_cfg);
}

bool usb_serial_connected(usb_serial_t *dev)
{
    if (dev == NULL) {
        return false;
    }

    return tud_mounted() && tud_cdc_n_connected(dev->port);
}

esp_err_t usb_serial_write(usb_serial_t *dev,
                           const uint8_t *data,
                           size_t len)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!usb_serial_connected(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    tinyusb_cdcacm_write_queue(dev->port, data, len);
    tinyusb_cdcacm_write_flush(dev->port, 0);

    return ESP_OK;
}

esp_err_t usb_serial_print(usb_serial_t *dev,
                           const char *text)
{
    return usb_serial_write(dev,
                            (const uint8_t *)text,
                            strlen(text));
}

esp_err_t usb_serial_printf(usb_serial_t *dev,
                            const char *fmt,
                            ...)
{
    char buffer[256];

    va_list args;
    va_start(args, fmt);

    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);

    va_end(args);

    if (len <= 0) {
        return ESP_FAIL;
    }

    size_t write_len = len < sizeof(buffer)
                       ? (size_t)len
                       : sizeof(buffer) - 1;

    return usb_serial_write(dev,
                            (const uint8_t *)buffer,
                            write_len);
}