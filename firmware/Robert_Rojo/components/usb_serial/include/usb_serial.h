#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include "esp_err.h"
#include "tinyusb_cdc_acm.h"

typedef struct {
    tinyusb_cdcacm_itf_t port;
    const char *name;
} usb_serial_t;

esp_err_t usb_serial_init_driver(void);
esp_err_t usb_serial_init(usb_serial_t *dev,
                          tinyusb_cdcacm_itf_t port,
                          const char *name);

bool usb_serial_connected(usb_serial_t *dev);
esp_err_t usb_serial_write(usb_serial_t *dev,
                           const uint8_t *data,
                           size_t len);
esp_err_t usb_serial_print(usb_serial_t *dev,
                           const char *text);
esp_err_t usb_serial_printf(usb_serial_t *dev,
                            const char *fmt,
                            ...);