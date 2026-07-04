#include "usb_serial.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "APP";

usb_serial_t USBSerialIMU;
usb_serial_t USBSerialGPS;

void app_main(void)
{
    ESP_ERROR_CHECK(usb_serial_init(&USBSerialIMU, TINYUSB_CDC_ACM_0, "IMU"));

    ESP_ERROR_CHECK(usb_serial_init(&USBSerialGPS, TINYUSB_CDC_ACM_1, "GPS"));

    int count = 0;

    while (1)
    {
        usb_serial_printf(&USBSerialIMU,
                          "IMU,count,%d\r\n",
                          count);

        usb_serial_printf(&USBSerialGPS,
                          "GPS,count,%d\r\n",
                          count);

        ESP_LOGI(TAG, "Enviado count=%d", count);

        count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}