#include <stdio.h>
#include <string.h>
#include <math.h>
#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "IMU";

SemaphoreHandle_t xSemaphoreImu = NULL;

#define BNO08X_I2C_TIMEOUT_MS 100
#define BNO08X_PRODUCT_ID_TIMEOUT_MS 500
#define BNO08X_Q_ACCEL 256.0f
#define BNO08X_Q_GYRO 512.0f
#define BNO08X_Q_MAG 16.0f
#define BNO08X_Q_QUAT 16384.0f
#define BNO08X_Q_ROT_ACCURACY 4096.0f
#define BNO08X_PI 3.14159265358979323846f
#define BNO08X_RAD_TO_DEG 57.29577951308232f
#define BNO08X_TARE_AXIS_Z 0x04
#define BNO08X_TARE_AXIS_ALL 0x07
#define BNO08X_TARE_SUBCOMMAND_NOW 0x00
#define BNO08X_TARE_SUBCOMMAND_PERSIST 0x01
#define BNO08X_TARE_SUBCOMMAND_CLEAR 0x02
#define BNO08X_TARE_BASIS_ROTATION_VECTOR 0x00

void IRAM_ATTR ImuISRHandler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSemaphoreImu != NULL)
    {
        xSemaphoreGiveFromISR(xSemaphoreImu, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}
static esp_err_t bno08x_gpio_init(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    if (xSemaphoreImu == NULL)
    {
        xSemaphoreImu = xSemaphoreCreateBinary();
        if (xSemaphoreImu == NULL)
            return ESP_ERR_NO_MEM;
    }
    esp_err_t err;
    gpio_config_t gpioConfig = {
        .pin_bit_mask = (1ULL << bno08x->config.int_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};

    if (bno08x->config.rst_pin != GPIO_NUM_NC)
    {
        gpio_config_t rst_cfg = {
            .pin_bit_mask = (1ULL << bno08x->config.rst_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};

        err = gpio_config(&rst_cfg);
        if (err != ESP_OK)
            return err;

        gpio_set_level(bno08x->config.rst_pin, 1);
    }

    err = gpio_config(&gpioConfig);
    if (err != ESP_OK)
        return err;

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;

    err = gpio_isr_handler_add(
        bno08x->config.int_pin,
        ImuISRHandler,
        NULL);

    return err;
}

static bool bno08x_data_available(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return false;

    // En BNO08x, INT activo en bajo significa que hay datos pendientes.
    return gpio_get_level(bno08x->config.int_pin) == 0;
}

static esp_err_t bno08x_wait_data(bno08x_t *bno08x, uint32_t timeout_ms)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    if (bno08x_data_available(bno08x))
        return ESP_OK;

    // La ISR solo libera el semaforo; la lectura I2C ocurre en la tarea.
    if (xSemaphoreTake(xSemaphoreImu, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
        return ESP_OK;

    return ESP_ERR_TIMEOUT;
}

static esp_err_t bno08x_i2c_write(bno08x_t *bno08x,
                                  const uint8_t *data,
                                  size_t len)
{
    if ((bno08x == NULL) || (data == NULL) || (len == 0))
        return ESP_ERR_INVALID_ARG;

    if (bno08x->i2c_dev == NULL)
        return ESP_ERR_INVALID_STATE;

    return i2c_master_transmit(
        bno08x->i2c_dev,
        data,
        len,
        BNO08X_I2C_TIMEOUT_MS);
}

static esp_err_t bno08x_hw_reset(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    if (bno08x->config.rst_pin == GPIO_NUM_NC)
        return ESP_ERR_NOT_SUPPORTED;

    gpio_set_level(bno08x->config.rst_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(bno08x->config.rst_pin, 1);

    /*
     * El SH-2 tarda un poco en arrancar.
     */
    vTaskDelay(pdMS_TO_TICKS(250));

    return ESP_OK;
}

static esp_err_t bno08x_i2c_read(bno08x_t *bno08x,
                                 uint8_t *data,
                                 size_t len)
{
    if ((bno08x == NULL) || (data == NULL) || (len == 0))
        return ESP_ERR_INVALID_ARG;

    if (bno08x->i2c_dev == NULL)
        return ESP_ERR_INVALID_STATE;

    return i2c_master_receive(
        bno08x->i2c_dev,
        data,
        len,
        BNO08X_I2C_TIMEOUT_MS);
}


static esp_err_t bno08x_send_packet(bno08x_t *bno08x,
                                    uint8_t channel,
                                    const uint8_t *payload,
                                    uint16_t payload_len)
{
    if ((bno08x == NULL) || (payload == NULL && payload_len > 0))
        return ESP_ERR_INVALID_ARG;

    if (channel >= 6)
        return ESP_ERR_INVALID_ARG;

    uint16_t packet_len = payload_len + 4;

    if (packet_len > sizeof(bno08x->tx_buffer))
        return ESP_ERR_INVALID_SIZE;

    uint8_t *packet = bno08x->tx_buffer;

    packet[0] = packet_len & 0xFF;
    packet[1] = (packet_len >> 8) & 0xFF;
    packet[2] = channel;
    packet[3] = bno08x->tx_seq[channel]++;

    if (payload_len > 0)
        memcpy(&packet[4], payload, payload_len);

    return bno08x_i2c_write(bno08x, packet, packet_len);
}

static esp_err_t bno08x_read_packet(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t header[4];

    // Bloquea hasta que INT indique datos disponibles o hasta timeout.
    esp_err_t err = bno08x_wait_data(bno08x, 100);
    if (err != ESP_OK)
        return err;

    bno08x->irq_timestamp_us = esp_timer_get_time();

    // Primero se lee el header SHTP para saber canal, secuencia y longitud.
    err = bno08x_i2c_read(bno08x, header, 4);
    if (err != ESP_OK)
        return err;

    uint16_t packet_len = ((uint16_t)header[1] << 8) | header[0];
    packet_len &= 0x7FFF;

    if (packet_len < 4 || packet_len > sizeof(bno08x->rx_buffer))
        return ESP_ERR_INVALID_SIZE;

    bno08x->rx_buffer[0] = header[0];
    bno08x->rx_buffer[1] = header[1];
    bno08x->rx_buffer[2] = header[2];
    bno08x->rx_buffer[3] = header[3];

    uint16_t payload_len = packet_len - 4;

    // Luego se lee el payload exacto indicado por el header.
    if (payload_len > 0)
    {
        err = bno08x_i2c_read(
            bno08x,
            &bno08x->rx_buffer[4],
            payload_len);

        if (err != ESP_OK)
            return err;
    }

    bno08x->rx_length = packet_len;

    if (bno08x->rx_buffer[2] == BNO08X_CHANNEL_EXECUTABLE)
    {
        // Este canal aparece cuando el SH-2 arranca/reinicia.
        bno08x->reset_detected = true;
    }

    return ESP_OK;
}

static int16_t bno08x_i16_le(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static uint16_t bno08x_u16_le(const uint8_t *data)
{
    return ((uint16_t)data[1] << 8) | data[0];
}

static uint32_t bno08x_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) |
           data[0];
}

static int64_t bno08x_report_timestamp_us(bno08x_t *bno08x,
                                          const bno08x_report_header_t *header)
{
    return bno08x->irq_timestamp_us - bno08x_get_delay_us(header);
}

static void bno08x_clear_update_flags(bno08x_data_t *data)
{
    data->accel_updated = false;
    data->gyro_updated = false;
    data->mag_updated = false;
    data->linear_accel_updated = false;
    data->gravity_updated = false;
    data->rotation_updated = false;
    data->game_rotation_updated = false;
    data->geomag_rotation_updated = false;
    data->gyro_rotation_updated = false;
    data->step_count_updated = false;
}

static esp_err_t bno08x_parse_vector3(bno08x_t *bno08x,
                                      const uint8_t *payload,
                                      uint16_t payload_len,
                                      float scale)
{
    if (payload_len < 10)
        return ESP_ERR_INVALID_SIZE;

    const bno08x_report_header_t *header =
        (const bno08x_report_header_t *)payload;

    // Los reports vectoriales comparten formato: header + x/y/z en Q-point.
    bno08x_vector3_t sample = {
        .x = (float)bno08x_i16_le(&payload[4]) / scale,
        .y = (float)bno08x_i16_le(&payload[6]) / scale,
        .z = (float)bno08x_i16_le(&payload[8]) / scale,
    };
    bno08x_accuracy_t accuracy = (bno08x_accuracy_t)bno08x_get_accuracy(header);
    int64_t timestamp_us = bno08x_report_timestamp_us(bno08x, header);

    switch (header->report_id)
    {
    case BNO08X_REPORTID_ACCELEROMETER:
        bno08x->data.accel = sample;
        bno08x->data.accel_accuracy = accuracy;
        bno08x->data.accel_timestamp_us = timestamp_us;
        bno08x->data.accel_updated = true;
        break;

    case BNO08X_REPORTID_GYROSCOPE:
        bno08x->data.gyro = sample;
        bno08x->data.gyro_accuracy = accuracy;
        bno08x->data.gyro_timestamp_us = timestamp_us;
        bno08x->data.gyro_updated = true;
        break;

    case BNO08X_REPORTID_MAGNETOMETER:
        bno08x->data.mag = sample;
        bno08x->data.mag_accuracy = accuracy;
        bno08x->data.mag_timestamp_us = timestamp_us;
        bno08x->data.mag_updated = true;
        break;

    case BNO08X_REPORTID_LINEAR_ACCELERATION:
        bno08x->data.linear_accel = sample;
        bno08x->data.linear_accel_accuracy = accuracy;
        bno08x->data.linear_accel_timestamp_us = timestamp_us;
        bno08x->data.linear_accel_updated = true;
        break;

    case BNO08X_REPORTID_GRAVITY:
        bno08x->data.gravity = sample;
        bno08x->data.gravity_accuracy = accuracy;
        bno08x->data.gravity_timestamp_us = timestamp_us;
        bno08x->data.gravity_updated = true;
        break;

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

static esp_err_t bno08x_parse_quaternion(bno08x_t *bno08x,
                                         const uint8_t *payload,
                                         uint16_t payload_len)
{
    if (payload_len < 12)
        return ESP_ERR_INVALID_SIZE;

    const bno08x_report_header_t *header =
        (const bno08x_report_header_t *)payload;

    // Quaternion viene como i/j/k/real en Q14; accuracy puede venir al final.
    bno08x_quaternion_t sample = {
        .i = (float)bno08x_i16_le(&payload[4]) / BNO08X_Q_QUAT,
        .j = (float)bno08x_i16_le(&payload[6]) / BNO08X_Q_QUAT,
        .k = (float)bno08x_i16_le(&payload[8]) / BNO08X_Q_QUAT,
        .real = (float)bno08x_i16_le(&payload[10]) / BNO08X_Q_QUAT,
        .accuracy_rad = 0.0f,
    };

    if (payload_len >= 14)
    {
        sample.accuracy_rad =
            (float)bno08x_u16_le(&payload[12]) / BNO08X_Q_ROT_ACCURACY;
    }

    bno08x_accuracy_t accuracy = (bno08x_accuracy_t)bno08x_get_accuracy(header);
    int64_t timestamp_us = bno08x_report_timestamp_us(bno08x, header);

    switch (header->report_id)
    {
    case BNO08X_REPORTID_ROTATION_VECTOR:
        bno08x->data.rotation_vector = sample;
        bno08x->data.rotation_accuracy = accuracy;
        bno08x->data.rotation_timestamp_us = timestamp_us;
        bno08x->data.rotation_updated = true;
        break;

    case BNO08X_REPORTID_GAME_ROTATION_VECTOR:
        bno08x->data.game_rotation_vector = sample;
        bno08x->data.game_rotation_accuracy = accuracy;
        bno08x->data.game_rotation_timestamp_us = timestamp_us;
        bno08x->data.game_rotation_updated = true;
        break;

    case BNO08X_REPORTID_GEOMAG_ROTATION_VECTOR:
        bno08x->data.geomag_rotation_vector = sample;
        bno08x->data.geomag_rotation_accuracy = accuracy;
        bno08x->data.geomag_rotation_timestamp_us = timestamp_us;
        bno08x->data.geomag_rotation_updated = true;
        break;

    case BNO08X_REPORTID_GYRO_ROTATION_VECTOR:
        bno08x->data.gyro_rotation_vector = sample;
        bno08x->data.gyro_rotation_accuracy = accuracy;
        bno08x->data.gyro_rotation_timestamp_us = timestamp_us;
        bno08x->data.gyro_rotation_updated = true;
        break;

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

static esp_err_t bno08x_parse_report(bno08x_t *bno08x)
{
    if (bno08x->rx_length < 5)
        return ESP_ERR_INVALID_SIZE;

    // rx_buffer contiene header SHTP en [0..3] y report SH-2 desde [4].
    uint8_t channel = bno08x->rx_buffer[2];
    uint16_t payload_len = bno08x->rx_length - 4;
    const uint8_t *payload = &bno08x->rx_buffer[4];
    uint8_t report_id = payload[0];

    if (channel != BNO08X_CHANNEL_INPUT_REPORTS &&
        channel != BNO08X_CHANNEL_WAKE_REPORTS &&
        channel != BNO08X_CHANNEL_GYRO_ROTATION)
    {
        // Paquetes de control/product-id no actualizan datos de sensores.
        return ESP_ERR_NOT_SUPPORTED;
    }

    switch (report_id)
    {
    case BNO08X_REPORTID_ACCELEROMETER:
    case BNO08X_REPORTID_LINEAR_ACCELERATION:
    case BNO08X_REPORTID_GRAVITY:
        return bno08x_parse_vector3(
            bno08x,
            payload,
            payload_len,
            BNO08X_Q_ACCEL);

    case BNO08X_REPORTID_GYROSCOPE:
        return bno08x_parse_vector3(
            bno08x,
            payload,
            payload_len,
            BNO08X_Q_GYRO);

    case BNO08X_REPORTID_MAGNETOMETER:
        return bno08x_parse_vector3(
            bno08x,
            payload,
            payload_len,
            BNO08X_Q_MAG);

    case BNO08X_REPORTID_ROTATION_VECTOR:
    case BNO08X_REPORTID_GAME_ROTATION_VECTOR:
    case BNO08X_REPORTID_GEOMAG_ROTATION_VECTOR:
    case BNO08X_REPORTID_GYRO_ROTATION_VECTOR:
        return bno08x_parse_quaternion(bno08x, payload, payload_len);

    case BNO08X_REPORTID_STEP_COUNTER:
        if (payload_len < 12)
            return ESP_ERR_INVALID_SIZE;
        bno08x->data.step_count = bno08x_u32_le(&payload[8]);
        bno08x->data.step_count_updated = true;
        return ESP_OK;

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t bno08x_wait_product_id_response(bno08x_t *bno08x)
{
    int64_t deadline_us =
        esp_timer_get_time() + ((int64_t)BNO08X_PRODUCT_ID_TIMEOUT_MS * 1000);

    while (esp_timer_get_time() < deadline_us)
    {
        esp_err_t err = bno08x_read_packet(bno08x);
        if (err == ESP_ERR_TIMEOUT)
            continue;
        if (err != ESP_OK)
            return err;

        if (bno08x->rx_length >= 5 &&
            bno08x->rx_buffer[2] == BNO08X_CHANNEL_CONTROL &&
            bno08x->rx_buffer[4] == BNO08X_CONTROL_PRODUCT_ID_RESPONSE)
        {
            ESP_LOGI(TAG, "Product ID response received");
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "Product ID response timeout");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t bno08x_request_product_id(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t payload = BNO08X_CONTROL_PRODUCT_ID_REQUEST;

    return bno08x_send_packet(
        bno08x,
        BNO08X_CHANNEL_CONTROL,
        &payload,
        sizeof(payload)
    );
}

static esp_err_t bno08x_send_command(bno08x_t *bno08x,
                                     uint8_t command,
                                     const uint8_t params[9])
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    /*
     * SH-2 Command Request:
     * byte 0 = report ID, byte 1 = command sequence, byte 2 = command,
     * bytes 3..11 = parametros del comando. La respuesta llega despues como
     * COMMAND_RESPONSE por el canal de control.
     */
    uint8_t payload[12] = {
        BNO08X_CONTROL_COMMAND_REQUEST,
        bno08x->command_seq++,
        command,
        0,
    };

    if (params != NULL)
    {
        memcpy(&payload[3], params, 9);
    }

    return bno08x_send_packet(
        bno08x,
        BNO08X_CHANNEL_CONTROL,
        payload,
        sizeof(payload));
}

static esp_err_t bno08x_enable_report(bno08x_t *bno08x,
                                      uint8_t report_id,
                                      uint32_t interval_us)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    bno08x_set_feature_t cmd = {
        .report_id = BNO08X_CONTROL_SET_FEATURE_COMMAND,
        .feature_report_id = report_id,
        .feature_flags = 0,
        .change_sensitivity = 0,
        .report_interval_us = interval_us,
        .batch_interval_us = 0,
        .sensor_specific = 0
    };

    return bno08x_send_packet(
        bno08x,
        BNO08X_CHANNEL_CONTROL,
        (const uint8_t *)&cmd,
        sizeof(cmd)
    );
}


static esp_err_t bno08x_device_create(bno08x_t *bno08x, bno08x_config_t *config)
{
    if ((bno08x == NULL) || (config == NULL))
        return ESP_ERR_INVALID_ARG;

    bno08x->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address,
        .scl_speed_hz = config->scl_speed_hz,
    };

    ESP_LOGI(TAG, "device_create for bno08x sensors on ADDR %X", bno08x->dev_cfg.device_address);
    if (config->bus_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    bno08x->bus_handle = config->bus_handle;
    esp_err_t err = i2c_master_bus_add_device(bno08x->bus_handle, &bno08x->dev_cfg, &bno08x->i2c_dev);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "device_create success on 0x%x", bno08x->dev_cfg.device_address);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "device_create error on 0x%x", bno08x->dev_cfg.device_address);
        return err;
    }
}

esp_err_t bno08x_init(bno08x_t *bno08x, const bno08x_config_t *config)
{
    if ((bno08x == NULL) || (config == NULL))
        return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    memset(&bno08x->data, 0, sizeof(bno08x->data));
    memset(bno08x->tx_seq, 0, sizeof(bno08x->tx_seq));
    memset(bno08x->rx_seq, 0, sizeof(bno08x->rx_seq));
    bno08x->command_seq = 0;
    bno08x->reset_detected = false;

    bno08x->config = *config;

    err = bno08x_device_create(bno08x, (bno08x_config_t *)config);
    if (err != ESP_OK)
        return err;

    err = bno08x_gpio_init(bno08x);
    if (err != ESP_OK)
        return err;

    err = bno08x_hw_reset(bno08x);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(300));

    /*
     * Después del reset, el BNO08X suele enviar paquetes iniciales.
     * Conviene leerlos/descartarlos antes de configurar reports.
     */
    for (int i = 0; i < 5; i++)
    {
        if (bno08x_data_available(bno08x))
        {
            bno08x_read_packet(bno08x);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    err = bno08x_request_product_id(bno08x);
    if (err != ESP_OK)
        return err;

    err = bno08x_wait_product_id_response(bno08x);
    if (err != ESP_OK)
        return err;

    err = bno08x_set_reports(bno08x);
    if (err != ESP_OK)
        return err;

    ESP_LOGI(TAG, "BNO08X initialized");

    return ESP_OK;
}

esp_err_t bno08x_set_reports(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    const bno08x_config_t *config = &bno08x->config;

    /*
     * Reenvia SET_FEATURE_COMMAND para cada sensor habilitado. El BNO08x
     * pierde esta configuracion si reinicia, por eso esta funcion queda
     * publica para usarse despues de bno08x_was_reset().
     */
    if (config->enable_accel)
    {
        err = bno08x_enable_report(
            bno08x,
            BNO08X_REPORTID_ACCELEROMETER,
            config->accel_interval_us);
        if (err != ESP_OK)
            return err;
    }

    if (config->enable_gyro)
    {
        err = bno08x_enable_report(
            bno08x,
            BNO08X_REPORTID_GYROSCOPE,
            config->gyro_interval_us);
        if (err != ESP_OK)
            return err;
    }

    if (config->enable_mag)
    {
        err = bno08x_enable_report(
            bno08x,
            BNO08X_REPORTID_MAGNETOMETER,
            config->mag_interval_us);
        if (err != ESP_OK)
            return err;
    }

    if (config->enable_rotation_vector)
    {
        err = bno08x_enable_report(
            bno08x,
            BNO08X_REPORTID_ROTATION_VECTOR,
            config->rotation_interval_us);
        if (err != ESP_OK)
            return err;
    }

    if (config->enable_linear_accel)
    {
        err = bno08x_enable_report(
            bno08x,
            BNO08X_REPORTID_LINEAR_ACCELERATION,
            config->linear_accel_interval_us);
        if (err != ESP_OK)
            return err;
    }

    if (config->enable_gravity)
    {
        err = bno08x_enable_report(
            bno08x,
            BNO08X_REPORTID_GRAVITY,
            config->gravity_interval_us);
        if (err != ESP_OK)
            return err;
    }

    return ESP_OK;
}

esp_err_t bno08x_read_data(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return ESP_ERR_INVALID_ARG;

    // Cada llamada marca solo lo que llega en este paquete nuevo.
    bno08x_clear_update_flags(&bno08x->data);

    // Internamente espera INT mediante semaforo y despues lee por I2C.
    esp_err_t err = bno08x_read_packet(bno08x);
    if (err != ESP_OK)
        return err;

    // Si el paquete no es de sensor, se ignora sin tratarlo como error fatal.
    err = bno08x_parse_report(bno08x);
    if (err == ESP_ERR_NOT_SUPPORTED)
        return ESP_OK;

    return err;
}

const bno08x_data_t *bno08x_get_data(const bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return NULL;

    // No copia datos; devuelve puntero a la ultima muestra interna.
    return &bno08x->data;
}

bool bno08x_was_reset(bno08x_t *bno08x)
{
    if (bno08x == NULL)
        return false;

    /*
     * Se consume una sola vez. Si devuelve true, el main debe llamar
     * bno08x_set_reports() para volver a habilitar los sensores activos.
     */
    bool was_reset = bno08x->reset_detected;
    bno08x->reset_detected = false;
    return was_reset;
}

void bno08x_quat_to_euler_deg(const bno08x_quaternion_t *quat,
                              float *roll_deg,
                              float *pitch_deg,
                              float *yaw_deg)
{
    if (quat == NULL)
        return;

    float qi = quat->i;
    float qj = quat->j;
    float qk = quat->k;
    float qr = quat->real;

    /*
     * El BNO08x entrega quaternion i,j,k,real. Esta conversion deja roll,
     * pitch y yaw en grados para que la aplicacion no repita trigonometria.
     */
    float sinr_cosp = 2.0f * (qr * qi + qj * qk);
    float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qr * qj - qk * qi);
    float pitch;
    if (sinp >= 1.0f)
    {
        pitch = BNO08X_PI / 2.0f;
    }
    else if (sinp <= -1.0f)
    {
        pitch = -BNO08X_PI / 2.0f;
    }
    else
    {
        pitch = asinf(sinp);
    }

    float siny_cosp = 2.0f * (qr * qk + qi * qj);
    float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    if (roll_deg != NULL)
        *roll_deg = roll * BNO08X_RAD_TO_DEG;
    if (pitch_deg != NULL)
        *pitch_deg = pitch * BNO08X_RAD_TO_DEG;
    if (yaw_deg != NULL)
        *yaw_deg = yaw * BNO08X_RAD_TO_DEG;
}

esp_err_t bno08x_tare_now(bno08x_t *bno08x, bool z_only)
{
    /*
     * Tare temporal: ajusta la orientacion actual como referencia. z_only
     * corrige solo yaw; false corrige todos los ejes.
     */
    uint8_t params[9] = {
        BNO08X_TARE_SUBCOMMAND_NOW,
        z_only ? BNO08X_TARE_AXIS_Z : BNO08X_TARE_AXIS_ALL,
        BNO08X_TARE_BASIS_ROTATION_VECTOR,
    };

    return bno08x_send_command(bno08x, BNO08X_COMMAND_TARE, params);
}

esp_err_t bno08x_save_tare(bno08x_t *bno08x)
{
    /*
     * Persiste el tare actual. El sensor debe haber recibido primero
     * bno08x_tare_now().
     */
    uint8_t params[9] = {
        BNO08X_TARE_SUBCOMMAND_PERSIST,
    };

    return bno08x_send_command(bno08x, BNO08X_COMMAND_TARE, params);
}

esp_err_t bno08x_clear_tare(bno08x_t *bno08x)
{
    /*
     * Limpia el tare guardado y vuelve a la referencia normal del sensor.
     */
    uint8_t params[9] = {
        BNO08X_TARE_SUBCOMMAND_CLEAR,
    };

    return bno08x_send_command(bno08x, BNO08X_COMMAND_TARE, params);
}

esp_err_t bno08x_save_calibration(bno08x_t *bno08x)
{
    /*
     * Solicita guardar el DCD/calibracion dinamica. La confirmacion exacta
     * llega por COMMAND_RESPONSE, que puede parsearse despues si necesitas
     * saber si el sensor acepto o rechazo el comando.
     */
    return bno08x_send_command(
        bno08x,
        BNO08X_COMMAND_DCD_PERIOD_SAVE,
        NULL);
}
