#include <stdio.h>
#include <inttypes.h>
#include "string.h"

#include "gps.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gps";

static esp_err_t gps_device_create(gps_t *gps)
{
    if (gps == NULL)
        return ESP_ERR_INVALID_ARG;

    if (gps->bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Evita registrar dos veces el mismo dispositivo
     * y perder el handle anterior.
     */
    if (gps->i2c_dev != NULL)
    {
        ESP_LOGW(TAG, "GPS I2C device is already created");
        return ESP_ERR_INVALID_STATE;
    }

    if ((gps->config.i2c_address == 0U) ||
        (gps->config.i2c_address > 0x7FU))
    {
        ESP_LOGE(TAG, "Invalid 7-bit I2C address: 0x%02X", gps->config.i2c_address);
        return ESP_ERR_INVALID_ARG;
    }

    if ((gps->config.i2c_clock_hz == 0U) || (gps->config.i2c_clock_hz > 400000U))
    {
        ESP_LOGE(TAG, "Invalid I2C clock: %" PRIu32 " Hz", gps->config.i2c_clock_hz);
        return ESP_ERR_INVALID_ARG;
    }

    gps->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = gps->config.i2c_address,
        .scl_speed_hz = gps->config.i2c_clock_hz,
    };

    ESP_LOGI(TAG, "Creating GPS I2C device at 0x%02X, clock=%" PRIu32 " Hz",
             gps->dev_cfg.device_address,
             gps->dev_cfg.scl_speed_hz);

    esp_err_t err = i2c_master_bus_add_device(
        gps->bus_handle,
        &gps->dev_cfg,
        &gps->i2c_dev);

    if (err != ESP_OK)
    {
        gps->i2c_dev = NULL;

        ESP_LOGE(TAG,
                 "Failed to create GPS I2C device at 0x%02X: %s",
                 gps->dev_cfg.device_address,
                 esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(TAG, "GPS I2C device handle created at 0x%02X", gps->dev_cfg.device_address);

    return ESP_OK;
}

static esp_err_t gps_read_register(gps_t *gps, uint8_t reg, uint8_t *dout, size_t size)
{
    if ((gps == NULL) || (dout == NULL) || (size == 0U))
        return ESP_ERR_INVALID_ARG;

    if (gps->i2c_dev == NULL)
        return ESP_ERR_INVALID_STATE;

    return i2c_master_transmit_receive(
        gps->i2c_dev,
        &reg,
        sizeof(reg),
        dout,
        size,
        (int)gps->config.io_timeout_ms);
}

static esp_err_t gps_get_available_bytes(gps_t *gps, uint16_t *available)
{
    if ((gps == NULL) || (available == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[2] = {0};

    esp_err_t err = gps_read_register(
        gps,
        GPS_I2C_REG_AVAILABLE_MSB,
        raw,
        sizeof(raw));

    if (err != ESP_OK)
    {
        *available = 0;
        return err;
    }

    /*
     * 0xFD: byte más significativo
     * 0xFE: byte menos significativo
     */
    *available = ((uint16_t)raw[0] << 8) | (uint16_t)raw[1];

    return ESP_OK;
}

static esp_err_t gps_read_stream(gps_t *gps, uint8_t *dout, size_t size)
{
    if ((gps == NULL) || (dout == NULL) || (size == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return gps_read_register(
        gps,
        GPS_I2C_REG_DATA_STREAM,
        dout,
        size);
}

static esp_err_t gps_write_stream(gps_t *gps, const uint8_t *din, size_t size)
{
    if ((gps == NULL) || (din == NULL) || (size < 2U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit(
        gps->i2c_dev,
        din,
        size,
        (int)gps->config.io_timeout_ms);
}

static esp_err_t gps_gpio_init(gps_t *gps)
{
    if (gps == NULL)
        return ESP_ERR_INVALID_ARG;

    uint64_t pin_mask = 0;

    if (gps->config.reset_pin != GPIO_NUM_NC)
    {
        pin_mask |= (1ULL << gps->config.reset_pin);
    }

    if (gps->config.safeboot_pin != GPIO_NUM_NC)
    {
        pin_mask |= (1ULL << gps->config.safeboot_pin);
    }

    if (pin_mask == 0)
    {
        return ESP_OK;
    }

    const gpio_config_t gpio_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&gpio_cfg);

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * SAFEBOOT_N activo en LOW.
     * HIGH significa arranque normal.
     */
    if (gps->config.safeboot_pin != GPIO_NUM_NC)
    {
        gpio_set_level(gps->config.safeboot_pin, 1);
    }

    /*
     * RESET_N también es activo en LOW.
     */
    if (gps->config.reset_pin != GPIO_NUM_NC)
    {
        gpio_set_level(gps->config.reset_pin, 1);
    }

    return ESP_OK;
}

static esp_err_t gps_hw_reset(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->config.safeboot_pin != GPIO_NUM_NC)
    {
        gpio_set_level(gps->config.safeboot_pin, 1);
    }

    if (gps->config.reset_pin == GPIO_NUM_NC)
    {
        /*
         * No hay control de reset. Solo esperamos que el módulo arranque.
         */
        vTaskDelay(pdMS_TO_TICKS(GPS_STARTUP_DELAY_MS));
        return ESP_OK;
    }

    gpio_set_level(gps->config.reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(GPS_RESET_PULSE_MS));

    gpio_set_level(gps->config.reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(GPS_STARTUP_DELAY_MS));

    return ESP_OK;
}

static void gps_ack_parser_reset(gps_ack_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = GPS_ACK_PARSE_SYNC_1;
}

static void gps_ack_checksum_add(
    gps_ack_parser_t *parser,
    uint8_t byte)
{
    parser->checksum_a =
        (uint8_t)(parser->checksum_a + byte);

    parser->checksum_b =
        (uint8_t)(parser->checksum_b + parser->checksum_a);
}




static void gps_parser_reset(gps_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = GPS_PARSE_WAIT_SYNC_1;
}

static void gps_parser_checksum_add(
    gps_parser_t *parser,
    uint8_t byte)
{
    parser->checksum_a =
        (uint8_t)(parser->checksum_a + byte);

    parser->checksum_b =
        (uint8_t)(parser->checksum_b + parser->checksum_a);
}


static gps_parse_result_t gps_parser_process_byte(
    gps_parser_t *parser,
    uint8_t byte)
{
    if (parser == NULL)
    {
        return GPS_PARSE_RESULT_NONE;
    }

    switch (parser->state)
    {
        case GPS_PARSE_WAIT_SYNC_1:
        {
            if (byte == GPS_UBX_SYNC_1)
            {
                parser->state = GPS_PARSE_WAIT_SYNC_2;
            }

            break;
        }

        case GPS_PARSE_WAIT_SYNC_2:
        {
            if (byte == GPS_UBX_SYNC_2)
            {
                parser->checksum_a = 0U;
                parser->checksum_b = 0U;

                parser->payload_length = 0U;
                parser->payload_index = 0U;
                parser->payload_overflow = false;

                parser->state = GPS_PARSE_CLASS;
            }
            else if (byte == GPS_UBX_SYNC_1)
            {
                /*
                 * Posible secuencia:
                 * B5 B5 62...
                 */
                parser->state = GPS_PARSE_WAIT_SYNC_2;
            }
            else
            {
                parser->state = GPS_PARSE_WAIT_SYNC_1;
            }

            break;
        }

        case GPS_PARSE_CLASS:
        {
            parser->message_class = byte;
            gps_parser_checksum_add(parser, byte);

            parser->state = GPS_PARSE_ID;
            break;
        }

        case GPS_PARSE_ID:
        {
            parser->message_id = byte;
            gps_parser_checksum_add(parser, byte);

            parser->state = GPS_PARSE_LENGTH_LSB;
            break;
        }

        case GPS_PARSE_LENGTH_LSB:
        {
            parser->payload_length = byte;
            gps_parser_checksum_add(parser, byte);

            parser->state = GPS_PARSE_LENGTH_MSB;
            break;
        }

        case GPS_PARSE_LENGTH_MSB:
        {
            parser->payload_length |=
                ((uint16_t)byte << 8);

            gps_parser_checksum_add(parser, byte);

            parser->payload_index = 0U;

            parser->payload_overflow =
                parser->payload_length >
                GPS_UBX_MAX_PAYLOAD_LENGTH;

            if (parser->payload_length == 0U)
            {
                parser->state = GPS_PARSE_CHECKSUM_A;
            }
            else
            {
                parser->state = GPS_PARSE_PAYLOAD;
            }

            break;
        }

        case GPS_PARSE_PAYLOAD:
        {
            /*
             * Aunque el payload sea demasiado grande,
             * seguimos consumiéndolo y calculando el checksum.
             */
            if (parser->payload_index <
                GPS_UBX_MAX_PAYLOAD_LENGTH)
            {
                parser->payload[parser->payload_index] = byte;
            }

            gps_parser_checksum_add(parser, byte);

            parser->payload_index++;

            if (parser->payload_index >=
                parser->payload_length)
            {
                parser->state = GPS_PARSE_CHECKSUM_A;
            }

            break;
        }

        case GPS_PARSE_CHECKSUM_A:
        {
            parser->received_checksum_a = byte;
            parser->state = GPS_PARSE_CHECKSUM_B;

            break;
        }

        case GPS_PARSE_CHECKSUM_B:
        {
            const bool checksum_valid =
                (parser->received_checksum_a ==
                 parser->checksum_a) &&
                (byte == parser->checksum_b);

            if (!checksum_valid)
            {
                return GPS_PARSE_RESULT_CHECKSUM_ERROR;
            }

            return GPS_PARSE_RESULT_FRAME_COMPLETE;
        }

        default:
        {
            gps_parser_reset(parser);
            break;
        }
    }

    return GPS_PARSE_RESULT_NONE;
}
static esp_err_t gps_flush_output(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t discard[GPS_FLUSH_CHUNK_SIZE];
    size_t total_discarded = 0U;

    const int64_t deadline_us =
        esp_timer_get_time() +
        ((int64_t)GPS_FLUSH_TIMEOUT_MS * 1000LL);

    while (esp_timer_get_time() < deadline_us)
    {
        uint16_t available = 0U;

        esp_err_t err = gps_get_available_bytes(
            gps,
            &available);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Failed to read available bytes during flush: %s",
                     esp_err_to_name(err));

            return err;
        }

        if (available == 0U)
        {
            ESP_LOGD(TAG,
                     "GPS output flushed, discarded %u bytes",
                     (unsigned)total_discarded);

            return ESP_OK;
        }

        size_t remaining = available;

        while (remaining > 0U)
        {
            size_t chunk_size = remaining;

            if (chunk_size > sizeof(discard))
            {
                chunk_size = sizeof(discard);
            }

            err = gps_read_stream(
                gps,
                discard,
                chunk_size);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG,
                         "Failed to discard GPS output: %s",
                         esp_err_to_name(err));

                return err;
            }

            total_discarded += chunk_size;
            remaining -= chunk_size;
        }
    }

    /*
     * No se considera necesariamente un fallo.
     * Puede haber mensajes periódicos entrando continuamente.
     */
    ESP_LOGW(TAG,
             "GPS flush reached time limit; discarded %u bytes",
             (unsigned)total_discarded);

    return ESP_OK;
}

static esp_err_t gps_wait_for_message(
    gps_t *gps,
    uint8_t expected_class,
    uint8_t expected_id,
    uint8_t *payload_out,
    size_t payload_capacity,
    uint16_t *payload_length_out,
    uint32_t timeout_ms)
{
    if ((gps == NULL) ||
        (payload_length_out == NULL) ||
        (timeout_ms == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((payload_capacity > 0U) &&
        (payload_out == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *payload_length_out = 0U;

    gps_parser_t parser;
    gps_parser_reset(&parser);

    uint8_t rx_buffer[64];

    const int64_t deadline_us =
        esp_timer_get_time() +
        ((int64_t)timeout_ms * 1000LL);

    while (esp_timer_get_time() < deadline_us)
    {
        uint16_t available = 0U;

        esp_err_t err = gps_get_available_bytes(
            gps,
            &available);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Failed to read available GPS bytes: %s",
                     esp_err_to_name(err));

            return err;
        }

        if (available == 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        size_t remaining = available;

        while (remaining > 0U)
        {
            size_t chunk_size = remaining;

            if (chunk_size > sizeof(rx_buffer))
            {
                chunk_size = sizeof(rx_buffer);
            }

            err = gps_read_stream(
                gps,
                rx_buffer,
                chunk_size);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG,
                         "Failed to read GPS stream: %s",
                         esp_err_to_name(err));

                return err;
            }

            remaining -= chunk_size;

            for (size_t i = 0U; i < chunk_size; i++)
            {
                gps_parse_result_t result =
                    gps_parser_process_byte(
                        &parser,
                        rx_buffer[i]);

                if (result ==
                    GPS_PARSE_RESULT_CHECKSUM_ERROR)
                {
                    ESP_LOGW(TAG,
                             "Discarding UBX frame with invalid checksum");

                    gps_parser_reset(&parser);
                    continue;
                }

                if (result !=
                    GPS_PARSE_RESULT_FRAME_COMPLETE)
                {
                    continue;
                }

                const bool expected_message =
                    (parser.message_class ==
                     expected_class) &&
                    (parser.message_id ==
                     expected_id);

                if (!expected_message)
                {
                    ESP_LOGD(TAG,
                             "Ignoring UBX message %02X %02X",
                             parser.message_class,
                             parser.message_id);

                    gps_parser_reset(&parser);
                    continue;
                }

                if (parser.payload_overflow ||
                    parser.payload_length >
                    payload_capacity)
                {
                    ESP_LOGE(
                        TAG,
                        "Payload for UBX %02X %02X is too large: "
                        "%u bytes, capacity=%u",
                        expected_class,
                        expected_id,
                        parser.payload_length,
                        (unsigned)payload_capacity);

                    return ESP_ERR_INVALID_SIZE;
                }

                if (parser.payload_length > 0U)
                {
                    memcpy(
                        payload_out,
                        parser.payload,
                        parser.payload_length);
                }

                *payload_length_out =
                    parser.payload_length;

                ESP_LOGD(TAG,
                         "Received UBX %02X %02X, payload=%u bytes",
                         expected_class,
                         expected_id,
                         parser.payload_length);

                return ESP_OK;
            }
        }
    }

    ESP_LOGE(TAG,
             "Timeout waiting for UBX %02X %02X",
             expected_class,
             expected_id);

    return ESP_ERR_TIMEOUT;
}



static esp_err_t gps_wait_for_ack(
    gps_t *gps,
    uint8_t expected_class,
    uint8_t expected_id,
    uint32_t timeout_ms)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (timeout_ms == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gps_ack_parser_t parser;
    gps_ack_parser_reset(&parser);

    uint8_t rx_buffer[GPS_ACK_READ_CHUNK_SIZE];

    const int64_t deadline_us =
        esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);

    while (esp_timer_get_time() < deadline_us)
    {
        uint16_t available = 0;

        esp_err_t err = gps_get_available_bytes(
            gps,
            &available);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Could not read GPS available bytes: %s",
                     esp_err_to_name(err));

            return err;
        }

        if (available == 0U)
        {
            /*
             * No ocupamos constantemente el procesador mientras
             * esperamos que el GPS coloque la respuesta en el buffer.
             */
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        size_t remaining = available;

        while (remaining > 0U)
        {
            size_t chunk_size = remaining;

            if (chunk_size > sizeof(rx_buffer))
            {
                chunk_size = sizeof(rx_buffer);
            }

            err = gps_read_stream(
                gps,
                rx_buffer,
                chunk_size);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG,
                         "Could not read GPS stream: %s",
                         esp_err_to_name(err));

                return err;
            }

            remaining -= chunk_size;

            for (size_t i = 0; i < chunk_size; i++)
            {
                const uint8_t byte = rx_buffer[i];

                switch (parser.state)
                {
                case GPS_ACK_PARSE_SYNC_1:
                {
                    if (byte == GPS_UBX_SYNC_1)
                    {
                        parser.state = GPS_ACK_PARSE_SYNC_2;
                    }

                    break;
                }

                case GPS_ACK_PARSE_SYNC_2:
                {
                    if (byte == GPS_UBX_SYNC_2)
                    {
                        parser.checksum_a = 0U;
                        parser.checksum_b = 0U;
                        parser.payload_index = 0U;
                        parser.payload_length = 0U;

                        parser.state = GPS_ACK_PARSE_CLASS;
                    }
                    else if (byte == GPS_UBX_SYNC_1)
                    {
                        /*
                         * Puede ser el inicio de una nueva secuencia:
                         * B5 B5 62...
                         */
                        parser.state = GPS_ACK_PARSE_SYNC_2;
                    }
                    else
                    {
                        parser.state = GPS_ACK_PARSE_SYNC_1;
                    }

                    break;
                }

                case GPS_ACK_PARSE_CLASS:
                {
                    parser.message_class = byte;
                    gps_ack_checksum_add(&parser, byte);

                    parser.state = GPS_ACK_PARSE_ID;
                    break;
                }

                case GPS_ACK_PARSE_ID:
                {
                    parser.message_id = byte;
                    gps_ack_checksum_add(&parser, byte);

                    parser.state = GPS_ACK_PARSE_LENGTH_LSB;
                    break;
                }

                case GPS_ACK_PARSE_LENGTH_LSB:
                {
                    parser.payload_length = byte;
                    gps_ack_checksum_add(&parser, byte);

                    parser.state = GPS_ACK_PARSE_LENGTH_MSB;
                    break;
                }

                case GPS_ACK_PARSE_LENGTH_MSB:
                {
                    parser.payload_length |=
                        ((uint16_t)byte << 8);

                    gps_ack_checksum_add(&parser, byte);

                    /*
                     * Protección frente a una sincronización falsa
                     * o una longitud corrupta.
                     */
                    if (parser.payload_length >
                        GPS_UBX_MAX_ACCEPTED_PAYLOAD)
                    {
                        gps_ack_parser_reset(&parser);
                        break;
                    }

                    parser.payload_index = 0U;

                    parser.state =
                        (parser.payload_length == 0U)
                            ? GPS_ACK_PARSE_CK_A
                            : GPS_ACK_PARSE_PAYLOAD;

                    break;
                }

                case GPS_ACK_PARSE_PAYLOAD:
                {
                    /*
                     * Solo almacenamos los primeros dos bytes.
                     * El checksum sí se calcula para todo el payload.
                     */
                    if (parser.payload_index <
                        sizeof(parser.payload))
                    {
                        parser.payload[parser.payload_index] = byte;
                    }

                    gps_ack_checksum_add(&parser, byte);

                    parser.payload_index++;

                    if (parser.payload_index >=
                        parser.payload_length)
                    {
                        parser.state = GPS_ACK_PARSE_CK_A;
                    }

                    break;
                }

                case GPS_ACK_PARSE_CK_A:
                {
                    parser.received_checksum_a = byte;
                    parser.state = GPS_ACK_PARSE_CK_B;
                    break;
                }

                case GPS_ACK_PARSE_CK_B:
                {
                    const bool checksum_valid =
                        (parser.received_checksum_a ==
                         parser.checksum_a) &&
                        (byte == parser.checksum_b);

                    if (checksum_valid)
                    {
                        const bool is_ack_message =
                            (parser.message_class ==
                             GPS_UBX_CLASS_ACK) &&
                            (parser.payload_length == 2U);

                        const bool is_expected_command =
                            (parser.payload[0] ==
                             expected_class) &&
                            (parser.payload[1] ==
                             expected_id);

                        if (is_ack_message &&
                            is_expected_command)
                        {
                            if (parser.message_id ==
                                GPS_UBX_ID_ACK_ACK)
                            {
                                ESP_LOGD(
                                    TAG,
                                    "ACK received for UBX %02X %02X",
                                    expected_class,
                                    expected_id);

                                return ESP_OK;
                            }

                            if (parser.message_id ==
                                GPS_UBX_ID_ACK_NAK)
                            {
                                ESP_LOGE(
                                    TAG,
                                    "NAK received for UBX %02X %02X",
                                    expected_class,
                                    expected_id);

                                return ESP_ERR_INVALID_RESPONSE;
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG,
                                 "Discarding UBX message with "
                                 "invalid checksum");
                    }

                    /*
                     * Era otro mensaje, un ACK para otro comando,
                     * o una trama corrupta. Seguimos buscando.
                     */
                    gps_ack_parser_reset(&parser);
                    break;
                }

                default:
                {
                    gps_ack_parser_reset(&parser);
                    break;
                }
                }
            }
        }
    }

    ESP_LOGE(TAG,
             "Timeout waiting ACK for UBX %02X %02X",
             expected_class,
             expected_id);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t gps_probe(gps_t *gps)
{
    if ((gps == NULL) || (gps->bus_handle == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const int attempts = 5;

    for (int i = 0; i < attempts; i++)
    {
        esp_err_t err = i2c_master_probe(
            gps->bus_handle,
            gps->config.i2c_address,
            gps->config.io_timeout_ms);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "GPS detected at I2C address 0x%02X", gps->config.i2c_address);

            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGE(TAG, "GPS did not respond at address 0x%02X", gps->config.i2c_address);

    return ESP_ERR_NOT_FOUND;
}

static void gps_ubx_checksum(
    const uint8_t *data,
    size_t length,
    uint8_t *ck_a,
    uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;

    for (size_t i = 0; i < length; i++)
    {
        *ck_a = (uint8_t)(*ck_a + data[i]);
        *ck_b = (uint8_t)(*ck_b + *ck_a);
    }
}

static esp_err_t gps_send_ubx(
    gps_t *gps,
    uint8_t message_class,
    uint8_t message_id,
    const uint8_t *payload,
    uint16_t payload_length)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if ((payload_length > 0U) && (payload == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (payload_length > GPS_UBX_MAX_PAYLOAD_LENGTH)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame[GPS_UBX_MAX_PAYLOAD_LENGTH + 8U];

    frame[0] = GPS_UBX_SYNC_1;
    frame[1] = GPS_UBX_SYNC_2;
    frame[2] = message_class;
    frame[3] = message_id;
    frame[4] = (uint8_t)(payload_length & 0xFFU);
    frame[5] = (uint8_t)((payload_length >> 8) & 0xFFU);

    if (payload_length > 0U)
    {
        memcpy(&frame[6], payload, payload_length);
    }

    uint8_t ck_a;
    uint8_t ck_b;

    /*
     * Checksum desde CLASS hasta el último byte del payload.
     */
    gps_ubx_checksum(
        &frame[2],
        4U + payload_length,
        &ck_a,
        &ck_b);

    frame[6U + payload_length] = ck_a;
    frame[7U + payload_length] = ck_b;

    return gps_write_stream(
        gps,
        frame,
        payload_length + 8U);
}

static void gps_write_u32_le(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)(value);
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);
}

static esp_err_t gps_enable_ubx_output(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[14];
    size_t index = 0;

    /* UBX-CFG-VALSET header */
    payload[index++] = 0x00;              // version
    payload[index++] = GPS_CFG_LAYER_RAM; // layers
    payload[index++] = 0x00;              // transactionless
    payload[index++] = 0x00;              // reserved

    /*
     * CFG-I2CINPROT-UBX = true
     */
    gps_write_u32_le(
        &payload[index],
        GPS_CFG_I2C_IN_UBX);

    index += 4;
    payload[index++] = 1;

    /*
     * CFG-I2COUTPROT-UBX = true
     */
    gps_write_u32_le(
        &payload[index],
        GPS_CFG_I2C_OUT_UBX);

    index += 4;
    payload[index++] = 1;

    esp_err_t err = gps_send_ubx(
        gps,
        GPS_UBX_CLASS_CFG,
        GPS_UBX_ID_CFG_VALSET,
        payload,
        (uint16_t)index);

    if (err != ESP_OK)
    {
        return err;
    }

    return gps_wait_for_ack(
        gps,
        GPS_UBX_CLASS_CFG,
        GPS_UBX_ID_CFG_VALSET,
        GPS_COMMAND_TIMEOUT_MS);
}

static esp_err_t gps_read_version(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = gps_send_ubx(
        gps,
        GPS_UBX_CLASS_MON,
        GPS_UBX_ID_MON_VER,
        NULL,
        0);

    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t payload[GPS_UBX_MAX_PAYLOAD_LENGTH];
    uint16_t payload_length = 0;

    err = gps_wait_for_message(
        gps,
        GPS_UBX_CLASS_MON,
        GPS_UBX_ID_MON_VER,
        payload,
        sizeof(payload),
        &payload_length,
        GPS_COMMAND_TIMEOUT_MS);

    if (err != ESP_OK)
    {
        return err;
    }

    if (payload_length < 40U)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(
        gps->info.software_version,
        &payload[0],
        GPS_SW_VERSION_LENGTH);

    gps->info.software_version[GPS_SW_VERSION_LENGTH] = '\0';

    memcpy(
        gps->info.hardware_version,
        &payload[30],
        GPS_HW_VERSION_LENGTH);

    gps->info.hardware_version[GPS_HW_VERSION_LENGTH] = '\0';

    gps->info.version_valid = true;

    ESP_LOGI(TAG,
             "GPS software: %s",
             gps->info.software_version);

    ESP_LOGI(TAG,
             "GPS hardware: %s",
             gps->info.hardware_version);

    return ESP_OK;
}


static esp_err_t gps_read_unique_id(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Poll de UBX-SEC-UNIQID:
     * class = 0x27
     * id    = 0x03
     * payload vacío
     */
    esp_err_t err = gps_send_ubx(
        gps,
        GPS_UBX_CLASS_SEC,
        GPS_UBX_ID_SEC_UNIQID,
        NULL,
        0);

    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t payload[16];
    uint16_t payload_length = 0;

    err = gps_wait_for_message(
        gps,
        GPS_UBX_CLASS_SEC,
        GPS_UBX_ID_SEC_UNIQID,
        payload,
        sizeof(payload),
        &payload_length,
        GPS_COMMAND_TIMEOUT_MS);

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * Payload:
     * offset 0: version
     * offset 1-3: reserved
     * offset 4-8: uniqueId
     */
    if ((payload_length != 9U) || (payload[0] != 0x01U))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(
        gps->info.unique_id,
        &payload[4],
        GPS_UNIQUE_ID_LENGTH);

    gps->info.unique_id_valid = true;

    ESP_LOGI(TAG,
             "GPS unique ID: %02X%02X%02X%02X%02X",
             gps->info.unique_id[0],
             gps->info.unique_id[1],
             gps->info.unique_id[2],
             gps->info.unique_id[3],
             gps->info.unique_id[4]);

    return ESP_OK;
}

static esp_err_t gps_apply_configuration(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[64];
    size_t index = 0;

    payload[index++] = 0x00; // version
    payload[index++] = gps->config.configuration_layers;
    payload[index++] = 0x00; // transactionless
    payload[index++] = 0x00; // reserved

    /* I2C input UBX = true */
    gps_write_u32_le(&payload[index], GPS_CFG_I2C_IN_UBX);
    index += 4;
    payload[index++] = 1;

    /* I2C output UBX = true */
    gps_write_u32_le(&payload[index], GPS_CFG_I2C_OUT_UBX);
    index += 4;
    payload[index++] = 1;

    /* I2C output NMEA */
    gps_write_u32_le(&payload[index], GPS_CFG_I2C_OUT_NMEA);
    index += 4;
    payload[index++] = gps->config.disable_nmea_output ? 0 : 1;

    /* CFG-RATE-MEAS, U2 */
    gps_write_u32_le(&payload[index], GPS_CFG_RATE_MEAS);
    index += 4;

    payload[index++] =
        (uint8_t)(gps->config.measurement_period_ms & 0xFFU);

    payload[index++] =
        (uint8_t)(gps->config.measurement_period_ms >> 8);

    /* CFG-RATE-NAV, U2 */
    gps_write_u32_le(&payload[index], GPS_CFG_RATE_NAV);
    index += 4;

    payload[index++] =
        (uint8_t)(gps->config.navigation_ratio & 0xFFU);

    payload[index++] =
        (uint8_t)(gps->config.navigation_ratio >> 8);

    /* NAV-PVT output rate on I2C, U1 */
    gps_write_u32_le(
        &payload[index],
        GPS_CFG_MSGOUT_NAV_PVT_I2C);

    index += 4;
    payload[index++] = gps->config.nav_pvt_output_rate;

    if (gps->config.configure_dynamic_model)
    {
        gps_write_u32_le(
            &payload[index],
            GPS_CFG_NAVSPG_DYNMODEL);

        index += 4;
        payload[index++] = (uint8_t)gps->config.dynamic_model;
    }

    esp_err_t err = gps_send_ubx(
        gps,
        GPS_UBX_CLASS_CFG,
        GPS_UBX_ID_CFG_VALSET,
        payload,
        (uint16_t)index);

    if (err != ESP_OK)
    {
        return err;
    }

    return gps_wait_for_ack(
        gps,
        GPS_UBX_CLASS_CFG,
        GPS_UBX_ID_CFG_VALSET,
        GPS_COMMAND_TIMEOUT_MS);
}




static uint16_t gps_read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] |
           ((uint16_t)p[1] << 8);
}

static uint32_t gps_read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int32_t gps_read_i32_le(const uint8_t *p)
{
    return (int32_t)gps_read_u32_le(p);
}



esp_err_t gps_init(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    if ((gps->config.i2c_address == 0U) ||
        (gps->config.i2c_address > 0x7FU))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->config.i2c_clock_hz > GPS_I2C_MAX_CLOCK_HZ)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->config.measurement_period_ms == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->config.navigation_ratio == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->config.configuration_layers == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gps->i2c_dev != NULL)
    {
        esp_err_t remove_err = i2c_master_bus_rm_device(gps->i2c_dev);

        if (remove_err != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Failed to remove previous GPS I2C device: %s",
                     esp_err_to_name(remove_err));

            return remove_err;
        }

        gps->i2c_dev = NULL;
    }

    gps->initialized = false;

    memset(&gps->data, 0, sizeof(gps->data));
    memset(&gps->info, 0, sizeof(gps->info));
    memset(&gps->parser, 0, sizeof(gps->parser));

    esp_err_t err;

    err = gps_gpio_init(gps);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO initialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = gps_hw_reset(gps);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPS reset failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = gps_probe(gps);

    if (err != ESP_OK)
    {
        return err;
    }

    err = gps_device_create(gps);

    if (err != ESP_OK)
    {
        return err;
    }

    /*
     * Puede haber mensajes NMEA del arranque esperando en el buffer.
     * Se descartan antes de iniciar la comunicación UBX.
     */
    gps_flush_output(gps);

    /*
     * Habilitación provisional de respuestas UBX.
     */
    err = gps_enable_ubx_output(gps);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Could not enable UBX output: %s",
                 esp_err_to_name(err));

        goto fail;
    }

    /*
     * MON-VER sí se considera obligatorio:
     * comprueba que realmente hablamos con un receptor u-blox.
     */
    err = gps_read_version(gps);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Could not read GPS version: %s",
                 esp_err_to_name(err));

        goto fail;
    }

    /*
     * El identificador único es útil, pero no es imprescindible
     * para obtener posición.
     */
    err = gps_read_unique_id(gps);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "Unique ID could not be read: %s",
                 esp_err_to_name(err));

        gps->info.unique_id_valid = false;
    }

    err = gps_apply_configuration(gps);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "GPS configuration failed: %s",
                 esp_err_to_name(err));

        goto fail;
    }

    gps->initialized = true;

    ESP_LOGI(TAG,
             "NEO-M9N initialized: period=%u ms, navRatio=%u, PVT rate=%u",
             gps->config.measurement_period_ms,
             gps->config.navigation_ratio,
             gps->config.nav_pvt_output_rate);

    return ESP_OK;

fail:

    if (gps->i2c_dev != NULL)
    {
        i2c_master_bus_rm_device(gps->i2c_dev);
        gps->i2c_dev = NULL;
    }

    gps->initialized = false;

    return err;
}

static esp_err_t gps_parse_nav_pvt(
    gps_t *gps,
    const uint8_t *payload,
    uint16_t payload_length)
{
    if ((gps == NULL) || (payload == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload_length != GPS_NAV_PVT_PAYLOAD_LENGTH)
    {
        ESP_LOGE(TAG,
                 "Invalid NAV-PVT payload length: %u",
                 (unsigned)payload_length);

        return ESP_ERR_INVALID_SIZE;
    }

    /*
     * Decodificamos primero en una estructura temporal.
     * Así evitamos dejar gps->data parcialmente actualizada
     * si ocurre algún problema durante el procesamiento.
     */
    gps_data_t next = {0};

    /* GPS time of week */
    next.i_tow_ms = gps_read_u32_le(&payload[0]);

    /* Fecha y hora UTC */
    next.year   = gps_read_u16_le(&payload[4]);
    next.month  = payload[6];
    next.day    = payload[7];
    next.hour   = payload[8];
    next.minute = payload[9];
    next.second = payload[10];

    const uint8_t valid = payload[11];

    next.valid_date =
        (valid & GPS_NAV_PVT_VALID_DATE_MASK) != 0U;

    next.valid_time =
        (valid & GPS_NAV_PVT_VALID_TIME_MASK) != 0U;

    next.fully_resolved =
        (valid & GPS_NAV_PVT_FULLY_RESOLVED_MASK) != 0U;

    next.nano = gps_read_i32_le(&payload[16]);

    /* Estado del fix */
    next.fix_type = (gps_fix_type_t)payload[20];

    next.flags  = payload[21];
    next.flags2 = payload[22];

    next.satellites_used = payload[23];

    next.gnss_fix_ok =
        (next.flags & GPS_NAV_PVT_GNSS_FIX_OK_MASK) != 0U;

    next.differential_solution =
        (next.flags & GPS_NAV_PVT_DIFF_SOLN_MASK) != 0U;

    /*
     * Posición.
     *
     * lon/lat: 1e-7 grados
     * height/hMSL: milímetros
     */
    next.longitude_deg =
        (double)gps_read_i32_le(&payload[24]) * 1e-7;

    next.latitude_deg =
        (double)gps_read_i32_le(&payload[28]) * 1e-7;

    next.height_ellipsoid_m =
        (float)gps_read_i32_le(&payload[32]) * 1e-3f;

    next.altitude_msl_m =
        (float)gps_read_i32_le(&payload[36]) * 1e-3f;

    /* Estimaciones de precisión en milímetros */
    next.horizontal_accuracy_m =
        (float)gps_read_u32_le(&payload[40]) * 1e-3f;

    next.vertical_accuracy_m =
        (float)gps_read_u32_le(&payload[44]) * 1e-3f;

    /*
     * Velocidades NED y ground speed.
     * Valores originales en mm/s.
     */
    next.velocity_north_mps =
        (float)gps_read_i32_le(&payload[48]) * 1e-3f;

    next.velocity_east_mps =
        (float)gps_read_i32_le(&payload[52]) * 1e-3f;

    next.velocity_down_mps =
        (float)gps_read_i32_le(&payload[56]) * 1e-3f;

    next.ground_speed_mps =
        (float)gps_read_i32_le(&payload[60]) * 1e-3f;

    /*
     * Heading:
     * escala 1e-5 grados.
     */
    next.heading_motion_deg =
        (float)gps_read_i32_le(&payload[64]) * 1e-5f;

    next.speed_accuracy_mps =
        (float)gps_read_u32_le(&payload[68]) * 1e-3f;

    next.heading_accuracy_deg =
        (float)gps_read_u32_le(&payload[72]) * 1e-5f;

    /*
     * pDOP tiene escala 0.01.
     */
    next.position_dop =
        (float)gps_read_u16_le(&payload[76]) * 0.01f;

    next.flags3 = gps_read_u16_le(&payload[78]);

    const bool invalid_llh =
        (next.flags3 & GPS_NAV_PVT_INVALID_LLH_MASK) != 0U;

    const bool position_fix_type =
        (next.fix_type == GPS_FIX_2D) ||
        (next.fix_type == GPS_FIX_3D) ||
        (next.fix_type == GPS_FIX_GNSS_DR_COMBINED);

    /*
     * Un fixType 2D o 3D no garantiza por sí solo que
     * la solución sea válida.
     */
    next.position_valid =
        next.gnss_fix_ok &&
        !invalid_llh &&
        position_fix_type;

    next.new_data = true;

    next.update_counter =
        gps->data.update_counter + 1U;

    next.last_update_us =
        esp_timer_get_time();

    /*
     * Se publica la estructura completa al final.
     */
    gps->data = next;

    return ESP_OK;
}



static esp_err_t gps_handle_ubx_message(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gps_parser_t *parser = &gps->parser;

    if ((parser->message_class == GPS_UBX_CLASS_NAV) &&
        (parser->message_id == GPS_UBX_ID_NAV_PVT))
    {
        return gps_parse_nav_pvt(
            gps,
            parser->payload,
            parser->payload_length);
    }

    ESP_LOGD(TAG,
             "Unhandled UBX message: class=0x%02X id=0x%02X",
             parser->message_class,
             parser->message_id);

    return ESP_OK;
}




esp_err_t gps_update(gps_t *gps, bool *new_data)
{
    if ((gps == NULL) || (new_data == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!gps->initialized || (gps->i2c_dev == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    *new_data = false;
    gps->data.new_data = false;

    uint16_t available = 0U;

    esp_err_t err =
        gps_get_available_bytes(gps, &available);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Failed to read available GPS bytes: %s",
                 esp_err_to_name(err));

        return err;
    }

    if (available == 0U)
    {
        return ESP_OK;
    }

    uint8_t rx_buffer[GPS_RX_CHUNK_SIZE];

    size_t chunk_limit = gps->config.read_chunk_size;

    if ((chunk_limit == 0U) ||
        (chunk_limit > sizeof(rx_buffer)))
    {
        chunk_limit = sizeof(rx_buffer);
    }

    size_t remaining = available;

    while (remaining > 0U)
    {
        size_t chunk_size = remaining;

        if (chunk_size > chunk_limit)
        {
            chunk_size = chunk_limit;
        }

        err = gps_read_stream(
            gps,
            rx_buffer,
            chunk_size);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Failed to read GPS stream: %s",
                     esp_err_to_name(err));

            return err;
        }

        remaining -= chunk_size;

        for (size_t i = 0U; i < chunk_size; i++)
        {
            const gps_parse_result_t result =
                gps_parser_process_byte(
                    &gps->parser,
                    rx_buffer[i]);

            if (result == GPS_PARSE_RESULT_NONE)
            {
                continue;
            }

            if (result ==
                GPS_PARSE_RESULT_CHECKSUM_ERROR)
            {
                ESP_LOGW(TAG,
                         "Discarding UBX frame with invalid checksum");

                gps_parser_reset(&gps->parser);
                continue;
            }

            if (result ==
                GPS_PARSE_RESULT_FRAME_COMPLETE)
            {
                const bool is_nav_pvt =
                    (gps->parser.message_class ==
                     GPS_UBX_CLASS_NAV) &&
                    (gps->parser.message_id ==
                     GPS_UBX_ID_NAV_PVT);

                err = gps_handle_ubx_message(gps);

                /*
                 * Siempre se reinicia el parser después de
                 * completar una trama.
                 */
                gps_parser_reset(&gps->parser);

                if (err != ESP_OK)
                {
                    return err;
                }

                if (is_nav_pvt)
                {
                    *new_data = true;
                }
            }
        }
    }

    return ESP_OK;
}

esp_err_t gps_get_data(
    const gps_t *gps,
    gps_data_t *data)
{
    if ((gps == NULL) || (data == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!gps->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (gps->data.update_counter == 0U)
    {
        /*
         * Aún no se ha recibido ningún NAV-PVT.
         */
        return ESP_ERR_NOT_FOUND;
    }

    *data = gps->data;

    return ESP_OK;
}

bool gps_has_valid_fix(const gps_t *gps)
{
    if ((gps == NULL) || !gps->initialized)
    {
        return false;
    }

    if (gps->data.update_counter == 0U)
    {
        return false;
    }

    return gps->data.position_valid;
}

bool gps_data_is_fresh(
    const gps_t *gps,
    uint32_t max_age_ms)
{
    if ((gps == NULL) || !gps->initialized)
    {
        return false;
    }

    if ((gps->data.update_counter == 0U) ||
        (gps->data.last_update_us <= 0))
    {
        return false;
    }

    const int64_t now_us =
        esp_timer_get_time();

    const int64_t age_us =
        now_us - gps->data.last_update_us;

    /*
     * No debería ocurrir, pero protege frente a
     * una referencia temporal inconsistente.
     */
    if (age_us < 0)
    {
        return false;
    }

    return age_us <=
           ((int64_t)max_age_ms * 1000LL);
}

esp_err_t gps_get_device_info(
    const gps_t *gps,
    gps_device_info_t *info)
{
    if ((gps == NULL) || (info == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!gps->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    *info = gps->info;

    return ESP_OK;
}


esp_err_t gps_deinit(gps_t *gps)
{
    if (gps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = ESP_OK;

    if (gps->i2c_dev != NULL)
    {
        result =
            i2c_master_bus_rm_device(gps->i2c_dev);

        if (result != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "Failed to remove GPS I2C device: %s",
                     esp_err_to_name(result));
        }

        gps->i2c_dev = NULL;
    }

    gps->initialized = false;

    memset(&gps->data, 0, sizeof(gps->data));
    memset(&gps->info, 0, sizeof(gps->info));
    gps_parser_reset(&gps->parser);

    /*
     * No eliminamos gps->bus_handle porque normalmente
     * puede estar compartido con otros dispositivos.
     */
    return result;
}
