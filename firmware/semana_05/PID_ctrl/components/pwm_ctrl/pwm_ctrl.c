#include <stdio.h>
#include "pwm_ctrl.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
static const char *TAG = "pwm: ";
void motor_init(void)
{
    //  Configuracion del timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 10 * 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    // Configuracion del canal 0
    ledc_channel_config_t ledc_ch0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIN1,
        .duty = 0,
        .hpoint = 0,
    };

    // Configuracion del canal 1
    ledc_channel_config_t ledc_ch1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIN2,
        .duty = 0,
        .hpoint = 0,
    };


    // Aplicar configuraciones
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch0));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch1));
    /*
    gpio_config_t io_Configin = {
        .pin_bit_mask = (1ULL << GPIN1) | (1ULL << GPIN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_Configin);*/
}

// Velocidad entre 0 y 100
void setSpeed(uint8_t speed)
{
    if (speed > 100)
    {
        speed = 100;
    }

    uint32_t pwm = (speed * 1023) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}
// 1 para derecha, 0 para izquierda
void setDirection(uint8_t direction)
{
    if (direction == 1)
    {
        gpio_set_level(GPIN1, 0);
        gpio_set_level(GPIN2, 1);
    }
    else if (direction == 0)
    {
        gpio_set_level(GPIN1, 1);
        gpio_set_level(GPIN2, 0);
    }
}


void motor_set(int8_t speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    uint32_t duty = (abs(speed) * 1023) / 100;

    if (speed > 0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
    else if (speed < 0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}


void brake(void)
{
    ESP_LOGI(TAG,"brake motor !!! ");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}
