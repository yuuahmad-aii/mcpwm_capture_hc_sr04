/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"

const static char *TAG = "example";

// verbose
// #define verbose 1

#define GPIO_STEP_PIN 18   // Pin untuk sinyal STEP
#define GPIO_DIR_PIN 19    // Pin untuk sinyal DIR
#define GPIO_ENABLE_PIN 21 // Pin untuk sinyal ENABLE

#define HC_SR04_TRIG_GPIO 15
#define HC_SR04_ECHO_GPIO 2

#define STEPS_PER_REVOLUTION 1600 // 1600 untuk stepper 1.8 derajat per langkah
#define STEP_SCANNING 2
int i = 0;
bool dir = false;

static void configure_gpio()
{
    gpio_set_direction(GPIO_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_ENABLE_PIN, GPIO_MODE_INPUT_OUTPUT);
}

static void gerakkan_motor(int steps)
{
    gpio_set_level(GPIO_ENABLE_PIN, 0); // Aktifkan motor (ENABLE pin)
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(GPIO_STEP_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
        gpio_set_level(GPIO_STEP_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    // calculate the interval in the ISR,
    // so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS)
    {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    }
    else
    {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

/**
 * @brief generate single pulse on Trig pin to start a new sample
 */
static void gen_trig_output(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0); // set low
}

void app_main(void)
{
    configure_gpio();

    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks;
    while (1)
    {
        // trigger the sensor to start a new sample
        gen_trig_output();
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            float derajat = i * 0.225;
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000)
            {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float)pulse_width_us / 58;
            printf("%.2f, %.2f, %d \n", distance, derajat, dir);
            // jika sudah, gerakkan stepper dan tambah nilai derajat
            gerakkan_motor(STEP_SCANNING);
            i += STEP_SCANNING;
            if (derajat > 180)
            {
                i = 0;      // reset nilai sweep
                dir = !dir; // balikkan pergerakan stepper
                gpio_set_level(GPIO_DIR_PIN, dir);
            }
        }
    }
}