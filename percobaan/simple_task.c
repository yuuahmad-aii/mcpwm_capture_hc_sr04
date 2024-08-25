#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO 2 // GPIO untuk LED

// Task untuk menyalakan dan mematikan LED
void led_task(void *pvParameter)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(LED_GPIO, 1);    // Nyalakan LED
        vTaskDelay(pdMS_TO_TICKS(500)); // Tunggu 500 ms
        gpio_set_level(LED_GPIO, 0);    // Matikan LED
        vTaskDelay(pdMS_TO_TICKS(500)); // Tunggu 500 ms
    }
}

// Task untuk menampilkan nilai incremental pada serial monitor
void print_task(void *pvParameter)
{
    int count = 0;

    while (1)
    {
        ESP_LOGI("print_task", "Count: %d", count);
        count++;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Tunggu 1 detik
    }
}

void app_main(void)
{
    // Buat task LED
    xTaskCreate(&led_task, "led_task", 2048, NULL, 5, NULL);

    // Buat task print
    xTaskCreate(&print_task, "print_task", 2048, NULL, 5, NULL);
}
