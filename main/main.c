#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define TOGGLE_GPIO GPIO_NUM_2 // Designe GPIO you want ot toggle
#define LED_GPIO GPIO_NUM_0 // Designe GPIO you wan tot toggle
#define BUF_SIZE (1024)
#define UART_NUM UART_NUM_0

void setup_uart();
void setup_GPIO();

void toggle_GPIO();

void app_main(void) {

    setup_GPIO();
    setup_uart();

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    // printf("Press\n");
    while (1) {
        // Read data from UART
        uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

        if (data[0] == 'l') {
            // printf("Target Detected\n");
            toggle_GPIO();
            printf("r\n");
            data[0] = 0;
        } else if (data[0] != 0) {
            // printf("recieved: %c\n", data[0]);
            data[0] = 0;
        }
    }
}

void toggle_GPIO() {
    gpio_set_level(TOGGLE_GPIO, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gpio_set_level(TOGGLE_GPIO, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
}

void setup_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

void setup_GPIO() {
    gpio_reset_pin(TOGGLE_GPIO);
    gpio_set_direction(TOGGLE_GPIO, GPIO_MODE_OUTPUT);
}
