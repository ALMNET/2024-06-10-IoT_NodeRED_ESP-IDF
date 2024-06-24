
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"


#define BLINK_GPIO 2

#define UART_NUM UART_NUM_1
#define INITIAL_BAUD_RATE 115200
#define NEW_BAUD_RATE 9600
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define BUF_SIZE 1024

static const char *TAG = "Debug";

// received data byte from uart
char receivedData;



static uint8_t s_led_state = 0;

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// PROTOTYPES //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

static void blink_led(void);


void init_uart();
char uart_read_char();
void change_baud_rate(uint32_t baud_rate);

//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// MAIN TASK //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{
    // Configure keepalive Led as output
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Initialize UART
    init_uart();

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to ensure initialization is complete
    change_baud_rate(NEW_BAUD_RATE);

    while (1) {

        // received data byte from uart
        char receivedData;

        if((receivedData = uart_read_char()) != '\0')
        {
            // printf("")
            ESP_LOGI(TAG, "Temperature: %d ºC", receivedData);
        }


        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        // blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// AUXILIARY FUNCTIONS /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = INITIAL_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}



static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

char uart_read_char() {
    uint8_t data;
    int length = uart_read_bytes(UART_NUM, &data, 1, portMAX_DELAY);
    if (length > 0) {
        return data;
    } else {
        return '\0'; // Return null character if nothing is read
    }
}



void change_baud_rate(uint32_t baud_rate) {
    esp_err_t res = uart_set_baudrate(UART_NUM, baud_rate);
    if (res == ESP_OK) {
        ESP_LOGI("UART", "Baud rate changed to %lu", baud_rate);
    } else {
        ESP_LOGE("UART", "Failed to change baud rate");
    }
}