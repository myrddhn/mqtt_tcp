/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "u8g2.h"
#include "u8g2_esp8266_hal.h"
#include "darwinistic.xbm"
#include "bmp180.h"
#include "i2cdev.h"

// #define GPIO_VCC      14
// #define GPIO_GND      12
#define GPIO_SDA      12
#define GPIO_SCL      14

static const char *TAG = "MQTT_EXAMPLE";
u8g2_t u8g2;
static char buff[64];
char topic[64];
char data[64];
float t;
esp_mqtt_client_handle_t client = NULL;
int msg_id;
static float temp;
static uint32_t pressure;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    client = event->client;
    // int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);

            snprintf(topic, 64, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
            snprintf(data, 64, "DATA=%.*s\r\n", event->data_len, event->data);

            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void printMessage(void *args) {
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawXBM(&u8g2, 0, 0, darwinistic_width, darwinistic_height, darwinistic_bits);
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(2500 / portTICK_PERIOD_MS);

    while (1) {
        vTaskSuspendAll();

        u8g2_ClearBuffer(&u8g2);

        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tf);
        u8g2_DrawUTF8(&u8g2, 15, 20, buff);
        u8g2_SetFont(&u8g2, u8g_font_6x12);
        u8g2_DrawStr(&u8g2, 6, 35, topic);
        u8g2_DrawStr(&u8g2, 6, 46, data);
        u8g2_DrawStr(&u8g2, 33, 62, "Hello MQTT");

        u8g2_SendBuffer(&u8g2);
        xTaskResumeAll();

        vTaskDelay(250 / portTICK_PERIOD_MS);
        taskYIELD();
        ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
    }
}

static void temperature_task(void *args) {
    bmp180_dev_t dev;
    gpio_num_t SDA = GPIO_NUM_12;
    gpio_num_t SCL = GPIO_NUM_14;
    static float lastvalue = 0.00F;
    memset(&dev, 0, sizeof(bmp180_dev_t)); // Zero descriptor

    bmp180_init_desc(&dev, 0, SDA, SCL);
    bmp180_init(&dev);

    while (1)
    {
        esp_err_t res = bmp180_measure(&dev, &temp, &pressure, BMP180_MODE_HIGH_RESOLUTION);
        if (res != ESP_OK) {
            printf("Could not measure: %d\n", res);
        } else {
            /* float is used in printf(). you need non-default configuration in
             * sdkconfig for ESP8266, which is enabled by default for this
             * example. see sdkconfig.defaults.esp8266
             */
            printf("Temperature: %.2f degrees Celsius; Pressure: %.2f mbar\n", temp, (double)pressure / 100.0F);
            snprintf(buff, 63, "%.2f" "\xc2\xb0" "C", temp);
            if (client != NULL) {
                //if (lastvalue != temp) {
                    msg_id = esp_mqtt_client_publish(client, "/home/temp/int", buff, 0, 0, 0);
                    snprintf(buff, 63, "%.2f", temp);
                    msg_id = esp_mqtt_client_publish(client, "/home/temp/int/raw", buff, 0, 0, 0);
                    printf("PUBLISHED TEMPERATURE!!!\r\n");
                    lastvalue = temp;
                //}
            }
            snprintf(buff, 63, "%.2f", (double)pressure / 100.0F);
            msg_id = esp_mqtt_client_publish(client, "/home/pressure/raw", buff, 0, 0, 0);
            snprintf(buff, 63, "%lu", (unsigned long)time(NULL));
            msg_id = esp_mqtt_client_publish(client, "/home/timestamp", buff, 0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10 * 1000));
    }

    /*static float lastvalue = 0.00F;
    while (1) {
        snprintf(buff, 31, "%.4f°C", t / 10);
        if (client != NULL) {
            if (lastvalue != t) {
                msg_id = esp_mqtt_client_publish(client, "/home/temp/int", buff, 0, 0, 0);
                lastvalue = t;
            }
        }
        taskYIELD();
    }
    */
}

void app_main(void) {
    u8g2_esp8266_hal_t hal = U8G2_ESP8266_HAL_DEFAULT;


    hal.scl = GPIO_NUM_4;
    hal.sda = GPIO_NUM_5;

    u8g2_esp8266_hal_init(hal);
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp8266_i2c_byte_cb, u8g2_esp8266_gpio_and_delay_cb);

    // u8g2_InitDisplay(&u8g2);

    i2cdev_init();

    // u8g2_SetPowerSave(&u8g2, 0);
    // u8g2_SetI2CAddress(&u8g2, 0x78);

    // //u8x8_cad_StartTransfer(&u8g2.u8x8);
    // //u8x8_cad_SendCmd(&u8g2.u8x8, 0xa7);
    // //u8x8_cad_EndTransfer(&u8g2.u8x8);
    // u8g2_SetContrast(&u8g2, 5);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();

    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT_OD;
    // io_conf.pin_bit_mask = (1ULL << GPIO_SCL) | (1ULL << GPIO_SDA);
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 0;
    // gpio_config(&io_conf);

    sprintf(topic, "TOPIC=no topic");
    sprintf(data, "DATA=no data");
    // xTaskCreate(printMessage, "Print Message", 2048, NULL, 3, NULL);
    xTaskCreate(temperature_task, "Temperature Task", 2048, NULL, 2, NULL);
}
