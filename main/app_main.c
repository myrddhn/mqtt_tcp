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
#include "ds18x20.h"
#include "darwinistic.xbm"

#define GPIO_OUT_1      14
#define GPIO_OUT_2      12
#define GPIO_OUT_3      13
#define GPIO_OUT_4      15

static const char *TAG = "MQTT_EXAMPLE";
u8g2_t u8g2;
static char buff[30];
char topic[64];
char data[64];
float t;
esp_mqtt_client_handle_t client = NULL;
int msg_id;

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

            /*strncpy(topic, event->topic, 63);
            topic[event->topic_len] = 0x00;

            strncpy(data, event->data, 63);
            data[event->data_len] = 0x00;
            */

            /*
            event->topic[event->topic_len] = 0x00;
            snprintf(topic, 64, "topic: %s", event->topic);

            event->data[event->data_len] = 0x00;
            snprintf(data, 64, "data : %s", event->data);
            */

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
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

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

        // u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp8266_i2c_byte_cb, u8g2_esp8266_gpio_and_delay_cb);
        // u8g2_SetI2CAddress(&u8g2, 0x78);

        u8g2_ClearBuffer(&u8g2);
        //u8g2_DrawRFrame(&u8g2, 0, 0, 128, 64, 5);
        // printf("Temperature: %.4f°C\r\n", t / 10);
        u8g2_SetDrawColor(&u8g2, 1);

        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tf);
        //u8g2_DrawStr(&u8g2, 20, 20, buff);
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
    static float lastvalue = 0.00F;
    while (1) {
        ds18x20_measure(GPIO_OUT_2, DS18X20_ANY, true);
        ds18x20_read_temperature(GPIO_OUT_2, DS18X20_ANY, &t);
        snprintf(buff, 31, "%.4f°C", t / 10);

        if (client != NULL) {
            if (lastvalue != t) {
                msg_id = esp_mqtt_client_publish(client, "/home/temp/int", buff, 0, 0, 0);
                lastvalue = t;
            }
            //client = NULL;
        }
        taskYIELD();
    }
}

void app_main(void) {
    u8g2_esp8266_hal_t hal = U8G2_ESP8266_HAL_DEFAULT;

    hal.scl = GPIO_NUM_4;
    hal.sda = GPIO_NUM_5;

    u8g2_esp8266_hal_init(hal);
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp8266_i2c_byte_cb, u8g2_esp8266_gpio_and_delay_cb);
 
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetI2CAddress(&u8g2, 0x78);

    u8x8_cad_StartTransfer(&u8g2.u8x8);
    //u8x8_cad_SendCmd(&u8g2.u8x8, 0xa7);
    u8x8_cad_EndTransfer(&u8g2.u8x8);
    u8g2_SetContrast(&u8g2, 5);

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

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUT_1) | (1ULL << GPIO_OUT_3);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUT_2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUT_1, 1);
    gpio_set_level(GPIO_OUT_3, 0);

    sprintf(topic, "TOPIC=no topic");
    sprintf(data, "DATA=no data");
    xTaskCreate(printMessage, "Print Message", 2048, NULL, 3, NULL);
    xTaskCreate(temperature_task, "Temperature Task", 2048, NULL, 2, NULL);
}
