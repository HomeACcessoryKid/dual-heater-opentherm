/*  (c) 2021-2022 HomeAccessoryKid
 *  Intended as a Domoticz publish only feed
 */
#include <stdarg.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <FreeRTOS.h>
#include <string.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <semphr.h>
#include "mqtt-client.h"

QueueHandle_t publish_queue;
mqtt_config_t *mqttconf;

static const char *  get_my_id(void) {
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i) {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

#define BACKOFF1 100/portTICK_PERIOD_MS
static void  mqtt_task(void *pvParameters) {
    int ret = 0;
    int backoff = BACKOFF1;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_readbuf[4]; //we do not intend to use this, but a minimum might be needed?? guessing 4
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;
    char msg[mqttconf->msg_len];
    int  mqtt_buf_len;
    
    ret=20+sizeof(mqtt_client_id)+strlen(mqttconf->user)+strlen(mqttconf->pass);
    mqtt_buf_len=8+strlen(mqttconf->topic)+mqttconf->msg_len;
    if (mqtt_buf_len<ret) mqtt_buf_len=ret;
    uint8_t *mqtt_buf=malloc(mqtt_buf_len);

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "MAC-");
    strcat(mqtt_client_id, get_my_id());

    data.willFlag           = 0;
    data.MQTTVersion        = 3;
    data.clientID.cstring   = mqtt_client_id;
    data.username.cstring   = mqttconf->user;
    data.password.cstring   = mqttconf->pass;
    data.keepAliveInterval  = 10;
    data.cleansession       = 0;

    printf("%s: started\n", __func__);
    while(1) {
        while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) vTaskDelay(200/portTICK_PERIOD_MS); //Check if we have an IP
        printf("%s: (re)connecting to MQTT server %s ... ",__func__, mqttconf->host);
        ret = mqtt_network_connect(&network, mqttconf->host, mqttconf->port);
        if( ret ){
            printf("error: %d\n", ret);
            vTaskDelay(backoff);
            if (backoff<BACKOFF1*128) backoff*=2; //max out at 12.8 seconds
            continue;
        }
        printf("done\n");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, mqtt_buf_len, mqtt_readbuf, 4);
        printf("%s: send MQTT connect ... ", __func__);
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n", ret);
            mqtt_network_disconnect(&network);
            vTaskDelay(backoff);
            if (backoff<BACKOFF1*128) backoff*=2; //max out at 12.8 seconds
            continue;
        }
        printf("done\n");
        backoff = BACKOFF1;

        while(1) {
            msg[mqttconf->msg_len - 1] = 0;
            while(xQueueReceive(publish_queue, (void *)msg, 0) == pdTRUE){
                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, mqttconf->topic , &message);
                if (ret != MQTT_SUCCESS ){
                    printf("%s: error while publishing message: %d\n", __func__, ret );
                    break;
                }
            }
            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED) break;
        }
        printf("%s: connection dropped, connecting again\n", __func__);
        mqtt_network_disconnect(&network);
        xQueueReset(publish_queue);
    }
}

int mqtt_client_publish(char *format, ...) {
    char msg[mqttconf->msg_len];
    va_list args;
    va_start(args, format);
    int n=vsnprintf(msg, mqttconf->msg_len,format,args);
    va_end(args);
    if (n>=mqttconf->msg_len) return -2; //truncated message
    if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) return -1; //message queue full
    return n;
}

void mqtt_client_init(mqtt_config_t *config) {
    mqttconf=config;
    publish_queue = xQueueCreate(mqttconf->queue_size, mqttconf->msg_len);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 2, NULL);
}
