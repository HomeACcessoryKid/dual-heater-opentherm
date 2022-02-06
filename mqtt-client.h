/*  (c) 2021-2022 HomeAccessoryKid 
 *  Intended as a Domoticz publish only feed
 *  In the Makefile add EXTRA_COMPONENTS = extras/paho_mqtt_c 
 *  create a mqtt_config_t with intial value MQTT_DEFAULT_CONFIG
 *  fill in the host, user and pass and non-default values
 *  then call mqtt_client_init
 *  mqtt_client_publish has the same syntax as printf
 */
#ifndef __MQTT_CLIENT_H__
#define __MQTT_CLIENT_H__

typedef struct mqtt_config {
    int  dummy; //somehow the first entry is treated as a stray pointer so this is a workaround
    int  queue_size;
    int  msg_len;
    char *host;
    int  port;
    char *user;
    char *pass;
    char *topic;
} mqtt_config_t;
#define MQTT_DEFAULT_CONFIG {0,3,48,NULL,1883,NULL,NULL,"domoticz/in"}
#define MQTT_CLIENT_ERROR(ret)    (ret==-1?"queue full":"message too long")

void mqtt_client_init(mqtt_config_t *config);
int  mqtt_client_publish(char *format,  ...);

#endif // __MQTT_CLIENT_H__
