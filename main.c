/*  (c) 2020 HomeAccessoryKid
 *  This is a dual heater concept. It is specific to my house so I do not try to make it very generic...
 *  It uses any ESP8266 with as little as 1MB flash. 
 *  GPIO-0 reads a button for manual instructions
 *  GPIO-2 is used as a bus with one-wire DS18B20 sensors to measure various temperatures
 *  UDPlogger is used to have remote logging
 *  LCM is enabled in case you want remote updates
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_system.h> //for timestamp report only
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <string.h>
#include "lwip/api.h"
// #include <wifi_config.h>
#include <udplogger.h>
//#include <adv_button.h>
#include "ds18b20/ds18b20.h"
#include "i2s_dma/i2s_dma.h"

#ifndef VERSION
 #error You must set VERSION=x.y.z to match github version tag x.y.z
#endif

#ifndef OT_RECV_PIN
 #error OT_RECV_PIN is not specified
#endif
#ifndef SENSOR_PIN
 #error SENSOR_PIN is not specified
#endif
#ifndef LED_PIN
 #error LED_PIN is not specified
#endif

int inhibit=0; //seconds pump will be inhibited

/* ============== BEGIN HOMEKIT CHARACTERISTIC DECLARATIONS =============================================================== */
// add this section to make your device OTA capable
// create the extra characteristic &ota_trigger, at the end of the primary service (before the NULL)
// it can be used in Eve, which will show it, where Home does not
// and apply the four other parameters in the accessories_information section

#include "ota-api.h"
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");

// next use these two lines before calling homekit_server_init(&config);
//    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
//                                      &model.value.string_value,&revision.value.string_value);
//    config.accessories[0]->config_number=c_hash;
// end of OTA add-in instructions

homekit_characteristic_t tgt_heat1 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  1 );
homekit_characteristic_t cur_heat1 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 1 );
homekit_characteristic_t tgt_temp1 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,         21.0 );
homekit_characteristic_t cur_temp1 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         1.0 );
homekit_characteristic_t dis_temp1 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t tgt_heat2 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  0 );
homekit_characteristic_t cur_heat2 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp2 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,         18.0 );
homekit_characteristic_t cur_temp2 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         2.0 );
homekit_characteristic_t dis_temp2 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t cur_temp3 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         3.0 );
homekit_characteristic_t cur_temp4 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         4.0 );

// void identify_task(void *_args) {
//     vTaskDelete(NULL);
// }

void identify(homekit_value_t _value) {
    UDPLUS("Identify\n");
//    xTaskCreate(identify_task, "identify", 256, NULL, 2, NULL);
}

/* ============== END HOMEKIT CHARACTERISTIC DECLARATIONS ================================================================= */

#define  ZERO 0xffffffff //inverted polarity half bits
#define  ONE  0x00000000 //inverted polarity half bits
static   dma_descriptor_t dma_block;
uint32_t dma_buf[68];
void send_OT_frame(int payload) {
    int i,j,even=0;
    printf("SENDING: %08x\n",payload);
    for (i=30,j=4 ; i>=0 ; i--,j+=2) { //j=2 is the first payload 
        if (payload&(1<<i)) dma_buf[j]=ONE,dma_buf[j+1]=ZERO,even++; else dma_buf[j]=ZERO,dma_buf[j+1]=ONE;
    }
    if (even%2) dma_buf[2]=ONE,dma_buf[3]=ZERO; else dma_buf[2]=ZERO,dma_buf[3]=ONE; //parity bit
    i2s_dma_start(&dma_block); //transmit the dma_buf once
}

int      j=0;
#define  READY 0
#define  START 1
#define  RECV  2
static QueueHandle_t xQueue;
int      resp_idx=0, rx_state=READY;
uint32_t response=0, before=0;
void test_task(void *argv) {
    uint32_t answer;
    while(1) {
        switch (tgt_heat2.value.int_value) {
            case 0:
                //generate a command pattern read slave configuration flags
                send_OT_frame( 0x00030000 );
                break;
            case 1:
                send_OT_frame( 0x00000200 ); //CH disable
                break;
            case 2:
                switch (j++){
                    case 0:
                        send_OT_frame( 0x00050000 ); //5 app specific flags
                        break;
                    case 1:
                        send_OT_frame( 0x00060000 ); //6 rem param flags
                        break;
                    case 2:
                        send_OT_frame( 0x00110000 ); //17 rel mod level
                        break;
                    case 3:
                        send_OT_frame( 0x00120000 ); //18 CH water pressure
                        break;
                    case 4:
                        send_OT_frame( 0x00190000 ); //25 boiler water temp
                        break;
                    case 5:
                        send_OT_frame( 0x001a0000 ); //26 DHW temp
                        break;
                    case 6:
                        send_OT_frame( 0x001c0000 ); //28 return water temp
                        break;
                    case 7:
                        send_OT_frame( 0x00300000 ); //48 DHW bounds
                        break;
                    case 8:
                        send_OT_frame( 0x00380000 ); //56 DHW setpoint
                        break;
                    case 9:
                        send_OT_frame( 0x007d0000 ); //125 version
                        break;
                    default:
                        send_OT_frame( 0x00030000 );
                        j=0;
                        break;
                }
                break;
            case 3:
                //generate a command pattern status set/read
                send_OT_frame( 0x00000300 ); //CH enable
                break;
            default:
                break;
        }
        if (xQueueReceive(xQueue, &(answer), (TickType_t)840/portTICK_PERIOD_MS) == pdTRUE) {
            printf("ANSWER: %08x\n",answer);
        } else printf("NO ANSWER\n");
        printf("response:%08x idx:%d\n",response,resp_idx);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

static void handle_rx(uint8_t interrupted_pin) {
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    uint32_t now=sdk_system_get_time(),delta=now-before;
    int     even=0, inv_read=gpio_read(OT_RECV_PIN);//note that gpio_read gives the inverted value of the symbol
    if (rx_state==READY) {
        if (inv_read) return;
        rx_state=START;
        before=now;
    } else if (rx_state==START) {
        if (400<delta && delta<650 && inv_read) {
            resp_idx=0; response=0; even=0;
            rx_state=RECV;
            before=now;
        } //else error state but might be a new start, so just stay in this state
    } else if (rx_state==RECV)  {
        if (900<delta && delta<1150) {
            if (resp_idx<32) {
                response=(response<<1)|inv_read;
                if (inv_read) even++;
                resp_idx++;
                before=now;
            } else {
                if (even%2==0) {
                    xQueueSendToBackFromISR(xQueue, (void*)&response, &xHigherPriorityTaskWoken);
                    //if( xHigherPriorityTaskWoken ) taskYIELD_FROM_ISR(); //TODO: find specific porting details
                } else resp_idx=-1; //signal issue
                rx_state=READY;
            }
        } else if (delta>=1150) { //error state
            if (inv_read) rx_state=READY;
            else {rx_state=START; before=now;}
        } //else do nothing so before+=500 and next transit is a databit
    }
}

#define NAN (0.0F/0.0F)
#define SENSORS 4
#define BEAT 10 //in seconds
void temp_task(void *argv) {
    ds18b20_addr_t addrs[SENSORS];
    float temps[SENSORS];
    float temp[16]; //using id as a single hex digit, then hardcode which sensor gets which meaning
    float old_t1,old_t2,old_t3,old_t4;
    int sensor_count=0,id,j;

    while( (sensor_count=ds18b20_scan_devices(SENSOR_PIN, addrs, SENSORS)) != SENSORS) {
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
        UDPLUS("Only found %d sensors\n",sensor_count);
    }

    while(1) {
        ds18b20_measure_and_read_multi(SENSOR_PIN, addrs, SENSORS, temps);
printf("after read_multi @%d\n",sdk_system_get_time()/1000);
        for (j = 0; j < SENSORS; j++) {
            // The DS18B20 address 64-bit and my batch turns out family C on https://github.com/cpetrich/counterfeit_DS18B20
            // I have manually selected that I have unique ids using the second hex digit of CRC
            id = (addrs[j]>>56)&0xF;
            temp[id] = temps[j];
            printf("  Sensor %x reports %2.4f deg C\n", id, temps[j] );
        }

        old_t1=cur_temp1.value.float_value; //TODO: do we need to test for changed values or is that embedded in notify routine?
        old_t2=cur_temp2.value.float_value;
        old_t3=cur_temp3.value.float_value;
        old_t4=cur_temp4.value.float_value;
        cur_temp1.value.float_value=isnan(temp[0])?100.0F:(float)(int)(temp[0]*2+0.5)/2; //TODO: isnan implicit declaration error in compiler?
        cur_temp2.value.float_value=isnan(temp[1])?100.0F:(float)(int)(temp[1]*2+0.5)/2;
        cur_temp3.value.float_value=isnan(temp[2])?100.0F:(float)(int)(temp[2]*2+0.5)/2;
        cur_temp4.value.float_value=isnan(temp[3])?100.0F:(float)(int)(temp[3]*2+0.5)/2;
        printf("temp1=%1.1f  temp2=%1.1f  temp3=%1.1f  temp4=%1.1f", \
            cur_temp1.value.float_value,cur_temp2.value.float_value,cur_temp3.value.float_value,cur_temp4.value.float_value);
        if (old_t1!=cur_temp1.value.float_value) {
            printf("  notify1");
            homekit_characteristic_notify(&cur_temp1,HOMEKIT_FLOAT(cur_temp1.value.float_value));
        }
        if (old_t2!=cur_temp2.value.float_value) {
            printf("  notify2");
            homekit_characteristic_notify(&cur_temp2,HOMEKIT_FLOAT(cur_temp2.value.float_value));
        }
        if (old_t3!=cur_temp3.value.float_value) {
            printf("  notify3");
            homekit_characteristic_notify(&cur_temp3,HOMEKIT_FLOAT(cur_temp3.value.float_value));
        }
        if (old_t4!=cur_temp4.value.float_value) {
            printf("  notify4");
            homekit_characteristic_notify(&cur_temp4,HOMEKIT_FLOAT(cur_temp4.value.float_value));
        }
        printf("\n");
        // ds18b20_measure_and_read_multi operation already takes at least 750ms to run
        vTaskDelay((BEAT*1000 - 800) / portTICK_PERIOD_MS);
    }
}

// void singlepress_callback(uint8_t gpio, void *args) {
//             UDPLUS("single press = stop here\n");
//             tgt_heat.value.int_value=cur_heat.value.int_value;
//             homekit_characteristic_notify(&tgt_heat,HOMEKIT_UINT8(tgt_heat.value.int_value));
// }
// 
// void doublepress_callback(uint8_t gpio, void *args) {
//             UDPLUS("double press = go open\n");
//             tgt_heat.value.int_value=100;
//             homekit_characteristic_notify(&tgt_heat,HOMEKIT_UINT8(tgt_heat.value.int_value));
// }
// 
// void longpress_callback(uint8_t gpio, void *args) {
//             UDPLUS("long press = go close\n");
//             tgt_heat.value.int_value=0;
//             homekit_characteristic_notify(&tgt_heat,HOMEKIT_UINT8(tgt_heat.value.int_value));
// }

void device_init() {
//     adv_button_set_evaluate_delay(10);
//     adv_button_create(BUTTON_PIN, true, false);
//     adv_button_register_callback_fn(BUTTON_PIN, singlepress_callback, 1, NULL);
//     adv_button_register_callback_fn(BUTTON_PIN, doublepress_callback, 2, NULL);
//     adv_button_register_callback_fn(BUTTON_PIN, longpress_callback, 3, NULL);
//     gpio_enable(LED_PIN, GPIO_OUTPUT); gpio_write(LED_PIN, 0);
    gpio_set_pullup(SENSOR_PIN, true, true);
    gpio_enable(OT_RECV_PIN, GPIO_INPUT);
    gpio_set_interrupt(OT_RECV_PIN, GPIO_INTTYPE_EDGE_ANY, handle_rx);
    //OT_SEND_PIN is GPIO3 = RX0 because hardcoded in i2s
    i2s_pins_t i2s_pins = {.data = true, .clock = false, .ws = false};
    i2s_clock_div_t clock_div = i2s_get_clock_div(64000); //1/2 OT-bit is 32bits@ 64kHz minimum value is ~40kHz at div={63,63}
    i2s_dma_init(NULL, NULL, clock_div, i2s_pins);
    dma_block.owner = 1; dma_block.sub_sof = 0; dma_block.unused = 0;
    dma_block.next_link_ptr = 0; dma_block.eof = 1; //only one block
    dma_block.datalen = 272; dma_block.blocksize = 272; // (start + 32 bits + stop) x2 1/2bits x 4byte data
    dma_block.buf_ptr = dma_buf; //uint32_t buffer type is 4byte data
    dma_buf[ 0]=ONE; dma_buf[ 1]=ZERO; //start bit
    dma_buf[66]=ONE; dma_buf[67]=ZERO; //stop  bit

    xQueue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(temp_task, "Temp", 512, NULL, 1, NULL);
    xTaskCreate(test_task, "Test", 512, NULL, 1, NULL);
}

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_thermostat,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater T-out"),
                    &cur_temp3,
                    &ota_trigger,
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater T-in"),
                    &cur_temp4,
                    NULL
                }),
            NULL
        }),
    HOMEKIT_ACCESSORY(
        .id=2,
        .category=homekit_accessory_category_thermostat,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(THERMOSTAT, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermostat"),
                    &tgt_heat1,
                    &cur_heat1,
                    &tgt_temp1,
                    &cur_temp1,
                    &dis_temp1,
                    NULL
                }),
            NULL
        }),
    HOMEKIT_ACCESSORY(
        .id=3,
        .category=homekit_accessory_category_thermostat,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(THERMOSTAT, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermo 2"),
                    &tgt_heat2,
                    &cur_heat2,
                    &tgt_temp2,
                    &cur_temp2,
                    &dis_temp2,
                    NULL
                }),
            NULL
        }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};


void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(3);
    UDPLUS("\n\n\nDual-Heater-OpenTherm " VERSION "\n");

    device_init();
    
    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
    //c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;
    
    homekit_server_init(&config);
}
