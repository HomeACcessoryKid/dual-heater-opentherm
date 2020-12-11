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
#include <timers.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <string.h>
#include "lwip/api.h"
#include <udplogger.h>
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
#ifndef SWITCH_PIN
 #error SWITCH_PIN is not specified
#endif
#ifndef LED_PIN
 #error LED_PIN is not specified
#endif

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

homekit_characteristic_t tgt_heat1 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  3 );
homekit_characteristic_t cur_heat1 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp1 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,         19.5 );
homekit_characteristic_t cur_temp1 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         1.0 );
homekit_characteristic_t dis_temp1 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t tgt_heat2 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  1 );
homekit_characteristic_t cur_heat2 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp2 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,         38.0 );
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
    printf("SND:%08x ",payload);
    for (i=30,j=4 ; i>=0 ; i--,j+=2) { //j=2 is the first payload 
        if (payload&(1<<i)) dma_buf[j]=ONE,dma_buf[j+1]=ZERO,even++; else dma_buf[j]=ZERO,dma_buf[j+1]=ONE;
    }
    if (even%2) dma_buf[2]=ONE,dma_buf[3]=ZERO; else dma_buf[2]=ZERO,dma_buf[3]=ONE; //parity bit
    i2s_dma_start(&dma_block); //transmit the dma_buf once
}

#ifdef DEBUG_RECV
 uint32_t times[1000], oldtime=0;
 int      level[1000], idx=0;
#endif
#define  READY 0
#define  START 1
#define  RECV  2
static QueueHandle_t xQueue;
int      resp_idx=0, rx_state=READY;
uint32_t response=0, before=0;
static void handle_rx(uint8_t interrupted_pin) {
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    uint32_t now=sdk_system_get_time(),delta=now-before;
    int     even=0, inv_read=gpio_read(OT_RECV_PIN);//note that gpio_read gives the inverted value of the symbol
#ifdef DEBUG_RECV
    times[idx]=now; level[idx++]=inv_read;
#endif
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
                    if (response&0x0f000000) resp_idx=-2; //signal issue reserved bits not zero
                    else {
                        response&=0x7fffffff; //mask parity bit
                        xQueueSendToBackFromISR(xQueue, (void*)&response, &xHigherPriorityTaskWoken);
                        //if( xHigherPriorityTaskWoken ) taskYIELD_FROM_ISR(); //TODO: find specific porting details
                    }
                } else resp_idx=-1; //signal issue parity failure
                rx_state=READY;
            }
        } else if (delta>=1150) { //error state
            if (inv_read) rx_state=READY;
            else {rx_state=START; before=now;}
        } //else do nothing so before+=500 and next transit is a databit
    }
}

#define BEAT 10 //in seconds
#define NAN (0.0F/0.0F)
#define SENSORS 4
#define S1 0 //salon temp sensor
#define S2 1 //upstairs temp sensor
#define S5 6 //outside temp sensor
#define S3 2 //outgoing water temp sensor
#define S4 3 //incoming water temp sensor
#define BW 4 //boiler water temp
#define RW 5 //return water temp
#define DW 8 //domestic home water temp
float temp[16]; //using id as a single hex digit, then hardcode which sensor gets which meaning
void temp_task(void *argv) {
    ds18b20_addr_t addrs[SENSORS];
    float temps[SENSORS];
    float old_t1,old_t2,old_t3,old_t4;
    int sensor_count=0,id,j;

    while( (sensor_count=ds18b20_scan_devices(SENSOR_PIN, addrs, SENSORS)) != SENSORS) {
        UDPLUS("Only found %d sensors\n",sensor_count);
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
    }

    while(1) {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        ds18b20_measure_and_read_multi(SENSOR_PIN, addrs, SENSORS, temps);
        for (j = 0; j < SENSORS; j++) {
            // The DS18B20 address 64-bit and my batch turns out family C on https://github.com/cpetrich/counterfeit_DS18B20
            // I have manually selected that I have unique ids using the second hex digit of CRC
            id = (addrs[j]>>56)&0xF;
            temp[id] = temps[j];
        }
        old_t1=cur_temp1.value.float_value; //TODO: do we need to test for changed values or is that embedded in notify routine?
        old_t2=cur_temp2.value.float_value; //TODO: convert this in a macro
        old_t3=cur_temp3.value.float_value;
        old_t4=cur_temp4.value.float_value;
        cur_temp1.value.float_value=isnan(temp[S1])?100.0F:(float)(int)(temp[S1]*2+0.5)/2; //TODO: isnan implicit declaration error in compiler?
        cur_temp2.value.float_value=isnan(temp[S2])?100.0F:(float)(int)(temp[S2]*2+0.5)/2;
        cur_temp3.value.float_value=isnan(temp[S3])?100.0F:(float)(int)(temp[S3]*2+0.5)/2;
        cur_temp4.value.float_value=isnan(temp[S4])?100.0F:(float)(int)(temp[S4]*2+0.5)/2;
        if (old_t1!=cur_temp1.value.float_value) homekit_characteristic_notify(&cur_temp1,HOMEKIT_FLOAT(cur_temp1.value.float_value));
        if (old_t2!=cur_temp2.value.float_value) homekit_characteristic_notify(&cur_temp2,HOMEKIT_FLOAT(cur_temp2.value.float_value));
        if (old_t3!=cur_temp3.value.float_value) homekit_characteristic_notify(&cur_temp3,HOMEKIT_FLOAT(cur_temp3.value.float_value));
        if (old_t4!=cur_temp4.value.float_value) homekit_characteristic_notify(&cur_temp4,HOMEKIT_FLOAT(cur_temp4.value.float_value));
        // ds18b20_measure_and_read_multi operation takes about 800ms to run, 3ms start, 750ms wait, 11ms/sensor to read
    }
}

float curr_mod=0,pressure=0;
int   stateflg=0,errorflg=0;
static TaskHandle_t tempTask = NULL;
int timeIndex=0,switch_state=0;
TimerHandle_t xTimer;
void vTimerCallback( TimerHandle_t xTimer ) {
    uint32_t counter = ( uint32_t ) pvTimerGetTimerID( xTimer );
    vTimerSetTimerID( xTimer, (void*)counter+1); //136 year to loop
    uint32_t message;
    int switch_on=0;
    if (gpio_read(SWITCH_PIN)) switch_state--; else switch_state++; //pin is low when switch is on
    if (switch_state<0) switch_state=0;
    if (switch_state>3) switch_state=3;
    switch_on=switch_state>>1;
    //TODO read recv pin and if it is a ONE, we have an OpenTherm error state
    printf("St%d Sw%d @%d ",timeIndex,switch_on,counter);
    switch (timeIndex) { //send commands
        case 0: //measure temperature
            xTaskNotifyGive( tempTask ); //temperature measurement start
            vTaskDelay(1); //prevent interference between OneWire and OT-receiver
            send_OT_frame(0x00190000); //25 read boiler water temperature
            break;
        case 1: //calculate heater decisions
            //blabla
            if (tgt_heat2.value.int_value==3) {
                   message=0x10014000; //64 deg
            } else message=0x10010000|(uint32_t)(tgt_temp1.value.float_value*2-1)*256; //range from 19 - 75 deg
            send_OT_frame(message); //1  CH setpoint in deg C
            break;
        case 2:
            if (tgt_heat2.value.int_value==3) {
                   message=0x100e6400; //100%
            } else message=0x100e0000|(uint32_t)(((tgt_temp2.value.float_value-10)*(100.0/28.0))*256);
            send_OT_frame(message); //14 max modulation level
            break;
        case 3:
            if (tgt_heat2.value.int_value==3) {
                   message=0x00000200|(switch_on?0x100:0x000);
            } else message=0x00000000|(tgt_heat1.value.int_value<<8);
            send_OT_frame( message ); //0  enable CH and DHW
            break; 
        case 4: send_OT_frame( 0x00380000 ); break; //56 DHW setpoint write
        case 5: send_OT_frame( 0x00050000 ); break; //5  app specific fault flags
        case 6: send_OT_frame( 0x00120000 ); break; //18 CH water pressure
        case 7: send_OT_frame( 0x001a0000 ); break; //26 DHW temp
        case 8: send_OT_frame( 0x001c0000 ); break; //28 return water temp
        case 9: send_OT_frame( 0x00110000 ); break; //17 rel mod level
        default: break;
    }
    
    if (xQueueReceive(xQueue, &(message), (TickType_t)850/portTICK_PERIOD_MS) == pdTRUE) {
        printf("RSP:%08x\n",message);
        switch (timeIndex) { //check answers
            case 0: temp[BW]=(float)(message&0x0000ffff)/256; break;
            case 3:
                stateflg=(message&0x0000007f);
                cur_heat1.value.int_value=stateflg&0x8?1:0;
                homekit_characteristic_notify(&cur_heat1,HOMEKIT_UINT8(cur_heat1.value.int_value));
                break;
            case 5: errorflg=       (message&0x00003f00)/256; break;
            case 6: pressure=(float)(message&0x0000ffff)/256; break;
            case 7: temp[DW]=(float)(message&0x0000ffff)/256; break;
            case 8: temp[RW]=(float)(message&0x0000ffff)/256; break;
            case 9: curr_mod=(float)(message&0x0000ffff)/256; break;
            default: break;
        }
    } else {
        printf("!!! NO_RSP: resp_idx=%d rx_state=%d response=%08x\n",resp_idx, rx_state, response);
        resp_idx=0, rx_state=READY, response=0;
#ifdef DEBUG_RECV
        for (int i=0;i<idx;i++) {
            printf("%4d=%d%s", ((times[i]-oldtime)/10)*10, level[i], i%16?" ":"\n");
            oldtime=times[i];
        }
        idx=0; printf("\n");
    }
    if (idx>500) {
        for (int i=0;i<750;i++) times[i]=times[i+250];
        idx-=250;
#endif
    }
    
    if (!timeIndex) printf("PR=%1.2f DW=%2.4f S2=%2.4f S3=%2.4f S4=%2.4f ERR=%02x RW=%2.4f BW=%2.4f S1=%2.4f MOD=%2.0f ST=%02x\n", \
                       pressure,temp[DW],temp[S2],temp[S3],temp[S4],errorflg,temp[RW],temp[BW],temp[S1],curr_mod,stateflg);
    timeIndex++; if (timeIndex==BEAT) timeIndex=0;
} //this is a timer that restarts every 1 second

void device_init() {
//     gpio_enable(LED_PIN, GPIO_OUTPUT); gpio_write(LED_PIN, 0);
    gpio_set_pullup(SENSOR_PIN, true, true);
    gpio_enable(SWITCH_PIN, GPIO_INPUT);
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
    xTaskCreate(temp_task,"Temp", 512, NULL, 1, &tempTask);
    xTimer=xTimerCreate( "Timer", 1000/portTICK_PERIOD_MS, pdTRUE, (void*)0, vTimerCallback);
    xTimerStart(xTimer, 0);
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
