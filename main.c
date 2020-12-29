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
#include "math.h"
#include <lwip/apps/sntp.h>

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

homekit_characteristic_t tgt_heat2 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  3 );
homekit_characteristic_t cur_heat2 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp2 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,         38.0 );
homekit_characteristic_t cur_temp2 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         2.0 );
homekit_characteristic_t dis_temp2 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t cur_temp3 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         3.0 );
homekit_characteristic_t cur_temp4 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         4.0 );
homekit_characteristic_t cur_temp5 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         5.0 );

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
        } //else error state but might be a new start, so just stay in this state
        before=now;
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

int  time_set=0;
void time_task(void *argv) {
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); tzset();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "0.pool.ntp.org");
    sntp_setservername(1, "1.pool.ntp.org");
    sntp_setservername(2, "2.pool.ntp.org");
    while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) vTaskDelay(20); //Check if we have an IP every 200ms
    sntp_init();
    time_t ts;
    do {ts = time(NULL);
        if (ts == ((time_t)-1)) printf("ts=-1 ");
        vTaskDelay(10);
    } while (!(ts>1608567890)); //Mon Dec 21 17:24:50 CET 2020
    printf("TIME SET: %u=%s", (unsigned int) ts, ctime(&ts));
    time_set=1;
    vTaskDelete(NULL); //check if NTP keeps running without this task
}

#define TEMP2HK(n)  do {old_t##n=cur_temp##n.value.float_value; \
                        cur_temp##n.value.float_value=isnan(temp[S##n])?S##n##avg:(float)(int)(temp[S##n]*10+0.5)/10; \
                        if (old_t##n!=cur_temp##n.value.float_value) \
                            homekit_characteristic_notify(&cur_temp##n,HOMEKIT_FLOAT(cur_temp##n.value.float_value)); \
                    } while (0) //TODO: do we need to test for changed values or is that embedded in notify routine?
#define BEAT 10 //in seconds
#define SENSORS 5
#define S1 0 //   salon temp sensor
#define S2 1 //upstairs temp sensor
#define S3 6 // outdoor temp sensor
#define S4 2 //outgoing water temp sensor
#define S5 3 // ingoing water temp sensor
#define BW 4 //boiler water temp
#define RW 5 //return water temp
#define DW 8 //domestic home water temp
float temp[16]; //using id as a single hex digit, then hardcode which sensor gets which meaning
float S1temp[6],S2temp[6],S3temp[6],S1avg,S2avg,S3avg,S4avg,S5avg;
void temp_task(void *argv) {
    ds18b20_addr_t addrs[SENSORS];
    float temps[SENSORS];
    float old_t1,old_t2,old_t3,old_t4,old_t5;
    int sensor_count=0,id;

    while( (sensor_count=ds18b20_scan_devices(SENSOR_PIN, addrs, SENSORS)) != SENSORS) {
        UDPLUS("Only found %d sensors\n",sensor_count);
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
    }

    while(1) {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        ds18b20_measure_and_read_multi(SENSOR_PIN, addrs, SENSORS, temps);
        for (int j = 0; j < SENSORS; j++) {
            // The DS18B20 address 64-bit and my batch turns out family C on https://github.com/cpetrich/counterfeit_DS18B20
            // I have manually selected that I have unique ids using the second hex digit of CRC
            id = (addrs[j]>>56)&0xF;
            temp[id] = temps[j];
        } // ds18b20_measure_and_read_multi operation takes about 800ms to run, 3ms start, 750ms wait, 11ms/sensor to read
        TEMP2HK(1);
        TEMP2HK(2);
        TEMP2HK(3);
        TEMP2HK(4);
        TEMP2HK(5);
    }
}

#define STABLE 0
#define HEAT   1
#define EVAL   2

int peak_time=0,time_on=0,mode=EVAL,heater1=0;
float factor=650, prev_setpoint=21.5, peak_temp=0;
time_t heat_till=0;
int   stateflg=0,errorflg=0;
void heater(uint32_t seconds) {
    if (!time_set) return; //need reliable time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&(tv.tv_sec));
    printf("Heater@ %4d DST%d day%d wd%d %02d:%02d:%02d.%06d = %s", \
            (seconds+10)/60,tm->tm_isdst,tm->tm_yday,tm->tm_wday, \
            tm->tm_hour,tm->tm_min,tm->tm_sec,(int)tv.tv_usec,ctime(&(tv.tv_sec)));

    heater1=0;
    float setpoint=tgt_temp1.value.float_value;
    if (setpoint!=prev_setpoint) {
        if (setpoint>prev_setpoint) {
            time_on=factor*(setpoint-S1avg);
            if (time_on>15) { //15 minutes at least, else too quick
                heat_till=tv.tv_sec+(time_on*60);
                mode=HEAT;
            } else mode=STABLE;
        } else mode=EVAL;
        prev_setpoint=setpoint;
    }

    if (mode==HEAT) {
        if (tv.tv_sec>heat_till) {
            time_on=0;
            mode=EVAL;
        } else {
            time_on--;
            heater1=1;
        }
    } else if (mode==EVAL) {
        if (peak_temp<S1avg) {
            peak_temp=S1avg;
            peak_time=0;
        } else if (S1avg<(peak_temp-0.07) || peak_time++>30) {
            mode=STABLE;
            //adjust factor
            peak_temp=0,peak_time=0;
        }
    } else if (mode==STABLE) {
        time_on=(factor*(setpoint-S1avg)*0.2);
        if (time_on>15) {
            heat_till=tv.tv_sec+(time_on*60);
            mode=HEAT;
        }
    }
    
    printf("S1=%2.4f S2=%2.4f S3=%2.4f f=%2.1f time-on=%dmin peak_time=%d peak_temp=%2.4f ST=%02x mode=%d till %s", \
            S1avg,S2avg,S3avg,factor,time_on,peak_time,peak_temp,stateflg,mode,mode==HEAT?ctime(&heat_till):"\n");
}

float curr_mod=0,pressure=0;
static TaskHandle_t tempTask = NULL;
int timeIndex=0,switch_state=0;
TimerHandle_t xTimer;
void vTimerCallback( TimerHandle_t xTimer ) {
    uint32_t seconds = ( uint32_t ) pvTimerGetTimerID( xTimer );
    vTimerSetTimerID( xTimer, (void*)seconds+1); //136 year to loop
    uint32_t message;
    int switch_on=0;
    if (gpio_read(SWITCH_PIN)) switch_state--; else switch_state++; //pin is low when switch is on
    if (switch_state<0) switch_state=0;
    if (switch_state>3) switch_state=3;
    switch_on=switch_state>>1;
    //TODO read recv pin and if it is a ONE, we have an OpenTherm error state
    printf("St%d Sw%d @%d ",timeIndex,switch_on,seconds);
    switch (timeIndex) { //send commands
        case 0: //measure temperature
            xTaskNotifyGive( tempTask ); //temperature measurement start
            vTaskDelay(1); //prevent interference between OneWire and OT-receiver
            send_OT_frame(0x00190000); //25 read boiler water temperature
            break;
        case 1: //execute heater decisions
            if (tgt_heat2.value.int_value==1) {
                   message=0x10014000; //64 deg
            } else if (tgt_heat2.value.int_value==3) { //run heater algoritm for floor heating
                   message=0x10014100; //65 deg
            } else message=0x10010000|(uint32_t)(tgt_temp1.value.float_value*2-1)*256; //range from 19 - 75 deg
            send_OT_frame(message); //1  CH setpoint in deg C
            break;
        case 2:
            if (tgt_heat2.value.int_value==1) {
                   message=0x00000200|(switch_on?0x100:0x000);
            } else if (tgt_heat2.value.int_value==3) { //run heater algoritm for floor heating
                   message=0x00000200|(  heater1?0x100:0x000);
            } else message=0x00000200|(tgt_heat1.value.int_value<<8);
            send_OT_frame( message ); //0  enable CH and DHW
            break; 
        case 3: send_OT_frame( 0x100e6400 ); break; //14 max modulation level 100%
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
            case 2:
                stateflg=(message&0x0000007f);
                cur_heat1.value.int_value=stateflg&0xa?1:0;
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
#endif //DEBUG_RECV
    }
    
    if (!timeIndex) {
        S1temp[5]=S1temp[4];S1temp[4]=S1temp[3];S1temp[3]=S1temp[2];S1temp[2]=S1temp[1];S1temp[1]=S1temp[0];
        S2temp[5]=S2temp[4];S2temp[4]=S2temp[3];S2temp[3]=S2temp[2];S2temp[2]=S2temp[1];S2temp[1]=S2temp[0];
        S3temp[5]=S3temp[4];S3temp[4]=S3temp[3];S3temp[3]=S3temp[2];S3temp[2]=S3temp[1];S3temp[1]=S3temp[0];
        if(!isnan(temp[S1]))S1temp[0]=temp[S1];if(!isnan(temp[S2]))S2temp[0]=temp[S2];if(!isnan(temp[S3]))S3temp[0]=temp[S3];
        S1avg=(S1temp[0]+S1temp[1]+S1temp[2]+S1temp[3]+S1temp[4]+S1temp[5])/6.0;
        S2avg=(S2temp[0]+S2temp[1]+S2temp[2]+S2temp[3]+S2temp[4]+S2temp[5])/6.0;
        S3avg=(S3temp[0]+S3temp[1]+S3temp[2]+S3temp[3]+S3temp[4]+S3temp[5])/6.0;
        S4avg=temp[S4]; S5avg=temp[S5];
        printf("S1=%2.4f S2=%2.4f S3=%2.4f PR=%1.2f DW=%2.4f S4=%2.4f S5=%2.4f ERR=%02x RW=%2.4f BW=%2.4f MOD=%02.0f ST=%02x\n", \
           temp[S1],temp[S2],temp[S3],pressure,temp[DW],temp[S4],temp[S5],errorflg,temp[RW],temp[BW],curr_mod,stateflg);
    }
    timeIndex++; if (timeIndex==BEAT) timeIndex=0;
    if (seconds%60==50) { //allow 6 temperature measurments to make sure all info is loaded
        heater(seconds);
    }
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
    xTaskCreate(time_task,"Time", 512, NULL, 6, NULL);
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
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Outdoor T"),
                    &cur_temp3,
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater T-out"),
                    &cur_temp4,
                    &ota_trigger,
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Heater T-in"),
                    &cur_temp5,
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
