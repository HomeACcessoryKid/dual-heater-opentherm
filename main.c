/*  (c) 2020-2021 HomeAccessoryKid
 *  This is a dual heater concept. It is specific to my house so I do not try to make it very generic...
 *  It uses any ESP8266 with as little as 1MB flash. 
 *  GPIO-2 is used as a bus with one-wire DS18B20 sensors to measure various temperatures
 *  GPIO-3 is used for I2S to generate the Opentherm Manchester code bit pattern
 *  GPIO-4 is used for interupt driven decoding of Manchester code from heater
 *  GPIO-0 is reading the conventional thermostat on/off value
 *  UDPlogger is used to have remote logging
 *  LCM is enabled in case you want remote updates
 */
// TODO: make factor depend on S3 long avg
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
#include <espressif/esp8266/eagle_soc.h>
#include "mqtt-client.h"
#include <sysparam.h>
#include "ping.h"
#include <rboot-api.h>

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

#define DEFAULT1 19.0
#define DEFAULT2 19.0

int idx; //the domoticz base index
#define PUBLISH(name) do {int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%.1f\"}", idx+name##_ix, name##_fv); \
                            if (n<0) printf("MQTT publish of %s failed because %s\n",#name,MQTT_CLIENT_ERROR(n)); \
                           } while(0)
#define tgt_temp1_fv tgt_temp1.value.float_value
#define tgt_temp1_ix 1
#define tgt_temp2_fv tgt_temp2.value.float_value
#define tgt_temp2_ix 2
#define     S1avg_fv S1avg
#define     S1avg_ix 3
#define     S2avg_fv S2avg
#define     S2avg_ix 4
#define  heat_mod_fv heat_mod
#define  heat_mod_ix 5
#define   heat_sp_fv heat_sp
#define   heat_sp_ix 6
#define   burnerW_fv temp[BW]
#define   burnerW_ix 7
#define   returnW_fv temp[RW]
#define   returnW_ix 8
#define  pressure_fv pressure*10.0
#define  pressure_ix 9
#define     S3avg_fv S3avg
#define     S3avg_ix 10
#define    S3long_fv S3long
#define    S3long_ix 11

char    *pinger_target=NULL;

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

void tgt_temp1_set(homekit_value_t value);
void tgt_temp2_set(homekit_value_t value);
homekit_characteristic_t tgt_heat1 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  3 );
homekit_characteristic_t cur_heat1 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp1 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,     DEFAULT1, .setter=tgt_temp1_set );
homekit_characteristic_t cur_temp1 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         1.0 );
homekit_characteristic_t dis_temp1 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t tgt_heat2 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  3 );
homekit_characteristic_t cur_heat2 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp2 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,     DEFAULT2, .setter=tgt_temp2_set );
homekit_characteristic_t cur_temp2 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         2.0 );
homekit_characteristic_t dis_temp2 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t cur_temp3 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         3.0 );


void tgt_temp1_set(homekit_value_t value) {
    if (value.format != homekit_format_float) {
        UDPLUO("Invalid target-value format: %d\n", value.format);
        return;
    }
    tgt_temp1.value=value;
    PUBLISH(tgt_temp1);
}

void tgt_temp2_set(homekit_value_t value) {
    if (value.format != homekit_format_float) {
        UDPLUO("Invalid target-value format: %d\n", value.format);
        return;
    }
    tgt_temp2.value=value;
    PUBLISH(tgt_temp2);
}


float ffactor=600;
#define HOMEKIT_CHARACTERISTIC_CUSTOM_FACTOR HOMEKIT_CUSTOM_UUID("F0000009")
#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_FACTOR(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_FACTOR, \
    .description = "HeaterFactor", \
    .format = homekit_format_uint16, \
    .min_value=(float[])   {50}, \
    .max_value=(float[]) {2000}, \
    .min_step  = (float[]) {50}, \
    .permissions = homekit_permissions_paired_read \
                 | homekit_permissions_paired_write, \
    .value = HOMEKIT_UINT16_(_value), \
    ##__VA_ARGS__
    
void factor_set(homekit_value_t value); 
homekit_characteristic_t factor=HOMEKIT_CHARACTERISTIC_(CUSTOM_FACTOR, 600, .setter=factor_set);
void factor_set(homekit_value_t value) {
    UDPLUS("Factor: %d\n", value.int_value);
    ffactor=value.int_value;
    factor.value=value;
}

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

#define TEMP2HK(n)  do {old_t##n=cur_temp##n.value.float_value; \
                        cur_temp##n.value.float_value=isnan(temp[S##n])?S##n##avg:(float)(int)(temp[S##n]*10+0.5)/10; \
                        if (old_t##n!=cur_temp##n.value.float_value) \
                            homekit_characteristic_notify(&cur_temp##n,HOMEKIT_FLOAT(cur_temp##n.value.float_value)); \
                    } while (0) //TODO: do we need to test for changed values or is that embedded in notify routine?
#define BEAT 10 //in seconds
#define SENSORS 3
#define S1 0 //   salon temp sensor
#define S2 1 //upstairs temp sensor
#define S3 6 // outdoor temp sensor
#define BW 4 //boiler water temp
#define RW 5 //return water temp
#define DW 8 //domestic home water temp
float temp[16]={85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85}; //using id as a single hex digit, then hardcode which sensor gets which meaning
float S1temp[6],S2temp[6],S3temp[6],S1avg,S2avg,S3avg;
float S3total=0;
int   S3samples=0;
void temp_task(void *argv) {
    ds18b20_addr_t addrs[SENSORS];
    float temps[SENSORS];
    float old_t1,old_t2,old_t3;
    int sensor_count=0,id,fail=0;

    while( (sensor_count=ds18b20_scan_devices(SENSOR_PIN, addrs, SENSORS)) != SENSORS) {
        UDPLUS("Only found %d sensors\n",sensor_count);
        if (fail++>50) {
            printf("restarting because can't find enough sensors\n");
            mqtt_client_publish("{\"idx\":%d,\"nvalue\":4,\"svalue\":\"Heater No Sensors\"}", idx);
            vTaskDelay(3000/portTICK_PERIOD_MS);
            sdk_system_restart();  //#include <rboot-api.h>
        }
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
        if (!isnan(temp[S3]) && temp[S3]!=85) S3samples++,S3total+=temp[S3];
        else {gpio_write(SENSOR_PIN,0);vTaskDelay(150/portTICK_PERIOD_MS);gpio_write(SENSOR_PIN,1);} //reset one-wire bus
        TEMP2HK(1);
        TEMP2HK(2);
        TEMP2HK(3);
    }
}

#define RTC_ADDR    0x600013B0
#define RTC_MAGIC   0xaabecede
#define PAST_TGT_N  10
#define BOOSTLEVEL  25.0
enum    modes { STABLE, HEAT, EVAL };
int     mode=EVAL,peak_time=15; //after update, evaluate only 15 minutes
float   peak_temp=0,prev_setp=DEFAULT1,setpoint2=DEFAULT2,hystsetpoint2=DEFAULT2,heat_sp=35;
float   stable_tgt_temp1=DEFAULT1, past_tgt_temp1[PAST_TGT_N];
float   curr_mod=0,heat_mod=0,pressure=0;
time_t  heat_till=0;
int     time_set=0,time_on=0,stateflg=0,errorflg=0;
int     heat_on, boost=0;
int heater(uint32_t seconds) {
    if (!time_set) return 0; //need reliable time
    char str[26], strtm[32]; // e.g. DST0wd2yd4    5|07:02:00.060303
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now=tv.tv_sec;
    struct tm *tm = localtime(&now);

    if ( (tm->tm_hour==22 || tm->tm_hour==7) && S3samples>360 ) {
        float S3long=S3total/S3samples;
        PUBLISH(S3long);
        S3samples=0;S3total=0;
        PUBLISH(tgt_temp1); //to refresh the report to MQTT because this setpoint almost never changes
    }

    int eval_time=0,heater1=0,heater2=0;
    sprintf(strtm,"DST%dwd%dyd%-3d %2d|%02d:%02d:%02d.%06d",tm->tm_isdst,tm->tm_wday,tm->tm_yday,tm->tm_mday, \
            tm->tm_hour,tm->tm_min,tm->tm_sec,(int)tv.tv_usec);
    //heater2 logic
    if (tgt_temp2.value.float_value>setpoint2) setpoint2+=0.0625; //slow adjust up
    if (tgt_temp2.value.float_value<setpoint2) setpoint2-=0.0625; //slow adjust down
    if (setpoint2-S2avg < -0.125) setpoint2=S2avg-0.125; //prevent it takes very long to actually start heating after turn up
    if (setpoint2-S2avg>   0) hystsetpoint2=setpoint2+0.4; //below setpoint, try to overshoot
    if (setpoint2-S2avg<-0.1) hystsetpoint2=setpoint2;     //0.1 above the setup, wait till we drop below again
    if (hystsetpoint2-S2avg>0) {
        heat_sp=35+(hystsetpoint2-S2avg)*16; if (heat_sp>75) heat_sp=75;
        heater2=1;
    } else heat_sp=35;//request lowest possible output for floor heating while not heating radiators explicitly

    //heater1 logic
    float setpoint1=tgt_temp1.value.float_value;
    int   i,j,stable=0;
    if (setpoint1<BOOSTLEVEL) { //shift history left
        for (i=0;i<PAST_TGT_N-1;i++) past_tgt_temp1[i]=past_tgt_temp1[i+1];
        past_tgt_temp1[i]=setpoint1;
    }
    for (i=0; i<PAST_TGT_N; i++)     if (past_tgt_temp1[i]==stable_tgt_temp1) stable++;
    for (j=0; j<PAST_TGT_N; j++) {
        int newer=0;
        for (i=0; i<PAST_TGT_N; i++) if (past_tgt_temp1[i]==past_tgt_temp1[j]) newer++;
        if (newer>stable) {
            stable_tgt_temp1=past_tgt_temp1[j]; stable=newer;
        }
    }
    
    if (setpoint1<BOOSTLEVEL) boost=0; else if (heat_mod) boost++;
    if (boost>30) {
        setpoint1=tgt_temp1.value.float_value=stable_tgt_temp1;
        homekit_characteristic_notify(&tgt_temp1,HOMEKIT_FLOAT(tgt_temp1.value.float_value));
        PUBLISH(tgt_temp1); //to refresh the report to MQTT
    }
    if (setpoint1!=prev_setp) {
        if (setpoint1>prev_setp) mode=STABLE; else mode=EVAL;
        prev_setp=setpoint1;
    }
    if (mode==EVAL) {
        eval_time=((setpoint1-peak_temp)>0.06) ? 4/(setpoint1-peak_temp) : 64 ; //max eval time will be 64 minutes
        if (S1avg<(peak_temp-0.07) || peak_time++>=eval_time || S1avg>=setpoint1) {
            mode=STABLE;
            //adjust ffactor
            peak_temp=0,peak_time=0;
        } else if (peak_temp<S1avg) {
            peak_temp=S1avg;
            peak_time=0;
        }
        if (S1avg<peak_temp) peak_time++; //double fast countdown if S1Avg temp < peak temp
    }
    if (mode==STABLE) {
        if (tm->tm_hour<7 || tm->tm_hour>=22) { //night time preparing for morning warmup
            time_on=(ffactor*(setpoint1-S1avg));
            heat_till=now+(time_on*60)-2;               // -2 makes switch off moment more logical
            struct tm *seven02 = localtime(&now);       // some bizar leaking of values between tm and seven02
            if (tm->tm_hour>=22) seven02->tm_mday++;                  // 7:02 AM is tomorrow
            seven02->tm_hour=7; seven02->tm_min=2; seven02->tm_sec=0; //  :02 makes transition for heater 2 better
            if (heat_till>mktime(seven02)) mode=HEAT;
        } else { //daytime control
            time_on=(ffactor*(setpoint1-S1avg)*0.3);
            heat_till=now+(time_on*60)-2;
            if (time_on>5) mode=HEAT; //5 minutes at least, else too quick and allows fixes of 1/16th degree C
        }
    }
    if (mode==HEAT) {
        if (now>heat_till) {
            time_on=0;
            mode=EVAL;
            peak_temp=S1avg;
            peak_time=0;
        } else if ( setpoint1-S1avg>0 ){
            if (heat_mod) time_on--; else heat_till+=60; //only when burner on, count progress else shift end time
            heater1=1;
        } else {
            mode=STABLE;
        }
    }
    
    //integrated logic for both heaters
    int result=0; if (heater1) result=1; else if (heater2) result=2; //we must inhibit floor heater pump

    //final report
    ctime_r(&heat_till,str);str[16]=0; str[5]=str[10]=' ';str[6]='t';str[7]='i';str[8]=str[9]='l'; // " till hh:mm"
    if (time_on<0) time_on=0;
    printf("S1=%7.4f S2=%7.4f S3=%7.4f f=%6.1f time-on=%3d peak_temp=%7.4f peak_time=%2d<%2d ST=%02x mode=%d%s\n", \
            S1avg,S2avg,S3avg,ffactor,time_on,peak_temp,peak_time,eval_time,stateflg,mode,(mode==1)?(str+5):"");
    printf("Heater@%-4d  stable:%4.1f %s   %s => heat_sp:%4.1f h1:%d + h2:%d = on:%d\n", \
            (seconds+10)/60,stable_tgt_temp1,boost?"boost":"     ",strtm,heat_sp,heater1,heater2,result);
    PUBLISH(S1avg);
    PUBLISH(S2avg);
    PUBLISH(heat_mod);
    PUBLISH(heat_sp);
    PUBLISH(burnerW);
    PUBLISH(returnW);
    PUBLISH(pressure);
    PUBLISH(S3avg);
    
    //save state to RTC memory
    uint32_t *dp;         WRITE_PERI_REG(RTC_ADDR+ 4,mode     ); //int
                          WRITE_PERI_REG(RTC_ADDR+ 8,heat_till); //time_t
    dp=(void*)&ffactor;   WRITE_PERI_REG(RTC_ADDR+12,*dp      ); //float
    dp=(void*)&prev_setp; WRITE_PERI_REG(RTC_ADDR+16,*dp      ); //float
    dp=(void*)&peak_temp; WRITE_PERI_REG(RTC_ADDR+20,*dp      ); //float
                          WRITE_PERI_REG(RTC_ADDR+24,peak_time); //int
    dp=(void*)&stable_tgt_temp1;            WRITE_PERI_REG(RTC_ADDR+28,*dp      ); //float
    dp=(void*)&tgt_temp2.value.float_value; WRITE_PERI_REG(RTC_ADDR+32,*dp      ); //float
    dp=(void*)&setpoint2; WRITE_PERI_REG(RTC_ADDR+36,*dp      ); //float
    dp=(void*)&S3total;   WRITE_PERI_REG(RTC_ADDR+40,*dp      ); //float
                          WRITE_PERI_REG(RTC_ADDR+44,S3samples); //int
                          WRITE_PERI_REG(RTC_ADDR   ,RTC_MAGIC);
    return result;
}

void init_task(void *argv) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    printf("RTC: "); for (int i=0;i<7;i++) printf("%08x ",READ_PERI_REG(RTC_ADDR+i*4)); printf("\n");
    uint32_t *dp;
	if (READ_PERI_REG(RTC_ADDR)==RTC_MAGIC) {
	    mode                    =READ_PERI_REG(RTC_ADDR+ 4);
        heat_till               =READ_PERI_REG(RTC_ADDR+ 8);
        dp=(void*)&ffactor;  *dp=READ_PERI_REG(RTC_ADDR+12); factor.value.int_value=ffactor;
        dp=(void*)&prev_setp;*dp=READ_PERI_REG(RTC_ADDR+16);
        dp=(void*)&peak_temp;*dp=READ_PERI_REG(RTC_ADDR+20);
        peak_time               =READ_PERI_REG(RTC_ADDR+24);
        dp=(void*)&(tgt_temp1.value.float_value);*dp=READ_PERI_REG(RTC_ADDR+28);
        dp=(void*)&(tgt_temp2.value.float_value);*dp=READ_PERI_REG(RTC_ADDR+32);
        dp=(void*)&setpoint2;*dp=READ_PERI_REG(RTC_ADDR+36);
        dp=(void*)&S3total;  *dp=READ_PERI_REG(RTC_ADDR+40);
        S3samples               =READ_PERI_REG(RTC_ADDR+44);
    }
    printf("INITIAL prev_setp=%2.1f f=%2.1f peak_time=%2d peak_temp=%2.4f mode=%d heat_till %s", \
            prev_setp,ffactor,peak_time,peak_temp,mode,ctime(&heat_till));
    PUBLISH(tgt_temp1);
    PUBLISH(tgt_temp2);
    for (int i=0; i<PAST_TGT_N; i++) past_tgt_temp1[i]=tgt_temp1.value.float_value;
    S1temp[0]=22;S2temp[0]=22;
    
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); tzset();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "0.pool.ntp.org");
    sntp_setservername(1, "1.pool.ntp.org");
    sntp_setservername(2, "2.pool.ntp.org");
    while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) vTaskDelay(200/portTICK_PERIOD_MS); //Check if we have an IP every 200ms
    sntp_init();
    time_t ts;
    do {ts = time(NULL);
        if (ts == ((time_t)-1)) printf("ts=-1 ");
        vTaskDelay(100/portTICK_PERIOD_MS);
    } while (!(ts>1608567890)); //Mon Dec 21 17:24:50 CET 2020
    printf("TIME SET: %u=%s", (unsigned int) ts, ctime(&ts));
    time_set=1;
    vTaskDelete(NULL);
}

#define CalcAvg(Sx) do {            Sx##temp[5]=Sx##temp[4];Sx##temp[4]=Sx##temp[3]; \
            Sx##temp[3]=Sx##temp[2];Sx##temp[2]=Sx##temp[1];Sx##temp[1]=Sx##temp[0]; \
            if ( !isnan(temp[Sx]) && temp[Sx]!=85 )         Sx##temp[0]=temp[Sx];    \
            Sx##avg=(Sx##temp[0]+Sx##temp[1]+Sx##temp[2]+Sx##temp[3]+Sx##temp[4]+Sx##temp[5])/6.0; \
        } while(0)
static TaskHandle_t tempTask = NULL;
int timeIndex=0,switch_state=0,pump_off_time=0,retrigger=0;
int push=-2;
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
    if (timeIndex==3) { // allow 3 seconds for two automation rules to succeed and repeat every 10 seconds
        if (pump_off_time) pump_off_time-=10;
        if (tgt_heat1.value.int_value==2) { //Pump Off rule confirmed
            cur_heat2.value.int_value= 1;   //confirm we are heating upstairs
            homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
            tgt_heat1.value.int_value= 3;   //set heater 1 mode back to auto and be ready for another trigger
            homekit_characteristic_notify(&tgt_heat1,HOMEKIT_UINT8(tgt_heat1.value.int_value)); //TODO: racecondition?
            pump_off_time=180; //seconds
            heat_on=1;
        }
        if (cur_heat2.value.int_value==2) {//send reminder notify
            cur_heat2.value.int_value= 0; //to assure it is considered as a new value we first set it to off
            homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value)); //and notify
            retrigger=1;
            if (pump_off_time>10) heat_on=1; //still time left
        }
    }
    if (retrigger && timeIndex==8) { retrigger=0; //retrigger needed 5 seconds offset
        cur_heat2.value.int_value= 2;
        homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
    }
    switch (timeIndex) { //send commands
        case 0: //measure temperature
            xTaskNotifyGive( tempTask ); //temperature measurement start
            vTaskDelay(1); //prevent interference between OneWire and OT-receiver
            send_OT_frame(0x00190000); //25 read boiler water temperature
            break;
        case 1: //execute heater decisions
            if (tgt_heat2.value.int_value==1) { //use on/off switching thermostat
                   message=0x10014b00; //75 deg
            } else if (tgt_heat2.value.int_value==3) { //run heater algoritm for floor heating
                   message=0x10010000|(uint32_t)(heat_sp*256);
            } else message=0x10010000|(uint32_t)(tgt_temp1.value.float_value*2-1)*256; //range from 19 - 75 deg
            send_OT_frame(message); //1  CH setpoint in deg C
            break;
        case 2:
            if (tgt_heat2.value.int_value==1) { //use on/off switching thermostat
                if (tgt_temp2.value.float_value>17) message=0x100e0000|(uint32_t)(4*tgt_temp2.value.float_value-52)*256; //18-100%
                else message=0x100e1100; //17%
            } else   message=0x100e6400; //100%
            send_OT_frame( message ); //14 max modulation level
            break;
        case 3:
            if (tgt_heat2.value.int_value==1) { //use on/off switching thermostat
                   message=0x00000200|(switch_on?0x100:0x000);
            } else if (tgt_heat2.value.int_value==3) { //run heater algoritm for floor heating
                   message=0x00000200|(  heat_on?0x100:0x000);
            } else message=0x00000200|(tgt_heat1.value.int_value<<8);
            send_OT_frame( message ); //0  enable CH and DHW
            break; 
        case 4:
            if (tgt_heat2.value.int_value==2) { //test BLOR
                   send_OT_frame( 0x10040100 ); //4.1  BoilerLockOutReset
            } else send_OT_frame( 0x00380000 ); //56 DHW setpoint write
            break;
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
                heat_mod=0.0;
                stateflg=(message&0x0000007f);
                if ((stateflg&0xa)==0xa) {
                    heat_mod=1.0; //at this point it is a multiplier
                    cur_heat1.value.int_value=pump_off_time?2:1; //present heater on but pump off as cur_heat1=2 COOL
                } else cur_heat1.value.int_value=0;
                homekit_characteristic_notify(&cur_heat1,HOMEKIT_UINT8(cur_heat1.value.int_value));
                break;
            case 5: errorflg=       (message&0x00003f00)/256; break;
            case 6: pressure=(float)(message&0x0000ffff)/256; break;
            case 7: temp[DW]=(float)(message&0x0000ffff)/256; break;
            case 8: temp[RW]=(float)(message&0x0000ffff)/256; break;
            case 9:
                curr_mod=(float)(message&0x0000ffff)/256;
                if (curr_mod==70.0) heat_mod=0.0; else heat_mod*=curr_mod;
                break;
            default: break;
        }
    } else {
        printf("!!! NO_RSP: resp_idx=%d rx_state=%d response=%08x\n",resp_idx, rx_state, response);
        resp_idx=0, rx_state=READY, response=0;
    }
    
    if (!timeIndex) {
        CalcAvg(S1); CalcAvg(S2); CalcAvg(S3);
    }
    
    //errorflg=(seconds/600)%2; //test trick to change outcome every 10 minutes
    if (seconds%60==5) {
        if (errorflg) { //publish a ORANGE (3) ALERT on domoticz
            if (push>0) {
                int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":3,\"svalue\":\"Heater ERR: 0x%02X\"}", idx, errorflg);
                if (n<0) printf("MQTT publish of ALERT failed because %s\n",MQTT_CLIENT_ERROR(n)); else push--;
                if (push==0) push=-2;
            }
        } else { //publish a GREY (0) clean ALERT on domoticz
            if (push<0) {
                int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"Heater OK (again) %d\"}", idx, seconds/60);
                if (n<0) printf("MQTT publish of ALERT failed because %s\n",MQTT_CLIENT_ERROR(n)); else push++;
                if (push==0) push=3;            
            }
        }
    }
//  if (seconds%3600==3599) { //force a alive report every hour
//      int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"Heater alive %d\"}", idx, seconds/60);    
//  }
//  
    if (seconds%60==50) { //allow 6 temperature measurments to make sure all info is loaded
        heat_on=0;
        cur_heat2.value.int_value=heater(seconds); //sets heat_sp and returns heater result, 0, 1 or 2
        if (cur_heat2.value.int_value==2 && pump_off_time>90) cur_heat2.value.int_value=1; //do not retrigger rules yet
        if (cur_heat2.value.int_value==1) heat_on=1;
        homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
    }

    if (timeIndex==3) {
        printf("S1=%7.4f S2=%7.4f S3=%7.4f PR=%4.2f DW=%4.1f ERR=%02x RW=%4.1f BW=%4.1f POT=%3d ON=%d MOD=%02.0f ST=%02x\n", \
           temp[S1],temp[S2],temp[S3],pressure,temp[DW],errorflg,temp[RW],temp[BW],pump_off_time,heat_on,curr_mod,stateflg);
    }

    timeIndex++; if (timeIndex==BEAT) timeIndex=0;
} //this is a timer that restarts every 1 second

void ping_task(void *argv) {
    int count=120,delay=1; //seconds
    ping_result_t res;
    ip_addr_t to_ping;
    inet_aton(pinger_target,&to_ping);
    
    printf("Pinging IP %s\n", ipaddr_ntoa(&to_ping));
    while(1){
        ping_ip(to_ping, &res);

        if (res.result_code == PING_RES_ECHO_REPLY) {
            count+=20; delay+=5;
            if (count>300) count=300; if (delay>60) delay=60;
            printf("good ping from %s %u ms -> count: %d s\n", ipaddr_ntoa(&res.response_ip), res.response_time_ms, count);
        } else {
            count--; delay=1;
            printf("failed ping err %d -> count: %d s\n", res.result_code, count);
        }
        if (count==0) {
            printf("restarting because can't ping home-hub\n");
            mqtt_client_publish("{\"idx\":%d,\"nvalue\":2,\"svalue\":\"Heater No Ping\"}", idx);
            vTaskDelay(3000/portTICK_PERIOD_MS);
            sdk_system_restart();  //#include <rboot-api.h>
        }
        vTaskDelay(delay*(1000/portTICK_PERIOD_MS));
    }
}

mqtt_config_t mqttconf=MQTT_DEFAULT_CONFIG;
char error[]="error";
static void ota_string() {
    char *dmtczbaseidx1=NULL;
    char *otas;
    if (sysparam_get_string("ota_string", &otas) == SYSPARAM_OK) {
        mqttconf.host=strtok(otas,";");
        mqttconf.user=strtok(NULL,";");
        mqttconf.pass=strtok(NULL,";");
        dmtczbaseidx1=strtok(NULL,";");
        //pinger_target=strtok(NULL,";");
    }
    if (mqttconf.host==NULL) mqttconf.host=error;
    if (mqttconf.user==NULL) mqttconf.user=error;
    if (mqttconf.pass==NULL) mqttconf.pass=error;
    if (dmtczbaseidx1==NULL) idx=1000; else idx=atoi(dmtczbaseidx1);
    //if (pinger_target==NULL) pinger_target=error;
    pinger_target="192.168.178.100";
}

homekit_server_config_t config;
void device_init() {
  if (homekit_is_paired()) {
    config.on_event=NULL;
    udplog_init(3);
    UDPLUS("\n\n\nDual-Heater-OpenTherm " VERSION "\n");
    
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

    //sysparam_set_string("ota_string", "192.168.178.5;heater;fakepassword;68"); //can be used if not using LCM
    ota_string();
    mqttconf.queue_size=20;
    mqttconf.msg_len   =64; //to fit the ALERT
    mqtt_client_init(&mqttconf);
    xQueue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(temp_task,"Temp", 512, NULL, 1, &tempTask);
    xTaskCreate(init_task,"Time", 512, NULL, 6, NULL);
    xTaskCreate(ping_task,"PingT",512, NULL, 1, NULL);
    xTimer=xTimerCreate( "Timer", 1000/portTICK_PERIOD_MS, pdTRUE, (void*)0, vTimerCallback);
    xTimerStart(xTimer, 0);
  }
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
                    HOMEKIT_CHARACTERISTIC(NAME, "Outdoor T"),
                    &cur_temp3,
                    &ota_trigger,
                    &factor,
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
    .on_event=device_init,
    .password = "111-11-111"
};


void user_init(void) {
    uart_set_baud(0, 115200);

    device_init();
    
    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
    //c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;
    
    homekit_server_init(&config);
}
