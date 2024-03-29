//inspired by https://github.com/maxgerhardt/esp-open-rtos-ping-example but needed serious fixing
#ifndef LWIP_PING_H
#define LWIP_PING_H

#include "lwip/ip_addr.h"

typedef enum {
    PING_RES_NO_MEM,                 /* internal memory alloc failure */
    PING_RES_ERR_SENDING,            /* socket could not send */
    PING_RES_ERR_NO_SOCKET,          /* socket could not be created */
    PING_RES_TIMEOUT,                /* no response received in time */
    PING_RES_ID_OR_SEQNUM_MISMATCH,  /* response ID or sequence number mismatched */
    PING_RES_ECHO_REPLY,             /* ping answer received */
    PING_RES_DESTINATION_UNREACHABLE,/* destination unreachable received */
    PING_RES_TIME_EXCEEDED,          /* for TTL to low or time during defrag exceeded (see wiki) */
    PING_RES_UNHANDLED_ICMP_CODE,    /* for all ICMP types which are not specifically handled */
    PING_RES_INITIAL_VALUE_UNCHANGED /* if the code never changed from the start */
} ping_result_code;

typedef struct {
    ping_result_code result_code;
    u32_t response_time_ms;
    ip_addr_t response_ip;
} ping_result_t;

void ping_ip(ip_addr_t ping_addr, ping_result_t *res);

#endif /* LWIP_PING_H */
