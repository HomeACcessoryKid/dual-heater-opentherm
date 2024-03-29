#include <string.h>
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/raw.h"
#include "lwip/icmp.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/inet_chksum.h"
#include "lwip/prot/ip4.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "ping.h"

#ifndef LWIP_RAW
#error "LWIP_RAW must be activated in lwipopts.h"
#endif//LWIP_RAW

#ifndef PING_RCV_TIMEO //ping receive timeout - in milliseconds
#define PING_RCV_TIMEO 1000
#endif
#ifndef PING_ID //ping identifier - must fit on a u16_t
#define PING_ID        0xAFAF
#endif
#define PING_DATA_SIZE 32

static u16_t ping_seq_num;

//Prepare a echo ICMP request
static void ping_prepare_echo(struct icmp_echo_hdr *iecho, u16_t len) {
    size_t i;
    size_t data_len = len - sizeof(struct icmp_echo_hdr);

    ICMPH_TYPE_SET(iecho, ICMP_ECHO);
    ICMPH_CODE_SET(iecho, 0);
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = lwip_htons(++ping_seq_num);
    for (i = 0; i < data_len; i++) { //fill the additional data buffer with some data
        ((char*) iecho)[sizeof(struct icmp_echo_hdr) + i] = (char) i;
    }
    iecho->chksum = inet_chksum(iecho, len);
}

static err_t ping_send(int sock, const ip_addr_t *addr) { //Ping using the socket ip
    int err;
    struct icmp_echo_hdr *iecho;
    struct sockaddr_storage to;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + PING_DATA_SIZE;

    iecho = (struct icmp_echo_hdr*) mem_malloc((mem_size_t) ping_size);
    if (!iecho) {
        return ERR_MEM;
    }

    ping_prepare_echo(iecho, (u16_t) ping_size);

    if (IP_IS_V4(addr)) {
        struct sockaddr_in *to4 = (struct sockaddr_in*) &to;
        to4->sin_len = sizeof(to4);
        to4->sin_family = AF_INET;
        inet_addr_from_ip4addr(&to4->sin_addr, ip_2_ip4(addr));
    }

    err = lwip_sendto(sock, iecho, ping_size, 0, (struct sockaddr*) &to,
            sizeof(to));

    mem_free(iecho);

    return (err ? ERR_OK : ERR_VAL);
}

static void ping_recv(int sock, ping_result_t *res) {
    char buf[64];
    int len;
    struct sockaddr_storage from;
    int fromlen = sizeof(from);

    while ((len = lwip_recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*) &from, (socklen_t*) &fromlen)) > 0) {
        if (len >= (int) (sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr))) { //received something usefull
            ip_addr_t fromaddr;
            memset(&fromaddr, 0, sizeof(fromaddr));

            if (from.ss_family == AF_INET) {
                struct sockaddr_in *from4 = (struct sockaddr_in*) &from;
                inet_addr_to_ip4addr(ip_2_ip4(&fromaddr), &from4->sin_addr);IP_SET_TYPE_VAL(fromaddr, IPADDR_TYPE_V4);
            }

            res->response_time_ms = sys_now() - res->response_time_ms;
            res->response_ip = fromaddr;
            //printf("ping: recv %s %u ms\n", ipaddr_ntoa(&fromaddr), res->response_time_ms);

            if (IP_IS_V4_VAL(fromaddr)) {
                struct ip_hdr *iphdr;
                struct icmp_echo_hdr *iecho;

                iphdr = (struct ip_hdr*) buf;
                iecho = (struct icmp_echo_hdr*) (buf + (IPH_HL(iphdr) * 4));
                if ((iecho->id == PING_ID) && (iecho->seqno == lwip_htons(ping_seq_num))) {
                    switch (ICMPH_TYPE(iecho)) { //do some ping result processing
                    case ICMP_ER:
                        res->result_code = PING_RES_ECHO_REPLY;
                        break;
                    case ICMP_DUR:
                        res->result_code = PING_RES_DESTINATION_UNREACHABLE;
                        break;
                    case ICMP_TE:
                        res->result_code = PING_RES_TIME_EXCEEDED;
                        //iecho->code has more info
                        break;
                    default:
                        res->result_code = PING_RES_UNHANDLED_ICMP_CODE;
                        break;
                    }
                    return;
                } else {
                    res->result_code = PING_RES_ID_OR_SEQNUM_MISMATCH;
                }
            }
        }
        fromlen = sizeof(from);
    }

    if (len == -1) { //timeout (should verify if error==11)
        res->response_time_ms = UINT32_MAX;
        res->result_code = PING_RES_TIMEOUT;
    }
}

void ping_ip(ip_addr_t ping_target, ping_result_t *res) {
    int sock;
    int ret;
    err_t err;

    if (res == NULL) {
        return;
    }
    res->result_code = PING_RES_INITIAL_VALUE_UNCHANGED;
    res->response_time_ms=0;
    inet_aton("0.0.0.0",&res->response_ip);
    
    struct timeval timeout;
    timeout.tv_sec = PING_RCV_TIMEO / 1000;
    timeout.tv_usec = (PING_RCV_TIMEO % 1000) * 1000;

    fflush(stdout);
    sock = lwip_socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);

    fflush(stdout);

    if (sock < 0) {
        res->result_code = PING_RES_ERR_NO_SOCKET;
        return;
    }

    ret = lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    LWIP_ASSERT("setting receive timeout failed", ret == 0);
    LWIP_UNUSED_ARG(ret);

    if ((err = ping_send(sock, &ping_target)) == ERR_OK) {
        //modified later
        res->response_time_ms = sys_now();
        ping_recv(sock, res);
    } else {
        if (err == ERR_VAL) {
            res->result_code = PING_RES_ERR_SENDING;
        } else if (err == ERR_MEM) {
            res->result_code = PING_RES_NO_MEM;
        }
    }
    lwip_close(sock);
}
