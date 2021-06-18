/**
 * \file
 *          Sink to receive energest data from observed motes
 *          Accepts messages from everybody
 * \author
 *         Marie-Paule Uwase
 *         August 13, 2012
 *         Roald Van Glabbeek
 *         March 3, 2020
 * 
 *         Updated for newer contiki release en Zolertia Zoul (firefly) and IPv6
 */
#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "project-conf.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sys/etimer.h"

#if MAC_CONF_WITH_TSCH
#include "net/mac/tsch/tsch.h"
#endif /* MAC_CONF_WITH_TSCH */

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

// sender power
// possible values =  0dBm = 31;  -1dBm = 27;  -3dBm = 23;  -5dBm = 19; 
//                   -7dBm = 15; -10dBm = 11; -15dBm =  7; -25dBM =  3; 
uint8_t power = 31;

// message counters
static uint16_t received = 0 ;

const linkaddr_t energest_addr = {{ 0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9d, 0x89 }};

PROCESS(energest_sink_process, "Energest sink process");
AUTOSTART_PROCESSES(&energest_sink_process);
// This is the receiver function
/*---------------------------------------------------------------------------*/
void input_callback(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest)
{
    if(len == sizeof(struct energestmsg)) {
        //rtimer_clock_t rtime = RTIMER_NOW();        //received time (for the latency)
        struct energestmsg msg;
        memcpy(&msg, data, len);
        received ++;
        //uint16_t timestamp = packetbuf_attr(PACKETBUF_ATTR_TIMESTAMP);        

        printf("%x%x,%" PRIu16 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32"\n\r",
                src->u8[sizeof(src->u8) - 2], src->u8[sizeof(src->u8) - 1], msg.seqno, 
                msg.cpu, msg.lpm, msg.transmit, msg.listen, msg.totaltime);
    }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(energest_sink_process, ev, data)
{

    PROCESS_BEGIN();

#if MAC_CONF_WITH_TSCH
    tsch_set_coordinator(1);
#endif /* MAC_CONF_WITH_TSCH */

    nullnet_set_input_callback(input_callback);


#if !UART_CONF_ENABLE
#error "No logging possible, set UART_CONF_ENABLE to 1"
#endif /* UART_CONF_ENABLE */

    while(1) {
        PROCESS_YIELD();
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/

