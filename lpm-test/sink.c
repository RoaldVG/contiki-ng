#include "contiki.h"
#include "project-conf.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include <inttypes.h>
#include <stdio.h>
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#include "net/mac/tsch/tsch.h"

linkaddr_t sink_addr = {{0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9d, 0x89}};

PROCESS(sink_process, "sink process");
AUTOSTART_PROCESSES(&sink_process);
struct observedmsg msg_o;
void input_callback(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest)
{
    struct monitormsg msg_m;
    struct observedmsg msg_o;
    if (len == sizeof(msg_m))
    {
        memcpy(&msg_m, data, len);
        printf("%x%x,%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n\r",
                src->u8[sizeof(src->u8) - 2], src->u8[sizeof(src->u8) - 1],
                msg_m.counter, msg_m.power, msg_m.total);
    }
    else if (len == sizeof(msg_o))
    {
        memcpy(&msg_o, data, sizeof(msg_o));
        printf("%x%x,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n\r",//,%" PRIu32 "\n\r",
                src->u8[sizeof(src->u8) - 2], src->u8[sizeof(src->u8) - 1],
                msg_o.cpu, msg_o.lpm, msg_o.tx, msg_o.rx, msg_o.total);//, msg_o.deep_lpm, msg_o.total, msg_o.tx, msg_o.rx);
    }
}

PROCESS_THREAD(sink_process, ev, data)
{
    PROCESS_BEGIN();

    #if MAC_CONF_WITH_TSCH
  tsch_set_coordinator(linkaddr_cmp(&sink_addr, &linkaddr_node_addr));
#endif /* MAC_CONF_WITH_TSCH */
    msg_o.cpu=0;
    nullnet_set_input_callback(input_callback);
    while(1) {
        PROCESS_YIELD();
    }
    PROCESS_END();
}