#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "project-conf.h"

#include "dev/gpio.h"
#include "sys/rtimer.h"
#include "sys/energest.h"
#include "arch/cpu/cc2538/lpm.h"

linkaddr_t sink_addr = {{0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9d, 0x89}};
#define RUN_INTERVAL 150*CLOCK_SECOND

PROCESS(observed_process, "Observed process");
AUTOSTART_PROCESSES(&observed_process);

PROCESS_THREAD(observed_process, ev, data)
{
    static struct etimer periodic;
    static struct observedmsg msg;
    static struct observedmsg init_vals;
    msg.cpu = 0;
    msg.lpm = 0;
    //msg.deep_lpm = 0;
    msg.total = 0;
    msg.rx = 0;
    msg.tx = 0;
    PROCESS_BEGIN();
    NETSTACK_RADIO.off();
    NETSTACK_MAC.off();

    GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));
    GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));

    etimer_set(&periodic, 2*CLOCK_SECOND);
    //printf("starting\n");

    GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
    etimer_reset_with_new_interval(&periodic, RUN_INTERVAL);
    //printf("entering lpm\n");
    energest_flush();
    init_vals.cpu = energest_type_time(ENERGEST_TYPE_CPU);
    init_vals.lpm = energest_type_time(ENERGEST_TYPE_LPM);
    //init_vals.deep_lpm = energest_type_time(ENERGEST_TYPE_DEEP_LPM);
    init_vals.total = energest_get_total_time();
    init_vals.rx = energest_type_time(ENERGEST_TYPE_LISTEN);
    init_vals.tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    lpm_enter();
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic));

    lpm_exit();
    //printf("exiting lpm\n");
    energest_flush();
    GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
    msg.cpu = energest_type_time(ENERGEST_TYPE_CPU);
    msg.lpm = energest_type_time(ENERGEST_TYPE_LPM);
    //msg.deep_lpm = energest_type_time(ENERGEST_TYPE_DEEP_LPM);
    msg.total = energest_get_total_time();
    msg.rx = energest_type_time(ENERGEST_TYPE_LISTEN);
    msg.tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);

    msg.cpu -= init_vals.cpu;
    msg.lpm -= init_vals.lpm;
    //msg.deep_lpm -= init_vals.deep_lpm;
    msg.total -= init_vals.total;
    msg.rx -= init_vals.rx;
    msg.tx -= init_vals.tx;

    NETSTACK_RADIO.on();
    NETSTACK_MAC.on();
    
    //printf("sending to sink\n");
    nullnet_buf = (uint8_t *) &msg;
    
    nullnet_len = sizeof(msg);
    NETSTACK_NETWORK.output(&sink_addr);
    //GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));

    PROCESS_END();
}