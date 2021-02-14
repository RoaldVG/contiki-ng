#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "project-conf.h"

#include "dev/gpio.h"
#include "sys/rtimer.h"
#include "sys/energest.h"
#include "arch/cpu/cc2538/lpm.h"
#include "lib/random.h"

linkaddr_t sink_addr = {{0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9d, 0x89}};
#define RUN_INTERVAL 10*CLOCK_SECOND

PROCESS(observed_process, "Observed process");
AUTOSTART_PROCESSES(&observed_process);

static inline unsigned long
to_mseconds(uint64_t time)
{
  return (unsigned long)(time / (ENERGEST_SECOND/1000));
}

PROCESS_THREAD(observed_process, ev, data)
{
    static struct etimer periodic;
    static struct observedmsg msg;
    static struct observedmsg init_vals;
    msg.cpu = 0;
    msg.lpm = 0;
    msg.deep_lpm = 0;
    msg.total = 0;
    msg.rx = 0;
    msg.tx = 0;
    PROCESS_BEGIN();
    NETSTACK_RADIO.off();
    NETSTACK_MAC.off();

    GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));
    GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));

    etimer_set(&periodic, 2*CLOCK_SECOND);
    printf("starting\n");

    static uint32_t c;
    static unsigned long time;

    GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
    etimer_set(&periodic, RUN_INTERVAL);
    printf("starting timer\n");
    energest_flush();
    init_vals.cpu = energest_type_time(ENERGEST_TYPE_CPU);
    init_vals.lpm = energest_type_time(ENERGEST_TYPE_LPM);
    init_vals.deep_lpm = energest_type_time(ENERGEST_TYPE_DEEP_LPM);
    init_vals.total = energest_get_total_time();
    init_vals.rx = energest_type_time(ENERGEST_TYPE_LISTEN);
    init_vals.tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    static int limit = 10*((1<<21)-(1<<19));
    time = RTIMER_NOW();
    while(c<limit){
        c++;
        //printf("%d\n",c);
    }
    time = RTIMER_NOW() - time;
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic));

    printf("timer expired %ld\n",c);
    printf("%lu\n",to_mseconds(time));

    energest_flush();
    GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
    msg.cpu = (uint32_t) energest_type_time(ENERGEST_TYPE_CPU);
    msg.lpm = (uint32_t) time;//energest_type_time(ENERGEST_TYPE_LPM);
    msg.deep_lpm = (uint32_t) energest_type_time(ENERGEST_TYPE_DEEP_LPM);
    //msg.total = to_mseconds(time);
    msg.rx = energest_type_time(ENERGEST_TYPE_LISTEN);
    msg.tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    msg.cpu-=init_vals.cpu;
    msg.lpm-=init_vals.lpm;
    msg.deep_lpm=init_vals.deep_lpm;
    msg.total=init_vals.total;
    msg.rx = init_vals.rx;
    msg.tx = init_vals.tx;

    NETSTACK_RADIO.on();
    NETSTACK_MAC.on();
    
    printf("sending to sink\n");
    nullnet_buf = (uint8_t *) &msg;
    
    nullnet_len = sizeof(msg);
    NETSTACK_NETWORK.output(&sink_addr);
    //GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));

    PROCESS_END();
}