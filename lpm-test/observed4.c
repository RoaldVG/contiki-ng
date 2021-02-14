#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "project-conf.h"

#include "dev/gpio.h"
#include "sys/rtimer.h"
#include "sys/energest.h"
#include "arch/cpu/cc2538/lpm.h"
#include "lib/random.h"

#include "net/mac/tsch/tsch.h"

linkaddr_t sink_addr = {{0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9d, 0x89}};
#define RUN_INTERVAL  600 //seconds
#define TX_PER_SEC    32
#define LIMIT         RUN_INTERVAL*TX_PER_SEC

//#include "net/mac/tsch/tsch.h"

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
    static int state = 1;
    static int c = 0;
    msg.cpu = 0;
    msg.lpm = 0;
    msg.total = 0;
    msg.rx = 0;
    msg.tx = 0;
    PROCESS_BEGIN();
    //NETSTACK_RADIO.off();
    //NETSTACK_RADIO.on();
    //NETSTACK_MAC.off();

    GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));
    GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));

#if MAC_CONF_WITH_TSCH
  tsch_set_coordinator(linkaddr_cmp(&sink_addr, &linkaddr_node_addr));
#endif /* MAC_CONF_WITH_TSCH */
    etimer_set(&periodic, 3*CLOCK_SECOND);
    //printf("starting\n");

    GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
    
    //printf("starting timer\n");
    energest_flush();
    init_vals.cpu = energest_type_time(ENERGEST_TYPE_CPU);
    init_vals.lpm = energest_type_time(ENERGEST_TYPE_LPM);
    init_vals.total = RTIMER_NOW();
    init_vals.rx = energest_type_time(ENERGEST_TYPE_LISTEN);
    init_vals.tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
#if MAC_CONF_WITH_TSCH
    etimer_set(&periodic, CLOCK_SECOND*2);
    while(!tsch_is_associated)
    {
        PROCESS_YIELD();
        etimer_reset(&periodic);
        //printf("not yet associated\n");
    }
#endif /* MAC_CONF_WITH_TSCH */

    etimer_set(&periodic, CLOCK_SECOND/TX_PER_SEC);
    //printf("lim:%d\n",LIMIT);
    while(state)
    {
        c++;
        nullnet_buf = (uint8_t *) &c;
	      nullnet_len = sizeof(c);
        NETSTACK_NETWORK.output(NULL);
        if (c>=LIMIT)
          state=0;
        //if (c%100==0) printf("%d\n",c);
        etimer_reset(&periodic);
        NETSTACK_RADIO.off();
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic));        
    }


    msg.total = RTIMER_NOW();
    energest_flush();
    GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
    //printf("%lld\n",energest_type_time(ENERGEST_TYPE_TRANSMIT));
    msg.cpu = (uint32_t) energest_type_time(ENERGEST_TYPE_CPU);
    msg.lpm = (uint32_t) energest_type_time(ENERGEST_TYPE_LPM);
    msg.rx = energest_type_time(ENERGEST_TYPE_LISTEN);
    //printf("%ld\n",msg.tx);
    msg.tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    msg.cpu-=init_vals.cpu;
    msg.lpm-=init_vals.lpm;
    msg.total -= init_vals.total;
    msg.rx -= init_vals.rx;
    msg.tx -= init_vals.tx;

    NETSTACK_RADIO.on();
    NETSTACK_MAC.on();
    
    //printf("sending to sink. %ld\n", msg.tx);
    nullnet_buf = (uint8_t *) &msg;
    
    nullnet_len = sizeof(msg);
    NETSTACK_NETWORK.output(&sink_addr);
    //GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));

    PROCESS_END();
}