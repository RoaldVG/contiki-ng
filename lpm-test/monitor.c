#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "project-conf.h"

#include "dev/gpio.h"
#include "dev/zoul-sensors.h"
#include "dev/adc-zoul.h"
#include "sys/rtimer.h"

#include "net/mac/tsch/tsch.h"

#define ADC_READ_INTERVAL (CLOCK_SECOND/56)

linkaddr_t sink_addr = {{0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9d, 0x89}};


PROCESS(monitor_process, "Monitor process");
AUTOSTART_PROCESSES(&monitor_process);

static uint32_t counter=0;
static uint64_t ADC_result=0;
static uint32_t measure_time = 0;

/*---------------------------------------------------------------------------*/
static void
send_packet()
{
    static struct monitormsg msg;
    msg.counter = counter;
    msg.total = RTIMER_NOW() - measure_time;
    msg.power = ADC_result;

    nullnet_buf = (uint8_t *) &msg;
    nullnet_len = sizeof(msg);
    NETSTACK_NETWORK.output(&sink_addr);
}
/*---------------------------------------------------------------------------*/
int phase = 0;  // 1 during "prep phase", 2 while running
int prev_state = 0;
PROCESS_THREAD(monitor_process, ev, data)
{
    static struct etimer periodic;

    PROCESS_BEGIN();

        #if MAC_CONF_WITH_TSCH
  tsch_set_coordinator(linkaddr_cmp(&sink_addr, &linkaddr_node_addr));
#endif /* MAC_CONF_WITH_TSCH */

    adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC2);
    counter=0;
    //printf("initializing\n");
    GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));
    etimer_set(&periodic, ADC_READ_INTERVAL);
    while(1){
        if(GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7))!=0)
        {
            if (!phase){// && !prev_state){
                phase = 1;
                printf("startup phase\n");
            }
            if (phase == 2 && !prev_state)
            {
                printf("sending packet and terminating\n");
                send_packet();
                PROCESS_EXIT();
            }
            prev_state = 1;
        }
        else
        {
            if (phase==1 && prev_state){
                phase = 2;
                measure_time = RTIMER_NOW();
                printf("entering measurement phase\n");
            }
            prev_state = 0;
        }
        if (phase == 2){
            counter++;
            int ADC_val = adc_zoul.value(ZOUL_SENSORS_ADC2);
            //printf("ADC: %d\n", ADC_val);
            ADC_result += ADC_val;
        }
        PROCESS_WAIT_UNTIL(etimer_expired(&periodic));
        etimer_reset(&periodic);
    }
    PROCESS_END();
}
