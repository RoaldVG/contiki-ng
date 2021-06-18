/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Unicast receiving test program
 *         Accepts messages from everybody
 * \author
 *         Marie-Paule Uwase
 *         August 13, 2012
 *         Roald Van Glabbeek
 * 		     March 3, 2020
 * 
 *         Updated for newer contiki release en Zolertia Zoul (firefly) and IPv6
 */
#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "project-conf.h"

#include <stdio.h>
#include <string.h>

#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "sys/energest.h"
#include "dev/gpio.h"
#include "dev/ioc.h"

#if MAC_CONF_WITH_TSCH
#include "net/mac/tsch/tsch.h"
#endif /* MAC_CONF_WITH_TSCH */

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#if ENERGEST_CONF_ON
const linkaddr_t energest_addr = {{ 0x00, 0x12, 0x4b, 0x00, 0x14, 0xd5, 0x2d, 0xe1 }};
#endif

uint16_t seqno=0;
struct energestmsg prev_energest_vals;
/*---------------------------------------------------------------------------*/
PROCESS(observed_receiver_process, "Observed receiver process");
AUTOSTART_PROCESSES(&observed_receiver_process);
/*---------------------------------------------------------------------------*/
void
GPIOS_init(void)
{

    ioc_set_over(0, 6, IOC_OVERRIDE_OE);
    ioc_set_over(0, 7, IOC_OVERRIDE_OE);
    ioc_set_over(2, 0, IOC_OVERRIDE_OE);
    ioc_set_over(2, 1, IOC_OVERRIDE_OE);
    ioc_set_over(2, 2, IOC_OVERRIDE_OE);
    ioc_set_over(2, 3, IOC_OVERRIDE_OE);
    ioc_set_over(2, 4, IOC_OVERRIDE_OE);
    ioc_set_over(2, 5, IOC_OVERRIDE_OE);
    ioc_set_over(2, 6, IOC_OVERRIDE_OE);
    ioc_set_over(3, 0, IOC_OVERRIDE_OE);
    ioc_set_over(3, 1, IOC_OVERRIDE_OE);
    ioc_set_over(3, 2, IOC_OVERRIDE_OE);

    GPIO_SOFTWARE_CONTROL(GPIO_A_BASE,GPIO_PIN_MASK(6));
    GPIO_SOFTWARE_CONTROL(GPIO_A_BASE,GPIO_PIN_MASK(7));
	GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(6));		//GPIO PA6
	GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7

    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6
  
 	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
    GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
    GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
    GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6

    GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
    GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2

	GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
    GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2

    GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6));		//GPIO PA6
    
}
/*---------------------------------------------------------------------------*/
void
clear_GPIOS(void)
{
    GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6));		//GPIO PA6
	
 	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
  	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
  	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6

	GPIO_CLR_PIN(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
  	GPIO_CLR_PIN(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_CLR_PIN(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2
}
/*---------------------------------------------------------------------------*/
#if ENERGEST_CONF_ON
static void
send_energest()
{
    struct energestmsg energest_values;

    energest_values.totaltime = RTIMER_NOW() - prev_energest_vals.totaltime;

    energest_flush();
    energest_values.cpu = energest_type_time(ENERGEST_TYPE_CPU) - prev_energest_vals.cpu;
    energest_values.lpm = energest_type_time(ENERGEST_TYPE_LPM) - prev_energest_vals.lpm;
    energest_values.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) - prev_energest_vals.transmit;
    energest_values.listen = energest_type_time(ENERGEST_TYPE_LISTEN) - prev_energest_vals.listen;
    energest_values.seqno = seqno;

    LOG_INFO("Energest data sent to ");
	LOG_INFO_LLADDR(&energest_addr);
  	LOG_INFO_("\n");

    nullnet_buf = (uint8_t *) &energest_values;
    nullnet_len = sizeof(energest_values);

	NETSTACK_NETWORK.output(&energest_addr);

    energest_flush();
    prev_energest_vals.cpu = energest_type_time(ENERGEST_TYPE_CPU);
    prev_energest_vals.lpm = energest_type_time(ENERGEST_TYPE_LPM);
    prev_energest_vals.transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    prev_energest_vals.listen = energest_type_time(ENERGEST_TYPE_LISTEN);
    prev_energest_vals.totaltime = RTIMER_NOW();
}
#endif /* ENERGEST_CONF_ON */
/*---------------------------------------------------------------------------*/
int state = 0;
void input_callback(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest)
{
    struct testmsg msg;
    memcpy(&msg, data, len);

    seqno++;

    msg.seqno=seqno;

    LOG_INFO("Received data from ");
    LOG_INFO_LLADDR(src);
    LOG_INFO_("\n");


    static uint8_t seqno_bits[IO_WIDTH];			
    uint8_t i;
    for (i = 0; i < IO_WIDTH; i++) {
        seqno_bits[i] = msg.seqno & (1 << i) ? 1 : 0;
    }		//least significant bit in seqno_bits[0]

    clear_GPIOS();
    
    if ( seqno_bits[0]==1 )		GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6));       //  write a 1 in A6
    if ( seqno_bits[1]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0));       //  write a 1 in C0
    if ( seqno_bits[2]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1));       //  write a 1 in C1
    if ( seqno_bits[3]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2));       //  write a 1 in C2
    if ( seqno_bits[4]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3));       //  write a 1 in C3
    if ( seqno_bits[5]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));       //  write a 1 in C4
    if ( seqno_bits[6]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5));       //  write a 1 in C5
    if ( seqno_bits[7]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(6));       //  write a 1 in C6
    if ( seqno_bits[8]==1 )		GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(0));       //  write a 1 in D0
    if ( seqno_bits[9]==1 )		GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(1));       //  write a 1 in D1
    if ( seqno_bits[10]==1 )	GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(2));       //  write a 1 in D2
    
    if (state == 0){
        GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
        state = 1;
    }
    else{
        GPIO_CLR_PIN(GPIO_PORT_TO_BASE(0),GPIO_PIN_MASK(7));
        state = 0;
    }

#if ENERGEST_CONF_ON
    if (seqno%ENERGEST_FREQ==0)
		send_energest();
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observed_receiver_process, ev, data)
{

    PROCESS_BEGIN();

  	prev_energest_vals.cpu = 0;
    prev_energest_vals.lpm = 0;
    prev_energest_vals.transmit = 0;
    prev_energest_vals.listen = 0;
    prev_energest_vals.seqno = 0;
	prev_energest_vals.totaltime = 0;

#if MAC_CONF_WITH_TSCH
  	tsch_set_coordinator(0);
#endif /* MAC_CONF_WITH_TSCH */

    GPIOS_init();

  	nullnet_set_input_callback(input_callback);

    while(1) {
        PROCESS_YIELD();
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
