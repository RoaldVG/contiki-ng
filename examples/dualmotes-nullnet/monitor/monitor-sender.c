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
 *         White (observer) sender program for the dual network.
 *         This program sends a messsage to the white sink whenever it is trigered via GPIO pin 1.0 by the connected black mote.
 *
 * \author
 *         Marie-Paule Uwase
 *         August 7, 2012
 *         Roald Van Glabbeek
 * 		     March 3, 2020
 * 
 *         Updated for newer contiki release en Zolertia Zoul (firefly) and IPv6
 */

#include "contiki.h"
#include "project-conf.h"
#include "sys/rtimer.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"

#include "dev/gpio.h"
#include "dev/gpio-hal.h"
#include "dev/zoul-sensors.h"
#include "dev/adc-zoul.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "dev/ioc.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

/*
 * Interval between consecutive probes of the triger bit P1.0
 */

#define ADC_READ_INTERVAL (CLOCK_SECOND/128)

struct testmsg {
    uint16_t  observed_seqno;
    uint16_t monitor_seqno;
    uint32_t energy;
    uint16_t counter_ADC;
    rtimer_clock_t timestamp_app;
    rtimer_clock_t timestamp_mac;
};

/* 
 * Data structure of sent messages
 */
uint16_t  monitor_seqno=0;
uint32_t  ADCResult=0;
uint32_t  counter=0;
uint8_t   flag;

/* sender power
 * possible values =  0dBm = 31;  -1dBm = 27;  -3dBm = 23;  -5dBm = 19; 
 *                    -7dBm = 15; -10dBm = 11; -15dBm =  7; -25dBM =  3;
 */ 
uint8_t power = 31;

linkaddr_t sink_addr = {{ 0x00, 0x12, 0x4b, 0x00, 0x14, 0xd5, 0x2d, 0xc8 }};


//static void send_packet(gpio_hal_pin_mask_t pin_mask);

/*---------------------------------------------------------------------------*/
PROCESS(monitor_sender_process, "Monitor sender process");
AUTOSTART_PROCESSES(&monitor_sender_process);
/*--------------------------------------------------------------------------------
 * SETTING THE GPIOS
 *-------------------------------------------------------------------------------*/
void
GPIOS_init(void)
{
	GPIO_SOFTWARE_CONTROL(GPIO_A_BASE,GPIO_PIN_MASK(6));
    GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(6));		//GPIO PA6
  
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
    GPIO_SOFTWARE_CONTROL(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6

	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
    GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
    GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
    GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6

	GPIO_SOFTWARE_CONTROL(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
    GPIO_SOFTWARE_CONTROL(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SOFTWARE_CONTROL(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2

	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
    GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2

    GPIO_SOFTWARE_CONTROL(GPIO_A_BASE,GPIO_PIN_MASK(7));
	GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));
}
/*---------------------------------------------------------------------------*/
uint16_t
read_GPIOS(void)
{
	//reading the value in each pin
	uint16_t  observed_seqno=0;

	if (GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6)))	observed_seqno=observed_seqno+1;		
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0)))    observed_seqno=observed_seqno+2;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1)))	observed_seqno=observed_seqno+4; 
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2)))	observed_seqno=observed_seqno+8;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3)))	observed_seqno=observed_seqno+16; 
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4)))	observed_seqno=observed_seqno+32; 
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5)))    observed_seqno=observed_seqno+64;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(6)))	observed_seqno=observed_seqno+128; 
	if (GPIO_READ_PIN(GPIO_D_BASE,GPIO_PIN_MASK(0)))	observed_seqno=observed_seqno+256;
	if (GPIO_READ_PIN(GPIO_D_BASE,GPIO_PIN_MASK(1)))	observed_seqno=observed_seqno+512; 
	if (GPIO_READ_PIN(GPIO_D_BASE,GPIO_PIN_MASK(2)))	observed_seqno=observed_seqno+1024; 

	return observed_seqno;
}
/*---------------------------------------------------------------------------*/
static void
send_packet()
{
	monitor_seqno++;
	static struct testmsg msg;

	msg.observed_seqno = read_GPIOS();	
	msg.monitor_seqno = monitor_seqno;	
	msg.energy = ADCResult;
	msg.counter_ADC = counter;
	msg.timestamp_app = RTIMER_NOW();

	LOG_INFO("Data sent to sink ");
    LOG_INFO_LLADDR(&sink_addr);
    LOG_INFO_("\n");

    nullnet_buf = (uint8_t *) &msg;
	nullnet_len = sizeof(msg);
	NETSTACK_NETWORK.output(&sink_addr);

    ADCResult=0;
    counter=0;
	NETSTACK_RADIO.get_object(RADIO_PARAM_LAST_PACKET_TIMESTAMP, &msg.timestamp_mac, sizeof(rtimer_clock_t));
}
/*---------------------------------------------------------------------------*/
int prev_io_flag = 0;
PROCESS_THREAD(monitor_sender_process, ev, data)
{
    static struct etimer periodic;
	static struct etimer sendtimer;
	//linkaddr_t local_addr = {{ 0x00, 0x12, 0x4b, 0x00, 0x18, 0x00, 0x01, 0x11 }};
	//linkaddr_set_node_addr(&local_addr);
	
    PROCESS_BEGIN();

	NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, power);

    // init ADC on A4
    adc_zoul.configure(SENSORS_HW_INIT,ZOUL_SENSORS_ADC2);
    counter = 0;

    etimer_set(&periodic, ADC_READ_INTERVAL);
	etimer_set(&sendtimer, CLOCK_SECOND);
	GPIOS_init();
    while(1) {
		PROCESS_YIELD();
        //PROCESS_WAIT_UNTIL(etimer_expired(&periodic));

		if(ev == PROCESS_EVENT_TIMER){
			if(data == &periodic){

				if(prev_io_flag != GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7))){
					send_packet();
					prev_io_flag = GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));
				}
			
				counter++;
				int ADC_val = adc_zoul.value(ZOUL_SENSORS_ADC2);
				ADCResult += ADC_val;
				etimer_reset(&periodic);	
			}
			/*
			if(data == &sendtimer){
				send_packet();
				etimer_reset(&sendtimer);
			}*/
		}
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
