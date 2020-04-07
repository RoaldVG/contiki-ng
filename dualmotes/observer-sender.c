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
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-udp-packet.h"

#include "dev/gpio.h"
#include "dev/gpio-hal.h"
#include "dev/zoul-sensors.h"
#include "dev/adc-zoul.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "dev/serial-line.h"
#include "net/ipv6/uip-ds6-route.h"

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

#define DEBUG DEBUG_FULL
#include "net/ipv6/uip-debug.h"

#define UIP_CONF_MAX_ROUTES   10

/*
 * Interval between consecutive probes of the triger bit P1.0
 */

#define ADC_READ_INTERVAL (CLOCK_SECOND/128)

/* 
 * Data structure of sent messages
 */
uint16_t  whiteseqno=0;
uint32_t  ADCResult=0;
uint32_t  counter=0;
uint8_t   flag;

struct whitemsg {
    uint8_t  blackseqno;
    uint16_t whiteseqno;
    uint32_t energy;
    uint16_t counter_ADC;
    uint16_t timestamp_app;
    uint16_t timestamp_mac;
};

/* sender power
 * possible values =  0dBm = 31;  -1dBm = 27;  -3dBm = 23;  -5dBm = 19; 
 *                    -7dBm = 15; -10dBm = 11; -15dBm =  7; -25dBM =  3;
 */ 
uint8_t power = 31;

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static void send_packet(gpio_hal_pin_mask_t pin_mask);

/*---------------------------------------------------------------------------*/
PROCESS(observer_sender_process, "Observer sender process");
AUTOSTART_PROCESSES(&observer_sender_process);
/*--------------------------------------------------------------------------------
 * SETTING THE GPIOS
 *-------------------------------------------------------------------------------*/
static gpio_hal_event_handler_t msg_handler = {
  .next = NULL,
  .handler = send_packet,
  .pin_mask = gpio_hal_pin_to_mask(2) << (GPIO_A_NUM << 3),
};
void
GPIOS_init(void)
{
  GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
  GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
  GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
  GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
  GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2

  // Configure PA2 to raise an interrupt on a rising edge
  GPIO_SOFTWARE_CONTROL(GPIO_A_BASE,GPIO_PIN_MASK(2));
  GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(2));
  GPIO_DETECT_EDGE(GPIO_A_BASE,GPIO_PIN_MASK(2));
  GPIO_TRIGGER_SINGLE_EDGE(GPIO_A_BASE,GPIO_PIN_MASK(2));
  GPIO_DETECT_RISING(GPIO_A_BASE,GPIO_PIN_MASK(2));
  GPIO_ENABLE_INTERRUPT(GPIO_A_BASE,GPIO_PIN_MASK(2));
  gpio_hal_register_handler(&msg_handler);
}
/*---------------------------------------------------------------------------*/
uint8_t
read_GPIOS(void)
{
	//reading the value in each pin
	uint8_t  blackseqno=0;

	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0)))	blackseqno=blackseqno+1;		
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1)))  blackseqno=blackseqno+2;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4)))	blackseqno=blackseqno+4; 
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5)))	blackseqno=blackseqno+8;
	if (GPIO_READ_PIN(GPIO_D_BASE,GPIO_PIN_MASK(1)))	blackseqno=blackseqno+16; 
	if (GPIO_READ_PIN(GPIO_D_BASE,GPIO_PIN_MASK(2)))	blackseqno=blackseqno+32; 

	return blackseqno;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(gpio_hal_pin_mask_t pin_mask)
{
	whiteseqno++;
	struct whitemsg msg;

	msg.blackseqno = read_GPIOS();	
	msg.whiteseqno = whiteseqno;	
	msg.energy = ADCResult;
	msg.counter_ADC = counter;
	msg.timestamp_app = RTIMER_NOW();
	msg.timestamp_mac = 0;

	PRINTF("DATA send to %d\n",
			server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1]);
	uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
							&server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

  ADCResult=0;
  counter=0;
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 102);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoWPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our
 * link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed
 * addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 0);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(observer_sender_process, ev, data)
{
  static struct etimer periodic;
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(&server_ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

	//flag=(GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(2)));

	// adjust power
	//cc2420_set_txpower(power);
	NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, power);

  // init ADC on A5, at 64 bit rate
  //adc_init();
  adc_zoul.configure(SENSORS_HW_INIT,ZOUL_SENSORS_ADC1);
  adc_zoul.configure(ZOUL_SENSORS_CONFIGURE_TYPE_DECIMATION_RATE, SOC_ADC_ADCCON_DIV_64);

	GPIOS_init();
  counter = 0;

  etimer_set(&periodic, ADC_READ_INTERVAL);
  while(1) {
    PROCESS_WAIT_UNTIL(etimer_expired(&periodic));

		counter++;
		int ADC_val = adc_zoul.value(ZOUL_SENSORS_ADC1);
		ADCResult += ADC_val;
    etimer_reset(&periodic);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
