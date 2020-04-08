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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ipv6/uip.h"
#include "net/routing/rpl-classic/rpl.h"

#include "net/netstack.h"
#include <stdio.h>
#include <string.h>

#define 

#include "dev/gpio.h"

#define DEBUG DEBUG_FULL
#include "net/ipv6/uip-debug.h"

//#define NBR_TABLE_CONF_MAX_NEIGHBORS     10
#define UIP_CONF_MAX_ROUTES   10
//#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

/* Data structure of messages sent from sender
 *
 */
struct testmsg {       
  uint16_t  blackseqno;
	uint16_t  timestamp_app;
  char      padding[44];
  int16_t   timestamp_mac;        
};

static struct uip_udp_conn *server_conn;

/*---------------------------------------------------------------------------*/
PROCESS(observed_receiver_process, "observed receiver process");
AUTOSTART_PROCESSES(&observed_receiver_process);
/*---------------------------------------------------------------------------*/
void
GPIOS_init(void)
{
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(0),GPIO_PIN_MASK(2));		//GPIO PA2
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(1));		//GPIO PC1
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(5));		//GPIO PC5
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(3),GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(3),GPIO_PIN_MASK(2));		//GPIO PD2
}
/*---------------------------------------------------------------------------*/
void
clear_GPIOS(void)
{
//clear all output pins
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(1));		//GPIO PC1
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(5));		//GPIO PC5
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(3),GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_CLR_PIN(GPIO_PORT_TO_BASE(3),GPIO_PIN_MASK(2));		//GPIO PD2
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  struct testmsg msg;

  if(uip_newdata()) {
    memcpy(&msg, uip_appdata, sizeof(msg));
    printf("DATA recv from %d\n", uip_udp_conn->ripaddr.u8[sizeof(uip_udp_conn->ripaddr.u8) - 1]);
  }

  static uint8_t seqno_bits[6];			
  uint8_t i;
  for (i = 0; i < 6; i++) {
    seqno_bits[i] = msg.blackseqno & (1 << i) ? 1 : 0;
  }		//least significant bit in seqno_bits[0]
	//struct testmsg msg;
	//memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
/*
 *      convert seqno into bits and set the GPIO bits accordingly
 */
	clear_GPIOS();

	if ( seqno_bits[0]==1 )	GPIO_SET_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(0));       //  write a 1 in C0
	if ( seqno_bits[1]==1 )	GPIO_SET_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(1));       //  write a 1 in C1
	if ( seqno_bits[2]==1 )	GPIO_SET_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(4));       //  write a 1 in C4
	if ( seqno_bits[3]==1 )	GPIO_SET_PIN(GPIO_PORT_TO_BASE(2),GPIO_PIN_MASK(5));       //  write a 1 in C5
	if ( seqno_bits[4]==1 )	GPIO_SET_PIN(GPIO_PORT_TO_BASE(3),GPIO_PIN_MASK(1));       //  write a 1 in D1
	if ( seqno_bits[5]==1 )	GPIO_SET_PIN(GPIO_PORT_TO_BASE(3),GPIO_PIN_MASK(2));       //  write a 1 in D2

	if (GPIO_READ_PIN(GPIO_PORT_TO_BASE(0),GPIO_PIN_MASK(2)) == 0)
		GPIO_SET_PIN(GPIO_PORT_TO_BASE(0),GPIO_PIN_MASK(2));
	else
		GPIO_CLR_PIN(GPIO_PORT_TO_BASE(0),GPIO_PIN_MASK(2));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
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
PROCESS_THREAD(observed_receiver_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  //uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 2);
  //uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  PROCESS_BEGIN();
  PROCESS_PAUSE();
PRINTF("UDP server started. nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

#if UIP_CONF_ROUTER
/* The choice of server address determines its 6LoWPAN header compression.
 * Obviously the choice made here must also be selected in udp-client.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 * Note Wireshark's IPCMV6 checksum verification depends on the correct
 * uncompressed addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 2);
#else
/* Mode 3 - derived from link local (MAC) address */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif

  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */

  print_local_addresses();

  GPIOS_init();

  server_conn = udp_new(&ipaddr, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));


  while(1) {
    PROCESS_WAIT_UNTIL(ev == tcpip_event);
    tcpip_handler();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
