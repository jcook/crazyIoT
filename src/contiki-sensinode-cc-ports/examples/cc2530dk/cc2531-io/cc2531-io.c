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

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "debug.h"

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define SEND_INTERVAL		2 * CLOCK_SECOND
#define MAX_PAYLOAD_LEN		40

static char buf[MAX_PAYLOAD_LEN];

/* Our destinations and udp conns. One link-local and one global */
#define LOCAL_CONN_PORT 3001
static struct uip_udp_conn *l_conn;

static uint8_t current_interval = 0;

static struct etimer et;

#include "command.h"

#define Assert(x)

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);

static uint8_t isCmd(void *data)
{
	return (((uint8_t *)data)[0] == CMD_MAGIC_NUM);
}

static uint8_t command_handler(void)
{
	uint8_t len = uip_datalen();
	printf("Got %x bytes\n", len);

	if (len > MAX_PAYLOAD_LEN) {
		printf("buffer overhead, re...\n");
		return -1;
	}
	memset(buf, 0, MAX_PAYLOAD_LEN);
	memcpy(buf, uip_appdata, len);

	Assert(buf[0] == CMD_MAGIC_NUM);

	switch (buf[1]) {
	case CMD_GET_INTERVAL:
	case CMD_INC_INTERVAL:
	case CMD_DES_INTERVAL:
		printf("Got cmd %x\n", buf[1]);
		current_interval += 10;
		if (current_interval > 60)
			current_interval = 60;
		etimer_set(&et, current_interval * CLOCK_SECOND);
	}

	return 0;
}
/*---------------------------------------------------------------------------*/
static void tcpip_handler(void)
{
	if(uip_newdata()) {
		if (uip_datalen() > 0 && isCmd(uip_appdata))
			command_handler();
	}
	return;
}

static void button_event(void)
{
	int i;
	printf("button event\n");

	leds_on(LEDS_RED);
	i = 0;
	while (i++ < 10000);
	leds_off(LEDS_RED);
	i = 0;
	while (i++ < 10000);
	leds_on(LEDS_RED);
	i = 0;
	while (i++ < 10000);
	leds_off(LEDS_RED);

	leds_on(LEDS_GREEN);
	i = 0;
	while (i++ < 10000);
	leds_off(LEDS_GREEN);
	i = 0;
	while (i++ < 10000);
	leds_on(LEDS_GREEN);
	i = 0;
	while (i++ < 10000);
	leds_off(LEDS_GREEN);
}

/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
  static int seq_id;
  struct uip_udp_conn *this_conn;

  leds_on(LEDS_RED);
  memset(buf, 0, MAX_PAYLOAD_LEN);
  seq_id++;

  this_conn = l_conn;

  PRINTF("Client to: ");
  PRINT6ADDR(&this_conn->ripaddr);

  memcpy(buf, &seq_id, sizeof(seq_id));

  PRINTF(" Remote Port %u,", UIP_HTONS(this_conn->rport));
  PRINTF(" (msg=0x%04x), %u bytes\n", *(uint16_t *) buf, sizeof(seq_id));

  uip_udp_packet_send(this_conn, buf, sizeof(seq_id));
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();
  PRINTF("UDP client process started\n");

  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0204, 0x2519, 0x1801, 0xb419);
  /* new connection with remote host */
  l_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);
  if(!l_conn) {
    PRINTF("udp_new l_conn error.\n");
  }
  udp_bind(l_conn, UIP_HTONS(LOCAL_CONN_PORT));

  PRINTF("Link-Local connection with ");
  PRINT6ADDR(&l_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
         UIP_HTONS(l_conn->lport), UIP_HTONS(l_conn->rport));

  etimer_set(&et, SEND_INTERVAL);

  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&et)) {
      timeout_handler();
      etimer_restart(&et);
    } else if(ev == tcpip_event) {
      tcpip_handler();
    }
#if BUTTON_SENSOR_ON
    else if(ev == sensors_event && data == &button_sensor) {
      button_event();
    }
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
