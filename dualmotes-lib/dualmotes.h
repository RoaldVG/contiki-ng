#ifndef DUALMOTES_H_
#define DUALMOTES_H_

#include <stdint.h>

#ifndef DUALMOTES_VERSION
#define DUALMOTES_VERSION 2
#endif

#if DUALMOTES_VERSION == 1
#define IO_WIDTH 11
#else
#define IO_WIDTH 8
#endif /* DUALMOTES_VERSION */

// Pin to alert monitor of reception/transmission
#define FLAG_PIN_PORT       GPIO_A_BASE
#define FLAG_PIN_PINMASK    GPIO_PIN_MASK(6)

#define ADC_USED ZOUL_SENSORS_ADC2

// Needed to get correct timestamps both when sending and receiving with the CC1200 sub-GHz radio
#if ZOUL_CONF_USE_CC1200_RADIO
#if !CC1200_CONF_USE_GPIO2
#define CC1200_CONF_USE_GPIO2                         1
#endif /* CC1200_CONF_USE_GPIO2 */

#ifdef GPIO2_IOCFG
#undef GPIO2_IOCFG
#endif /* ifdef */
#define GPIO2_IOCFG         CC1200_IOCFG_PKT_SYNC_RXTX
#endif /* ZOUL_CONF_USE_CC1200_RADIO */

void dualmotes_init(void);
void dualmotes_send(uint16_t number);

extern process_event_t gpio_flag_event;

#endif /* DUALMOTES_H_ */