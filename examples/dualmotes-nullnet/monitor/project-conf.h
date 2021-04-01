#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

//#define ZOUL_CONF_USE_CC1200_RADIO 1
#define IEEE802154_CONF_DEFAULT_CHANNEL 20

#define ENERGEST_FREQ 2       // every x messages a message is sent to the energest sink
#define IO_WIDTH 11

// UART pins are used for parallel communication, serial comm over UART overwrites some pins
#define UART_CONF_ENABLE 1

#include "stdint.h"
struct testmsg {
    uint16_t  observed_seqno;
    uint16_t monitor_seqno;
    uint32_t energy;
    uint16_t counter_ADC;
    uint32_t timestamp_app;
    uint32_t timestamp_mac;
};

#endif // PROJECT_CONF_H_
