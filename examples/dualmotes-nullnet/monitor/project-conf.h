#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#include "stdint.h"
#include "sys/rtimer.h"

//#define ZOUL_CONF_USE_CC1200_RADIO 1
#undef IEEE802154_CONF_DEFAULT_CHANNEL
#define IEEE802154_CONF_DEFAULT_CHANNEL 20

#define ENERGEST_FREQ 100       // every x messages a message is sent to the energest sink
#define IO_WIDTH 11

// UART pins are used for parallel communication, serial comm over UART overwrites some pins
<<<<<<< HEAD
#undef UART_CONF_ENABLE
#define UART_CONF_ENABLE 0
#undef USB_SERIAL_CONF_ENABLE
#define USB_SERIAL_CONF_ENABLE 0

#endif // PROJECT_CONF_H_p
=======
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
>>>>>>> 37c9dabffb447a16f8c0157359f01fa9f8d22293
