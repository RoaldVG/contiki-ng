#ifndef _PROJECT_CONF_H
#define _PROJECT_CONF_H

#define LPM_CONF_ENABLE 0

#define cc2538_CONF_QUIET 0


#if cc2538_CONF_QUIET
#define UART_CONF_ENABLE 0
#define DBG_CONF_USB 0
#endif

#define ENERGEST_CONF_ON 1

#include "stdint.h"

struct monitormsg {
    uint32_t counter;
    uint32_t power;
    uint32_t total;
};

struct observedmsg {
    uint32_t cpu;
    uint32_t lpm;
    //uint32_t deep_lpm;
    uint32_t total;
    uint32_t tx;
    uint32_t rx;
};
#endif
