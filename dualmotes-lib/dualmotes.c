#include "contiki.h"
#include "sys/process.h"
#include "dualmotes.h"
#include "dev/gpio.h"
#include "dev/gpio-hal.h"
#include "dev/zoul-sensors.h"
#include "dev/adc-zoul.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "Dualmotes"
#define LOG_LEVEL LOG_LEVEL_INFO

process_event_t gpio_flag_event;
#if !DUALMOTES_OBSERVED
static gpio_hal_event_handler_t gpio_flag_event_handler;
#endif
/* A mask of all pins that have changed state since the last process poll */
static volatile gpio_hal_pin_mask_t pmask;

uint16_t read_GPIOS();
/*---------------------------------------------------------------------------*/
#if !DUALMOTES_OBSERVED
static void
gpio_flag_handler( gpio_hal_pin_mask_t pin_mask )
{
	//pmask = 0;
	//pmask |= pin_mask;
	//if(gpio_hal_pin_to_mask(6 & pmask)) {
  		uint16_t number = read_GPIOS();
  		process_post(PROCESS_BROADCAST, gpio_flag_event, &number);
	//}
}
#endif
/*---------------------------------------------------------------------------*/
void
dualmotes_init( void )
{
	/* config for reading the flags raised by the observed mote on pin A6 */
	#if !DUALMOTES_OBSERVED
	gpio_hal_pin_cfg_t cfg;
	gpio_flag_event = process_alloc_event();

	gpio_flag_event_handler.pin_mask = 0;
	gpio_flag_event_handler.handler = gpio_flag_handler;

	cfg = GPIO_HAL_PIN_CFG_EDGE_BOTH | GPIO_HAL_PIN_CFG_INT_ENABLE |
      GPIO_HAL_PIN_CFG_PULL_NONE;
    gpio_hal_arch_pin_set_input(GPIO_A_NUM, 6);
    gpio_hal_arch_pin_cfg_set(GPIO_A_NUM, 6, cfg);
    gpio_hal_arch_interrupt_enable(GPIO_A_NUM, 6);
    gpio_flag_event_handler.pin_mask |= gpio_hal_pin_to_mask(6);

  	gpio_hal_register_handler(&gpio_flag_event_handler);
	#endif 	/* !DUALMOTES_OBSERVED */

	// config pins as input/output, depending on version and observed/monitoring mote
	#if DUALMOTES_VERSION == 1
	#if DUALMOTES_OBSERVED
	GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(6));		//GPIO PA6
	GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7
  
 	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
  	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
  	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6

	GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
  	GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2

	GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6));

	#else 	/* DUALMOTES_OBSERVED */
	GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7
  
 	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
  	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5
  	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(6));		//GPIO PC6

	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(0));		//GPIO PD0
  	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(2));		//GPIO PD2
	adc_zoul.configure(SENSORS_HW_INIT,ADC_USED);
	#endif 	/* DUALMOTES_OBSERVED */

	#else	/* DUALMOTES_VERSION */
	#if DUALMOTES_OBSERVED
	GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(6));		//GPIO PA6
	GPIO_SET_OUTPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7
  
 	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
  	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_OUTPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5

  	GPIO_SET_OUTPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1

	GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6));

	#else	/* DUALMOTES_OBSERVED */
	GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(2));		//GPIO PA2
	GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(5));		//GPIO PA5
	GPIO_SET_INPUT(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7
  
 	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
  	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_SET_INPUT(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5

  	GPIO_SET_INPUT(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1

	adc_zoul.configure(SENSORS_HW_INIT,ADC_USED);
	#endif 	/* DUALMOTES_OBSERVED */
	#endif 	/* DUALMOTES_VERSION */
}
/*---------------------------------------------------------------------------*/
void
clear_GPIOS( void )
{
	#if DUALMOTES_VERSION == 1
	GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7
	
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

	#else
	#if DUALMOTES_OBSERVED
	GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));		//GPIO PA7
  
 	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0));		//GPIO PC0
	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1));		//GPIO PC1
  	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2));		//GPIO PC2
  	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3));		//GPIO PC3
	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));		//GPIO PC4
	GPIO_CLR_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5));		//GPIO PC5

  	GPIO_CLR_PIN(GPIO_D_BASE,GPIO_PIN_MASK(1));		//GPIO PD1
	#endif /* DUALMOTES_OBSERVED */
	#endif /* DUALMOTES_VERSION */
}

uint16_t
read_GPIOS( void )
{	
	uint16_t  observed_seqno=0;

	//read the value in each pin

	#if !DUALMOTES_OBSERVED
	#if DUALMOTES_VERSION == 1
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
	
	#else
	if (GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7)))	observed_seqno=observed_seqno+1;		
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0)))    observed_seqno=observed_seqno+2;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1)))	observed_seqno=observed_seqno+4; 
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2)))	observed_seqno=observed_seqno+8;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3)))	observed_seqno=observed_seqno+16; 
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4)))	observed_seqno=observed_seqno+32; 
	if (GPIO_READ_PIN(GPIO_A_BASE,GPIO_PIN_MASK(2)))    observed_seqno=observed_seqno+64;
	if (GPIO_READ_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5)))	observed_seqno=observed_seqno+256;
	#endif /* DUALMOTES_VERSION */
	#endif /* !DUALMOTES_OBSERVED */

	return observed_seqno;
}

/*---------------------------------------------------------------------------*/
void
dualmotes_send( uint16_t number )
{
	#if DUALMOTES_OBSERVED
	static uint8_t io_flag = 0;
    static uint8_t number_bits[IO_WIDTH];			
	uint8_t i;
	for (i = 0; i < IO_WIDTH; i++) {
		number_bits[i] = number & (1 << i) ? 1 : 0;
	}		//least significant bit in number_bits[0]
	
	clear_GPIOS();
	
	#if DUALMOTES_VERSION == 1
	if ( number_bits[0]==1 )		GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));       //  write a 1 in A7
	if ( number_bits[1]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0));       //  write a 1 in C0
	if ( number_bits[2]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1));       //  write a 1 in C1
	if ( number_bits[3]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2));       //  write a 1 in C2
	if ( number_bits[4]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3));       //  write a 1 in C3
	if ( number_bits[5]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));       //  write a 1 in C4
	if ( number_bits[6]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5));       //  write a 1 in C5
	if ( number_bits[7]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(6));       //  write a 1 in C6
	if ( number_bits[8]==1 )		GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(0));       //  write a 1 in D0
	if ( number_bits[9]==1 )		GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(1));       //  write a 1 in D1
	if ( number_bits[10]==1 )	    GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(2));       //  write a 1 in D2

	#else
	if ( number_bits[0]==1 )		GPIO_SET_PIN(GPIO_A_BASE,GPIO_PIN_MASK(7));       //  write a 1 in A7
	if ( number_bits[1]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(0));       //  write a 1 in C0
	if ( number_bits[2]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(1));       //  write a 1 in C1
	if ( number_bits[3]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(2));       //  write a 1 in C2
	if ( number_bits[4]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(3));       //  write a 1 in C3
	if ( number_bits[5]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(4));       //  write a 1 in C4
	if ( number_bits[6]==1 )		GPIO_SET_PIN(GPIO_C_BASE,GPIO_PIN_MASK(5));       //  write a 1 in C5
	if ( number_bits[7]==1 )		GPIO_SET_PIN(GPIO_D_BASE,GPIO_PIN_MASK(0));       //  write a 1 in D0
	#endif /* DUALMOTES_VERSION */

    if (io_flag){
		GPIO_CLR_PIN(GPIO_A_BASE,GPIO_PIN_MASK(6));
		io_flag = 0;
	}
	else{
		GPIO_SET_PIN(GPIO_PORT_TO_BASE(0),GPIO_PIN_MASK(6));
		io_flag = 1;
	}
	#endif /* DUALMOTES_OBSERVED */
}