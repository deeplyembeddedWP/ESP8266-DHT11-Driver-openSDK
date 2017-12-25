#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_MyConf.h"
#include "esp8266_dht11.h"

/* App # definitions */
#define DATA_READ_LED 5

/* App objects */
static DHT11_T dht11_obj;
static os_timer_t dht11_timer, dht11_app_timer;

/* App functions */
static void Read_dht11_data(DHT11_Ptr dhtt_app_Ptr);
static void dht11_app_tmr(DHT11_Ptr dhtt_app_Ptr, os_timer_t *esp_app_timer,
                int timer_app_us, bool app_repeat);

void user_rf_pre_init (void)
{
}

/*Init function*/
void ICACHE_FLASH_ATTR
user_init ()
{
	uart_div_modify(0, UART_CLK_FREQ / 9600);
	init_dht11_dev1(&dht11_obj, &dht11_timer, DHT11_TIMER_US, DHT11_REPEAT_TMR);
	dht11_app_tmr(&dht11_obj, &dht11_app_timer, 3000, 1);
}

/****************************************************************
 * Function Name : dht11_app_tmr
 * Description   : Read and print the temperature and humidity
 *                 data.
 * Returns       : NONE.
 * Params        : @dhtt_app_Ptr  - Pointer to the dht11 object.
 ****************************************************************/
void Read_dht11_data(DHT11_Ptr dhtt_app_Ptr)
{
	if(dhtt_app_Ptr->dht11_state == CHECKSUM_PASS)
	{
		/* print the temperature, humidity and checksum data */
		os_printf("\r\n");
		os_printf("Temperature  = %dC\r\n", dhtt_app_Ptr->temperature);
		os_printf("Humidity     = %d%\r\n", dhtt_app_Ptr->rh);
		os_printf("Checksum     = %d + %d + %d + %d = %d\r\n", dhtt_app_Ptr->dht11_pkt_compressed[0],
				dhtt_app_Ptr->dht11_pkt_compressed[1], dhtt_app_Ptr->dht11_pkt_compressed[2], dhtt_app_Ptr->dht11_pkt_compressed[3],
				dhtt_app_Ptr->dht11_pkt_compressed[4]);
		dhtt_app_Ptr->dht11_state = ERROR;

		digitalWrite(DATA_READ_LED, HIGH);
		os_delay_us(50000);
		digitalWrite(DATA_READ_LED, LOW);
	}
	else
		os_printf("Waiting for data...\r\n");
}

/****************************************************************
 * Function Name : dht11_app_tmr
 * Description   : Initialize and set up the app for reading the 
                   temperature and humidity data.
 * Returns       : NONE.
 * Params        : @dhtt_app_Ptr  - Pointer to the dht11 object.
 *                 @esp_app_timer - Pointer to timer object.
 *                 @timer_app_us  - Timer firing interval
 *                 @app_repeat    - 1 for repeat and 0 for one shot.
 * Note          : The app timer interval for reading the data
 *                 should always be lesser than driver timer and
                   greater than atleast 1 second to comprimise
                   with the slowness of the dht11 sensor.
 ****************************************************************/
void dht11_app_tmr(DHT11_Ptr dhtt_app_Ptr, os_timer_t *esp_app_timer,
		int timer_app_us, bool app_repeat)
{
	/* Set up LED to blink everytime a data is read */
	pinMode(DATA_READ_LED, OUTPUT);
	digitalWrite(DATA_READ_LED, LOW);

	/* Setup and initialize the timer to read data */
	os_timer_disarm(esp_app_timer);
	os_timer_setfn(esp_app_timer, (os_timer_func_t *)Read_dht11_data, dhtt_app_Ptr);
	os_timer_arm(esp_app_timer, timer_app_us, app_repeat);
}
