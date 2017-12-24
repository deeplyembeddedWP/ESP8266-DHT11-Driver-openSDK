/*
 * MIT License
Copyright (c) 2017 DeeplyEmbedded

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

/*
 * esp8266_dht11.c
 *
 * Created on  : Dec 23, 2017
 * Author      : Vinay Divakar
 * Description : DHT11 Driver for ESP8266 - open SDK. This driver has been designed
 *               considering that the ESP8266 is running at 80 MHz clock frequency.
 *               The driver supports usage of multiple DHT11's simultaneously.
 * Website     : www.deeplyembedded.org
 */

/* Lib Includes */
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_MyConf.h"

/* Custom Libs */
#include "esp8266_dht11.h"

/* Debug Utilities */
//#define DHT_DBG_RESP

/* Static Functions */
static void DHT11_start(unsigned char data_pin);
static DHT11_State DHT11_detect_response(unsigned char dat_pin);
static DHT11_State DHT11_decode_data(unsigned char dat_pin_d, unsigned char *pkt_cnt);
static DHT11_State DHT11_package_data(unsigned char *comp_pkt, unsigned char *uncomp_pkt);
static void DHT11_Init(DHT11_Ptr dht_dev_Ptr,unsigned int st_sig_low_ms, unsigned int st_sig_high_us,
		int data_pin);
static void DHT11_Get_Data(DHT11_Ptr dhtt_dev_Ptr);

/* Function  Prototypes - API to be used by the Application Programmer */
void init_dht11_dev1(DHT11_Ptr dhtt_dev_Ptr, os_timer_t *esp_timer,
		int timer_us, bool repeat);


/****************************************************************
 * Function Name : DHT11_Init
 * Description   : Initialize the data structure.
 * Returns       : NONE.
 * Params        : @dht_dev_Ptr    - Pointer to the object
 *                 @st_sig_low_ms  - Request signal low time
 *                 @st_sig_high_us - Request signal high time
 *                 @data_pin       - Communication pin
 ****************************************************************/
void DHT11_Init(DHT11_Ptr dht_dev_Ptr,
		unsigned int st_sig_low_ms,
		unsigned int st_sig_high_us,
		int data_pin)
{
	dht_dev_Ptr->start_sig_low_ms = st_sig_low_ms;
	dht_dev_Ptr->start_sig_high_us = st_sig_high_us;
	dht_dev_Ptr->comm_pin_no = data_pin;
	dht_dev_Ptr->dht11_state = ERROR;
	dht_dev_Ptr->rh_integral_data = 0x00;
	dht_dev_Ptr->rh_dec_data = 0x00;
	dht_dev_Ptr->temp_integral_data = 0x00;
	dht_dev_Ptr->temp_dec_data = 0x00;
	dht_dev_Ptr->rh = 0.0;
	dht_dev_Ptr->temperature = 0.0;
	memset(dht_dev_Ptr->dht11_pkt, 0x00, DTH11_PACKET_SIZE);
	memset(dht_dev_Ptr->dht11_pkt_compressed, 0x00, DHT11_PKT_COMPRESSED_SIZE);
}

/****************************************************************
 * Function Name : DHT11_Get_Data
 * Description   : Request and get data from the DTH11 sensor.
 * Returns       : NONE.
 * Params        : @dht_dev_Ptr - Pointer to the dht11 object
 ****************************************************************/
void DHT11_Get_Data(DHT11_Ptr dhtt_dev_Ptr)
{
	unsigned long count = 0x00;
	char T_str[8], H_str[8];

	dhtt_dev_Ptr->dht11_state = ERROR;

	/* Request for data from DTH11 */
	DHT11_start(dhtt_dev_Ptr->comm_pin_no);

	/* Capture and print response signal low counts */
	dhtt_dev_Ptr->dht11_state = DHT11_detect_response(dhtt_dev_Ptr->comm_pin_no);

	/* Decode the data if the response is valid */
	if(dhtt_dev_Ptr->dht11_state == GOOD_RESPONSE)
		dhtt_dev_Ptr->dht11_state = DHT11_decode_data(dhtt_dev_Ptr->comm_pin_no, dhtt_dev_Ptr->dht11_pkt);
	else
		os_printf("Time out\r\n");

	/* Compress and the data if the packet is valid */
	if (dhtt_dev_Ptr->dht11_state == GOOD_DATA)
		dhtt_dev_Ptr->dht11_state = DHT11_package_data(dhtt_dev_Ptr->dht11_pkt_compressed, dhtt_dev_Ptr->dht11_pkt);
	else
		os_printf("No Packet Detected\r\n");

	/* Compute temperature and RH if checksum has passed*/
	if (dhtt_dev_Ptr->dht11_state == CHECKSUM_PASS)
	{
		dhtt_dev_Ptr->rh_dec_data = dhtt_dev_Ptr->dht11_pkt_compressed[0];
		dhtt_dev_Ptr->rh_integral_data = dhtt_dev_Ptr->dht11_pkt_compressed[1];
		dhtt_dev_Ptr->temp_dec_data = dhtt_dev_Ptr->dht11_pkt_compressed[2];
		dhtt_dev_Ptr->temp_integral_data = dhtt_dev_Ptr->dht11_pkt_compressed[3];

		/* Compute the temperature and RH value for DTH11 or DTH22 */
		dhtt_dev_Ptr->rh = dhtt_dev_Ptr->rh_dec_data;
		dhtt_dev_Ptr->temperature = dhtt_dev_Ptr->temp_dec_data;

#ifdef DHT_DBG_RESP
		/* print the temperature, humidity and checksum data */
		os_printf("\r\n");
		os_printf("Temperature  = %dC\r\n", dhtt_dev_Ptr->temperature);
		os_printf("Humidity     = %d%\r\n", dhtt_dev_Ptr->rh);
		os_printf("Checksum     = %d + %d + %d + %d = %d\r\n", dhtt_dev_Ptr->dht11_pkt_compressed[0],
				dhtt_dev_Ptr->dht11_pkt_compressed[1], dhtt_dev_Ptr->dht11_pkt_compressed[2], dhtt_dev_Ptr->dht11_pkt_compressed[3],
				dhtt_dev_Ptr->dht11_pkt_compressed[4]);
#endif
	}
	else
		os_printf("Checksum Failed\r\n");
}

/****************************************************************
 * Function Name : DHT11_start
 * Description   : Send a request signal to the DTH11 sensor.
 * Returns       : NONE.
 * Params        : @data_pin - Communication pin
 ****************************************************************/
void DHT11_start(unsigned char data_pin)
{
	pinMode(data_pin, OUTPUT);
	digitalWrite(data_pin, LOW);
	os_delay_us(DTH11_START_SIGNAL_LOW_MS_US);
	digitalWrite(data_pin, HIGH);
	os_delay_us(DTH11_START_SIGNAL_HIGH_US);
	pinMode(data_pin, INPUT);
}

/****************************************************************
 * Function Name : DHT11_detect_response
 * Description   : Detect the response from the DHT11 sensor.
 * Returns       : @GOOD_RESPONSE - Got response
 *                 @TIMEOUT       - Response wait timed out
 * Params        : @data_pin      - Communication pin
 ****************************************************************/
DHT11_State DHT11_detect_response(unsigned char dat_pin)
{
	unsigned long cnt_L = 0x00, cnt_H = 0x00;

	while(digitalRead(dat_pin) == LOW)
	{
		cnt_L++;

		/* Check for timeout */
		if(cnt_L > DTH11_RESP_SIG_LOW_CNTS_HE)
			break;
	}

	while(digitalRead(dat_pin) == HIGH)
	{
		cnt_H++;

		/* Check for timeout */
		if(cnt_H > DTH11_RESP_SIG_HIGH_CNTS_HE)
			break;
	}

	/* Check if this is actually a response and not some crap */
	if((cnt_L < DTH11_RESP_SIG_LOW_CNTS_HE && cnt_L > DTH11_RESP_SIG_LOW_CNTS_LE)
			&& (cnt_H < DTH11_RESP_SIG_HIGH_CNTS_HE && cnt_H > DTH11_RESP_SIG_HIGH_CNTS_LE))
		return (GOOD_RESPONSE);
	else
		return (TIMEOUT);
}

/****************************************************************
 * Function Name : DHT11_decode_data
 * Description   : Decode the the data sent by DHT11 sensor.
 * Returns       : @GOOD_DATA - Decode completed
 *                 @ERROR     - Decode error
 * Params        : @dat_pin_d - Communication pin
 *                 @pkt_cnt   - Packet to fill with the logic data
 *                 bits.
 ****************************************************************/
DHT11_State DHT11_decode_data(unsigned char dat_pin_d, unsigned char *pkt_cnt)
{
	int i = 0x00;
	unsigned long cnt_L_d = 0x00, cnt_H_d = 0x00;
	DHT11_State state_ret;

	/* Clear the buffer before adding new elements */
	memset(pkt_cnt, 0x00, DTH11_PACKET_SIZE);

	for(i=0 ; i<=DTH11_PACKET_SIZE ; i++)
	{
		while(digitalRead(dat_pin_d) == LOW)
		{
			cnt_L_d++;
			if(cnt_L_d > DHT11_LOGIC_MAX_LOW_CNTS)
				break;
		}
		while(digitalRead(dat_pin_d) == HIGH)
		{
			cnt_H_d++;
			if(cnt_H_d > DHT11_LOGIC_MAX_HIGH_CNTS)
				break;
		}

		/* Perform bound checks */
		if (cnt_L_d > DHT11_LOGIC_MAX_LOW_CNTS || cnt_H_d > DHT11_LOGIC_MAX_HIGH_CNTS)
			break;
		else
		{
			if(cnt_L_d > cnt_H_d)
				pkt_cnt[i] = DHT11_LOGIC_0;
			else
				pkt_cnt[i] = DHT11_LOGIC_1;

			/* Reset the counters */
			cnt_L_d = 0x00;
			cnt_H_d = 0x00;
		}
	}

	/* Check if we have the entire packet */
	if(i == DTH11_PACKET_SIZE)
	{
		/* Check for end of frame */
		if(cnt_L_d < DHT11_LOGIC_MAX_LOW_CNTS)
			state_ret = GOOD_DATA;
		else
			state_ret = ERROR;
	}
	else
		state_ret = ERROR;

	return (state_ret);
}

/****************************************************************
 * Function Name : DHT11_package_data
 * Description   : Compress and package the data to be sent to
 *                 application layer.
 * Returns       : @CHECKSUM_PASS  - Data is valid
 *                 @CHECKSUM_ERROR - Error data
 * Params        : @comp_pkt       - Pointer to uncompressed pkt
 *                 @uncomp_pkt     - Pointer to compressed pkt
 ****************************************************************/
DHT11_State DHT11_package_data(unsigned char *comp_pkt, unsigned char *uncomp_pkt)
{
	int i = 0x00, j = 0x00, pos = 0x00, check_sum = 0x00;

	/* Clear the buffer before adding new elements */
	memset(comp_pkt, 0x00, DHT11_PKT_COMPRESSED_SIZE);

	/* Create a compressed package of the frame */
	for(i=0x00, pos=0x07 ; i<DTH11_PACKET_SIZE ; i++)
	{
		if(uncomp_pkt[i] == DHT11_LOGIC_0)
			comp_pkt[j] &= ~(0x01 << pos);
		else if (uncomp_pkt[i] == DHT11_LOGIC_1)
			comp_pkt[j] |= (0x01 << pos);
		else
			os_printf("It should never come ere!");

		/* Bit position tracker */
		pos--;

		/* Byte is ready- Package the next set */
		if(pos == -1)
		{
			j++;
			pos = 0x07;
		}
	}

	/* Compute and verify the checksum */
	check_sum = comp_pkt[0] + comp_pkt[1] + comp_pkt[2] + comp_pkt[3];

	if (check_sum == comp_pkt[4])
		return (CHECKSUM_PASS);
	else
		return (CHECKSUM_ERROR);
}

/****************************************************************
 * Function Name : init_dht11_dev1
 * Description   : Initialize and set up the DHT11 sensor & esp
 *                 for reading the temperature and humidity data.
 * Returns       : NONE.
 * Params        : @dhtt_dev_Ptr - Pointer to the dht11 object.
 *                 @esp_timer    - Pointer to timer object.
 *                 @timer_us     - Timer firing interval
 *                 @repeat       - 1 for repeat and 0 for one shot.
 ****************************************************************/
void init_dht11_dev1(DHT11_Ptr dhtt_dev_Ptr, os_timer_t *esp_timer,
		int timer_us, bool repeat)
{
	/* Initialize the DHT11 data structure */
	DHT11_Init(dhtt_dev_Ptr, DTH11_START_SIGNAL_LOW_MS_US,
			DTH11_START_SIGNAL_HIGH_US, DTH11_DATA_PIN);

	/* Setup and initialize the timer */
	os_timer_disarm(esp_timer);
	os_timer_setfn(esp_timer, (os_timer_func_t *)DHT11_Get_Data, dhtt_dev_Ptr);
	os_timer_arm(esp_timer, timer_us, repeat);
}
