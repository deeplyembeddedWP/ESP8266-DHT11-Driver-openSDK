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
 * esp8266_dht11.h
 *
 * Created on  : Dec 23, 2017
 * Author      : Vinay Divakar
 * Description : DHT11 Driver Definitions - open SDK.
 * Website     : www.deeplyembedded.org
 */

#ifndef __ESP8266_DTH11_H__
#define __ESP8266_DTH11_H__

/* Timing Tolerances in terms of counts */
#define DTH11_RESP_TOL_CNT_LOWER_END                       2
#define DTH11_RESP_TOL_CNT_HIGHER_END                      2
#define DHT11_DAT_TOL_CNT_LOWER_END                        2
#define DHT11_DAT_TOL_CNT_HIGHER_END                       2

/* Timing in terms of counts - For more
 * info check the dht11 data sheet.
 * These counts have been determined for
 * ESP8266 running at clock of 80 MHz*/
#define DTH11_START_SIGNAL_LOW_MS_US                       20000
#define DTH11_START_SIGNAL_HIGH_US                         40
#define DTH11_RESP_SIG_LOW_CNTS_LE                         (88  - DTH11_RESP_TOL_CNT_LOWER_END)
#define DTH11_RESP_SIG_LOW_CNTS_HE                         (88  + DTH11_RESP_TOL_CNT_HIGHER_END)
#define DTH11_RESP_SIG_HIGH_CNTS_LE                        (137 - DTH11_RESP_TOL_CNT_LOWER_END)
#define DTH11_RESP_SIG_HIGH_CNTS_HE                        (137 + DTH11_RESP_TOL_CNT_HIGHER_END)
#define DHT11_DATA_LOW_CNTS_L0                             (97  - DHT11_DAT_TOL_CNT_LOWER_END)
#define DHT11_DATA_HIGH_CNTS_L0                            (97  + DHT11_DAT_TOL_CNT_HIGHER_END)
#define DHT11_DATA_LOW_CNTS_L1                             (130 - DHT11_DAT_TOL_CNT_LOWER_END)
#define DHT11_DATA_HIGH_CNTS_L1                            (130 + DHT11_DAT_TOL_CNT_HIGHER_END)
#define DTH11_EOF_SIG_LOW_CNTS                             1000
#define DTH11_PACKET_SIZE                                  40
#define DHT11_PKT_COMPRESSED_SIZE                          5
#define DHT11_LOGIC_MAX_LOW_CNTS                           95
#define DHT11_LOGIC_MAX_HIGH_CNTS                          150

/* ESP8266 - Definitions */
#define DTH11_DATA_PIN                                     4

/* Generic */
#define DHT11_LOGIC_1                                      1
#define DHT11_LOGIC_0                                      0
#define DHT11_TIMER_US                                     3000
#define DHT11_REPEAT_TMR                                   1
#define DHT11_ONE_SHOT_TMR                                 0

/* States */
typedef enum{
	ERROR,
	TIMEOUT,
	GOOD_RESPONSE,
	GOOD_DATA,
	CHECKSUM_PASS,
	CHECKSUM_ERROR
}DHT11_State;

/* Structure that will contain all the in-
 * coming, outgoing, state and packet-user
 * information. The application can get the
 * temperature and humidity data by reading
 * the 'temperature' and 'rh' variable once
 * the state is equal to 'CHECKSUM_PASS'.
 */
typedef struct{
	unsigned int start_sig_low_ms;
	unsigned int start_sig_high_us;
	unsigned char rh_integral_data;
	unsigned char rh_dec_data;
	unsigned char temp_integral_data;
	unsigned char temp_dec_data;
	unsigned char dht11_pkt[DTH11_PACKET_SIZE];
	unsigned char dht11_pkt_compressed[DHT11_PKT_COMPRESSED_SIZE];
	int comm_pin_no;
	unsigned char temperature;
	unsigned char rh;
	DHT11_State dht11_state;
}DHT11_T, *DHT11_Ptr;

/* Function  Prototypes - API to be used by the Application Programmer */
extern void ICACHE_FLASH_ATTR init_dht11_dev1(DHT11_Ptr dhtt_dev_Ptr, os_timer_t *esp_timer,
                int timer_us, bool repeat);

#endif
