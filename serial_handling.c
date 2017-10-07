/*
 * serial_handling.c
 *
 *  Created on: 9 août 2017
 *      Author: Vincent
 */

#include "ant.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_uart.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"

#include "nrf_drv_wdt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "serial_handling.h"


#define TAILLE_BUFFER 30

#define UART_TX_BUF_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 64                           /**< UART RX buffer size. */


#define RB_SIZE 100
static uint8_t read_byte[RB_SIZE];
static uint8_t glasses_payload[8];
static uint16_t status_byte = 0;
static uint16_t marque_byte = 0;



void uart_error_handle(app_uart_evt_t * p_event);
uint8_t encode (uint8_t byte);
void set_glasses_buffer ();


/**@brief Function for initializing the UART.
 */
void uart_init(void)
{
	uint32_t                     err_code;
	const app_uart_comm_params_t comm_params =
	{
			RX_PIN_NUMBER,
			TX_PIN_NUMBER,
			RTS_PIN_NUMBER,
			CTS_PIN_NUMBER,
			APP_UART_FLOW_CONTROL_DISABLED,
			false,
			UART_BAUDRATE_BAUDRATE_Baud115200
	};

	APP_UART_FIFO_INIT(&comm_params,
			UART_RX_BUF_SIZE,
			UART_TX_BUF_SIZE,
			uart_error_handle,
			APP_IRQ_PRIORITY_LOW,
			err_code);
	APP_ERROR_CHECK(err_code);
}


/**
 *
 */
void uart_error_handle(app_uart_evt_t * p_event)
{
	uint8_t read_byte = 0;

	if (p_event->evt_type == APP_UART_DATA_READY)
	{
		// get data
		while(app_uart_get(&read_byte) != NRF_SUCCESS) {;}

		if (encode(read_byte)) {
			set_glasses_buffer ();
		}

	}

	if (0) {
		if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
		{
			APP_ERROR_CHECK(p_event->data.error_communication);
		}
		else if (p_event->evt_type == APP_UART_FIFO_ERROR)
		{
			APP_ERROR_CHECK(p_event->data.error_code);
		}
	}
}



uint8_t encode (uint8_t byte) {

	switch (byte) {
	case '$':
		status_byte = 0;
		marque_byte = 1;
		memset(read_byte, 0, RB_SIZE);

		break;
	case '\r':
	case '\n':
	case '\0':
		if (status_byte < 1 || marque_byte == 0) return 0;
#ifdef USE_TUNES
		if (read_byte[0]=='T' && read_byte[1]=='U') {
			switch(read_byte[2]) {
			case '0':
				whichTune = 0;
				err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
				break;
			case '1':
				whichTune = 1;
				err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
				break;
			}

			status_byte = 0;
			marque_byte = 0;
			return 0;
		}
#endif
		if (status_byte == 8) {
			return 1;
		}

		marque_byte = 0;
		status_byte = 0;
		return 0;

	default:
		if (status_byte < RB_SIZE - 10 && marque_byte == 1) {
			read_byte[status_byte] = byte;
			status_byte++;
		} else {
			status_byte = 0;
			marque_byte = 0;
		}
		break;
	}

	return 0;
}



void set_glasses_buffer () {
	// pass-through
	uint8_t i;

	NRF_LOG_INFO("Glasses buffer set\r\n");

	memset(glasses_payload, 0, 8);

	for (i=0; i<4; i++) {
		if (read_byte[2*i] <= '9') {
			glasses_payload[i] = read_byte[2*i] - '0';
		} else if (read_byte[2*i] <= 'F') {
			// lettres majuscules
			glasses_payload[i] = 10 + read_byte[2*i] - 'A';
		} else {
			// lettres minuscules
			glasses_payload[i] = 10 + read_byte[2*i] - 'a';
		}

		glasses_payload[i] *= 16;

		if (read_byte[2*i+1] <= '9') {
			glasses_payload[i] += read_byte[2*i+1] - '0';
		} else if (read_byte[2*i+1] <= 'F') {
			// lettres majuscules
			glasses_payload[i] += 10 + read_byte[2*i+1] - 'A';
		} else {
			// lettres minuscules
			glasses_payload[i] += 10 + read_byte[2*i+1] - 'a';
		}

	}
}

/**
 *
 */
uint8_t *get_glasses_payload() {

	return glasses_payload;

}
