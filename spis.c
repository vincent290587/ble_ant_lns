/*
 * spis.c
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#include "sdk_config.h"
#include "nrf_drv_spis.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "spis.h"
#include "spis_pages.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */


uint8_t              m_tx_buf[SPIS_BUFFER_SIZE];           /**< TX buffer. */
static uint8_t       m_rx_buf[SPIS_BUFFER_SIZE];           /**< RX buffer. */

static volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */


/**
 * @brief SPIS user event handler.
 *
 * @param event
 */
static void spis_event_handler(nrf_drv_spis_event_t event)
{
	if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
	{
		spis_xfer_done = true;
		NRF_LOG_INFO(" Transfer completed. Received: %u bytes", event.rx_amount);
	}
}

/**
 *
 */
void spis_init(void) {
	// Enable the constant latency sub power mode to minimize the time it takes
	// for the SPIS peripheral to become active after the CSN line is asserted
	// (when the CPU is in sleep mode).
	NRF_POWER->TASKS_CONSTLAT = 1;

	NRF_LOG_INFO("SPIS init");

	nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
	spis_config.csn_pin               = SPIS_MISO_PIN;
	spis_config.miso_pin              = SPIS_MISO_PIN;
	spis_config.mosi_pin              = SPIS_MOSI_PIN;
	spis_config.sck_pin               = SPIS_SCK_PIN;

	APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));

	memset(m_rx_buf, 0, sizeof(m_rx_buf));
	spis_xfer_done = false;

	APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, SPIS_BUFFER_SIZE, m_rx_buf, SPIS_BUFFER_SIZE));
}

/**
 *
 */
void spis_tasks(void) {

	if (spis_xfer_done) {

		// decode info
		sSpisRxInfo decoded_info;
		spis_decode_rx_page(m_rx_buf, &decoded_info);

		// reset transfer
		memset(m_rx_buf, 0, sizeof(m_rx_buf));
		memset(m_tx_buf, 0, sizeof(m_tx_buf));

		spis_xfer_done = false;

		APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, SPIS_BUFFER_SIZE, m_rx_buf, SPIS_BUFFER_SIZE));

		// parse info and use it
		mk64f_parse_rx_info(&decoded_info);
	}

}
