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
#include "nrf_pwr_mgmt.h"
#include <string.h>
#include "spis.h"
#include "spis_pages.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPIS_INSTANCE 0 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */


uint8_t*             m_tx_buf;
static uint8_t       m_tx_dma_buf1[SPIS_BUFFER_SIZE];           /**< RX buffer. */
static uint8_t       m_tx_dma_buf2[SPIS_BUFFER_SIZE];           /**< RX buffer. */
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
//	sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);

	nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
	spis_config.csn_pin               = SPIS_CSN_PIN;
	spis_config.miso_pin              = SPIS_MISO_PIN;
	spis_config.mosi_pin              = SPIS_MOSI_PIN;
	spis_config.sck_pin               = SPIS_SCK_PIN;

	APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));

	memset(m_rx_buf, 0, sizeof(m_rx_buf));
	memset(m_tx_dma_buf1, 0, sizeof(m_tx_dma_buf1));
	memset(m_tx_dma_buf2, 0, sizeof(m_tx_dma_buf2));

	spis_xfer_done = false;

	m_tx_buf = m_tx_dma_buf1;

	APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, SPIS_BUFFER_SIZE, m_rx_buf, SPIS_BUFFER_SIZE));

	NRF_LOG_INFO("SPIS init");
}

/**
 *
 */
void spis_tasks(void) {

	if (spis_xfer_done) {

		nrf_pwr_mgmt_feed();

		// decode info
		sSpisRxInfo decoded_info;
		spis_decode_rx_page(m_rx_buf, &decoded_info);

		NRF_LOG_RAW_INFO("SPIS tx buffer:");
		for (uint8_t i = 0; i < SPIS_BUFFER_SIZE; i++) {
			NRF_LOG_RAW_INFO("%02X ", m_tx_buf[i]);
		}
		NRF_LOG_RAW_INFO("\r\n");

		// reset transfer
		memset(m_rx_buf, 0, SPIS_BUFFER_SIZE);

		spis_xfer_done = false;

		// invert buffers
		if (m_tx_buf == m_tx_dma_buf1) {
			m_tx_buf = m_tx_dma_buf2;
			APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_dma_buf1, SPIS_BUFFER_SIZE, m_rx_buf, SPIS_BUFFER_SIZE));
		} else {
			m_tx_buf = m_tx_dma_buf1;
			APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_dma_buf2, SPIS_BUFFER_SIZE, m_rx_buf, SPIS_BUFFER_SIZE));
		}

		// reset next TX buffer
		memset(m_tx_buf, 0, SPIS_BUFFER_SIZE);

		// parse info and use it
		mk64f_parse_rx_info(&decoded_info);
	}

}
