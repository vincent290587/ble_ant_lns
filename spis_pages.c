/*
 * spis_pages.c
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#include "helper.h"
#include "spis.h"
#include "spis_pages.h"


/**
 *
 * @param info
 */
void spis_encode_lns(sLnsInfo* info) {
	m_tx_buf[TX_BUFF_FLAGS_POS] |= 1 << TX_BUFF_FLAGS_LNS_BIT;

	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START +  0, (uint32_t) info->lat);
	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START +  4, (uint32_t) info->lon);
	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START +  8, (uint32_t) info->ele);
	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START + 12, info->secj);
}

/**
 *
 * @param info
 */
void spis_encode_hrm(sHrmInfo* info) {
	m_tx_buf[TX_BUFF_FLAGS_POS] |= 1 << TX_BUFF_FLAGS_HRM_BIT;

	m_tx_buf[TX_BUFF_HRM_START] = info->bpm;
	encode_uint16 (m_tx_buf + TX_BUFF_HRM_START + 1, info->rr);
}

/**
 *
 * @param info
 */
void spis_encode_bsc(sBscInfo* info) {
	m_tx_buf[TX_BUFF_FLAGS_POS] |= 1 << TX_BUFF_FLAGS_BSC_BIT;

	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START + 0, info->cadence);
	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START + 4, info->speed);
}

/**
 *
 * @param rx_buf
 * @param output
 */
static void spis_decode_page0(uint8_t *rx_buf, sSpisRxInfo *output) {
	output->page_id = eSpiRxPage0;

	output->pages.page0.batt_info.soc = rx_buf[RX_BUFF_BAT_START];
	output->pages.page0.batt_info.mv  = decode_uint16 (rx_buf + RX_BUFF_BAT_START + 1U);

	output->pages.page0.glasses_info.led    = rx_buf[RX_BUFF_GLA_START];
	output->pages.page0.glasses_info.av_ent = rx_buf[RX_BUFF_GLA_START + 1U];
	output->pages.page0.glasses_info.av_dec = rx_buf[RX_BUFF_GLA_START + 2U];

	output->pages.page0.neo_info.event_type = rx_buf[RX_BUFF_NEO_START];
	output->pages.page0.neo_info.on_time    = rx_buf[RX_BUFF_NEO_START + 1U];
	output->pages.page0.neo_info.rgb[0]     = rx_buf[RX_BUFF_NEO_START + 2U];
	output->pages.page0.neo_info.rgb[1]     = rx_buf[RX_BUFF_NEO_START + 3U];
	output->pages.page0.neo_info.rgb[2]     = rx_buf[RX_BUFF_NEO_START + 4U];
}

/**
 *
 * @param rx_buf
 * @param output
 */
static void spis_decode_page1(uint8_t *rx_buf, sSpisRxInfo *output) {
	output->page_id = eSpiRxPage1;
}

/** TODO
 *
 * @param rx_buf
 * @param output
 */
void spis_decode_rx_page(uint8_t *rx_buf, sSpisRxInfo *output) {

	switch (rx_buf[RX_BUFF_PAGE_POS]) {
	case eSpiRxPage0:
		spis_decode_page0(rx_buf, output);
		break;

	case eSpiRxPage1:
		spis_decode_page1(rx_buf, output);
		break;

	default:
		output->page_id = eSpiRxPageInv;
		break;
	}


}

