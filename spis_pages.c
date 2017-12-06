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

	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START + 0, info->lat);
	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START + 4, info->lon);
	encode_uint32 (m_tx_buf + TX_BUFF_LNS_START + 8, (uint32_t) info->ele);
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

}

/**
 *
 * @param rx_buf
 * @param output
 */
static void spis_decode_page1(uint8_t *rx_buf, sSpisRxInfo *output) {

}

/** TODO
 *
 * @param rx_buf
 * @param output
 */
void spis_decode_rx_page(uint8_t *rx_buf, sSpisRxInfo *output) {

	switch (rx_buf[0]) {
	case eSpiRxPage0:
		spis_decode_page0(rx_buf, output);
		break;

	case eSpiRxPage1:
		spis_decode_page1(rx_buf, output);
		break;

	default:
		break;
	}


}

