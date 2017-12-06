/*
 * spis_pages.c
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#include "spis_pages.h"


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

