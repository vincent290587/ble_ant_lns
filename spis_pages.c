/*
 * spis_pages.c
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#include "spis_pages.h"

/** TODO
 *
 * @param rx_buf
 * @param output
 */
void spis_decode_rx_page(uint8_t *rx_buf, sSpisRxInfo *output) {

	switch (rx_buf[0]) {
	case eSpiRxPage0:

		break;

	case eSpiRxPage1:

		break;

	default:
		break;
	}


}

