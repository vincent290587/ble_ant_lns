/*
 * mk64f_parser.c
 *
 *  Created on: 6 d�c. 2017
 *      Author: Vincent
 */

#include "glasses.h"
#include "fec.h"
#include "nrf_assert.h"
#include "notifications.h"
#include "mk64f_parser.h"

/** TODO
 *
 * @param output
 */
void mk64f_parse_rx_info(sSpisRxInfo* input) {

	ASSERT(input);

	switch (input->page_id) {
	case eSpiRxPage0:
		notifications_setNotify(&input->pages.page0.neo_info);
		set_glasses_buffer(&input->pages.page0.glasses_info);
		fec_set_control(&input->pages.page0.fec_info);
		// TODO battery orders
		break;

	case eSpiRxPage1:

		break;

	default:
		break;
	}
}
