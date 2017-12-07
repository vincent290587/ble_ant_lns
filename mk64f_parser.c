/*
 * mk64f_parser.c
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#include "glasses.h"
#include "notifications.h"
#include "mk64f_parser.h"

/** TODO
 *
 * @param output
 */
void mk64f_parse_rx_info(sSpisRxInfo* input) {

	switch (input->page_id) {
	case eSpiRxPage0:
		notifications_setNotify(&input->pages.page0.neo_info);
		set_glasses_buffer(&input->pages.page0.glasses_info);
		// TODO battery orders
		break;

	case eSpiRxPage1:

		break;

	default:
		break;
	}
}
