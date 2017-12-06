/*
 * spis_pages.h
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#ifndef SPIS_PAGES_H_
#define SPIS_PAGES_H_

#include "mk64f_parser.h"



#ifdef __cplusplus
extern "C" {
#endif

void spis_decode_rx_page(uint8_t *rx_buf, sSpisRxInfo *output);

#ifdef __cplusplus
}
#endif

#endif /* SPIS_PAGES_H_ */
