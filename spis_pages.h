/*
 * spis_pages.h
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#ifndef SPIS_PAGES_H_
#define SPIS_PAGES_H_

#include "mk64f_parser.h"

#define TX_BUFF_FLAGS_POS         0U

#define TX_BUFF_FLAGS_LNS_BIT     0U
#define TX_BUFF_FLAGS_HRM_BIT     1U
#define TX_BUFF_FLAGS_BSC_BIT     2U

#define TX_BUFF_FLAGS_SIZE        1U
#define TX_BUFF_LNS_SIZE          12U
#define TX_BUFF_HRM_SIZE          3U
#define TX_BUFF_BSC_SIZE          8U

#define TX_BUFF_HRM_START         (TX_BUFF_FLAGS_SIZE)
#define TX_BUFF_BSC_START         (TX_BUFF_HRM_START+TX_BUFF_HRM_SIZE)
#define TX_BUFF_LNS_START         (TX_BUFF_BSC_START+TX_BUFF_BSC_SIZE)

#ifdef __cplusplus
extern "C" {
#endif

// DECODE

void spis_decode_rx_page(uint8_t *rx_buf, sSpisRxInfo *output);

// ENCODE

void spis_encode_lns(sLnsInfo* info);

void spis_encode_hrm(sHrmInfo* info);

void spis_encode_bsc(sBscInfo* info);

#ifdef __cplusplus
}
#endif

#endif /* SPIS_PAGES_H_ */
