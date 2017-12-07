/*
 * mk64f_parser.h
 *
 *  Created on: 6 d�c. 2017
 *      Author: Vincent
 */

#ifndef MK64F_PARSER_H_
#define MK64F_PARSER_H_

#include <stdint.h>

typedef enum {
	eSpiRxPageInv  = 0xFF,
	eSpiRxPage0    = 0U,
	eSpiRxPage1    = 1U,
} eSpisRxPages;

////////////// RX SPIS

typedef struct {
	uint8_t event_type;
	uint8_t on_time;
	uint8_t rgb[3];
} sNeopixelOrders;

typedef struct {
	uint8_t  led;
	uint8_t  av_ent;
	uint8_t  av_dec;
} sGlassesOrders;

typedef struct {
	uint8_t  soc;
	uint16_t mv;
} sBatteryOrders;

////////////// TX SPIS

typedef struct {
	uint8_t bpm;
	uint16_t rr;
} sHrmInfo;

typedef struct {
	uint32_t cadence;
	uint32_t speed;
} sBscInfo;

typedef struct {
	uint32_t lat;
	uint32_t lon;
	int32_t ele;
} sLnsInfo;

////////////// RX PAGES

typedef struct {
	sBatteryOrders  batt_info;
	sGlassesOrders  glasses_info;
	sNeopixelOrders neo_info;
} sSpisRxInfoPage0;

typedef struct {
	uint8_t dummy;
} sSpisRxInfoPage1;

typedef union {
	sSpisRxInfoPage0 page0;
	sSpisRxInfoPage1 page1;
} sSpisRxPages;

typedef struct {
	eSpisRxPages page_id;
	sSpisRxPages pages;
} sSpisRxInfo;



#ifdef __cplusplus
extern "C" {
#endif

void mk64f_parse_rx_info(sSpisRxInfo* output);

#ifdef __cplusplus
}
#endif

#endif /* MK64F_PARSER_H_ */
