/*
 * mk64f_parser.h
 *
 *  Created on: 6 déc. 2017
 *      Author: Vincent
 */

#ifndef MK64F_PARSER_H_
#define MK64F_PARSER_H_

#include <stdint.h>

typedef enum {
	eSpiRxPage0    = 0U,
	eSpiRxPage1    = 1U,
} eSpiRxPages;

////////////// RX SPIS

typedef struct {
	uint8_t weak;
	uint8_t rgb[3];
	uint8_t on_time;
} sNeopixelOrders;

typedef struct {
	uint8_t  led;
	uint16_t millieme_avance;
} sGlassesOrders;

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
	sNeopixelOrders neo_info;
	sGlassesOrders  glasses_info;
} sSpisRxInfoPage0;

typedef struct {
	uint8_t dummy;
} sSpisRxInfoPage1;

typedef union {
	sSpisRxInfoPage0 page0;
	sSpisRxInfoPage1 page1;
} sSpisRxInfo;

#ifdef __cplusplus
extern "C" {
#endif

void mk64f_parse_rx_info(sSpisRxInfo* output);

#ifdef __cplusplus
}
#endif

#endif /* MK64F_PARSER_H_ */
