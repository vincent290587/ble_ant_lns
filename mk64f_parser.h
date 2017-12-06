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

typedef struct {
	uint8_t weakness;
	uint8_t rgb[3];
	uint8_t on_time;
} sNeopixelOrders;

typedef struct {
	uint8_t  led;
	uint16_t millieme_avance;
} sGlassesOrders;

typedef struct {
	sNeopixelOrders neo_info;
	sGlassesOrders  glasses_info;
} sSpisRxInfoPage0;

typedef struct {
	sNeopixelOrders neo_info;
	sGlassesOrders  glasses_info;
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
