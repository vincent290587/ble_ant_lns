/*
 * fec.h
 *
 *  Created on: 9 déc. 2017
 *      Author: Vincent
 */

#ifndef FEC_H_
#define FEC_H_

#include "ant_fec.h"
#include "nrf_sdh_ant.h"

/////////////  STRUCTS

typedef enum {
	eFecControlTargetPower,
	eFecControlSlope,
} eFecControlType;

typedef struct {
	uint16_t target_power_w;
} sControlTargetPower;

typedef struct {
	float slope_ppc;
	float rolling_resistance;
} sControlSlope;

typedef union {
	sControlTargetPower power_control;
	sControlSlope       slope_control;
} uControlPages;

typedef struct {
	eFecControlType type;
	uControlPages   data;
} sFecControl;

extern sFecControl              m_fec_control;

extern ant_fec_profile_t        m_ant_fec;

extern ant_fec_message_layout_t m_fec_message_payload;

/////////////  FUNCTIONS

void fec_init(void);

void ant_evt_fec (ant_evt_t * p_ant_evt);

void ant_fec_evt_handler(ant_fec_profile_t * p_profile, ant_fec_evt_t event);

#endif /* FEC_H_ */
