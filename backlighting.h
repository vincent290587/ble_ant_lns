/*
 * backlighting.h
 *
 *  Created on: 10 d�c. 2017
 *      Author: Vincent
 */

#ifndef BACKLIGHTING_H_
#define BACKLIGHTING_H_

#include "mk64f_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

void backlighting_init(void);

void backlighting_tasks(void);

void backlighting_set_control(sBacklightOrders* control);

#ifdef __cplusplus
}
#endif

#endif /* BACKLIGHTING_H_ */
