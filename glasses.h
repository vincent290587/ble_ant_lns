
#ifndef GLASSES_H_
#define GLASSES_H_

#include "mk64f_parser.h"


extern uint8_t m_glasses_payload[8];

#ifdef __cplusplus
extern "C" {
#endif

void set_glasses_buffer (sGlassesOrders* orders);

#ifdef __cplusplus
}
#endif

#endif /* GLASSES_H_ */
