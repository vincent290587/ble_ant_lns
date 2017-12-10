/*
 * backlighting.c
 *
 *  Created on: 10 déc. 2017
 *      Author: Vincent
 */

#include <stdbool.h>
#include "boards.h"
#include "nrf_assert.h"
#include "app_timer.h"
#include "app_error.h"
#include "backlighting.h"

APP_TIMER_DEF(m_back_timer);

static volatile bool m_timer_is_off = 0;


static void _backlight_callback(void* p_context) {

	m_timer_is_off = true;

}


void backlighting_init(void) {

	nrf_gpio_cfg_output(LED_PIN);

	// Create timer.
	uint32_t err_code = app_timer_create(&m_back_timer, APP_TIMER_MODE_SINGLE_SHOT, _backlight_callback);
	APP_ERROR_CHECK(err_code);

	m_timer_is_off = true;

}


void backlighting_set_control(sBacklightOrders* control) {

	ASSERT(control);

	if (control->state && control->freq) {

		// start to blink
		uint32_t ticks = APP_TIMER_TICKS(control->freq * 10);

		uint32_t err_code = app_timer_start(m_back_timer, ticks, NULL);
		APP_ERROR_CHECK(err_code);

		m_timer_is_off = false;

	} else if (control->state) {

		// turn on/off
		nrf_gpio_pin_set(LED_PIN);

	} else {

		nrf_gpio_pin_clear(LED_PIN);

	}

}
