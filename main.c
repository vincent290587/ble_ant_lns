/*
 * App.cpp
 *
 *  Created on: 8 oct. 2017
 *      Author: Vincent
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "spis.h"
#include "notifications.h"
#include "helper.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define LDO_PIN             30
#define NEO_PIN             7
#define LED_PIN             14    // 8 ou 14
#define AT42_COUT           18
#define APP_DELAY           APP_TIMER_TICKS(100)

#define SCHED_MAX_EVENT_DATA_SIZE      APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               20                                           /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE               10                                           /**< Maximum number of events in the scheduler queue. */
#endif

APP_TIMER_DEF(m_job_timer);

extern void ble_ant_init(void);


volatile bool job_to_do = true;


/**
 * @brief Handler for timer events.
 */
void timer_event_handler(void* p_context)
{
	job_to_do = true;
}

/**
 *
 * @return
 */
int main(void)
{
	uint32_t err_code;

	nrf_gpio_cfg_input(AT42_COUT, NRF_GPIO_PIN_PULLUP);

	nrf_gpio_cfg_output(LDO_PIN);
	nrf_gpio_cfg_output(LED_PIN);

	nrf_gpio_pin_clear(LDO_PIN);

	nrf_delay_ms(500);

	nrf_gpio_pin_set(LDO_PIN);

	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	// init BLE + ANT
	ble_ant_init();

	spis_init();
	notifications_init(NEO_PIN);

	err_code = app_timer_create(&m_job_timer, APP_TIMER_MODE_REPEATED, timer_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_job_timer, APP_DELAY, NULL);
	APP_ERROR_CHECK(err_code);

    for (;;)
    {
    	app_sched_execute();

    	spis_tasks();

    	sd_app_evt_wait();

    	if (job_to_do) {
    		job_to_do = false;

    		// TODO job
    		notifications_tasks();
    	}
    }
}

