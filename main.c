/*
 * App.cpp
 *
 *  Created on: 8 oct. 2017
 *      Author: Vincent
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ant.h"
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


#define DEAD_BEEF           0xDEADBEEF                                  /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

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


static volatile bool job_to_do = true;


/**
 * @brief Handler for timer events.
 */
void timer_event_handler(void* p_context)
{
	job_to_do = true;
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {

	if (error_code == NRF_SUCCESS) return;

	NRF_LOG_ERROR("Erreur: 0x%x ligne %u file %s !!\n", (unsigned int)error_code, (unsigned int)line_num, (uint32_t) p_file_name);

}

/**
 *
 * @param error_code
 */
void app_error_handler_bare(uint32_t error_code) {

	if (error_code == NRF_SUCCESS) return;

	NRF_LOG_ERROR("Erreur bare: 0x%x\n", error_code);


}

/**
 *
 * @param id
 * @param pc
 * @param info
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {

}

/**
 *
 * @param line_num
 * @param p_file_name
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**
 *
 * @return 0
 */
int main(void)
{
	ret_code_t err_code;

	nrf_gpio_cfg_input(AT42_COUT, NRF_GPIO_PIN_PULLUP);

	nrf_gpio_cfg_output(LDO_PIN);
	nrf_gpio_cfg_output(LED_PIN);

	nrf_gpio_pin_clear(LDO_PIN);

	nrf_delay_ms(500);

	nrf_gpio_pin_set(LDO_PIN);

	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	// Initialize.
	log_init();

	// Initialize timer module
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	ant_timers_init();

	spis_init();
	notifications_init(NEO_PIN);

	// init BLE + ANT
	ble_ant_init();


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

