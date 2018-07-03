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
#include "backlighting.h"
#include "mk64f_parser.h"
#include "helper.h"
#include "bsp.h"
#include "fec.h"
#include "bsp_btn_ble.h"
#include "buttons_att.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_sdm.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_wdt.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define DEAD_BEEF           0xDEADBEEF                                  /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define APP_DELAY           APP_TIMER_TICKS(100)

#define SCHED_MAX_EVENT_DATA_SIZE      APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               20                                           /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE               10                                           /**< Maximum number of events in the scheduler queue. */
#endif

APP_TIMER_DEF(m_job_timer);

nrf_drv_wdt_channel_id m_channel_id;

extern void ble_ant_init(void);


static volatile bool job_to_do = false;


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
void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    assert_info_t assert_info =
    {
        .line_num    = line_num,
        .p_file_name = file_name,
    };
    app_error_fault_handler(NRF_FAULT_ID_SDK_ASSERT, 0, (uint32_t)(&assert_info));

#ifndef DEBUG_NRF
    NRF_LOG_WARNING("System reset");
    NVIC_SystemReset();
#else
    NRF_BREAKPOINT_COND;

    bool loop = true;
    while (loop) ;
#endif // DEBUG

    UNUSED_VARIABLE(assert_info);
}

/**
 *
 * @param id
 * @param pc
 * @param info
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_FLUSH();

    switch (id)
    {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            NRF_LOG_ERROR("ERROR %u [%s] at %s:%u",
                          p_info->err_code,
                          nrf_strerror_get(p_info->err_code),
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        default:
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
    }

#ifdef DEBUG_NRF
    NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.
#endif

}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for handling bsp events.
 */
static void bsp_evt_handler(bsp_event_t evt)
{

	switch (evt)
	{
	case BSP_EVENT_KEY_0:
		mk64f_toggle_line(eMk64fRightButton);
		break;
	case BSP_EVENT_KEY_1:
		mk64f_toggle_line(eMk64fCentralButton);
		break;
	case BSP_EVENT_KEY_2:
		mk64f_toggle_line(eMk64fLeftButton);
		break;
	default:
		return; // no implementation needed
	}

}


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
	if (NRF_PWR_MGMT_EVT_PREPARE_SYSOFF == event) {
		nrf_gpio_pin_clear(LDO_PIN);
		nrf_gpio_pin_set(LED_PIN);
		return true;
	}

	NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
	return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
	uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_evt_handler);
	APP_ERROR_CHECK(err_code);
}

/**
 *
 * @return 0
 */
int main(void)
{
	ret_code_t err_code;

//	nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);

	nrf_gpio_cfg_input(SHARP_CS, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_cfg_output(LDO_PIN);
	nrf_gpio_pin_clear(LDO_PIN);

	nrf_gpio_cfg_output(INT_PIN);
	nrf_gpio_pin_clear(INT_PIN);

	nrf_delay_ms(500);

	nrf_gpio_pin_set(LDO_PIN);
	nrf_gpio_cfg_output(LED_PIN);

	// Initialize.
    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();

	log_init();

	NRF_LOG_INFO("Init start");

	nrf_pwr_mgmt_init();

	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	// Initialize timer module
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	ant_timers_init();

	backlighting_init();

	buttons_leds_init();

	notifications_init(NEO_PIN);

	spis_init();

	// init BLE + ANT
	ble_ant_init();

	err_code = app_timer_create(&m_job_timer, APP_TIMER_MODE_REPEATED, timer_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_job_timer, APP_DELAY, NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("LNS central start");

	sNeopixelOrders neo_order;
	neo_order.event_type = eNeoEventNotify;
	neo_order.on_time = 0;
	neo_order.rgb[0] = 0xFF;
	neo_order.rgb[1] = 0x00;
	neo_order.rgb[2] = 0x00;
	notifications_setNotify(&neo_order);

	for (;;)
	{
		if (job_to_do) {
			job_to_do = false;

			NRF_LOG_DEBUG("Job");

//			nrf_gpio_pin_toggle(LED_PIN);

			notifications_tasks();

			buttons_att_tasks();

			roller_manager_tasks();

			nrf_drv_wdt_channel_feed(m_channel_id);
		}

		app_sched_execute();

		spis_tasks();

		backlighting_tasks();

		if (NRF_LOG_PROCESS() == false)
		{
			nrf_pwr_mgmt_run();
		}

	}
}

