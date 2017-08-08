/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_apple_notification_main main.c
 * @{
 * @ingroup ble_sdk_app_apple_notification
 * @brief Apple Notification Client Sample Application main file. Disclaimer: 
 * This client implementation of the Apple Notification Center Service can and 
 * will be changed at any time by Nordic Semiconductor ASA.
 *
 * Server implementations such as the ones found in iOS can be changed at any 
 * time by Apple and may cause this client implementation to stop working.
 *
 * This file contains the source code for a sample application using the Apple 
 * Notification Center Service Client.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ant.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "app_uart.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"

#include "nrf_drv_wdt.h"

#include "ant_stack_config.h"
#include "ant_key_manager.h"
#include "ant_hrm.h"
#include "ant_bsc.h"
#include "ant_interface.h"

#define NRF_LOG_MODULE_NAME "M_ANT"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define TAILLE_BUFFER 30

#define GPIO_BUTTON                     30

#define ANT_DELAY                       APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)


/** @snippet [ANT BSC RX Instance] */
#define WHEEL_CIRCUMFERENCE         2070                                                            /**< Bike wheel circumference [mm] */
#define BSC_EVT_TIME_FACTOR         1024                                                            /**< Time unit factor for BSC events */
#define BSC_RPM_TIME_FACTOR         60                                                              /**< Time unit factor for RPM unit */
#define BSC_MS_TO_KPH_NUM           36                                                              /**< Numerator of [m/s] to [kph] ratio */
#define BSC_MS_TO_KPH_DEN           10                                                              /**< Denominator of [m/s] to [kph] ratio */
#define BSC_MM_TO_M_FACTOR          1000                                                            /**< Unit factor [m/s] to [mm/s] */
#define BSC_SPEED_UNIT_FACTOR       (BSC_MS_TO_KPH_DEN * BSC_MM_TO_M_FACTOR)                        /**< Speed unit factor */
#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM) /**< Coefficient for speed value calculation */
#define CADENCE_COEFFICIENT         (BSC_EVT_TIME_FACTOR * BSC_RPM_TIME_FACTOR)                     /**< Coefficient for cadence value calculation */


#define WILDCARD_TRANSMISSION_TYPE      0x00

#define HRM_CHANNEL_NUMBER              0x00  
#define HRM_DEVICE_NUMBER               0x0D22    /**< Device Number. */

#define BSC_CHANNEL_NUMBER              0x01     
#define BSC_DEVICE_NUMBER               0xB02B    /**< Device Number. */    
#define BSC_DEVICE_TYPE                 0x79

#define ANTPLUS_NETWORK_NUMBER          0x00                                           /**< Network number. */

#define UART_TX_BUF_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 64                           /**< UART RX buffer size. */



APP_TIMER_DEF(m_sec_hrm);
APP_TIMER_DEF(m_sec_bsc);

nrf_drv_wdt_channel_id wdt_channel_id;


void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event);


BSC_DISP_CHANNEL_CONFIG_DEF(m_ant_bsc,
                            BSC_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            BSC_DEVICE_TYPE,
                            BSC_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            BSC_MSG_PERIOD_4Hz);
BSC_DISP_PROFILE_CONFIG_DEF(m_ant_bsc, ant_bsc_evt_handler);
ant_bsc_profile_t m_ant_bsc;


static int32_t accumulated_s_rev_cnt, previous_s_evt_cnt, prev_s_accumulated_rev_cnt,
               accumulated_s_evt_time, previous_s_evt_time, prev_s_accumulated_evt_time = 0;

static int32_t accumulated_c_rev_cnt, previous_c_evt_cnt, prev_c_accumulated_rev_cnt,
               accumulated_c_evt_time, previous_c_evt_time, prev_c_accumulated_evt_time = 0;


/** @snippet [ANT HRM RX Instance] */
HRM_DISP_CHANNEL_CONFIG_DEF(m_ant_hrm,
                            HRM_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            HRM_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            HRM_MSG_PERIOD_4Hz);
ant_hrm_profile_t           m_ant_hrm;


static uint8_t is_hrm_init = 0;
static uint8_t is_cad_init = 0;




void uart_error_handle(app_uart_evt_t * p_event)
{
    uint8_t read_byte = 0;
  
    if (p_event->evt_type == APP_UART_DATA_READY)
    {
      // get data
      while(app_uart_get(&read_byte) != NRF_SUCCESS) {;}
      
    }
  
    if (0) {
      if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
      {
        APP_ERROR_CHECK(p_event->data.error_communication);
      }
      else if (p_event->evt_type == APP_UART_FIFO_ERROR)
      {
        APP_ERROR_CHECK(p_event->data.error_code);
      }
    }
}


static void hrm_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_hrm_disp_open(&m_ant_hrm);
  APP_ERROR_CHECK(err_code);
}

static void bsc_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_bsc_disp_open(&m_ant_bsc);
  APP_ERROR_CHECK(err_code);
}



void ant_evt_bsc (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;
	

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_cad_init) {
                    sd_ant_channel_id_get (BSC_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,CAD 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_cad_init = 1;
                  }
                  ant_bsc_disp_evt_handler(&m_ant_bsc, p_ant_evt);
				  NRF_LOG_INFO("HRM RX\r\n");
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_RX_SEARCH_TIMEOUT:
									break;
							case EVENT_CHANNEL_CLOSED:
                  is_cad_init = 0;
                  //err_code = app_timer_stop(m_sec_bsc);
				err_code = app_timer_start(m_sec_bsc, ANT_DELAY, NULL);
				break;
					}
          
    APP_ERROR_CHECK(err_code);
}

void ant_evt_hrm (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_hrm_init) {
                    sd_ant_channel_id_get (HRM_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,HRM 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_hrm_init = 1;
                  }
                  ant_hrm_disp_evt_handler(&m_ant_hrm, p_ant_evt);
				  NRF_LOG_INFO("HRM RX\r\n");
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_RX_SEARCH_TIMEOUT:
									break;
							case EVENT_CHANNEL_CLOSED:
                  is_hrm_init = 0;
                  //err_code = app_timer_stop(m_sec_hrm);
				  err_code = app_timer_start(m_sec_hrm, ANT_DELAY, NULL);
                  break;
					}
          
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{

	  switch(p_ant_evt->channel) {
			case HRM_CHANNEL_NUMBER:
				ant_evt_hrm (p_ant_evt);
				break;
      
			case BSC_CHANNEL_NUMBER:
				ant_evt_bsc (p_ant_evt);
				break;
      
			default:
				break;
		}

}


__STATIC_INLINE float calculate_speed(int32_t rev_cnt, int32_t evt_time)
{
    static float computed_speed   = 0;

    if (rev_cnt != previous_s_evt_cnt)
    {
        accumulated_s_rev_cnt  += rev_cnt - previous_s_evt_cnt;
        accumulated_s_evt_time += evt_time - previous_s_evt_time;

        /* Process rollover */
        if (previous_s_evt_cnt > rev_cnt)
        {
            accumulated_s_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_s_evt_time > evt_time)
        {
            accumulated_s_evt_time += UINT16_MAX + 1;
        }

        previous_s_evt_cnt  = rev_cnt;
        previous_s_evt_time = evt_time;

        computed_speed   = SPEED_COEFFICIENT * (accumulated_s_rev_cnt  - prev_s_accumulated_rev_cnt);
        computed_speed   /= (accumulated_s_evt_time - prev_s_accumulated_evt_time);
        computed_speed   /= BSC_SPEED_UNIT_FACTOR;

        prev_s_accumulated_rev_cnt  = accumulated_s_rev_cnt;
        prev_s_accumulated_evt_time = accumulated_s_evt_time;
    }

    return (float)computed_speed;
}

static uint32_t calculate_cadence(int32_t rev_cnt, int32_t evt_time)
{
    static uint32_t computed_cadence = 0;

    if (rev_cnt != previous_c_evt_cnt)
    {
        accumulated_c_rev_cnt  += rev_cnt - previous_c_evt_cnt;
        accumulated_c_evt_time += evt_time - previous_c_evt_time;

        /* Process rollover */
        if (previous_c_evt_cnt > rev_cnt)
        {
            accumulated_c_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_c_evt_time > evt_time)
        {
            accumulated_c_evt_time += UINT16_MAX + 1;
        }

        previous_c_evt_cnt  = rev_cnt;
        previous_c_evt_time = evt_time;

        computed_cadence = CADENCE_COEFFICIENT *
                           (accumulated_c_rev_cnt  - prev_c_accumulated_rev_cnt) /
                           (accumulated_c_evt_time - prev_c_accumulated_evt_time);

        prev_c_accumulated_rev_cnt  = accumulated_c_rev_cnt;
        prev_c_accumulated_evt_time = accumulated_c_evt_time;
    }

    return (uint32_t) computed_cadence;
}




void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event)
{
    unsigned int _cadence;
	  float _speed;
  
  
    switch (event)
    {
        case ANT_BSC_PAGE_0_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_2_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_3_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_4_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_5_UPDATED:
            /* Log computed value */
            break;

        case ANT_BSC_COMB_PAGE_0_UPDATED:
          
            _speed = calculate_speed(p_profile->BSC_PROFILE_speed_rev_count, p_profile->BSC_PROFILE_speed_event_time);
        
            _cadence = calculate_cadence(p_profile->BSC_PROFILE_cadence_rev_count, p_profile->BSC_PROFILE_cadence_event_time);
                         
			printf("$CAD,%u,%u\n\r", (unsigned int)_cadence, (unsigned int)(100.*_speed));
				
            NRF_LOG_INFO( "Evenement BSC speed=%u cad=%u\n", (unsigned int)_cadence, (unsigned int)(100.*_speed));

            break;

        default:
            break;
    }
}



/**@brief Handle received ANT+ HRM data.
 * 
 * @param[in]   p_profile       Pointer to the ANT+ HRM profile instance.
 * @param[in]   event           Event related with ANT+ HRM Display profile. 
 */
static void ant_hrm_evt_handler(ant_hrm_profile_t * p_profile, ant_hrm_evt_t event)
{
	static uint32_t     s_previous_beat_count  = 0;    // Heart beat count from previously received page
    uint16_t            beat_time              = p_profile->page_0.beat_time;
    uint32_t            beat_count             = p_profile->page_0.beat_count;
    uint32_t            computed_heart_rate    = p_profile->page_0.computed_heart_rate;
	  uint16_t rrInterval;
    uint16_t rrInterval_ms;
  
    switch (event)
    {
        case ANT_HRM_PAGE_0_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_1_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_2_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_3_UPDATED:
            break;
        case ANT_HRM_PAGE_4_UPDATED:
          
            NRF_LOG_INFO( "Evenement HR BPM=%u\n", (unsigned int)computed_heart_rate);

            // Ensure that there is only one beat between time intervals.
            if ((beat_count - s_previous_beat_count) == 1)
            {
                uint16_t prev_beat = p_profile->page_4.prev_beat;
							
							  rrInterval = (beat_time - prev_beat);
                rrInterval_ms = rrInterval * 1000. / 1024.;
							
				printf("$HRM,%u,%u\n\r",
                      (unsigned int)computed_heart_rate,
                      (unsigned int)rrInterval_ms);
                
                // Subtracting the event time gives the R-R interval
                //ble_hrs_rr_interval_add(&m_hrs, beat_time - prev_beat);
                NRF_LOG_INFO( "Evenement HR RR=%u\n", (unsigned int)rrInterval_ms);
											
            }
            
            s_previous_beat_count = beat_count;
            break;

        default:
            break;
    }


}




/**@brief Function for initializing the timer module.
 */
void ant_timers_init(void)
{
    uint32_t err_code;

    err_code = app_timer_create(&m_sec_hrm, APP_TIMER_MODE_SINGLE_SHOT, hrm_connect);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_bsc, APP_TIMER_MODE_SINGLE_SHOT, bsc_connect);
    APP_ERROR_CHECK(err_code);
  
}

/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
  
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            printf("$BTN,0\n\r");
            break;
        case BSP_EVENT_KEY_1:
            printf("$BTN,1\n\r");
            break;
        case BSP_EVENT_KEY_2:
            printf("$BTN,2\n\r");
            break;
        default:
            return; // no implementation needed
    }

}


/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{

}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ant_stack_init(void)
{
    uint32_t err_code;
  
    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);
    
}

/**@brief Function for HRM profile initialization.
 *
 * @details Initializes the HRM profile and open ANT channel.
 */
static void ant_profile_setup(void)
{
/** @snippet [ANT HRM RX Profile Setup] */
    uint32_t err_code;
        
    // HRM
    err_code = ant_hrm_disp_init(&m_ant_hrm,
                                 HRM_DISP_CHANNEL_CONFIG(m_ant_hrm),
                                 ant_hrm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = ant_hrm_disp_open(&m_ant_hrm);
    APP_ERROR_CHECK(err_code);

    
    // CAD
    err_code = ant_bsc_disp_init(&m_ant_bsc,
                                 BSC_DISP_CHANNEL_CONFIG(m_ant_bsc),
                                 BSC_DISP_PROFILE_CONFIG(m_ant_bsc));
    APP_ERROR_CHECK(err_code);
	
    err_code = ant_bsc_disp_open(&m_ant_bsc);
    APP_ERROR_CHECK(err_code);

/** @snippet [ANT HRM RX Profile Setup] */
}



/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
    //nrf_gpio_cfg_output(GPIO_BUTTON);
	// pin to ground
    //nrf_gpio_pin_clear(GPIO_BUTTON);

    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_evt_handler);

    APP_ERROR_CHECK(err_code);

}



/**@brief Function for initializing the UART.
 */
void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry.
 */
int ant_setup_start(void)
{
	buttons_leds_init();

    ant_profile_setup();
	
	return 0;
}


/**
 * @}
 */

