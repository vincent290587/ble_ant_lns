/*
 * serial_handling.c
 *
 *  Created on: 9 août 2017
 *      Author: Vincent
 */

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

#define NRF_LOG_MODULE_NAME "SH"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "serial_handling.h"
#include "ant.h"


#define TAILLE_BUFFER 30

#define UART_TX_BUF_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 64                           /**< UART RX buffer size. */


void uart_error_handle(app_uart_evt_t * p_event);


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


/**
 *
 */
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


