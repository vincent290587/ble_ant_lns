/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**
 * @brief BLE Heart Rate Collector application main file.
 *
 * This file contains the source code for a sample heart rate collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "ble_radio_notification.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "peer_manager.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "helper.h"
#include "ble_bas_c.h"
#include "ble_lns_c.h"
#include "ant.h"
#include "glasses.h"
#include "spis_pages.h"
#include "neopixel.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define BLE_DEVICE_NAME             "myStrava"

#define APP_BLE_CONN_CFG_TAG        1                                   /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_BLE_OBSERVER_PRIO       1                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< Applications' SoC observer priority. You shoulnd't need to modify this value. */


#define SEC_PARAM_BOND              1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              0                                   /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS          0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                   /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                  /**< Maximum encryption key size in octets. */

#define SCAN_INTERVAL               0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                              /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY               0                                   /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID                 BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE         /**< Target device name that application is looking for. */



/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
		do                           \
		{                            \
			(*(DST))   = (SRC)[1];   \
			(*(DST)) <<= 8;          \
			(*(DST))  |= (SRC)[0];   \
		} while (0)


/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
	uint8_t  * p_data;      /**< Pointer to data. */
	uint16_t   data_len;    /**< Length of data. */
} data_t;


BLE_LNS_C_DEF(m_ble_lns_c);                                             /**< Structure used to identify the heart rate client module. */
BLE_BAS_C_DEF(m_ble_bas_c);                                             /**< Structure used to identify the Battery Service client module. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                    /**< DB discovery module instance. */

static ble_gap_scan_params_t m_scan_param;                 /**< Scan parameters requested for scanning and connection. */
//static uint16_t              m_conn_handle;                /**< Current connection handle. */
static bool                  m_whitelist_disabled;         /**< True if whitelist has been temporarily disabled. */
static bool                  m_memory_access_in_progress;  /**< Flag to keep track of ongoing operations on persistent memory. */

static bool                  m_retry_db_disc;              /**< Flag to keep track of whether the DB discovery should be retried. */
static uint16_t              m_pending_db_disc_conn = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for which the DB discovery is retried. */


/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
		(uint16_t)MIN_CONNECTION_INTERVAL,  /**< Minimum connection. */
		(uint16_t)MAX_CONNECTION_INTERVAL,  /**< Maximum connection. */
		(uint16_t)SLAVE_LATENCY,            /**< Slave latency. */
		(uint16_t)SUPERVISION_TIMEOUT       /**< Supervision time-out. */
};

/**@brief Names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static const char m_target_periph_name[] = "";          /**< If you want to connect to a peripheral using a given advertising name, type its name here. */
static bool  is_connect_per_addr = false;               /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */
static const ble_gap_addr_t m_target_periph_addr =
{
		/* Possible values for addr_type:
       BLE_GAP_ADDR_TYPE_PUBLIC,
       BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
       BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE. */
		.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
		.addr      = {0x8D, 0xFE, 0x23, 0x86, 0x77, 0xD9}
};


static void scan_start(void);




/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
	ble_lns_c_on_db_disc_evt(&m_ble_lns_c, p_evt);
	ble_bas_on_db_disc_evt(&m_ble_bas_c, p_evt);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
	ret_code_t err_code;

	switch (p_evt->evt_id)
	{
	case PM_EVT_BONDED_PEER_CONNECTED:
	{
		NRF_LOG_INFO("Connected to a previously bonded device.");
	} break;

	case PM_EVT_CONN_SEC_SUCCEEDED:
	{
		NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
				ble_conn_state_role(p_evt->conn_handle),
				p_evt->conn_handle,
				p_evt->params.conn_sec_succeeded.procedure);
	} break;

	case PM_EVT_CONN_SEC_FAILED:
	{
		/* Often, when securing fails, it shouldn't be restarted, for security reasons.
		 * Other times, it can be restarted directly.
		 * Sometimes it can be restarted, but only after changing some Security Parameters.
		 * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
		 * Sometimes it is impossible, to secure the link, or the peer device does not support it.
		 * How to handle this error is highly application dependent. */
	} break;

	case PM_EVT_CONN_SEC_CONFIG_REQ:
	{
		// Reject pairing request from an already bonded peer.
		pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
		pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
	} break;

	case PM_EVT_STORAGE_FULL:
	{
		// Run garbage collection on the flash.
		err_code = fds_gc();
		if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
		{
			// Retry.
		}
		else
		{
			APP_ERROR_CHECK(err_code);
		}
	} break;

	case PM_EVT_PEERS_DELETE_SUCCEEDED:
	{
		// Bonds are deleted. Start scanning.
		scan_start();
	} break;

	case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
	{
		// The local database has likely changed, send service changed indications.
		pm_local_database_has_changed();
	} break;

	case PM_EVT_PEER_DATA_UPDATE_FAILED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
	} break;

	case PM_EVT_PEER_DELETE_FAILED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
	} break;

	case PM_EVT_PEERS_DELETE_FAILED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
	} break;

	case PM_EVT_ERROR_UNEXPECTED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
	} break;

	case PM_EVT_CONN_SEC_START:
	case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
	case PM_EVT_PEER_DELETE_SUCCEEDED:
	case PM_EVT_LOCAL_DB_CACHE_APPLIED:
	case PM_EVT_SERVICE_CHANGED_IND_SENT:
	case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
	default:
		break;
	}
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
	uint32_t  index = 0;
	uint8_t * p_data;

	p_data = p_advdata->p_data;

	while (index < p_advdata->data_len)
	{
		uint8_t field_length = p_data[index];
		uint8_t field_type   = p_data[index + 1];

		if (field_type == type)
		{
			p_typedata->p_data   = &p_data[index + 2];
			p_typedata->data_len = field_length - 1;
			return NRF_SUCCESS;
		}
		index += field_length + 1;
	}
	return NRF_ERROR_NOT_FOUND;
}



/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, const char * name_to_find)
{
	uint32_t err_code;
	data_t   adv_data;
	data_t   dev_name;

	// Initialize advertisement report for parsing
	adv_data.p_data     = (uint8_t *)p_adv_report->data;
	adv_data.data_len   = p_adv_report->dlen;


	//search for advertising names
	err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
			&adv_data,
			&dev_name);
	if (err_code == NRF_SUCCESS)
	{
		if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len )== 0)
		{
			return true;
		}
	}
	else
	{
		// Look for the short local name if it was not found as complete
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
				&adv_data,
				&dev_name);
		if (err_code != NRF_SUCCESS)
		{
			return false;
		}
		if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len )== 0)
		{
			return true;
		}
	}
	return false;
}


/**@brief Function for searching a given addr in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * addr in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   p_addr   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_peer_addr(const ble_gap_evt_adv_report_t *p_adv_report, const ble_gap_addr_t * p_addr)
{
	if (p_addr->addr_type == p_adv_report->peer_addr.addr_type)
	{
		if (memcmp(p_addr->addr, p_adv_report->peer_addr.addr, sizeof(p_adv_report->peer_addr.addr)) == 0)
		{
			return true;
		}
	}
	return false;
}


/**@brief Function for searching a UUID in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * UUID in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   uuid_to_find   UUIID to search.
 * @return   true if the given UUID was found, false otherwise.
 */
static bool find_adv_uuid(const ble_gap_evt_adv_report_t *p_adv_report, const uint16_t uuid_to_find)
{
	uint32_t err_code;
	data_t   adv_data;
	data_t   type_data;

	// Initialize advertisement report for parsing.
	adv_data.p_data     = (uint8_t *)p_adv_report->data;
	adv_data.data_len   = p_adv_report->dlen;

	err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
			&adv_data,
			&type_data);

	if (err_code != NRF_SUCCESS)
	{
		// Look for the services in 'complete' if it was not found in 'more available'.
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
				&adv_data,
				&type_data);

		if (err_code != NRF_SUCCESS)
		{
			// If we can't parse the data, then exit.
			return false;
		}
	}

	// Verify if any UUID match the given UUID.
	for (uint32_t u_index = 0; u_index < (type_data.data_len / sizeof(uint16_t)); u_index++)
	{
		uint16_t extracted_uuid;

		UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[u_index * sizeof(uint16_t)]);

		if (extracted_uuid == uuid_to_find)
		{
			return true;
		}
	}
	return false;
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
	ret_code_t            err_code;
	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
	{
		NRF_LOG_INFO("Connected.");
		m_pending_db_disc_conn = p_ble_evt->evt.gap_evt.conn_handle;
		m_retry_db_disc = false;
		// Discover peer's services.
		err_code = ble_db_discovery_start(&m_db_disc, m_pending_db_disc_conn);
		if (err_code == NRF_ERROR_BUSY)
		{
			NRF_LOG_INFO("ble_db_discovery_start() returned busy, will retry later.");
			m_retry_db_disc = true;
		}
		else
		{
			APP_ERROR_CHECK(err_code);
		}

		err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
		APP_ERROR_CHECK(err_code);

		if (ble_conn_state_n_centrals() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
		{
			scan_start();
		}
	} break;

	case BLE_GAP_EVT_ADV_REPORT:
	{
		bool do_connect = false;
		if (is_connect_per_addr)
		{
			if (find_peer_addr(&p_gap_evt->params.adv_report, &m_target_periph_addr))
			{
				NRF_LOG_INFO("Address match send connect_request.");
				do_connect = true;
			}
		}
		else if (strlen(m_target_periph_name) != 0)
		{
			if (find_adv_name(&p_gap_evt->params.adv_report, m_target_periph_name))
			{
				do_connect = true;
				NRF_LOG_INFO("Name match send connect_request.");
			}
		}
		else
		{
			if (find_adv_uuid(&p_gap_evt->params.adv_report, TARGET_UUID))
			{
				do_connect = true;
				NRF_LOG_INFO("UUID match send connect_request.");
			}
		}
		if (do_connect)
		{
			// Stop scanning.
			(void) sd_ble_gap_scan_stop();

#if (NRF_SD_BLE_API_VERSION <= 2)
			m_scan_param.selective = 0;
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
			m_scan_param.use_whitelist = 0;
#endif

			// Initiate connection.
			err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
					&m_scan_param,
					&m_connection_param,
					APP_BLE_CONN_CFG_TAG);

			m_whitelist_disabled = false;

			if (err_code != NRF_SUCCESS)
			{
				NRF_LOG_ERROR("Connection Request Failed, reason %d.", err_code);
			}
		}
	} break; // BLE_GAP_EVT_ADV_REPORT

	case BLE_GAP_EVT_DISCONNECTED:
	{
		NRF_LOG_INFO("Disconnected, reason 0x%x.",
				p_ble_evt->evt.gap_evt.params.disconnected.reason);

		err_code = bsp_indication_set(BSP_INDICATE_IDLE);
		APP_ERROR_CHECK(err_code);

		// Reset DB discovery structure.
		memset(&m_db_disc, 0 , sizeof (m_db_disc));

		if (ble_conn_state_n_centrals() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
		{
			scan_start();
		}
	} break;

	case BLE_GAP_EVT_TIMEOUT:
	{
		if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
		{
			NRF_LOG_DEBUG("Scan timed out.");
			scan_start();
		}
		else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
		{
			NRF_LOG_INFO("Connection Request timed out.");
		}
	} break;

	case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		// Accepting parameters requested by peer.
		err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
				&p_gap_evt->params.conn_param_update_request.conn_params);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
	{
		NRF_LOG_DEBUG("PHY update request.");
		ble_gap_phys_t const phys =
		{
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
		};
		err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
		APP_ERROR_CHECK(err_code);
	} break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		NRF_LOG_DEBUG("GATT Client Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
				BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		NRF_LOG_DEBUG("GATT Server Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
				BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		break;
	}
}


/**@brief SoftDevice SoC event handler.
 *
 * @param[in]   evt_id      SoC event.
 * @param[in]   p_context   Context.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
	switch (evt_id)
	{
	case NRF_EVT_FLASH_OPERATION_SUCCESS:
		/* fall through */
	case NRF_EVT_FLASH_OPERATION_ERROR:

		if (m_memory_access_in_progress)
		{
			m_memory_access_in_progress = false;
			scan_start();
		}
		break;

	default:
		// No implementation needed.
		break;
	}
}


void ble_radio_callback_handler(bool radio_active)
{
	if (radio_active == false)
	{
		neopixel_radio_callback_handler(radio_active);
	}
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	ASSERT(nrf_sdh_is_enabled());

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register handlers for BLE and SoC events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
	NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);

	// radio callback to write to the neopixels right ;-)
	err_code = ble_radio_notification_init(7, NRF_RADIO_NOTIFICATION_DISTANCE_1740US, ble_radio_callback_handler);
	APP_ERROR_CHECK(err_code);

	// set name
	ble_gap_conn_sec_mode_t sec_mode; // Struct to store security parameters
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	/*Get this device name*/
	uint8_t device_name[20];
	memset(device_name, 0, sizeof(device_name));
	memcpy(device_name, BLE_DEVICE_NAME, strlen(BLE_DEVICE_NAME));
	err_code = sd_ble_gap_device_name_set(&sec_mode, device_name, strlen(BLE_DEVICE_NAME));
}



/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(void)
{
	ble_gap_sec_params_t sec_param;
	ret_code_t err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond           = SEC_PARAM_BOND;
	sec_param.mitm           = SEC_PARAM_MITM;
	sec_param.lesc           = SEC_PARAM_LESC;
	sec_param.keypress       = SEC_PARAM_KEYPRESS;
	sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob            = SEC_PARAM_OOB;
	sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
	sec_param.kdist_own.id   = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id  = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Heart Rate Collector Handler.
 */
static void lns_c_evt_handler(ble_lns_c_t * p_lns_c, ble_lns_c_evt_t * p_lns_c_evt)
{
	uint32_t err_code;

	NRF_LOG_DEBUG("LNS event: 0x%X\r\n", p_lns_c_evt->evt_type);

	switch (p_lns_c_evt->evt_type)
	{
	case BLE_LNS_C_EVT_DISCOVERY_COMPLETE:
		err_code = ble_lns_c_handles_assign(p_lns_c ,
				p_lns_c_evt->conn_handle,
				&p_lns_c_evt->params.peer_db);
		APP_ERROR_CHECK(err_code);

		// Initiate bonding.
		err_code = pm_conn_secure(p_lns_c_evt->conn_handle, false);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}

		// LNS service discovered. Enable notification of LNS.
		err_code = ble_lns_c_pos_notif_enable(p_lns_c);
		APP_ERROR_CHECK(err_code);

		NRF_LOG_DEBUG("LNS service discovered.");
		break;

	case BLE_LNS_C_EVT_LNS_NOTIFICATION:
	{
		sLnsInfo lns_info;

		lns_info.lat = p_lns_c_evt->params.lns.lat;
		lns_info.lon = p_lns_c_evt->params.lns.lon;
		lns_info.ele = 0;
		lns_info.speed = 0;

		NRF_LOG_INFO("Latitude  = %ld", p_lns_c_evt->params.lns.lat);
		NRF_LOG_INFO("Longitude = %ld", p_lns_c_evt->params.lns.lon);

		NRF_LOG_INFO("Ele %ld", p_lns_c_evt->params.lns.ele);

		lns_info.secj = p_lns_c_evt->params.lns.utc_time.seconds;
		lns_info.secj += p_lns_c_evt->params.lns.utc_time.minutes * 60;
		lns_info.secj += p_lns_c_evt->params.lns.utc_time.hours * 3600;

		lns_info.date = p_lns_c_evt->params.lns.utc_time.year   % 100;
		lns_info.date += p_lns_c_evt->params.lns.utc_time.day   * 10000;
		lns_info.date += p_lns_c_evt->params.lns.utc_time.month * 100;

		if (p_lns_c_evt->params.lns.flags & ELE_PRESENT) {
			lns_info.ele = p_lns_c_evt->params.lns.ele;
		}

		if (p_lns_c_evt->params.lns.flags & INST_SPEED_PRESENT) {
			lns_info.speed = p_lns_c_evt->params.lns.inst_speed;
		}

		NRF_LOG_INFO("Sec jour = %d %d %d\r\n", p_lns_c_evt->params.lns.utc_time.hours,
				p_lns_c_evt->params.lns.utc_time.minutes,
				p_lns_c_evt->params.lns.utc_time.seconds);

//		printf("$LOC,%lu,%ld,%ld,%ld,%u", sec_jour,
//				p_lns_c_evt->params.lns.lat, p_lns_c_evt->params.lns.lon, elev, speed);

		spis_encode_lns(&lns_info);

		break;
	}

	default:
		break;
	}
}


/**@brief Battery level Collector Handler.
 */
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
	uint32_t err_code;

	switch (p_bas_c_evt->evt_type)
	{
	case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
	{
		err_code = ble_bas_c_handles_assign(p_bas_c,
				p_bas_c_evt->conn_handle,
				&p_bas_c_evt->params.bas_db);
		APP_ERROR_CHECK(err_code);

		// Initiate bonding.
		err_code = pm_conn_secure(p_bas_c_evt->conn_handle, false);
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			APP_ERROR_CHECK(err_code);
		}

		// Batttery service discovered. Enable notification of Battery Level.
		NRF_LOG_DEBUG("Battery Service discovered. Reading battery level.");

		err_code = ble_bas_c_bl_read(p_bas_c);
		APP_ERROR_CHECK(err_code);

		NRF_LOG_DEBUG("Enabling Battery Level Notification.");
		err_code = ble_bas_c_bl_notif_enable(p_bas_c);
		APP_ERROR_CHECK(err_code);

	} break;

	case BLE_BAS_C_EVT_BATT_NOTIFICATION:
		NRF_LOG_INFO("Battery Level received %d %%.\r\n", p_bas_c_evt->params.battery_level);
		break;

	case BLE_BAS_C_EVT_BATT_READ_RESP:
		NRF_LOG_INFO("Battery Level Read as %d %%.\r\n", p_bas_c_evt->params.battery_level);
		break;

	default:
		break;
	}
}


/**
 * @brief Heart rate collector initialization.
 */
static void lns_c_init(void)
{
	ble_lns_c_init_t lns_c_init_obj;

	lns_c_init_obj.evt_handler = lns_c_evt_handler;

	uint32_t err_code = ble_lns_c_init(&m_ble_lns_c, &lns_c_init_obj);
	APP_ERROR_CHECK(err_code);
}


/**
 * @brief Battery level collector initialization.
 */
static void bas_c_init(void)
{
	ble_bas_c_init_t bas_c_init_obj;

	bas_c_init_obj.evt_handler = bas_c_evt_handler;

	uint32_t err_code = ble_bas_c_init(&m_ble_bas_c, &bas_c_init_obj);
	APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
	ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Retrieve a list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
	pm_peer_id_t peer_id;
	uint32_t     peers_to_copy;

	peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
			*p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

	peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
	*p_size = 0;

	while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
	{
		p_peers[(*p_size)++] = peer_id;
		peer_id = pm_next_peer_id_get(peer_id);
	}
}


static void whitelist_load()
{
	ret_code_t   ret;
	pm_peer_id_t peers[8];
	uint32_t     peer_cnt;

	memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
	peer_cnt = (sizeof(peers) / sizeof(pm_peer_id_t));

	// Load all peers from flash and whitelist them.
	peer_list_get(peers, &peer_cnt);

	ret = pm_whitelist_set(peers, peer_cnt);
	APP_ERROR_CHECK(ret);

	// Setup the device identies list.
	// Some SoftDevices do not support this feature.
	ret = pm_device_identities_list_set(peers, peer_cnt);
	if (ret != NRF_ERROR_NOT_SUPPORTED)
	{
		APP_ERROR_CHECK(ret);
	}
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
	if (nrf_fstorage_is_busy(NULL))
	{
		m_memory_access_in_progress = true;
		return;
	}

	// Whitelist buffers.
	ble_gap_addr_t whitelist_addrs[8];
	ble_gap_irk_t  whitelist_irks[8];

	memset(whitelist_addrs, 0x00, sizeof(whitelist_addrs));
	memset(whitelist_irks,  0x00, sizeof(whitelist_irks));

	uint32_t addr_cnt = (sizeof(whitelist_addrs) / sizeof(ble_gap_addr_t));
	uint32_t irk_cnt  = (sizeof(whitelist_irks)  / sizeof(ble_gap_irk_t));

#if (NRF_SD_BLE_API_VERSION <= 2)

	ble_gap_addr_t * p_whitelist_addrs[8];
	ble_gap_irk_t  * p_whitelist_irks[8];

	for (uint32_t i = 0; i < 8; i++)
	{
		p_whitelist_addrs[i] = &whitelist_addrs[i];
		p_whitelist_irks[i]  = &whitelist_irks[i];
	}

	ble_gap_whitelist_t whitelist =
	{
			.pp_addrs = p_whitelist_addrs,
			.pp_irks  = p_whitelist_irks,
	};

#endif

	// Reload the whitelist and whitelist all peers.
	whitelist_load();

	ret_code_t ret;

	// Get the whitelist previously set using pm_whitelist_set().
	ret = pm_whitelist_get(whitelist_addrs, &addr_cnt,
			whitelist_irks,  &irk_cnt);

	m_scan_param.active   = 0;
	m_scan_param.interval = SCAN_INTERVAL;
	m_scan_param.window   = SCAN_WINDOW;

	if (((addr_cnt == 0) && (irk_cnt == 0)) ||
			(m_whitelist_disabled))
	{
		// Don't use whitelist.
#if (NRF_SD_BLE_API_VERSION <= 2)
		m_scan_param.selective   = 0;
		m_scan_param.p_whitelist = NULL;
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
		m_scan_param.use_whitelist  = 0;
		m_scan_param.adv_dir_report = 0;
#endif
		m_scan_param.timeout  = 0x0000; // No timeout.
	}
	else
	{
		// Use whitelist.
#if (NRF_SD_BLE_API_VERSION <= 2)
		whitelist.addr_count     = addr_cnt;
		whitelist.irk_count      = irk_cnt;
		m_scan_param.selective   = 1;
		m_scan_param.p_whitelist = &whitelist;
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
		m_scan_param.use_whitelist  = 1;
		m_scan_param.adv_dir_report = 0;
#endif
		m_scan_param.timeout  = 0x001E; // 30 seconds.
	}

	NRF_LOG_INFO("Starting scan.");

	ret = sd_ble_gap_scan_start(&m_scan_param);
	APP_ERROR_CHECK(ret);

}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
	switch (p_evt->evt_id)
	{
	case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
	{
		NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
				p_evt->conn_handle,
				p_evt->params.att_mtu_effective);
	} break;

	case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
	{
		NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
				p_evt->conn_handle,
				p_evt->params.data_length);
	} break;

	default:
		break;
	}

	if (m_retry_db_disc)
	{
		NRF_LOG_DEBUG("Retrying DB discovery.");

		m_retry_db_disc = false;

		// Discover peer's services.
		ret_code_t err_code;
		err_code = ble_db_discovery_start(&m_db_disc, m_pending_db_disc_conn);

		if (err_code == NRF_ERROR_BUSY)
		{
			NRF_LOG_DEBUG("ble_db_discovery_start() returned busy, will retry later.");
			m_retry_db_disc = true;
		}
		else
		{
			APP_ERROR_CHECK(err_code);
		}
	}
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);
}


void ble_ant_init(void)
{
	ble_stack_init();
	ant_stack_init();

	peer_manager_init();

	gatt_init();
	db_discovery_init();

	lns_c_init();
	bas_c_init();

	ant_setup_start();

	// Start scanning for peripherals and initiate connection
	// with devices that advertise LNS UUID.
	scan_start();

}


