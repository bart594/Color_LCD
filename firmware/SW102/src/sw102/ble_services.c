/*
 * Bafang LCD SW102 Bluetooth firmware
 *
 * Released under the GPL License, Version 3
 */
#include "common.h"
#include "fstorage.h"
#include "ble_services.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_advertising.h"
#include "peer_manager.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "nrf_ble_gatt.h"
#include "fds.h"
#include "state.h"
#include "ble_conn_state.h"


// define to enable the (not yet used) serial service
 #define BLE_SERIAL


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "OS-EBike"                                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "https://github.com/OpenSourceEBike"

#define APP_ADV_INTERVAL                80                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      300                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define NO_PACKET					    0
#define PACKET_STATUS					1
#define PACKET_DEBUG					2
#define PACKET_CONFIG					3
#define PACKET_MOTOR					4
#define PACKET_TRIP_STATS				5

#define DATA_LENGHT_VALUE				120

#ifdef BLE_SERIAL
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

ble_nus_t                        		m_nus;                                      /**< Structure to identify the Nordic UART Service. */
nrf_ble_gatt_t							m_gatt;	
#endif

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static uint8_t 							ble_command = NO_PACKET;
volatile bool 							ble_config_update = false;
uint8_t 								tx_data_buffer[DATA_LENGHT_VALUE] = {0};
uint8_t 							    tx_data[BLE_NUS_MAX_DATA_LEN] = {0};
static bool								ble_gap_adv_started  = false;
volatile uint32_t 						ui32_g_ble_time_seconds = 0;
//bool 									erase_bonds = false;  

static ble_uuid_t                       m_adv_uuids[] = {
#ifdef BLE_SERIAL
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
#endif
};  /**< Universally unique service identifier. */


/**@brief This function will disconnect if connected, and stop advertising if advertising. */
void disconnect_stop_adv(void)
{
    ret_code_t err_code;
    
	//  stop advertising
	if(ble_gap_adv_started)
    err_code = sd_ble_gap_adv_stop();
	
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    } 
        
        // If connected, disconnect.
	if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
	{
	  
	  err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	        
	}
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    APP_ERROR_CHECK(sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME)));

    //APP_ERROR_CHECK(sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_CYCLING));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
}

#ifdef BLE_SERIAL
/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */

 static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	
	if(p_data[0] == 'e'){
	uint32_t temp = ((((uint32_t) p_data[3]) << 16) + (((uint32_t) p_data[2]) << 8) + ((uint32_t) p_data[1]));
	if(temp <  86400) //we don't need some garbage instead time
	ui32_g_ble_time_seconds  = temp;
	ble_command = NO_PACKET;
	} 
	if(p_data[0] == 'R'){ble_command = PACKET_STATUS;}
	if(p_data[0] == 'D'){ble_command = PACKET_DEBUG;}
	if(p_data[0] == 'T'){
	ble_command = PACKET_TRIP_STATS;
	}
	if(p_data[0] == 'T' && p_data[1] == 't'){    //trip data reset
	ble_reset_trip_stats();
	}	
	if(p_data[0] == 'C'){ble_command = PACKET_CONFIG;}
	if(p_data[0] == 'C' && p_data[1] == 'c'){
	ble_config_defaults();}  					//reset to defaults
	if(p_data[0] == 'M'){
	ble_motor_test(p_data);
	}
	
	if(p_data[0] == 'S'){
	 
	 ble_config_set(p_data);
	}
	
	if(p_data[0] == 'O'){
	 
	 ble_nav_info(p_data);
	}
    //for (uint32_t i = 0; i < length; i++)
   // {
    //    while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    //}
    //while(app_uart_put('\n') != NRF_SUCCESS);
}

// Init the serial port service
static void serial_init()
{
  ble_nus_init_t nus_init;
  memset(&nus_init, 0, sizeof(nus_init));

  nus_init.data_handler = nus_data_handler;

  APP_ERROR_CHECK(ble_nus_init(&m_nus, &nus_init));
}
#endif

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
#ifdef BLE_SERIAL
    serial_init();
#endif
}




/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  switch(p_evt->evt_type) {
  case BLE_CONN_PARAMS_EVT_FAILED:
    APP_ERROR_CHECK(sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE));
    break;
  case BLE_CONN_PARAMS_EVT_SUCCEEDED:
    break;
  default:
    break;
  }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
	
    APP_ERROR_CHECK(ble_conn_params_init(&cp_init));

    //APP_ERROR_CHECK(ble_advertising_start(BLE_ADV_MODE_FAST));
	
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
	   
        case BLE_ADV_EVT_FAST:
			ble_gap_adv_started = true;
            break;
        case BLE_ADV_EVT_IDLE:
			 ble_gap_adv_started = false; 
		     //APP_ERROR_CHECK(ble_advertising_start(BLE_ADV_MODE_FAST));
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            //sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);						   
    ble_conn_params_on_ble_evt(p_ble_evt);
#ifdef BLE_SERIAL
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
#endif
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    //APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    APP_ERROR_CHECK(softdevice_enable(&ble_enable_params));

    // Subscribe for BLE events.
    APP_ERROR_CHECK(softdevice_ble_evt_handler_set(ble_evt_dispatch));

    // Register with the SoftDevice handler module for system events.
    // Important for FDS and fstorage or event handler doesn't fire!
    APP_ERROR_CHECK(softdevice_sys_evt_handler_set(sys_evt_dispatch));
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
	
    APP_ERROR_CHECK(ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL));
}

static void peer_manager_event_handler(pm_evt_t const *p_evt)
{
    ret_code_t err_code;
    switch(p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
            // Update the rank of the peer.
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            break;
        case PM_EVT_CONN_SEC_START:
            break;
        case PM_EVT_CONN_SEC_SUCCEEDED:
            // Update the rank of the peer.
            ble_conn_state_role(p_evt->conn_handle);
            break;
        case PM_EVT_CONN_SEC_FAILED:
            // In some cases, when securing fails, it can be restarted directly. Sometimes it can be
            // restarted, but only after changing some Security Parameters. Sometimes, it cannot be
            // restarted until the link is disconnected and reconnected. Sometimes it is impossible
            // to secure the link, or the peer device does not support it. How to handle this error
            // is highly application-dependent.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // A connected peer (central) is trying to pair, but the Peer Manager already has a bond
            // for that peer. Setting allow_repairing to false rejects the pairing request.
            // If this event is ignored (pm_conn_sec_config_reply is not called in the event
            // handler), the Peer Manager assumes allow_repairing to be false.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        }
        break;
        case PM_EVT_STORAGE_FULL:
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
            break;
        case PM_EVT_ERROR_UNEXPECTED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
            break;
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break;
        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            // Assert.
            APP_ERROR_CHECK_BOOL(false);
            break;
        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break;
        case PM_EVT_PEER_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
            break;
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            // At this point it is safe to start advertising or scanning.
			advertising_start(false);
		    //ble_advertising_start(BLE_ADV_MODE_FAST);
            break;
        case PM_EVT_PEERS_DELETE_FAILED:
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
            break;
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break;
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break;
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break;
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break;
    }
}


static void peer_init() {
  
  //bool erase_bonds = false; // FIXME, have UX have a place to delete remembered BT devices
  ret_code_t err_code;
  err_code = pm_init();
  APP_ERROR_CHECK(err_code);
  
  //if (erase_bonds)
  //{
    //  pm_peers_delete();
  //}

  ble_gap_sec_params_t sec_param;
  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
  // Security parameters to be used for all security procedures.
  // per https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v12.3.0/lib_peer_manager.html?cp=5_5_7_3_1_8
  // currently set to be super open and pair with anyone
  sec_param.bond = true;
  sec_param.mitm = false;
  sec_param.lesc = 0;
  sec_param.keypress = 0;
  sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
  sec_param.oob = false;
  sec_param.min_key_size = 7;
  sec_param.max_key_size = 16;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;
  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);
  err_code = pm_register(peer_manager_event_handler);
  APP_ERROR_CHECK(err_code);
}



/**@brief Clear bond information from persistent storage.
 */
void delete_bonds(void)
{
    ret_code_t err_code;
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        if (ble_gap_adv_started == false){
		ret_code_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
		}
    }
}

void ble_init(void)
{
  bool erase_bonds;
  
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  peer_init();
  advertising_start(erase_bonds);
}


void ble_uart_send(ui_vars_t *ui_vars) {
 
 char data_arr[12] = {"eBike"}; 
 uint8_t hours;
 uint8_t minutes;
 uint8_t ui16_temp;
 
 switch (ble_command)
      {
        case NO_PACKET:
         						
			// Calculate time
			if(ui_vars->ui32_calc_time_seconds){
            hours = ui_vars->ui32_calc_time_seconds / 3600;
            minutes = (ui_vars->ui32_calc_time_seconds % 3600) / 60;
			sprintf(data_arr, "eBike %d:%02d", hours, minutes);
	        }
 		    ble_nus_string_send(&m_nus, data_arr, 12);
				
		break;
		
		case PACKET_STATUS:

		tx_data[0] = 0x52;  // "R" 
		tx_data[1] = ui_vars->ui8_riding_mode;
		tx_data[2] = ui_vars->ui8_assist_level;
		tx_data[3] = (uint8_t) (ui_vars->ui16_wheel_speed_x10 & 0xff);
		tx_data[4] = (uint8_t) (ui_vars->ui16_wheel_speed_x10 >> 8);
		tx_data[5] = ui_vars->ui8_pedal_cadence_filtered;
		tx_data[6] = ui_vars->ui8_motor_temperature;
		tx_data[7] = (uint8_t) (ui_vars->ui16_pedal_power_filtered & 0xff);
		tx_data[8] = (uint8_t) (ui_vars->ui16_pedal_power_filtered >> 8);
		tx_data[9] = (uint8_t) (ui_vars->ui16_battery_voltage_soc_x10 & 0xff);
		tx_data[10] = (uint8_t) (ui_vars->ui16_battery_voltage_soc_x10 >> 8);
		tx_data[11] = (uint8_t) ((ui_vars->ui16_battery_current_filtered_x5 * 2) & 0xff);
		tx_data[12] = (uint8_t) ((ui_vars->ui16_battery_current_filtered_x5 * 2) >> 8);
		tx_data[13] = ui_vars->ui8_error_states;
		ui16_temp = ui_vars->ui32_wh_x10 / 10;
		tx_data[14] = (uint8_t) (ui16_temp & 0xff);
		tx_data[15] = (uint8_t) (ui16_temp >> 8);
		tx_data[16] = (uint8_t) (ui16_temp >> 16);
		tx_data[17] = ui_vars->ui8_street_mode_feature_enabled;
		ui16_temp = ui_vars->ui16_battery_estimated_range_x10;
		tx_data[18] = (uint8_t) (ui16_temp & 0xff);
		tx_data[19] = (uint8_t) (ui16_temp >> 8);
		  
		  if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				ret_code_t err_code;
				err_code = ble_nus_string_send(&m_nus, tx_data, 20);
				if (err_code == NRF_SUCCESS)
				{
				ble_command = PACKET_DEBUG;
				}
			}		
		break;
		
		case PACKET_DEBUG:
		
		tx_data[0] = 0x44;  // "D" 
		tx_data[1] = ui_vars->ui8_adc_throttle;
		tx_data[2] = ui_vars->ui8_throttle;
		tx_data[3] = (uint8_t) (ui_vars->ui16_adc_pedal_torque_sensor & 0xff);
		tx_data[4] = (uint8_t) (ui_vars->ui16_adc_pedal_torque_sensor >> 8);		
		tx_data[5] = ui_vars->ui8_duty_cycle;
		tx_data[6] = (uint8_t) (ui_vars->ui16_motor_speed_erps & 0xff);
		tx_data[7] = (uint8_t) (ui_vars->ui16_motor_speed_erps >> 8);		
		tx_data[8] = ui_vars->ui8_foc_angle;		
		tx_data[9] = (uint8_t) (ui_vars->ui16_pedal_torque_x100 & 0xff);
		tx_data[10] = (uint8_t) (ui_vars->ui16_pedal_torque_x100 >> 8);
		ui16_temp = (uint16_t) ui_vars->battery_energy_h_km_ui32_value_x10;
		tx_data[11] = (uint8_t) (ui16_temp & 0xff);
		tx_data[12] = (uint8_t) (ui16_temp >> 8);		
		tx_data[13] = (uint8_t) (ui_vars->ui16_pedal_weight & 0xff);
		tx_data[14] = (uint8_t) (ui_vars->ui16_pedal_weight >> 8);
		
		  if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				ret_code_t err_code;
								
				err_code = ble_nus_string_send(&m_nus, tx_data, 15);
				if (err_code == NRF_SUCCESS)
				{
				ble_command = PACKET_STATUS;
				}
			}
	
		break;
		
		case PACKET_TRIP_STATS:
		
		tx_data[0] = 0x54;  // "T" 
	    ui16_temp = ui_vars->ui32_trip_time;
        tx_data[1] = (uint8_t) (ui16_temp & 0xff);
        tx_data[2] = (uint8_t) (ui16_temp >> 8);
		tx_data[3] = (uint8_t) (ui16_temp >> 16);	
        ui16_temp = (ui_vars->ui32_trip_distance_x100);
        tx_data[4] = (uint8_t) (ui16_temp & 0xff);
        tx_data[5] = (uint8_t) (ui16_temp >> 8);
        tx_data[6] = (uint8_t) (ui16_temp >> 16);		
        ui16_temp = ui_vars->ui16_trip_avg_speed_x10;
        tx_data[7] = (uint8_t) (ui16_temp & 0xff);
        tx_data[8] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_trip_max_speed_x10;
        tx_data[9] = (uint8_t) (ui16_temp & 0xff);
        tx_data[10] = (uint8_t) (ui16_temp >> 8);		
		ui16_temp = ui_vars->ui16_battery_current_avg;
        tx_data[11] = (uint8_t) (ui16_temp & 0xff);
        tx_data[12] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_battery_power_avg;
        tx_data[13] = (uint8_t) (ui16_temp & 0xff);
        tx_data[14] = (uint8_t) (ui16_temp >> 8);	
        ui16_temp = ui_vars->ui16_pedal_power_avg;
        tx_data[15] = (uint8_t) (ui16_temp & 0xff);
        tx_data[16] = (uint8_t) (ui16_temp >> 8);
        tx_data[17] = (uint8_t) ui_vars->ui16_pedal_cadence_avg;
        ui16_temp = ui_vars->ui16_battery_energy_h_km_avg_x100;		
        tx_data[18] = (uint8_t) (ui16_temp & 0xff);
        tx_data[19] = (uint8_t) (ui16_temp >> 8);

		  if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				ret_code_t err_code;
				err_code = ble_nus_string_send(&m_nus, tx_data, 20);
				if (err_code == NRF_SUCCESS)
				{
				ble_command = PACKET_STATUS;
				}
			}		
		
		break;

		case PACKET_CONFIG:

		tx_data_buffer[0] = 0x43; 		// C = config
        tx_data_buffer[1] = 1;
		tx_data_buffer[2] = ui_vars->ui8_motor_type;
        tx_data_buffer[3] = ui_vars->ui8_motor_temperature_min_value_to_limit;
        tx_data_buffer[4] = ui_vars->ui8_motor_temperature_max_value_to_limit;
        tx_data_buffer[5] = ui_vars->ui8_motor_acceleration;
        tx_data_buffer[6] = ui_vars->ui8_number_of_assist_levels; 	
        tx_data_buffer[7] = ui_vars->ui8_eMTB_assist_level; 		
        tx_data_buffer[8] = ui_vars->ui8_torque_sensor_calibration_feature_enabled;
        tx_data_buffer[9] =  ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100;
        tx_data_buffer[10] =  ui_vars->ui8_optional_ADC_function;
        tx_data_buffer[11] =  ui_vars->ui8_assist_without_pedal_rotation_threshold;
        tx_data_buffer[12] = ui_vars->ui8_lights_configuration;
        tx_data_buffer[13] = (uint8_t) (ui_vars->ui16_wheel_perimeter & 0xff);
        tx_data_buffer[14] = (uint8_t) (ui_vars->ui16_wheel_perimeter >> 8);
        tx_data_buffer[15] = ui_vars->ui8_walk_assist_feature_enabled;
        tx_data_buffer[16] = (uint8_t) (ui_vars->ui16_battery_voltage_reset_wh_counter_x10 & 0xff);
        tx_data_buffer[17] = (uint8_t)(ui_vars->ui16_battery_voltage_reset_wh_counter_x10 >> 8);
        tx_data_buffer[18] = ui_vars->ui8_battery_max_current;
        tx_data_buffer[19] = ui_vars->ui8_target_max_battery_power_div25;

        tx_data_buffer[20] = 0x43;
		tx_data_buffer[21] = 2;
		tx_data_buffer[22] = (uint8_t) (ui_vars->ui16_battery_pack_resistance_x1000 & 0xff);
		tx_data_buffer[23] = (uint8_t) (ui_vars->ui16_battery_pack_resistance_x1000 >> 8);
        tx_data_buffer[24] = (uint8_t) (ui_vars->ui16_battery_low_voltage_cut_off_x10 & 0xff);
        tx_data_buffer[25] = (uint8_t) (ui_vars->ui16_battery_low_voltage_cut_off_x10 >> 8);
        tx_data_buffer[26] = ui_vars->ui8_street_mode_feature_enabled;
        tx_data_buffer[27] = ui_vars->ui8_street_mode_throttle_enabled;
        tx_data_buffer[28] = ui_vars->ui8_street_mode_power_limit_div25;
        tx_data_buffer[29] = ui_vars->ui8_street_mode_speed_limit;
		tx_data_buffer[30] = ui_vars->ui8_field_weakening_enabled;
		tx_data_buffer[31] = ui_vars->ui8_field_weakening_current_adc;
		tx_data_buffer[32] = ui_vars->ui8_hybrid_mode_enabled;
		tx_data_buffer[33] = ui_vars->ui8_soft_start_feature_enabled;
        tx_data_buffer[34] = ui_vars->ui8_time_field_enable;		
		tx_data_buffer[35] = ui_vars->ui8_motor_current_min_adc;
		tx_data_buffer[36] = ui_vars->wheel_max_speed_x10 / 10;
		tx_data_buffer[37] = (uint8_t) (ui_vars->ui32_wh_x10_100_percent & 0xff);
		tx_data_buffer[38] = (uint8_t) (ui_vars->ui32_wh_x10_100_percent >> 8);
		tx_data_buffer[39] = (uint8_t) (ui_vars->ui32_wh_x10_100_percent  >> 16);		

        tx_data_buffer[40] = 0x43;
		tx_data_buffer[41] = 3;
        
		for (int i=0;i<5;i++)
            tx_data_buffer[42+i] = ui_vars->ui8_target_peak_battery_power_div25[i];
        
		for (int i=0;i<5;i++)
            tx_data_buffer[47+i] = ui_vars->ui8_motor_acceleration_level[i];	
		
		for (int i=0;i<5;i++)
			tx_data_buffer[52+i] = ui_vars->ui8_walk_assist_level_factor[i];			
		
		tx_data_buffer[57] = (uint8_t) (ui_vars->ui32_wh_x10_offset & 0xff);
		tx_data_buffer[58] = (uint8_t) (ui_vars->ui32_wh_x10_offset >> 8);
		tx_data_buffer[59] = (uint8_t) (ui_vars->ui32_wh_x10_offset >> 16);
		
		tx_data_buffer[60] = 0x43;
		tx_data_buffer[61] = 4;
		
		for (int i=0;i<5;i++)
            tx_data_buffer[62+i] = ui_vars->ui8_assist_level_power_assist[i];
		
		for (int i=0;i<5;i++)
            tx_data_buffer[67+i] = ui_vars->ui8_assist_level_torque_assist[i];			
		tx_data_buffer[72] = ui_vars->ui8_lcd_power_off_time_minutes;
		tx_data_buffer[73] = ui_vars->ui8_units_type;
		tx_data_buffer[74] = ui_vars->ui8_energy_saving_mode_level;
		tx_data_buffer[75] = ui_vars->ui8_plus_long_press_switch;
		tx_data_buffer[76] = 0;
		tx_data_buffer[77] = 0;
		tx_data_buffer[78] = 0;
		tx_data_buffer[79] = 0;
			
		tx_data_buffer[80] = 0x43;
		tx_data_buffer[81] = 5;
		for (uint8_t i = 0; i < 6; i++) 
        tx_data_buffer[82+i] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[i][0]); //kg_weight
        		
		tx_data_buffer[88] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[0][1] & 0xff); // adc_values
        tx_data_buffer[89] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[0][1] >> 8);
		tx_data_buffer[90] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[1][1] & 0xff); // adc_values
        tx_data_buffer[91] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[1][1] >> 8);
		tx_data_buffer[92] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[2][1] & 0xff); // adc_values
        tx_data_buffer[93] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[2][1] >> 8);
		tx_data_buffer[94] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[3][1] & 0xff); // adc_values
        tx_data_buffer[95] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[3][1] >> 8);
		tx_data_buffer[96] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[4][1] & 0xff); // adc_values
        tx_data_buffer[97] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[4][1] >> 8);	
		tx_data_buffer[98] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[5][1] & 0xff); // adc_values
        tx_data_buffer[99] = (uint8_t) (ui_vars->ui16_torque_sensor_calibration_ble_table[5][1] >> 8);	

		tx_data_buffer[100] = 0x43;
		tx_data_buffer[101] = 6;
		for (uint8_t i = 0; i < 6; i++) 
        tx_data_buffer[102+i] = ui_vars->ui8_hall_ref_angles[i];
		
		for (uint8_t i = 0; i < 6; i++) 		
		tx_data_buffer[108+i] = ui_vars->ui8_hall_counter_offset[i]; 
		tx_data_buffer[114] = 0;
		tx_data_buffer[115] = 0;		
		tx_data_buffer[116] = 0;
		tx_data_buffer[117] = 0;
		tx_data_buffer[118] = 0;
		tx_data_buffer[119] = 0;
		
		  if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				ret_code_t err_code;
				err_code = ble_nus_send_file(&m_nus, tx_data_buffer, DATA_LENGHT_VALUE, 20); //data lenght 120
				if (err_code == NRF_SUCCESS)
				{
				}		
			}
		
		ble_command = NO_PACKET;
		
		//sprintf(data_array, "D1,%d,%d,%d,!",    rt_vars->ui8_duty_cycle, rt_vars->ui16_pedal_torque_x100, rt_vars->ui16_adc_pedal_torque_sensor);
	    //	ble_nus_string_send(&m_nus, data_array, strlen(data_array));

        break;
		
		case PACKET_MOTOR:
		
		tx_data[0] = 0x4D;  // "M" 
		tx_data[1] = ui_vars->ui8_riding_mode;
	    ui16_temp = ui_vars->ui16_hall_calib_cnt[0];
        tx_data[2] = (uint8_t) (ui16_temp & 0xff);
        tx_data[3] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_hall_calib_cnt[1];
        tx_data[4] = (uint8_t) (ui16_temp & 0xff);
        tx_data[5] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_hall_calib_cnt[2];
        tx_data[6] = (uint8_t) (ui16_temp & 0xff);
        tx_data[7] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_hall_calib_cnt[3];
        tx_data[8] = (uint8_t) (ui16_temp & 0xff);
        tx_data[9] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_hall_calib_cnt[4];
        tx_data[10] = (uint8_t) (ui16_temp & 0xff);
        tx_data[11] = (uint8_t) (ui16_temp >> 8);
        ui16_temp = ui_vars->ui16_hall_calib_cnt[5];
        tx_data[12] = (uint8_t) (ui16_temp & 0xff);
        tx_data[13] = (uint8_t) (ui16_temp >> 8);

		  if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				ret_code_t err_code;
				err_code = ble_nus_string_send(&m_nus, tx_data, 14);
				if (err_code == NRF_SUCCESS)
				{
				}
			}
		break;

	  }
	  
}

static void ble_config_defaults(){

ui8_g_configuration_display_reset_to_defaults = 1;
ble_command = NO_PACKET;

}
static void ble_reset_trip_stats()
{
	 
	  ui8_g_configuration_trip_reset = 1;
	  ble_command = PACKET_TRIP_STATS;
	  
}

static void ble_motor_test(uint8_t *p_data)
{
	  ui_vars_t *ui_vars = get_ui_vars();

      ui_vars->ui8_riding_mode = p_data[1];
	  ui_vars->ui8_calibration_duty_cycle_target = p_data[2];
	  if (ui_vars->ui8_riding_mode == MOTOR_CALIBRATION_MODE){
	  ble_command = PACKET_MOTOR;
	  g_motor_init_state = MOTOR_INIT_CALIBRATION;
	  }else{ble_command = NO_PACKET;
	  g_motor_init_state = MOTOR_INIT_READY;
	  }
}
	  
static void ble_nav_info(uint8_t *p_data)
{

	  //ble_config_update = true;
	  ui_vars_t *ui_vars = get_ui_vars();
	  
	  uint32_t ui32_nav_turn_distance_temp = (((uint32_t) p_data[3]) << 16) + (((uint32_t) p_data[2]) << 8) + ((uint32_t) p_data[1]);;
      
	  ui_vars->ui32_nav_total_distance = (((uint32_t) p_data[6]) << 16) + (((uint32_t) p_data[5]) << 8) + ((uint32_t) p_data[4]);
      ui_vars->ui8_nav_info = p_data[7];
	  ui_vars->ui8_nav_info_extra = p_data[8];
	  
	  if (ui32_nav_turn_distance_temp > ui_vars->ui32_nav_turn_distance){
	  ui_vars->ui32_nav_total_turn_distance = ui32_nav_turn_distance_temp;
	  //nav_command_update = true;
	  }
	  
	  ui_vars->ui32_nav_turn_distance = ui32_nav_turn_distance_temp;
	  
	  if(ui_vars->ui8_nav_info_extra == 255){
	  ui_vars->ui32_nav_turn_distance = 0;
	  ui_vars->ui32_nav_total_distance = 0;
	  ui_vars->ui32_nav_total_turn_distance = 0;
	  }
	  
	  
      //ble_config_update = false;
}
	  
static void ble_config_set(uint8_t *p_data)
{

	 ble_config_update = true;
	 ui_vars_t *ui_vars = get_ui_vars();
	 
	 static uint8_t data[20];
	 memcpy(&data, p_data, 20);
	 
		if(data[1] == 0x01){
	  
      ui_vars->ui8_motor_type = data[2];
      ui_vars->ui8_motor_temperature_min_value_to_limit = data[3];
      ui_vars->ui8_motor_temperature_max_value_to_limit = data[4];
      ui_vars->ui8_motor_acceleration = data[5];
      ui_vars->ui8_number_of_assist_levels = data[6];
      ui_vars->ui8_eMTB_assist_level = data[7];		
      ui_vars->ui8_torque_sensor_calibration_feature_enabled = data[8];
      ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100 = data[9]; 
      ui_vars->ui8_optional_ADC_function = data[10]; 
      ui_vars->ui8_assist_without_pedal_rotation_threshold = data[11]; 
      ui_vars->ui8_lights_configuration = data[12]; 
      ui_vars->ui16_wheel_perimeter = (((uint16_t) data[14]) << 8) + ((uint16_t) data[13]); 
      ui_vars->ui8_walk_assist_feature_enabled = data[15]; 
      ui_vars->ui16_battery_voltage_reset_wh_counter_x10 = (((uint16_t) data[17]) << 8) + ((uint16_t) data[16]); 
      ui_vars->ui8_battery_max_current = data[18]; 
      ui_vars->ui8_target_max_battery_power_div25 = data[19];
	  
	  ui_vars->ui16_target_max_battery_power = ui_vars->ui8_target_max_battery_power_div25 * 25;
	}
	
		if(data[1] == 0x02){
  
      ui_vars->ui16_battery_pack_resistance_x1000 = (((uint16_t) data[3]) << 8) + ((uint16_t) data[2]);
      ui_vars->ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) data[5]) << 8) + ((uint16_t) data[4]);
      ui_vars->ui8_street_mode_enabled = data[6];
      ui_vars->ui8_street_mode_throttle_enabled = data[7];
      ui_vars->ui8_street_mode_power_limit_div25 = data[8]; 
      ui_vars->ui8_street_mode_speed_limit = data[9]; 
	  ui_vars->ui8_field_weakening_enabled = data[10];
	  ui_vars->ui8_field_weakening_current_adc = data[11];
	  ui_vars->ui8_hybrid_mode_enabled = data[12];	
  	  ui_vars->ui8_soft_start_feature_enabled = data[13];
	  ui_vars->ui8_time_field_enable = data[14];
	  ui_vars->ui8_motor_current_min_adc = data[15];
	  ui_vars->wheel_max_speed_x10 = (uint16_t) (data[16] * 10);
      ui_vars->ui8_energy_saving_mode_level	=  data[17];
	  ui_vars->ui8_plus_long_press_switch	=  data[18];
	}
	
		if(data[1] == 0x03){
  
      for (int i = 0; i < 5; i++)
      ui_vars->ui8_target_peak_battery_power_div25[i] = data[2 + i];
      
	  for (int i = 0; i < 5; i++)
      ui_vars->ui8_motor_acceleration_level[i] = data[7 + i];
      
	  for (int i = 0; i < 5; i++)
      ui_vars->ui8_walk_assist_level_factor[i] = data[12 + i];	  
	  
	}	
	
		if(data[1] == 0x04){
		
      for (int i = 0; i < 5; i++)
      ui_vars->ui8_assist_level_power_assist[i] = data[2 + i];
      
	  for (int i = 0; i < 5; i++)
      ui_vars->ui8_assist_level_torque_assist[i] = data[7 + i];	  
	  
	  ui_vars->ui32_wh_x10_100_percent = ((((uint32_t) data[14]) << 16) + (((uint32_t) data[13]) << 8) + ((uint32_t) data[12]));
	  ui_vars->ui32_wh_x10_offset = ((((uint32_t) data[17]) << 16) + (((uint32_t) data[16]) << 8) + ((uint32_t) data[15]));
	  ui_vars->ui8_lcd_power_off_time_minutes = data[18];;
	  ui_vars->ui8_units_type = data[19];
 
	}
		if(data[1] == 0x05){

	  for (uint8_t i = 0; i < 6; i++) 
      ui_vars->ui16_torque_sensor_calibration_table[i][0] = data[2 + i]; //kg values

      ui_vars->ui16_torque_sensor_calibration_table[0][1] = (((uint16_t) data[9]) << 8) + ((uint16_t) data[8]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[1][1] = (((uint16_t) data[11]) << 8) + ((uint16_t) data[10]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[2][1] = (((uint16_t) data[13]) << 8) + ((uint16_t) data[12]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[3][1] = (((uint16_t) data[15]) << 8) + ((uint16_t) data[14]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[4][1] = (((uint16_t) data[17]) << 8) + ((uint16_t) data[16]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[5][1] = (((uint16_t) data[19]) << 8) + ((uint16_t) data[18]); //adc values
	}
	
		if(data[1] == 0x06){

	  for (uint8_t i = 0; i < 6; i++) 
      ui_vars->ui8_hall_ref_angles[i] = data[2+ i]; 
	  
  	  for (uint8_t i = 0; i < 6; i++) 
      ui_vars->ui8_hall_counter_offset[i] = data[8+ i];
	}
	
	ble_config_update = false;
	ble_command = NO_PACKET;
	
	if (g_motor_init_state == MOTOR_INIT_READY)
    g_motor_init_state = MOTOR_UPDATE_CONFIG;
	
 }
 
 
