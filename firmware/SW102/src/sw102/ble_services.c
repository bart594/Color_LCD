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
#define MANUFACTURER_NAME               "https://github.com/OpenSource-EBike-firmware"

#define APP_ADV_INTERVAL                40                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define NO_PACKET					    0
#define PACKET_REGULAR					1
#define PACKET_CONFIG					2
#define SAVE_CONFIG						3

#ifdef BLE_SERIAL
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

ble_nus_t                        		m_nus;                                      /**< Structure to identify the Nordic UART Service. */
nrf_ble_gatt_t							m_gatt;	
#endif

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static uint8_t 							ble_command = PACKET_REGULAR;
volatile bool 							ble_config_update = false;

static ble_uuid_t                       m_adv_uuids[] = {
#ifdef BLE_SERIAL
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
#endif
};  /**< Universally unique service identifier. */


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

    APP_ERROR_CHECK(sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_CYCLING));

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
	
	if(p_data[0] == 'N'){ble_command = NO_PACKET;} 
	if(p_data[0] == 'R'){ble_command = PACKET_REGULAR;}
	if(p_data[0] == 'C'){ble_command = PACKET_CONFIG;}
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

    APP_ERROR_CHECK(ble_advertising_start(BLE_ADV_MODE_FAST));
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
            break;
        case BLE_ADV_EVT_IDLE:
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
		    ble_advertising_start(BLE_ADV_MODE_FAST);
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
  bool erase_bonds = false; // FIXME, have UX have a place to delete remembered BT devices
  ret_code_t err_code;
  err_code = pm_init();
  APP_ERROR_CHECK(err_code);
  if (erase_bonds)
  {
      pm_peers_delete();
  }

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

void ble_init(void)
{
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  peer_init();
}


void ble_uart_send(rt_vars_t *rt_vars) {
 static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
 char data_arr[10] = {"NOP"}; 
 
 switch (ble_command)
      {
        case 0:
         
        ble_nus_string_send(&m_nus, data_arr, 10);
		break;
		
		case 1:
		data_array[0] = 0x52;  // "R" 
		data_array[1] = rt_vars->ui8_riding_mode;
		data_array[2] = rt_vars->ui8_assist_level;
		data_array[3] = (uint8_t) (rt_vars->ui16_wheel_speed_x10 & 0xff);
		data_array[4] = (uint8_t) (rt_vars->ui16_wheel_speed_x10 >> 8);
		data_array[5] = rt_vars->ui8_pedal_cadence_filtered;
		data_array[6] = rt_vars->ui8_motor_temperature;
		data_array[7] = (uint8_t) (rt_vars->ui16_pedal_power_filtered & 0xff);
		data_array[8] = (uint8_t) (rt_vars->ui16_pedal_power_filtered >> 8);
		data_array[9] = (uint8_t) (rt_vars->ui16_battery_voltage_soc_x10 & 0xff);
		data_array[10] = (uint8_t) (rt_vars->ui16_battery_voltage_soc_x10 >> 8);
		data_array[11] = (uint8_t) ((rt_vars->ui16_battery_current_filtered_x5 * 2) & 0xff);
		data_array[12] = (uint8_t) ((rt_vars->ui16_battery_current_filtered_x5 * 2) >> 8);
		data_array[13] = rt_vars->ui8_error_states;
		uint16_t ui16_temp = rt_vars->ui32_wh_x10 / 10;
		data_array[14] = (uint8_t) (ui16_temp & 0xff);
		data_array[15] = (uint8_t) (ui16_temp >> 8);
		data_array[16] = rt_vars->ui8_street_mode_feature_enabled;
		ble_nus_string_send(&m_nus, data_array, 17);
		
		data_array[0] = 0x44;  // "D" 
		data_array[1] = rt_vars->ui8_adc_throttle;
		data_array[2] = rt_vars->ui8_throttle;
		data_array[3] = (uint8_t) (rt_vars->ui16_adc_pedal_torque_sensor & 0xff);
		data_array[4] = (uint8_t) (rt_vars->ui16_adc_pedal_torque_sensor >> 8);		
		data_array[5] = rt_vars->ui8_duty_cycle;
		data_array[6] = (uint8_t) (rt_vars->ui16_motor_speed_erps & 0xff);
		data_array[7] = (uint8_t) (rt_vars->ui16_motor_speed_erps >> 8);		
		data_array[8] = rt_vars->ui8_foc_angle;		
		data_array[9] = (uint8_t) (rt_vars->ui16_pedal_torque_x100 & 0xff);
		data_array[10] = (uint8_t) (rt_vars->ui16_pedal_torque_x100 >> 8);
		uint16_t ui16_temp_wh_km = rt_vars->battery_energy_h_km.ui32_value_x10;
		data_array[11] = (uint8_t) (ui16_temp_wh_km & 0xff);
		data_array[12] = (uint8_t) (ui16_temp_wh_km >> 8);		
		data_array[13] = (uint8_t) (rt_vars->ui16_pedal_weight & 0xff);
		data_array[14] = (uint8_t) (rt_vars->ui16_pedal_weight >> 8);
		ble_nus_string_send(&m_nus, data_array, 15);

        break;

		case 2:
		data_array[0] = 0x43; 		// C = config
		data_array[1] = 1;  			//first chunk of config data
        data_array[2] = rt_vars->ui8_motor_type;
        data_array[3] = rt_vars->ui8_motor_temperature_min_value_to_limit;
        data_array[4] = rt_vars->ui8_motor_temperature_max_value_to_limit;
        data_array[5] = rt_vars->ui8_motor_acceleration;
        data_array[6] = rt_vars->ui8_number_of_assist_levels; 	
        data_array[7] = rt_vars->ui8_eMTB_assist_level; 		
        data_array[8] = rt_vars->ui8_torque_sensor_calibration_feature_enabled;
        data_array[9] =  rt_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100;
        data_array[10] =  rt_vars->ui8_optional_ADC_function;
        data_array[11] =  rt_vars->ui8_motor_assistance_startup_without_pedal_rotation;
        data_array[12] = rt_vars->ui8_lights_configuration;
        data_array[13] = (uint8_t) (rt_vars->ui16_wheel_perimeter & 0xff);
        data_array[14] = (uint8_t) (rt_vars->ui16_wheel_perimeter >> 8);
        data_array[15] = rt_vars->ui8_walk_assist_feature_enabled;
        data_array[16] = (uint8_t) (rt_vars->ui16_battery_voltage_reset_wh_counter_x10 & 0xff);
        data_array[17] = (uint8_t)(rt_vars->ui16_battery_voltage_reset_wh_counter_x10 >> 8);
        data_array[18] = rt_vars->ui8_battery_max_current;
        data_array[19] = rt_vars->ui8_target_max_battery_power_div25;
		ble_nus_string_send(&m_nus, data_array, 20);
		
		data_array[0] = 0x43;
		data_array[1] = 2;  			//second  chunk of config data
        data_array[2] = (uint8_t) (rt_vars->ui16_battery_pack_resistance_x1000 & 0xff);
        data_array[3] = (uint8_t) (rt_vars->ui16_battery_pack_resistance_x1000 >> 8);
        data_array[4] = (uint8_t) (rt_vars->ui16_battery_low_voltage_cut_off_x10 & 0xff);
        data_array[5] = (uint8_t) (rt_vars->ui16_battery_low_voltage_cut_off_x10 >> 8);
        data_array[6] = rt_vars->ui8_street_mode_feature_enabled;
        data_array[7] = rt_vars->ui8_street_mode_throttle_enabled;
        data_array[8] = rt_vars->ui8_street_mode_power_limit_div25;
        data_array[9] = rt_vars->ui8_street_mode_speed_limit;
		data_array[10] = rt_vars->ui8_field_weakening_enabled;
		data_array[11] = rt_vars->ui8_field_weakening_current;
		data_array[12] = rt_vars->ui8_cadence_RPM_limit;
		data_array[13] = rt_vars->ui8_torque_boost_factor;
        data_array[14] = rt_vars->ui8_battery_soc_enable;		

        ble_nus_string_send(&m_nus, data_array, 20);
		
		data_array[0] = 0x43;
		data_array[1] = 3;  			//third  chunk of config data
        for (int i=0;i<5;i++)
            data_array[2+i] = rt_vars->ui8_target_peak_battery_power_div25[i];
        for (int i=0;i<5;i++)
            data_array[7+i] = rt_vars->ui8_motor_acceleration_level[i];	
		for (int i=0;i<5;i++)
			data_array[12+i] = rt_vars->ui8_walk_assist_level_factor[i];			

		ble_nus_string_send(&m_nus, data_array, 20);
		
		
		data_array[0] = 0x43;
		data_array[1] = 4;  			//fourth  chunk of config data
		for (int i=0;i<5;i++)
            data_array[2+i] = rt_vars->ui8_assist_level_power_assist[i];		
	    data_array[7] = (uint8_t) (rt_vars->ui32_wh_x10_100_percent & 0xff);
		data_array[8] = (uint8_t) (rt_vars->ui32_wh_x10_100_percent >> 8);
		data_array[9] = (uint8_t) (rt_vars->ui32_wh_x10_100_percent  >> 16); 
		data_array[10] = (uint8_t) (rt_vars->ui32_wh_x10_offset & 0xff);
		data_array[11] = (uint8_t) (rt_vars->ui32_wh_x10_offset >> 8);
		data_array[12] = (uint8_t) (rt_vars->ui32_wh_x10_offset >> 16);		

		ble_nus_string_send(&m_nus, data_array, 20);
		
		data_array[0] = 0x43;
		data_array[1] = 5;  			//fifth chunk of config data
		for (uint8_t i = 0; i < 6; i++) {
        data_array[2+i] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[i][0]); //kg_weight
        }		
		data_array[8] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[0][1] & 0xff); // adc_values
        data_array[9] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[0][1] >> 8);
		data_array[10] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[1][1] & 0xff); // adc_values
        data_array[11] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[1][1] >> 8);
		data_array[12] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[2][1] & 0xff); // adc_values
        data_array[13] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[2][1] >> 8);
		data_array[14] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[3][1] & 0xff); // adc_values
        data_array[15] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[3][1] >> 8);
		data_array[16] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[4][1] & 0xff); // adc_values
        data_array[17] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[4][1] >> 8);	
		data_array[18] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[5][1] & 0xff); // adc_values
        data_array[19] = (uint8_t) (rt_vars->ui16_torque_sensor_calibration_ble_table[5][1] >> 8);	

		ble_nus_string_send(&m_nus, data_array, 20);
		
		//sprintf(data_array, "D1,%d,%d,%d,!",    rt_vars->ui8_duty_cycle, rt_vars->ui16_pedal_torque_x100, rt_vars->ui16_adc_pedal_torque_sensor);
	    //	ble_nus_string_send(&m_nus, data_array, strlen(data_array));

          
        break;
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
	 
		if(p_data[1] == 0x01){
	  
      ui_vars->ui8_motor_type = p_data[2];
      ui_vars->ui8_motor_temperature_min_value_to_limit = p_data[3];
      ui_vars->ui8_motor_temperature_max_value_to_limit = p_data[4];
      ui_vars->ui8_motor_acceleration = p_data[5];
      ui_vars->ui8_number_of_assist_levels = p_data[6];
      ui_vars->ui8_eMTB_assist_level = p_data[7];		
      ui_vars->ui8_torque_sensor_calibration_feature_enabled = p_data[8];
      ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100 = p_data[9]; 
      ui_vars->ui8_optional_ADC_function = p_data[10]; 
      ui_vars->ui8_motor_assistance_startup_without_pedal_rotation = p_data[11]; 
      ui_vars->ui8_lights_configuration = p_data[12]; 
      ui_vars->ui16_wheel_perimeter = (((uint16_t) p_data[14]) << 8) + ((uint16_t) p_data[13]); 
      ui_vars->ui8_walk_assist_feature_enabled = p_data[15]; 
      ui_vars->ui16_battery_voltage_reset_wh_counter_x10 = (((uint16_t) p_data[17]) << 8) + ((uint16_t) p_data[16]); 
      ui_vars->ui8_battery_max_current = p_data[18]; 
      ui_vars->ui8_target_max_battery_power_div25 = p_data[19]; 
	}
	
		if(p_data[1] == 0x02){
  
      ui_vars->ui16_battery_pack_resistance_x1000 = (((uint16_t) p_data[3]) << 8) + ((uint16_t) p_data[2]);
      ui_vars->ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) p_data[5]) << 8) + ((uint16_t) p_data[4]);
      ui_vars->ui8_street_mode_enabled = p_data[6];
      ui_vars->ui8_street_mode_throttle_enabled = p_data[7];
      ui_vars->ui8_street_mode_power_limit_div25 = p_data[8]; 
      ui_vars->ui8_street_mode_speed_limit = p_data[9]; 
	  ui_vars->ui8_field_weakening_enabled = p_data[10];
	  ui_vars->ui8_field_weakening_current = p_data[11];
	  ui_vars->ui8_cadence_RPM_limit = p_data[12];	
  	  ui_vars->ui8_torque_boost_factor = p_data[13];
	  ui_vars->ui8_battery_soc_enable = p_data[14];
	}
	
		if(p_data[1] == 0x03){
  
      for (int i = 0; i < 5; i++)
      ui_vars->ui8_target_peak_battery_power_div25[i] = p_data[2 + i];
      for (int i = 0; i < 5; i++)
      ui_vars->ui8_motor_acceleration_level[i] = p_data[7 + i];
      for (int i = 0; i < 5; i++)
      ui_vars->ui8_walk_assist_level_factor[i] = p_data[12 + i];	  
	  
	}	
	
		if(p_data[1] == 0x04){
		
      for (int i = 0; i < 5; i++)
      ui_vars->ui8_assist_level_power_assist[i] = p_data[2 + i];
	  ui_vars->ui32_wh_x10_100_percent = ((((uint32_t) p_data[9]) << 16) + (((uint32_t) p_data[8]) << 8) + ((uint32_t) p_data[7]));
	  ui_vars->ui32_wh_x10_offset = ((((uint32_t) p_data[12]) << 16) + (((uint32_t) p_data[11]) << 8) + ((uint32_t) p_data[10]));
 
	}
		if(p_data[1] == 0x05){

	  for (uint8_t i = 0; i < 6; i++) 
      ui_vars->ui16_torque_sensor_calibration_table[i][0] = p_data[2 + i]; //kg values

      ui_vars->ui16_torque_sensor_calibration_table[0][1] = (((uint16_t) p_data[9]) << 8) + ((uint16_t) p_data[8]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[1][1] = (((uint16_t) p_data[11]) << 8) + ((uint16_t) p_data[10]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[2][1] = (((uint16_t) p_data[13]) << 8) + ((uint16_t) p_data[12]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[3][1] = (((uint16_t) p_data[15]) << 8) + ((uint16_t) p_data[14]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[4][1] = (((uint16_t) p_data[17]) << 8) + ((uint16_t) p_data[16]); //adc values
	  ui_vars->ui16_torque_sensor_calibration_table[5][1] = (((uint16_t) p_data[19]) << 8) + ((uint16_t) p_data[18]); //adc values
	}
	
	ble_config_update = false;
 }