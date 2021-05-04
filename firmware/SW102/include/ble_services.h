#ifndef INCLUDE_BLE_SERVICES_H_
#define INCLUDE_BLE_SERVICES_H_
#include "state.h"

void ble_init(void);
void ble_uart_send(ui_vars_t *ui_vars);
void ble_send_status_data(ui_vars_t *ui_vars);
static void ble_config_set(uint8_t *data);
static void ble_nav_info(uint8_t *data);
static void ble_motor_test(uint8_t *data);
static void ble_reset_trip_stats();
static void ble_config_defaults();
void advertising_start(bool erase_bonds);
void delete_bonds(void);
void disconnect_stop_adv(void);

#endif /* INCLUDE_BLE_SERVICES_H_ */
