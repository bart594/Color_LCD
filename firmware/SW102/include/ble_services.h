
#ifndef INCLUDE_BLE_SERVICES_H_
#define INCLUDE_BLE_SERVICES_H_
#include "state.h"

void ble_init(void);
void ble_uart_send(rt_vars_t *rt_vars);
static void ble_config_set(uint8_t *data);
static void ble_nav_info(uint8_t *data);

#endif /* INCLUDE_BLE_SERVICES_H_ */
