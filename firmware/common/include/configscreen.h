#pragma once

void configscreen_show();

extern Screen configScreen;
extern uint8_t ui8_g_configuration_display_reset_bluetooth_peers;
extern uint8_t ui8_g_configuration_display_reset_to_defaults;
extern uint8_t ui8_g_configuration_trip_reset;
extern uint16_t ui16_g_trip_time;
extern uint32_t ui32_g_configuration_wh_100_percent;
extern bool g_configscreen_state;
