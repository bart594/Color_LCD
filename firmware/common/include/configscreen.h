#pragma once

void configscreen_show();

extern Screen configScreen;
extern Screen quickScreen;

bool quickScreenOnPress(buttons_events_t events);

extern uint8_t ui8_g_configuration_display_reset_bluetooth_peers;
extern uint8_t ui8_g_configuration_battery_soc_reset;
extern uint16_t ui16_g_trip_time;
extern uint32_t ui32_g_configuration_wh_100_percent;
extern bool g_configscreen_state;
