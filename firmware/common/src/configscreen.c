#include "screen.h"
#include "mainscreen.h"
#include "configscreen.h"
#include "eeprom.h"


static Field tripMenus[] =
{
		FIELD_EDITABLE_ENUM(_S("Reset stats", "Rst stats"), &ui8_g_configuration_trip_reset, "no", "yes"),
		FIELD_READONLY_UINT(_S("Trip time", "Trip time"), &ui16_g_trip_time, "h", false, .div_digits = 2),
		FIELD_READONLY_UINT(_S("Trip distance", "Trip dist"), &ui_vars.ui32_trip_distance_x100, "km", false, .div_digits = 2),
		FIELD_READONLY_UINT(_S("Max speed", "Max speed"), &ui_vars.ui16_trip_max_speed_x10, "km/h", false, .div_digits = 1),
		FIELD_READONLY_UINT(_S("Trip avg speed", "Avg speed"), &ui_vars.ui16_trip_avg_speed_x10, "km/h", false, .div_digits = 1),	
		FIELD_READONLY_UINT(_S("Avg battery current", "Avg curr"), &ui_vars.ui16_battery_current_avg, "amps", false, .div_digits = 1),
	    FIELD_READONLY_UINT(_S("Avg motor power", "Avg motPwr"), &ui_vars.ui16_battery_power_avg, "watts", false),
		FIELD_READONLY_UINT(_S("Avg pedal power", "Avg pedPwr"), &ui_vars.ui16_pedal_power_avg, "watts", false),
		FIELD_READONLY_UINT(_S("Avg pedal cadence", "Avg cad"), &ui_vars.ui16_pedal_cadence_avg, "RPM", false),
		FIELD_READONLY_UINT(_S("Avg energy consmp", "Avg energy"), &ui_vars.ui16_battery_energy_h_km_avg_x100, "Wh/km", false, .div_digits = 2),		
		FIELD_END };
	
static Field wheelMenus[] =
		{
		FIELD_EDITABLE_UINT("Max speed", &ui_vars.wheel_max_speed_x10, "kph", 1, 990, .div_digits = 1, .inc_step = 10, .hide_fraction = true),
		FIELD_EDITABLE_UINT(_S("Circumference", "Circumfere"), &ui_vars.ui16_wheel_perimeter, "mm", 750, 3000, .inc_step = 10),
		FIELD_END };

static Field batteryMenus[] =
		{
		FIELD_EDITABLE_UINT(_S("Max current", "Max curren"), &ui_vars.ui8_battery_max_current, "amps", 1, 18),
		FIELD_EDITABLE_UINT(_S("Low cut-off", "Lo cut-off"), &ui_vars.ui16_battery_low_voltage_cut_off_x10, "volts", 160, 630, .div_digits = 1),
        FIELD_EDITABLE_UINT(_S("Battery resistance", "Bat resist"), &ui_vars.ui16_battery_pack_resistance_x1000, "mohm", 0, 1000),
        FIELD_READONLY_UINT(_S("Voltage est", "Voltag est"), &ui_vars.ui16_battery_voltage_soc_x10, "volts", false, .div_digits = 1),		
		FIELD_READONLY_UINT(_S("Power loss est", "Power loss"), &ui_vars.ui16_battery_power_loss, "watts", false, .div_digits = 0),
		FIELD_READONLY_UINT(_S("Resistance est", "Resist est"), &ui_vars.ui16_battery_pack_resistance_x1000, "mohm", false),
		FIELD_END };

static Field batterySOCMenus[] =
		{
		FIELD_EDITABLE_ENUM(_S("Manual reset", "Manual rst"), &ui8_g_configuration_battery_soc_reset, "no", "yes"),
		FIELD_EDITABLE_UINT(_S("Reset at voltage", "Reset at"), &ui_vars.ui16_battery_voltage_reset_wh_counter_x10, "volts", 160, 630, .div_digits = 1),
		FIELD_EDITABLE_UINT(_S("Battery total Wh", "Batt total"), &ui_vars.ui32_wh_x10_100_percent, "whr", 0, 9990, .div_digits = 1, .inc_step = 100),
		FIELD_EDITABLE_UINT("Used Wh", &ui_vars.ui32_wh_x10, "whr", 0, 99900, .div_digits = 1, .inc_step = 100, .onSetEditable = onSetConfigurationBatterySOCUsedWh),
		FIELD_END };

static Field motorMenus[] = {
        FIELD_EDITABLE_ENUM(_S("Motor voltage", "Motor volt"), &ui_vars.ui8_motor_type, "48V", "36V"),
        FIELD_EDITABLE_UINT(_S("Motor power max", "Power max"), &ui_vars.ui16_target_max_battery_power, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),		
		FIELD_EDITABLE_ENUM(_S("Field weakening", "Field wk"), &ui_vars.ui8_field_weakening_enabled, "disable", "enable"),
		FIELD_EDITABLE_UINT(_S("Field Weak ADC Step", "FW ADC"), &ui_vars.ui8_field_weakening_current_adc, "adc", 0, 25, .inc_step = 5),
        FIELD_EDITABLE_UINT(_S("Motor accel", "Mot accel"), &ui_vars.ui8_motor_acceleration, "value", 0, 100),
		FIELD_EDITABLE_UINT(_S("Min current ADC", "MinADCcur"), &ui_vars.ui8_motor_current_min_adc, "amps", 0, 3), // 1 ADC step = 0.16 amp
		FIELD_END };

static Field torqueSensorMenus[] =
    {
		FIELD_EDITABLE_UINT(_S("0-cad assist thd", "0-cad thd"), &ui_vars.ui8_assist_without_pedal_rotation_threshold, "value", 0, 100),
		FIELD_EDITABLE_ENUM(_S("TQS Calibration", "TQS Calib"), &ui_vars.ui8_torque_sensor_calibration_feature_enabled, "disable", "enable"),		
	    FIELD_EDITABLE_UINT(_S("Weight adjustment", "Weight adj"), &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100, "value", 0, 100),
		FIELD_READONLY_UINT(_S("Weight", "Weight"), &ui_vars.ui16_pedal_weight, "kg", .div_digits = 1),
		FIELD_READONLY_UINT(_S("Pedal torque", "Pedal Tor"), &ui_vars.ui16_pedal_torque_x100, "Nm"),
		FIELD_READONLY_UINT(_S("ADC torque sensor", "ADC torque"), &ui_vars.ui16_adc_pedal_torque_sensor, "adc",),
#ifndef SW102
        FIELD_EDITABLE_UINT(_S("weight 1", "weight 1"), &ui_vars.ui16_torque_sensor_calibration_table[0][0], "kg", 0, 150),
        FIELD_EDITABLE_UINT("ADC 1", &ui_vars.ui16_torque_sensor_calibration_table[0][1], "", 0, 400),
        FIELD_EDITABLE_UINT(_S("weight 2", "weight 2"), &ui_vars.ui16_torque_sensor_calibration_table[1][0], "kg", 0, 150),
        FIELD_EDITABLE_UINT("ADC 2", &ui_vars.ui16_torque_sensor_calibration_table[1][1], "", 0, 400),
        FIELD_EDITABLE_UINT(_S("weight 3", "weight 3"), &ui_vars.ui16_torque_sensor_calibration_table[2][0], "kg", 0, 150),
        FIELD_EDITABLE_UINT("ADC 3", &ui_vars.ui16_torque_sensor_calibration_table[2][1], "", 0, 400),
        FIELD_EDITABLE_UINT(_S("weight 4", "weight 4"), &ui_vars.ui16_torque_sensor_calibration_table[3][0], "kg", 0, 150),
        FIELD_EDITABLE_UINT("ADC 4", &ui_vars.ui16_torque_sensor_calibration_table[3][1], "", 0, 400),
        FIELD_EDITABLE_UINT(_S("weight 5", "weight 5"), &ui_vars.ui16_torque_sensor_calibration_table[4][0], "kg", 0, 150),
        FIELD_EDITABLE_UINT("ADC 5", &ui_vars.ui16_torque_sensor_calibration_table[4][1], "", 0, 400),	
		FIELD_EDITABLE_UINT(_S("weight 6", "weight 6"), &ui_vars.ui16_torque_sensor_calibration_table[5][0], "kg", 0, 150),
        FIELD_EDITABLE_UINT("ADC 6", &ui_vars.ui16_torque_sensor_calibration_table[5][1], "", 0, 400),	
#endif
		FIELD_END };

static Field assistMenus[] =
		{
		//FIELD_EDITABLE_ENUM(_S("Assist mode", "Asst mode"), &ui_vars.ui8_riding_mode, "OFF", "PAM"),
		FIELD_EDITABLE_UINT(_S("eMTB sensitivity", "eMTB sens"), &ui_vars.ui8_eMTB_assist_level, "level", 1, 10),						
		FIELD_EDITABLE_UINT(_S("Num assist levels", "Num Levels"), &ui_vars.ui8_number_of_assist_levels, "value", 1, 5),
		FIELD_EDITABLE_ENUM("Soft start", &ui_vars.ui8_soft_start_feature_enabled, "disable", "enable"),
		FIELD_EDITABLE_ENUM("Hybrid mode", &ui_vars.ui8_hybrid_mode_enabled, "disable", "enable"),
#ifndef SW102		
		FIELD_EDITABLE_UINT("PA Level 1", &ui_vars.ui8_assist_level_power_assist[0], "", 0, 50, .div_digits = 1),
		FIELD_EDITABLE_UINT("PA Level 2", &ui_vars.ui8_assist_level_power_assist[1], "", 0, 50, .div_digits = 1),
		FIELD_EDITABLE_UINT("PA Level 3", &ui_vars.ui8_assist_level_power_assist[2], "", 0, 50, .div_digits = 1),
		FIELD_EDITABLE_UINT("PA Level 4", &ui_vars.ui8_assist_level_power_assist[3], "", 0, 50, .div_digits = 1),
		FIELD_EDITABLE_UINT("PA Level 5", &ui_vars.ui8_assist_level_power_assist[4], "", 0, 50, .div_digits = 1),
		FIELD_EDITABLE_UINT("TQ Level 1", &ui_vars.ui8_assist_level_torque_assist[0], "", 0, 120),
		FIELD_EDITABLE_UINT("TQ Level 2", &ui_vars.ui8_assist_level_torque_assist[1], "", 0, 120),
		FIELD_EDITABLE_UINT("TQ Level 3", &ui_vars.ui8_assist_level_torque_assist[2], "", 0, 120),
		FIELD_EDITABLE_UINT("TQ Level 4", &ui_vars.ui8_assist_level_torque_assist[3], "", 0, 120),
		FIELD_EDITABLE_UINT("TQ Level 5", &ui_vars.ui8_assist_level_torque_assist[4], "", 0, 120),
		FIELD_EDITABLE_UINT("Acc Lvl 1", &ui_vars.ui8_motor_acceleration_level[0], "", 0, 100),
		FIELD_EDITABLE_UINT("Acc Lvl 2", &ui_vars.ui8_motor_acceleration_level[1], "", 0, 100),
		FIELD_EDITABLE_UINT("Acc Lvl 3", &ui_vars.ui8_motor_acceleration_level[2], "", 0, 100),
		FIELD_EDITABLE_UINT("Acc Lvl 4", &ui_vars.ui8_motor_acceleration_level[3], "", 0, 100),
		FIELD_EDITABLE_UINT("Acc Lvl 5", &ui_vars.ui8_motor_acceleration_level[4], "", 0, 100),
		FIELD_EDITABLE_UINT("Peek Power 1", &ui_vars.ui8_target_peak_battery_power_div25[0], "", 0, 40),
		FIELD_EDITABLE_UINT("Peek Power 2", &ui_vars.ui8_target_peak_battery_power_div25[1], "", 0, 40),
		FIELD_EDITABLE_UINT("Peek Power 3", &ui_vars.ui8_target_peak_battery_power_div25[2], "", 0, 40),
		FIELD_EDITABLE_UINT("Peek Power 4", &ui_vars.ui8_target_peak_battery_power_div25[3], "", 0, 40),
		FIELD_EDITABLE_UINT("Peek Power 5", &ui_vars.ui8_target_peak_battery_power_div25[4], "", 0, 40),		
#endif
		FIELD_END };

static Field walkAssistMenus[] =
		{
		FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_walk_assist_feature_enabled, "disable", "enable"), // FIXME, share one array of disable/enable strings
#ifndef SW102			
		FIELD_EDITABLE_UINT("Level 1", &ui_vars.ui8_walk_assist_level_factor[0], "", 0, 100),
		FIELD_EDITABLE_UINT("Level 2", &ui_vars.ui8_walk_assist_level_factor[1], "", 0, 100),
		FIELD_EDITABLE_UINT("Level 3", &ui_vars.ui8_walk_assist_level_factor[2], "", 0, 100),
		FIELD_EDITABLE_UINT("Level 4", &ui_vars.ui8_walk_assist_level_factor[3], "", 0, 100),
		FIELD_EDITABLE_UINT("Level 5", &ui_vars.ui8_walk_assist_level_factor[4], "", 0, 255),
#endif		
		FIELD_END };

static Field motorTempMenus[] =
		{
		FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_optional_ADC_function, "disable", "temp", "throttle"),
		FIELD_EDITABLE_UINT("Min limit", &ui_vars.ui8_motor_temperature_min_value_to_limit, "C", 0, 255),
		FIELD_EDITABLE_UINT("Max limit", &ui_vars.ui8_motor_temperature_max_value_to_limit, "C", 0, 255),
		FIELD_END };

static Field streetModeMenus[] =
    {
		FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_street_mode_feature_enabled, "disable", "enable"),
		//FIELD_EDITABLE_ENUM(_S("Enable at startup", "Enabl stup"), &ui_vars.ui8_street_mode_enabled_on_startup, "no", "yes"),
		FIELD_EDITABLE_UINT(_S("Speed limit", "Speed limt"), &ui_vars.ui8_street_mode_speed_limit, "kph", 1, 99, .div_digits = 0, .inc_step = 1, .hide_fraction = true),
		FIELD_EDITABLE_UINT(_S("Motor power limit", "Power limt"), &ui_vars.ui16_street_mode_power_limit, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
		FIELD_EDITABLE_ENUM(_S("Throttle enable", "Throt enab"), &ui_vars.ui8_street_mode_throttle_enabled, "no", "yes"),
		FIELD_END };

static Field displayMenus[] =
		{
	    FIELD_EDITABLE_ENUM("Time field", &ui_vars.ui8_time_field_enable, "disable", "time", "SOC %", "volts"),
#ifndef SW102		
		FIELD_EDITABLE_UINT("Clock hours", &ui8_g_configuration_clock_hours, "", 0, 23, .onSetEditable = onSetConfigurationClockHours),
		FIELD_EDITABLE_UINT("Clock minutes", &ui8_g_configuration_clock_minutes, "", 0, 59, .onSetEditable = onSetConfigurationClockMinutes),
		FIELD_EDITABLE_UINT("Brightness on", &ui_vars.ui8_lcd_backlight_on_brightness, "", 5, 100, .inc_step = 5, .onSetEditable = onSetConfigurationDisplayLcdBacklightOnBrightness),
		FIELD_EDITABLE_UINT("Brightness off", &ui_vars.ui8_lcd_backlight_off_brightness, "", 5, 100, .inc_step = 5, .onSetEditable = onSetConfigurationDisplayLcdBacklightOffBrightness),
		FIELD_EDITABLE_ENUM("Buttons invert", &ui_vars.ui8_buttons_up_down_invert, "default", "invert"),
#else		
		FIELD_EDITABLE_ENUM(_S("Reset BLE connections", "Reset BLE"), &ui8_g_configuration_display_reset_bluetooth_peers, "no", "yes"),
#endif		
		FIELD_EDITABLE_ENUM(_S("Reset to defaults", "Reset def"), &ui8_g_configuration_display_reset_to_defaults, "no", "yes"),
        FIELD_EDITABLE_ENUM(_S("Plus long press", "PlusLngPr"), &ui_vars.ui8_plus_long_press_switch, "qmenu", "nxtscr", "light"),		
		FIELD_EDITABLE_UINT(_S("Auto power off", "Auto p off"), &ui_vars.ui8_lcd_power_off_time_minutes, "mins", 0, 255),		
		FIELD_EDITABLE_ENUM("Units", &ui_vars.ui8_units_type, "SI", "Imperial"),
#ifndef SW102
//  FIELD_READONLY_STRING("Display firmware", ""),
//  FIELD_READONLY_STRING("TSDZ2 firmware", ""),
		FIELD_READONLY_ENUM("LCD type", &g_lcd_ic_type, "ILI9481", "ST7796", "unknown"),
#endif
		FIELD_END };

static Field variousMenus[] = {
	    //FIELD_EDITABLE_UINT("Odometer", &ui_vars.ui32_odometer_x10, "km", 0, UINT32_MAX, .div_digits = 1, .inc_step = 100, .onSetEditable = onSetConfigurationWheelOdometer),
		FIELD_EDITABLE_UINT(_S("EnSavMode lvl", "EnSavMode"), &ui_vars.ui8_energy_saving_mode_level, "%", 0, 100),
		FIELD_EDITABLE_UINT(_S("Lights configuration", "Light conf"), &ui_vars.ui8_lights_configuration, "", 0, 8),
		FIELD_END };

#ifndef SW102

static Field varSpeedMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsWheelSpeed].auto_max_min, "auto", "man", "semi"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsWheelSpeed].max, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsWheelSpeed].min, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsWheelSpeed].auto_thresholds, "disabled", "manual", "auto"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsWheelSpeed].config_error_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsWheelSpeed].config_warn_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_END };

static Field varTripDistanceMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsTripDistance].auto_max_min, "yes", "no"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsTripDistance].max, "km", 0, INT32_MAX, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsTripDistance].min, "km", 0, INT32_MAX, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsTripDistance].auto_thresholds, "disabled", "manual", "auto"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsTripDistance].config_error_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsTripDistance].config_warn_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_END };

static Field varCadenceMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsCadence].auto_max_min, "auto", "man"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsCadence].max, "", 0, 200, .inc_step = 1),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsCadence].min, "", 0, 200, .inc_step = 1),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsCadence].auto_thresholds, "disabled", "manual", "auto"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsCadence].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsCadence].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_END };

static Field varHumanPowerMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsHumanPower].auto_max_min, "auto", "man"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsHumanPower].max, "", 0, 5000, .inc_step = 10),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsHumanPower].min, "", 0, 5000, .inc_step = 10),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsHumanPower].auto_thresholds, "disabled", "manual"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsHumanPower].config_error_threshold, "", 0, 20000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsHumanPower].config_warn_threshold, "", 0, 20000, .div_digits = 1, .inc_step = 10),
		FIELD_END };

static Field varBatteryPowerMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryPower].auto_max_min, "auto", "man"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryPower].max, "", 0, 5000, .inc_step = 10),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryPower].min, "", 0, 5000, .inc_step = 10),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryPower].auto_thresholds, "disabled", "manual"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryPower].config_error_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryPower].config_warn_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
		FIELD_END };

static Field varBatteryPowerUsageMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryPowerUsage].auto_max_min, "auto", "man"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryPowerUsage].max, "", 0, 5000, .inc_step = 10),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryPowerUsage].min, "", 0, 5000, .inc_step = 10),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryPowerUsage].auto_thresholds, "disabled", "manual"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryPowerUsage].config_error_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryPowerUsage].config_warn_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
		FIELD_END };

static Field varBatteryVoltageMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryVoltage].auto_max_min, "auto", "man", "semi"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryVoltage].max, "", 0, 1000, .div_digits = 1, .inc_step = 1),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryVoltage].min, "", 0, 1000, .div_digits = 1, .inc_step = 1),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryVoltage].auto_thresholds, "disabled", "manual", "auto"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryVoltage].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryVoltage].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_END };

static Field varBatteryCurrentMenus[] = {
		FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryCurrent].auto_max_min, "auto", "man", "semi"),
		FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryCurrent].max, "", 0, 50, .inc_step = 1),
		FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryCurrent].min, "", 0, 50, .inc_step = 1),
		FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryCurrent].auto_thresholds, "disabled", "manual", "auto"),
		FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryCurrent].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryCurrent].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
		FIELD_END };
 
/*
static Field varMotorCurrentMenus[] = {
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorCurrent].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorCurrent].max, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorCurrent].min, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorCurrent].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorCurrent].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorCurrent].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
  FIELD_END };

static Field varBatterySOCMenus[] = {
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatterySOC].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatterySOC].max, "", 0, 100, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatterySOC].min, "", 0, 100, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatterySOC].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatterySOC].config_error_threshold, "", 0, 200, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatterySOC].config_warn_threshold, "", 0, 200, .div_digits = 1, .inc_step = 1),
  FIELD_END };
*/
static Field varMotorTempMenus[] = {
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorTemp].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorTemp].max, "C", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorTemp].min, "C", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorTemp].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorTemp].config_error_threshold, "C", 0, 200, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorTemp].config_warn_threshold, "C", 0, 200, .div_digits = 1, .inc_step = 1),
  FIELD_END };

static Field varMotorERPSMenus[] = {
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorERPS].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorERPS].max, "", 0, 2000, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorERPS].min, "", 0, 2000, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorERPS].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorERPS].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorERPS].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 1),
  FIELD_END };

static Field varMotorPWMMenus[] = {
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorPWM].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorPWM].max, "", 0, 255, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorPWM].min, "", 0, 255, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorPWM].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorPWM].config_error_threshold, "", 0, 500, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorPWM].config_warn_threshold, "", 0, 500, .div_digits = 1, .inc_step = 1),
  FIELD_END };

static Field varMotorFOCMenus[] = {
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorFOC].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorFOC].max, "", 0, 60, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorFOC].min, "", 0, 60, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorFOC].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorFOC].config_error_threshold, "", 0, 120, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorFOC].config_warn_threshold, "", 0, 120, .div_digits = 1, .inc_step = 1),
  FIELD_END };

static Field variablesMenus[] = {
  FIELD_SCROLLABLE("Speed", varSpeedMenus),
  FIELD_SCROLLABLE(_S("Trip distance", "Trip dist"), varTripDistanceMenus),
  FIELD_SCROLLABLE("Cadence", varCadenceMenus),
  FIELD_SCROLLABLE(_S("human power", "human powr"), varHumanPowerMenus),
  FIELD_SCROLLABLE(_S("motor power", "motor powr"), varBatteryPowerMenus),
  FIELD_SCROLLABLE(_S("Watts/km", "Watts/km"), varBatteryPowerUsageMenus),
  FIELD_SCROLLABLE(_S("batt voltage", "bat volts"), varBatteryVoltageMenus),
  FIELD_SCROLLABLE(_S("batt current", "bat curren"), varBatteryCurrentMenus),
  //FIELD_SCROLLABLE(_S("battery SOC", "bat SOC"), varBatterySOCMenus),
  //FIELD_SCROLLABLE(_S("motor current", "mot curren"), varMotorCurrentMenus),
  FIELD_SCROLLABLE(_S("motor temp", "mot temp"), varMotorTempMenus),
  FIELD_SCROLLABLE(_S("motor speed", "mot speed"), varMotorERPSMenus),
  FIELD_SCROLLABLE(_S("motor pwm", "mot pwm"), varMotorPWMMenus),
  FIELD_SCROLLABLE(_S("motor foc", "mot foc"), varMotorFOCMenus),
  FIELD_END };


static Field technicalMenus[] = {
  FIELD_READONLY_UINT(_S("ADC throttle sensor", "ADC thrott"), &ui_vars.ui8_adc_throttle, ""),
  FIELD_READONLY_UINT(_S("Throttle sensor", "Throttle s"), &ui_vars.ui8_throttle, ""),
  FIELD_READONLY_UINT(_S("ADC torque sensor", "ADC torque"), &ui_vars.ui16_adc_pedal_torque_sensor, ""),
  FIELD_READONLY_UINT(_S("Pedal torque", "Pedal Tor"), &ui_vars.ui16_pedal_torque_x100, "Nm"),
  FIELD_READONLY_UINT(_S("Pedal cadence", "Cadence"), &ui_vars.ui8_pedal_cadence, "rpm"),
  FIELD_READONLY_UINT(_S("PWM duty-cycle", "PWM duty"), &ui_vars.ui8_duty_cycle, ""),
  FIELD_READONLY_UINT(_S("Motor speed", "Mot speed"), &ui_vars.ui16_motor_speed_erps, ""),
  FIELD_READONLY_UINT("Motor FOC", &ui_vars.ui8_foc_angle, ""),
  FIELD_END };
#endif

static Field topMenus[] = {
  FIELD_SCROLLABLE("Display", displayMenus),
  FIELD_SCROLLABLE("Motor", motorMenus),
  FIELD_SCROLLABLE("Battery", batteryMenus),
  FIELD_SCROLLABLE("Wheel", wheelMenus),
  FIELD_SCROLLABLE(_S("Battery SOC", "Bat SOC"), batterySOCMenus),
  FIELD_SCROLLABLE(_S("Torque sensor", "Torque sen"), torqueSensorMenus),
  FIELD_SCROLLABLE(_S("Assist modes", "Assist M"), assistMenus),
  FIELD_SCROLLABLE(_S("Walk/Cruise modes", "Walk Mod"), walkAssistMenus),
  FIELD_SCROLLABLE(_S("Motor temperature", "Motor temp"), motorTempMenus),
  FIELD_SCROLLABLE(_S("Street mode", "Street mod"), streetModeMenus),
  FIELD_SCROLLABLE("Trip stats", tripMenus),
#ifndef SW102
  FIELD_SCROLLABLE("Variables", variablesMenus),
#endif
  FIELD_SCROLLABLE("Various", variousMenus),
#ifndef SW102  
  FIELD_SCROLLABLE("Technical infos", technicalMenus),
#endif  
  FIELD_END };
  
static Field quickMenus[] = {
  FIELD_SCROLLABLE("Trip stats", tripMenus),
  FIELD_EDITABLE_ENUM(_S("Reset BLE connections", "Reset BLE"), &ui8_g_configuration_display_reset_bluetooth_peers, "no", "yes"),
  FIELD_EDITABLE_ENUM(_S("Battery SOC rst", "SOC rst"), &ui8_g_configuration_battery_soc_reset, "no", "yes"),
  FIELD_EDITABLE_ENUM("Light", &ui_vars.ui8_lights, "disable", "enable"),
  FIELD_EDITABLE_UINT(_S("eMTB sensitivity", "eMTB sens"), &ui_vars.ui8_eMTB_assist_level, "level", 1, 10),
  FIELD_EDITABLE_ENUM(_S("Street Mode", "StreetMod"), &ui_vars.ui8_street_mode_feature_enabled, "disable", "enable"),
  FIELD_END };
  

static Field configRoot = FIELD_SCROLLABLE(_S("Configurations", "CONFIG"), topMenus);
static Field quickRoot = FIELD_SCROLLABLE(_S("Quick screen", "Quick Scr"), quickMenus);

uint8_t ui8_g_configuration_display_reset_to_defaults = 0;
uint8_t ui8_g_configuration_display_reset_bluetooth_peers = 0;
uint8_t ui8_g_configuration_battery_soc_reset = 0;
uint8_t ui8_g_configuration_trip_reset = 0;

uint32_t ui32_g_configuration_wh_100_percent = 0;
uint16_t ui16_g_trip_time = 0;
bool g_configscreen_state = false;


bool quickScreenOnPress(buttons_events_t events) {
  
   if ((events & DOWN_LONG_CLICK)) {
    showNextScreen();
    return true;
  }
     if ((events & UP_LONG_CLICK)) {
    screenShow(&configScreen);
    return true;
  }
   return false;
}


static void configScreenOnEnter() {
	g_configscreen_state = true;
	ui_vars.ui8_riding_mode = OFF_MODE;
	g_showNextScreenIndex =  g_showNextScreenPreviousIndex; // don't change to next screen on exit (power button push)
	// Set the font preference for this screen
	editable_label_font = &CONFIGURATIONS_TEXT_FONT;
	editable_value_font = &CONFIGURATIONS_TEXT_FONT;
	editable_units_font = &CONFIGURATIONS_TEXT_FONT;
}

static void quickScreenOnEnter() {
    g_configscreen_state = true;
	g_showNextScreenIndex =  g_showNextScreenPreviousIndex; // don't change to next screen on exit (power button push)
	// Set the font preference for this screen
	editable_label_font = &CONFIGURATIONS_TEXT_FONT;
	editable_value_font = &CONFIGURATIONS_TEXT_FONT;
	editable_units_font = &CONFIGURATIONS_TEXT_FONT;
}

static void configExit() {
    //out of configscreen
	g_configscreen_state = false;
	// save the variables on EEPROM
	eeprom_write_variables();
	set_conversions(); // we just changed units
    update_battery_power_usage_label();
	// we need to update riding_mode
	if (ui_vars.ui8_assist_level > ui_vars.ui8_number_of_assist_levels){
    ui_vars.ui8_riding_mode = eMTB_ASSIST_MODE;}else {ui_vars.ui8_riding_mode = POWER_ASSIST_MODE;}
	// send config to TSDZ2
    if (g_motor_init_state == MOTOR_INIT_READY)
       g_motor_init_state = MOTOR_UPDATE_CONFIG;

}

static void quickScreenExit() {
    g_configscreen_state = false;
}

static void configPreUpdate() {
  set_conversions(); // while in the config menu we might change units at any time - keep the display looking correct
}

//
// Screens
//
Screen configScreen = {
    .onExit = configExit,
    .onEnter = configScreenOnEnter,
    .onPreUpdate = configPreUpdate,

.fields = {
		{ .color = ColorNormal, .field = &configRoot },
		{ .field = NULL } } };

Screen quickScreen = {
    .onPress = quickScreenOnPress,
    .onExit = quickScreenExit,
    .onEnter = quickScreenOnEnter,

.fields = {
		{ .color = ColorNormal, .field = &quickRoot },
		{ .field = NULL } } };
