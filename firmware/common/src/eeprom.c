/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stdio.h"
#include <string.h>
#include "eeprom.h"
#include "eeprom_hw.h"
#include "main.h"
#include "mainscreen.h"
//#include "lcd_configurations.h"

static eeprom_data_t m_eeprom_data;

// get rid of some copypasta with this little wrapper for copying arrays between structs
#define COPY_ARRAY(dest, src, field) memcpy((dest)->field, (src)->field, sizeof((dest)->field))

const eeprom_data_t m_eeprom_data_defaults = {
    .eeprom_version = EEPROM_VERSION,
		.ui8_assist_level = DEFAULT_VALUE_ASSIST_LEVEL,
		.ui16_wheel_perimeter = DEFAULT_VALUE_WHEEL_PERIMETER,
		.ui8_wheel_max_speed = DEFAULT_VALUE_WHEEL_MAX_SPEED,
		.ui8_units_type = DEFAULT_VALUE_UNITS_TYPE,
		.ui32_wh_x10_offset = DEFAULT_VALUE_WH_X10_OFFSET,
		.ui32_wh_x10_100_percent = DEFAULT_VALUE_WH_X10_100_PERCENT,
		.ui8_time_field_enable = DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC,
		.ui8_battery_max_current = DEFAULT_VALUE_BATTERY_MAX_CURRENT,
		.ui8_target_max_battery_power_div25 = DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER_DIV25,
		.ui16_battery_low_voltage_cut_off_x10 = DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10,
		.ui8_motor_type = DEFAULT_VALUE_MOTOR_TYPE,
		.ui8_assist_without_pedal_rotation_threshold = DEFAULT_VALUE_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION,
		.ui8_number_of_assist_levels = DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS,
		.ui8_motor_temperature_min_value_to_limit =	DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,
		.ui8_motor_temperature_max_value_to_limit =	DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,
		.ui16_battery_voltage_reset_wh_counter_x10 = DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10,
		.ui8_lcd_power_off_time_minutes = DEFAULT_VALUE_LCD_POWER_OFF_TIME,
		.ui8_lcd_backlight_on_brightness =	DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS,
		.ui8_lcd_backlight_off_brightness =	DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS,
		.ui16_battery_pack_resistance_x1000 = DEFAULT_VALUE_BATTERY_PACK_RESISTANCE,
		.ui32_odometer_x10 = DEFAULT_VALUE_ODOMETER_X10,
		.ui8_walk_assist_feature_enabled = DEFAULT_VALUE_WALK_ASSIST_FEATURE_ENABLED,
		.ui8_walk_assist_level_factor = {
		DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_1,
		DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_2,
		DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_3,
		DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_4,
		DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_5 },
		.ui8_assist_level_power_assist = {
		DEFAULT_VALUE_ASSIST_LEVEL_POWER_1,
		DEFAULT_VALUE_ASSIST_LEVEL_POWER_2,
		DEFAULT_VALUE_ASSIST_LEVEL_POWER_3,
		DEFAULT_VALUE_ASSIST_LEVEL_POWER_4,
		DEFAULT_VALUE_ASSIST_LEVEL_POWER_5 },
		.ui8_assist_level_torque_assist = {
		DEFAULT_VALUE_ASSIST_LEVEL_TORQUE_1,
		DEFAULT_VALUE_ASSIST_LEVEL_TORQUE_2,
		DEFAULT_VALUE_ASSIST_LEVEL_TORQUE_3,
		DEFAULT_VALUE_ASSIST_LEVEL_TORQUE_4,
		DEFAULT_VALUE_ASSIST_LEVEL_TORQUE_5 },		
		.ui8_target_peak_battery_power_div25 = {
		DEFAULT_VALUE_PEAK_POWER_LEVEL_1,
		DEFAULT_VALUE_PEAK_POWER_LEVEL_2,
		DEFAULT_VALUE_PEAK_POWER_LEVEL_3,
		DEFAULT_VALUE_PEAK_POWER_LEVEL_4,
		DEFAULT_VALUE_PEAK_POWER_LEVEL_5 },
		.ui8_motor_acceleration_level = {
		DEFAULT_VALUE_MOTOR_ACCELERATION_LEVEL_1,
		DEFAULT_VALUE_MOTOR_ACCELERATION_LEVEL_2,
		DEFAULT_VALUE_MOTOR_ACCELERATION_LEVEL_3,
		DEFAULT_VALUE_MOTOR_ACCELERATION_LEVEL_4,
		DEFAULT_VALUE_MOTOR_ACCELERATION_LEVEL_5 },			

#ifdef SW102
  .field_selectors = {
    6, // human power
    7, // motor power

    1, // trip time
    2, // trip distance

    13, // ERPS
    14, // PWM
  },
#else
  .field_selectors = {
    1,  // odometer
    12, // human power
    0,  // up time
    13, // motor power

    2, // trip A distance
    4, // trip A avg speed
    3, // trip A time
    5, // trip A max speed

    20, // PWM
    15, // battery current
    19, // motor speed
    13, // motor power
  },

  .graphs_field_selectors = {
    0, // wheel speed
    3, // human power
    5, // battery voltage
  },
#endif

    .showNextScreenIndex = 0,
    .x_axis_scale = DEFAULT_VALUE_X_AXIS_SCALE,
    .ui8_buttons_up_down_invert = DEFAULT_VALUE_BUTTONS_UP_DOWN_INVERT,
	.ui8_riding_mode = DEFAULT_VALUE_RIDING_MODE,
	.ui8_eMTB_assist_level = DEFAULT_VALUE_EMTB_ASSIST_LEVEL,
	.ui8_optional_ADC_function = DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION,
	.ui8_motor_acceleration = DEFAULT_VALUE_MOTOR_ACCELERATION,
	.ui8_pedal_torque_per_10_bit_ADC_step_x100 = DEFAULT_VALUE_PEDAL_TORQUE_CALIBRATION,
	.ui8_cruise_function_target_speed_kph = DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_KPH,
	.ui8_field_weakening_enabled = DEFAULT_VALUE_FIELD_WEAKENING_ENABLED,
	.ui8_field_weakening_current_adc = DEFAULT_VALUE_FIELD_WEAKENING_CURRENT,		
	.ui8_hybrid_mode_enabled = DEFAULT_VALUE_HYBRID_MODE,	
	.ui8_soft_start_feature_enabled = DEFAULT_VALUE_SOFT_START_FEATURE,
	.ui8_motor_current_min_adc = DEFAULT_VALUE_MOTOR_CURRENT_MIN_ADC,
	.ui8_energy_saving_mode_level = DEFAULT_VALUE_ENERGY_SAVING_MODE,
	
#ifndef SW102
    // enable automatic graph max min for every variable
    .graph_eeprom[VarsWheelSpeed].auto_max_min = GRAPH_AUTO_MAX_MIN_MANUAL,
    .graph_eeprom[VarsWheelSpeed].max = 350, // 35 km/h
    .graph_eeprom[VarsWheelSpeed].min = 0,

    .graph_eeprom[VarsTripDistance].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsCadence].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsHumanPower].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsBatteryPower].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsBatteryPowerUsage].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    //.graph_eeprom[VarsBatteryVoltage].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsBatteryCurrent].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    //.graph_eeprom[VarsMotorCurrent].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    //.graph_eeprom[VarsBatterySOC].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsMotorTemp].auto_max_min = GRAPH_AUTO_MAX_MIN_SEMI_AUTO,
    .graph_eeprom[VarsMotorERPS].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
    .graph_eeprom[VarsMotorPWM].auto_max_min = GRAPH_AUTO_MAX_MIN_SEMI_AUTO,
    .graph_eeprom[VarsMotorFOC].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,

    .tripDistanceField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,

    .wheelSpeedField_auto_thresholds = FIELD_THRESHOLD_MANUAL,
    .wheelSpeedField_config_error_threshold = 350,
    .wheelSpeedField_config_warn_threshold = 300,
    .wheelSpeedField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,

    .cadenceField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .cadenceField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    .batteryPowerField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .batteryPowerField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    .batteryPowerUsageField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .batteryPowerUsageField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_15M,
    //.batteryVoltageField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    //.batteryVoltageField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    .batteryCurrentField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .batteryCurrentField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    //.motorCurrentField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    //.motorCurrentField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    .motorTempField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .motorTempField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_15M,
    .motorErpsField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .motorErpsField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    .pwmDutyField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .pwmDutyField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
    .motorFOCField_auto_thresholds = FIELD_THRESHOLD_AUTO,
    .motorFOCField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
#endif

    .ui16_torque_sensor_calibration_table[0][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_1,
    .ui16_torque_sensor_calibration_table[0][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_1,
    .ui16_torque_sensor_calibration_table[1][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_2,
    .ui16_torque_sensor_calibration_table[1][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_2,
    .ui16_torque_sensor_calibration_table[2][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_3,
    .ui16_torque_sensor_calibration_table[2][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_3,
    .ui16_torque_sensor_calibration_table[3][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_4,
    .ui16_torque_sensor_calibration_table[3][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_4,
    .ui16_torque_sensor_calibration_table[4][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_5,
    .ui16_torque_sensor_calibration_table[4][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_5,
    .ui16_torque_sensor_calibration_table[5][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_6,
    .ui16_torque_sensor_calibration_table[5][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_6,
   /* .ui16_torque_sensor_calibration_table[6][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_7,
    .ui16_torque_sensor_calibration_table[6][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_7,
    .ui16_torque_sensor_calibration_table[7][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_8,
    .ui16_torque_sensor_calibration_table[7][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_8,*/

    .ui8_hall_counter_offset[0] = DEFAULT_VALUE_HALL_COUNTER_OFFSET_UP,
	.ui8_hall_counter_offset[1] = DEFAULT_VALUE_HALL_COUNTER_OFFSET_DOWN,
	.ui8_hall_counter_offset[2] = DEFAULT_VALUE_HALL_COUNTER_OFFSET_UP,
	.ui8_hall_counter_offset[3] = DEFAULT_VALUE_HALL_COUNTER_OFFSET_DOWN,
	.ui8_hall_counter_offset[4] = DEFAULT_VALUE_HALL_COUNTER_OFFSET_UP,
	.ui8_hall_counter_offset[5] = DEFAULT_VALUE_HALL_COUNTER_OFFSET_DOWN,
	
	.ui8_hall_ref_angles[0] = DEFAULT_VALUE_PHASE_ROTOR_ANGLE_30,
	.ui8_hall_ref_angles[1] = DEFAULT_VALUE_PHASE_ROTOR_ANGLE_90,
	.ui8_hall_ref_angles[2] = DEFAULT_VALUE_PHASE_ROTOR_ANGLE_150,
	.ui8_hall_ref_angles[3] = DEFAULT_VALUE_PHASE_ROTOR_ANGLE_210,
	.ui8_hall_ref_angles[4] = DEFAULT_VALUE_PHASE_ROTOR_ANGLE_270,
	.ui8_hall_ref_angles[5] = DEFAULT_VALUE_PHASE_ROTOR_ANGLE_330,
	
	.ui8_street_mode_feature_enabled = DEFAULT_STREET_MODE_FEATURE_ENABLE,
    .ui8_street_mode_enabled_on_startup = DEFAULT_STREET_MODE_ENABLE_AT_STARTUP,
    .ui8_street_mode_enabled = DEFAULT_STREET_MODE_ENABLE,
    .ui8_street_mode_speed_limit = DEFAULT_STREET_MODE_SPEED_LIMIT,
    .ui8_street_mode_power_limit_div25 = DEFAULT_STREET_MODE_POWER_LIMIT_DIV25,
    .ui8_street_mode_throttle_enabled = DEFAULT_STREET_MODE_THROTTLE_ENABLE,
	.ui32_trip_distance_x1000 = DEFAULT_VALUE_TRIP_DISTANCE,
    .ui32_trip_time = DEFAULT_VALUE_TRIP_TIME,	
    .ui16_trip_max_speed_x10 = DEFAULT_VALUE_TRIP_MAX_SPEED,
    .ui16_trip_avg_speed_x10 = DEFAULT_VALUE_TRIP_AVG_SPEED,
    .ui16_battery_current_avg = DEFAULT_VALUE_AVG_CURRENT,
    .ui16_battery_power_avg = DEFAULT_VALUE_AVG_BATT_POWER,
  	.ui16_pedal_power_avg = DEFAULT_VALUE_PEDAL_POWER,
  	.ui16_pedal_cadence_avg = DEFAULT_VALUE_AVG_CADENCE,	
    .ui16_battery_energy_h_km_avg_x100 = DEFAULT_VALUE_AVG_ENERGY,
	.ui8_plus_long_press_switch	= DEFAULT_VALUE_LONG_PRESS,
};

void eeprom_init() {
	eeprom_hw_init();

	// read the values from EEPROM to array
	memset(&m_eeprom_data, 0, sizeof(m_eeprom_data));

	// if eeprom is blank use defaults
	// if eeprom version is less than the min required version, wipe and use defaults
	// if eeprom version is greater than the current app version, user must have downgraded - wipe and use defaults
	if (!flash_read_words(&m_eeprom_data,
			sizeof(m_eeprom_data)
					/ sizeof(uint32_t))
	    || m_eeprom_data.eeprom_version < EEPROM_MIN_COMPAT_VERSION
	    || m_eeprom_data.eeprom_version > EEPROM_VERSION
	    )
		// If we are using default data it doesn't get written to flash until someone calls write
		memcpy(&m_eeprom_data, &m_eeprom_data_defaults,
				sizeof(m_eeprom_data_defaults));

//	// Perform whatever migrations we need to update old eeprom formats
//	if (m_eeprom_data.eeprom_version < EEPROM_VERSION) {
//
//		m_eeprom_data.ui8_lcd_backlight_on_brightness =
//				m_eeprom_data_defaults.ui8_lcd_backlight_on_brightness;
//		m_eeprom_data.ui8_lcd_backlight_off_brightness =
//				m_eeprom_data_defaults.ui8_lcd_backlight_off_brightness;
//
//		m_eeprom_data.eeprom_version = EEPROM_VERSION;
//	}

	eeprom_init_variables();

	set_conversions();

	// prepare torque_sensor_calibration_table as it will be used at begin to init the motor
	prepare_torque_sensor_calibration_table();
}

void eeprom_init_variables(void) {
	ui_vars_t *ui_vars = get_ui_vars();
	rt_vars_t *rt_vars = get_rt_vars();

	// copy data final variables
	ui_vars->ui8_assist_level = m_eeprom_data.ui8_assist_level;
	ui_vars->ui16_wheel_perimeter = m_eeprom_data.ui16_wheel_perimeter;
	ui_vars->wheel_max_speed_x10 =
			m_eeprom_data.ui8_wheel_max_speed * 10;
	ui_vars->ui8_units_type = m_eeprom_data.ui8_units_type;
	ui_vars->ui32_wh_x10_offset = m_eeprom_data.ui32_wh_x10_offset;
	ui_vars->ui32_wh_x10_100_percent =
			m_eeprom_data.ui32_wh_x10_100_percent;
	ui_vars->ui8_time_field_enable =
			m_eeprom_data.ui8_time_field_enable;
    ui_vars->ui8_target_max_battery_power_div25 =
      m_eeprom_data.ui8_target_max_battery_power_div25;
	ui_vars->ui8_battery_max_current =
			m_eeprom_data.ui8_battery_max_current;
	ui_vars->ui16_battery_low_voltage_cut_off_x10 =
			m_eeprom_data.ui16_battery_low_voltage_cut_off_x10;
	ui_vars->ui8_motor_type = m_eeprom_data.ui8_motor_type;
	ui_vars->ui8_assist_without_pedal_rotation_threshold =
			m_eeprom_data.ui8_assist_without_pedal_rotation_threshold;
	ui_vars->ui8_number_of_assist_levels =
			m_eeprom_data.ui8_number_of_assist_levels;
	ui_vars->ui8_motor_temperature_min_value_to_limit =
			m_eeprom_data.ui8_motor_temperature_min_value_to_limit;
	ui_vars->ui8_motor_temperature_max_value_to_limit =
			m_eeprom_data.ui8_motor_temperature_max_value_to_limit;
	ui_vars->ui16_battery_voltage_reset_wh_counter_x10 =
			m_eeprom_data.ui16_battery_voltage_reset_wh_counter_x10;
	ui_vars->ui8_lcd_power_off_time_minutes =
			m_eeprom_data.ui8_lcd_power_off_time_minutes;
	ui_vars->ui8_lcd_backlight_on_brightness =
			m_eeprom_data.ui8_lcd_backlight_on_brightness;
	ui_vars->ui8_lcd_backlight_off_brightness =
			m_eeprom_data.ui8_lcd_backlight_off_brightness;
	ui_vars->ui16_battery_pack_resistance_x1000 =
			m_eeprom_data.ui16_battery_pack_resistance_x1000;
	rt_vars->ui32_odometer_x10 = m_eeprom_data.ui32_odometer_x10; // odometer value should reside on RT vars
	ui_vars->ui8_walk_assist_feature_enabled =
			m_eeprom_data.ui8_walk_assist_feature_enabled;
	ui_vars->ui8_riding_mode = m_eeprom_data.ui8_riding_mode;
	ui_vars->ui8_eMTB_assist_level = m_eeprom_data.ui8_eMTB_assist_level;
	ui_vars->ui8_optional_ADC_function = m_eeprom_data.ui8_optional_ADC_function;
	ui_vars->ui8_motor_acceleration = m_eeprom_data.ui8_motor_acceleration;
	ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100 = m_eeprom_data.ui8_pedal_torque_per_10_bit_ADC_step_x100;
	ui_vars->ui8_cruise_function_target_speed_kph = m_eeprom_data.ui8_cruise_function_target_speed_kph;			
	ui_vars->ui8_field_weakening_enabled = m_eeprom_data.ui8_field_weakening_enabled;
	ui_vars->ui8_field_weakening_current_adc = m_eeprom_data.ui8_field_weakening_current_adc;
	ui_vars->ui8_hybrid_mode_enabled = m_eeprom_data.ui8_hybrid_mode_enabled;	
	ui_vars->ui8_soft_start_feature_enabled = m_eeprom_data.ui8_soft_start_feature_enabled;	
	ui_vars->ui8_motor_current_min_adc =  m_eeprom_data.ui8_motor_current_min_adc;	
	ui_vars->ui8_energy_saving_mode_level = m_eeprom_data.ui8_energy_saving_mode_level;
	
	COPY_ARRAY(ui_vars, &m_eeprom_data, ui8_walk_assist_level_factor);
	COPY_ARRAY(ui_vars, &m_eeprom_data, field_selectors);
	COPY_ARRAY(ui_vars, &m_eeprom_data, graphs_field_selectors);
	COPY_ARRAY(ui_vars, &m_eeprom_data, ui8_assist_level_power_assist);
	COPY_ARRAY(ui_vars, &m_eeprom_data, ui8_assist_level_torque_assist);	
	COPY_ARRAY(ui_vars, &m_eeprom_data, ui8_target_peak_battery_power_div25);
    COPY_ARRAY(ui_vars, &m_eeprom_data, ui8_motor_acceleration_level);
	
  ui_vars->ui8_buttons_up_down_invert = m_eeprom_data.ui8_buttons_up_down_invert;
  
#ifndef SW102
  for (uint8_t i = 0; i < VARS_SIZE; i++) {
    g_graphVars[i].auto_max_min = m_eeprom_data.graph_eeprom[i].auto_max_min;
    g_graphVars[i].max = m_eeprom_data.graph_eeprom[i].max;
    g_graphVars[i].min = m_eeprom_data.graph_eeprom[i].min;
  }
  tripDistanceGraph.rw->graph.x_axis_scale_config = m_eeprom_data.tripDistanceField_x_axis_scale_config;
  graph_x_axis_scale_config_t temp = GRAPH_X_AXIS_SCALE_15M;
  if (tripDistanceGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = tripDistanceGraph.rw->graph.x_axis_scale_config;
  }
  tripDistanceGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsWheelSpeed].auto_thresholds = m_eeprom_data.wheelSpeedField_auto_thresholds;
  g_vars[VarsWheelSpeed].config_error_threshold = m_eeprom_data.wheelSpeedField_config_error_threshold;
  g_vars[VarsWheelSpeed].config_warn_threshold = m_eeprom_data.wheelSpeedField_config_warn_threshold;
  wheelSpeedGraph.rw->graph.x_axis_scale_config = m_eeprom_data.wheelSpeedField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (wheelSpeedGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = wheelSpeedGraph.rw->graph.x_axis_scale_config;
  }
  wheelSpeedGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsCadence].auto_thresholds = m_eeprom_data.cadenceField_auto_thresholds;
  g_vars[VarsCadence].config_error_threshold = m_eeprom_data.cadenceField_config_error_threshold;
  g_vars[VarsCadence].config_warn_threshold = m_eeprom_data.cadenceField_config_warn_threshold;
  cadenceGraph.rw->graph.x_axis_scale_config = m_eeprom_data.cadenceField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (cadenceGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = cadenceGraph.rw->graph.x_axis_scale_config;
  }
  cadenceGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsHumanPower].auto_thresholds = m_eeprom_data.humanPowerField_auto_thresholds;
  g_vars[VarsHumanPower].config_error_threshold = m_eeprom_data.humanPowerField_config_error_threshold;
  g_vars[VarsHumanPower].config_warn_threshold = m_eeprom_data.humanPowerField_config_warn_threshold;
  humanPowerGraph.rw->graph.x_axis_scale_config = m_eeprom_data.humanPowerField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (humanPowerGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = humanPowerGraph.rw->graph.x_axis_scale_config;
  }
  humanPowerGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryPower].auto_thresholds = m_eeprom_data.batteryPowerField_auto_thresholds;
  g_vars[VarsBatteryPower].config_error_threshold = m_eeprom_data.batteryPowerField_config_error_threshold;
  g_vars[VarsBatteryPower].config_warn_threshold = m_eeprom_data.batteryPowerField_config_warn_threshold;
  batteryPowerGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryPowerField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryPowerGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryPowerGraph.rw->graph.x_axis_scale_config;
  }
  batteryPowerGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryPowerUsage].auto_thresholds = m_eeprom_data.batteryPowerUsageField_auto_thresholds;
  g_vars[VarsBatteryPowerUsage].config_error_threshold = m_eeprom_data.batteryPowerUsageField_config_error_threshold;
  g_vars[VarsBatteryPowerUsage].config_warn_threshold = m_eeprom_data.batteryPowerUsageField_config_warn_threshold;
  batteryPowerGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryPowerUsageField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryPowerUsageGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryPowerUsageGraph.rw->graph.x_axis_scale_config;
  }
  batteryPowerUsageGraph.rw->graph.x_axis_scale = temp;
/*
  g_vars[VarsBatteryVoltage].auto_thresholds = m_eeprom_data.batteryVoltageField_auto_thresholds;
  g_vars[VarsBatteryVoltage].config_error_threshold = m_eeprom_data.batteryVoltageField_config_error_threshold;
  g_vars[VarsBatteryVoltage].config_warn_threshold = m_eeprom_data.batteryVoltageField_config_warn_threshold;
  batteryVoltageGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryVoltageField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryVoltageGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryVoltageGraph.rw->graph.x_axis_scale_config;
  }
  batteryVoltageGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryCurrent].auto_thresholds = m_eeprom_data.batteryCurrentField_auto_thresholds;
  g_vars[VarsBatteryCurrent].config_error_threshold = m_eeprom_data.batteryCurrentField_config_error_threshold;
  g_vars[VarsBatteryCurrent].config_warn_threshold = m_eeprom_data.batteryCurrentField_config_warn_threshold;
  batteryCurrentGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryCurrentField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryCurrentGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryCurrentGraph.rw->graph.x_axis_scale_config;
  }
  batteryCurrentGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorCurrent].auto_thresholds = m_eeprom_data.motorCurrentField_auto_thresholds;
  g_vars[VarsMotorCurrent].config_error_threshold = m_eeprom_data.motorCurrentField_config_error_threshold;
  g_vars[VarsMotorCurrent].config_warn_threshold = m_eeprom_data.motorCurrentField_config_warn_threshold;
  motorCurrentGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorCurrentField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorCurrentGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorCurrentGraph.rw->graph.x_axis_scale_config;
  }
  motorCurrentGraph.rw->graph.x_axis_scale = temp;
*/
  g_vars[VarsMotorTemp].auto_thresholds = m_eeprom_data.motorTempField_auto_thresholds;
  g_vars[VarsMotorTemp].config_error_threshold = m_eeprom_data.motorTempField_config_error_threshold;
  g_vars[VarsMotorTemp].config_warn_threshold = m_eeprom_data.motorTempField_config_warn_threshold;
  motorTempGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorTempField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorTempGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorTempGraph.rw->graph.x_axis_scale_config;
  }
  motorTempGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorERPS].auto_thresholds = m_eeprom_data.motorErpsField_auto_thresholds;
  g_vars[VarsMotorERPS].config_error_threshold = m_eeprom_data.motorErpsField_config_error_threshold;
  g_vars[VarsMotorERPS].config_warn_threshold = m_eeprom_data.motorErpsField_config_warn_threshold;
  motorErpsGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorErpsField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorErpsGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorErpsGraph.rw->graph.x_axis_scale_config;
  }
  motorErpsGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorPWM].auto_thresholds = m_eeprom_data.pwmDutyField_auto_thresholds;
  g_vars[VarsMotorPWM].config_error_threshold = m_eeprom_data.pwmDutyField_config_error_threshold;
  g_vars[VarsMotorPWM].config_warn_threshold = m_eeprom_data.pwmDutyField_config_warn_threshold;
  pwmDutyGraph.rw->graph.x_axis_scale_config = m_eeprom_data.pwmDutyField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (pwmDutyGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = pwmDutyGraph.rw->graph.x_axis_scale_config;
  }
  pwmDutyGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorFOC].auto_thresholds = m_eeprom_data.motorFOCField_auto_thresholds;
  g_vars[VarsMotorFOC].config_error_threshold = m_eeprom_data.motorFOCField_config_error_threshold;
  g_vars[VarsMotorFOC].config_warn_threshold = m_eeprom_data.motorFOCField_config_warn_threshold;
  motorFOCGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorFOCField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorFOCGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorFOCGraph.rw->graph.x_axis_scale_config;
  }
  motorFOCGraph.rw->graph.x_axis_scale = temp;
#endif

  ui_vars->ui8_torque_sensor_calibration_feature_enabled = m_eeprom_data.ui8_torque_sensor_calibration_feature_enabled;
  
  for (uint8_t i = 0; i < 6; i++) {
    ui_vars->ui16_torque_sensor_calibration_table[i][0] = m_eeprom_data.ui16_torque_sensor_calibration_table[i][0];
    ui_vars->ui16_torque_sensor_calibration_table[i][1] = m_eeprom_data.ui16_torque_sensor_calibration_table[i][1];
  }

  for (uint8_t i = 0; i < 6; i++)
  ui_vars->ui8_hall_ref_angles[i] = m_eeprom_data.ui8_hall_ref_angles[i];
  
  for (uint8_t i = 0; i < 6; i++)
  ui_vars->ui8_hall_counter_offset[i] = m_eeprom_data.ui8_hall_counter_offset[i];
  
  g_showNextScreenIndex = m_eeprom_data.showNextScreenIndex;

  ui_vars->ui8_street_mode_feature_enabled =
      m_eeprom_data.ui8_street_mode_feature_enabled;
  ui_vars->ui8_street_mode_enabled =
      m_eeprom_data.ui8_street_mode_enabled;
  ui_vars->ui8_street_mode_enabled_on_startup =
      m_eeprom_data.ui8_street_mode_enabled_on_startup;
  ui_vars->ui8_street_mode_speed_limit =
      m_eeprom_data.ui8_street_mode_speed_limit;
  ui_vars->ui8_street_mode_power_limit_div25 =
      m_eeprom_data.ui8_street_mode_power_limit_div25;
  ui_vars->ui8_street_mode_throttle_enabled =
      m_eeprom_data.ui8_street_mode_throttle_enabled;
  
  // trip values should reside on RT vars
  rt_vars->ui32_trip_distance_x1000 =
      m_eeprom_data.ui32_trip_distance_x1000;
  rt_vars->ui32_trip_time =
      m_eeprom_data.ui32_trip_time;	  
  rt_vars->ui16_trip_max_speed_x10 =
      m_eeprom_data.ui16_trip_max_speed_x10;	  
  rt_vars->ui16_trip_avg_speed_x10 =
      m_eeprom_data.ui16_trip_avg_speed_x10;
  rt_vars->ui16_battery_current_avg =
      m_eeprom_data.ui16_battery_current_avg;
  rt_vars->ui16_battery_power_avg =
      m_eeprom_data.ui16_battery_power_avg;
  rt_vars->ui16_pedal_power_avg =
      m_eeprom_data.ui16_pedal_power_avg; 
  rt_vars->ui16_pedal_cadence_avg =
      m_eeprom_data.ui16_pedal_cadence_avg;	
  rt_vars->ui16_battery_energy_h_km_avg_x100 =
      m_eeprom_data.ui16_battery_energy_h_km_avg_x100;
  ui_vars->ui8_plus_long_press_switch =
      m_eeprom_data.ui8_plus_long_press_switch;	  
	  
}

void eeprom_write_variables(void) {
	ui_vars_t *ui_vars = get_ui_vars();
	m_eeprom_data.ui8_assist_level = ui_vars->ui8_assist_level;
	m_eeprom_data.ui16_wheel_perimeter = ui_vars->ui16_wheel_perimeter;
	m_eeprom_data.ui8_wheel_max_speed =
			ui_vars->wheel_max_speed_x10 / 10;
	m_eeprom_data.ui8_units_type = ui_vars->ui8_units_type;
	m_eeprom_data.ui32_wh_x10_offset = ui_vars->ui32_wh_x10_offset;
	m_eeprom_data.ui32_wh_x10_100_percent =
			ui_vars->ui32_wh_x10_100_percent;
	m_eeprom_data.ui8_time_field_enable =
			ui_vars->ui8_time_field_enable;
	m_eeprom_data.ui8_battery_max_current =
			ui_vars->ui8_battery_max_current;			
    m_eeprom_data.ui8_target_max_battery_power_div25 =
			ui_vars->ui8_target_max_battery_power_div25;
	m_eeprom_data.ui8_motor_max_current =
			ui_vars->ui8_motor_max_current;
	m_eeprom_data.ui8_motor_current_min_adc =
			ui_vars->ui8_motor_current_min_adc;
	m_eeprom_data.ui16_battery_low_voltage_cut_off_x10 =
			ui_vars->ui16_battery_low_voltage_cut_off_x10;
	m_eeprom_data.ui8_motor_type = ui_vars->ui8_motor_type;
	m_eeprom_data.ui8_assist_without_pedal_rotation_threshold =
			ui_vars->ui8_assist_without_pedal_rotation_threshold;
	m_eeprom_data.ui8_number_of_assist_levels =
			ui_vars->ui8_number_of_assist_levels;
	m_eeprom_data.ui8_motor_temperature_min_value_to_limit =
			ui_vars->ui8_motor_temperature_min_value_to_limit;
	m_eeprom_data.ui8_motor_temperature_max_value_to_limit =
			ui_vars->ui8_motor_temperature_max_value_to_limit;
	m_eeprom_data.ui16_battery_voltage_reset_wh_counter_x10 =
			ui_vars->ui16_battery_voltage_reset_wh_counter_x10;
	m_eeprom_data.ui8_lcd_power_off_time_minutes =
			ui_vars->ui8_lcd_power_off_time_minutes;
	m_eeprom_data.ui8_lcd_backlight_on_brightness =
			ui_vars->ui8_lcd_backlight_on_brightness;
	m_eeprom_data.ui8_lcd_backlight_off_brightness =
			ui_vars->ui8_lcd_backlight_off_brightness;
	m_eeprom_data.ui16_battery_pack_resistance_x1000 =
			ui_vars->ui16_battery_pack_resistance_x1000;
	m_eeprom_data.ui32_odometer_x10 = ui_vars->ui32_odometer_x10;
	m_eeprom_data.ui8_walk_assist_feature_enabled =
			ui_vars->ui8_walk_assist_feature_enabled;
	m_eeprom_data.ui8_riding_mode =
			ui_vars->ui8_riding_mode;
	m_eeprom_data.ui8_eMTB_assist_level =
			ui_vars->ui8_eMTB_assist_level;
	m_eeprom_data.ui8_optional_ADC_function =
			ui_vars->ui8_optional_ADC_function;
	m_eeprom_data.ui8_motor_acceleration =
			ui_vars->ui8_motor_acceleration;	
	m_eeprom_data.ui8_pedal_torque_per_10_bit_ADC_step_x100 =
			ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100;	
	m_eeprom_data.ui8_cruise_function_target_speed_kph =
			ui_vars->ui8_cruise_function_target_speed_kph;	
	m_eeprom_data.ui8_field_weakening_enabled = ui_vars->ui8_field_weakening_enabled;
	m_eeprom_data.ui8_field_weakening_current_adc = ui_vars->ui8_field_weakening_current_adc;	
	m_eeprom_data.ui8_hybrid_mode_enabled = ui_vars->ui8_hybrid_mode_enabled;
	m_eeprom_data.ui8_soft_start_feature_enabled = ui_vars->ui8_soft_start_feature_enabled;
    m_eeprom_data.ui8_motor_current_min_adc = ui_vars->ui8_motor_current_min_adc;
	m_eeprom_data.ui8_energy_saving_mode_level = ui_vars->ui8_energy_saving_mode_level;
		
	COPY_ARRAY(&m_eeprom_data, ui_vars, ui8_walk_assist_level_factor);
	COPY_ARRAY(&m_eeprom_data, ui_vars, ui8_assist_level_power_assist);
	COPY_ARRAY(&m_eeprom_data, ui_vars, ui8_assist_level_torque_assist);	
	COPY_ARRAY(&m_eeprom_data, ui_vars, ui8_target_peak_battery_power_div25);
	COPY_ARRAY(&m_eeprom_data, ui_vars, ui8_motor_acceleration_level);
	COPY_ARRAY(&m_eeprom_data, ui_vars, field_selectors);
    COPY_ARRAY(&m_eeprom_data, ui_vars, graphs_field_selectors);
    
	m_eeprom_data.ui8_buttons_up_down_invert = ui_vars->ui8_buttons_up_down_invert;
    m_eeprom_data.ui8_torque_sensor_calibration_feature_enabled = ui_vars->ui8_torque_sensor_calibration_feature_enabled;
  
  for (uint8_t i = 0; i < 6; i++) {
    m_eeprom_data.ui16_torque_sensor_calibration_table[i][0] = ui_vars->ui16_torque_sensor_calibration_table[i][0];
    m_eeprom_data.ui16_torque_sensor_calibration_table[i][1] = ui_vars->ui16_torque_sensor_calibration_table[i][1];
  }
  
   for (uint8_t i = 0; i < 6; i++)
   m_eeprom_data.ui8_hall_ref_angles[i] = ui_vars->ui8_hall_ref_angles[i];
   
   for (uint8_t i = 0; i < 6; i++)
   m_eeprom_data.ui8_hall_counter_offset[i] = ui_vars->ui8_hall_counter_offset[i];


#ifndef SW102
  for (uint8_t i = 0; i < VARS_SIZE; i++) {
    m_eeprom_data.graph_eeprom[i].auto_max_min = g_graphVars[i].auto_max_min;
    m_eeprom_data.graph_eeprom[i].max = g_graphVars[i].max;
    m_eeprom_data.graph_eeprom[i].min = g_graphVars[i].min;
  }
  m_eeprom_data.wheelSpeedField_auto_thresholds = g_vars[VarsWheelSpeed].auto_thresholds;
  m_eeprom_data.wheelSpeedField_config_error_threshold = g_vars[VarsWheelSpeed].config_error_threshold;
  m_eeprom_data.wheelSpeedField_config_warn_threshold = g_vars[VarsWheelSpeed].config_warn_threshold;
  m_eeprom_data.wheelSpeedField_x_axis_scale_config = wheelSpeedGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.cadenceField_auto_thresholds = g_vars[VarsCadence].auto_thresholds;
  m_eeprom_data.cadenceField_config_error_threshold = g_vars[VarsCadence].config_error_threshold;
  m_eeprom_data.cadenceField_config_warn_threshold = g_vars[VarsCadence].config_warn_threshold;
  m_eeprom_data.cadenceField_x_axis_scale_config = cadenceGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.humanPowerField_auto_thresholds = g_vars[VarsHumanPower].auto_thresholds;
  m_eeprom_data.humanPowerField_config_error_threshold = g_vars[VarsHumanPower].config_error_threshold;
  m_eeprom_data.humanPowerField_config_warn_threshold = g_vars[VarsHumanPower].config_warn_threshold;
  m_eeprom_data.humanPowerField_x_axis_scale_config = humanPowerGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryPowerField_auto_thresholds = g_vars[VarsBatteryPower].auto_thresholds;
  m_eeprom_data.batteryPowerField_config_error_threshold = g_vars[VarsBatteryPower].config_error_threshold;
  m_eeprom_data.batteryPowerField_config_warn_threshold = g_vars[VarsBatteryPower].config_warn_threshold;
  m_eeprom_data.batteryPowerField_x_axis_scale_config = batteryPowerGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryPowerUsageField_auto_thresholds = g_vars[VarsBatteryPowerUsage].auto_thresholds;
  m_eeprom_data.batteryPowerUsageField_config_error_threshold = g_vars[VarsBatteryPowerUsage].config_error_threshold;
  m_eeprom_data.batteryPowerUsageField_config_warn_threshold = g_vars[VarsBatteryPowerUsage].config_warn_threshold;
  m_eeprom_data.batteryPowerUsageField_x_axis_scale_config = batteryPowerUsageGraph.rw->graph.x_axis_scale_config;

  //m_eeprom_data.batteryVoltageField_auto_thresholds = g_vars[VarsBatteryVoltage].auto_thresholds;
  //m_eeprom_data.batteryVoltageField_config_error_threshold = g_vars[VarsBatteryVoltage].config_error_threshold;
  //m_eeprom_data.batteryVoltageField_config_warn_threshold = g_vars[VarsBatteryVoltage].config_warn_threshold;
  //m_eeprom_data.batteryVoltageField_x_axis_scale_config = batteryVoltageGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryCurrentField_auto_thresholds = g_vars[VarsBatteryCurrent].auto_thresholds;
  m_eeprom_data.batteryCurrentField_config_error_threshold = g_vars[VarsBatteryCurrent].config_error_threshold;
  m_eeprom_data.batteryCurrentField_config_warn_threshold = g_vars[VarsBatteryCurrent].config_warn_threshold;
  m_eeprom_data.batteryCurrentField_x_axis_scale_config = batteryCurrentGraph.rw->graph.x_axis_scale_config;

  //m_eeprom_data.motorCurrentField_auto_thresholds = g_vars[VarsMotorCurrent].auto_thresholds;
  //m_eeprom_data.motorCurrentField_config_error_threshold = g_vars[VarsMotorCurrent].config_error_threshold;
  //m_eeprom_data.motorCurrentField_config_warn_threshold = g_vars[VarsMotorCurrent].config_warn_threshold;
  //m_eeprom_data.motorCurrentField_x_axis_scale_config = motorCurrentGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorTempField_auto_thresholds = g_vars[VarsMotorTemp].auto_thresholds;
  m_eeprom_data.motorTempField_config_error_threshold = g_vars[VarsMotorTemp].config_error_threshold;
  m_eeprom_data.motorTempField_config_warn_threshold = g_vars[VarsMotorTemp].config_warn_threshold;
  m_eeprom_data.motorTempField_x_axis_scale_config = motorTempGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorErpsField_auto_thresholds = g_vars[VarsMotorERPS].auto_thresholds;
  m_eeprom_data.motorErpsField_config_error_threshold = g_vars[VarsMotorERPS].config_error_threshold;
  m_eeprom_data.motorErpsField_config_warn_threshold = g_vars[VarsMotorERPS].config_warn_threshold;
  m_eeprom_data.motorErpsField_x_axis_scale_config = motorErpsGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.pwmDutyField_auto_thresholds = g_vars[VarsMotorPWM].auto_thresholds;
  m_eeprom_data.pwmDutyField_config_error_threshold = g_vars[VarsMotorPWM].config_error_threshold;
  m_eeprom_data.pwmDutyField_config_warn_threshold = g_vars[VarsMotorPWM].config_warn_threshold;
  m_eeprom_data.pwmDutyField_x_axis_scale_config = pwmDutyGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorFOCField_auto_thresholds = g_vars[VarsMotorFOC].auto_thresholds;
  m_eeprom_data.motorFOCField_config_error_threshold = g_vars[VarsMotorFOC].config_error_threshold;
  m_eeprom_data.motorFOCField_config_warn_threshold = g_vars[VarsMotorFOC].config_warn_threshold;
  m_eeprom_data.motorFOCField_x_axis_scale_config = motorFOCGraph.rw->graph.x_axis_scale_config;
#endif

  m_eeprom_data.showNextScreenIndex = g_showNextScreenPreviousIndex;

  m_eeprom_data.ui8_street_mode_feature_enabled =
      ui_vars->ui8_street_mode_feature_enabled;
  m_eeprom_data.ui8_street_mode_enabled =
      ui_vars->ui8_street_mode_enabled;
  m_eeprom_data.ui8_street_mode_enabled_on_startup =
      ui_vars->ui8_street_mode_enabled_on_startup;
  m_eeprom_data.ui8_street_mode_speed_limit =
      ui_vars->ui8_street_mode_speed_limit;
  m_eeprom_data.ui8_street_mode_power_limit_div25 =
      ui_vars->ui8_street_mode_power_limit_div25;
  m_eeprom_data.ui8_street_mode_throttle_enabled =
      ui_vars->ui8_street_mode_throttle_enabled;
  m_eeprom_data.ui32_trip_distance_x1000 =
      ui_vars->ui32_trip_distance_x1000;
  m_eeprom_data.ui32_trip_time =
      ui_vars->ui32_trip_time;	  
  m_eeprom_data.ui16_trip_max_speed_x10 =
      ui_vars->ui16_trip_max_speed_x10;
  m_eeprom_data.ui16_trip_avg_speed_x10 =
      ui_vars->ui16_trip_avg_speed_x10;
  m_eeprom_data.ui16_battery_current_avg =
      ui_vars->ui16_battery_current_avg;
  m_eeprom_data.ui16_battery_power_avg =
      ui_vars->ui16_battery_power_avg;
  m_eeprom_data.ui16_pedal_power_avg =
      ui_vars->ui16_pedal_power_avg; 
  m_eeprom_data.ui16_pedal_cadence_avg =
      ui_vars->ui16_pedal_cadence_avg;	
  m_eeprom_data.ui16_battery_energy_h_km_avg_x100 =
      ui_vars->ui16_battery_energy_h_km_avg_x100;
  m_eeprom_data.ui8_plus_long_press_switch =
      ui_vars->ui8_plus_long_press_switch;

	flash_write_words(&m_eeprom_data, sizeof(m_eeprom_data) / sizeof(uint32_t));
}

void eeprom_init_defaults(void)
{
#ifdef SW102
  memset(&m_eeprom_data, 0, sizeof(m_eeprom_data));
  memcpy(&m_eeprom_data,
      &m_eeprom_data_defaults,
      sizeof(m_eeprom_data_defaults));

  eeprom_init_variables();
  set_conversions();
  flash_write_words(&m_eeprom_data, sizeof(m_eeprom_data) / sizeof(uint32_t));
#else
  // first force KEY value to 0
  eeprom_write(ADDRESS_KEY, 0);

  // eeprom_init() will read the default values now
  eeprom_init();

#endif
}
