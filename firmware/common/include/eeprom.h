/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "lcd.h"
#include "state.h"
#include "screen.h"

// For compatible changes, just add new fields at the end of the table (they will be inited to 0xff for old eeprom images).  For incompatible
// changes bump up EEPROM_MIN_COMPAT_VERSION and the user's EEPROM settings will be discarded.
#define EEPROM_MIN_COMPAT_VERSION 0x37
#define EEPROM_VERSION 0x37

typedef struct {
  graph_auto_max_min_t auto_max_min;
  int32_t max;
  int32_t min;
} Graph_eeprom;

typedef struct eeprom_data {
	uint8_t eeprom_version; // Used to detect changes in eeprom encoding, if != EEPROM_VERSION we will not use it

	uint8_t ui8_assist_level;
	uint16_t ui16_wheel_perimeter;
	uint8_t ui8_wheel_max_speed;
	uint8_t ui8_units_type;
	uint32_t ui32_wh_x10_offset;
	uint32_t ui32_wh_x10_100_percent;
	uint8_t ui8_battery_soc_enable;
	uint8_t ui8_target_max_battery_power_div25;
	uint8_t ui8_battery_max_current;
	uint8_t ui8_motor_max_current;
    uint8_t ui8_motor_current_min_adc;
	uint8_t ui8_ramp_up_amps_per_second_x10;
	uint16_t ui16_battery_low_voltage_cut_off_x10;
	uint8_t ui8_motor_type;
	uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
	uint16_t ui16_assist_level_factor[ASSIST_LEVEL_NUMBER];
	uint8_t ui8_number_of_assist_levels;
	uint8_t ui8_startup_motor_power_boost_feature_enabled;
	uint8_t ui8_startup_motor_power_boost_always;
	uint8_t ui8_startup_motor_power_boost_limit_power;
	uint16_t ui16_startup_motor_power_boost_factor[ASSIST_LEVEL_NUMBER];
	uint8_t ui8_startup_motor_power_boost_time;
	uint8_t ui8_startup_motor_power_boost_fade_time;
	uint8_t ui8_motor_temperature_min_value_to_limit;
	uint8_t ui8_motor_temperature_max_value_to_limit;
	uint16_t ui16_battery_voltage_reset_wh_counter_x10;
	uint8_t ui8_lcd_power_off_time_minutes;
	uint8_t ui8_lcd_backlight_on_brightness;
	uint8_t ui8_lcd_backlight_off_brightness;
	uint16_t ui16_battery_pack_resistance_x1000;
	uint32_t ui32_odometer_x10;
	uint8_t  ui8_walk_assist_feature_enabled;
	uint8_t  ui8_walk_assist_level_factor[ASSIST_LEVEL_NUMBER];
	uint8_t	 ui8_assist_level_power_assist[ASSIST_LEVEL_NUMBER];
	uint8_t  ui8_target_peak_battery_power_div25[ASSIST_LEVEL_NUMBER];
	uint8_t  ui8_motor_acceleration_level[ASSIST_LEVEL_NUMBER];	
	uint8_t ui8_riding_mode_ui;
	uint8_t ui8_eMTB_assist_level;
	uint8_t ui8_optional_ADC_function;
	uint8_t ui8_target_battery_max_power_div25;
	uint8_t ui8_motor_acceleration;
	uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
	uint8_t ui8_cruise_function_target_speed_kph;

	uint8_t ui8_battery_soc_increment_decrement;
	uint8_t ui8_buttons_up_down_invert;
    uint8_t ui8_torque_sensor_calibration_feature_enabled;
    uint8_t ui8_torque_sensor_calibration_pedal_ground;
	uint16_t ui16_torque_sensor_calibration_table[6][2];

	uint8_t field_selectors[NUM_CUSTOMIZABLE_FIELDS]; // this array is opaque to the app, but the screen layer uses it to store which field is being displayed (it is stored to EEPROM)
	uint8_t graphs_field_selectors[3]; // 3 screen main pages

	uint8_t x_axis_scale; // x axis scale
	uint8_t showNextScreenIndex;

  uint8_t ui8_street_mode_feature_enabled;
  uint8_t ui8_street_mode_enabled;
  uint8_t ui8_street_mode_enabled_on_startup;
  uint8_t ui8_street_mode_speed_limit;
  uint8_t ui8_street_mode_power_limit_div25;
  uint8_t ui8_street_mode_throttle_enabled;
  uint8_t ui8_field_weakening_enabled;
  uint8_t ui8_field_weakening_current;
  uint8_t ui8_cadence_RPM_limit;
  uint8_t ui8_torque_boost_factor;

#ifndef SW102
	Graph_eeprom graph_eeprom[VARS_SIZE];
  uint8_t tripDistanceField_x_axis_scale_config;
	field_threshold_t wheelSpeedField_auto_thresholds;
	int32_t wheelSpeedField_config_error_threshold;
	int32_t wheelSpeedField_config_warn_threshold;
	uint8_t wheelSpeedField_x_axis_scale_config;
	field_threshold_t cadenceField_auto_thresholds;
  int32_t cadenceField_config_error_threshold;
  int32_t cadenceField_config_warn_threshold;
  uint8_t cadenceField_x_axis_scale_config;
  field_threshold_t humanPowerField_auto_thresholds;
  int32_t humanPowerField_config_error_threshold;
  int32_t humanPowerField_config_warn_threshold;
  uint8_t humanPowerField_x_axis_scale_config;
  field_threshold_t batteryPowerField_auto_thresholds;
  int32_t batteryPowerField_config_error_threshold;
  int32_t batteryPowerField_config_warn_threshold;
  uint8_t batteryPowerField_x_axis_scale_config;
  field_threshold_t batteryPowerUsageField_auto_thresholds;
  int32_t batteryPowerUsageField_config_error_threshold;
  int32_t batteryPowerUsageField_config_warn_threshold;
  uint8_t batteryPowerUsageField_x_axis_scale_config;
  field_threshold_t batteryVoltageField_auto_thresholds;
  int32_t batteryVoltageField_config_error_threshold;
  int32_t batteryVoltageField_config_warn_threshold;
  uint8_t batteryVoltageField_x_axis_scale_config;
  field_threshold_t batteryCurrentField_auto_thresholds;
  int32_t batteryCurrentField_config_error_threshold;
  int32_t batteryCurrentField_config_warn_threshold;
  uint8_t batteryCurrentField_x_axis_scale_config;
  field_threshold_t motorCurrentField_auto_thresholds;
  int32_t motorCurrentField_config_error_threshold;
  int32_t motorCurrentField_config_warn_threshold;
  uint8_t motorCurrentField_x_axis_scale_config;
  field_threshold_t batterySOCField_auto_thresholds;
  int32_t batterySOCField_config_error_threshold;
  int32_t batterySOCField_config_warn_threshold;
  uint8_t batterySOCField_x_axis_scale_config;
  field_threshold_t motorTempField_auto_thresholds;
  int32_t motorTempField_config_error_threshold;
  int32_t motorTempField_config_warn_threshold;
  uint8_t motorTempField_x_axis_scale_config;
  field_threshold_t motorErpsField_auto_thresholds;
  int32_t motorErpsField_config_error_threshold;
  int32_t motorErpsField_config_warn_threshold;
  uint8_t motorErpsField_x_axis_scale_config;
  field_threshold_t pwmDutyField_auto_thresholds;
  int32_t pwmDutyField_config_error_threshold;
  int32_t pwmDutyField_config_warn_threshold;
  uint8_t pwmDutyField_x_axis_scale_config;
  field_threshold_t motorFOCField_auto_thresholds;
  int32_t motorFOCField_config_error_threshold;
  int32_t motorFOCField_config_warn_threshold;
  uint8_t motorFOCField_x_axis_scale_config;
#endif

// FIXME align to 32 bit value by end of structure and pack other fields
} eeprom_data_t;

void eeprom_init(void);
void eeprom_init_variables(void);
void eeprom_write_variables(void);
void eeprom_init_defaults(void);

// *************************************************************************** //
// EEPROM memory variables default values
#define DEFAULT_VALUE_ASSIST_LEVEL                                  2
#define DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS                       5
#define DEFAULT_VALUE_WHEEL_PERIMETER                               2100 // 26'' wheel: 2100mm perimeter
#define DEFAULT_VALUE_WHEEL_MAX_SPEED                               45
#define DEFAULT_VALUE_UNITS_TYPE                                    0 // 0 = km/h
#define DEFAULT_VALUE_WH_X10_OFFSET                                 0
#define DEFAULT_VALUE_WH_X10_100_PERCENT                            5000 // default to a battery of 500 Wh
#define DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC                      1 // SOC
#define DEFAULT_VALUE_BATTERY_MAX_CURRENT                           16 // 16 amps
#define DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER_DIV25                30 // e.g. 20 = 20 * 25 = 500, 0 is disabled
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10               440 // 52v battery, LVC = 42.0 (3.0 * 14)
#define DEFAULT_VALUE_MOTOR_TYPE                                    0 // ui8_motor_type = 0 = 48V
#define DEFAULT_VALUE_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION       0 // 0 to keep this feature disable
#define DEFAULT_VALUE_ASSIST_LEVEL_POWER_1                          4 // 0.4
#define DEFAULT_VALUE_ASSIST_LEVEL_POWER_2                          8
#define DEFAULT_VALUE_ASSIST_LEVEL_POWER_3                          12
#define DEFAULT_VALUE_ASSIST_LEVEL_POWER_4                          18
#define DEFAULT_VALUE_ASSIST_LEVEL_POWER_5                          32
#define DEFAULT_VALUE_WALK_ASSIST_FEATURE_ENABLED                   1
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_1                    17
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_2                    23
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_3                    30
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_4                    43
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_5                    50
#define DEFAULT_VALUE_PEAK_POWER_LEVEL_1                    		30
#define DEFAULT_VALUE_PEAK_POWER_LEVEL_2                    		30
#define DEFAULT_VALUE_PEAK_POWER_LEVEL_3                    		30
#define DEFAULT_VALUE_PEAK_POWER_LEVEL_4                    		30
#define DEFAULT_VALUE_PEAK_POWER_LEVEL_5                    		30
#define DEFAULT_VALUE_ACCELERATION_LEVEL_1                    		0
#define DEFAULT_VALUE_ACCELERATION_LEVEL_2                    		0
#define DEFAULT_VALUE_ACCELERATION_LEVEL_3                    		0
#define DEFAULT_VALUE_ACCELERATION_LEVEL_4                    		0
#define DEFAULT_VALUE_ACCELERATION_LEVEL_5                    		0
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT             60 // 75 degrees celsius
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT             85 // 85 degrees celsius
#define DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10          580 // 52v battery, 58.4 volts at fully charged
#define DEFAULT_VALUE_LCD_POWER_OFF_TIME                            60 // 60 minutes, each unit 1 minute
#ifdef SW102
#define DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS                   100 // 8 = 40%
#define DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS                  20 // 20 = 100%
#else
#define DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS                   15 // 100 = 100%
#define DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS                  100
#endif
#define DEFAULT_VALUE_BATTERY_PACK_RESISTANCE                       300 // 52v battery, 14S3P measured 300 milli ohms
#define DEFAULT_VALUE_ODOMETER_X10                                  0
#define DEFAULT_VALUE_BUTTONS_UP_DOWN_INVERT                        0 // regular state
#define DEFAULT_VALUE_X_AXIS_SCALE                                  0 // 15m
#define DEFAULT_VALUE_X_AXIS_SCALE                                  0 // 15m
#define DEFAULT_CUSTOMIZABLE_CHOICES_SELECTOR                       0 // the very first one
#define DEFAULT_CUSTOMIZABLE_FIELD_INDEX                            0 // the very first one
#define DEFAULT_STREET_MODE_FEATURE_ENABLE                          0 // disabled
#define DEFAULT_STREET_MODE_ENABLE                                  0 // disabled
#define DEFAULT_STREET_MODE_ENABLE_AT_STARTUP                       0 // disabled
#define DEFAULT_STREET_MODE_SPEED_LIMIT                             25 // 25 km/h
#define DEFAULT_STREET_MODE_POWER_LIMIT_DIV25                       10 // 250W --> 250 / 25 = 10
#define DEFAULT_STREET_MODE_THROTTLE_ENABLE                         0 // disabled
// default values for riding mode
#define DEFAULT_VALUE_RIDING_MODE                 					1
// default values motor acceleration
#define DEFAULT_VALUE_MOTOR_ACCELERATION                            0
// default value pedal torque conversion
#define DEFAULT_VALUE_PEDAL_TORQUE_CALIBRATION         				67
// default values for emtb level
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL		    					2
// default values for cruise function
#define DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_KPH              0  // 0 kph
// default value optional ADC function
#define DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION 						0
#define DEFAULT_VALUE_FIELD_WEAKENING_ENABLED						0
#define DEFAULT_VALUE_FIELD_WEAKENING_CURRENT						20
#define DEFAULT_VALUE_RPM_LIMIT										30
#define DEFAULT_VALUE_BOOST_FACTOR									1

#define BICYCLE_1
//#define BICYCLE_2

#ifdef BICYCLE_1
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_FEATURE_ENABLE       0 // disabled
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_1             0
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_1                155
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_2             10
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_2                210
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_3             20
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_3                240
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_4             30
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_4                260
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_5             65
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_5                280
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_6             90
#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_6                300
//#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_7             40
//#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_7                273
//#define DEFAULT_TORQUE_SENSOR_CALIBRATION_WEIGHT_8             100
//#define DEFAULT_TORQUE_SENSOR_CALIBRATION_ADC_8                291
#endif

// *************************************************************************** //

// Torque sensor value found experimentaly
// measuring with a cheap digital hook scale, we found that each torque sensor unit is equal to 0.556 Nm
// using the scale, was found that each 1kg was measured as 3 torque sensor units
// Force (Nm) = Kg * 9.18 * 0.17 (arm cranks size)
#define TORQUE_SENSOR_FORCE_SCALE_X1000 556

// *************************************************************************** //
// BATTERY

// ADC Battery voltage
// 0.344 per ADC_8bits step: 17.9V --> ADC_8bits = 52; 40V --> ADC_8bits = 116; this signal atenuated by the opamp 358
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 44
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 (ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 >> 1)
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP 0.344

// ADC Battery current
// 1A per 5 steps of ADC_10bits
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512 102
// *************************************************************************** //

#endif /* _EEPROM_H_ */
