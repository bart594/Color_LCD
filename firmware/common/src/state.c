/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <math.h>
#include "stdio.h"
#include "main.h"
#include "utils.h"
#include "screen.h"
#include "rtc.h"
#include "fonts.h"
#include "uart.h"
#include "mainscreen.h"
#include "eeprom.h"
#include "buttons.h"
#include "fault.h"
#include "state.h"
#include "adc.h"
#include "timer.h"
#include <stdlib.h>
#include "ble_services.h"

static uint8_t ui8_m_usart1_received_first_package = 0;
uint8_t ui8_g_battery_soc;
volatile uint8_t ui8_g_motorVariablesStabilized = 0;


volatile motor_init_state_t g_motor_init_state = MOTOR_INIT_NOT_READY;
static uint8_t ui8_packet_type = UART_PACKET_REGULAR;


rt_vars_t rt_vars;
ui_vars_t ui_vars;

volatile bool m_reset_wh_flag = false;

ui_vars_t* get_ui_vars(void) {
	return &ui_vars;
}

rt_vars_t* get_rt_vars(void) {
  return &rt_vars;
}

/// Set correct backlight brightness for current headlight state
void set_lcd_backlight() {
	lcd_set_backlight_intensity(
			ui_vars.ui8_lights ?
					ui_vars.ui8_lcd_backlight_on_brightness :
					ui_vars.ui8_lcd_backlight_off_brightness);
}

void rt_send_tx_package(void) {

    static uint8_t ui8_i;
    uint16_t ui16_crc_tx;
    static uint8_t ui8_message_id = 0;

	uint8_t *ui8_usart1_tx_buffer = uart_get_tx_buffer();

	/************************************************************************************************/
	// send tx package
	// start up byte
	ui8_usart1_tx_buffer[0] = 0x59;

    ui8_usart1_tx_buffer[1] = ui8_packet_type;    

	ui8_usart1_tx_buffer[2] = ui8_message_id;
 
    // riding mode
    ui8_usart1_tx_buffer[3] = rt_vars.ui8_riding_mode;

	// riding mode parameter
      switch (rt_vars.ui8_riding_mode)
      {
        case POWER_ASSIST_MODE:
        
          if (rt_vars.ui8_assist_level > 0)
          {
            //send power/torque asssist value
			if (ui8_message_id % 2 == 0 ){
			ui8_usart1_tx_buffer[4] = rt_vars.ui8_assist_level_power_assist[((rt_vars.ui8_assist_level) - 1)];
			}else{ui8_usart1_tx_buffer[4] = rt_vars.ui8_assist_level_torque_assist[((rt_vars.ui8_assist_level) - 1)];}
          }
          else
          {
            ui8_usart1_tx_buffer[4] = 0;
          }
          
        break;

        case eMTB_ASSIST_MODE:
        
          ui8_usart1_tx_buffer[4] = rt_vars.ui8_eMTB_assist_level;

        break;
        
        case WALK_ASSIST_MODE:
        
          if (rt_vars.ui8_assist_level > rt_vars.ui8_number_of_assist_levels)
          {
            ui8_usart1_tx_buffer[4] = 0;
          }
		  else if ( rt_vars.ui8_assist_level > 0)
          {
            ui8_usart1_tx_buffer[4] = rt_vars.ui8_walk_assist_level_factor[((rt_vars.ui8_assist_level) - 1)];
          }
          else 
          {
            ui8_usart1_tx_buffer[4] = 0;
          }
        break;
        
        case CRUISE_MODE:

          ui8_usart1_tx_buffer[4] = rt_vars.ui8_cruise_function_target_speed_kph;
              
        break;
		
		default:
        
          ui8_usart1_tx_buffer[4] = 0;
          
        break;
      }
  	  

      ui8_usart1_tx_buffer[5] = rt_vars.ui8_lights | (rt_vars.ui8_torque_sensor_calibration_feature_enabled << 1) | (rt_vars.ui8_field_weakening_enabled << 2);

	if(ui8_packet_type == UART_PACKET_REGULAR){	  
 
	switch(ui8_message_id)
  {
    case 0:
      // battery low voltage cut-off
      ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_battery_low_voltage_cut_off_x10 & 0xff);
      ui8_usart1_tx_buffer[7] = (uint8_t) (rt_vars.ui16_battery_low_voltage_cut_off_x10 >> 8);
	  
	  // wheel max speed
          if (rt_vars.ui8_street_mode_feature_enabled)
          {
            ui8_usart1_tx_buffer[8] = rt_vars.ui8_street_mode_speed_limit;
          }
          else
          {
            ui8_usart1_tx_buffer[8] = rt_vars.ui8_wheel_max_speed;
          }
    
	break;

    case 1:
      // wheel perimeter
     ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_wheel_perimeter & 0xff);
     ui8_usart1_tx_buffer[7] = (uint8_t) (rt_vars.ui16_wheel_perimeter >> 8);

	          // enable/disable motor temperature limit function or throttle
          if (rt_vars.ui8_street_mode_feature_enabled && rt_vars.ui8_street_mode_throttle_enabled && rt_vars.ui8_optional_ADC_function == THROTTLE_CONTROL)
          {
            ui8_usart1_tx_buffer[8] = 0;
          }
          else
          {
            ui8_usart1_tx_buffer[8] = rt_vars.ui8_optional_ADC_function;
          }
	
	break;

    case 2:
      // set motor type
      ui8_usart1_tx_buffer[6] = rt_vars.ui8_motor_type;
      // motor over temperature min value limit
      ui8_usart1_tx_buffer[7] = rt_vars.ui8_motor_temperature_min_value_to_limit;
      // motor over temperature max value limit
      ui8_usart1_tx_buffer[8] = rt_vars.ui8_motor_temperature_max_value_to_limit;
          
    break;

	case 3:
	  ui8_usart1_tx_buffer[6] = rt_vars.ui8_soft_start_feature_enabled;

	  ui8_usart1_tx_buffer[7] = rt_vars.ui8_cadence_RPM_limit;

      ui8_usart1_tx_buffer[8] = rt_vars.ui8_field_weakening_current;

    break;

    case 4:
	  
	  ui8_usart1_tx_buffer[6] = rt_vars.ui8_lights_configuration;
	  // start without pedal rotation
	  ui8_usart1_tx_buffer[7] = rt_vars.ui8_motor_assistance_startup_without_pedal_rotation;
      // motor acceleration
	  if (rt_vars.ui8_assist_level > rt_vars.ui8_number_of_assist_levels){
      ui8_usart1_tx_buffer[8] = rt_vars.ui8_motor_acceleration;
	  }
	  else{ui8_usart1_tx_buffer[8] = rt_vars.ui8_motor_acceleration_level[((rt_vars.ui8_assist_level) - 1)];}

    break;

    case 5:
	  // pedal torque conversion
	  ui8_usart1_tx_buffer[6] = rt_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100;
	  // max battery current in amps    
      ui8_usart1_tx_buffer[7] = rt_vars.ui8_battery_max_current;
          
      // battery power limit
      if (rt_vars.ui8_street_mode_feature_enabled)
          {
	  if (rt_vars.ui8_assist_level > rt_vars.ui8_number_of_assist_levels){		  
      ui8_usart1_tx_buffer[8] = rt_vars.ui8_street_mode_power_limit_div25;}		  
      else if (rt_vars.ui8_target_peak_battery_power_div25[((rt_vars.ui8_assist_level) - 1)] > rt_vars.ui8_street_mode_power_limit_div25){
	  ui8_usart1_tx_buffer[8] = rt_vars.ui8_street_mode_power_limit_div25;}
	  else {ui8_usart1_tx_buffer[8] = rt_vars.ui8_target_peak_battery_power_div25[((rt_vars.ui8_assist_level) - 1)];}
          }
      else
          {
	  if (rt_vars.ui8_assist_level > rt_vars.ui8_number_of_assist_levels){		  
      ui8_usart1_tx_buffer[8] = rt_vars.ui8_target_max_battery_power_div25;
	  }
	  else{ui8_usart1_tx_buffer[8] = rt_vars.ui8_target_peak_battery_power_div25[((rt_vars.ui8_assist_level) - 1)];}	  
	  }

    break;
	 
    case 6:
      // cadence sensor mode
      //ui8_usart1_tx_buffer[6] = rt_vars.ui8_cadence_sensor_mode;
	  // cadence sensor pulse high percentage
	  //if (rt_vars.ui8_cadence_sensor_mode == ADVANCED_MODE){
      //uint16_t ui16_temp = rt_vars.ui16_cadence_sensor_pulse_high_percentage_x10;
      //ui8_usart1_tx_buffer[7] = (uint8_t) (ui16_temp & 0xff);
      //ui8_usart1_tx_buffer[8] = (uint8_t) (ui16_temp >> 8);
	//}
	break;
	
    default:
      ui8_message_id = 0;
    break;
  }
}

	if(ui8_packet_type == UART_PACKET_CONFIG){
	
	switch(ui8_message_id)
  {
    case 0:
      ui8_usart1_tx_buffer[6] = (uint8_t)  rt_vars.ui16_torque_sensor_calibration_table[0][0];     
      ui8_usart1_tx_buffer[7] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[0][1] & 0xff);
      ui8_usart1_tx_buffer[8] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[0][1] >> 8);
    
	break;

    case 1:
      ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[1][0] & 0xff);
      ui8_usart1_tx_buffer[7] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[1][0] >> 8);
	  ui8_usart1_tx_buffer[8] = (uint8_t)  rt_vars.ui16_torque_sensor_calibration_table[1][1]; 
	 
	break;

    case 2:
      ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[2][0] & 0xff);
      ui8_usart1_tx_buffer[8] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[2][0] >> 8);
	  ui8_usart1_tx_buffer[8] = (uint8_t)  rt_vars.ui16_torque_sensor_calibration_table[2][1]; 
          
    break;

	case 3:
	  ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[3][0] & 0xff);
      ui8_usart1_tx_buffer[7] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[3][0] >> 8);
      ui8_usart1_tx_buffer[8] = (uint8_t)  rt_vars.ui16_torque_sensor_calibration_table[3][1];     
	 
    break;

    case 4:
	  ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[4][0] & 0xff);
      ui8_usart1_tx_buffer[8] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[4][0] >> 8);
      ui8_usart1_tx_buffer[8] = (uint8_t)  rt_vars.ui16_torque_sensor_calibration_table[4][1];     

    break;

    case 5:
	  ui8_usart1_tx_buffer[6] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[5][0] & 0xff);
      ui8_usart1_tx_buffer[8] = (uint8_t) (rt_vars.ui16_torque_sensor_calibration_table[5][0] >> 8);
      ui8_usart1_tx_buffer[8] = (uint8_t)  rt_vars.ui16_torque_sensor_calibration_table[5][1];    

    break;
	//parameters for soft start 
    case 6:
	  ui8_usart1_tx_buffer[6] = rt_vars.ui8_assist_level_power_assist[0];
      ui8_usart1_tx_buffer[7] = rt_vars.ui8_assist_level_torque_assist[0];
      ui8_usart1_tx_buffer[8] = 0;

	break;
	
    default:
      ui8_message_id = 0;
    break;
  }
}
	// prepare crc of the package
  ui16_crc_tx = 0xffff;
  for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_SEND; ui8_i++)
  {
    crc16 (ui8_usart1_tx_buffer[ui8_i], &ui16_crc_tx);
  }
  ui8_usart1_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 1] = (uint8_t) (ui16_crc_tx & 0xff);
  ui8_usart1_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 2] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;
  
	// send the full package to UART
  //we are sending 11 bytes but needs to be extra 1  don't know why???
  uart_send_tx_buffer(ui8_usart1_tx_buffer, UART_NUMBER_DATA_BYTES_TO_SEND + 4);
		
  // increment message_id for next package
  if(++ui8_message_id > UART_MAX_NUMBER_MESSAGE_ID)
  {
    ui8_message_id = 0;
  }

}  

void rt_low_pass_filter_battery_voltage_current_power(void) {
	static uint32_t ui32_battery_voltage_accumulated_x1000 = 0;
	static uint16_t ui16_battery_current_accumulated_x5 = 0;

	// low pass filter battery voltage
	ui32_battery_voltage_accumulated_x1000 -= ui32_battery_voltage_accumulated_x1000 >> BATTERY_VOLTAGE_FILTER_COEFFICIENT;
	ui32_battery_voltage_accumulated_x1000 += (uint32_t) rt_vars.ui16_adc_battery_voltage;
	rt_vars.ui16_battery_voltage_filtered_x10 =	(uint32_t) (ui32_battery_voltage_accumulated_x1000 >> BATTERY_VOLTAGE_FILTER_COEFFICIENT) / 100;

	// low pass filter battery current
	ui16_battery_current_accumulated_x5 -= ui16_battery_current_accumulated_x5 >> BATTERY_CURRENT_FILTER_COEFFICIENT;
	ui16_battery_current_accumulated_x5 += (uint16_t) rt_vars.ui8_battery_current_x5;
	rt_vars.ui16_battery_current_filtered_x5 = ui16_battery_current_accumulated_x5 >> BATTERY_CURRENT_FILTER_COEFFICIENT;

	// full battery power, considering the power loss also inside the battery and cables, because we are using the battery resistance
  //
  uint16_t ui16_battery_power_filtered_x50 = rt_vars.ui16_battery_current_filtered_x5 * rt_vars.ui16_battery_voltage_filtered_x10;
  rt_vars.ui16_battery_power_filtered = ui16_battery_power_filtered_x50 / 50;

  // P = R * I^2
  uint32_t ui32_temp = (uint32_t) rt_vars.ui16_battery_current_filtered_x5;
  ui32_temp = ui32_temp * ui32_temp; // I * I
  ui32_temp /= 25;

  ui32_temp *= (uint32_t) rt_vars.ui16_battery_pack_resistance_x1000; // R * I * I
  ui32_temp /= 20; // now is _x50
  rt_vars.ui16_battery_power_loss = (uint16_t) (ui32_temp / 50);

  rt_vars.ui16_full_battery_power_filtered_x50 = ui16_battery_power_filtered_x50 + (uint16_t) ui32_temp;
}

void rt_low_pass_filter_pedal_power(void) {
	
	static uint32_t ui32_pedal_power_accumulated = 0;
    
	rt_vars.ui16_pedal_power_x10 = ((uint32_t) rt_vars.ui16_pedal_torque_x100 * rt_vars.ui8_pedal_cadence) / 96;
	
	// low pass filter
	ui32_pedal_power_accumulated -= ui32_pedal_power_accumulated >> PEDAL_POWER_FILTER_COEFFICIENT;
	ui32_pedal_power_accumulated += (uint32_t) rt_vars.ui16_pedal_power_x10	/ 10;
	rt_vars.ui16_pedal_power_filtered =	((uint32_t) (ui32_pedal_power_accumulated >> PEDAL_POWER_FILTER_COEFFICIENT));
}

void rt_calc_battery_voltage_soc(void) {
    
	uint16_t ui16_fluctuate_battery_voltage_x10;

	// calculate flutuate voltage, that depends on the current and battery pack resistance
	ui16_fluctuate_battery_voltage_x10 =
			(uint16_t) ((((uint32_t) rt_vars.ui16_battery_pack_resistance_x1000)
					* ((uint32_t) rt_vars.ui16_battery_current_filtered_x5))
					/ ((uint32_t) 500));
	// now add fluctuate voltage value
	rt_vars.ui16_battery_voltage_soc_x10 = rt_vars.ui16_battery_voltage_filtered_x10 + ui16_fluctuate_battery_voltage_x10;
	
}

void rt_calc_wh(void) {
	static uint8_t ui8_1s_timer_counter = 0;
	uint32_t ui32_temp = 0;

	if (m_reset_wh_flag == false) {
    if (rt_vars.ui16_full_battery_power_filtered_x50 > 0) {
      rt_vars.ui32_wh_sum_x5 += rt_vars.ui16_full_battery_power_filtered_x50 / 10;
      rt_vars.ui32_wh_sum_counter++;
    }

    // calc at 1s rate
    if (++ui8_1s_timer_counter >= 10) {
      ui8_1s_timer_counter = 0;

      // avoid zero divisison
      if (rt_vars.ui32_wh_sum_counter != 0) {
        ui32_temp = rt_vars.ui32_wh_sum_counter / 36;
        ui32_temp = (ui32_temp
            * (rt_vars.ui32_wh_sum_x5 / rt_vars.ui32_wh_sum_counter))
            / 500;
      }

      rt_vars.ui32_wh_x10 = rt_vars.ui32_wh_x10_offset + ui32_temp;
    }
	}
}

void reset_wh(void) {
  m_reset_wh_flag = true;
  rt_vars.ui32_wh_sum_x5 = 0;
  rt_vars.ui32_wh_sum_counter = 0;
  m_reset_wh_flag = false;
}

static void rt_calc_odometer(void) {
    static uint8_t ui8_1s_timer_counter;
	uint32_t uint32_temp;
	uint8_t ui8_01km_flag = 0;

	// calc at 1s rate
	if (++ui8_1s_timer_counter >= 10) {
		ui8_1s_timer_counter = 0;

		uint32_temp = (rt_vars.ui32_wheel_speed_sensor_tick_counter
				- ui_vars.ui32_wheel_speed_sensor_tick_counter_offset)
				* ((uint32_t) rt_vars.ui16_wheel_perimeter);
		// avoid division by 0
		if (uint32_temp > 100000) {
			uint32_temp /= 100000;
		}  // milimmeters to 0.1kms
		else {
			uint32_temp = 0;
		}

		// now store the value on the global variable
		// rt_vars.ui16_odometer_distance_x10 = (uint16_t) uint32_temp;

		// calculate how many revolutions since last reset and convert to distance traveled
		uint32_t ui32_temp = (rt_vars.ui32_wheel_speed_sensor_tick_counter
				- rt_vars.ui32_wheel_speed_sensor_tick_counter_offset)
				* ((uint32_t) rt_vars.ui16_wheel_perimeter);

		// if traveled distance is more than 100 meters update all distance variables and reset
		if (ui32_temp >= 100000) { // 100000 -> 100000 mm -> 0.1 km
			// update all distance variables
			// ui_vars.ui16_distance_since_power_on_x10 += 1;
			rt_vars.ui32_odometer_x10 += 1;
			rt_vars.ui32_trip_x10 += 1;
			ui8_01km_flag = 1;

			// reset the always incrementing value (up to motor controller power reset) by setting the offset to current value
			rt_vars.ui32_wheel_speed_sensor_tick_counter_offset =
					rt_vars.ui32_wheel_speed_sensor_tick_counter;
		}
	}

  // calc battery energy per km
#define BATTERY_ENERGY_H_KM_FACTOR_X2 1800 // (60 * 60) / 2, each step at fixed interval of 100ms and apply 1 / 2 for have value from _x50 to _x100

	// keep accumulating the energy
  rt_vars.battery_energy_h_km.ui32_sum_x50 += rt_vars.ui16_full_battery_power_filtered_x50;

  static uint16_t ui16_one_km_timeout_counter = 0;

  // reset value if riding at very low speed or being stopped for 2 minutes
  if (++ui16_one_km_timeout_counter >= 600) { // 600 equals min of average 2km/h for 2 minutes, at least
    ui16_one_km_timeout_counter = 600; // keep on this state...
    rt_vars.battery_energy_h_km.ui32_value_x100 = 0;
    rt_vars.battery_energy_h_km.ui32_value_x10 = 0;
    rt_vars.battery_energy_h_km.ui32_sum_x50 = 0;
  }

	if (ui8_01km_flag) {
    ui16_one_km_timeout_counter = 0;
    rt_vars.battery_energy_h_km.ui32_value_x100 = rt_vars.battery_energy_h_km.ui32_sum_x50 / BATTERY_ENERGY_H_KM_FACTOR_X2;
    rt_vars.battery_energy_h_km.ui32_value_x10 = rt_vars.battery_energy_h_km.ui32_value_x100 / 10;
    rt_vars.battery_energy_h_km.ui32_sum_x50 = 0;
  }
}

static void rt_low_pass_filter_pedal_cadence(void) {
	static uint16_t ui16_pedal_cadence_accumulated = 0;

	// low pass filter
	ui16_pedal_cadence_accumulated -= (ui16_pedal_cadence_accumulated
			>> PEDAL_CADENCE_FILTER_COEFFICIENT);
	ui16_pedal_cadence_accumulated += (uint16_t) rt_vars.ui8_pedal_cadence;

	// consider the filtered value only for medium and high values of the unfiltered value
	if (rt_vars.ui8_pedal_cadence > 20) {
		rt_vars.ui8_pedal_cadence_filtered =
				(uint8_t) (ui16_pedal_cadence_accumulated
						>> PEDAL_CADENCE_FILTER_COEFFICIENT);
	} else {
		rt_vars.ui8_pedal_cadence_filtered = rt_vars.ui8_pedal_cadence;
	}
}

void rt_first_time_management(void) {
  static uint32_t ui32_counter = 0;

  // wait 5 seconds to help motor variables data stabilize
  if (ui8_g_motorVariablesStabilized == 0)
    if (++ui32_counter > 40) {
      ui8_g_motorVariablesStabilized = 1;
#ifndef SW102
      extern Field *activeGraphs; // FIXME, move this extern someplace better, placing here for review purposes
  	  activeGraphs = &(*graphs[g_showNextScreenIndex]); // allow graph plotting to start
#endif
    }

   // motor ok so we are sending first values for torque linearizaton table
	if ((ui8_m_usart1_received_first_package < 10) && (g_motor_init_state == MOTOR_INIT_NOT_READY)){
		ui8_packet_type = UART_PACKET_CONFIG;
	    g_motor_init_state = MOTOR_INIT_SEND_CONFIG;
		//prepare_torque_sensor_calibration_table();
	}
	// this will be executed only 1 time at startup
    else if(ui8_g_motorVariablesStabilized) {
    // reset Wh value if battery voltage is over ui16_battery_voltage_reset_wh_counter_x10 (value configured by user)
    if (((uint32_t) ui_vars.ui16_adc_battery_voltage)
        > ((uint32_t) ui_vars.ui16_battery_voltage_reset_wh_counter_x10
            * 100)) {
      ui_vars.ui32_wh_x10_offset = 0;
    }
    // all values hopefully sent
	if (g_motor_init_state == MOTOR_INIT_SEND_CONFIG){
	ui8_packet_type = UART_PACKET_REGULAR;
	g_motor_init_state = MOTOR_INIT_READY;
	}
	//else if (g_motor_init_state == MOTOR_INIT_NOT_READY){
	//g_motor_init_state = MOTOR_INIT_ERROR;
	//}
  }
}

void rt_calc_battery_soc(void) {
	uint32_t ui32_temp;

	ui32_temp = rt_vars.ui32_wh_x10 * 100;

	if (rt_vars.ui32_wh_x10_100_percent > 0) {
		ui32_temp /= rt_vars.ui32_wh_x10_100_percent;
	} else {
		ui32_temp = 0;
	}

	if (ui32_temp > 100)
		ui32_temp = 100;

  ui8_g_battery_soc = (uint8_t) (100 - ui32_temp);
}

void rt_processing_stop(void) {
#ifndef SW102
  Display850C_rt_processing_stop();
#else
  SW102_rt_processing_stop();
#endif
}

void rt_processing_start(void) {
#ifndef SW102
  Display850C_rt_processing_start();
#else
  SW102_rt_processing_start();
#endif
}

/**
 * Called from the main thread every 100ms
 *
 */
void copy_rt_to_ui_vars(void) {
	ui_vars.ui16_adc_battery_voltage = rt_vars.ui16_adc_battery_voltage;
	ui_vars.ui8_battery_current_x5 = rt_vars.ui8_battery_current_x5;
	ui_vars.ui16_battery_power_loss = rt_vars.ui16_battery_power_loss;
	//ui_vars.ui8_motor_current_x5 = rt_vars.ui8_motor_current_x5;
	ui_vars.ui8_throttle = rt_vars.ui8_throttle;
    ui_vars.ui8_adc_throttle = rt_vars.ui8_adc_throttle;
	ui_vars.ui16_adc_pedal_torque_sensor = rt_vars.ui16_adc_pedal_torque_sensor;
	ui_vars.ui16_pedal_weight = rt_vars.ui16_pedal_weight;
	ui_vars.ui8_duty_cycle = rt_vars.ui8_duty_cycle;
	ui_vars.ui8_error_states = rt_vars.ui8_error_states;
	ui_vars.ui16_wheel_speed_x10 = rt_vars.ui16_wheel_speed_x10;
	ui_vars.ui8_pedal_cadence = rt_vars.ui8_pedal_cadence;
	ui_vars.ui8_pedal_cadence_filtered = rt_vars.ui8_pedal_cadence_filtered;
	ui_vars.ui16_motor_speed_erps = rt_vars.ui16_motor_speed_erps;
	ui_vars.ui8_motor_temperature = rt_vars.ui8_motor_temperature;
	ui_vars.ui32_wheel_speed_sensor_tick_counter =
			rt_vars.ui32_wheel_speed_sensor_tick_counter;
	ui_vars.ui16_battery_voltage_filtered_x10 =
			rt_vars.ui16_battery_voltage_filtered_x10;
	ui_vars.ui16_battery_current_filtered_x5 =
			rt_vars.ui16_battery_current_filtered_x5;
  //ui_vars.ui16_motor_current_filtered_x5 =
    //  rt_vars.ui16_motor_current_filtered_x5;
	ui_vars.ui16_full_battery_power_filtered_x50 =
			rt_vars.ui16_full_battery_power_filtered_x50;
	ui_vars.ui16_battery_power = rt_vars.ui16_battery_power_filtered;
	ui_vars.ui16_pedal_power = rt_vars.ui16_pedal_power_filtered;
	ui_vars.ui16_battery_voltage_soc_x10 = rt_vars.ui16_battery_voltage_soc_x10;
	ui_vars.ui32_wh_sum_x5 = rt_vars.ui32_wh_sum_x5;
	ui_vars.ui32_wh_sum_counter = rt_vars.ui32_wh_sum_counter;
	ui_vars.ui32_wh_x10 = rt_vars.ui32_wh_x10;
    ui_vars.battery_energy_km_value_x100 = rt_vars.battery_energy_h_km.ui32_value_x100;
	ui_vars.ui8_braking = rt_vars.ui8_braking;
	ui_vars.ui8_foc_angle = (((uint16_t) rt_vars.ui8_foc_angle) * 14) / 10; // each units is equal to 1.4 degrees ((360 degrees / 256) = 1.4)
	ui_vars.ui32_trip_x10 = rt_vars.ui32_trip_x10;
	ui_vars.ui32_odometer_x10 = rt_vars.ui32_odometer_x10;
	ui_vars.ui16_pedal_torque_x100 = rt_vars.ui16_pedal_torque_x100;	
    ui_vars.ui16_pedal_power_x10 = rt_vars.ui16_pedal_power_x10;
    
	//if (rt_vars.ui8_cadence_sensor_mode == CALIBRATION_MODE)
	//ui_vars.ui16_cadence_sensor_pulse_high_percentage_x10 = rt_vars.ui16_cadence_sensor_pulse_high_percentage_x10;	

	//ui_vars.ui32_nav_turn_distance = rt_vars.ui32_nav_turn_distance;
	//ui_vars.ui32_nav_total_distance = rt_vars.ui32_nav_total_distance;
	//ui_vars.ui32_nav_total_turn_distance = rt_vars.ui32_nav_total_distance;
	//ui_vars.ui8_nav_info = rt_vars.ui8_nav_info;
	//ui_vars.ui8_nav_info_extra = rt_vars.ui8_nav_info_extra;
    
	rt_vars.ui8_walk_assist_feature_enabled = ui_vars.ui8_walk_assist_feature_enabled;
	rt_vars.ui32_wh_x10_100_percent = ui_vars.ui32_wh_x10_100_percent;
	rt_vars.ui32_wh_x10_offset = ui_vars.ui32_wh_x10_offset;
	rt_vars.ui16_battery_pack_resistance_x1000 = ui_vars.ui16_battery_pack_resistance_x1000;
	rt_vars.ui8_assist_level = ui_vars.ui8_assist_level;
	rt_vars.ui8_number_of_assist_levels = ui_vars.ui8_number_of_assist_levels;
	rt_vars.ui8_eMTB_assist_level = ui_vars.ui8_eMTB_assist_level;
    rt_vars.ui8_soft_start_feature_enabled = ui_vars.ui8_soft_start_feature_enabled;
	rt_vars.ui8_cadence_RPM_limit = ui_vars.ui8_cadence_RPM_limit;
	rt_vars.ui8_lights_configuration = ui_vars.ui8_lights_configuration;
	rt_vars.ui8_motor_acceleration = ui_vars.ui8_motor_acceleration;
	
	for (uint8_t i = 0; i < 5; i++) {
    rt_vars.ui8_target_peak_battery_power_div25[i] = ui_vars.ui8_target_peak_battery_power_div25[i];
    }

	for (uint8_t i = 0; i < 5; i++) {
    rt_vars.ui8_motor_acceleration_level[i] = ui_vars.ui8_motor_acceleration_level[i];
    }
	
	for (uint8_t i = 0; i < 5; i++) {
    rt_vars.ui8_walk_assist_level_factor[i] = ui_vars.ui8_walk_assist_level_factor[i];
    }
	
	for (uint8_t i = 0; i < 5; i++) {
	  rt_vars.ui8_assist_level_power_assist[i] = ui_vars.ui8_assist_level_power_assist[i];
	}	

	for (uint8_t i = 0; i < 5; i++) {
	  rt_vars.ui8_assist_level_torque_assist[i] = ui_vars.ui8_assist_level_torque_assist[i];
	}		
	
	
	rt_vars.ui16_battery_voltage_reset_wh_counter_x10 = ui_vars.ui16_battery_voltage_reset_wh_counter_x10;
	rt_vars.ui8_lights = ui_vars.ui8_lights;
	rt_vars.ui8_walk_assist = ui_vars.ui8_walk_assist;
	rt_vars.ui8_battery_max_current = ui_vars.ui8_battery_max_current;
	rt_vars.ui8_target_max_battery_power_div25 = ui_vars.ui8_target_max_battery_power_div25;
	rt_vars.ui16_battery_low_voltage_cut_off_x10 =
			ui_vars.ui16_battery_low_voltage_cut_off_x10;
	rt_vars.ui16_wheel_perimeter = ui_vars.ui16_wheel_perimeter;
	rt_vars.ui8_wheel_max_speed = ui_vars.wheel_max_speed_x10 / 10;
	rt_vars.ui8_motor_type = ui_vars.ui8_motor_type;
	rt_vars.ui8_motor_assistance_startup_without_pedal_rotation =
			ui_vars.ui8_motor_assistance_startup_without_pedal_rotation;
	rt_vars.ui8_motor_temperature_min_value_to_limit =
			ui_vars.ui8_motor_temperature_min_value_to_limit;
	rt_vars.ui8_motor_temperature_max_value_to_limit =
			ui_vars.ui8_motor_temperature_max_value_to_limit;
	rt_vars.ui8_street_mode_feature_enabled = ui_vars.ui8_street_mode_feature_enabled;
	rt_vars.ui8_street_mode_enabled_on_startup =
			ui_vars.ui8_street_mode_enabled_on_startup;
	rt_vars.ui8_street_mode_speed_limit = ui_vars.ui8_street_mode_speed_limit;
    rt_vars.ui8_street_mode_power_limit_div25 = ui_vars.ui8_street_mode_power_limit_div25;
    rt_vars.ui8_street_mode_throttle_enabled = ui_vars.ui8_street_mode_throttle_enabled;
    rt_vars.ui8_riding_mode = ui_vars.ui8_riding_mode;
	//rt_vars.ui8_cadence_sensor_mode = ui_vars.ui8_cadence_sensor_mode;
	//rt_vars.ui16_cadence_sensor_pulse_high_percentage_x10 = ui_vars.ui16_cadence_sensor_pulse_high_percentage_x10;
	rt_vars.ui8_optional_ADC_function = ui_vars.ui8_optional_ADC_function;
	rt_vars.ui8_target_battery_max_power_div25 = ui_vars.ui8_target_battery_max_power_div25;
    rt_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100;
	rt_vars.ui8_cruise_function_target_speed_kph = 	ui_vars.ui8_cruise_function_target_speed_kph;
	rt_vars.ui8_torque_sensor_calibration_feature_enabled = ui_vars.ui8_torque_sensor_calibration_feature_enabled;
	rt_vars.ui8_field_weakening_enabled = ui_vars.ui8_field_weakening_enabled;
	rt_vars.ui8_field_weakening_current = ui_vars.ui8_field_weakening_current;
	rt_vars.ui8_battery_soc_enable = ui_vars.ui8_battery_soc_enable;
	
}

/// must be called from main() idle loop
void automatic_power_off_management(void) {
	static uint32_t ui16_lcd_power_off_time_counter = 0;

	if (ui_vars.ui8_lcd_power_off_time_minutes != 0) {
		// see if we should reset the automatic power off minutes counter
		if ((ui_vars.ui16_wheel_speed_x10 > 0) ||   // wheel speed > 0
				(ui_vars.ui8_battery_current_x5 > 0) || // battery current > 0
				(ui_vars.ui8_braking) ||                // braking
				buttons_get_events()) {                 // any button active
			ui16_lcd_power_off_time_counter = 0;
		} else {
			// increment the automatic power off ticks counter
			ui16_lcd_power_off_time_counter++;

			// check if we should power off the LCD
			if (ui16_lcd_power_off_time_counter
					>= (ui_vars.ui8_lcd_power_off_time_minutes * 10 * 60)) { // have we passed our timeout?
//				lcd_power_off(1);
			}
		}
	} else {
		ui16_lcd_power_off_time_counter = 0;
	}
}

void uart_data_clock(void) {

  static uint16_t num_missed_packets = 0;
  const uint8_t *p_rx_buffer = uart_get_rx_buffer();

  // process rx package 
      if (p_rx_buffer) {

          num_missed_packets = 0; // reset missed packet count
		  
		  //battery voltage
		  rt_vars.ui16_adc_battery_voltage = (((uint16_t) p_rx_buffer[2]) << 8) + ((uint16_t) p_rx_buffer[1]);	
          // battery current x10
		  uint8_t ui8_battery_current_temp;
		  ui8_battery_current_temp = p_rx_buffer[3];
		  rt_vars.ui8_battery_current_x5 = ui8_battery_current_temp  >> 1;
          
		  rt_vars.ui16_wheel_speed_x10 = (((uint16_t) p_rx_buffer[5]) << 8) + ((uint16_t) p_rx_buffer[4]);

          // for some reason, the previous value of rt_vars.ui16_wheel_speed_x10 is 16384, because p_rx_buffer[7] is 64,
          // this even when rx_buffer[6] and rx_buffer[7] are both 0 on the motor controller
          //rt_vars.ui16_wheel_speed_x10 = rt_vars.ui16_wheel_speed_x10; // 0x7ff = 204.7km/h

          rt_vars.ui8_braking  = p_rx_buffer[6];

		  rt_vars.ui8_adc_throttle = p_rx_buffer[7];

         
      switch (rt_vars.ui8_optional_ADC_function)
      {
        case THROTTLE_CONTROL:
        
          // throttle value with offset applied and mapped from 0 to 255
		  rt_vars.ui8_throttle = p_rx_buffer[8];
        
        break;
        
        case TEMPERATURE_CONTROL:
        
          // temperature
          rt_vars.ui8_motor_temperature = p_rx_buffer[8];
        
        break;
      }

          rt_vars.ui16_adc_pedal_torque_sensor = (((uint16_t) p_rx_buffer[10]) << 8) + ((uint16_t) p_rx_buffer[9]);
          
		  rt_vars.ui8_pedal_cadence = p_rx_buffer[11];
          
		  rt_vars.ui8_duty_cycle = p_rx_buffer[12];
          
		  rt_vars.ui16_motor_speed_erps = (((uint16_t) p_rx_buffer[14]) << 8) + ((uint16_t) p_rx_buffer[13]);
          
		  rt_vars.ui8_foc_angle = p_rx_buffer[15];
          
		  rt_vars.ui8_error_states = p_rx_buffer[16];
          
		  rt_vars.ui32_wheel_speed_sensor_tick_counter = (((uint32_t) p_rx_buffer[19]) << 16) + (((uint32_t) p_rx_buffer[18]) << 8) + ((uint32_t) p_rx_buffer[17]);
		  
		  rt_vars.ui16_pedal_torque_x100 = (((uint16_t) p_rx_buffer[21]) << 8) + ((uint16_t) p_rx_buffer[20]);
		  
		  uint16_t ui16_pedal_torque_temp = rt_vars.ui16_pedal_torque_x100;
		  
		  if (rt_vars.ui8_torque_sensor_calibration_feature_enabled){
		  rt_vars.ui16_pedal_weight = ui16_pedal_torque_temp / 17;
          }else{rt_vars.ui16_pedal_weight = ui16_pedal_torque_temp * 10 / 167;}
		  
		  //cadence sensor pulse high percentage calibration
		 // if (rt_vars.ui8_cadence_sensor_mode == CALIBRATION_MODE)
		 // rt_vars.ui16_cadence_sensor_pulse_high_percentage_x10 = (((uint16_t) p_rx_buffer[23]) << 8) + ((uint16_t) p_rx_buffer[22]);

    

    // let's wait for 10 packages, seems that first ADC battery voltages have incorrect values
    ui8_m_usart1_received_first_package++;
    if (ui8_m_usart1_received_first_package > 10){
      ui8_m_usart1_received_first_package = 10;
    }
   
    if (g_motor_init_state == MOTOR_INIT_READY || MOTOR_INIT_SEND_CONFIG) 
    rt_send_tx_package();
  
   //now  we do all calculations that must be done after receiving data  
    rt_processing();
#ifdef SW102
    ble_uart_send(&rt_vars);
#endif	
	}

	//prepare_torque_sensor_calibration_table();    
	// We expected a packet during this 120ms window but one did not arrive.  This might happen if the motor is still booting and we don't want to declare failure
    // unless something is seriously busted (because we will be raising the fault screen and eventually forcing the bike to shutdown) so be very conservative
    // and wait for 10 seconds of missed packets.
    if ((g_motor_init_state == MOTOR_INIT_READY) && (num_missed_packets++ == 100))
         APP_ERROR_HANDLER(FAULT_LOSTRX);
}


void rt_processing(void)
{

  /************************************************************************************************/
  // now do all the calculations that must be done every 120ms
  rt_low_pass_filter_battery_voltage_current_power();
  rt_low_pass_filter_pedal_power();
  rt_low_pass_filter_pedal_cadence();
  rt_calc_battery_voltage_soc();
  rt_calc_odometer();
  rt_calc_wh();
  rt_graph_process();
  /************************************************************************************************/
  rt_first_time_management();
  rt_calc_battery_soc();
}

void prepare_torque_sensor_calibration_table(void) {
  static bool first_time = true;

  // we need to make this atomic
  rt_processing_stop();

  // at the very first time, copy the ADC values from one table to the other
  if (first_time) {
    first_time = false;

    for (uint8_t i = 0; i < 6; i++) {
      rt_vars.ui16_torque_sensor_calibration_table[i][0] = ui_vars.ui16_torque_sensor_calibration_table[i][1];
	  //we need raw values for android app
	  rt_vars.ui16_torque_sensor_calibration_ble_table[i][0] = ui_vars.ui16_torque_sensor_calibration_table[i][0]; //kg vlues	  
	  rt_vars.ui16_torque_sensor_calibration_ble_table[i][1] = ui_vars.ui16_torque_sensor_calibration_table[i][1]; //adc values
    }
  }
  // get the delta values of ADC steps per kg
  for (uint8_t i = 1; i < 6; i++) {
    // get the deltas x100
    rt_vars.ui16_torque_sensor_calibration_table[i][1] =
        ((ui_vars.ui16_torque_sensor_calibration_table[i][0] - ui_vars.ui16_torque_sensor_calibration_table[i - 1][0]) * 100) /
        (ui_vars.ui16_torque_sensor_calibration_table[i][1] - ui_vars.ui16_torque_sensor_calibration_table[i - 1][1]);

  }
  // very first table value need to the calculated here
  rt_vars.ui16_torque_sensor_calibration_table[0][1] = rt_vars.ui16_torque_sensor_calibration_table[1][1]; // the first delta is equal the the second one
  rt_vars.ui16_torque_sensor_calibration_table[0][0] = 0; // the first must be 0 

  rt_processing_start();
}

#define MIN_VOLTAGE_10X 140 // If our measured bat voltage (using ADC in the display) is lower than this, we assume we are running on a developers desk
