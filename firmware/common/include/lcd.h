/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include "main.h"
#include "ugui.h"
#include "ugui_bafang_850c.h"

// riding modes
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define eMTB_ASSIST_MODE                          2
#define WALK_ASSIST_MODE                          3
#define CRUISE_MODE                               4

// optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2 

// walk assist
#define WALK_ASSIST_THRESHOLD_SPEED_X10           80  // 80 -> 8.0 kph, this is the maximum speed limit from which walk assist can be activated

// cruise
#define CRUISE_THRESHOLD_SPEED_X10                90  // 90 -> 9.0 kph, this is the minimum speed limit from which cruise can be activated


#define MAX_NUMBER_DIGITS 5 // max of 5 digits: 1234.5 or 12345

typedef enum
{
  LCD_SCREEN_MAIN = 1,
  LCD_SCREEN_CONFIGURATIONS = 2
} lcd_screen_states_t;

typedef enum
{
  MAIN_SCREEN_STATE_MAIN = 0,
  MAIN_SCREEN_STATE_POWER,
  MAIN_SCREEN_STATE_CHANGE_GRAPH
} lcd_main_screen_states_t;

typedef struct lcd_vars_struct
{
  uint32_t ui32_main_screen_draw_static_info;
  lcd_screen_states_t lcd_screen_state;
  uint8_t ui8_lcd_menu_counter_1000ms_state;
  uint8_t ui8_lcd_menu_counter_1000ms_trigger;
  lcd_main_screen_states_t main_screen_state;
} lcd_vars_t;

typedef struct _print_number
{
  const UG_FONT* font;
  UG_COLOR fore_color;
  UG_COLOR back_color;
  uint32_t ui32_x_position;
  uint32_t ui32_y_position;
  uint32_t ui32_x_final_position;
  uint32_t ui32_y_final_position;
  uint8_t ui8_previous_digits_array[MAX_NUMBER_DIGITS];
  uint8_t ui8_field_number_of_digits;
  uint8_t ui8_left_zero_paddig;
  uint8_t ui8_left_paddig;
  uint8_t ui8_refresh_all_digits;
  uint32_t ui32_number;
  uint8_t ui8_digit_number_start_previous;
  uint8_t ui8_clean_area_all_digits;
  uint8_t ui8_decimal_digits;
} print_number_t;

extern lcd_IC_t g_lcd_ic_type;

void lcd_init(void);
void lcd_clock(void);
volatile lcd_vars_t* get_lcd_vars(void);
void lcd_print_number(print_number_t* number);
void lcd_draw_main_menu_mask(void);
void graphs_measurements_update(void);
void lcd_set_backlight_intensity(uint8_t ui8_intensity);

#endif /* LCD_H_ */
