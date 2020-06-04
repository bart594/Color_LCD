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
#include "lcd.h"
#include "adc.h"
#include "ugui.h"
#include "configscreen.h"
#include "battery_gui.h"
#include "state.h"

Field batteryField = FIELD_CUSTOM(renderBattery);
uint8_t old_power_strip_segment = 0;
  
static void mainScreenOnEnter() {
	// Set the font preference for this screen
	editable_label_font = &SMALL_TEXT_FONT;
	editable_value_font = &SMALL_TEXT_FONT;
	editable_units_font = &SMALL_TEXT_FONT;
}

/**
 * Appears at the bottom of all screens, includes status msgs or critical fault alerts
 * FIXME - get rid of this nasty define - instead add the concept of Subscreens, so that the battery bar
 * at the top and the status bar at the bottom can be shared across all screens
 */
#define STATUS_BAR \
{ \
  .x = 0, .y = 118, \
  .width = 64, .height = -1, \
  .field = &warnField, \
  .border = BorderNone, \
  .font = &REGULAR_TEXT_FONT, \
}

#define BATTERY_BAR \
{ \
    .x = 3, .y = 2, \
    .width = -1, .height = -1, \
    .field = &batteryField, \
}, \
{ \
  .x = 38, .y = 2, \
  .width = 24, .height = -1, \
  .align_x = AlignRight, \
  .font = &REGULAR_TEXT_FONT, \
  .field = &socField \
}

//
// Screens
//

#define MAIN_SCREEN_WHEELSPEED_X                  8
#define MAIN_SCREEN_WHEELSPEED_Y                  17
#define MAIN_SCREEN_WHEELSPEED_WIDTH              50
#define MAIN_SCREEN_WHEELSPEED_HEIGHT             40

#define MAIN_SCREEN_MAXPOWER_X                    2
#define MAIN_SCREEN_MAXPOWER_Y                    21
#define MAIN_SCREEN_MAXPOWER_WIDTH                61
#define MAIN_SCREEN_MAXPOWER_HEIGHT               34

#define MAIN_SCREEN_CUSTOM_1_X                    1
#define MAIN_SCREEN_CUSTOM_1_Y                    66
#define MAIN_SCREEN_CUSTOM_1_WIDTH                62
#define MAIN_SCREEN_CUSTOM_1_HEIGHT               22

#define MAIN_SCREEN_CUSTOM_2_X                    1
#define MAIN_SCREEN_CUSTOM_2_Y                    91
#define MAIN_SCREEN_CUSTOM_2_WIDTH                62
#define MAIN_SCREEN_CUSTOM_2_HEIGHT               22


Screen mainScreen1 = {
    .onPress = mainScreenOnPress,
	.onEnter = mainScreenOnEnter,
	.onDirtyClean = mainScreenOnDirtyClean,
	.onPostUpdate = mainScreenonPostUpdate,

  .fields = {
    BATTERY_BAR,
    { 
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &wheelSpeedIntegerField,
	  .font = &BIG_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_MAXPOWER_X,
      .y = MAIN_SCREEN_MAXPOWER_Y,
      .width = MAIN_SCREEN_MAXPOWER_WIDTH,
      .height = MAIN_SCREEN_MAXPOWER_HEIGHT,
      .field = &motorMaxPowerField,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_y = AlignTop,
      .align_x = AlignCenter,
      .inset_y = 6,
      .border = BorderNone,
      .show_units = Hide
    },
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &assistLevelField,
      .font = &BIG_SPECIAL_CHARS_ASSIST_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &navTurnField,
      .font = &BIG_SPECIAL_CHARS_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_CUSTOM_1_X,
      .y = MAIN_SCREEN_CUSTOM_1_Y,
      .width = MAIN_SCREEN_CUSTOM_1_WIDTH,
      .height = MAIN_SCREEN_CUSTOM_1_HEIGHT,
      .field = &custom1,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
	  .unit_align_y = AlignCenter,
      .align_x = AlignCenter,
      .border = BorderNone,
      .inset_y = 3,
      .show_units = Hide
    },
    {
      .x = MAIN_SCREEN_CUSTOM_2_X,
      .y = MAIN_SCREEN_CUSTOM_2_Y,
      .width = MAIN_SCREEN_CUSTOM_2_WIDTH,
      .height = MAIN_SCREEN_CUSTOM_2_HEIGHT,
      .field = &custom2,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .border = BorderNone,
      .inset_y = 3,
      .show_units = Hide
    },
    STATUS_BAR,
    {
      .field = NULL
    }
  }
};

Screen mainScreen2 = {
  .onPress = mainScreenOnPress,
  .onEnter = mainScreenOnEnter,
  .onDirtyClean = mainScreenOnDirtyClean,
  .onPostUpdate = mainScreenonPostUpdate,

  .fields = {
    BATTERY_BAR,
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &wheelSpeedIntegerField,
      .font = &BIG_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_MAXPOWER_X,
      .y = MAIN_SCREEN_MAXPOWER_Y,
      .width = MAIN_SCREEN_MAXPOWER_WIDTH,
      .height = MAIN_SCREEN_MAXPOWER_HEIGHT,
      .field = &motorMaxPowerField,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_y = AlignTop,
      .align_x = AlignCenter,
      .inset_y = 6,
      .border = BorderNone,
      .show_units = Hide
    },
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &assistLevelField,
      .font = &BIG_SPECIAL_CHARS_ASSIST_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &navTurnField,
      .font = &BIG_SPECIAL_CHARS_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },		
    {
      .x = MAIN_SCREEN_CUSTOM_1_X,
      .y = MAIN_SCREEN_CUSTOM_1_Y,
      .width = MAIN_SCREEN_CUSTOM_1_WIDTH,
      .height = MAIN_SCREEN_CUSTOM_1_HEIGHT,
      .field = &custom3,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .border = BorderNone,
      .inset_y = 3,
      .show_units = Hide
    },
    {
      .x = MAIN_SCREEN_CUSTOM_2_X,
      .y = MAIN_SCREEN_CUSTOM_2_Y,
      .width = MAIN_SCREEN_CUSTOM_2_WIDTH,
      .height = MAIN_SCREEN_CUSTOM_2_HEIGHT,
      .field = &custom4,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .border = BorderNone,
      .inset_y = 3,
      .show_units = Hide
    },
    STATUS_BAR,
    {
      .field = NULL
    }
  }
};

Screen mainScreen3 = {
  .onPress = mainScreenOnPress,
  .onEnter = mainScreenOnEnter,
  .onDirtyClean = mainScreenOnDirtyClean,
  .onPostUpdate = mainScreenonPostUpdate,

  .fields = {
    BATTERY_BAR,
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &wheelSpeedIntegerField,
      .font = &BIG_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_MAXPOWER_X,
      .y = MAIN_SCREEN_MAXPOWER_Y,
      .width = MAIN_SCREEN_MAXPOWER_WIDTH,
      .height = MAIN_SCREEN_MAXPOWER_HEIGHT,
      .field = &motorMaxPowerField,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_y = AlignTop,
      .align_x = AlignCenter,
      .inset_y = 6,
      .border = BorderNone,
      .show_units = Hide
    },
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &assistLevelField,
      .font = &BIG_SPECIAL_CHARS_ASSIST_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },
    {
      .x = MAIN_SCREEN_WHEELSPEED_X,
      .y = MAIN_SCREEN_WHEELSPEED_Y,
      .width = MAIN_SCREEN_WHEELSPEED_WIDTH,
      .height = MAIN_SCREEN_WHEELSPEED_HEIGHT,
      .field = &navTurnField,
      .font = &BIG_SPECIAL_CHARS_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .show_units = Hide,
      .border = BorderNone,
    },	
    {
      .x = MAIN_SCREEN_CUSTOM_1_X,
      .y = MAIN_SCREEN_CUSTOM_1_Y,
      .width = MAIN_SCREEN_CUSTOM_1_WIDTH,
      .height = MAIN_SCREEN_CUSTOM_1_HEIGHT,
      .field = &custom5,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .border = BorderNone,
      .inset_y = 3,
      .show_units = Hide
    },
    {
      .x = MAIN_SCREEN_CUSTOM_2_X,
      .y = MAIN_SCREEN_CUSTOM_2_Y,
      .width = MAIN_SCREEN_CUSTOM_2_WIDTH,
      .height = MAIN_SCREEN_CUSTOM_2_HEIGHT,
      .field = &custom6,
      .font = &MEDIUM_NUMBERS_TEXT_FONT,
      .label_align_x = AlignHidden,
      .align_x = AlignCenter,
      .border = BorderNone,
      .inset_y = 3,
      .show_units = Hide
    },
    STATUS_BAR,
    {
      .field = NULL
    }
  }
};

// Show our battery graphic
void battery_display() {
  static uint8_t old_soc = 0xff;

  if (ui8_g_battery_soc != old_soc) {
    old_soc = ui8_g_battery_soc;
    batteryField.rw->dirty = true;
  }
}

void mainScreenOnDirtyClean(void) {
  batteryClearSymbol();
  batteryField.rw->dirty = true;
  

  // find if the next lines should be draw or not (white color to draw them)
  UG_COLOR street_mode_color = C_BLACK;
  if (ui_vars.ui8_street_mode_feature_enabled &&
      ui_vars.ui8_street_mode_enabled) {
    street_mode_color = C_WHITE;
  }

  UG_DrawFrame(0, 0, 63, 59, street_mode_color);
  
  UG_DrawFrame(0, 59, 63, 115, C_WHITE);
  UG_DrawLine(0, 65, 63, 65, C_WHITE);
  UG_DrawLine(0, 90, 63, 90, C_WHITE);
  
}

void secondMainScreenOnDirtyClean(void) {
  batteryClearSymbol();
}

void PowerStripOnDirtyClean(uint8_t power_strip_segment_draw_number) {
  
  int shift_x = 2;
  //uint8_t old_power_strip_segment;
  
  if (power_strip_segment_draw_number > 0){
  if(old_power_strip_segment > power_strip_segment_draw_number){
  UG_FillFrame(power_strip_segment_draw_number + 3, 61, old_power_strip_segment + shift_x, 63, C_BLACK);
  }else{UG_FillFrame(2, 61, power_strip_segment_draw_number + shift_x, 63, C_WHITE);}
  }
  else{UG_FillFrame(2, 61, 61, 63, C_BLACK);};

  old_power_strip_segment = power_strip_segment_draw_number;
  
  
}

void mainScreenonPostUpdate(void) {
}

// Screens in a loop, shown when the user short presses the power button
Screen *screens[] = { &mainScreen1, &mainScreen2, &mainScreen3,
		NULL };


