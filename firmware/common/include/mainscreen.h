#pragma once

#include "screen.h"

#define eMTB_MODE_SYMBOL		6
#define LIGHT_SYMBOL			7
#define WALK_MODE_SYMBOL		8
#define	CRUISE_MODE_SYMBOL		9

extern Screen mainScreen1, mainScreen2, mainScreen3, bootScreen;
extern Screen *screens[];
extern Field
  naviStrip_0,
  naviStrip_1,
  naviStrip_2,
  naviStrip_3,
  naviStrip_4,
  naviStrip_5, 
  naviStrip_6,
  naviStrip_7,
  naviStrip_8,
  naviStrip_9,
  naviStrip_10,
  socField,
  timeField,
  assistLevelField,
  wheelSpeedIntegerField,
  wheelSpeedDecimalField,
  tripTimeField,
  tripDistanceField,
  odoField,
  wheelSpeedField,
  cadenceField,
  humanPowerField,
  batteryPowerField,
  batteryPowerUsageField,
  motorMaxPowerField,
  batteryVoltageField,
  batteryCurrentField,
  //motorCurrentField,
  //batterySOCField,
  motorTempField,
  motorErpsField,
  pwmDutyField,
  motorFOCField,
  motorTempGraph,
  bootStatus2,
#ifdef SW102
  navTurnField,
  navTurnDistanceField,
  navDistanceField,
  custom1, custom2,
  custom3, custom4,
  custom5, custom6,
  warnField; // just close previous definition
#endif
#ifndef SW102 // we don't have any graphs yet on SW102, possibly move this into mainscreen_850.c
  graph1, graph2, graph3,
  *graphs[3],
  custom1, custom2, custom3, custom4,
  custom5, custom6, custom7, custom8,
  custom9, custom10, custom11, custom12,
  warnField,

  wheelSpeedGraph,
  tripDistanceGraph,
  odoGraph,
  cadenceGraph,
  humanPowerGraph,
  batteryPowerGraph,
  batteryPowerUsageGraph,
  batteryPowerUsageFieldGraph,
  batteryVoltageGraph,
  batteryCurrentGraph,
  motorCurrentGraph,
  batterySOCGraph,
  motorTempGraph,
  motorErpsGraph,
  pwmDutyGraph,
  motorFOCGraph;
  void mainScreenOnDirtyClean();
#endif

extern uint8_t g_showNextScreenIndex;
extern uint8_t g_showNextScreenPreviousIndex;
extern uint16_t ui16_g_target_max_motor_power;

extern Field batteryField; // These fields are custom for board type
void battery_display(); // 850C and sw102 provide alternative versions due to different implementations
void set_conversions();
bool anyscreen_onpress(buttons_events_t events);
void clock_time(void);
void onSetConfigurationClockHours(uint32_t v);
void onSetConfigurationClockMinutes(uint32_t v);
void onSetConfigurationDisplayLcdBacklightOnBrightness(uint32_t v);
void onSetConfigurationDisplayLcdBacklightOffBrightness(uint32_t v);
void onSetConfigurationBatteryTotalWh(uint32_t v);
void onSetConfigurationWheelOdometer(uint32_t v);
void onSetConfigurationBatterySOCUsedWh(uint32_t v);
void mainScreenOnDirtyClean(void);
void secondMainScreenOnDirtyClean(void);
void mainScreenonPostUpdate(void);
bool mainScreenOnPress(buttons_events_t events);
void showNextScreen();
void main_idle(); // call every 20ms
void setWarning(ColorOp color, const char *str);

/// set to true if this boot was caused because we had a watchdog failure, used to show user the problem in the fault line
extern bool wd_failure_detected;

extern uint8_t ui8_g_configuration_clock_hours;
extern uint8_t ui8_g_configuration_clock_minutes;

extern uint8_t ui8_g_motor_max_power_state;
