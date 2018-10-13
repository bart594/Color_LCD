/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"

#include "pins.h"

void lcd_backlight (uint32_t ui32_state);

void lcd_init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = LCD_BACKLIGHT__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LCD_BACKLIGHT__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_PIN_1__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LCD_PIN_1__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_PIN_2__PORT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LCD_PIN_2__PORT, &GPIO_InitStructure);

  // these pins must be at 1 logic level (don't really know why)
  GPIO_SetBits(LCD_PIN_1__PORT, LCD_PIN_1__PIN);
  GPIO_SetBits(LCD_PIN_2__PORT, LCD_PIN_2__PIN);

  // enable backlight
  lcd_backlight (1);
}

void lcd_backlight (uint32_t ui32_state)
{
  if (ui32_state)
  {
    GPIO_SetBits(LCD_BACKLIGHT__PORT, LCD_BACKLIGHT__PIN);
  }
  else
  {
    GPIO_ResetBits(LCD_BACKLIGHT__PORT, LCD_BACKLIGHT__PIN);
  }
}
