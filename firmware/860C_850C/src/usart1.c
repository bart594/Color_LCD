/*
 * Bafang LCD 860C/850C firmware
 *
 * Copyright (C) Casainho, 2018, 2019, 2020
 *
 * Released under the GPL License, Version 3
 */

#include <string.h>
#include "usart1.h"
#include "stm32f10x.h"
#include "pins.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "lcd.h"
#include "utils.h"
#include "usart1.h"
#include "main.h"
#include "uart.h"



uint8_t ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3];

typedef struct uart_rx_buff_typedef uart_rx_buff_typedef;
struct uart_rx_buff_typedef
{
  uint8_t uart_rx_data[ UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3];
  uart_rx_buff_typedef* next_uart_rx_buff;
};

uart_rx_buff_typedef* uart_rx_buffer;
volatile uint8_t* uart_rx_data;
volatile uint8_t ui8_received_package_flag = 0;

void usart1_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  // enable GPIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART1->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) uart_get_tx_buffer();
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = UART_NUMBER_DATA_BYTES_TO_SEND + 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  // USART pins
  GPIO_InitStructure.GPIO_Pin = USART1_RX__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART1__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USART1_TX__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART1__PORT, &GPIO_InitStructure);

  USART_DeInit(USART1);
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // enable the USART Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  USART_ClearITPendingBit(USART1, USART_IT_TC);

  // enable the USART
  USART_Cmd(USART1, ENABLE);

  DMA_Cmd(DMA1_Channel4, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART1, ENABLE);

  // enable USART Receive and Transmit interrupts
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

// USART1 Tx and Rx interrupt handler.
void USART1_IRQHandler()
{
  uint8_t ui8_byte_received;
  static uint8_t ui8_state_machine = 0;
  static uint8_t ui8_rx_counter = 0;
  //static uint8_t ui8_tx_data_index = 0;
  //uint8_t* tx_data;
  //tx_data = (uint8_t*) ui8_g_usart1_tx_buffer;

    /* Init RX buffer */
  static uart_rx_buff_typedef rxb1, rxb2;
  rxb1.next_uart_rx_buff = &rxb2;
  rxb2.next_uart_rx_buff = &rxb1;
  uart_rx_buffer = &rxb1;
  

  // The interrupt may be from Tx, Rx, or both.
  if(USART_GetITStatus(USART1, USART_IT_ORE) == SET)
  {
    USART_ReceiveData(USART1); // get ride of this interrupt flag
    return;
  }
  else if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
  {
    /*if(ui8_tx_data_index <= (UART_NUMBER_DATA_BYTES_TO_SEND + 3))  // bytes to send
    {
      // clearing the TXE bit is always performed by a write to the data register
      USART_SendData(USART1, tx_data[ui8_tx_data_index]);
      ++ui8_tx_data_index;
      if(ui8_tx_data_index > (UART_NUMBER_DATA_BYTES_TO_SEND + 3))
      {
        // buffer empty
        // disable TIEN (TXE)
      ui8_tx_data_index = 0;  
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      }
    }*/
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    return;
  }
  else if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    // receive byte
    ui8_byte_received = (uint8_t) USART1->DR;

    switch(ui8_state_machine)
    {
      case 0:
      if(ui8_byte_received == 67) // see if we get start package byte
      {
        uart_rx_buffer->uart_rx_data[ui8_rx_counter] = ui8_byte_received;
        ui8_rx_counter++;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      uart_rx_buffer->uart_rx_data[ui8_rx_counter] = ui8_byte_received;
      ui8_rx_counter++;

      // see if is the last byte of the package
      if(ui8_rx_counter >= (UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3))
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;

	 /* Signal that we have a full package to be processed */
	  
      uart_rx_data = uart_rx_buffer->uart_rx_data;
      /* Switch buffer */
      uart_rx_buffer = uart_rx_buffer->next_uart_rx_buff;

      }
      break;

      default:
      break;
    }
  }
}

void usart1_start_dma_transfer(uint8_t ui8_len)
{
  DMA_Cmd(DMA1_Channel4, DISABLE);
  DMA_SetCurrDataCounter(DMA1_Channel4, ui8_len);
  DMA_Cmd(DMA1_Channel4, ENABLE);
}



const uint8_t* usart1_get_rx_buffer(void)
{
  uint8_t* rx_rdy;
  rx_rdy = (uint8_t*) uart_rx_data;
  uint8_t ui8_i;
 
	
  if (rx_rdy != NULL)
  {
    uint16_t crc_rx = 0xffff;
    for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_RECEIVE; ui8_i++)
      crc16(rx_rdy[ui8_i], &crc_rx);

    if (((((uint16_t) rx_rdy[UART_NUMBER_DATA_BYTES_TO_RECEIVE + 2]) << 8)
        + ((uint16_t) rx_rdy[UART_NUMBER_DATA_BYTES_TO_RECEIVE + 1])) != crc_rx)
      rx_rdy = NULL;  // Invalidate buffer if CRC not OK
  }

  return rx_rdy;
}

