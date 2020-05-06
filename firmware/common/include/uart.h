#pragma once

#include <stdint.h>

void uart_init(void);
const uint8_t* uart_get_rx_buffer(void);
uint8_t* uart_get_tx_buffer(void);
void uart_send_tx_buffer(uint8_t *tx_buffer, uint8_t ui8_len);


#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   23  // change this value depending on how many data bytes there is to receive ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_NUMBER_DATA_BYTES_TO_SEND      8   // change this value depending on how many data bytes there is to send ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_MAX_NUMBER_MESSAGE_ID          6   // change this value depending on how many different packages there is to send

