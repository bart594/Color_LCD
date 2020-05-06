#include "uart.h"
#include "usart1.h"

uint8_t ui8_usart1_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 3];

/**
 * @brief Init UART peripheral
 */
void uart_init(void)
{

}

/**
 * @brief Returns pointer to RX buffer ready for parsing or NULL
 */
const uint8_t* uart_get_rx_buffer(void)
{
	uint8_t *r = usart1_get_rx_buffer();

	return r;
}

/**
 * @brief Returns pointer to TX buffer
 */
uint8_t* uart_get_tx_buffer(void)
{
  return ui8_usart1_tx_buffer;
}

/**
 * @brief Send TX buffer over UART.
 */
void uart_send_tx_buffer(uint8_t *tx_buffer, uint8_t ui8_len)
{
  // start DMA UART transfer
  usart1_start_dma_transfer(ui8_len);
}
