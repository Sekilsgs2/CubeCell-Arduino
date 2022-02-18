#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "lora_config.h"
#include "HardwareSerial.h"

HardwareSerial Serial(0);
HardwareSerial Serial1(1);
HardwareSerial Serial2(2);
HardwareSerial Serial3(3);

volatile uint8_t *rx_buff;
volatile uint16_t rx_head;
volatile uint16_t rx_tail;
uint8_t *tx_buff;
volatile uint16_t tx_head;
volatile uint16_t tx_tail;
volatile size_t tx_size;

HardwareSerial::HardwareSerial(int uart) : _uart(uart) 
{

}

bool HardwareSerial::begin(uint32_t baud, uint32_t config, int rxPin, int txPin, bool invert, unsigned long timeout_ms)
{
	if(0 > _uart || _uart > 3) {
		return false;
	}
	rx_head = 0;
	rx_tail = 0;
	rx_buff = _rx_buffer;
    tx_buff = _tx_buffer;
    tx_head = 0;
    tx_tail = 0;
    tx_size = 0;	
	_baud = baud;
	_config = config;

	if(_uart == 0 && (rxPin < 0 || txPin < 0)) {
		rxPin = UART0_RX;
		txPin = UART0_TX;
	}
	else if(_uart == 1 && (rxPin < 0 || txPin < 0)) {
		rxPin = UART1_RX;
		txPin = UART1_TX;
	}
	else if(_uart == 2 && (rxPin < 0 || txPin < 0)) {
		rxPin = UART2_RX;
		txPin = UART2_TX;
	}
	else if(_uart == 3 && (rxPin < 0 || txPin < 0)) {
		rxPin = UART3_RX;
		txPin = UART3_TX;
	}
	_rxPin = rxPin;
	_txPin = txPin;
	
	_uart = uartStart(baud,config,_rxPin,_txPin);
	uart_cmd(UART0, DISABLE);
	uart_set_rx_fifo_threshold(UART0,UART_IFLS_RX_7_8);
	uart_set_tx_fifo_threshold(UART0,UART_IFLS_TX_7_8);
	uart_cmd(UART0, ENABLE);
	uart_attach_rx_callback(_rx_complete_irq);
	uart_config_interrupt(UART0,UART_INTERRUPT_TX_DONE | UART_INTERRUPT_RX_DONE | UART_INTERRUPT_OVERRUN_ERROR,ENABLE);
	//uart_config_interrupt(UART0,UART_INTERRUPT_RX_DONE,ENABLE);
	//NVIC_SetPriority(UART0_IRQn,128);
	if(_uart<0)
		return false;
	else
		return true;
}

void HardwareSerial::updateBaudRate(unsigned long baud)
{
	_baud = baud;
	uartStart(baud,_config,_rxPin,_txPin);
}

void HardwareSerial::end()
{
	uartEnd(_uart,_rxPin,_txPin);
}

int HardwareSerial::available(void)
{
	return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + rx_head - rx_tail)) % SERIAL_RX_BUFFER_SIZE;
	//return uartAvailable(_uart);
}
//int HardwareSerial::availableForWrite(void)
//{
	//return uartAvailableForWrite(_uart);
//}

int HardwareSerial::availableForWrite(void)
{
  tx_buffer_index_t head = tx_head;
  tx_buffer_index_t tail = tx_tail;

  if (head >= tail) {
    return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  }
  return tail - head - 1;
}

bool HardwareSerial::busy(void)
{
	return uartBusy(_uart);
}

int HardwareSerial::peek(void)
{
  if (rx_head == rx_tail) {
    return -1;
  } else {
    return rx_buff[rx_tail];
  }
}

int HardwareSerial::read(void)
{
  if (rx_head == rx_tail) {
    return -1;
  } else {
    unsigned char c = rx_buff[rx_tail];
    rx_tail = (rx_buffer_index_t)(rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
	//if(available()) {
		//return uartRead(_uart);
	//}
	//return -1;
}

void HardwareSerial::flush(void)
{
  if (!_written) {
    return;
  }

  while ((tx_head != tx_tail)) {
    // nop, the interrupt handler will free up space for us
  }
}

void HardwareSerial::flush(bool txOnly)
{
  if (!_written) {
    return;
  }

  while ((tx_head != tx_tail)) {
    // nop, the interrupt handler will free up space for us
  }
}

size_t HardwareSerial::write(uint8_t c)
{
  uint8_t buff = c;
  return write(&buff, 1);
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
  size_t size_intermediate;
  size_t ret = size;
  size_t available = availableForWrite();
  size_t available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - tx_head;

  _written = true;

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to free space
  while (!availableForWrite()) {
    // nop, the interrupt handler will free up space for us
  }

  // HAL doesn't manage rollover, so split transfer till end of TX buffer
  // Also, split transfer according to available space in buffer
  while ((size > available_till_buffer_end) || (size > available)) {
    size_intermediate = min(available, available_till_buffer_end);
    write(buffer, size_intermediate);
    size -= size_intermediate;
    buffer += size_intermediate;
    available = availableForWrite();
    available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - tx_head;
  }

  // Copy data to buffer. Take into account rollover if necessary.
  if (tx_head + size <= SERIAL_TX_BUFFER_SIZE) {
    memcpy(&tx_buff[tx_head], buffer, size);
    size_intermediate = size;
  } else {
    // memcpy till end of buffer then continue memcpy from beginning of buffer
    size_intermediate = SERIAL_TX_BUFFER_SIZE - tx_head;
    memcpy(&tx_buff[tx_head], buffer, size_intermediate);
    memcpy(&tx_buff[0], buffer + size_intermediate,
           size - size_intermediate);
  }

  // Data are copied to buffer, move head pointer accordingly
  tx_head = (tx_head + size) % SERIAL_TX_BUFFER_SIZE;

  // Transfer data with HAL only is there is no TX data transfer ongoing
  // otherwise, data transfer will be done asynchronously from callback
  if (!serial_tx_active()) {
    // note: tx_size correspond to size of HAL data transfer,
    // not the total amount of data in the buffer.
    // To compute size of data in buffer compare head and tail
    tx_size = size_intermediate;
    uart_attach_tx_callback(_tx_complete_irq, size_intermediate);
  }

  /* There is no real error management so just return transfer size requested*/
  return ret;
}

uint32_t  HardwareSerial::baudRate()
{
	return _baud;
	//return uartGetBaudRate(_uart);
}

void HardwareSerial::_rx_complete_irq(void)
{
  // No Parity error, read byte and store it in the buffer if there is room
  unsigned char c = uart_get_dr0();

  //if (  uint8_t uart_get_dr0() == 0) {

    rx_buffer_index_t i = (unsigned int)(rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != rx_tail) {
      rx_buff[rx_head] = c;
      rx_head = i;
    }
  //}
}

// Actual interrupt handlers //////////////////////////////////////////////////

void HardwareSerial::_tx_complete_irq(void)
{
  size_t remaining_data;
  // previous HAL transfer is finished, move tail pointer accordingly
  tx_tail = (tx_tail + tx_size) % SERIAL_TX_BUFFER_SIZE;

  // If buffer is not empty (head != tail), send remaining data
  if (tx_head != tx_tail) {
    remaining_data = (SERIAL_TX_BUFFER_SIZE + tx_head - tx_tail)
                     % SERIAL_TX_BUFFER_SIZE;
    // Limit the next transmission to the buffer end
    // because HAL is not able to manage rollover
    tx_size = min(remaining_data,
                       (size_t)(SERIAL_TX_BUFFER_SIZE - tx_tail));
    uart_attach_tx_callback(_tx_complete_irq, tx_size);
    return;
  }

  return;
}


