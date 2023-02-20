#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "lora_config.h"
#include "HardwareSerial.h"

HardwareSerial Serial(UART0);
//HardwareSerial Serial1(1);

HardwareSerial::HardwareSerial(void *peripheral) {
  //_uart = uart;
  _serial.rx_buff = _rx_buffer;
  _serial.rx_head = 0;
  _serial.rx_tail = 0;
  _serial.tx_buff = _tx_buffer;
  _serial.tx_head = 0;
  _serial.tx_tail = 0;
  

}

bool HardwareSerial::begin(unsigned long baud) {
 
  
  uart_init_stm32(&_serial, (uint32_t)baud, UART_DATA_WIDTH_8, UART_PARITY_NO, UART_STOP_BITS_1);
  //uart_config_interrupt(UART0, UART_INTERRUPT_TX_DONE | UART_INTERRUPT_RX_DONE, ENABLE);
  uart_attach_rx_callback(&_serial, _rx_complete_irq);

  return true;
}

int HardwareSerial::getfrerr(void)
{
	return _serial.fre;
	
};
int HardwareSerial::getoverr(void)
{
	return _serial.ove;
	
};
int HardwareSerial::getpaerr(void)
{
	return _serial.pae;
};

void HardwareSerial::end() {
  // wait for transmission of outgoing data
  flush();

  uart_deinit(_serial.uart);

  // clear any received data
  _serial.rx_head = _serial.rx_tail;
}

int HardwareSerial::available(void) {
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _serial.rx_head - _serial.rx_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerial::availableForWrite(void) {
  tx_buffer_index_t head = _serial.tx_head;
  tx_buffer_index_t tail = _serial.tx_tail;

  if (head >= tail) {
    return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  }
  return tail - head - 1;
}

int HardwareSerial::peek(void) {
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  } else {
    return _serial.rx_buff[_serial.rx_tail];
  }
}

int HardwareSerial::read(void) {
  // if the head isn't ahead of the tail, we don't have any characters
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  } else {
    unsigned char c = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (rx_buffer_index_t)(_serial.rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

extern uint16_t cc;

void HardwareSerial::flush(void) {
  // If we have never written a byte, no need to flush. This special
  // case is needed since there is no way to force the TXC (transmit
  // complete) bit to 1 during initialization
  if (!_written) {
    return;
  }

  while ((_serial.tx_head != _serial.tx_tail)) {
    // nop, the interrupt handler will free up space for us
  }
  // If we get here, nothing is queued anymore (DRIE is disabled) and
  // the hardware finished transmission (TXC is set).
}

size_t HardwareSerial::write(uint8_t c) {
  uint8_t buff = c;
  return write(&buff, 1);
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
  size_t size_intermediate;
  size_t ret = size;
  size_t available = availableForWrite();
  size_t available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - _serial.tx_head;

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
    available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - _serial.tx_head;
  }

  // Copy data to buffer. Take into account rollover if necessary.
  if (_serial.tx_head + size <= SERIAL_TX_BUFFER_SIZE) {
    memcpy(&_serial.tx_buff[_serial.tx_head], buffer, size);
    size_intermediate = size;
  } else {
    // memcpy till end of buffer then continue memcpy from beginning of buffer
    size_intermediate = SERIAL_TX_BUFFER_SIZE - _serial.tx_head;
    memcpy(&_serial.tx_buff[_serial.tx_head], buffer, size_intermediate);
    memcpy(&_serial.tx_buff[0], buffer + size_intermediate,
           size - size_intermediate);
  }

  // Data are copied to buffer, move head pointer accordingly
  _serial.tx_head = (_serial.tx_head + size) % SERIAL_TX_BUFFER_SIZE;

  // Transfer data with HAL only is there is no TX data transfer ongoing
  // otherwise, data transfer will be done asynchronously from callback
  if (!serial_tx_active(&_serial)) {
    // note: tx_size correspond to size of HAL data transfer,
    // not the total amount of data in the buffer.
    // To compute size of data in buffer compare head and tail
    _serial.tx_size = size_intermediate;
    uart_attach_tx_callback(&_serial, _tx_complete_irq, size_intermediate);
  }

  /* There is no real error management so just return transfer size requested*/
  return ret;
}

void HardwareSerial::setRx(uint32_t _rx) {}

void HardwareSerial::setTx(uint32_t _tx) {}

uint32_t HardwareSerial::baudRate() {
  return _baud;
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerial::_rx_complete_irq(serial_t *obj)
{
  // No Parity error, read byte and store it in the buffer if there is room
  unsigned char c;

  if (uart_getc(obj, &c) == 0) {

    rx_buffer_index_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != obj->rx_tail) {
      obj->rx_buff[obj->rx_head] = c;
      obj->rx_head = i;
    }
  }
}

// Actual interrupt handlers //////////////////////////////////////////////////

int HardwareSerial::_tx_complete_irq(serial_t *obj)
{
  size_t remaining_data;
  // previous HAL transfer is finished, move tail pointer accordingly
  obj->tx_tail = (obj->tx_tail + obj->tx_size) % SERIAL_TX_BUFFER_SIZE;

  // If buffer is not empty (head != tail), send remaining data
  if (obj->tx_head != obj->tx_tail) {
    remaining_data = (SERIAL_TX_BUFFER_SIZE + obj->tx_head - obj->tx_tail)
                     % SERIAL_TX_BUFFER_SIZE;
    // Limit the next transmission to the buffer end
    // because HAL is not able to manage rollover
    obj->tx_size = min(remaining_data,
                       (size_t)(SERIAL_TX_BUFFER_SIZE - obj->tx_tail));
    uart_attach_tx_callback(obj, _tx_complete_irq, obj->tx_size);
    return -1;
  }

  return 0;
}
