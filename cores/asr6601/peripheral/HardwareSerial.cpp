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

volatile uint8_t rx_buff[SERIAL_RX_BUFFER_SIZE];
uint8_t * rx_bounce_buf[2];
uint8_t * tx_buff;
volatile uint16_t rx_head;
volatile uint16_t rx_tail;
volatile uint16_t tx_head;
volatile uint16_t tx_tail;
volatile size_t tx_size;
static dma_dev_t dma_dev = {0};
volatile uint8_t rx_bounce_idx;
rx_buffer_index_t ind = 0;
static uart_config_t uart_init_struct;

#define DMA_RX_SIZE 16

HardwareSerial::HardwareSerial(int uart): _uart(uart) {

}

void tx_uart_dma_irq_handle(void) {
  /*	
    size_t remaining_data;
    __disable_irq();
    // previous HAL transfer is finished, move tail pointer accordingly
    tx_tail = (tx_tail + tx_size) % SERIAL_TX_BUFFER_SIZE;
    __enable_irq();
    // If buffer is not empty (head != tail), send remaining data
    if (tx_head != tx_tail) {
      remaining_data = (SERIAL_TX_BUFFER_SIZE + tx_head - tx_tail)
                       % SERIAL_TX_BUFFER_SIZE;
      // Limit the next transmission to the buffer end
      // because HAL is not able to manage rollover
      tx_size = min(remaining_data,
                         (size_t)(SERIAL_TX_BUFFER_SIZE - tx_tail));
  	memcpy(printf_dma_buf,&tx_buff[tx_tail],tx_size);		   
  	dma_dev.src        = (uint32_t)(printf_dma_buf);
  	dma_dev.block_size = tx_size;
  	
      dma_init(&dma_dev);
      dma_ch_enable(dma_dev.dma_num, 0);

      uart_dma_config(UART0, UART_DMA_REQ_TX, true);
  	
  	//interrupts();
    }
    else {
  	  dma_finalize(&dma_dev);
  	  tx_state = 0;
    }
    */
}

RAM_FUNC_ATTR void dma_rx_enable(void) {
  rx_bounce_idx ^= 1;
  dma_dev.dest = (uint32_t) rx_bounce_buf[rx_bounce_idx];
  dma_init( & dma_dev);
  dma_ch_enable(dma_dev.dma_num, 0);
 uart_dma_config(UART0, UART_DMA_REQ_RX, true);
}

RAM_FUNC_ATTR void rx_uart_dma_irq_handle(void) {

  const uint8_t curidx = rx_bounce_idx;
  dma_rx_enable();
  int size = DMA_RX_SIZE;
  for (int i = 0; i < DMA_RX_SIZE; i++) {
    // No Parity error, read byte and store it in the buffer if there is room
    unsigned char c = rx_bounce_buf[curidx][i];

    ind = (unsigned int)(rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (ind != rx_tail) {
      rx_buff[rx_head] = c;
      rx_head = ind;
    }
  }
}

bool HardwareSerial::begin(uint32_t baud, uint32_t config, int rxPin, int txPin, bool invert, unsigned long timeout_ms) {
  rx_head = 0;
  rx_tail = 0;
  tx_buff = _tx_buffer;
  tx_head = 0;
  tx_tail = 0;
  tx_size = 0;
  _baud = baud;
  _config = config;

  if (_uart == 0 && (rxPin < 0 || txPin < 0)) {
    rxPin = UART0_RX;
    txPin = UART0_TX;
  } else if (_uart == 1 && (rxPin < 0 || txPin < 0)) {
    rxPin = UART1_RX;
    txPin = UART1_TX;
  } else if (_uart == 2 && (rxPin < 0 || txPin < 0)) {
    rxPin = UART2_RX;
    txPin = UART2_TX;
  } else if (_uart == 3 && (rxPin < 0 || txPin < 0)) {
    rxPin = UART3_RX;
    txPin = UART3_TX;
  }
  _rxPin = rxPin;
  _txPin = txPin;

  _uart = 1;

  uart_cmd(UART0, DISABLE);

  rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA0, true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA1, true);

  gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
  gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);

  GPIOB -> PER |= (1 << GPIO_PIN_0);
  GPIOB -> PSR |= (1 << GPIO_PIN_0);

  uart_config_init( & uart_init_struct);
  uart_init_struct.fifo_mode = ENABLE;
  uart_init_struct.baudrate = baud;
  uart_init_struct.data_width = UART_DATA_WIDTH_8;
  uart_init_struct.parity = UART_PARITY_NO;
  uart_init_struct.stop_bits = UART_STOP_BITS_1;
  uart_init_struct.mode = UART_MODE_TXRX;
  uart_init_struct.flow_control = UART_FLOW_CONTROL_DISABLED;

  uart_init(UART0, & uart_init_struct);
  uart_config_interrupt(UART0, UART_INTERRUPT_TX_DONE, ENABLE);
  uart_set_rx_fifo_threshold(UART0, UART_IFLS_RX_7_8);
  uart_set_tx_fifo_threshold(UART0, UART_IFLS_TX_7_8);
  uart_cmd(UART0, ENABLE);

  rx_bounce_buf[0] = (uint8_t * ) malloc(DMA_RX_SIZE);
  rx_bounce_buf[1] = (uint8_t * ) malloc(DMA_RX_SIZE);

  __disable_irq();
  dma_dev.dma_num = 0;
  dma_dev.ch = 0;
  dma_dev.mode = P2M_MODE;
  dma_dev.src = (uint32_t) & (UART0 -> DR);
  dma_dev.dest = (uint32_t) rx_bounce_buf[rx_bounce_idx];
  dma_dev.priv = rx_uart_dma_irq_handle;
  dma_dev.data_width = 0;
  dma_dev.block_size = DMA_RX_SIZE;
  dma_dev.src_msize = 1;
  dma_dev.dest_msize = 1;
  dma_dev.handshake = DMA_HANDSHAKE_UART_0_RX;

  dma_init( & dma_dev);
  dma_ch_enable(dma_dev.dma_num, 0);

  uart_dma_config(UART0, UART_DMA_REQ_RX, true);
  __enable_irq();

  return true;
}

void HardwareSerial::updateBaudRate(unsigned long baud) {
  _baud = baud;
}

void HardwareSerial::end() {
}

int HardwareSerial::available(void) {
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + rx_head - rx_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerial::availableForWrite(void) {
  tx_buffer_index_t head = tx_head;
  tx_buffer_index_t tail = tx_tail;

  if (head >= tail) {
    return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  }
  return tail - head - 1;
}

bool HardwareSerial::busy(void) {
  return serial_tx_active();
}

int HardwareSerial::peek(void) {
  if (rx_head == rx_tail) {
    return -1;
  } else {
    return rx_buff[rx_tail];
  }
}

int HardwareSerial::read(void) {

  if (rx_head == rx_tail) {
    return -1;
  } else {
    unsigned char c = rx_buff[rx_tail];
    rx_tail = (rx_buffer_index_t)(rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

extern uint16_t cc;

int HardwareSerial::rxst(void) {
  return cc;
}

void HardwareSerial::flush(void) {
  if (!_written) {
    return;
  }

  while ((tx_head != tx_tail)) {
    // nop, the interrupt handler will free up space for us
  }
}

void HardwareSerial::flush(bool txOnly) {
  if (!_written) {
    return;
  }

  while ((tx_head != tx_tail)) {
    // nop, the interrupt handler will free up space for us
  }
}

size_t HardwareSerial::write(uint8_t c) {
  uint8_t buff = c;
  return write( & buff, 1);
}

size_t HardwareSerial::write(const uint8_t * buffer, size_t size) {
  size_t size_intermediate;
  size_t ret = size;
  size_t available = availableForWrite();
  size_t available_till_buffer_end = SERIAL_TX_BUFFER_SIZE - tx_head;

  _written = true;

  if (!serial_tx_active()) {
    if (uart_get_flag_status(UART0, UART_FLAG_TX_FIFO_EMPTY) == 1) {
      while ((uart_get_flag_status(UART0, UART_FLAG_TX_FIFO_FULL) != 1) && (size > 0)) {
        UART0 -> DR = (uint8_t)( * buffer & (uint8_t) 0xFF);
        buffer++;
        size--;
      }
    }
  }
  if (size == 0)
    return ret;

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
    memcpy( & tx_buff[tx_head], buffer, size);
    size_intermediate = size;
  } else {
    //__disable_irq();
    // memcpy till end of buffer then continue memcpy from beginning of buffer
    size_intermediate = SERIAL_TX_BUFFER_SIZE - tx_head;
    memcpy( & tx_buff[tx_head], buffer, size_intermediate);
    memcpy( & tx_buff[0], buffer + size_intermediate,
      size - size_intermediate);
    //__enable_irq();
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

void HardwareSerial::setRx(uint32_t _rx) {}

void HardwareSerial::setTx(uint32_t _tx) {}

uint32_t HardwareSerial::baudRate() {
  return _baud;
}

void HardwareSerial::_rx_complete_irq(void) {
  while (uart_get_flag_status(UART0, UART_FLAG_RX_FIFO_EMPTY) != 1) {
    // No Parity error, read byte and store it in the buffer if there is room
    unsigned char c = uart_get_dr0();

    ind = (unsigned int)(rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (ind != rx_tail) {
      rx_buff[rx_head] = c;
      rx_head = ind;
    }
  }
}

// Actual interrupt handlers //////////////////////////////////////////////////

RAM_FUNC_ATTR void HardwareSerial::_tx_complete_irq(void) {
  size_t remaining_data;
  // previous HAL transfer is finished, move tail pointer accordingly
  tx_tail = (tx_tail + tx_size) % SERIAL_TX_BUFFER_SIZE;

  // If buffer is not empty (head != tail), send remaining data
  if (tx_head != tx_tail) {
    remaining_data = (SERIAL_TX_BUFFER_SIZE + tx_head - tx_tail) %
      SERIAL_TX_BUFFER_SIZE;
    // Limit the next transmission to the buffer end
    // because HAL is not able to manage rollover
    tx_size = min(remaining_data,
      (size_t)(SERIAL_TX_BUFFER_SIZE - tx_tail));
    uart_attach_tx_callback(_tx_complete_irq, tx_size);
    return;
  }

  return;
}