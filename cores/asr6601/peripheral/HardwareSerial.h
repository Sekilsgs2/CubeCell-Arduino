
#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Stream.h"
#include "uart.h"

#if !defined(SERIAL_TX_BUFFER_SIZE)
#define SERIAL_TX_BUFFER_SIZE 64
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
#define SERIAL_RX_BUFFER_SIZE 512
#endif

#if (SERIAL_TX_BUFFER_SIZE > 256)
typedef uint16_t tx_buffer_index_t;
#else
typedef uint8_t tx_buffer_index_t;
#endif
#if (SERIAL_RX_BUFFER_SIZE > 256)
typedef uint16_t rx_buffer_index_t;
#else
typedef uint8_t rx_buffer_index_t;
#endif

class HardwareSerial: public Stream
{
protected:
	bool _written;
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];
    int _uart;
    uint32_t _baud;
    int _rxPin=-1;
    int _txPin=-1;
    uint32_t _config;
public:
    HardwareSerial(int uart_nr);

    bool begin(uint32_t baud=115200, uint32_t config=SERIAL_8N1, int rxPin=-1, int txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL);
    void end();
    void updateBaudRate(unsigned long baud);
    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    void flush( bool txOnly);
    //size_t write(uint8_t c);
    //size_t write(const uint8_t *buffer, size_t size);
    bool busy(void);

    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
      return write((uint8_t)n);
    }
    size_t write(const uint8_t *buffer, size_t size);
    using Print::write; // pull in write(str) from Print
    operator bool()
    {
      return true;
    }
    uint32_t baudRate();
    //operator bool() const;
    size_t setRxBufferSize(size_t);
    void setDebugOutput(bool);
  // Interrupt handlers
    static void _rx_complete_irq(void);
	static void _tx_complete_irq(void);
	
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;

#endif // HardwareSerial_h
