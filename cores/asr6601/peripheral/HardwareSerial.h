
#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Stream.h"
#include "uart.h"

#if !defined(SERIAL_TX_BUFFER_SIZE)
  #define SERIAL_TX_BUFFER_SIZE 256
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
  #define SERIAL_RX_BUFFER_SIZE 64
#endif
#if (SERIAL_TX_BUFFER_SIZE>256)
  typedef uint16_t tx_buffer_index_t;
#else
  typedef uint8_t tx_buffer_index_t;
#endif
#if  (SERIAL_RX_BUFFER_SIZE>256)
  typedef uint16_t rx_buffer_index_t;
#else
  typedef uint8_t rx_buffer_index_t;
#endif

class HardwareSerial: public Stream
{
protected:
    // Has any byte been written to the UART since begin()
    bool _written;

    // Don't put any members after these buffers, since only the first
    // 32 bytes of this struct can be accessed quickly using the ldd
    // instruction.
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

    serial_t _serial;
public:
    HardwareSerial(void *peripheral);
    bool begin(unsigned long baud);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    int availableForWrite(void);
    virtual void flush(void);
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
    void setRx(uint32_t _rx);
    void setTx(uint32_t _tx);
	int getfrerr(void);
	int getoverr(void);
	int getpaerr(void);
	
    uint32_t baudRate();
    //operator bool() const;
    size_t setRxBufferSize(size_t);
    void setDebugOutput(bool);
    // Interrupt handlers
    static void _rx_complete_irq(serial_t *obj);
    static int _tx_complete_irq(serial_t *obj);

  private:
    bool _rx_enabled;
    uint8_t _config;
    unsigned long _baud;
	
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
//extern HardwareSerial Serial2;
//extern HardwareSerial Serial3;

#endif // HardwareSerial_h
