/**
 ******************************************************************************
 * @file    tremo_uart.h
 * @author  ASR Tremo Team
 * @version v1.1.0
 * @date    2020-10-19
 * @brief   This file contains all the functions prototypes for the I2C firmware
 *          library.
 * @addtogroup Tremo_Drivers
 * @{
 * @defgroup UART
 * @{
 */

#ifndef __TREMO_UART_H
#define __TREMO_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "tremo_regs.h"

#define UART_BAUDRATE_110    (110)     /*!< baudrate 110 */
#define UART_BAUDRATE_300    (300)     /*!< baudrate 300 */
#define UART_BAUDRATE_600    (600)     /*!< baudrate 600 */
#define UART_BAUDRATE_1200   (1200)    /*!< baudrate 1200 */
#define UART_BAUDRATE_2400   (2400)    /*!< baudrate 2400 */
#define UART_BAUDRATE_4800   (4800)    /*!< baudrate 4800 */
#define UART_BAUDRATE_9600   (9600)    /*!< baudrate 9600 */
#define UART_BAUDRATE_14400  (14400)   /*!< baudrate 14400 */
#define UART_BAUDRATE_19200  (19200)   /*!< baudrate 19200 */
#define UART_BAUDRATE_38400  (38400)   /*!< baudrate 38400 */
#define UART_BAUDRATE_57600  (57600)   /*!< baudrate 57600 */
#define UART_BAUDRATE_115200 (115200)  /*!< baudrate 115200 */
#define UART_BAUDRATE_230400 (230400)  /*!< baudrate 230400 */
#define UART_BAUDRATE_460800 (460800)  /*!< baudrate 460800 */
#define UART_BAUDRATE_921600 (921600)  /*!< baudrate 921600 */

#define UART_FLAG_TX_FIFO_EMPTY (1 << 7)  /*!< TX FIFO is empty */
#define UART_FLAG_RX_FIFO_FULL  (1 << 6)  /*!< RX FIFO is full */
#define UART_FLAG_TX_FIFO_FULL  (1 << 5)  /*!< TX FIFO is full */
#define UART_FLAG_RX_FIFO_EMPTY (1 << 4)  /*!< RX FIFO is empty */
#define UART_FLAG_BUSY          (1 << 3)  /*!< Busy */

#define UART_INTERRUPT_RX_DONE       (1 << 4)  /*!< RX done */
#define UART_INTERRUPT_TX_DONE       (1 << 5)  /*!< TX done */
#define UART_INTERRUPT_RX_TIMEOUT    (1 << 6)  /*!< RX timeout */
#define UART_INTERRUPT_FRAME_ERROR   (1 << 7)  /*!< Frame error */
#define UART_INTERRUPT_PARITY_ERROR  (1 << 8)  /*!< Parity */
#define UART_INTERRUPT_BREAK_ERROR   (1 << 9)  /*!< Break error */
#define UART_INTERRUPT_OVERRUN_ERROR (1 << 10) /*!< Overrun error */

#define UART_IRDA_MODE_NORMAL 0  /*!< IRDA normal mode */
#define UART_IRDA_MODE_LP     1  /*!< IRDA low power mode */
#define UART_IRDA_LPBAUD16    ((uint32_t)1843200) /*!< IRDA low power baudrate */

#ifndef UART_IRQ_PRIO
#define UART_IRQ_PRIO       1
#endif
#ifndef UART_IRQ_SUBPRIO
#define UART_IRQ_SUBPRIO    0
#endif



/** @defgroup UART_State_Definition UART State Code Definition
  * @{
  */
#define  HAL_UART_STATE_RESET         0x00000000U    /*!< Peripheral is not initialized
                                                          Value is allowed for gState and RxState */
#define  HAL_UART_STATE_READY         0x00000020U    /*!< Peripheral Initialized and ready for use
                                                          Value is allowed for gState and RxState */
#define  HAL_UART_STATE_BUSY          0x00000024U    /*!< an internal process is ongoing
                                                          Value is allowed for gState only */
#define  HAL_UART_STATE_BUSY_TX       0x00000021U    /*!< Data Transmission process is ongoing
                                                          Value is allowed for gState only */
#define  HAL_UART_STATE_BUSY_RX       0x00000022U    /*!< Data Reception process is ongoing
                                                          Value is allowed for RxState only */
#define  HAL_UART_STATE_BUSY_TX_RX    0x00000023U    /*!< Data Transmission and Reception process is ongoing
                                                          Not to be used for neither gState nor RxState.Value is result
                                                          of combination (Or) between gState and RxState values */
#define  HAL_UART_STATE_TIMEOUT       0x000000A0U    /*!< Timeout state
                                                          Value is allowed for gState only */
#define  HAL_UART_STATE_ERROR         0x000000E0U    /*!< Error
                                                          Value is allowed for gState only */
														  
#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0U)

#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0U)

/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/* Use of interrupt control for register exclusive access */
/* Atomic 32-bit register access macro to set one or several bits */
#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t primask;                                        \
    primask = __get_PRIMASK();                               \
    __set_PRIMASK(1);                                        \
    SET_BIT((REG), (BIT));                                   \
    __set_PRIMASK(primask);                                  \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t primask;                                        \
    primask = __get_PRIMASK();                               \
    __set_PRIMASK(1);                                        \
    CLEAR_BIT((REG), (BIT));                                 \
    __set_PRIMASK(primask);                                  \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)            \
  do {                                                       \
    uint32_t primask;                                        \
    primask = __get_PRIMASK();                               \
    __set_PRIMASK(1);                                        \
    MODIFY_REG((REG), (CLEARMSK), (SETMASK));                \
    __set_PRIMASK(primask);                                  \
  } while(0)

/** @defgroup UART_Error_Definition   UART Error Definition
  * @{
  */
#define  HAL_UART_ERROR_NONE             (0x00000000U)    /*!< No error                */
#define  HAL_UART_ERROR_PE               (0x00000001U)    /*!< Parity error            */
#define  HAL_UART_ERROR_NE               (0x00000002U)    /*!< Noise error             */
#define  HAL_UART_ERROR_FE               (0x00000004U)    /*!< Frame error             */
#define  HAL_UART_ERROR_ORE              (0x00000008U)    /*!< Overrun error           */
#define  HAL_UART_ERROR_DMA              (0x00000010U)    /*!< DMA transfer error      */
#define  HAL_UART_ERROR_RTO              (0x00000020U)    /*!< Receiver Timeout error  */										



/**
 * @brief UART data width
 */
typedef enum {
    UART_DATA_WIDTH_5 = UART_LCR_H_WLEN_5,  /*!< data width is 5 bits */
    UART_DATA_WIDTH_6 = UART_LCR_H_WLEN_6,  /*!< data width is 6 bits */
    UART_DATA_WIDTH_7 = UART_LCR_H_WLEN_7,  /*!< data width is 7 bits */
    UART_DATA_WIDTH_8 = UART_LCR_H_WLEN_8   /*!< data width is 8 bits */
} uart_data_width_t;

/**
 * @brief UART stop bits
 */
typedef enum { 
    UART_STOP_BITS_1 = UART_LCR_H_STOP_1,  /*!< stop bits is 1*/
    UART_STOP_BITS_2 = UART_LCR_H_STOP_2   /*!< stop bits is 2*/
} uart_stop_bits_t;

/**
 * @brief UART flow control
 */
typedef enum {
    UART_FLOW_CONTROL_DISABLED = UART_CR_FLOW_CTRL_NONE,    /*!< flow control is disabled */
    UART_FLOW_CONTROL_RTS      = UART_CR_FLOW_CTRL_RTS,     /*!< RTS is enabled */
    UART_FLOW_CONTROL_CTS      = UART_CR_FLOW_CTRL_CTS,     /*!< CTS is enabled */
    UART_FLOW_CONTROL_CTS_RTS  = UART_CR_FLOW_CTRL_CTS_RTS  /*!< Both RTS and CTS are enabled */
} uart_flow_control_t;

/**
 * @brief UART parity
 */
typedef enum { 
    UART_PARITY_NO,   /*!< No parity */
    UART_PARITY_ODD,  /*!< Odd parity */
    UART_PARITY_EVEN  /*!< Even parity */
} uart_parity_t;

/**
 * @brief UART mode
 */
typedef enum {
    UART_MODE_TX   = UART_CR_UART_MODE_TX,   /*!< TX mode */
    UART_MODE_RX   = UART_CR_UART_MODE_RX,   /*!< RX mode */
    UART_MODE_TXRX = UART_CR_UART_MODE_TXRX  /*!< TX and RX mode */
} uart_mode_t;

/**
 * @brief UART TX fifo threshold
 */
typedef enum {
    UART_TX_FIFO_LEVEL_1_8 = UART_IFLS_TX_1_8,  /*!< 1/8 */
    UART_TX_FIFO_LEVEL_1_4 = UART_IFLS_TX_1_4,  /*!< 1/4 */
    UART_TX_FIFO_LEVEL_1_2 = UART_IFLS_TX_1_2,  /*!< 1/2 */
    UART_TX_FIFO_LEVEL_3_4 = UART_IFLS_TX_3_4,  /*!< 3/4 */
    UART_TX_FIFO_LEVEL_7_8 = UART_IFLS_TX_7_8   /*!< 7/8 */
} uart_tx_fifo_level_t;

/**
 * @brief UART RX fifo threshold
 */
typedef enum {
    UART_RX_FIFO_LEVEL_1_8 = UART_IFLS_RX_1_8,  /*!< 1/8 */
    UART_RX_FIFO_LEVEL_1_4 = UART_IFLS_RX_1_4,  /*!< 1/4 */
    UART_RX_FIFO_LEVEL_1_2 = UART_IFLS_RX_1_2,  /*!< 1/2 */
    UART_RX_FIFO_LEVEL_3_4 = UART_IFLS_RX_3_4,  /*!< 3/4 */
    UART_RX_FIFO_LEVEL_7_8 = UART_IFLS_RX_7_8   /*!< 7/8 */
} uart_rx_fifo_level_t;

/**
 * @brief UART DMA req
 */
typedef enum { 
    UART_DMA_REQ_TX = UART_DMACR_TX_EN_MASK,   /*!< DMA tx req */
    UART_DMA_REQ_RX = UART_DMACR_RX_EN_MASK    /*!< DMA rx req */
} uart_dma_req_t;

/**
 * @brief UART configuration
 */
typedef struct {
    uint32_t baudrate;                 /*!< Baudrate */
    uint32_t data_width;      /*!< Data width */
    uint32_t parity;              /*!< Parity */
    uint32_t stop_bits;        /*!< Stop bits */
    uint32_t flow_control;  /*!< Flow control policy */
    uint32_t mode;                  /*!< UART mode */
    uint8_t fifo_mode;                 /*!< FIFO mode */
} uart_config_t;

uint32_t calc_uart_baud(uint32_t uart_clk, uint32_t baud);

void uart_deinit(uart_t* uartx);
void uart_config_init(uart_config_t* config);
int32_t uart_init(uart_t* uartx, uart_config_t* config);


void uart_cmd(uart_t* uartx, bool newstate);
void uart_send_data(uart_t* uartx, uint8_t data);
uint8_t uart_receive_data(uart_t* uartx);

void uart_set_tx_fifo_threshold(uart_t* uartx, uint32_t uart_fifo_level);
void uart_set_rx_fifo_threshold(uart_t* uartx, uint32_t uart_fifo_level);

flag_status_t uart_get_flag_status(uart_t* uartx, uint32_t uart_flag);

void uart_config_interrupt(uart_t* uartx, uint32_t uart_interrupt, bool new_state);
void uart_clear_interrupt(uart_t* uartx, uint32_t uart_interrupt);
it_status_t uart_get_interrupt_status(uart_t* uartx, uint32_t uart_interrupt);

void uart_irda_config(uart_t* uartx, uint32_t uart_irda_mode);
void uart_irda_cmd(uart_t* uartx, bool new_state);

void uart_dma_config(uart_t* uartx, uart_dma_req_t dma_req, bool new_state);
void uart_dma_onerror_config(uart_t* uartx, bool new_state);

uint8_t uart_get_dr0(void);
//void uart_attach_rx_callback(void (*callback)(void));
//void uart_attach_tx_callback(void (*callback)(void), volatile uint8_t *Data, size_t size);

//bool serial_tx_active(void);
//bool serial_rx_active(void);

uint8_t UART_Transmit_IT(uint8_t *pData, uint16_t Size);

extern void tx_uart_dma_irq_handle(void);

/* Exported types ------------------------------------------------------------*/


/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/** 
  * @brief  HAL Lock structures definition  
  */
typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

typedef uint32_t HAL_UART_StateTypeDef;

/**
  * @brief  UART handle Structure definition
  */
typedef struct __UART_HandleTypeDef
{
  uart_t            	   *Instance;                /*!< UART registers base address        */

  uart_config_t         Init;                     /*!< UART communication parameters      */

  uint8_t                  *pTxBuffPtr;              /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                 TxXferSize;               /*!< UART Tx Transfer size              */

  __IO uint16_t            TxXferCount;              /*!< UART Tx Transfer Counter           */

  uint8_t                  *pRxBuffPtr;              /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                 RxXferSize;               /*!< UART Rx Transfer size              */

  __IO uint16_t            RxXferCount;              /*!< UART Rx Transfer Counter           */

  void (*RxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Rx IRQ handler */

  void (*TxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Tx IRQ handler */

  HAL_LockTypeDef           Lock;                    /*!< Locking object                     */

  __IO HAL_UART_StateTypeDef    gState;              /*!< UART state information related to global Handle management
                                                          and also related to Tx operations. This parameter
                                                          can be a value of @ref HAL_UART_StateTypeDef */

  __IO HAL_UART_StateTypeDef    RxState;             /*!< UART state information related to Rx operations. This
                                                          parameter can be a value of @ref HAL_UART_StateTypeDef */

  __IO uint32_t                 ErrorCode;           /*!< UART Error code                    */

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  void (* TxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Tx Half Complete Callback        */
  void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Tx Complete Callback             */
  void (* RxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);        /*!< UART Rx Half Complete Callback        */
  void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Rx Complete Callback             */
  void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);             /*!< UART Error Callback                   */
  void (* AbortCpltCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Abort Complete Callback          */
  void (* AbortTransmitCpltCallback)(struct __UART_HandleTypeDef *huart); /*!< UART Abort Transmit Complete Callback */
  void (* AbortReceiveCpltCallback)(struct __UART_HandleTypeDef *huart);  /*!< UART Abort Receive Complete Callback  */
#if defined(USART_CR1_UESM)
#if defined(USART_CR3_WUFIE)
  void (* WakeupCallback)(struct __UART_HandleTypeDef *huart);            /*!< UART Wakeup Callback                  */
#endif /* USART_CR3_WUFIE */
#endif /* USART_CR1_UESM */
  void (* RxEventCallback)(struct __UART_HandleTypeDef *huart, uint16_t Pos); /*!< UART Reception Event Callback     */

  void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);           /*!< UART Msp Init callback                */
  void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);         /*!< UART Msp DeInit callback              */
#endif  /* USE_HAL_UART_REGISTER_CALLBACKS */

} UART_HandleTypeDef;


typedef struct serial_s serial_t;

struct serial_s {
  /*  The 1st 2 members USART_TypeDef *uart
   *  and UART_HandleTypeDef handle should
   *  be kept as the first members of this struct
   *  to have get_serial_obj() function work as expected
   */
  uart_t *uart;
  UART_HandleTypeDef handle;
  void (*rx_callback)(serial_t *);
  int (*tx_callback)(serial_t *);
  IRQn_Type irq;
  uint8_t index;
  uint8_t recv;
  uint8_t *rx_buff;
  uint8_t *tx_buff;
  uint16_t rx_tail;
  uint16_t tx_head;
  volatile uint16_t rx_head;
  volatile uint16_t tx_tail;
  size_t tx_size;
  int fre;
  int pae;
  int ove;
};

int uart_getc(serial_t *obj, unsigned char *c);
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);

void uart_attach_rx_callback(serial_t *obj, void (*callback)(serial_t *));
void uart_attach_tx_callback(serial_t *obj, int (*callback)(serial_t *), size_t size);

uint8_t serial_tx_active(serial_t *obj);
uint8_t serial_rx_active(serial_t *obj);

void uart_init_stm32(serial_t *obj, uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits);

#ifdef __cplusplus
}
#endif
#endif //__TREMO_UART_H

/**
 * @} 
 * @}
 */
