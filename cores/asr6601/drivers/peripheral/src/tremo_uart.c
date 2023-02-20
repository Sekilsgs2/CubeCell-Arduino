#include <stdio.h>
#include <stdbool.h>
#include "tremo_rcc.h"
#include "tremo_uart.h"
#include "uart.h"

static UART_HandleTypeDef *uart_handlers[2] = {NULL};

#define UART_CR_FLOW_CTRL_RXE     ((uint32_t)0x0200)
#define UART_CR_FLOW_CTRL_TXE     ((uint32_t)0x0100)
#define UART_HARDWARE_FLOW_CONTROL_MASK		((uint16_t)0xFF80)

#define CR_EN_Set                 ((uint16_t)0x0001)  /*!< UART Enable Mask */
#define CR_EN_Reset               ((uint16_t)0xFFFE)  /*!< UART Disable Mask */

#define CR_SIREN_Set              ((uint16_t)0x0002)  /*!< UART IrDA mode Enable Mask */
#define CR_SIREN_Reset            ((uint16_t)0xFFFD)  /*!< UART IrDA mode Disable Mask */

#define CR_FC_Mask                ((uint16_t)0xFF80)  /*!< UART CR Flow control Bits Mask */

#define LCR_H_BRK_Set             ((uint16_t)0x0001)  /*!< UART Break Line Set Mask */
#define LCR_H_BRK_Reset           ((uint16_t)0xFFFE)  /*!< UART Break Line Reset Mask */
#define LCR_H_Clear_Mask          ((uint16_t)0x00FF)  /*!< UART LCR_H Mask */

#define FBRD_Fract_Mask           ((uint16_t)0x003F)  /*!< Fractional divider Mask */

#define IrLPBaud16                ((uint32_t)1843200) /*!< F_IrLPBaud16 nominal frequency Hz */

#define UART1_BRG_Mask            ((uint32_t)0x0007)  /*!< UART1 clock divider Mask */
#define UART2_BRG_Mask            ((uint32_t)0x0700)  /*!< UART2 clock divider Mask */
#define UART2_BRG_Offs            ((uint32_t)0x0008)  /*!< UART2 clock divider Offset */


///////////////////////

/* Aim of the function is to get serial_s pointer using huart pointer */
/* Highly inspired from magical linux kernel's "container_of" */
serial_t *get_serial_obj(UART_HandleTypeDef *huart)
{
  struct serial_s *obj_s;
  serial_t *obj;

  obj_s = (struct serial_s *)((char *)huart - offsetof(struct serial_s, handle));
  obj = (serial_t *)((char *)obj_s - offsetof(serial_t, uart));

  return (obj);
}

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t prioritygroup = 0x00U;

  /* Check the parameters */
  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));

  prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

void HAL_UART_Init(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  if (huart->gState == HAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = HAL_UNLOCKED;

  }
  
  /* Initialize the UART ErrorCode */
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  /* Initialize the UART State */
  huart->gState = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;
  
  __HAL_UNLOCK(huart);

  return;
}

/**
  * @brief  Function called to initialize the uart interface
  * @param  obj : pointer to serial_t structure
  * @retval None
  */
void uart_init_stm32(serial_t *obj, uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits)
{
  if (obj == NULL) {
    return;
  }

  UART_HandleTypeDef *huart = &(obj->handle);

  obj->uart = UART0;
  
  obj->pae = 0;
  obj->fre = 0;
  obj->ove = 0;



  if (obj->uart == UART0) {
  uart_cmd(UART0, DISABLE);
  uart_cmd(UART3, DISABLE);

  rcc_rst_peripheral(RCC_PERIPHERAL_UART0, true);
  rcc_rst_peripheral(RCC_PERIPHERAL_UART0, false);

  rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
  
  rcc_rst_peripheral(RCC_PERIPHERAL_UART3, true);
  rcc_rst_peripheral(RCC_PERIPHERAL_UART3, false);

  rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART3, true);
  rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);

  //gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);
  gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);
  gpio_set_iomux(GPIOC, GPIO_PIN_12, 1);

  //GPIOB -> PER |= (1 << GPIO_PIN_0);
  //GPIOB -> PSR |= (1 << GPIO_PIN_0);
  
  
	uart_cmd(UART0, DISABLE);
	uart_cmd(UART3, DISABLE);
    obj->index = 0;
    obj->irq = UART0_IRQn;
  }

  /* Configure uart */
  uart_handlers[obj->index] = huart;
  uart_config_init( &huart->Init);
  huart->Instance          = (uart_t *)(obj->uart);
  huart->Init.baudrate     = baudrate;
  huart->Init.data_width   = databits;
  huart->Init.stop_bits     = stopbits;
  huart->Init.parity       = parity;
  huart->Init.mode         = UART_MODE_TX;
  huart->Init.flow_control    = UART_FLOW_CONTROL_DISABLED;
  //huart->Init.fifo_mode    = ENABLE;

  /* Set the NVIC priority for future interrupts */
  HAL_NVIC_SetPriority(UART3_IRQn, UART_IRQ_PRIO, UART_IRQ_SUBPRIO);

  uart_init(obj->uart, &huart->Init);
  huart->Init.mode         = UART_MODE_RX;
  uart_init(UART3, &huart->Init);
  HAL_UART_Init(huart);
  uart_cmd(UART0, ENABLE);
  uart_cmd(UART3, ENABLE);
}

/**
  * @brief RX interrupt handler for 7 or 8 bits data word length .
  * @param huart UART handle.
  * @retval None
  */
static void UART_RxISR_8BIT(UART_HandleTypeDef *huart)
{
  uint16_t uhMask = 0xFF;
  uint16_t  uhdata;
  serial_t *obj = get_serial_obj(huart);

  /* Check that a Rx process is ongoing */
  if (huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(UART3->DR);
    *huart->pRxBuffPtr = (uint8_t)(uhdata & (uint8_t)uhMask);
    huart->pRxBuffPtr++;
    huart->RxXferCount--;

    if (huart->RxXferCount == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupts */
      ATOMIC_CLEAR_BIT(UART3->ICR, (UART_INTERRUPT_RX_DONE | UART_INTERRUPT_FRAME_ERROR | UART_INTERRUPT_OVERRUN_ERROR | UART_INTERRUPT_PARITY_ERROR));

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = HAL_UART_STATE_READY;

      /* Clear RxISR function pointer */
      huart->RxISR = NULL;


      /*Call legacy weak Rx complete callback*/
	  if (obj) {
		obj->rx_callback(obj);
	  }

    }
  }
  else
  {
    /* Clear RXNE interrupt flag */
    //__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
  }
}


/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all HAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  huart->pRxBuffPtr  = pData;
  huart->RxXferSize  = Size;
  huart->RxXferCount = Size;
  huart->RxISR       = NULL;

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_BUSY_RX;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  ATOMIC_SET_BIT(UART3->IMSC, UART_INTERRUPT_OVERRUN_ERROR);

  huart->RxISR = UART_RxISR_8BIT;

  __HAL_UNLOCK(huart);

  /* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
  ATOMIC_SET_BIT(UART3->IMSC, UART_INTERRUPT_RX_DONE);
  return HAL_OK;
}


HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    __HAL_LOCK(huart);

    /* Set Reception type to Standard reception */
    //huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

    /* Check that USART RTOEN bit is set */
    //if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U)
    //{
      /* Enable the UART Receiver Timeout Interrupt */
      //ATOMIC_SET_BIT(huart->Instance->IMSC, UART_INTERRUPT_RX_TIMEOUT);
    //}

    return (UART_Start_Receive_IT(huart, pData, Size));
  }
  else
  {
    return HAL_BUSY;
  }
}


/**
  * @brief TX interrupt handler for 7 or 8 bits data word length .
  * @note   Function is called under interruption only, once
  *         interruptions have been enabled by HAL_UART_Transmit_IT().
  * @param huart UART handle.
  * @retval None
  */
static void UART_TxISR_8BIT(UART_HandleTypeDef *huart)
{
  serial_t *obj = get_serial_obj(huart);
  /* Check that a Tx process is ongoing */
  if (huart->gState == HAL_UART_STATE_BUSY_TX)
  {
    if (huart->TxXferCount == 0U)
    {
      /* Disable the UART Transmit Data Register Empty Interrupt */
      //ATOMIC_CLEAR_BIT(huart->Instance->IMSC, UART_INTERRUPT_TX_DONE);

	  /* Tx process is ended, restore huart->gState to Ready */
      huart->gState = HAL_UART_STATE_READY;

      /* Cleat TxISR function pointer */
      huart->TxISR = NULL;
	  
	        /*Call legacy weak Rx complete callback*/
	  if (obj) {
		obj->tx_callback(obj);
	  }
	  
    }
    else
    {
      huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr & (uint8_t)0xFF);
      huart->pTxBuffPtr++;
      huart->TxXferCount--;
    }
  }
}

/**
  * @brief Send an amount of data in interrupt mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         address of user data buffer containing data to be sent, should be aligned on a half word frontier (16 bits)
  *         (as sent data will be handled using u16 pointer cast). Depending on compilation chain,
  *         use of specific alignment compilation directives or pragmas might be required
  *         to ensure proper alignment for pData.
  * @param huart UART handle.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be sent.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  /* Check that a Tx process is not already ongoing */
  if (huart->gState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    __HAL_LOCK(huart);

    huart->pTxBuffPtr  = pData;
    huart->TxXferSize  = Size;
    huart->TxXferCount = Size;
    huart->TxISR       = NULL;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_BUSY_TX;

    huart->TxISR = UART_TxISR_8BIT;

    __HAL_UNLOCK(huart);

    /* Enable the Transmit Data Register Empty interrupt */
    ATOMIC_SET_BIT(huart->Instance->IMSC, UART_INTERRUPT_TX_DONE);
	
	
	if (huart->TxISR != NULL)
    {
      huart->TxISR(huart);
    }

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
 * Begin asynchronous RX transfer (enable interrupt for data collecting)
 *
 * @param obj : pointer to serial_t structure
 * @param callback : function call at the end of reception
 * @retval none
 */
void uart_attach_rx_callback(serial_t *obj, void (*callback)(serial_t *))
{
  if (obj == NULL) {
    return;
  }

  /* Exit if a reception is already on-going */
  if (serial_rx_active(obj)) {
    return;
  }
  obj->rx_callback = callback;

  /* Must disable interrupt to prevent handle lock contention */
  NVIC_DisableIRQ(UART3_IRQn);

  HAL_UART_Receive_IT(uart_handlers[obj->index], &(obj->recv), 1);

  /* Enable interrupt */
  NVIC_EnableIRQ(UART3_IRQn);
}


//////////////////////


uint32_t calc_uart_baud(uint32_t uart_clk, uint32_t baud)
{
    uint32_t int_div;
    uint32_t fac_div;
    uint32_t remainder;
    uint32_t temp;

/*

    temp = 16 * baud;
    if ((0 == baud) || uart_clk < temp) {
        return 0;
    }

    int_div   = (uint32_t)(uart_clk / temp);
    remainder = uart_clk % temp;
    temp      = 8 * remainder / baud;
    fac_div   = (temp >> 1) + (temp & 1);

    temp = ((int_div << 16) | (fac_div & 0xFFFF));
    return temp;
	*/
	/* Determine the integer part */
	remainder = uart_clk / (baud >> 2);
	int_div = remainder >> 6;
	/* Determine the fractional part */
	fac_div = (remainder & FBRD_Fract_Mask);
	return ((int_div << 16) | (fac_div & 0xFFFF));
}


/**
 * @brief Get the flag status of the specified UART flag
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_flag The specified flag
 *          This parameter can be one of the following values:
 *           @arg UART_FLAG_TX_FIFO_EMPTY: TX FIFO is empty
 *           @arg UART_FLAG_RX_FIFO_FULL:  RX FIFO is full
 *           @arg UART_FLAG_TX_FIFO_FULL:  TX FIFO is full
 *           @arg UART_FLAG_RX_FIFO_EMPTY: RX FIFO is empty
 *           @arg UART_FLAG_BUSY:          Busy
 * @return SET or RESET 
 */
flag_status_t uart_get_flag_status(uart_t* uartx, uint32_t uart_flag)
{
    if (uartx->FR & uart_flag)
        return SET;
    else
        return RESET;
}

/**
 * @brief Send 8 bit data through UART
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param data The data to be sent
 * @retval None 
 */
void uart_send_data(uart_t* uartx, uint8_t data)
{
    /* wait till tx fifo is not full */
    while (uart_get_flag_status(uartx, UART_FLAG_TX_FIFO_FULL) == SET)
        ;
    uartx->DR = data;
}

/**
 * @brief Receive 8 bit data through UART
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @retval uint8_t Data received 
 */
  uint8_t uart_receive_data(uart_t* uartx)
{
    /* wait till rx fifo is not empty */
    //while (uart_get_flag_status(uartx, UART_FLAG_RX_FIFO_EMPTY) == SET)
        //;

    return uartx->DR & 0xFF;
}

  uint8_t uart_get_dr0(void)
{

    return UART0->DR & 0xFF;
}

/**
 * @brief Config the interrupt of the specified UART flag
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_interrupt The specified interrupt
 *          This parameter can be one of the following values:
 *           @arg UART_INTERRUPT_RX_DONE:      RX done
 *           @arg UART_INTERRUPT_TX_DONE:      TX done
 *           @arg UART_INTERRUPT_RX_TIMEOUT:   RX timeout
 *           @arg UART_INTERRUPT_FRAME_ERROR:  Frame error
 *           @arg UART_INTERRUPT_PARITY_ERROR: Busy
 * @param new_state true or false
 * @retval None 
 */
void uart_config_interrupt(uart_t* uartx, uint32_t uart_interrupt, bool new_state)
{
    if (new_state == DISABLE) {
        uartx->IMSC &= ~(uart_interrupt);
    } else {
        uartx->IMSC |= (uart_interrupt);
    }
}

/**
 * @brief Deinitializes the UART peripheral registers to the reset values
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @retval None
 */
void uart_deinit(uart_t* uartx)
{
    uint32_t peripheral = RCC_PERIPHERAL_UART0;

    if (uartx == UART1)
        peripheral = RCC_PERIPHERAL_UART1;
    else if (uartx == UART2)
        peripheral = RCC_PERIPHERAL_UART2;
    else if (uartx == UART3)
        peripheral = RCC_PERIPHERAL_UART3;

    rcc_enable_peripheral_clk(peripheral, false);
    rcc_rst_peripheral(peripheral, true);
    rcc_rst_peripheral(peripheral, false);
}

/**
 * @brief Set the default value of the UART configuration
 * @param config UART configuration
 * @retval None
 */
void uart_config_init(uart_config_t* config)
{
    config->data_width   = UART_DATA_WIDTH_8;
    config->baudrate     = UART_BAUDRATE_115200;
    config->parity       = UART_PARITY_NO;
    config->stop_bits    = UART_STOP_BITS_1;
    config->mode         = UART_MODE_TXRX;
    config->flow_control = UART_FLOW_CONTROL_DISABLED;
    config->fifo_mode    = DISABLE;
}

/**
 * @brief Set the threshold of RX FIFO
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_fifo_level The threshold
 *        - @ref uart_rx_fifo_level_t
 * @retval None 
 */
void uart_set_rx_fifo_threshold(uart_t* uartx, uint32_t uart_fifo_level)
{
    TREMO_REG_SET(uartx->IFLS, UART_IFLS_RX, uart_fifo_level);
}

/**
 * @brief Set the threshold of TX FIFO
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_fifo_level The threshold
 *        - @ref uart_tx_fifo_level_t
 * @retval None 
 */
void uart_set_tx_fifo_threshold(uart_t* uartx, uint32_t uart_fifo_level)
{
    TREMO_REG_SET(uartx->IFLS, UART_IFLS_TX, uart_fifo_level);
}

/**
 * @brief Enable or disable the UART peripheral
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param new_state true or false
 * @retval None
 */
void uart_cmd(uart_t* uartx, bool new_state)
{
    TREMO_REG_EN(uartx->CR, UART_CR_UART_EN, new_state);
}

/**
 * @brief Get the interrupt status of the specified UART interrupt
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_interrupt The specified interrupt
 *          This parameter can be one of the following values:
 *           @arg UART_INTERRUPT_RX_DONE:      RX done
 *           @arg UART_INTERRUPT_TX_DONE:      TX done
 *           @arg UART_INTERRUPT_RX_TIMEOUT:   RX timeout
 *           @arg UART_INTERRUPT_FRAME_ERROR:  Frame error
 *           @arg UART_INTERRUPT_PARITY_ERROR: Busy
 * @return SET or RESET 
 */
it_status_t uart_get_interrupt_status(uart_t* uartx, uint32_t uart_interrupt)
{
    if (uartx->MIS & uart_interrupt)
        return SET;
    else
        return RESET;
}

/**
 * @brief Clear the interrupt status of the specified UART interrupt
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_interrupt The specified interrupt
 *          This parameter can be one of the following values:
 *           @arg UART_INTERRUPT_RX_DONE:      RX done
 *           @arg UART_INTERRUPT_TX_DONE:      TX done
 *           @arg UART_INTERRUPT_RX_TIMEOUT:   RX timeout
 *           @arg UART_INTERRUPT_FRAME_ERROR:  Frame error
 *           @arg UART_INTERRUPT_PARITY_ERROR: Busy
 * @retval None 
 */
void uart_clear_interrupt(uart_t* uartx, uint32_t uart_interrupt)
{
    uartx->ICR = uart_interrupt;
}

/**
 * @brief Init the UART peripheral registers according to the specified parameters
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param config UART configuration
 * @retval ERRNO_OK Init successfully
 * @retval ERRNO_ERROR Init failed
 */
 


int32_t uart_init(uart_t* uartx, uart_config_t* config)
{
    uint32_t uart_clk_freq = 0;
    uint32_t clk_src       = 0;
	uint32_t tmpreg;

    // disable UART
	uartx->RSC_ECR = 0;
	//uartx->CR = 0;
    //uartx->CR &= ~UART_CR_UART_EN;
    // flush fifo by setting FEN = 0
    uartx->LCR_H &= ~UART_LCR_H_FEN;
    uartx->IMSC = 0;

    if (uartx == UART0)
        clk_src = (uint32_t)rcc_get_uart0_clk_source() >> 15;
    else if (uartx == UART1)
        clk_src = (uint32_t)rcc_get_uart1_clk_source() >> 13;
    else if (uartx == UART2)
        clk_src = (uint32_t)rcc_get_uart2_clk_source() >> 11;
    else if (uartx == UART3)
        clk_src = (uint32_t)rcc_get_uart3_clk_source() >> 9;

    switch (clk_src) {
    case 1:
        uart_clk_freq = RCC_FREQ_4M;
        break;
    case 2:
        uart_clk_freq = RCC_FREQ_32768;
        break;
    case 3:
        uart_clk_freq = RCC_FREQ_24M;
        break;
    case 0:
    default: {
        uart_clk_freq = (uartx == UART0 || uartx == UART1) ? rcc_get_clk_freq(RCC_PCLK0) : rcc_get_clk_freq(RCC_PCLK1);
        break;
    }
    }

    if (uart_clk_freq < 16 * config->baudrate)
        return ERRNO_ERROR;

    // set baudrate
    uint32_t br_div = calc_uart_baud(uart_clk_freq, config->baudrate);
    uartx->IBRD     = br_div >> 16; /* baudrate divdier register must be placed
                                       before a LCR_H write */
    uartx->FBRD = br_div & 0x3f;
	
	//tmpreg = uartx->LCR_H;
	//tmpreg |= config->data_width | config->stop_bits
			//| config->parity | config->fifo_mode;
	//uartx->LCR_H = tmpreg;
	
	/* UART CR configuration */
	//tmpreg = uartx->CR;
	/* Clear UART CR Flow control bits */
	//tmpreg &= ~CR_FC_Mask;
	/* Set UART CR Flow control bits */
	//tmpreg |= UART_CR_FLOW_CTRL_RXE | UART_CR_FLOW_CTRL_TXE;
	/* Write to UART CR */
	//uartx->CR = tmpreg;

    // set LCR_H
    TREMO_REG_SET(uartx->LCR_H, UART_LCR_H_WLEN, config->data_width);
    TREMO_REG_SET(uartx->LCR_H, UART_LCR_H_STOP, config->stop_bits);
    TREMO_REG_EN(uartx->LCR_H, UART_LCR_H_FEN, config->fifo_mode);
    switch (config->parity) {
    case UART_PARITY_ODD:
        uartx->LCR_H |= UART_LCR_H_PEN;
        uartx->LCR_H &= ~UART_LCR_H_EPS_EVEN;
        break;
    case UART_PARITY_EVEN:
        uartx->LCR_H |= UART_LCR_H_PEN;
        uartx->LCR_H |= UART_LCR_H_EPS_EVEN;
        break;
    case UART_PARITY_NO:
        uartx->LCR_H &= ~UART_LCR_H_PEN;
        break;
    default:
        break;
    }
	

    TREMO_REG_SET(uartx->CR, UART_CR_UART_MODE, config->mode);
    TREMO_REG_SET(uartx->CR, UART_CR_FLOW_CTRL, config->flow_control);

    return ERRNO_OK;
}

/**
 * @brief Config UART IRDA Mode
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param uart_irda_mode IRDA mode
 *          This parameter can be one of the following values:
 *           @arg UART_IRDA_MODE_NORMAL: Normal mode
 *           @arg UART_IRDA_MODE_LP:     Low power mode
 * @retval None
 */
void uart_irda_config(uart_t* uartx, uint32_t uart_irda_mode)
{
    uint32_t uart_clk_freq = 0;
    uint32_t clk_src       = 0;

    if (uartx == UART0)
        clk_src = (uint32_t)rcc_get_uart0_clk_source();
    else if (uartx == UART1)
        clk_src = (uint32_t)rcc_get_uart1_clk_source();
    else if (uartx == UART2)
        clk_src = (uint32_t)rcc_get_uart2_clk_source();
    else if (uartx == UART3)
        clk_src = (uint32_t)rcc_get_uart3_clk_source();

    switch (clk_src) {
    case 1:
        uart_clk_freq = RCC_FREQ_4M;
        break;
    case 2:
        uart_clk_freq = RCC_FREQ_32768;
        break;
    case 3:
        uart_clk_freq = RCC_FREQ_24M;
        break;
    case 0:
    default: {
        uart_clk_freq = (uartx == UART0 || uartx == UART1) ? rcc_get_clk_freq(RCC_PCLK0) : rcc_get_clk_freq(RCC_PCLK1);
        break;
    }
    }
    if (uart_irda_mode == UART_IRDA_MODE_LP) {
        uartx->ILPR = (uart_clk_freq + UART_IRDA_LPBAUD16 / 2) / UART_IRDA_LPBAUD16;
        uartx->CR |= UART_CR_SIR_LPIRDA_EN;
    } else {
        uartx->CR &= ~UART_CR_SIR_LPIRDA_EN;
    }
}

/**
 * @brief Enable or disable the UART IRDA
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param new_state true or false
 * @retval None
 */
void uart_irda_cmd(uart_t* uartx, bool new_state)
{
    TREMO_REG_EN(uartx->CR, UART_CR_SIR_EN, new_state);
}

/**
 * @brief Enable or disable the UART DMA function
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param dma_req UART DMA request
 *        - @ref uart_dma_req_t
 * @param new_state true or false
 * @retval None
 */
void uart_dma_config(uart_t* uartx, uart_dma_req_t dma_req, bool new_state)
{
    TREMO_REG_EN(uartx->DMACR, dma_req, new_state);
}

/**
 * @brief Enable or disable the UART DMA on error function
 * @param uartx Select the UART peripheral number(UART0, UART1, UART2 and UART3)
 * @param new_state true or false
 * @retval None
 */
void uart_dma_onerror_config(uart_t* uartx, bool new_state)
{
    TREMO_REG_EN(uartx->DMACR, UART_DMACR_ONERR_EN_MASK, new_state);
}


/**
  * @brief Handle UART interrupt request.
  * @param huart UART handle.
  * @retval None
  */
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(UART3->MIS);

  uint32_t errorflags;
  uint32_t errorcode;
  
  serial_t *obj = get_serial_obj(huart);

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(UART_INTERRUPT_PARITY_ERROR | UART_INTERRUPT_FRAME_ERROR | UART_INTERRUPT_OVERRUN_ERROR));
  if (errorflags == 0U)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
   if ( uart_get_interrupt_status(UART3, UART_INTERRUPT_RX_DONE) )
    {
      if (huart->RxISR != NULL)
      {
        huart->RxISR(huart);
      }
	  uart_clear_interrupt(UART3, UART_INTERRUPT_RX_DONE);
      //return;
    }
  }

  /* If some errors occur */
  if ((errorflags != 0))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isrflags & UART_INTERRUPT_PARITY_ERROR) != 0U))
    {
      //__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);
	  obj->pae++;
	  uart_clear_interrupt(UART3, UART_INTERRUPT_PARITY_ERROR);

      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isrflags & UART_INTERRUPT_FRAME_ERROR) != 0U) )
    {
		obj->fre++;
      //__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
	  uart_clear_interrupt(UART3, UART_INTERRUPT_FRAME_ERROR);

      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (((isrflags & UART_INTERRUPT_OVERRUN_ERROR) != 0U))
    {
      //__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
	  obj->ove++;
	  uart_clear_interrupt(UART3, UART_INTERRUPT_OVERRUN_ERROR);

      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be ----------------------------*/
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver --------------------------------------------------*/
      if (((isrflags & UART_INTERRUPT_RX_DONE) != 0U))
      {
        if (huart->RxISR != NULL)
        {
          huart->RxISR(huart);
        }
      }

      /* If Error is to be considered as blocking :
          - Receiver Timeout error in Reception
          - Overrun error in Reception
          - any error occurs in DMA mode reception
      */
      errorcode = huart->ErrorCode;

    }
    //return;

  } /* End if some error occurs */

}

void UART00_IRQHandler(void) {

  /* Clear pending interrupt */
  
  NVIC_ClearPendingIRQ(UART0_IRQn);
  /* UART in mode Transmitter ------------------------------------------------*/
  if ( uart_get_interrupt_status(UART0, UART_INTERRUPT_TX_DONE) )
  {
	uart_clear_interrupt(UART0, UART_INTERRUPT_TX_DONE);
    if (uart_handlers[0]->TxISR != NULL)
    {
      uart_handlers[0]->TxISR(uart_handlers[0]);
    }
    //return;
  }
  //HAL_UART_IRQHandler(uart_handlers[0]);  
}

void UART33_IRQHandler(void) {

  /* Clear pending interrupt */
  NVIC_ClearPendingIRQ(UART3_IRQn);
  HAL_UART_IRQHandler(uart_handlers[0]);  
}


/**
 * Begin asynchronous TX transfer.
 *
 * @param obj : pointer to serial_t structure
 * @param callback : function call at the end of transmission
 * @retval none
 */
void uart_attach_tx_callback(serial_t *obj, int (*callback)(serial_t *), size_t size)
{
  if (obj == NULL) {
    return;
  }
  obj->tx_callback = callback;

  /* Must disable interrupt to prevent handle lock contention */
  NVIC_DisableIRQ(obj->irq);
  NVIC_DisableIRQ(UART3_IRQn);

  /* The following function will enable UART_IT_TXE and error interrupts */
  HAL_UART_Transmit_IT(uart_handlers[obj->index], &obj->tx_buff[obj->tx_tail], size);

  /* Enable interrupt */
  NVIC_EnableIRQ(obj->irq);
  NVIC_EnableIRQ(UART3_IRQn);
}

/**
  * @brief Return the UART handle state.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval HAL state
  */
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart)
{
  uint32_t temp1;
  uint32_t temp2;
  temp1 = huart->gState;
  temp2 = huart->RxState;

  return (HAL_UART_StateTypeDef)(temp1 | temp2);
}

/**
 * Attempts to determine if the serial peripheral is already in use for RX
 *
 * @param obj The serial object
 * @return Non-zero if the RX transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
  return ((HAL_UART_GetState(uart_handlers[obj->index]) & HAL_UART_STATE_BUSY_RX) == HAL_UART_STATE_BUSY_RX);
}

/**
 * Attempts to determine if the serial peripheral is already in use for TX
 *
 * @param obj The serial object
 * @return Non-zero if the TX transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
  return ((HAL_UART_GetState(uart_handlers[obj->index]) & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX);
}

/**
  * @brief  Read receive byte from uart
  * @param  obj : pointer to serial_t structure
  * @retval last character received
  */
int uart_getc(serial_t *obj, unsigned char *c)
{
  if (obj == NULL) {
    return -1;
  }

  if (serial_rx_active(obj)) {
    return -1; /* Transaction ongoing */
  }

  *c = (unsigned char)(obj->recv);
  /* Restart RX irq */
  HAL_UART_Receive_IT(uart_handlers[obj->index], &(obj->recv), 1);

  return 0;
}

