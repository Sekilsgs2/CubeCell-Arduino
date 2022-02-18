/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project
*/

#include "Arduino.h"

SX126XHal *SX126XHal::instance = NULL;
void(*lora_callbacks[4])(void);

SX126XHal::SX126XHal()
{
    instance = this;
}

void SX126XHal::end()
{	
	digitalWrite(CONFIG_LORA_RFSW_VDD_PIN,LOW);
	LORAC->SSP_CR1 &= ~(0x1 << 1);
	lora_callbacks[0] = nullptr;
    IsrCallback = nullptr; // remove callbacks
}

uint16_t SpiInOut( uint16_t outData )
{
    uint8_t read_data = 0;
	bool tempIrq = BoardDisableIrq( );
	
	while (!(ssp_get_flag_status(LORAC_SSP, SSP_FLAG_TX_FIFO_NOT_FULL)))
            ; /* wait till tx fifo is not full, change to timeout mechanism */
    
    LORAC->SSP_DR = outData;
	
	while (!(ssp_get_flag_status(LORAC_SSP, SSP_FLAG_RX_FIFO_NOT_EMPTY)))
            ; /* wait till rx fifo is not empty, change to timeout mechanism */
	
	read_data = LORAC->SSP_DR & 0xFF;
	BoardEnableIrq(tempIrq);

    return( read_data );
}

void lorac_ssp_init(void)
{
    uint8_t scr;
    uint32_t ssp_clk_freq = 0;
	uint32_t spi_freq = 20000000; //SPI speed
    LORAC->SSP_CR1 &= ~(0x1 << 1); //disable LORA SPI
	LORAC->SSP_IMSC &= ~SSP_INTERRUPT_ALL;
	LORAC->SSP_ICR |= SSP_INTERRUPT_ALL;

    ssp_clk_freq = rcc_get_clk_freq(RCC_PCLK1);

    /* set frame format */
    LORAC->SSP_CR0 &= ~(0x3 << 4); /* reset FRF to 0 */
    LORAC->SSP_CR0 |= SSP_FRAME_FORMAT_SPI;

    /* set sclk divider */
    scr = ((ssp_clk_freq / 2 / spi_freq) - 1) ? ((ssp_clk_freq / 2 / spi_freq) - 1) : 0;
    LORAC->SSP_CPSR &= ~0xff;       /* reset CPSR to 0 */
    LORAC->SSP_CPSR |= 0x2;         /* set CPSR to 2, shoule be even number between 2-254 */
    LORAC->SSP_CR0 &= ~(0xff << 8); /* reset SCR to 0 */
    LORAC->SSP_CR0 |= scr << 8;     /* set SCR to 0x7, serial clk = 16M/2/(1+7) = 1M */

    /* set sclk polarity & phase */
    LORAC->SSP_CR0 &= ~(0x3 << 6); /* reset SPI clk phase/polarity setting to mode 0 */
    LORAC->SSP_CR0 |= SPI_CLK_POLARITY_LOW | SPI_CLK_PHASE_1EDGE;

    /* set data size */
    LORAC->SSP_CR0 &= ~(0xf); /* reset data size to 0 */
    LORAC->SSP_CR0 |= SSP_DATA_SIZE_8BIT;

    LORAC->SSP_CR1 &= ~(0x1 << 2); /* reset master/slave select to 0, which is master mode */

    /* dma handshake config,
    should be enabled after dmac has been configured and ready */
    LORAC->SSP_DMA_CR &= ~SSP_DMA_TX_EN;
    LORAC->SSP_DMA_CR &= ~SSP_DMA_RX_EN;
	LORAC->SSP_CR1 |= (0x1 << 1); //enable LORA SPI
	
}

void SX126XHal::init()
{
    //printf("SX126X_Hal Init\r\n");
	
	LORAC->CR0 = 0x00000200;
    lorac_ssp_init();

    //enable LORA_IRQHandler wakeup
    TREMO_REG_WR(0x40001804,TREMO_REG_RD(0x40001804)|0x80);
    //NVIC_SetPriority(LORAC_IRQn, 2);
    pinMode(CONFIG_LORA_RFSW_VDD_PIN,OUTPUT);
    digitalWrite(CONFIG_LORA_RFSW_VDD_PIN,1);
	pinMode(10,OUTPUT);
    digitalWrite(10,1);
    iomux(CONFIG_LORA_RFSW_CTRL_PIN, 3);
    //iomux(60, 3);
    //iomux(58, 6);	
	
	lora_callbacks[0] = dioISR;
}

void SX126XHal::reset(void)
{
    //printf("SX126X Reset\r\n");

	LORAC->CR1 = 0x80;
    LORAC->CR1 |= 1<<5;  //nreset
    LORAC->CR1 &= ~(1<<7); //por
    //LORAC->CR0 |= 1<<5; //irq0
    //LORAC->CR1 |= 0x1;  //tcxo

    while((LORAC->SR & 0x100));  
	
    LORAC->NSS_CR = 0;
    delayMicroseconds(10);

    SpiInOut( SX126X_RADIO_GET_STATUS );
    uint8_t stat = SpiInOut( 0x00 );

    LORAC->NSS_CR = 1;
	
	if (stat == 0xA2) {}
		//printf("SX126X Ready with status = 0x%x\r\n", stat);
	else {
		//printf("Critical! SX1262 not found!\r\n");
		while(1);
	}
    NVIC_EnableIRQ(LORA_IRQn);
    NVIC_SetPriority(LORA_IRQn,2);
	LORAC->CR0 |= 1<<5; //irq0
    LORAC->CR1 |= 0x1;  //tcxo
}

void SX126XHal::WriteCommand(SX126X_RadioCommands_t command, uint8_t val)
{
    WaitOnBusy();
    LORAC->NSS_CR = 0;

    SpiInOut( ( uint8_t )command );

    SpiInOut( val );

    LORAC->NSS_CR = 1;
	WaitOnBusy( );

}

void SX126XHal::WriteCommand(SX126X_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
	WaitOnBusy();
	
    LORAC->NSS_CR = 0;

    SpiInOut( ( uint8_t )command );

    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    LORAC->NSS_CR = 1;
	WaitOnBusy( );
}

void SX126XHal::ReadCommand(SX126X_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{

    WaitOnBusy();

    LORAC->NSS_CR = 0;

    SpiInOut( ( uint8_t )command );
    SpiInOut( 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    LORAC->NSS_CR = 1;
	WaitOnBusy( );
}

void SX126XHal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
	WaitOnBusy();
	
    LORAC->NSS_CR = 0;
    
    SpiInOut( SX126X_RADIO_WRITE_REGISTER );
    SpiInOut( ( address & 0xFF00 ) >> 8 );
    SpiInOut( address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    LORAC->NSS_CR = 1;
	WaitOnBusy( );
}

void SX126XHal::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegister(address, &value, 1);
}

void SX126XHal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
	WaitOnBusy();
	
    LORAC->NSS_CR = 0;

    SpiInOut( SX126X_RADIO_READ_REGISTER );
    SpiInOut( ( address & 0xFF00 ) >> 8 );
    SpiInOut( address & 0x00FF );
    SpiInOut( 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }
    LORAC->NSS_CR = 1;
	WaitOnBusy( );
}

uint8_t SX126XHal::ReadRegister(uint16_t address)
{
    uint8_t data;
    ReadRegister(address, &data, 1);
    return data;
}

void SX126XHal::WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();

    LORAC->NSS_CR = 0;

    SpiInOut( SX126X_RADIO_WRITE_BUFFER );
    SpiInOut( offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }
    LORAC->NSS_CR = 1;
	WaitOnBusy( );
}

void SX126XHal::ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
	WaitOnBusy();
	
    LORAC->NSS_CR = 0;

    SpiInOut( SX126X_RADIO_READ_BUFFER );
    SpiInOut( offset );
    SpiInOut( 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }
    LORAC->NSS_CR = 1;
	WaitOnBusy( );
}

bool SX126XHal::WaitOnBusy()
{
    int n = 0;
    while( LORAC->SR & 0x100 )
    {
        n++;
        if(n>=10000)
        {
           //printf("spi busy\r\n");
		   return false;
        }
    }
	return true;
}

void SX126XHal::dioISR()
{
    if (instance->IsrCallback)
        instance->IsrCallback();
}

void SX126XHal::TXenable()
{

}

void SX126XHal::RXenable()
{

}

void SX126XHal::TXRXdisable()
{
}

extern "C" {
void LORA_IRQHandler()
{
	//printf("Lora irq\r\n");
    if (lora_callbacks[0])
		lora_callbacks[0]();
}
}