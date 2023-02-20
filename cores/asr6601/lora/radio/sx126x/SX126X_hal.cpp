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
	LORAC->SSP_CR1 &= ~(0x1 << 1);
	lora_callbacks[0] = nullptr;
    IsrCallback = nullptr; // remove callbacks
}

uint16_t SpiInOut( uint16_t outData , bool read=true)
{
    uint8_t read_data = 0;
	//bool tempIrq = BoardDisableIrq( );
	
	//while((LORAC->SSP_SR & SSP_FLAG_BUSY) == 1);
	LORAC->SSP_DR = outData & 0xff;
	while( (LORAC->SSP_SR & SSP_FLAG_TX_FIFO_EMPTY) == 0);
	while ((LORAC->SSP_SR & SSP_FLAG_RX_FIFO_NOT_EMPTY) == 0);
	
	read_data = LORAC->SSP_DR & 0xFF;

    return( read_data );
}

uint16_t SpiOut( volatile uint8_t *outData, size_t size )
{
    uint8_t ret = size;
	bool tempIrq = BoardDisableIrq( );
	
	while( ((LORAC->SSP_SR & SSP_FLAG_TX_FIFO_NOT_FULL) == 1) && (size > 0)) {
		LORAC->SSP_DR = *outData & 0xff;
		outData++;
		size--;
	}
	if (size == 0 )
		return ret;
	else {
		while((LORAC->SSP_SR & SSP_FLAG_BUSY) == 1);
		while( ((LORAC->SSP_SR & SSP_FLAG_TX_FIFO_NOT_FULL) == 1) && (size > 0)) {
		LORAC->SSP_DR = *outData & 0xff;
		outData++;
		size--;
	}
	}
	return ret;
}

void SX126XHal::init()
{
	LORAC->SSP_CR1 &= ~(0x1 << 1); //disable LORA SPI
	LORAC->CR0 = 0x00000200; //???
    LORAC->SSP_CR0 = 0x07;  //CR0 - bit 8-15 - speed devision - Max speed ??? Dont know real clk source for SSP bus
    LORAC->SSP_CPSR = 0x02; // prescaler : 2-254
	LORAC->SSP_CR1 |= (0x1 << 1); //enable LORA SPI

    //enable LORA_IRQHandler wakeup
    TREMO_REG_WR(0x40001804,TREMO_REG_RD(0x40001804)|0x80);
    //NVIC_SetPriority(LORA_IRQn, 254);
    gpio_set_iomux(GPIOD, CONFIG_LORA_RFSW_CTRL_PIN, 3);
	gpio_init(GPIOA, CONFIG_LORA_RFSW_VDD_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
	lora_callbacks[0] = dioISR;
}

void SX126XHal::reset(void)
{
	LORAC->CR1 = 0x80; //reset on
    LORAC->CR1 |= 1<<5;  //reset off
    LORAC->CR1 &= ~(1<<7); //por
    LORAC->CR0 |= 1<<5; //irq0
    LORAC->CR1 |= 0x1;  //tcxo
	

    while((LORAC->SR & 0x100));  
	
    LORAC->NSS_CR = 0;
    delayMicroseconds(10);

    SpiInOut( SX126X_RADIO_GET_STATUS );
    uint8_t stat = SpiInOut( 0x00);

    LORAC->NSS_CR = 1;
	
	if (stat != 0xA2) //chip not found!
		while(1);
	
	HAL_NVIC_SetPriority(LORA_IRQn, 6, 0);
    NVIC_EnableIRQ(LORA_IRQn);
}

void SX126XHal::WriteCommand(SX126X_RadioCommands_t command, uint8_t val)
{
    WaitOnBusy();
    LORAC->NSS_CR = 0;

    SpiInOut( ( uint8_t )command );

    SpiInOut( val );

    LORAC->NSS_CR = 1;

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
}

void SX126XHal::ReadCommand(SX126X_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{

    WaitOnBusy();

    LORAC->NSS_CR = 0;

    SpiInOut( ( uint8_t )command );
    SpiInOut( 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0);
    }

    LORAC->NSS_CR = 1;
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
        buffer[i] = SpiInOut( 0);
    }
    LORAC->NSS_CR = 1;
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
	
	uint8_t buf[2];

    LORAC->NSS_CR = 0;
	//buf[0] = SX126X_RADIO_WRITE_BUFFER;
	//buf[1] = offset;
	
    SpiInOut( SX126X_RADIO_WRITE_BUFFER );
    SpiInOut( offset );
	//SpiOut(buf,2);
	//SpiOut(buffer,size);
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }
    LORAC->NSS_CR = 1;
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
        buffer[i] = SpiInOut( 0, true );
    }
    LORAC->NSS_CR = 1;
}

bool SX126XHal::WaitOnBusy()
{
    #define wtimeoutUS 1000
    uint32_t startTime = micros();

    while ((LORAC->SR) & 0x100) // wait untill not busy or until wtimeoutUS
    {
        if ((micros() - startTime) > wtimeoutUS)
        {
            return false;
        }
        else
        {
            __NOP();
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
    if (lora_callbacks[0])
		lora_callbacks[0]();
}
}