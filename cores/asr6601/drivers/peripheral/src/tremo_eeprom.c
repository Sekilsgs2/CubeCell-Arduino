/**
  ******************************************************************************
  * @file    stm32_eeprom.c
  * @brief   Provides emulated eeprom from flash
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016-2021, STMicroelectronics
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "ASR_Arduino.h"
//#include "tremo_eeprom.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define DATA_EEPROM_BASE FLASH_EEPROM_BASE
//#define DATA_EEPROM_END FLASH_EEPROM_END

#define E2END 1024

static uint8_t eeprom_buffer[E2END + 1] __attribute__((aligned(8))) = {0};

/**
  * @brief  Function reads a byte from emulated eeprom (flash)
  * @param  pos : address to read
  * @retval byte : data read from eeprom
  */
uint8_t eeprom_read_byte(const uint32_t pos)
{
  eeprom_buffer_fill();
  return eeprom_buffered_read_byte(pos);
}

/**
  * @brief  Function writes a byte to emulated eeprom (flash)
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void eeprom_write_byte(uint32_t pos, uint8_t value)
{
  eeprom_buffered_write_byte(pos, value);
  eeprom_buffer_flush();
}


/**
  * @brief  Function reads a byte from the eeprom buffer
  * @param  pos : address to read
  * @retval byte : data read from eeprom
  */
uint8_t eeprom_buffered_read_byte(const uint32_t pos)
{
  return eeprom_buffer[pos];
}

/**
  * @brief  Function writes a byte to the eeprom buffer
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value)
{
  eeprom_buffer[pos] = value;
}

/**
  * @brief  This function copies the data from flash into the buffer
  * @param  none
  * @retval none
  */
void eeprom_buffer_fill(void)
{
  memcpy(eeprom_buffer, (uint8_t *)(FLASH_EEPROM_BASE), E2END + 1);
}

/**
  * @brief  This function writes the buffer content into the flash
  * @param  none
  * @retval none
  */
int eeprom_buffer_flush(void)
{
  //uint8_t data = 0;

  //memcpy(&data, eeprom_buffer, _EEPROM_SIZE);
  return FLASH_update_sdk(FLASH_EEPROM_BASE, eeprom_buffer, _EEPROM_SIZE);
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
