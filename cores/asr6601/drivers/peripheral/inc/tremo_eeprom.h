/**
  ******************************************************************************
  * @file    stm32_eeprom.h
  * @brief   Header for eeprom module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TREMO_EEPROM_H
#define __TREMO_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define _EEPROM_SIZE      1024
#define _EEPROM_BASE      FLASH_EEPROM_BASE

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t eeprom_read_byte(const uint32_t pos);
void eeprom_write_byte(uint32_t pos, uint8_t value);

#if !defined(DATA_EEPROM_BASE)
void eeprom_buffer_fill();
int eeprom_buffer_flush();
uint8_t eeprom_buffered_read_byte(const uint32_t pos);
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value);
#endif /* ! DATA_EEPROM_BASE */

#ifdef __cplusplus
}
#endif

#endif /* __STM32_EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
