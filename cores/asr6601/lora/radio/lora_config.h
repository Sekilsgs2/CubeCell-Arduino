#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tremo_gpio.h"

#define PRINT_BY_DMA

#define CONFIG_LORA_RFSW_CTRL_PIN   GPIO_PIN_59
#define CONFIG_LORA_RFSW_VDD_PIN    GPIO_PIN_45

#ifdef __cplusplus
}
#endif

#endif /* __LORA_CONFIG_H */
