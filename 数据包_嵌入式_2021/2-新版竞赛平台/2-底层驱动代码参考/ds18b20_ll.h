#ifndef __DS18B20_LL_H
#define __DS18B20_LL_H

#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"

#define OW_PIN_CLOCK	LL_AHB2_GRP1_PERIPH_GPIOA
#define OW_PIN_PORT		GPIOA
#define OW_PIN		    LL_GPIO_PIN_6


#define OW_DIR_OUT() 	 mode_output1()
#define OW_DIR_IN() 	 mode_input1()
#define OW_OUT_LOW() 	 (LL_GPIO_ResetOutputPin(OW_PIN_PORT,OW_PIN))
#define OW_GET_IN()  	 (LL_GPIO_IsInputPinSet(OW_PIN_PORT,OW_PIN))
#define OW_SKIP_ROM 		0xCC
#define DS18B20_CONVERT 	0x44
#define DS18B20_READ 		0xBE

void ds18b20_init_x(void);
s16 ds18b20_read(void);


#endif

