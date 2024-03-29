/*
 * AnalogOut.h
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */

#ifndef INC_ANALOGOUT_H_
#define INC_ANALOGOUT_H_

#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

extern void AnalogOutInit(void);
extern void AnalogOutConvert(uint16_t value);
extern void AnalogOutPulse(uint16_t increment);

#endif /* INC_ANALOGOUT_H_ */
