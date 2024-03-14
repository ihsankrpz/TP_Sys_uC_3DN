/*
 * Led.h
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#define MAX_BRIGHTNESS 255

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


// Variable globale pour stocker la luminosité actuelle de la LED
extern uint8_t brightness;

// Démarre le timer
extern void LedStart(void);
// Configure le rapport cyclique de la PWM entre 0 et 255
extern void LedSetValue(uint8_t val);
// À chaque appel, cette fonction incrémente la luminosité de la LED
// Arrivé à la valeur maximale, chaque appel décrémente la LED
extern void LedPulse(void);

#endif /* INC_LED_H_ */
