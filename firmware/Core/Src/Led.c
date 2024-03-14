/*
 * Led.c
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */
#include "Led.h"

// Variable globale pour stocker la luminosité actuelle de la LED
uint8_t brightness = 0;

// Démarre le timer
void LedStart(void)
{
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
}
// Configure le rapport cyclique de la PWM entre 0 et 255
void LedSetValue(uint8_t val)
{
	LL_TIM_OC_SetCompareCH1(TIM2, val);
}
// À chaque appel, cette fonction incrémente la luminosité de la LED
// Arrivé à la valeur maximale, chaque appel décrémente la LED
void LedPulse() {
    static uint8_t direction = 1; // 1 pour incrémenter, -1 pour décrémenter

    // Incrémente ou décrémente la luminosité en fonction de la direction
    brightness += direction;

    // Si la luminosité atteint la valeur maximale, change de direction
    if (brightness >= MAX_BRIGHTNESS) {
        direction = -1; // Change de direction pour décrémenter
    }

    // Si la luminosité atteint 0, change de direction
    if (brightness <= 0) {
        direction = 1; // Change de direction pour incrémenter
    }

    // Vous pouvez ajouter ici le code pour contrôler la LED avec la luminosité actuelle
    LedSetValue(brightness);
}
