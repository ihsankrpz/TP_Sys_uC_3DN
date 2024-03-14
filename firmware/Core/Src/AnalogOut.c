/*
 * AnalogOut.c
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */

#include "AnalogOut.h"

// Initialisation du DAC
void AnalogOutInit(void)
{
    // Activation du DAC
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
}

// Conversion d'une valeur numérique en une tension analogique et envoi au DAC
void AnalogOutConvert(uint16_t value)
{
    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, value);
}

// Génération d'un signal triangle avec le DAC
void AnalogOutPulse(uint16_t increment)
{
    static uint16_t value = 0;
    static uint8_t direction = 0; // 0 pour incrémenter, 1 pour décrémenter

    if (direction == 0) {
        value += increment;
        if (value >= 4095) {
            direction = 1;
        }
    } else {
        value -= increment;
        if (value == 0) {
            direction = 0;
        }
    }

    AnalogOutConvert(value);
}
