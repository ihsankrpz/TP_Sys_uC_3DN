/*
 * TimeBase.c
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */

#include "TimeBase.h"

//start tim2 ch2
void TimeBaseStartIT(void)
{
	LL_TIM_EnableIT_UPDATE(TIM21);
	LL_TIM_EnableCounter(TIM21);
}



