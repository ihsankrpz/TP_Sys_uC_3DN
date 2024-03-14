/*
 * TimeBase.c
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */

#ifndef INC_TIMEBASE_H_
#define INC_TIMEBASE_H_

#include "TimeBase.h"

//start tim2 ch2
void TimeBaseStartIT(void)
{
	LL_TIM_EnableIT_UPDATE(TIM21);
	LL_TIM_EnableCounter(TIM21);
}

#endif /*INC_TIMEBASE_H_*/


