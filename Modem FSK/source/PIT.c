#include "PIT.h"
#include "MK64F12.h"
#include "MK64F12_features.h"
#include "Assert.h"
#include "stdlib.h"

static PIT_Callback PIT_Callbacks[FSL_FEATURE_PIT_TIMER_COUNT];
static  void  * PIT_CallbackData[FSL_FEATURE_PIT_TIMER_COUNT];

void PIT_GetDefaultConfig(PIT_Config * config)
{
	ASSERT(config != NULL);

	config->debugModeEnable = false;
}

void PIT_Init(PIT_Config * config)
{
	ASSERT(config != NULL);

	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
	PIT->MCR |= PIT_MCR_MDIS_MASK;

	if(config->debugModeEnable == true)
		PIT->MCR |= PIT_MCR_FRZ_MASK;
}

void PIT_SetTimerPeriod (PIT_Channel n, uint32_t count)
{
	ASSERT(n<FSL_FEATURE_PIT_TIMER_COUNT);

	PIT->CHANNEL[n].LDVAL = count;
}
 
uint32_t PIT_GetTimerCount (PIT_Channel n)
{
	ASSERT(n<FSL_FEATURE_PIT_TIMER_COUNT);

	return PIT->CHANNEL[n].CVAL;
}

void PIT_TimerEnable(PIT_Channel n, bool enable)
{
	ASSERT(n<FSL_FEATURE_PIT_TIMER_COUNT);

	if(enable)
		PIT->CHANNEL[n].TCTRL |= PIT_TCTRL_TEN_MASK;
	else
		PIT->CHANNEL[n].TCTRL &= ~PIT_TCTRL_TEN_MASK;
}

void PIT_TimerReset(PIT_Channel n)
{
	PIT->CHANNEL[n].TCTRL &= ~PIT_TCTRL_TEN_MASK;
	PIT->CHANNEL[n].TCTRL |= PIT_TCTRL_TEN_MASK;
}


void PIT_SetTimerIntrruptHandler(PIT_Channel n, PIT_Callback callback, void * data)
{
	ASSERT(n<FSL_FEATURE_PIT_TIMER_COUNT);

	PIT_Callbacks[n] = callback;
	PIT_CallbackData[n] = data;
}

void PIT_TimerIntrruptEnable(PIT_Channel n, bool enable)
{
	ASSERT(n<FSL_FEATURE_PIT_TIMER_COUNT);

	if(enable)
	{
		PIT->CHANNEL[n].TFLG = 1;
		PIT->CHANNEL[n].TCTRL |= PIT_TCTRL_TIE_MASK;
	}
	else
		PIT->CHANNEL[n].TCTRL &= ~PIT_TCTRL_TIE_MASK;
}


void PIT_ChainMode(PIT_Channel n,bool enable)
{
	ASSERT(n<FSL_FEATURE_PIT_TIMER_COUNT);

	if(enable)
		PIT->CHANNEL[n].TCTRL |= PIT_TCTRL_CHN_MASK;
	else
		PIT->CHANNEL[n].TCTRL &= ~PIT_TCTRL_CHN_MASK;
}

void PIT0_IRQHandler(void)
{
	ASSERT(PIT_Callbacks[0] != NULL);

	PIT_Callbacks[0](PIT_CallbackData[0]);
	PIT->CHANNEL[0].TFLG = 1;
}
void PIT1_IRQHandler(void)
{
	ASSERT(PIT_Callbacks[1] != NULL);

	PIT_Callbacks[1](PIT_CallbackData[1]);
	PIT->CHANNEL[1].TFLG = 1;
}
void PIT2_IRQHandler(void)
{
	ASSERT(PIT_Callbacks[2] != NULL);

	PIT_Callbacks[2](PIT_CallbackData[2]);
	PIT->CHANNEL[2].TFLG = 1;
}
void PIT3_IRQHandler(void)
{
	ASSERT(PIT_Callbacks[3] != NULL);

	PIT_Callbacks[3](PIT_CallbackData[3]);
	PIT->CHANNEL[3].TFLG = 1;
}

