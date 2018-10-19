#ifndef _PIT_H_
#define _PIT_H_


#include "stdbool.h"
#include "stdint.h"

typedef struct 
{
	bool debugModeEnable;
}PIT_Config;

typedef enum
{
	PIT_CHNL_0,
	PIT_CHNL_1,
	PIT_CHNL_2,
	PIT_CHNL_3
}PIT_Channel;

typedef void (*PIT_Callback)(void*);

/**
	@brief
*/
void PIT_GetDefaultConfig(PIT_Config * config);

/**
	@brief
*/
void PIT_Init(PIT_Config * config);


void PIT_Enable();
/**
	@brief
*/
void PIT_SetTimerPeriod (PIT_Channel n, uint32_t count);

/**
	@brief
*/
uint32_t PIT_GetTimerCount (PIT_Channel n);

/**
	@brief
*/
void PIT_TimerEnable(PIT_Channel n, bool enable);

/**
	@brief
*/
void PIT_TimerReset(PIT_Channel n);

/**
	@brief
*/
void PIT_TimerIntrruptEnable(PIT_Channel n, bool enable);

/**
	@brief
*/
void PIT_SetTimerIntrruptHandler(PIT_Channel n, PIT_Callback callback, void * data);

/**
	@brief
*/
void PIT_ChainMode(PIT_Channel n,bool enable);


#endif // _PIT_H_
