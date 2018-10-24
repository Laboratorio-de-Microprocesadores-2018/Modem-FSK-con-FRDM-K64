/////////////////////////////////////////////////////////////////////////////////
//                     				                                    	   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////


#ifndef FTM_H_
#define FTM_H_


#include <stdbool.h>
#include "stdint.h"
// Los que tienen * son los que se pueden acceder en el header
/*

PTA1 Alt3: FTM0_CH6
PTA2 Alt3: FTM0_CH7

PTB2 Alt6 FTM0_FLT3
PTB3 Alt6 FTM0_FLT0
PTB10 Alt6 FTM0_FLT1
PTB11 Alt6 FTM0_FLT2
PTB18 Alt3 FTM2_CH0 Alt6: FTM2_QD_PHA
PTB19 Alt3 FTM2_CH1 Alt6: FTM2_QD_PHB

PTC1  Alt4 FTM0_CH0
PTC2  Alt4 FTM0_CH1
PTC3  Alt4 FTM0_CH2
PTC4  Alt4 FTM0_CH3
PTC5  Alt7 FTM0_CH2
PTC8  Alt3 FTM3_CH4
PTC9  Alt4 FTM3_CH5 Alt6: FTM2_FLT0
PTC10 Alt3 FTM3_CH6
PTC11 Alt4 FTM3_CH7
PTC12 Alt6 FTM3_FLT0

PTD0  Alt4 FTM3_CH0
PTD1  Alt4 FTM3_CH1
PTD2  Alt4 FTM3_CH2
PTD3  Alt4 FTM3_CH3

*/
typedef enum{FTM_0,FTM_1,FTM_2,FTM_3}FTM_Instance;

typedef enum{FTM_NO_CLOCK,
			FTM_SYSTEM_CLOCK,
			FTM_FIXED_FREQ,
			FTM_EXTERNAL_CLOCK}FTM_ClockSource;


typedef enum{FTM_PRESCALE_1,
			FTM_PRESCALE_2,
			FTM_PRESCALE_4,
			FTM_PRESCALE_8,
			FTM_PRESCALE_16,
			FTM_PRESCALE_32,
			FTM_PRESCALE_64,
			FTM_PRESCALE_128
}FTM_Prescale;

typedef enum{
	FTM_PWM_CENTER_ALIGNED,
	FTM_PWM_EDGE_ALIGNED,
	FTM_PWM_COMBINED
}FTM_PWMMode;

typedef enum{
	FTM_CHNL_0,
	FTM_CHNL_1,
	FTM_CHNL_2,
	FTM_CHNL_3,
	FTM_CHNL_4,
	FTM_CHNL_5,
	FTM_CHNL_6,
	FTM_CHNL_7
}FTM_Channel;





typedef struct{
	FTM_Prescale prescale;
	FTM_ClockSource clockSource;
}FTM_Config;

typedef struct{
	FTM_Channel channel;
	FTM_PWMMode mode;
	bool enableDMA;
	uint32_t PWMFreq;
	uint8_t dutyCyclePercent;					//agregue a partir de esto
	//se podria agregar level para poner high true o low true pulse
}FTM_PwmConfig;

void FTM_GetDefaultConfig(FTM_Config * config);
void FTM_Init(FTM_Instance instance, FTM_Config * config);
bool FTM_SetupPwm(FTM_Instance 	instance,FTM_PwmConfig * config);
void FTM_UpdatePwmDutycycle(FTM_Instance 	instance,FTM_Channel channel,uint8_t dutyCyclePercent);//en principio no seria necesaria
uint32_t FTM_GetCnVAddress(FTM_Instance 	instance,FTM_Channel channel);
uint16_t FTM_GetModValue(FTM_Instance instance);
#endif /* FTM_H_ */
