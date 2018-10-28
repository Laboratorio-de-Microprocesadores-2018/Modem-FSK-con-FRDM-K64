/////////////////////////////////////////////////////////////////////////////////
//                     				                                    	   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////


#ifndef FTM_H_
#define FTM_H_
/**
 * @file     FTM.h
 * @brief    FTM driver to control the FTM module for PWM generation, counting
 * and for input capture
 */
/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include "stdint.h"

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////
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
typedef void (*FTMCaptureFun_t)(uint16_t);
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
	FTM_CHNL_0,	FTM_CHNL_1,	FTM_CHNL_2,	FTM_CHNL_3,	FTM_CHNL_4,
	FTM_CHNL_5,	FTM_CHNL_6,	FTM_CHNL_7
}FTM_Channel;


typedef enum{
	FTM_RISING_EDGE=1, //Capture on rising edge only.
	FTM_FALLING_EDGE,//Capture on falling edge only.
	FTM_BOTH_EDGES //Capture on rising or falling edge.
}FTM_InputCaptureMode;



typedef struct{
	FTM_Prescale prescale;
	FTM_ClockSource clockSource;
}FTM_Config;

typedef struct{
	FTM_Channel channel;
	FTM_PWMMode mode;
	bool enableDMA;
	uint16_t mod;
	uint8_t CnV;

}FTM_PwmConfig;

typedef struct {
	FTM_Channel channel;
	FTMCaptureFun_t callback;
	uint16_t mod;
	bool enableDMA;
	FTM_InputCaptureMode mode;
	uint32_t filterValue;//Only channels 0,1,2 and 3 can have filter
}FTM_InputCaptureConfig;
/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Gives a default configuration for the FTM module
 * @param config struct where the function will return the default configuration
 */
void FTM_GetDefaultConfig(FTM_Config * config);
/**
 * @brief Initializes FTM module
 * @param instance to be used
 * @param configuration struct
 */
void FTM_Init(FTM_Instance instance, FTM_Config * config);
/**
 * @brief configuration function for PWM generation
 * @param instance FTM instance used
 * @param config , configuration structure
 */
bool FTM_SetupPwm(FTM_Instance 	instance,FTM_PwmConfig * config);

/**
 * @brief Enables clock for the FTM instance selected
 * @param instance
 */
void FTM_EnableClock(FTM_Instance instance);
/**
 * @brief configuration function for input capture
 * @param instance FTM instance used
 * @param config , configuration structure
 */
bool FTM_SetupInputCapture(FTM_Instance instance,FTM_InputCaptureConfig *config);
/**
 * @brief Gives a pointer to the address of the CnV register for the specified instance
 * and channel.
 * @param instance FTM instance used
 * @param channel
 *  */
uint32_t FTM_GetCnVAddress(FTM_Instance 	instance,FTM_Channel channel);
uint16_t FTM_GetModValue(FTM_Instance instance);
/**
 * @brief Sets to 0 the count of the selected FTM instance
 * @param instance FTM instance used
 *  */
void FTM_ClearCount(FTM_Instance instance);

void FTM_EnableOverflowInterrupt(FTM_Instance instance);

void FTM_EnableInterrupts(FTM_Instance 	instance,FTM_Channel channel);

void FTM_DisableInterrupts(FTM_Instance 	instance,FTM_Channel channel);
#endif /* FTM_H_ */
