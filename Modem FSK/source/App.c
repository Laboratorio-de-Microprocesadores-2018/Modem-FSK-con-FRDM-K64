/////////////////////////////////////////////////////////////////////////////////
//                     				MODEM FSK                              	   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file App.c
 * @brief Trabajo Practico N°3.
 */

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "DAC.h"
#include "ADC.h"
#include "math.h"
#include "SysTick.h"
#include "GPIO.h"
#include "DMAMUX.h"
#include "DMA.h"
#include "PIT.h"
/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

#define SINE_FREQ (1100)
#define N_SAMPLE (256)

#define DAC_BUFFER_SIZE (1)
#define DAC_WATERMARK (2)
/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

static uint16_t signal[N_SAMPLE];

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////

void DAC0_IRQHandler()
{
	//digitalToggle(PORTNUM2PIN(PC,10));

	static uint8_t index = 0;

	//DAC_WriteValue(DAC_0,signal[index++]);

	/*if(DAC_GetFlag(DAC_0,DAC_INTERRUPT_POINTER_TOP)==true)
	{
		for(int n=DAC_BUFFER_SIZE-DAC_WATERMARK-1;n<DAC_BUFFER_SIZE; n++)
			DAC_SetBufferValue (DAC_0, n, signal[index+n]);

		index = (index + DAC_BUFFER_SIZE)%N_SAMPLE;

		DAC_ClearFlag(DAC_0,DAC_INTERRUPT_POINTER_TOP);
	}
	else if(DAC_GetFlag(DAC_0,DAC_INTERRUPT_WATERMARK)==true)
	{
		for(int n=0;n<DAC_BUFFER_SIZE-DAC_WATERMARK-1; n++)
				DAC_SetBufferValue (DAC_0, n, signal[index+n]);

		DAC_ClearFlag(DAC_0,DAC_INTERRUPT_WATERMARK);
	}
	digitalToggle(PORTNUM2PIN(PC,10));*/

	/*for(int n=0;n<DAC_BUFFER_SIZE; n++)
		DAC_SetBufferValue (DAC_0, n, signal[index+n]);

	index = (index + DAC_BUFFER_SIZE)%N_SAMPLE;

	// Reset pointer
	//DAC_SetBufferPointer (DAC_0,0);

	DAC_ClearFlag(DAC_0,DAC_INTERRUPT_POINTER_TOP);*/


}

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

uint8_t srcARR[] = {1,2,3,4,5};
uint8_t destARR[5]={0,0,0,0,0};


void App_Init (void)
{

	DAC_Init(DAC_VREF_2);

	for(int i=0; i<N_SAMPLE; i++)
		signal[i]=sin((float)i/N_SAMPLE*2*M_PI)*2048+2047;


	sysTickInit();
	sysTickAddCallback(&updateDAC,1/(float)(N_SAMPLE*SINE_FREQ));
	ADC_Init();
	//FTM_Config config;
	//config.clockSource = FTM_SYSTEM_CLOCK;
	//config.prescale = FTM_PRESCALE_4;
	//FTM_Init(FTM_0,&config);
	//FTM_EnableOverflowInterrupt(FTM_0);


	FTM_PwmConfig PWMConfig;
	PWMConfig.channel = FTM_CHNL_0;
	PWMConfig.mode = FTM_PWM_EDGE_ALIGNED;
	PWMConfig.PWMFreq = 1000;

	FTM_SetupPwm(FTM_0,&PWMConfig);

	PORTC->PCR[1] =  PORT_PCR_MUX(4); //PTC1  Alt4 FTM0_CH0


	FTM_EnableOverflowInterrupt(FTM_0);*/

}

void DMA0_IRQHandler(void)
{
	int i =0;
	i++;
}
void App_Run (void)
{
	//PDB_Trigger();
	while(1)
	{
		//DMA_TriggerChannelStart(0);
	}


	//DAC_TriggerBuffer(DAC_0);
	//uint16_t n= 0x00F;
	//while(n--);
	/*static uint64_t debounceTime;
	static uint8_t currState, lastState;

	currState = digitalRead(PIN_SW2);

	if(currState!=lastState)
	{
		lastState = currState;
		debounceTime = millis();
	}
	else if (lastState == 1 && (millis() - debounceTime) > 25)
	{
		lastState=0;

		uint8_t pointe = DAC_GetBufferPointer(DAC_0);
	}*/


}

