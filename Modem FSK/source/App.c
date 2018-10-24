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
#include "Modem.h"
#include "GPIO.h"
#include "DMAMUX.h"
#include "hardware.h"//SACA LA MANO DE AHI APP.C!!!!
#include "FTM.h"
#include "DMA.h"
#include "PIT.h"


/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////


void DAC0_IRQHandler()
{
	//digitalToggle(PORTNUM2PIN(PC,10));

	//static uint8_t index = 0;

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

//uint8_t srcARR[] = {1,2,3,4,5};
//uint8_t destARR[5]={0,0,0,0,0};

#define PI               3.14159265358979f
#include "math.h"


#define BIT_FREC 1200
#define PWM_FREC 98400
uint16_t CnVTableL[BIT_FREC/PWM_FREC];
uint16_t CnVTableH[BIT_FREC/PWM_FREC];
void App_Init (void)
{

	/*------------------------------------------------------------------------*/
	/*primero seteo dma*/

	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMA_Init(&DMAconfig);



	DMA_TransferConfig DMATransfer;
	DMATransfer.sourceAddress = (uint32_t)CnVTableL;
	DMATransfer.destinationAddress = FTM_GetCnVAddress(0,0);
	DMATransfer.destinationOffset = 0;
	DMATransfer.sourceOffset = 1;
	DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.majorLoopCounts = BIT_FREC/PWM_FREC;//5
	DMATransfer.minorLoopBytes = 1;

	DMA_SetTransferConfig(1,&DMATransfer);
	//DMA_EnableInterrupts(1);
	DMA_EnableChannelRequest (1);



	DMAMUX_SetSource(1,DMAMUX_FTM0_CH0);
	DMAMUX_EnableChannel(1,false);


	/*despues configuro ftm(y le activo dma requests)*/



	//PORT_Config portConf={PORT_LockRegister ,PORT_MuxAlt3,PORT_PullDisable,PORT_FastSlewRate,PORT_OpenDrainDisable,PORT_PassiveFilterDisable, PORT_LowDriveStrength, PORT_InterruptOrDMADisabled };
	//PORT_PinConfig (PORT_A,0,&portConf);
	PORTC->PCR[1] =  PORT_PCR_MUX(4); //PTC1  Alt4 FTM0_CH0
	//FTM PWM Test
	FTM_Config config;
	config.clockSource = FTM_SYSTEM_CLOCK;
	config.prescale = FTM_PRESCALE_1;
	FTM_Init(FTM_0,&config);

	FTM_PwmConfig PWMConfig;
	PWMConfig.channel = FTM_CHNL_0;
	PWMConfig.mode = FTM_PWM_CENTER_ALIGNED;
	PWMConfig.enableDMA=false;
	PWMConfig.PWMFreq = 1000;
	PWMConfig.dutyCyclePercent=30;

	FTM_SetupPwm(FTM_0,&PWMConfig);
/*----------------------------------------------------------------------------*/
	//PORTA->PCR[0] =  PORT_PCR_MUX(3); //PTA0  Alt3 FTM0_CH5

	/*
	// Fill table with samples
	for(int i=0; i<N_SAMPLE; i++)
		signal[i]=sin((float)i/N_SAMPLE*2*M_PI)*2048+2047;
	*/
	/*
  	//DMA + PIT
	DMAMUX_Init();



	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMA_Init(&DMAconfig);



	DMA_TransferConfig DMATransfer;
	DMATransfer.sourceAddress = (uint32_t)srcARR;
	DMATransfer.destinationAddress = (uint32_t)destARR;
	DMATransfer.destinationOffset = 1;
	DMATransfer.sourceOffset = 1;
	DMATransfer.destinationTransferSize = DMA_TransferSize1Bytes;
	DMATransfer.sourceTransferSize = DMA_TransferSize1Bytes;
	DMATransfer.majorLoopCounts = 5;//5
	DMATransfer.minorLoopBytes = 1;

	DMA_SetTransferConfig(1,&DMATransfer);
	DMA_EnableInterrupts(1);
	DMA_EnableChannelRequest (1);



	DMAMUX_SetSource(1,DMAMUX_AlwaysEnabled3);
	DMAMUX_EnableChannel(1,true);



	PIT_Config PITConfig;
	PIT_GetDefaultConfig(&PITConfig);
	PIT_Init(&PITConfig);
	PIT_Enable();
	PIT_SetTimerPeriod (PIT_CHNL_1, 0xFFFFFF);
	//PIT_TimerIntrruptEnable(PIT_CHNL_1, true); // Probar comentar
	PIT_TimerEnable(PIT_CHNL_1, true);
*/

	/*
	// Configure PDB module
	PDB_Config PDBconfig;
	PDBconfig.enableContinuousMode = true;
	PDBconfig.loadValueMode = PDB_ON_COUNTER_OVERFLOW;
	PDBconfig.multiplicationFactor = PDB_MULT_FACTOR_1;
	PDBconfig.prescalerDivider = PDB_PRESCALER_DIVIDER_64;
	PDBconfig.triggerInputSource = PDB_TRIGGER_SOFTWARE;
	PDB_Init(&PDBconfig);
	PDB_SetModulusValue((int)(50000000.0/(N_SAMPLE*SINE_FREQ)));
	PDB_SetDacTriggerPeriod(355);
	PDB_EnableDacTriger(true);


	// Configure DAC buffer
	DAC_Init(DAC_0,DAC_VREF_2);

	DAC_BufferConfig DACBufferConfig;
	DACBufferConfig.triggerMode = DAC_HARDWARE_TRIGGER;
	DACBufferConfig.upperLimit = DAC_BUFFER_SIZE-1;
	DACBufferConfig.watermark = DAC_WATERMARK_2_WORD;
	DACBufferConfig.workMode = DAC_MODE_NORMAL;//DAC_MODE_ONE_TIME;
	//DAC_SetBufferConfig(DAC_0,&DACBufferConfig);

	DAC_EnableInterrupts(DAC_0,DAC_INTERRUPT_POINTER_TOP);

	pinMode(PIN_SW2,INPUT);
	pinMode(PORTNUM2PIN(PC,10),OUTPUT);
*/

	/*FTM_Config config;
	config.clockSource = FTM_SYSTEM_CLOCK;
	config.prescale = FTM_PRESCALE_4;
	FTM_Init(FTM_0,&config);


	FTM_PwmConfig PWMConfig;
	PWMConfig.channel = FTM_CHNL_0;
	PWMConfig.mode = FTM_PWM_EDGE_ALIGNED;
	PWMConfig.PWMFreq = 1000;

	FTM_SetupPwm(FTM_0,&PWMConfig);

	PORTC->PCR[1] =  PORT_PCR_MUX(4); //PTC1  Alt4 FTM0_CH0


	FTM_EnableOverflowInterrupt(FTM_0);*/


	MODEM_Init();

}

/*void DMA1_IRQHandler(void)
{
	int i =0;
	debugFlag=1;
	i++;
}

void DMA0_IRQHandler(void)
{
	int i =0;
	debugFlag=1;
	i++;
}*/

void App_Run (void)
{

	//PDB_Trigger();
//	while(1)
//	{
		/*if( destARR[4]==5)
		//if(debugFlag)
		{

			debugFlag=9;
		}
		//DMA_TriggerChannelStart(0);*/
//	}


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


void createCnVSineTables(uint16_t *arr1,uint16_t *arr2, uint16_t size,uint16_t mod)//size=PWM_FREC/BIT_FREC
{
	double a=0;
	for(uint16_t i; i<size; i++)
	{
		//DUTY VALUES
		a=(1+sin(2*PI*i/size))/2.0;


		arr1[i]=(uint16_t)(a*mod);
		if(arr1[i]==0)
			arr1[i]=1;
		if(arr1[i]==mod)
			arr1[i]-=1;
	}
	for(uint16_t i; i<size; i++)
	{
		a=(1+sin(2*PI*i*2/size))/2.0;
		arr2[i]=(uint16_t)(a*mod);
		if(arr2[i]==0)
			arr2[i]=1;
		if(arr2[i]==mod)
			arr2[i]-=1;
	}




}
