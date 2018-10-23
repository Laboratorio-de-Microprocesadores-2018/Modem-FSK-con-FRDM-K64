#include "Modem.h"

#include "math.h"
#include "SysTick.h"
#include "DAC.h"
#include "GPIO.h"
#include "DMAMUX.h"
#include "DMA.h"
#include "PIT.h"
#include "stdlib.h"


#define BUS_CLOCK 50000000
#define N_SAMPLE (128)
#define F1 (1100)
#define F2 (2200)
#define	T1 BUS_CLOCK/(N_SAMPLE*F1)
#define T2 BUS_CLOCK/(N_SAMPLE*F2)

static uint16_t signal[N_SAMPLE];

#define BUFFER_SIZE 20
static uint32_t OUT_BITS[BUFFER_SIZE];
static uint8_t head,tail;


#define MEASURE_CPU_TIME
#ifdef MEASURE_CPU_TIME
	#define MEASURE_CPU_TIME_PORT PORTC
	#define MEASURE_CPU_TIME_GPIO GPIOC
	#define MEASURE_CPU_TIME_PIN	9
#endif


void modulate(void * data)
{

#ifdef MEASURE_CPU_TIME
	BITBAND_REG(MEASURE_CPU_TIME_GPIO->PDOR, MEASURE_CPU_TIME_PIN) = 1;
#endif
	static int i=0;
	static uint16_t N[] = {T1,T2};
	if(i==5)
	{

	}
	else
	{
		PIT_SetTimerPeriod (PIT_CHNL_0, N[i]);
	}
	i = (i+1)%2;
#ifdef MEASURE_CPU_TIME
	BITBAND_REG(MEASURE_CPU_TIME_GPIO->PDOR, MEASURE_CPU_TIME_PIN) = 0;
#endif
}


void MODEM_Init()
{
#ifdef MEASURE_CPU_TIME
	MEASURE_CPU_TIME_PORT->PCR[MEASURE_CPU_TIME_PIN] = PORT_PCR_MUX(1);
	MEASURE_CPU_TIME_GPIO->PDDR |= (1<<MEASURE_CPU_TIME_PIN);
	MEASURE_CPU_TIME_GPIO->PDOR &= ~(1<<MEASURE_CPU_TIME_PIN);
#endif

	// Fill table with samples
	for(int i=0; i<N_SAMPLE; i++)
	{
		uint16_t s = sin((float)i/(N_SAMPLE)*2*M_PI)*2048+2047;
		signal[i]= s;
	}
/*
	// DMA0 AlwaysEnabled
	DMAMUX_Init();
	DMAMUX_SetSource(0,DMAMUX_AlwaysEnabled3);
	DMAMUX_EnableChannel(0,true);

	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMAconfig.enableDebugMode=true;
	DMA_Init(&DMAconfig);


	DMA_TransferConfig DMATransfer;
	DMATransfer.sourceAddress = (uint32_t)signal;
	DMATransfer.destinationAddress = (uint32_t)DAC_GetBufferAddress(DAC_0);
	DMATransfer.destinationOffset = 0;
	DMATransfer.sourceOffset = 2;
	DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.majorLoopCounts = N_SAMPLE;
	DMATransfer.minorLoopBytes = 2;
	DMATransfer.majorLoopAdjust = -1*sizeof(signal);
	DMA_SetTransferConfig(0,&DMATransfer);
	DMA_EnableChannelRequest (0);


	DAC_Init(DAC_0,DAC_VREF_2);
	DAC_Enable(DAC_0);
*/
	ADC_Init();

/*
	PIT_Config PITConfig;
	PITConfig.debugModeEnable=true;
	PIT_Init(&PITConfig);
	PIT_Enable();

    PIT_SetTimerPeriod (PIT_CHNL_0, T1);
	PIT_TimerIntrruptEnable(PIT_CHNL_0, true); // Probar comentar
	PIT_TimerEnable(PIT_CHNL_0, true);
*/
	// Set periodic interrupt to modulate
	//PIT_SetTimerPeriod (PIT_CHNL_1, 41666);
	//PIT_TimerIntrruptEnable(PIT_CHNL_1, true);
	//PIT_SetTimerIntrruptHandler(PIT_CHNL_1,&modulate, NULL);
	//PIT_TimerEnable(PIT_CHNL_1, true);

}

void MODEM_SendByte(uint8_t byte)
{

}

uint8_t MODEM_ReceiveByte()
{

}
