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
#define N_SAMPLE (256)
#define F1 (1100)
#define F2 (2200)
#define	T1 BUS_CLOCK/(N_SAMPLE*F1)
#define T2 BUS_CLOCK/(N_SAMPLE*F2)
#define MARK T1
#define SPACE T2
#define BIT2PERIOD(x) (x)==1? MARK:SPACE

#define DAC_DMA_CHANNEL 0
#define ADC_DMA_CHANNEL 1

static uint16_t signal[N_SAMPLE];

// Generating the look-up table while pre-processing
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
#define LOOK_UP P6(0), P6(1), P6(1), P6(0)

// LOOK_UP is the macro expansion to generate the table
static unsigned int parityTable[256] = { LOOK_UP };


#define bufferSizeDataBytes (30)
#define frameSizeBits (11)
#define bufferSize bufferSizeDataBytes*frameSizeBits
#define bufferEmpty (head==tail)
#define bufferFull	(tail+1==head)
static uint32_t outcomingBits[bufferSize];
static uint8_t head,tail;

static uint16_t ADCSamples[11];
static uint8_t inputBytes[10];


#define MEASURE_CPU_TIME
#ifdef MEASURE_CPU_TIME
	#define MEASURE_CPU_TIME_PORT PORTC
	#define MEASURE_CPU_TIME_GPIO GPIOC
	#define MEASURE_CPU_TIME_PIN	9
	#define SET_TEST_PIN BITBAND_REG(MEASURE_CPU_TIME_GPIO->PDOR, MEASURE_CPU_TIME_PIN) = 1
	#define CLEAR_TEST_PIN BITBAND_REG(MEASURE_CPU_TIME_GPIO->PDOR, MEASURE_CPU_TIME_PIN) = 0
#else
	#define SET_TEST_PIN
	#define CLEAR_TEST_PIN
#endif


void modulate(void * data)
{
	SET_TEST_PIN;

	// If no data in buffer, idle state is MARK
	if(bufferEmpty)
		PIT_SetTimerPeriod (PIT_CHNL_0, MARK);
	else
	{
		PIT_SetTimerPeriod (PIT_CHNL_0, outcomingBits[tail]);
		tail = (tail + 1)%bufferSize;
	}

	CLEAR_TEST_PIN;
}

void deModulate()
{
	uint8_t m[SAMPLE_NUM];
	k = ceil((446e-6)/(1/fs));
	for(int i = 0; i<n; i++)
	{
		m(i) =  x(i) * x(i-k);
	}
	err = FSKLPFilter(m, h, d);
	if(~err)
	{
		comparator(d,out);
	}else
	{
		return;
	}


}

bool FSKLPFilter(uint8_t m[SAMPLE_NUM], uint8_t d[SAMPLE_NUM])
{
	for(int i = 0; i<SAMPLE_NUM; i++)
	{
		d(i) = convolution(x, h);
	}
}

uint8_t convolution(uint8_t x[],uint8_t h[])
{
	uint8_t y = 0;
	if(sizeof(x) == sizeof(h))
	{
		err = false;
		n = sizeof(h);
		for (int i=0; i<n; i++)
			y = y + x(i)*h(n - i);
	}
	else{
		err = true;
	}
	return y;
}

void comparator(uint8_t x[SAMPLE_NUM], uint8_t out[BITS_NUM])
{
	for(int i=0; i<SAMPLE_NUM; i++)
	{
	    uint8_t newVal = x(i);
	    if (newVal > HYSTERESIS)
	        out(i) = 1;
	    else if (newVal < -HYSTERESIS)
	        out(i) = 0;
	}
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

	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMAconfig.enableDebugMode=false;
	DMA_Init(&DMAconfig);
	DMAMUX_Init();

    // Configure DMA0 to copy from sine table to DAC
	DMAMUX_SetSource(DAC_DMA_CHANNEL,DMAMUX_AlwaysEnabled3);
	DMAMUX_EnableChannel(DAC_DMA_CHANNEL,true);

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
	DMA_SetTransferConfig(DAC_DMA_CHANNEL,&DMATransfer);
	DMA_EnableChannelRequest (DAC_DMA_CHANNEL);

	 // Configure DMA1 to copy from ADC to buffer

	 DMAMUX_Init();
	 DMAMUX_SetSource(1,DMAMUX_ADC0);

	 DMATransfer.sourceAddress = (uint32_t)ADC_GetDataResultAddress(ADC_0);
	 DMATransfer.sourceOffset = 0;
	 DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;

	 DMATransfer.destinationAddress = (uint32_t)ADCSamples;
	 DMATransfer.destinationOffset = 2;
	 DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;

	 DMATransfer.majorLoopCounts = sizeof(ADCSamples)/sizeof(ADCSamples[0]);
	 DMATransfer.minorLoopBytes = 2;
	 DMATransfer.majorLoopAdjust = -1*sizeof(ADCSamples);

	 DMA_SetTransferConfig(ADC_DMA_CHANNEL,&DMATransfer);
	 DMA_EnableChannelRequest (ADC_DMA_CHANNEL);
	 DMAMUX_EnableChannel(ADC_DMA_CHANNEL,false);


	DAC_Init(DAC_0,DAC_VREF_2);
	DAC_Enable(DAC_0);


	PIT_Config PITConfig;
	PITConfig.debugModeEnable=true;
	PIT_Init(&PITConfig);
	PIT_Enable();

    PIT_SetTimerPeriod (PIT_CHNL_0, T1);
	PIT_TimerIntrruptEnable(PIT_CHNL_0, true); // Probar comentar
	PIT_TimerEnable(PIT_CHNL_0, true);

	// Set periodic interrupt to modulate
	PIT_SetTimerPeriod (PIT_CHNL_1, 41666);
	PIT_TimerIntrruptEnable(PIT_CHNL_1, true);
	PIT_SetTimerIntrruptHandler(PIT_CHNL_1,&modulate, NULL);
	PIT_TimerEnable(PIT_CHNL_1, true);

}

void MODEM_SendData(uint8_t data)
{
	SET_TEST_PIN;

	// Disable interrupts while adding data to output buffer
	PIT_TimerIntrruptEnable(PIT_CHNL_1, false);

	// Start bit
	outcomingBits[head]=BIT2PERIOD(0);
	head = (head + 1)%bufferSize;

	// Data bits
	for(int i=0; i<8; i++)
	{
		outcomingBits[head]=BIT2PERIOD( (data>>i)&(1) );
		head = (head + 1)%bufferSize;
	}

	// Parity bit
	outcomingBits[head]=BIT2PERIOD(parityTable[data]);
	head = (head + 1)%bufferSize;

	// Stop bit
	outcomingBits[head]=BIT2PERIOD(1);
	head = (head + 1)%bufferSize;

	// Enable interrupts
	PIT_TimerIntrruptEnable(PIT_CHNL_1, true);

	CLEAR_TEST_PIN;
}

bool MODEM_ReceiveData(uint8_t * buffer, uint8_t * length)
{



}

