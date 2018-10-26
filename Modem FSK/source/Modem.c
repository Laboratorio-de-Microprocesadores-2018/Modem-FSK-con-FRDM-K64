#include "Modem.h"
#include "math.h"
#include "SysTick.h"
#include "ADC.h"
#include "DAC.h"
#include "GPIO.h"
#include "DMAMUX.h"
#include "DMA.h"
#include "PIT.h"
#include "PDB.h"
#include "stdlib.h"

#include "FloatBuffer.h"

#define BUS_CLOCK 50000000

#define BIT_RATE	(1200.0)

#define N_SAMPLE (256)
#define F1 (1200)
#define F2 (2200)
#define	T1 BUS_CLOCK/(N_SAMPLE*F1)
#define T2 BUS_CLOCK/(N_SAMPLE*F2)
#define MARK T1
#define SPACE T2
#define BIT2PERIOD(x) (x)==1? MARK:SPACE

// Module usage definitions
#define DAC_DMA_CHANNEL 0
#define ADC_DMA_CHANNEL 1
#define DAC_TIMER PIT_CHNL_0
#define MODULATION_TIMER PIT_CHNL_1

// DAC output samples
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
static uint8_t outcomingBits[bufferSize];
static uint8_t head,tail;

//static uint16_t ADCSamples[11];
static uint8_t inputBytes[10];


static const float h[] = {0.000184258321387766,	-0.00221281271600225,	-0.00875721735248610,	-0.0157935638369741,	-0.0125404257819552,
							0.0140848855293968,	0.0690100446059607,	0.140515735542082,	0.202192975479381,	0.226632240418417,
							0.202192975479381,	0.140515735542082,	0.0690100446059607,	0.0140848855293968,	-0.0125404257819552,
							-0.0157935638369741,	-0.00875721735248610,	-0.00221281271600225,	0.000184258321387766};

static const uint16_t filterNum = sizeof(h)/sizeof(h[0]);

#define SAMPLE_FREQ (13200.0)
#define DELTA	(6) //(ceil((446e-6)/(1/SAMPLE_FREQ)));
#define SAMPLES_BUFFER_SIZE	(256)

#define DEM_COMP_HYSTERERSIS	(0.01)

#define SAMPLES_PER_BIT			(SAMPLE_FREQ/BIT_RATE)
#define SAMPLES_TO_BIT_MIDDLE	(SAMPLES_PER_BIT/2.0)
#define DATA_BITS_PER_FRAME		(8)
typedef enum{MODEM_DEM_IDLE, MODEM_DEM_READING, MODEM_DEM_ENDED_FRAME, MODEM_DEM_TRANSITION}MODEM_DemState;

typedef struct
{
	float samples[SAMPLES_BUFFER_SIZE];     /** Pointer to statically reserved memory array. */
//	char * const buffer_end; /** Pointer to end of the array. */
	uint16_t head;	         /** Pointer to the head of the buffer. */
	uint16_t tail;	         /** Pointer to the tail of the buffer. */
	uint16_t capacity;            /** Maximum number of elements in the buffer. */
	uint16_t count;               /** Number of elements in the buffer. */
//	uint16_t size;                /** Size of each element in the buffer. */
} MODEMCircularBuffer;

static MODEMCircularBuffer xBuffer, mBuffer, dBuffer;
static uint8_t outSamples[SAMPLES_BUFFER_SIZE];

static uint16_t firstSample;
static uint16_t timeIndex;


static uint16_t demodulationSampleCount;
static MODEM_DemState demodulationState;
//static uint16_t sampleNumInBit;
static uint16_t demodulationTxBitNum;
static uint8_t demodulationTxByte;
static bool demodulationParityBit;
static bool demodulationSynked;
static uint16_t demodulationTimeIndexResto;


/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////


void modulate(void * data);



void MODEM_demodulationUpdateState(void);
void MODEM_demodulationGoToNextSample(void);




bool MOEDM_BuffInit(MODEMCircularBuffer *this);
bool MODEM_BuffPush(MODEMCircularBuffer *this, float sample);
bool MODEM_BuffPop(MODEMCircularBuffer *this,float *sample);
void MODEM_BuffFlush(MODEMCircularBuffer * this);
uint16_t MODEM_BuffNumel(MODEMCircularBuffer *this);
bool MODEM_BuffIsEmpty(MODEMCircularBuffer *this);
bool MODEM_BuffIsFull(MODEMCircularBuffer *this);

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
		PIT_SetTimerPeriod (DAC_TIMER, MARK);
	else
	{
		PIT_SetTimerPeriod (DAC_TIMER, outcomingBits[tail]);
		tail = (tail + 1)%bufferSize;
	}

	CLEAR_TEST_PIN;
}

//  Fir coeffs
static float FIR[6];

// Buffer to store delayed samples: x(n) to x(n-delta)
NEW_FLOAT_BUFFER(x,5+1);

// Buffer to store m(n) = x(n)*x(n-delta)
NEW_FLOAT_BUFFER(m,6);

void demodulate()
{
	static float y;

	// Get value from ADC x(n)
	PUSH(x,( (float)ADC_GetConversionValue()/1024.0 - 0.5) *3.3);

	// Push m(n)=x(n)*x(n-delta)
	PUSH(m,GET_TAIL(x)*GET_HEAD(x));

	// Apply filter
	y = 0;
	for(int i=0; i<6; i++)
		y += GET(m,i) * FIR[i];

	uint8_t output = y < 0 ? 1:0;


}



void MODEM_Init(MODEM_Config * config)
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


	// 						MODULES INITIALIZATION

	//----------------------------- DMA -------------------------------//
	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMAconfig.enableDebugMode=false;
	DMA_Init(&DMAconfig);
	DMAMUX_Init();


	//----------------------------- DAC -------------------------------//
	DAC_Init(DAC_0,DAC_VREF_2);
	DAC_Enable(DAC_0);

	//----------------------------- ADC -------------------------------//
	ADC_Config ADCconfig;
	ADC_GetDefaultConfig(&ADCconfig);
	ADCconfig.DMAEnable = false; // ADC triggers DMA request
	ADC_Init(ADC_0,&ADCconfig);
	ADC_SetHardwareTrigger(ADC_0);

	//----------------------------- PDB -------------------------------//
	/*
	 * VER ACA SI PODEMOS CONFIGURAR EL PDB, PARA PASARLE UN callback QUE SE LLAME CADA VEZ
	 * QUE GENERA UN TRIGGER DE ADC, HABRIA QUE PRENDERLE LAS INTERRUPCIONES.
	 * */
	PDB_Config PDBconfig;
	PDB_GetDefaultConfig(&PDBconfig);
	PDBconfig.MODValue = ((BUS_CLOCK/SAMPLE_FREQ)+0.5); // PDB running at sample rate
	PDB_Init(&PDBconfig);

	//----------------------------- PIT -------------------------------//
	PIT_Config PITConfig;
	PITConfig.debugModeEnable=true;
	PIT_Init(&PITConfig);
	PIT_Enable();

	// 								INPUT
	// Configure DMA0 to copy from sine table to DAC
	DMAMUX_SetSource(DAC_DMA_CHANNEL,DMAMUX_AlwaysEnabled0);

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
	DMATransfer.sourceLastAdjust = -1*sizeof(signal);
	DMATransfer.destinationLastAdjust = 0;
	DMA_SetTransferConfig(DAC_DMA_CHANNEL,&DMATransfer);
	DMA_EnableChannelRequest (DAC_DMA_CHANNEL);


	//	Configure PIT0 timer to trigger DMA copy from sine table to DAC
	PIT_SetTimerPeriod (DAC_TIMER, T1);



	//	Configure PIT1 timer to interrupt periodically and modulate every bit
	PIT_SetTimerPeriod (MODULATION_TIMER, 41666);
	PIT_TimerIntrruptEnable(MODULATION_TIMER, true);
	PIT_SetTimerIntrruptHandler(MODULATION_TIMER,&modulate, NULL);

	// 								OUTPUT


	// Configure ADC channel to sample input signal
	ADC_ChannelConfig ADCchannelConfig;
	ADC_GetDefaultChannelConfig(&ADCchannelConfig);
	ADCchannelConfig.InterruptsEnable = true;
	ADC_SetChannelConfig(ADC_0,ADC_ChannelA,&ADCchannelConfig);


	// Configure PDB to trigger ADC conversions periodically
	PDB_SetADCTriggerDelay(PDB_Channel0,PDB_PreTrigger0, 1500); // This delay value doesnt matter
	PDB_EnableADCTrigger(PDB_Channel0,PDB_PreTrigger0,true);
	PDB_DoLoadValues();

/*
	// Configure DMA1 to copy from ADC to buffer
	DMAMUX_SetSource(ADC_DMA_CHANNEL,DMAMUX_ADC0);
	DMAMUX_EnableChannel(ADC_DMA_CHANNEL,false);

	DMATransfer.sourceAddress = (uint32_t)ADC_GetDataResultAddress(ADC_0,ADC_ChannelA);
	DMATransfer.sourceOffset = 0;
	DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceLastAdjust = 0;

	DMATransfer.destinationAddress = (uint32_t)ADCSamples;
	DMATransfer.destinationOffset = 2;
	DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.destinationLastAdjust= -1*sizeof(ADCSamples);

	DMATransfer.majorLoopCounts = sizeof(ADCSamples)/sizeof(ADCSamples[0]);
	DMATransfer.minorLoopBytes = 2;

	DMA_SetTransferConfig(ADC_DMA_CHANNEL,&DMATransfer);
	DMA_EnableChannelRequest (ADC_DMA_CHANNEL);
*/

	// Start timers to start output signal and modulation
	PIT_TimerEnable(DAC_TIMER, true);
	PIT_TimerEnable(MODULATION_TIMER, true);


	// Trigger PDB to start ADC sampling (and DMA requests)
	PDB_SoftwareTrigger();


}
void DMA0_IRQHandler()
{
	int i=0;
	i++;
	if(i==0)
		while(1);
}

void ADC0_IRQHandler(void){

	digitalToggle(PORTNUM2PIN(PC,5));
//
//	static MODEMCircularBuffer xBuffer, mBuffer, dBuffer;

	if(MODEM_BuffIsEmpty(&xBuffer))
	{
		ASSERT(MOEDM_BuffInit(&xBuffer));
		ASSERT(MOEDM_BuffInit(&mBuffer));
		ASSERT(MOEDM_BuffInit(&dBuffer));
	}

	static float dataSample;
	static float xn;
	xn = (float)ADC_GetConversionResult(ADC_0,ADC_ChannelA)/ADC_RESOLUTION*ADC_VCC-ADC_OFFSET;

	ASSERT(MODEM_BuffPush(&xBuffer, xn));

	if(xBuffer.count > DELTA)
	{
		static float mn;
		mn = xBuffer.samples[(xBuffer.tail + DELTA)%xBuffer.capacity] * xBuffer.samples[xBuffer.tail]; //Aca debería poder mirar los elmentos dentro del buffer circular.
		ASSERT(MODEM_BuffPush(&mBuffer, mn));
		if((mBuffer.count) > filterNum)
		{
			static float dn = 0;
			firstSample = mBuffer.tail;
			for(timeIndex = firstSample; timeIndex < firstSample + filterNum; timeIndex++)
				dn += ((mBuffer.samples[(timeIndex)%mBuffer.capacity]) * (h[filterNum - (timeIndex - firstSample) - 1]));

			if (dn > DEM_COMP_HYSTERERSIS)
				dn = ADC_GND;
			else if (dn < -DEM_COMP_HYSTERERSIS)
				dn = 1;
			ASSERT(MODEM_BuffPush(&dBuffer, dn));


			ASSERT(MODEM_BuffPop(&mBuffer, &dataSample));

			if(MODEM_BuffIsFull(&dBuffer))
				ASSERT(MODEM_BuffPop(&dBuffer, &dataSample));
			demodulationSampleCount ++;
		}

		ASSERT(MODEM_BuffPop(&xBuffer, &dataSample));
	}

//	float printVal = dBuffer.samples[dBuffer.tail];
//	uint16_t printVal = xBuffer.samples[xBuffer.tail];
//	DAC_WriteValue(DAC_0,(uint16_t)((printVal+1.65)*1024.0/3.3));
	if(demodulationSampleCount == SAMPLES_BUFFER_SIZE)
	{
		switch(demodulationState)
		{

			case MODEM_DEM_IDLE:

				if(demodulationSynked)
				{
					firstSample = dBuffer.tail + SAMPLES_PER_BIT - demodulationTimeIndexResto;
					for(timeIndex = firstSample; (timeIndex < firstSample + dBuffer.count) && ((dBuffer.samples[(timeIndex)%dBuffer.capacity]) == ADC_GND); timeIndex += SAMPLES_PER_BIT){};
				}
				else
				{
//					uint16_t a = dBuffer.count;
					firstSample = dBuffer.tail;
					for(timeIndex = firstSample; timeIndex < firstSample + dBuffer.count; timeIndex ++)
					{
						dataSample = 2;
//						uint8_t k;
//						k = (timeIndex)%dBuffer.capacity;
						dataSample = dBuffer.samples[dBuffer.tail];
//						if(dBuffer.samples[(timeIndex)%dBuffer.capacity] == ADC_GND)
//							break;
					}
				}

				if(dataSample == ADC_GND)
				{
					demodulationState = MODEM_DEM_READING;

					if(demodulationSynked)
						timeIndex += SAMPLES_PER_BIT;
					else
					{
						timeIndex += SAMPLES_PER_BIT + SAMPLES_TO_BIT_MIDDLE - 1;
						demodulationSynked = true;
					}

					for(; timeIndex < firstSample + dBuffer.count; timeIndex += SAMPLES_PER_BIT)
					{
						MODEM_demodulationGoToNextSample();
					}
					demodulationTimeIndexResto = timeIndex - (firstSample + dBuffer.count);
				}
				break;

			case MODEM_DEM_READING: case MODEM_DEM_ENDED_FRAME: case MODEM_DEM_TRANSITION:
				MODEM_demodulationUpdateState();
				break;

		}
		demodulationSampleCount = 0;
	}





	digitalToggle(PORTNUM2PIN(PC,5));

}
void MODEM_demodulationUpdateState(void)
{
	firstSample = dBuffer.tail + SAMPLES_PER_BIT - demodulationTimeIndexResto;
	for(timeIndex = firstSample; timeIndex < firstSample + dBuffer.count; timeIndex += SAMPLES_PER_BIT)
	{
		MODEM_demodulationGoToNextSample();
	}
	demodulationTimeIndexResto = timeIndex - (firstSample + dBuffer.count);

}
void MODEM_demodulationGoToNextSample(void)
{
	if((demodulationTxBitNum != DATA_BITS_PER_FRAME) && (demodulationState == MODEM_DEM_READING))
	{
		if(dBuffer.samples[(timeIndex)%dBuffer.capacity] == 1)
			demodulationTxByte |= true << demodulationTxBitNum;
		else
			demodulationTxByte &= ~(true << demodulationTxBitNum);

		demodulationTxBitNum ++;
	}else if(demodulationState == MODEM_DEM_READING)
	{
		demodulationParityBit = (dBuffer.samples[(timeIndex)%dBuffer.capacity] == 1)? true : false;
		demodulationState = MODEM_DEM_ENDED_FRAME;
	}else if(demodulationState == MODEM_DEM_ENDED_FRAME)
	{
//		ASSERT(dBuffer.samples[(timeIndex)%dBuffer.capacity] == 1);
		demodulationState = MODEM_DEM_TRANSITION;
	}else if(demodulationState == MODEM_DEM_TRANSITION)
	{
		if(dBuffer.samples[(timeIndex)%dBuffer.capacity] == ADC_GND)
		{
			demodulationState = MODEM_DEM_READING;
			demodulationTxBitNum = 0;
		}else
		{
			demodulationState = MODEM_DEM_IDLE;
			demodulationTxBitNum = 0;
		}
	}
}






void MODEM_SendData(uint8_t data)
{
	//SET_TEST_PIN;

	// Disable interrupts while adding data to output buffer
	PIT_TimerIntrruptEnable(MODULATION_TIMER, false);

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
	PIT_TimerIntrruptEnable(MODULATION_TIMER, true);

	//CLEAR_TEST_PIN;
}

bool MODEM_ReceiveData(uint8_t * buffer, uint8_t * length)
{


	return true;
}


bool MOEDM_BuffInit(MODEMCircularBuffer *this)
{
	this->capacity = SAMPLES_BUFFER_SIZE;
	this->count = 0;
	this->head = 0;
	this->tail = 0;

	return true;
}
bool MODEM_BuffPush(MODEMCircularBuffer *this, float samples)
{
	if(this->count == this->capacity)
		return false;
	else
	{
		this->samples[this->head] = samples;
		this->head ++;
	    if(this->head == this->capacity)
	    	this->head = 0;
	    this->count++;
	    return true;
	}
}
bool MODEM_BuffPop(MODEMCircularBuffer *this,float *samples)
{
	if(this->count == 0)
		return false;
	else
	{
		*samples = this->samples[this->tail];
		this->tail ++;
		if(this->tail == this->capacity)
			this->tail = 0;
		this->count--;
		return true;
	}
}
void MODEM_BuffFlush(MODEMCircularBuffer * this)
{
	this->head = this->tail;
	this->count = 0;
}
uint16_t MODEM_BuffNumel(MODEMCircularBuffer *this)
{
	return this->count;
}
bool MODEM_BuffIsEmpty(MODEMCircularBuffer *this)
{
	return (this->count == 0);
}
bool MODEM_BuffIsFull(MODEMCircularBuffer *this)
{
	return (this->count == this->capacity);
}
