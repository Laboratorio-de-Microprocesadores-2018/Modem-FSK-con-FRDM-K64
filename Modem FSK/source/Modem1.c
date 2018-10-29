/////////////////////////////////////////////////////////////////////////////////
//                     				MODEM FSK                              	   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

#include "Modem.h"

#if MODEM_VERSION == 1
#pragma message ("Using version 1 of the modem")


/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "ADC.h"
#include "DAC.h"
#include "DMAMUX.h"
#include "DMA.h"
#include "PIT.h"
#include "PDB.h"
#include "math.h"
#include "stdlib.h"
#include "Assert.h"
#include "CircularBuffer.h"
#include "FloatBuffer.h"
#include "CPUTimeMeasurement.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

/** Bitrate of the modem */
#define BIT_RATE	(1200.0)
/** Number of samples of the output sine */
#define N_SAMPLE (256)
/** Bus clock of the cpu */
#define SYSTEM_CLOCK_FREC 50000000.0
/** Mark and space frequency */
#define FREQ_MARK (1200)
#define FREQ_SPACE (2200)
/** Period in Bus Clock between samples */
#define	MARK SYSTEM_CLOCK_FREC/(N_SAMPLE*FREQ_MARK)
#define SPACE SYSTEM_CLOCK_FREC/(N_SAMPLE*FREQ_SPACE)
/** Idle state definition*/
#define IDLE_STATE MARK
/** Helper macro to map a bit to its respective PIT period*/
#define BIT2PERIOD(x) (x)==1? MARK:SPACE
/** Parity table generation */ //https://www.geeksforgeeks.org/compute-parity-number-using-xor-table-look/
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
#define LOOK_UP P6(0), P6(1), P6(1), P6(0)

/** Capacity of bitstream buffer */
#define FRAMES_BITSTREAM_BUFFER (30)
#define BITS_PER_FRAME (11)
#define BITSTREAM_BUFFER_SIZE (FRAMES_BITSTREAM_BUFFER*BITS_PER_FRAME)


#define DEMOD_SAMPLE_FREQ 			(13200.0)
#define DEMOD_DELTA					(6) //(ceil((446e-6)/(1/SAMPLE_FREQ)));
#define DEMOD_SAMPLES_BUFFER_SIZE	(256)

#define DEMOD_COMP_HYSTERERSIS		(0)

#define DEMOD_SAMPLES_PER_BIT			(DEMOD_SAMPLE_FREQ/BIT_RATE)
#define DEMOD_SAMPLES_TO_BIT_MIDDLE		((uint8_t)(DEMOD_SAMPLES_PER_BIT/2))
#define DEMOD_DATA_BITS_PER_FRAME		(8)

#define DEMOD_FIR_ORDER				(19)


/** Module usage definitions */
#define DAC_DMA_CHANNEL 0
#define DAC_TIMER PIT_CHNL_0
#define MODULATION_TIMER PIT_CHNL_1

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

typedef enum{MODEM_DEM_IDLE, MODEM_DEM_READING, MODEM_DEM_ENDED_FRAME, MODEM_DEM_TRANSITION}MODEM_DemState;

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

// Table with parities
static unsigned int parityTable[256] = { LOOK_UP };

// DAC output samples
static uint16_t signal[N_SAMPLE];

/** Buffer with PIT period values to modulate*/
NEW_CIRCULAR_BUFFER(outputBitstream,BITSTREAM_BUFFER_SIZE, sizeof(uint8_t));

/** Buffer with received bytes + parity bit */
NEW_CIRCULAR_BUFFER(transmitBuffer, 10, sizeof(uint16_t));

/** Buffer to store delayed samples: x(n) to x(n-delta) */
NEW_FLOAT_BUFFER(xBuffer,DEMOD_DELTA+1);

/** Buffer to store m(n) = x(n)*x(n-delta) */
NEW_FLOAT_BUFFER(mBuffer,DEMOD_FIR_ORDER);

//  Fir coeffs
static float FIR[DEMOD_FIR_ORDER] = {0.000184258321387766,	-0.00221281271600225,	-0.00875721735248610,	-0.0157935638369741,	-0.0125404257819552,
		0.0140848855293968,	0.0690100446059607,	0.140515735542082,	0.202192975479381,	0.226632240418417,
		0.202192975479381,	0.140515735542082,	0.0690100446059607,	0.0140848855293968,	-0.0125404257819552,
		-0.0157935638369741,	-0.00875721735248610,	-0.00221281271600225,	0.000184258321387766};




/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////


/**
 * @brief A simple stub function to show how links do work.
 *
 * @param param1 Description of the first parameter of the function.
 */
static void MODEM_modulate(void * data)
{
	SET_TEST_PIN;

	uint8_t bit ;
	if(pop(&outputBitstream,&bit))
	{
		PIT_SetTimerPeriod (DAC_TIMER,bit);
	}
	else
		PIT_SetTimerPeriod (DAC_TIMER, MARK);

	CLEAR_TEST_PIN;
}

/**
 * @brief Initialization of the MODEM. Enables all the neccesary modulates and starts them in order to start recieving info.
 *
 * The modules that are used in this case are: DMA, DMAMUX, ADC, DAC, PDB and PIT
 */
void MODEM_Init(void)
{
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
	PDBconfig.MODValue = ((SYSTEM_CLOCK_FREC/DEMOD_SAMPLE_FREQ)+0.5); // PDB running at sample rate
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
	PIT_SetTimerPeriod (DAC_TIMER, IDLE_STATE);



	//	Configure PIT1 timer to interrupt periodically and modulate every bit
	PIT_SetTimerPeriod (MODULATION_TIMER, 41666);
	PIT_TimerIntrruptEnable(MODULATION_TIMER, true);
	PIT_SetTimerIntrruptHandler(MODULATION_TIMER,&MODEM_modulate, NULL);

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

/**
 * @brief Loads the output buffer, given something is recieved through the UART module.
 *
 * @param data the byte recieved through the UART module.
 */
void MODEM_SendByte(uint8_t data)
{

	uint8_t temp;

	// Disable interrupts while adding data to output buffer
	PIT_TimerIntrruptEnable(MODULATION_TIMER, false);

	// Start bit
	temp = BIT2PERIOD(0);
	push(&outputBitstream,&temp);

	// Data bits
	for(int i=0; i<8; i++)
	{
		temp = BIT2PERIOD( (data>>i)&(1) );
		push(&outputBitstream,&temp);
	}

	// Parity bit
	temp = BIT2PERIOD(parityTable[data]);
	push(&outputBitstream,&temp);

	// Stop bit
	temp = BIT2PERIOD(1);
	push(&outputBitstream,&temp);

	// Enable interrupts
	PIT_TimerIntrruptEnable(MODULATION_TIMER, true);

}

/**
 * @brief If there is something to transmit, it loads the demodulated info in the "byte" param in order to send it through UART.
 * It checks parity recieved against parity calculated. If they are different it returns false. If there is nothing to send it returns false.
 *
 * @param byte pointer to the byte where the info to transmit will be copied.
 */
bool MODEM_ReceiveByte(uint8_t * byte)
{
	uint16_t b;

	if(pop(&transmitBuffer,&b))
	{
		// Store byte without parity bit
		(*byte)=(uint8_t)(b&0xFF);

		// Check parity, note that if it doesnt match, the byte is lost
		if(((b>>8)&1) == parityTable[(*byte)])
			return true;
		else
			return false;
	}
	else return false;

}

/**
 * @brief Demodulation function. It is called in App_Run(). So it checks if the ADC has converted a new sample, if so, it process the given
 * sample through the delay block, the FIR filter and finally the comparator. Finally it runs the sample through the "State machine" and
 * eventually loads a new info byte to the transmissioBuffer in order to send it through UART
 *
 * @param byte pointer to the byte where the info to transmit will be copied.
 */
void MODEM_Demodulate(void)
{
	SET_TEST_PIN;

	static MODEM_DemState demodulationState;
	static uint8_t demodulationSampleCount, demodulationTxBitNum;
	static uint16_t demodulationTxByte;

	static float xn, dn;

	CircularBuffer * ADC_samples = ADC_getConversionSamples(ADC_0,ADC_ChannelA);
	while(!isEmpty(ADC_samples))
	{
		ADC_DisableInterrupts(ADC_0,ADC_ChannelA);
		pop(ADC_samples, &xn);
		ADC_EnableInterrupts(ADC_0,ADC_ChannelA);

		// Get value from ADC x(n)
		PUSH(xBuffer,xn);

		// Push m(n)=x(n)*x(n-delta)
		PUSH(mBuffer,GET_TAIL(xBuffer)*GET_HEAD(xBuffer));

		// Apply filter
		dn = 0;
		for(uint16_t i=0; i < DEMOD_FIR_ORDER; i++)
			dn += GET(mBuffer,i) * FIR[i];

		uint8_t output;
		if (dn > DEMOD_COMP_HYSTERERSIS)
			output = 0;
		else if (dn < -DEMOD_COMP_HYSTERERSIS)
			output = 1;


		switch(demodulationState)
		{

			case MODEM_DEM_IDLE:
				if(output == 0)
				{
					demodulationSampleCount ++;
					if(demodulationSampleCount == DEMOD_SAMPLES_PER_BIT - 1)
					{
						demodulationSampleCount = 0;
						demodulationState = MODEM_DEM_READING;
					}
				}else if((output == 1) && (demodulationSampleCount > 0))
					demodulationSampleCount = 0;

				break;

			case MODEM_DEM_READING:

				ASSERT((demodulationSampleCount) <= DEMOD_SAMPLES_PER_BIT);

				if(demodulationSampleCount == DEMOD_SAMPLES_TO_BIT_MIDDLE)
				{
					demodulationTxByte |= output << demodulationTxBitNum;
					demodulationTxBitNum ++;
					demodulationSampleCount ++;

				}
				else if(demodulationSampleCount == DEMOD_SAMPLES_PER_BIT - 1)
				{
					if(demodulationTxBitNum == DEMOD_DATA_BITS_PER_FRAME + 1){
						push(&transmitBuffer , &demodulationTxByte);
						demodulationTxBitNum = 0;
						demodulationTxByte = 0;
						demodulationState = MODEM_DEM_ENDED_FRAME;
					}
					demodulationSampleCount = 0;
				}else
					demodulationSampleCount ++;

				break;

			case MODEM_DEM_ENDED_FRAME:
				ASSERT((demodulationSampleCount) <= DEMOD_SAMPLES_PER_BIT);

				if(demodulationSampleCount == DEMOD_SAMPLES_TO_BIT_MIDDLE)
				{
	//				ASSERT(output);
					demodulationSampleCount ++;

				}
				else if(demodulationSampleCount == DEMOD_SAMPLES_PER_BIT - 1)
				{
					demodulationSampleCount = 0;
					demodulationState = MODEM_DEM_IDLE;
				}else
					demodulationSampleCount ++;

				break;
			default:
				ASSERT(0);
				break;

		}
	}
	CLEAR_TEST_PIN;

}

#endif // MODEM_VERSION==1
