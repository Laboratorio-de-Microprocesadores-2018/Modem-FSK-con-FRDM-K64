/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "Modem.h"

#if MODEM_VERSION == 2

#pragma message ("Using version 2 of the modem")

#include "DMA.h"
#include "DMAMUX.h"
#include "FTM.h"
#include "CMP.h"
#include "PORT.h"
#include "Assert.h"
#include "math.h"
#include "CircularBuffer.h"
#include "CPUTimeMeasurement.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////
#define PI 3.1415926

// Look up table for parity
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
#define LOOK_UP P6(0), P6(1), P6(1), P6(0)

/*-------------------------------DEC--------------------------------------*/


#define BIT_1FREC (1200)
#define BIT_0FREC (2400)
#define MAX_MOD ((1<<16)-1)
#define MOD_IC_USED MAX_MOD
#define SYSTEM_CLOCK_FREC 50000000
#define CNV_VAL0 SYSTEM_CLOCK_FREC/BIT_0FREC
#define CNV_VAL1 SYSTEM_CLOCK_FREC/BIT_1FREC
#define PS_USED FTM_PRESCALE_1

/*-------------------------------------GEN-------------------------------*/
#define BIT_FREC (1200)
/** Capacity of bitstream buffer */
#define FRAMES_BITSTREAM_BUFFER (32)
/** Bits per frame: start + 8 data bits + parity + stop */
#define	BITS_PER_FRAME (11)
/** Size of bitstream buffer */
#define BITSTREAM_BUFFER_SIZE (FRAMES_BITSTREAM_BUFFER*BITS_PER_FRAME)
/** Helper macro to map a bit to its respective samples table*/
#define BIT2FTABLE(x) (x)==0 ? (uint32_t)CnVTableH : (uint32_t)CnVTableL

/** Choose either edge aligned or center aligned PWM */
//#define CPWM
#ifdef CPWM
#define MOD4PWM  (SYSTEM_CLOCK_FREC / (2*PWM_FREC*(1<<PS_USED)) )
#define TABLE_SIZE (2*PWM_FREC/BIT_FREC)
#define HALF_TABLE_SIZE (PWM_FREC/BIT_FREC)
#define PWM_MODE_USED FTM_PWM_CENTER_ALIGNED
#endif

#define EPWM

#ifdef EPWM
#define MOD4PWM ((SYSTEM_CLOCK_FREC/(PWM_FREC*(1<<PS_USED)))-1)
#define TABLE_SIZE (PWM_FREC/BIT_FREC)
#define PWM_MODE_USED FTM_PWM_EDGE_ALIGNED
#endif


/** Module usage definitions */
#define DMA_CHANNEL_USED 1
#define PWM_FREC 98400
#define PWM_FTM_INSTANCE FTM_0

typedef struct{
	uint32_t outcomingBits[BITSTREAM_BUFFER_SIZE];
	uint32_t head,tail;
}CircBuff;

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

/** Buffer with received data (uint16_t is used because byte is stored with parity bit).*/
NEW_CIRCULAR_BUFFER(receivedBytes,10,sizeof(uint16_t));

/** Tables with CnV for FTM module to generate PWM*/
static uint16_t CnVTableL[TABLE_SIZE];
static uint16_t CnVTableH[TABLE_SIZE];

// Look up table with parity
static unsigned int parityTable[256] = { LOOK_UP };

static CircBuff outputBuffer;


/////////////////////////////////////////////////////////////////////////////////
//                   Local function declarations                               //
/////////////////////////////////////////////////////////////////////////////////
static void createCnVSineTables(uint16_t *arr1,uint16_t *arr2);//size=PWM_FREC/BIT_FREC
static void processCaptureTime(uint16_t captureValue);
static void callback4DMA(void);
static void GenInit(void);
static void DecInit(void);

/////////////////////////////////////////////////////////////////////////////////
//                   	Local functions and global Services  			       //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Modem version 2 initialization
 */
void MODEM_Init(void)
{
	GenInit();
	DecInit();
}

/**
 * @brief Modem version 2 initialization decodification part
 */
static void DecInit(void)
{
	// PIN configuration for CMP
	PORT_Config PORTconfig;
	PORT_GetPinDefaultConfig(&PORTconfig);
	// PC5 as CMP0_OUT
	PORTconfig.mux = PORT_MuxAlt6;
	PORT_PinConfig(PORT_C,5,&PORTconfig);
	// PC7 as CMP0_IN1
	PORTconfig.mux = PORT_MuxAlt0;
	PORT_PinConfig(PORT_C,7,&PORTconfig);

	//			CMP
	CMP_Config CMPconfig;
	CMP_GetDefaultConfig(&CMPconfig);
	CMPconfig.hysteresisMode = CMP_HysteresisLevel3;
	CMPconfig.enableHighSpeed = false;
	CMPconfig.enableOutputPin = true;
	CMP_Init(CMP_0,&CMPconfig);
	CMP_SetInputChannels(CMP_0,CMP_IN1,CMP_IN7);

	CMP_DACConfig CMP_DACconfig = {.vref = CMP_VrefSourceVin2, .dacValue= 1.65/(3.33/64)-1};
	CMP_SetDACConfig (CMP_0, &CMP_DACconfig);

	// Configure filter with 5us period, and to filter glitches of less than 5 samples (25us)
	CMP_FilterConfig CMPFilterConfig = {.filterPeriod = 200, .filterCount = 4};
	CMP_SetFilterConfig (CMP_0, &CMPFilterConfig);

	//Input capture configuration
	FTM_Config config4IC;
	config4IC.clockSource = FTM_SYSTEM_CLOCK;
	config4IC.prescale = FTM_PRESCALE_1;
	FTM_Init(FTM_1,&config4IC);

	FTM_InputCaptureConfig ICConf;
	ICConf.channel=0;
	ICConf.enableDMA=false;
	ICConf.filterValue=0;
	ICConf.mod=MOD_IC_USED;
	ICConf.mode=FTM_RISING_EDGE;
	ICConf.callback=processCaptureTime;//set callback to process capture
	FTM_SetupInputCapture(FTM_1, &ICConf);
	FTM_EnableClock(FTM_1);

	CMP_SetOutputDestination(CMP_OUT_FTM1_CH0);

}

/**
 * @brief Modem version 2 initialization PWM generation part
 */
static void GenInit(void)
{
	// DMA configuration
	createCnVSineTables(CnVTableL,CnVTableH);
	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMA_Init(&DMAconfig);

	//DMA Transfer configuration
	DMA_TransferConfig DMATransfer;
	DMATransfer.sourceAddress = (uint32_t)CnVTableL;
	DMATransfer.destinationAddress = FTM_GetCnVAddress(0,0);
	DMATransfer.destinationOffset = 0;
	DMATransfer.sourceOffset = 2;
	DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceLastAdjust=-sizeof(CnVTableL);
	DMATransfer.destinationLastAdjust=0;
	DMATransfer.majorLoopCounts = sizeof(CnVTableL)/sizeof(CnVTableL[0]);
	DMATransfer.minorLoopBytes = 2;

	DMA_SetTransferConfig(DMA_CHANNEL_USED,&DMATransfer);

	DMA_SetCallback(DMA_CHANNEL_USED,callback4DMA);
	DMA_EnableInterrupts(DMA_CHANNEL_USED);
	DMA_EnableChannelRequest (DMA_CHANNEL_USED);

	//DMA MUX
	DMAMUX_Init ();
	DMAMUX_SetSource(DMA_CHANNEL_USED,DMAMUX_FTM0_CH0);
	DMAMUX_EnableChannel(DMA_CHANNEL_USED,false);



	//FTM configuration for PWM generation and DMA triggering

	//Configure PORT C pin 1 to be the PWM output
	PORT_Config portConf;
	PORT_GetPinDefaultConfig(&portConf);
	portConf.mux = PORT_MuxAlt4;	//Alt4 FTM0 Chnl 0
	PORT_PinConfig(PORT_C,1,&portConf);

	//Basic FTM configuration
	FTM_Config config4PWM;
	config4PWM.clockSource = FTM_SYSTEM_CLOCK;
	config4PWM.prescale = FTM_PRESCALE_1;
	FTM_Init(PWM_FTM_INSTANCE,&config4PWM);

	//PWM configuration
	FTM_PwmConfig PWMConfig;
	PWMConfig.channel = FTM_CHNL_0;
	PWMConfig.mode = PWM_MODE_USED;
	PWMConfig.enableDMA=true;
	PWMConfig.CnV =(uint16_t)((MOD4PWM)*0.5) ;
	PWMConfig.mod=MOD4PWM;

	FTM_SetupPwm(PWM_FTM_INSTANCE,&PWMConfig);


	FTM_EnableClock(PWM_FTM_INSTANCE);
}


/**
 * @brief Modem version 2  function to send a byte
 * @param data byte to be sent
 */
void MODEM_SendByte(uint8_t data)
{
	SET_TEST_PIN;

	DMA_DisableInterrupts(1);

	//Start bit
	outputBuffer.outcomingBits[outputBuffer.head]= BIT2FTABLE(0);
	outputBuffer.head = (outputBuffer.head + 1)%BITSTREAM_BUFFER_SIZE;

	//DATA
	for(int i=0; i<8; i++)
	{
		outputBuffer.outcomingBits[outputBuffer.head]=BIT2FTABLE((data>>i)&(1) );
		outputBuffer.head = (outputBuffer.head + 1)%BITSTREAM_BUFFER_SIZE;
	}

	// Parity bit
	outputBuffer.outcomingBits[outputBuffer.head]=BIT2FTABLE(parityTable[data]);
	outputBuffer.head = (outputBuffer.head + 1)%BITSTREAM_BUFFER_SIZE;

	// Stop bit
	outputBuffer.outcomingBits[outputBuffer.head]=BIT2FTABLE(1);
	outputBuffer.head = (outputBuffer.head + 1)%BITSTREAM_BUFFER_SIZE;

	// Enable interrupts
	DMA_EnableInterrupts(1);

	CLEAR_TEST_PIN;
}

/**
 * @brief Modem version 2  function to receive a byte
 * @param byte to be sent
 * @return true when there is new data, false if no data has arrived
 */
bool MODEM_ReceiveByte(uint8_t * byte)
{
	uint16_t d=0;

	FTM_DisableInterrupts(FTM_1);
	bool b = pop(&receivedBytes,&d);
	FTM_EnableInterrupts(FTM_1);

	if(b)
	{
		(*byte) = (uint8_t)(d&0xFF);

		if(((d>>8)&1) == parityTable[(*byte)])
			return true;
		else
			return false;
	}
	else
		return false;
}


/**
 * This function creates a table of Cnv values to be used in the generation of a sine wave
 * according to PWM_FREC and BIT_FREC
 * arr1 has BIT_FREC and arr2 BIT_FREC*2
 */
#ifdef CPWM
static void createCnVSineTables(uint16_t *arr1,uint16_t *arr2)
{

	double a=0;
	for(uint16_t i=0; i<HALF_TABLE_SIZE; i++)
	{
		//DUTY VALUES
		a=(1.0+sin(2*PI*(double)i/(double)HALF_TABLE_SIZE))/2.0;
		//CnV VALUES
		arr1[2*i]=(uint16_t)((a*(MOD4PWM-2))+1+0.5);
		arr1[(2*i)+1]=arr1[2*i];
	}
	for(uint16_t i=0; i<HALF_TABLE_SIZE; i++)
	{
		//DUTY VALUES
		a=(1.0+sin(2*PI*(double)i*2/(double)HALF_TABLE_SIZE))/2.0;
		//CnV VALUES
		arr2[2*i]=(uint16_t) (a*(MOD4PWM-2)+1+0.5);
		arr2[(2*i)+1]=arr2[2*i];

	}

}
#endif

#ifdef EPWM
static void createCnVSineTables(uint16_t *arr1,uint16_t *arr2)
{

	double a=0;
	for(uint16_t i=0; i<TABLE_SIZE; i++)
	{
		//DUTY VALUES
		a=(1.0+sin(2*PI*(double)i/(double)TABLE_SIZE))/2.0;
		//CnV VALUES
		arr1[i]=(uint16_t)((a*(MOD4PWM-2))+1+0.5);
	}
	for(uint16_t i=0; i<TABLE_SIZE; i++)
	{
		a=(1.0+sin(2*PI*(double)i*2/(double)TABLE_SIZE))/2.0;
		arr2[i]=(uint16_t) (a*(MOD4PWM-2)+1+0.5);
	}

}
#endif

void MODEM_Demodulate()
{

}
/*
 * Callback for Input Capture, recieves the captured value and pushes bytes to the buffer
 * */
static void processCaptureTime(uint16_t captureValue)
{
	typedef enum{IDLE,START,DATA,STOP}ParsingStatus;

	FTM_ClearCount(FTM_1);

	static ParsingStatus status = IDLE;
	static uint8_t bit0count = 0;
	static uint16_t byteWithParity = 0;
	static uint8_t byteIndex = 0;
	static bool firstCall;

	if(firstCall==false)
	{
		firstCall=true;
		return;
	}
	uint8_t bitRecived;


	if( fabs(captureValue-CNV_VAL1)<(CNV_VAL1*0.35))//  || (captureValue-CNV_VAL1)> (-CNV_VAL1*30.0/100.0)   )
		bitRecived=1;
	else if( fabs(captureValue-CNV_VAL0)<CNV_VAL0*0.35  )//|| captureValue-CNV_VAL0> (-CNV_VAL0*30.0/100.0)   )
		bitRecived=0;
	else
		ASSERT(0);//cambiar

	switch (status)
	{
		case IDLE:
			if(bitRecived==0)
				status=START;
			break;
		case START:
			if(bitRecived==0)
				status=DATA;
			else if(bitRecived==1)
			{	ASSERT(0);}//cambiar
			break;
		case DATA:
			if(bitRecived==1)
			{
				if(bit0count==0)
				{
					byteWithParity|=bitRecived<<byteIndex;
					byteIndex++;
				}
				else
					ASSERT(0);
			}
			else if(bitRecived==0)
			{
				bit0count++;
				if(bit0count==2)
				{
					bit0count=0;
					byteIndex++;
				}
			}
			if(byteIndex==9)
			{
				byteIndex=0;
				push(&receivedBytes,&byteWithParity);
				byteWithParity=0;
				status=STOP;
			}
			break;
		case STOP:
			if(bitRecived==1)
				status=IDLE;
			else if(bitRecived==0)
				status=START;
			break;
	}

}


/**
 * major loop callback for DMA, according to the data received selects one sine table
 * or the other (high or low frequency to represent 0 or 1)
 */

static void callback4DMA(void)
{
	if(outputBuffer.head==outputBuffer.tail)

		DMA_ModifySourceAddress(1,BIT2FTABLE(1));
	else
	{
		DMA_ModifySourceAddress(1,outputBuffer.outcomingBits[outputBuffer.tail]);
		outputBuffer.tail = (outputBuffer.tail + 1)%BITSTREAM_BUFFER_SIZE;
	}

}

#endif // MODEM_VERSION == 2
