#include "PWMGen.h"
#include "DMAMUX.h"
#include "DMA.h"
#include "FTM.h"
#include "math.h"


#include "hardware.h"/*SACARLO DESPUES*/


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

// Generating the look-up table while pre-processing
#define P2(n) n, n ^ 1, n ^ 1, n
#define P4(n) P2(n), P2(n ^ 1), P2(n ^ 1), P2(n)
#define P6(n) P4(n), P4(n ^ 1), P4(n ^ 1), P4(n)
#define LOOK_UP P6(0), P6(1), P6(1), P6(0)

// LOOK_UP is the macro expansion to generate the table
static unsigned int parityTable[256] = { LOOK_UP };



#define OUTCOMING_BUFF_BYTES (32)
#define	BITS_PER_BYTE (11)
#define OUTCOMING_BUFF_SIZE (OUTCOMING_BUFF_BYTES*BITS_PER_BYTE)
#define PI 3.1415926

#define BIT2FTABLE(x) (x)==0 ? (uint32_t)CnVTableH : (uint32_t)CnVTableL

#define BIT_FREC 1200
#define PWM_FREC 98400


static uint16_t CnVTableL[2*PWM_FREC/BIT_FREC];
static uint16_t CnVTableH[2*PWM_FREC/BIT_FREC];

typedef struct{
	uint32_t outcomingBits[OUTCOMING_BUFF_SIZE];
	uint8_t head,tail;

}CircBuff;



static CircBuff outputBuffer;

static void createCnVSineTables(uint16_t *arr1,uint16_t *arr2, uint16_t size,uint16_t mod);//size=PWM_FREC/BIT_FREC

static void callback(void);

void PWMGen_SendData(uint8_t data)
{
	SET_TEST_PIN;
	DMA_DisableInterrupts(1);

	//Start bit
	outputBuffer.outcomingBits[outputBuffer.head]= BIT2FTABLE(0);
	outputBuffer.head = (outputBuffer.head + 1)%OUTCOMING_BUFF_SIZE;

	//DATA
	for(int i=0; i<8; i++)
	{
		outputBuffer.outcomingBits[outputBuffer.head]=BIT2FTABLE((data>>i)&(1) );
		outputBuffer.head = (outputBuffer.head + 1)%OUTCOMING_BUFF_SIZE;
	}

	// Parity bit
	outputBuffer.outcomingBits[outputBuffer.head]=BIT2FTABLE(parityTable[data]);
	outputBuffer.head = (outputBuffer.head + 1)%OUTCOMING_BUFF_SIZE;

	// Stop bit
	outputBuffer.outcomingBits[outputBuffer.head]=BIT2FTABLE(1);
	outputBuffer.head = (outputBuffer.head + 1)%OUTCOMING_BUFF_SIZE;

	// Enable interrupts
	DMA_EnableInterrupts(1);

	CLEAR_TEST_PIN;
}


void PWMGen_Init()
{
#ifdef MEASURE_CPU_TIME
	MEASURE_CPU_TIME_PORT->PCR[MEASURE_CPU_TIME_PIN] = PORT_PCR_MUX(1);
	MEASURE_CPU_TIME_GPIO->PDDR |= (1<<MEASURE_CPU_TIME_PIN);
	MEASURE_CPU_TIME_GPIO->PDOR &= ~(1<<MEASURE_CPU_TIME_PIN);
#endif

	// PRUEBA EN ORDEN
		/*	uint16_t MOD = 50000000 / (2*PWM_FREC*(1<<FTM_PRESCALE_1)) ;
			createCnVSineTables(CnVTableL,CnVTableH, PWM_FREC/BIT_FREC,MOD);

			FTM_Config config;
			config.clockSource = FTM_SYSTEM_CLOCK;
			config.prescale = FTM_PRESCALE_1;
			FTM_Init(FTM_0,&config);
			DMA_Config DMAconfig;
			DMA_GetDefaultConfig(&DMAconfig);
			DMA_Init(&DMAconfig);
			DMAMUX_Init ();
			FTM_PwmConfig PWMConfig;
			PWMConfig.channel = FTM_CHNL_0;
			PWMConfig.mode = FTM_PWM_CENTER_ALIGNED;
			PWMConfig.enableDMA=true;
			PWMConfig.CnV =(uint16_t)((MOD)*0.5) ;
			PWMConfig.mod=MOD;

			////FTM_EnableOverflowInterrupt(FTM_0);
			FTM_SetupPwm(FTM_0,&PWMConfig);


			DMA_TransferConfig DMATransfer;
			DMATransfer.sourceAddress = (uint32_t)CnVTableH;
			DMATransfer.destinationAddress = FTM_GetCnVAddress(0,0);
			DMATransfer.destinationOffset = 0;
			DMATransfer.sourceOffset = 2;
			DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;
			DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;
			DMATransfer.sourceLastAdjust=-sizeof(CnVTableH);
			DMATransfer.destinationLastAdjust=0x00;
			DMATransfer.majorLoopCounts = sizeof(CnVTableH)/sizeof(CnVTableH[0]);
			DMATransfer.minorLoopBytes = 2;

			DMA_SetTransferConfig(1,&DMATransfer);
			DMA_EnableInterrupts(1);

			DMAMUX_SetSource(1,20);
			DMAMUX_EnableChannel(1,false);
		//esto hay que cambiarlo
			NVIC_ClearPendingIRQ(DMA0_IRQn);
			NVIC_EnableIRQ(DMA0_IRQn);
		//
			DMA_EnableChannelRequest (1);
			FTM_EnableClock(FTM_0);

			SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

			PORTC->PCR[1] |=  PORT_PCR_MUX(4); //PTC1  Alt4 FTM0_CH0*/
	/*------------------------------------------------------------------------*/
	/*calculo el MOD que voy a usar para mi frec de PWM*/
	uint16_t MOD = 50000000 / (2*PWM_FREC*(1<<FTM_PRESCALE_1)) ;

	/*primero seteo dma*/
	createCnVSineTables(CnVTableL,CnVTableH,PWM_FREC/BIT_FREC,MOD);
	DMA_Config DMAconfig;
	DMA_GetDefaultConfig(&DMAconfig);
	DMA_Init(&DMAconfig);



	DMA_TransferConfig DMATransfer;
	DMATransfer.sourceAddress = (uint32_t)CnVTableL;
	DMATransfer.destinationAddress = FTM_GetCnVAddress(0,0);
	DMATransfer.destinationOffset = 0;
	DMATransfer.sourceOffset = 2;
	DMATransfer.destinationTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceTransferSize = DMA_TransferSize2Bytes;
	DMATransfer.sourceLastAdjust=-sizeof(CnVTableL);
	DMATransfer.destinationLastAdjust=0x00;
	DMATransfer.majorLoopCounts = sizeof(CnVTableL)/sizeof(CnVTableL[0]);
	DMATransfer.minorLoopBytes = 2;

	DMA_SetTransferConfig(1,&DMATransfer);

	DMA_SetCallback(1,callback);
	DMA_EnableInterrupts(1);
	DMA_EnableChannelRequest (1);


	DMAMUX_Init ();
	DMAMUX_SetSource(1,DMAMUX_FTM0_CH0);
	DMAMUX_EnableChannel(1,false);



	/*despues configuro ftm(y le activo dma requests)*/


/*	///SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	///PORT_Config portConf={PORT_LockRegister ,PORT_MuxAlt4,PORT_PullDisable,PORT_FastSlewRate,PORT_OpenDrainDisable,PORT_PassiveFilterDisable, PORT_LowDriveStrength, PORT_InterruptOrDMADisabled };
	///PORT_PinConfig (PORT_C,1,&portConf);*/
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[1] =  PORT_PCR_MUX(4); //PTC1  Alt4 FTM0_CH0
	//FTM PWM Test
	FTM_Config config;
	config.clockSource = FTM_SYSTEM_CLOCK;
	config.prescale = FTM_PRESCALE_1;
	FTM_Init(FTM_0,&config);

	FTM_PwmConfig PWMConfig;
	PWMConfig.channel = FTM_CHNL_0;
	PWMConfig.mode = FTM_PWM_CENTER_ALIGNED;
	PWMConfig.enableDMA=true;
	PWMConfig.CnV =(uint16_t)((MOD)*0.5) ;
	PWMConfig.mod=MOD;

	//FTM_EnableOverflowInterrupt(FTM_0);
	FTM_SetupPwm(FTM_0,&PWMConfig);


	FTM_EnableClock(FTM_0);

}


static void createCnVSineTables(uint16_t *arr1,uint16_t *arr2, uint16_t size,uint16_t mod)//size=PWM_FREC/BIT_FREC
{

	double a=0;
	for(uint16_t i=0; i<size; i++)
	{
		//DUTY VALUES
		a=(1.0+sin(2*PI*(double)i/(double)size))/2.0;
		arr1[2*i]=(uint16_t)((a*(mod-2))+1+0.5);
		arr1[(2*i)+1]=arr1[2*i];
	}
	for(uint16_t i=0; i<size; i++)
	{
		a=(1.0+sin(2*PI*(double)i*2/(double)size))/2.0;
		arr2[2*i]=(uint16_t) (a*(mod-2)+1+0.5);
		arr2[(2*i)+1]=arr2[2*i];

	}

}



static void callback(void)
{
	SET_TEST_PIN;
	if(outputBuffer.head==outputBuffer.tail)

		DMA_ModifySourceAddress(1,BIT2FTABLE(1));
	else
	{
		DMA_ModifySourceAddress(1,outputBuffer.outcomingBits[outputBuffer.tail]);
		outputBuffer.tail = (outputBuffer.tail + 1)%OUTCOMING_BUFF_SIZE;
	}
	CLEAR_TEST_PIN;
}


