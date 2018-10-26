#include "DMA.h"
#include "Assert.h"
#include "hardware.h"


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


static IRQn_Type irqTable[]=DMA_CHN_IRQS;
static DMAIrqFun_t DMAcallbacks[FSL_FEATURE_EDMA_DMAMUX_CHANNELS];
void DMA_GetDefaultConfig(DMA_Config * config)
{

	config->enableContinuousLinkMode=false;
	config->enableDebugMode = false;
	config->enableHaltOnError = false;
	config->enableRoundRobinArbitration = false;
}


void DMA_Init(DMA_Config *config)
{
#ifdef MEASURE_CPU_TIME
	MEASURE_CPU_TIME_PORT->PCR[MEASURE_CPU_TIME_PIN] = PORT_PCR_MUX(1);
	MEASURE_CPU_TIME_GPIO->PDDR |= (1<<MEASURE_CPU_TIME_PIN);
	MEASURE_CPU_TIME_GPIO->PDOR &= ~(1<<MEASURE_CPU_TIME_PIN);
#endif


	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;

	uint32_t temp=0;
	if(config->enableContinuousLinkMode)
		temp |= DMA_CR_CLM_MASK;
	if(config->enableDebugMode)
		temp |= DMA_CR_EDBG_MASK;
	if(config->enableHaltOnError)
		temp |= DMA_CR_HOE_MASK;
	if(config->enableRoundRobinArbitration)
		temp |= DMA_CR_ERCA_MASK;

	DMA0->CR = temp;
	DMA0->TCD[1].CSR=0;

}

void DMA_SetTransferConfig	(uint32_t channel,DMA_TransferConfig * 	config)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMA0->TCD[channel].SADDR = config->sourceAddress;
	DMA0->TCD[channel].DADDR = config->destinationAddress;

	DMA0->TCD[channel].DOFF = config->destinationOffset;
	DMA0->TCD[channel].SOFF = config->sourceOffset;

	DMA0->TCD[channel].ATTR = DMA_ATTR_SSIZE(config->sourceTransferSize) | DMA_ATTR_DSIZE(config->destinationTransferSize);

	DMA0->TCD[channel].NBYTES_MLNO = config->minorLoopBytes;

	DMA0->TCD[channel].SLAST = config->sourceLastAdjust;
	DMA0->TCD[channel].DLAST_SGA=config->destinationLastAdjust;


	DMA0->TCD[channel].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(config->majorLoopCounts);
	DMA0->TCD[channel].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(config->majorLoopCounts);

	DMA0->CR|=DMA_CR_HOE_MASK;

}

void DMA_EnableInterrupts (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	/**POR AHORA PONGO IGUAL ACA ASI SE LIMPIA*/
	/*		CAMBIARLO DESPUES								*/
	DMA0->TCD[channel].CSR|=DMA_CSR_INTMAJOR_MASK;//|DMA_CSR_DREQ_MASK;//nuevo, la segunda mascara para que no siga trigereando requests
//	NVIC_ClearPendingIRQ(DMA1_IRQn);//nuevo, ver si se saca
	NVIC_EnableIRQ(irqTable[channel]);
}

void DMA_DisableInterrupts (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	DMA0->TCD[channel].CSR&=~DMA_CSR_INTMAJOR_MASK;
	NVIC_DisableIRQ(irqTable[channel]);
}

void DMA_TriggerChannelStart (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	DMA0->TCD[channel].CSR|=DMA_CSR_START_MASK;

}

void DMA_EnableChannelRequest (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMA0->ERQ |= (1<<channel);
}

void DMA_DisableChannelRequest (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMA0->ERQ &= ~(1<<channel);
}

void DMA_SetCallback(uint32_t channel,DMAIrqFun_t majorIntCallback)//se podria hacer mas lindo
{
	DMAcallbacks[0]=majorIntCallback;
}

void DMA_ModifySourceAddress(uint32_t channel, uint32_t newAddress)
{
	if(newAddress!=0)
	{
		DMA0->TCD[channel].SADDR = newAddress;
	}
}


void DMA1_IRQHandler(void)//se podria hacer mas lindo
{
	SET_TEST_PIN;

	DMA0->INT |= (1 << 1);
	DMAcallbacks[0]();

	CLEAR_TEST_PIN;
}

/*void DMA_Error_IRQHandler(void)
{
	int a=0; //Si entró acá es porque hay algo mal configurado
}*/
