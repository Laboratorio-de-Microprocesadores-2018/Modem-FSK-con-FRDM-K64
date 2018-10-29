/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "DMA.h"
#include "hardware.h"
#include "Assert.h"
#include "CPUTimeMeasurement.h"
/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

static IRQn_Type irqTable[]=DMA_CHN_IRQS;
static DMAIrqFun_t DMAcallbacks[FSL_FEATURE_EDMA_DMAMUX_CHANNELS];

/////////////////////////////////////////////////////////////////////////////////
//                   	Local functions and global Services  			       //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Gives a default configuration for the DMA module
 * @param config struct where the function will retunr the default configuration
 */
void DMA_GetDefaultConfig(DMA_Config * config)
{

	config->enableContinuousLinkMode=false;
	config->enableDebugMode = false;
	config->enableHaltOnError = false;
	config->enableRoundRobinArbitration = false;
}

/**
 * @brief Initializes DMA module
 *
 * Ungate the DMA clock and configure DMA peripheral.
 */
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

/**
 *	@brief Sets a a transfer configuration for a certain DMA channel
 *	@param channel used
 *	@param config , struct containing the configuration wished for the transfer
 */
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

	// Enable major interrupts
	DMA0->TCD[channel].CSR|=DMA_CSR_INTMAJOR_MASK;

}
/**
 * @brief enables interrupts in a desired channel
 * @param channel
 */
void DMA_EnableInterrupts (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	NVIC_EnableIRQ(irqTable[channel]);
}


/**
 * @brief disables interrupts in a desired channel
 * @param channel
 */
void DMA_DisableInterrupts (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	NVIC_DisableIRQ(irqTable[channel]);
}
/**
 * @brief software triggers the data transfer configured in a desired channel
 * @param channel to trigger
 */
void DMA_TriggerChannelStart (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	DMA0->TCD[channel].CSR|=DMA_CSR_START_MASK;

}
/**
 * @brief Enables DMA requests for the selected channel
 * @param channel
 */
void DMA_EnableChannelRequest (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMA0->ERQ |= (1<<channel);
}
/**
 * @brief Disables DMA requests for the selected channel
 * @param channel
 */
void DMA_DisableChannelRequest (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMA0->ERQ &= ~(1<<channel);
}
/**
 * @brief Sets a callback to be executed when a transfer ended. The callback should
 * be as quick as posible as it is called inside an interruption
 * @param channel
 * @param majorIntCallback   callback
 */
void DMA_SetCallback(uint32_t channel,DMAIrqFun_t majorIntCallback)//se podria hacer mas lindo
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMAcallbacks[channel]=majorIntCallback;
}
/**
 * @brief Sets a new source address for the DMA transfer for the selected channel
 * @param channel
 * @param newAddress
 */
void DMA_ModifySourceAddress(uint32_t channel, uint32_t newAddress)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	ASSERT(newAddress!=0);

	DMA0->TCD[channel].SADDR = newAddress;
}


void DMA0_IRQHandler(void)
{
	SET_TEST_PIN;

	DMA0->INT |= (1 << 0);
	DMAcallbacks[0]();

	CLEAR_TEST_PIN;
}

void DMA1_IRQHandler(void)
{
//	SET_TEST_PIN;

	DMA0->INT |= (1 << 1);
	DMAcallbacks[1]();

	//CLEAR_TEST_PIN;
}
void DMA2_IRQHandler(void)
{
	SET_TEST_PIN;

	DMA0->INT |= (1 << 2);
	DMAcallbacks[2]();

	CLEAR_TEST_PIN;
}

void DMA_Error_IRQHandler(void)
{
	ASSERT(0);
}
