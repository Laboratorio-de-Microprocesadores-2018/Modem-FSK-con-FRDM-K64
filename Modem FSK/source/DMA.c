#include "DMA.h"
#include "Assert.h"
#include "MK64F12.h"
#include "MK64F12_features.h"

static int irqTable[]=DMA_CHN_IRQS;

void DMA_Init(DMA_Config *config)
{
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
}

void DMA_SetTransferConfig	(uint32_t channel,DMA_TransferConfig * 	config)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);

	DMA0->TCD[channel].SADDR = config->sourceAddress;
	DMA0->TCD[channel].DADDR = config->destinationAddress;
	DMA0->TCD[channel].ATTR = DMA_ATTR_SSIZE(config->sourceTransferSize) | DMA_ATTR_DSIZE(config->destinationTransferSize);
	DMA0->TCD[channel].DOFF = config->destinationOffset;
	DMA0->TCD[channel].SOFF = config->sourceOffset;
	DMA0->TCD[channel].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(config->majorLoopCounts);
	DMA0->TCD[channel].NBYTES_MLNO = config->minorLoopBytes;
}

void DMA_EnableInterrupts (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	NVIC_EnableIRQ(irqTable[channel]);
}

void DMA_DisableInterrupts (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
	NVIC_DisableIRQ(irqTable[channel]);
}

void DMA_TriggerChannelStart (uint32_t channel)
{
	ASSERT(channel<FSL_FEATURE_EDMA_MODULE_CHANNEL);
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