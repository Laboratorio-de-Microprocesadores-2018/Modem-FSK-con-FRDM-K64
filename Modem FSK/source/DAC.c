
#include "DAC.h"
#include "MK64F12.h"
#include "MK64F12_features.h"
#include "Assert.h"

static DAC_Type * DACs[] = DAC_BASE_PTRS;
static IRQn_Type DAC_IRQ[] = DAC_IRQS;


#define DAC_DATL_DATA0_WIDTH (8)
/*


@brief Define the size of hardware buffer
 FSL_FEATURE_DAC_BUFFER_SIZE (16)

@brief Define whether the buffer supports watermark event detection or not.
 FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION (1)

@brief Define whether the buffer supports watermark selection detection or not.
 FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION (1)

@brief Define whether the buffer supports watermark event 1 word before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_1_WORD (1)

@brief Define whether the buffer supports watermark event 2 words before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_2_WORDS (1)

@brief Define whether the buffer supports watermark event 3 words before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_3_WORDS (1)

@brief Define whether the buffer supports watermark event 4 words before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_4_WORDS (1)

@brief Define whether FIFO buffer mode is available or not.
 FSL_FEATURE_DAC_HAS_BUFFER_FIFO_MODE (0)

@brief Define whether swing buffer mode is available or not..
 FSL_FEATURE_DAC_HAS_BUFFER_SWING_MODE (1)*/

void DAC_Init(DAC_Instance n,DAC_Vref vref)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	// Enable clock gating
	if(n==DAC_0)
		SIM->SCGC2 |= SIM_SCGC2_DAC0_MASK;
	else if (n==DAC_1)
		SIM->SCGC2 |= SIM_SCGC2_DAC1_MASK;

	// Enable DAC and select the voltage reference
	if(vref == DAC_VREF_1)
		DACs[n]->C0 = DAC_C0_DACEN_MASK; // CAMBIAR LO DE SOFTWARE TRIGER POR CONFIG
	else if (vref == DAC_VREF_2)
		DACs[n]->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;

}

void DAC_Deinit(DAC_Instance n)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	// Diasble DAC
	DACs[n]->C0 &= ~DAC_C0_DACEN_MASK;

	// Disable clock gating
	if(n==DAC_0)
		SIM->SCGC2 &= ~SIM_SCGC2_DAC0_MASK;
	else if (n==DAC_1)
		SIM->SCGC2 &= ~SIM_SCGC2_DAC1_MASK;
}

void DAC_Enable(DAC_Instance n)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	DACs[n]->C0 |= DAC_C0_DACEN_MASK;
}

void DAC_Disable(DAC_Instance n)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	DACs[n]->C0 &= ~DAC_C0_DACEN_MASK;
}

void DAC_WriteValue(DAC_Instance n,uint16_t value)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	if(~(DACs[n]->C1 & DAC_C1_DACBFEN_MASK))
	{
		DACs[n]->DAT[0].DATL = DAC_DATL_DATA0(value);
		DACs[n]->DAT[0].DATH = DAC_DATH_DATA1(value >> DAC_DATL_DATA0_WIDTH);
	}
}

void DAC_GetDefaultBufferConfig (DAC_BufferConfig * config)
{

}

void DAC_SetBufferConfig(DAC_Instance n, DAC_BufferConfig * config)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	// Configure trigger mode
	if(config->triggerMode == DAC_HARDWARE_TRIGGER)
		DACs[n]->C0 &= ~DAC_C0_DACTRGSEL_MASK;
	else if(config->triggerMode == DAC_SOFTWARE_TRIGGER)
		DACs[n]->C0 |= DAC_C0_DACTRGSEL_MASK;

	// Configure work mode
	DACs[n]->C1 &= ~DAC_C1_DACBFMD_MASK;
	DACs[n]->C1 |= DAC_C1_DACBFMD(config->workMode);

	// Set the watermark
	DACs[n]->C1 &= ~DAC_C1_DACBFWM_MASK;
	DACs[n]->C1 |= DAC_C1_DACBFWM(config->watermark);

	// Set upper limit
	DACs[n]->C2 = DAC_C2_DACBFRP(0) | DAC_C2_DACBFUP(config->upperLimit);

	// Enable buffer
	DACs[n]->C1 |= DAC_C1_DACBFEN_MASK;
}

void DAC_SetBufferValue (DAC_Instance n, uint8_t index, uint16_t value)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);
	ASSERT(index<FSL_FEATURE_DAC_BUFFER_SIZE);

	DACs[n]->DAT[index].DATL = DAC_DATL_DATA0(value);
	DACs[n]->DAT[index].DATH = DAC_DATH_DATA1(value >> DAC_DATL_DATA0_WIDTH);
}

void DAC_EnableBufferDMA (DAC_Instance n, bool enable	)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);

	if(enable)
		DACs[n]->C1 |= DAC_C1_DMAEN_MASK;
	else
		DACs[n]->C1 &= ~DAC_C1_DMAEN_MASK;
}

uint32_t DAC_GetBufferAddress(DAC_Instance n)
{
	return (uint32_t)&(DACs[n]->DAT[0].DATL);
}
uint8_t DAC_GetBufferPointer (DAC_Instance n)
{
	return ((DACs[n]->C2&DAC_C2_DACBFRP_MASK)>>DAC_C2_DACBFRP_SHIFT);
}

void 	DAC_SetBufferPointer (DAC_Instance n, uint8_t index)
{
	DACs[n]->C2 &= ~DAC_C2_DACBFRP_MASK;
	DACs[n]->C2 |= DAC_C2_DACBFRP(index);
}

void DAC_TriggerBuffer (DAC_Instance n)
{
	ASSERT(n<FSL_FEATURE_SOC_DAC_COUNT);
	DACs[n]->C0 |= DAC_C0_DACSWTRG_MASK;
}


void 	DAC_EnableInterrupts (DAC_Instance n, uint32_t mask)
{
	if(mask != 0)
		NVIC_EnableIRQ(DAC_IRQ[n]);
	uint32_t INTERRUPT_MASK = DAC_C0_DACBBIEN_MASK|DAC_C0_DACBTIEN_MASK|DAC_C0_DACBWIEN_MASK;
	DACs[n]->C0 &= ~INTERRUPT_MASK;

	DACs[n]->C0 |= (INTERRUPT_MASK&mask);
}

void 	DAC_DisableInterrupts (DAC_Instance n, uint32_t mask)
{
	mask = ~mask;

	DACs[n]->C0 &= ~(DAC_C0_DACBBIEN_MASK|DAC_C0_DACBTIEN_MASK|DAC_C0_DACBWIEN_MASK);
	DACs[n]->C0 |= DAC_C0_DACBBIEN(mask);
	DACs[n]->C0 |= DAC_C0_DACBTIEN(mask);
	DACs[n]->C0 |= DAC_C0_DACBWIEN(mask);

	// PREGUNTAR
	if((DACs[n]->C0 & (DAC_C0_DACBBIEN_MASK|DAC_C0_DACBTIEN_MASK|DAC_C0_DACBWIEN_MASK))==0)
		NVIC_DisableIRQ(DAC_IRQ[n]);
}

bool DAC_GetFlag(DAC_Instance n, DAC_Flag flag)
{
	return (bool)(DACs[n]->SR&flag);
}

void DAC_ClearFlag(DAC_Instance n, DAC_Flag flag)
{
	DACs[n]->SR&= ~flag;
}

