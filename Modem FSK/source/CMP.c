#include "hardware.h"
#include "CMP.h"

static CMP_Type * CMPs[] = CMP_BASE_PTRS;

static IRQn_Type IRQs[] = CMP_IRQS;

void CMP_GetDefaultConfig(CMP_Config * config)
{
	config->enableModule = true;
	config->enableHighSpeed = true;
	config->enableOutputPin = true;
	config->hysteresisMode = CMP_HysteresisLevel0;
	config->invertOutput = false;
	config->useUnfilteredOutput = false;

}

void CMP_Init(CMP_Instance n, CMP_Config * config)
{
	// Enable clock
	SIM->SCGC4 |= SIM_SCGC4_CMP_MASK;

	if(config->enableModule)
		CMPs[n]->CR1 |= CMP_CR1_EN_MASK;

	if(config->enableHighSpeed)
		CMPs[n]->CR1 |= CMP_CR1_PMODE_MASK;

	if(config->enableOutputPin)
		CMPs[n]->CR1 |= CMP_CR1_OPE_MASK;

	if(config->invertOutput)
		CMPs[n]->CR1 |= CMP_CR1_INV_MASK;

	if(config->useUnfilteredOutput)
		CMPs[n]->CR1 |= CMP_CR1_COS_MASK;

	CMPs[n]->CR0 |= CMP_CR0_HYSTCTR(config->hysteresisMode);

}

void CMP_SetOutputDestination(CMP_Output o)
{
	if(o==CMP_OUT_FTM1_CH0)
		SIM->SOPT4 |= SIM_SOPT4_FTM1CH0SRC_MASK;
	else if(o==CMP_OUT_FTM2_CH0)
		SIM->SOPT4 |= SIM_SOPT4_FTM2CH0SRC_MASK;

}

void CMP_Enable(CMP_Instance n, bool enable)
{
	if(enable)
		CMPs[n]->CR1 |= CMP_CR1_EN_MASK;
	else
		CMPs[n]->CR1 &= ~CMP_CR1_EN_MASK;
}

void CMP_SetInputChannels (CMP_Instance n,CMP_Input positive, CMP_Input negative)
{
	CMPs[n]->MUXCR = CMP_MUXCR_PSEL(positive) | CMP_MUXCR_MSEL(negative);
}

void CMP_SetFilterConfig (CMP_Instance n, CMP_FilterConfig *config)
{
	CMPs[n]->CR0 &= ~CMP_CR0_FILTER_CNT_MASK;
	CMPs[n]->CR0 |= CMP_CR0_FILTER_CNT(config->filterCount);
	CMPs[n]->FPR = config->filterPeriod;
}

void CMP_SetDACConfig (CMP_Instance n, CMP_DACConfig *config)
{
	CMPs[n]->DACCR = CMP_DACCR_DACEN(1) | CMP_DACCR_VRSEL(config->vref) | CMP_DACCR_VOSEL(config->dacValue);
}

uint8_t CMP_GetOutput(CMP_Instance n)
{
	return (uint8_t)((CMPs[n]->SCR & CMP_SCR_COUT_MASK)>> CMP_SCR_COUT_SHIFT);
}
void CMP_EnableInterrupts (CMP_Instance n, uint32_t mask)
{
	CMPs[n]->SCR |= (mask & (CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK));

	if( (CMPs[n]->SCR  & (CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK)) !=0 )
			NVIC_EnableIRQ(IRQs[n]);
}

void CMP_DisableInterrupts (CMP_Instance n, uint32_t mask)
{
	CMPs[n]->SCR &= ~(mask & (CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK));

	if((CMPs[n]->SCR&(CMP_SCR_IER_MASK | CMP_SCR_IEF_MASK))==0 )
		NVIC_DisableIRQ(IRQs[n]);
}

