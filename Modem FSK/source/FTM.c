#include "FTM.h"
#include "MK64F12.h"
#include "Assert.h"
#include "MK64F12_features.h"


#include "GPIO.h"



static FTM_Type * FTMs[] = FTM_BASE_PTRS;

void FTM_Init(FTM_Instance instance, FTM_Config * config)
{
	ASSERT(instance < FSL_FEATURE_SOC_FTM_COUNT);

	switch(instance)
	{
	case FTM_0:
		SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
		NVIC_EnableIRQ(FTM0_IRQn);
		break;
	case FTM_1:
		SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
		break;
	case FTM_2:
		SIM->SCGC6 |= SIM_SCGC6_FTM2_MASK;
		SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;
		break;
	case FTM_3:
		SIM->SCGC3 |= SIM_SCGC3_FTM3_MASK;
		break;
	}

	FTM_Type * FTM = FTMs[instance];

	FTM->SC = FTM_SC_CLKS(config->clockSource) | FTM_SC_PS(config->prescale);

	FTM->MOD = 0xFFFF;
	pinMode(32+18,OUTPUT);
}

void FTM_EnableOverflowInterrupt(FTM_Instance instance)
{
	FTMs[instance]->SC |= FTM_SC_TOIE_MASK;
}

void FTM_DisableOverflowInterrupt(FTM_Instance instance)
{
	FTMs[instance]->SC &= ~FTM_SC_TOIE_MASK;
}

void FTM0_IRQHandler(void)
{
	if(FTM0->SC & FTM_SC_TOF_MASK)
	{
		FTM0->SC &= ~FTM_SC_TOF_MASK;

		GPIOB->PTOR = 1<<18;
	}
}
void FTM1_IRQHandler(void)
{

}
void FTM2_IRQHandler(void)
{

}
