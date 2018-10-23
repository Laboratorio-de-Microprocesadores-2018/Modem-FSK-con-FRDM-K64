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
	//pinMode(32+18,OUTPUT);
}

bool FTM_SetupPwm(FTM_Instance 	instance, FTM_PwmConfig * config)
{
	ASSERT(instance < FSL_FEATURE_SOC_FTM_COUNT);

	if(config->mode == FTM_PWM_CENTER_ALIGNED)
	{

	}
	else if(config->mode == FTM_PWM_EDGE_ALIGNED)
	{
		// QUADEN = 0
		FTMs[instance]->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;

		//DECAPEN = 0 y COMBINE = 0
		uint32_t COMBINE_MASK[]= {FTM_COMBINE_COMBINE0_MASK,
									FTM_COMBINE_COMBINE1_MASK,
									FTM_COMBINE_COMBINE2_MASK,
									FTM_COMBINE_COMBINE3_MASK};

		uint32_t DECAPEN_MASK[] = {FTM_COMBINE_DECAPEN0_MASK,
									FTM_COMBINE_DECAPEN1_MASK,
									FTM_COMBINE_DECAPEN2_MASK,
									FTM_COMBINE_DECAPEN3_MASK};

		FTMs[instance]->COMBINE &= ~(DECAPEN_MASK[instance]|COMBINE_MASK[instance]);

		// CPWMS = 0 (Centered PWM)
		FTMs[instance]->SC &= ~FTM_SC_CPWMS_MASK;
/*
		// MSnB = 1//esta bien esto? parece que hace otra cosa
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_ELSB_MASK;
*/
		/*------------------yo pondria-------------------------------------------*/
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
		/*-----------------------------------------------------------------------*/

		uint8_t PS = (FTMs[instance]->SC>> FTM_SC_PS_SHIFT)&FTM_SC_PS_MASK;
/*		Es con 1 en lugar de 2, no? */
//		uint16_t MOD = 50000000/(config->PWMFreq*(2<<PS))-1;
		/*---------------cambio-----------------------------------------*/
		uint16_t MOD = ( 50000000 / (config->PWMFreq*(1<<PS)) )-1;
		FTMs[instance]->MOD=MOD;
		//FTM->MOD = MOD;
		/*---------------------------------------------------------------*/
		uint16_t CnV= (uint16_t)((MOD+1)*(config->dutyCyclePercent/100.0) );
		FTMs[instance]->CONTROLS[config->channel].CnV=CnV;

		return true; //cambiar
		// HACER CUENTAS
		//CnV = 65536/2;
		// Width = CnV − CNTIN
	}

	if(config->enableDMA == true)
	{
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_DMA_MASK;
		////////FTMs[instance]->SC |= FTM_SC_CHIE_MASK; // CHANNEL INTERRUPT ENABLE
	}
	else
		FTMs[instance]->CONTROLS[config->channel].CnSC &= ~FTM_CnSC_DMA_MASK;

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
