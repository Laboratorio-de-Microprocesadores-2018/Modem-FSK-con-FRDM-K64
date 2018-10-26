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

	FTM->SC |= FTM_SC_PS(config->prescale);
	NVIC_ClearPendingIRQ(FTM0_IRQn);
	NVIC_EnableIRQ(FTM0_IRQn);
	//FTM->SC = FTM_SC_CLKS(config->clockSource) | FTM_SC_PS(config->prescale);


	/*NO DARLE CLOCK SOURCE AL FTM HASTA QUE ESTE LISTO PARA ANDAR!!!!!!!!!!!!!*/
	//FTM->SC = FTM_SC_CLKS(config->clockSource) | FTM_SC_PS(config->prescale);

	//FTM->MOD = 0xFFFF;
	//pinMode(32+18,OUTPUT);
}

bool FTM_SetupPwm(FTM_Instance 	instance, FTM_PwmConfig * config)
{
	ASSERT(instance < FSL_FEATURE_SOC_FTM_COUNT);

	//uint8_t clkSource=(FTMs[instance]->SC&FTM_SC_CLKS_MASK)>>FTM_SC_CLKS_SHIFT;
	//FTMs[instance]->SC&=~FTM_SC_CLKS_MASK;

	if(config->enableDMA == true)
	{
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_DMA_MASK|FTM_CnSC_CHIE_MASK;
	}
	else
		(FTMs[instance]->CONTROLS[config->channel].CnSC) &= ~FTM_CnSC_DMA_MASK;

	FTMs[instance]->SYNCONF|=FTM_SYNCONF_SWWRBUF_MASK|FTM_SYNCONF_SWSOC_MASK;
	FTMs[instance]->SYNC|=FTM_SYNC_SWSYNC_MASK;

	if(config->mode == FTM_PWM_CENTER_ALIGNED)
	{


		FTMs[instance]->CONTROLS[config->channel].CnSC|=FTM_CnSC_ELSB_MASK;

		// QUADEN = 0
		FTMs[instance]->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;
		//DECAPEN = 0 y COMBINE = 0
		uint32_t COMBINE_MASK[]= {FTM_COMBINE_COMBINE0_MASK,FTM_COMBINE_COMBINE1_MASK,
									FTM_COMBINE_COMBINE2_MASK,FTM_COMBINE_COMBINE3_MASK};

		uint32_t DECAPEN_MASK[] = {FTM_COMBINE_DECAPEN0_MASK,FTM_COMBINE_DECAPEN1_MASK,
				FTM_COMBINE_DECAPEN2_MASK,FTM_COMBINE_DECAPEN3_MASK};

		uint32_t SYNCEN_MASK[]= {FTM_COMBINE_SYNCEN0_MASK,FTM_COMBINE_SYNCEN1_MASK,
				FTM_COMBINE_SYNCEN2_MASK,FTM_COMBINE_SYNCEN3_MASK};

		FTMs[instance]->COMBINE &= ~(DECAPEN_MASK[instance]|COMBINE_MASK[instance]);
		FTMs[instance]->COMBINE|=SYNCEN_MASK[instance];
		/* CPWMS = 1 (Para counting up & down)*/
		FTMs[instance]->SC |= FTM_SC_CPWMS_MASK;
		/*CnSC for high true pulses */

		/*MOD*/
		FTMs[instance]->MOD=config->mod;
		/*Set CnV*/
		FTMs[instance]->CONTROLS[config->channel].CnV=config->CnV;
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

		/* CPWMS = 0 (No habilito Centered PWM)*/
		FTMs[instance]->SC &= ~FTM_SC_CPWMS_MASK;

		/*CnSC for edge alligned con high true pulses */
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

		//uint8_t PS = (FTMs[instance]->SC>> FTM_SC_PS_SHIFT)&FTM_SC_PS_MASK;

		/*Set mod*/
		//uint16_t MOD = ( 50000000 / (config->PWMFreq*(1<<PS)) )-1;
		FTMs[instance]->MOD=config->mod;
		/*Set CnV*/
		//uint16_t CnV= (uint16_t)((MOD+1)*(config->dutyCyclePercent/100.0) );
		FTMs[instance]->CONTROLS[config->channel].CnV=config->CnV;
	}

	FTMs[instance]->CNTIN=0;
	//FTMs[instance]->SC |= FTM_SC_CLKS(clkSource);
	//FTMs[instance]->SC |=FTM_SC_CLKS(FTM_SYSTEM_CLOCK);
	return true; //cambiar
}
uint32_t FTM_GetCnVAddress(FTM_Instance instance,FTM_Channel channel)
{
	return (uint32_t)(&(FTMs[instance]->CONTROLS[channel].CnV));
}
//
void FTM_EnableClock(FTM_Instance instance)
{
	FTMs[instance]->SC |=FTM_SC_CLKS(FTM_SYSTEM_CLOCK);
}

void FTM_EnableOverflowInterrupt(FTM_Instance instance)
{
	FTMs[instance]->SC |= FTM_SC_TOIE_MASK;
}

void FTM_DisableOverflowInterrupt(FTM_Instance instance)
{
	FTMs[instance]->SC &= ~FTM_SC_TOIE_MASK;
}

uint16_t FTM_GetModValue(FTM_Instance instance)
{
	return (uint16_t) FTMs[instance]->MOD;
}

/*
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
*/
