#include "FTM.h"
#include "MK64F12.h"
#include "Assert.h"
#include "MK64F12_features.h"

#include "GPIO.h"


#define MEASURE_CPU_TIME
#ifdef MEASURE_CPU_TIME
	#include "hardware.h"
	#define MEASURE_CPU_TIME_PORT PORTC
	#define MEASURE_CPU_TIME_GPIO GPIOC
	#define MEASURE_CPU_TIME_PIN	9
	#define SET_TEST_PIN BITBAND_REG(MEASURE_CPU_TIME_GPIO->PDOR, MEASURE_CPU_TIME_PIN) = 1
	#define CLEAR_TEST_PIN BITBAND_REG(MEASURE_CPU_TIME_GPIO->PDOR, MEASURE_CPU_TIME_PIN) = 0
#else
	#define SET_TEST_PIN
	#define CLEAR_TEST_PIN
#endif



#define FTM_CHANNELS 8
static FTM_Type * FTMs[] = FTM_BASE_PTRS;
static FTMCaptureFun_t FTM_ICCallback[FTM_CHANNELS];


void FTM_Init(FTM_Instance instance, FTM_Config * config)
{

#ifdef MEASURE_CPU_TIME
	MEASURE_CPU_TIME_PORT->PCR[MEASURE_CPU_TIME_PIN] = PORT_PCR_MUX(1);
	MEASURE_CPU_TIME_GPIO->PDDR |= (1<<MEASURE_CPU_TIME_PIN);
	MEASURE_CPU_TIME_GPIO->PDOR &= ~(1<<MEASURE_CPU_TIME_PIN);
#endif


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

	NVIC_EnableIRQ(FTM1_IRQn);
	//FTM->SC = FTM_SC_CLKS(config->clockSource) | FTM_SC_PS(config->prescale);


	/*NO DARLE CLOCK SOURCE AL FTM HASTA QUE ESTE LISTO PARA ANDAR!!!!!!!!!!!!!*/
	//FTM->SC = FTM_SC_CLKS(config->clockSource) | FTM_SC_PS(config->prescale);

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

	// QUADEN = 0
	FTMs[instance]->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;

	uint32_t COMBINE_MASK[]= {FTM_COMBINE_COMBINE0_MASK,FTM_COMBINE_COMBINE1_MASK,
								FTM_COMBINE_COMBINE2_MASK,FTM_COMBINE_COMBINE3_MASK};
	uint32_t DECAPEN_MASK[] = {FTM_COMBINE_DECAPEN0_MASK,FTM_COMBINE_DECAPEN1_MASK,
			FTM_COMBINE_DECAPEN2_MASK,FTM_COMBINE_DECAPEN3_MASK};
	uint32_t SYNCEN_MASK[]= {FTM_COMBINE_SYNCEN0_MASK,FTM_COMBINE_SYNCEN1_MASK,
			FTM_COMBINE_SYNCEN2_MASK,FTM_COMBINE_SYNCEN3_MASK};

	//DECAPEN = 0 y COMBINE = 0
	FTMs[instance]->COMBINE &= ~(DECAPEN_MASK[instance]|COMBINE_MASK[instance]);
	FTMs[instance]->COMBINE|=SYNCEN_MASK[instance];

	if(config->mode == FTM_PWM_CENTER_ALIGNED)
	{
		/*CnSC*/
		FTMs[instance]->CONTROLS[config->channel].CnSC|=FTM_CnSC_ELSB_MASK;
		/* CPWMS = 1 (Para counting up & down)*/
		FTMs[instance]->SC |= FTM_SC_CPWMS_MASK;
	}
	else if(config->mode == FTM_PWM_EDGE_ALIGNED)
	{
		/* CPWMS = 0 (No habilito Centered PWM)*/
		FTMs[instance]->SC &= ~FTM_SC_CPWMS_MASK;
		/*CnSC for edge alligned con high true pulses */
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
		/*uint16_t MOD = ( 50000000 / (config->PWMFreq*(1<<PS)) )-1;
		//uint16_t CnV= (uint16_t)((MOD+1)*(config->dutyCyclePercent/100.0) );*/
	}
	/*MOD*/
	FTMs[instance]->MOD=config->mod;
	/*Set CnV*/
	FTMs[instance]->CONTROLS[config->channel].CnV=config->CnV;
	FTMs[instance]->CNTIN=0;

	return true;
}
uint32_t FTM_GetCnVAddress(FTM_Instance instance,FTM_Channel channel)
{
	return (uint32_t)(&(FTMs[instance]->CONTROLS[channel].CnV));
}

bool FTM_SetupInputCapture(FTM_Instance instance,FTM_InputCaptureConfig *config)
{

	FTMs[instance]->CONTROLS[config->channel].CnSC= FTM_CnSC_ELSA(config->mode&1)|FTM_CnSC_ELSB((config->mode <<1)&1);

	if(config->channel<=3)
	{
	uint32_t FILTER_MASK_TABLE[]={FTM_FILTER_CH0FVAL(config->filterValue),FTM_FILTER_CH1FVAL(config->filterValue),
			FTM_FILTER_CH2FVAL(config->filterValue),FTM_FILTER_CH3FVAL(config->filterValue)};

	FTMs[instance]->FILTER|=FILTER_MASK_TABLE[config->channel];//CHANNEL 0
	}

	FTM_ICCallback[config->channel]=config->callback;

	if(config->enableDMA == true)
	{
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_DMA_MASK|FTM_CnSC_CHIE_MASK;
	}
	else
	{
		(FTMs[instance]->CONTROLS[config->channel].CnSC) &= ~FTM_CnSC_DMA_MASK;
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_CHIE_MASK;
	}
	FTMs[instance]->CNTIN=0;
	/*MOD*/
	FTMs[instance]->MOD=config->mod;
	return true;
}

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


void FTM_ClearCount(FTM_Instance instance)
{
	FTMs[instance]->CNT=0;
}


/*
void FTM0_IRQHandler(void)
{
	if(FTM0->SC & FTM_SC_TOF_MASK)
	{
		FTM0->SC &= ~FTM_SC_TOF_MASK;

		GPIOB->PTOR = 1<<18;
	}
}*/
void FTM1_IRQHandler(void)
{
	SET_TEST_PIN;

	FTMs[FTM_1]->CONTROLS[0].CnSC &=  ~FTM_CnSC_CHF_MASK;
	FTM_ICCallback[0](FTMs[FTM_1]->CONTROLS[0].CnV);

	CLEAR_TEST_PIN;
}

/*
void FTM2_IRQHandler(void)
{

}
*/
