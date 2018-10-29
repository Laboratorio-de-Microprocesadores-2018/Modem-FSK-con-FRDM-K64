/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "FTM.h"
#include "hardware.h"
#include "Assert.h"
#include "CPUTimeMeasurement.h"


/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

#define FTM_CHANNELS 8

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////
static FTM_Type * FTMs[] = FTM_BASE_PTRS;
static FTMCaptureFun_t FTM_ICCallback[FTM_CHANNELS];

/////////////////////////////////////////////////////////////////////////////////
//                   	Local functions and global Services  			       //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes FTM module
 * @param instance to be used
 * @param configuration struct
 */
void FTM_Init(FTM_Instance instance, FTM_Config * config)
{

	ASSERT(instance < FSL_FEATURE_SOC_FTM_COUNT);
	//Clocking
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
	//Prescaler
	FTM->SC |= FTM_SC_PS(config->prescale);
	//NVIC IRQ enables
	NVIC_ClearPendingIRQ(FTM0_IRQn);
	NVIC_EnableIRQ(FTM0_IRQn);

	NVIC_EnableIRQ(FTM1_IRQn);
}
/**
 * @brief configuration function for PWM generation
 * @param instance FTM instance used
 * @param config , configuration structure
 */
bool FTM_SetupPwm(FTM_Instance 	instance, FTM_PwmConfig * config)
{
	ASSERT(instance < FSL_FEATURE_SOC_FTM_COUNT);

	//						General FTM Configuration
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

	//						Specific EPWM o CPWM configuration
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
	}
	/*MOD*/
	FTMs[instance]->MOD=config->mod;
	/*Set CnV*/
	FTMs[instance]->CONTROLS[config->channel].CnV=config->CnV;
	FTMs[instance]->CNTIN=0;

	return true;
}

void FTM_EnableInterrupts(FTM_Instance 	instance,FTM_Channel channel)
{
	NVIC_EnableIRQ(FTM1_IRQn);
}

void FTM_DisableInterrupts(FTM_Instance 	instance,FTM_Channel channel)
{
	NVIC_DisableIRQ(FTM1_IRQn);
}
/**
 * @brief Gives a pointer to the address of the CnV register for the specified instance
 * and channel.
 * @param instance FTM instance used
 * @param channel
 *  */
uint32_t FTM_GetCnVAddress(FTM_Instance instance,FTM_Channel channel)
{
	return (uint32_t)(&(FTMs[instance]->CONTROLS[channel].CnV));
}

/**
 * @brief configuration function for input capture
 * @param instance FTM instance used
 * @param config , configuration structure
 */
bool FTM_SetupInputCapture(FTM_Instance instance,FTM_InputCaptureConfig *config)
{

	//INPUT CAPTURE
	FTMs[instance]->CONTROLS[config->channel].CnSC= FTM_CnSC_ELSA(config->mode&1)|FTM_CnSC_ELSB((config->mode <<1)&1);

	//FILTER
	if(config->channel<=3)
	{
	uint32_t FILTER_MASK_TABLE[]={FTM_FILTER_CH0FVAL(config->filterValue),FTM_FILTER_CH1FVAL(config->filterValue),
			FTM_FILTER_CH2FVAL(config->filterValue),FTM_FILTER_CH3FVAL(config->filterValue)};
	FTMs[instance]->FILTER|=FILTER_MASK_TABLE[config->channel];//CHANNEL 0
	}
	//CALLBACK
	FTM_ICCallback[config->channel]=config->callback;
	//DMA
	if(config->enableDMA == true)
	{
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_DMA_MASK|FTM_CnSC_CHIE_MASK;
	}
	else
	{
		(FTMs[instance]->CONTROLS[config->channel].CnSC) &= ~FTM_CnSC_DMA_MASK;
		FTMs[instance]->CONTROLS[config->channel].CnSC |= FTM_CnSC_CHIE_MASK;
	}
	/*INIT COUNT =0*/
	FTMs[instance]->CNTIN=0;
	/*MOD*/
	FTMs[instance]->MOD=config->mod;
	return true;
}

/**
 * @brief Enables clock for the FTM instance selected
 * @param instance
 */
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

/**
 * @brief Sets to 0 the count of the selected FTM instance
 * @param instance FTM instance used
 *  */
void FTM_ClearCount(FTM_Instance instance)
{
	FTMs[instance]->CNT=0;
}



/*FTM IRQ handler, calls its respective callback*/
void FTM1_IRQHandler(void)
{
	SET_TEST_PIN;

	FTMs[FTM_1]->CONTROLS[0].CnSC &=  ~FTM_CnSC_CHF_MASK;
	FTM_ICCallback[0](FTMs[FTM_1]->CONTROLS[0].CnV);

	CLEAR_TEST_PIN;
}

