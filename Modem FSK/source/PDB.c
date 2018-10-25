#include "PDB.h"
#include "GPIO.h"
#include "MK64F12.h"
#include "MK64F12_features.h"
#define DEFAULT_PDB_MOD		0xFFFF

void PDB_GetDefaultConfig(PDB_Config * config)
{
	config->loadValueMode = PDB_IMMEDIATELY;
    config->prescalerDivider = PDB_PRESCALER_DIVIDER_1;
    config->multiplicationFactor = PDB_MULT_FACTOR_1;
    config->triggerInputSource = PDB_TRIGGER_SOFTWARE;
    config->enableContinuousMode = true;
    config->MODValue = DEFAULT_PDB_MOD;
}

void PDB_Init(PDB_Config * config)
{
	// Enable clock
	SIM->SCGC6 |= SIM_SCGC6_PDB_MASK;

	PDB0->SC = 	PDB_SC_LDMOD(config->loadValueMode) |
				PDB_SC_PRESCALER(config->prescalerDivider) |
				PDB_SC_TRGSEL(config->triggerInputSource) |
				PDB_SC_MULT(config->multiplicationFactor);

	if(config->enableContinuousMode==true)
		PDB0->SC |= PDB_SC_CONT_MASK;

	//Set MOD
	PDB0->MOD = config->MODValue;

	// Do load values from buffered registers
	PDB0->SC |= PDB_SC_LDOK_MASK;

	// Enable PDB
	PDB0->SC |= PDB_SC_PDBEN_MASK;
}

void PDB_Deinit(void)
{
	// Disable PDB
	PDB0->SC &= ~PDB_SC_PDBEN_MASK;

	// Disable clock
	SIM->SCGC6 &= ~SIM_SCGC6_PDB_MASK;
}


void PDB_Enable(void)
{
	// Enable PDB
	PDB0->SC |= PDB_SC_PDBEN_MASK;
}

void PDB_Disable(void)
{
	// Disable PDB
	PDB0->SC &= ~PDB_SC_PDBEN_MASK;
}

void PDB_SoftwareTrigger(void)
{
	PDB0->SC |= PDB_SC_SWTRIG_MASK;
}

void PDB_DoLoadValues(void)
{
	PDB0->SC |= PDB_SC_LDOK_MASK;
}

void PDB_EnableDMA(bool enable)
{
	PDB0->SC |= PDB_SC_DMAEN_MASK;
}

void PDB_EnableInterrupts(uint32_t mask)
{
	NVIC_EnableIRQ(PDB0_IRQn);
	PDB0->SC |= PDB_SC_PDBIE_MASK;
}

void PDB_DisableInterrupts(uint32_t mask)
{
	PDB0->SC &= ~PDB_SC_PDBIE_MASK;
}

uint32_t PDB_GetStatusFlags(void)
{

}

void PDB_ClearStatusFlags(uint32_t mask)
{

}

void PDB_SetModulusValue(uint16_t value)
{
	PDB0->MOD = PDB_MOD_MOD(value);
}

uint32_t PDB_GetCounterValue(void)
{
	return (PDB0->CNT&PDB_CNT_CNT_MASK);
}

void PDB_SetInterruptDelay(uint32_t delay)
{
	PDB0->IDLY = PDB_IDLY_IDLY(delay);
}

void PDB_SetDACTriggerDelay(PDB_DACTrigger n,uint16_t delay)
{
	ASSERT(n<FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);

	// Disable external trigger
	PDB0->DAC[n].INTC &= ~PDB_INTC_EXT(0);
	// Set interval value for DAC trigger
	PDB0->DAC[n].INT = PDB_INT_INT(delay);
}

void PDB_EnableDACTrigger(PDB_DACTrigger n,bool enable)
{
	ASSERT(n<FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);

	if(enable)
		PDB0->DAC[n].INTC |= PDB_INTC_TOE_MASK;
	else
		PDB0->DAC[n].INTC &= ~PDB_INTC_TOE_MASK;
}

void PDB_SetADCTriggerDelay(PDB_Channel n, PDB_PreTrigger m,  uint32_t delay)
{
	ASSERT(n<FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);
	ASSERT(m<FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);

	PDB0->CH[n].DLY[m] = delay;
}

void PDB_EnableADCTrigger(PDB_Channel n, PDB_PreTrigger m, bool enable)
{
	ASSERT(n<FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);
	ASSERT(m<FSL_FEATURE_PDB_ADC_PRE_CHANNEL_COUNT);

	if(enable)
		PDB0->CH[n].C1 |= PDB_C1_EN(1<<n)|PDB_C1_TOS(1<<n);
	else
		PDB0->CH[n].C1 &= ~(PDB_C1_EN(1<<n)|PDB_C1_TOS(1<<n));
}

void PDB0_IRQHandler(void)
{
	digitalToggle(PORTNUM2PIN(PC,5));

	// Turn off flag
	PDB0->SC &= ~PDB_SC_PDBIF_MASK;
}
