#include "PDB.h"
#include "MK64F12.h"


void PDB_GetDefaultConfig(PDB_Config * config)
{
	config->loadValueMode = PDB_IMMEDIATELY;
    config->prescalerDivider = PDB_PRESCALER_DIVIDER_1;
    config->multiplicationFactor = PDB_MULT_FACTOR_1;
    config->triggerInputSource = PDB_TRIGGER_SOFTWARE;
    config->enableContinuousMode = false;
}

void PDB_Init(PDB_Config * config)
{
	// Enable clock
	SIM->SCGC6 |= SIM_SCGC6_PDB_MASK;

	PDB0->SC = PDB_SC_LDMOD(config->loadValueMode) |
			PDB_SC_PRESCALER(config->prescalerDivider) |
			PDB_SC_TRGSEL(config->triggerInputSource) |
			PDB_SC_MULT(config->multiplicationFactor);

	if(config->enableContinuousMode==true)
		PDB0->SC |= PDB_SC_CONT_MASK;

	// Enable PDB
	PDB0->SC |= PDB_SC_PDBEN_MASK;
}

void PDB_Deinit()
{
	// Disable PDB
	PDB0->SC &= ~PDB_SC_PDBEN_MASK;

	// Disable clock
	SIM->SCGC6 &= ~SIM_SCGC6_PDB_MASK;
}

void PDB_Enable()
{
	// Enable PDB
	PDB0->SC |= PDB_SC_PDBEN_MASK;
}

void PDB_Disable()
{
	// Disable PDB
	PDB0->SC &= ~PDB_SC_PDBEN_MASK;

}

void PDB_Trigger()
{
	PDB0->SC |= PDB_SC_SWTRIG_MASK;
}

void PDB_LoadValues()
{
	PDB0->SC |= PDB_SC_LDOK_MASK;
}

void PDB_EnableDMA(bool enable)
{
	PDB0->SC |= PDB_SC_DMAEN_MASK;
}

void PDB_EnableInterrupts(uint32_t mask)
{
	PDB0->SC |= PDB_SC_PDBIE_MASK;
}

void PDB_DisableInterrupts(uint32_t mask)
{
	PDB0->SC &= ~PDB_SC_PDBIE_MASK;
}


uint32_t PDB_GetStatusFlags();

void PDB_ClearStatusFlags(uint32_t mask);

void PDB_SetModulusValue(uint32_t value)
{
	PDB0->MOD = PDB_MOD_MOD(value);
}

uint32_t PDB_GetCounterValue()
{
	return (PDB0->CNT&PDB_CNT_CNT_MASK);
}

void PDB_SetCounterDelayValue(uint32_t value)
{
	PDB0->IDLY = PDB_IDLY_IDLY(value);
}

void PDB_SetDacTriggerPeriod(uint16_t value)
{
	// Disable external trigger
	PDB0->DAC[0].INTC &= ~PDB_INTC_EXT(0);
	// Set interval value for DAC trigger
	PDB0->DAC[0].INT = PDB_INT_INT(value);
}

void PDB_EnableDacTriger(bool enable)
{
	if(enable)
		PDB0->DAC[0].INTC |= PDB_INTC_TOE_MASK;
	else
		PDB0->DAC[0].INTC &= ~PDB_INTC_TOE_MASK;
}
