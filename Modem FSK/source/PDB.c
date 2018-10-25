#include "PDB.h"
#include "GPIO.h"
#include "MK64F12.h"

#define DEFAULT_PDB_MOD		3788 //Given the bus clock at 50 (MHz) for MOD of 5000, the trigger frequency will be set at 10 (KHz).

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



	PDB0->SC = PDB_SC_LDMOD(config->loadValueMode) |
			PDB_SC_PRESCALER(config->prescalerDivider) |
			PDB_SC_TRGSEL(config->triggerInputSource) |
			PDB_SC_MULT(config->multiplicationFactor);

	if(config->enableContinuousMode==true)
		PDB0->SC |= PDB_SC_CONT_MASK;

	//Set MOD
	PDB0->MOD = config->MODValue;

	// Interrupt delay
	PDB0->IDLY = config->MODValue/2;

	PDB_Enable();

	PDB_LoadValues();

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



void PDB_Trigger(void)
{
	PDB0->SC |= PDB_SC_SWTRIG_MASK;
}

void PDB_LoadValues(void)
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



void PDB_SetChannelDelay(PDB_PreTrigger m, PDB_Channel n,  uint32_t CHDelay)
{
	ASSERT((n < 2) && (m < 2));
	PDB0->CH[n].DLY[m] = CHDelay;
}

uint32_t PDB_GetStatusFlags(void);

void PDB_ClearStatusFlags(uint32_t mask);

void PDB_SetModulusValue(uint16_t value)
{
	PDB0->MOD = PDB_MOD_MOD(value);
}

uint32_t PDB_GetCounterValue(void)
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

void PDB_EnableDACTrigger(bool enable)
{
	if(enable)
		PDB0->DAC[0].INTC |= PDB_INTC_TOE_MASK;
	else
		PDB0->DAC[0].INTC &= ~PDB_INTC_TOE_MASK;
}

void PDB_EnableADCTrigger(void)
{
	PDB0->CH[0].C1 = (1<<0)|(1<<8);//PDB_C1_EN_MASK | PDB_C1_TOS_MASK;
}

void PDB0_IRQHandler(void)
{
	digitalToggle(PORTNUM2PIN(PC,5));

	// Turn off flag
	PDB0->SC &= ~PDB_SC_PDBIF_MASK;
}
