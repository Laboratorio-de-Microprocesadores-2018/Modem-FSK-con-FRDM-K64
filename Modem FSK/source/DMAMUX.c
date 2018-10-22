#include "hardware.h"
#include "DMAMUX.h"

void DMAMUX_Init ()
{
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
}
void DMAMUX_Deinit ()
{
	SIM->SCGC6 &= ~SIM_SCGC6_DMAMUX_MASK;
}
void DMAMUX_EnableChannel ( uint32_t channel, bool periodicTrigger)
{

	if (periodicTrigger)
	{
		DMAMUX->CHCFG[channel]|=DMAMUX_CHCFG_TRIG_MASK;
	}
	DMAMUX->CHCFG[channel]|=DMAMUX_CHCFG_ENBL_MASK;
}

void DMAMUX_DisableChannel (uint32_t channel)
{
	DMAMUX->CHCFG[channel] &= ~DMAMUX_CHCFG_ENBL_MASK;
}
void DMAMUX_SetSource ( uint32_t channel, uint8_t source)
{
	DMAMUX->CHCFG[channel]|=DMAMUX_CHCFG_SOURCE(source);
}
