#include "PORT.h"
#include "MK64F12.h"
#include "MK64F12_features.h"
#include "Assert.h"


static PORT_Type * ports[] = PORT_BASE_PTRS;

void PORT_GetPinDefaultConfig(PORT_Config * config)
{
	config->ds = PORT_LowDriveStrength;
	config->filter = PORT_PassiveFilterDisable;
	config->interrupt = PORT_InterruptOrDMADisabled;
	config->lk =  PORT_UnlockRegister;
	config->mux = PORT_MuxAlt0;
	config->od = PORT_OpenDrainDisable;
	config->pull = PORT_PullDisable;
	config->sr = PORT_FastSlewRate;
}

void 	PORT_PinConfig (PORT_Instance n, uint32_t pin, PORT_Config *config)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);

	// Clock gating
	uint32_t PORTS_SCG[] = {SIM_SCGC5_PORTA_MASK,SIM_SCGC5_PORTB_MASK,SIM_SCGC5_PORTC_MASK,SIM_SCGC5_PORTD_MASK,SIM_SCGC5_PORTE_MASK};
	SIM->SCGC5 |= PORTS_SCG[n];

	ports[n]->PCR[pin] = 	PORT_PCR_SRE(config->sr) | 
							PORT_PCR_PFE(config->filter) | 
							PORT_PCR_ODE(config->od) | 
							PORT_PCR_DSE(config->ds) | 
							PORT_PCR_PS (config->pull) | 
							PORT_PCR_MUX(config->mux) | 
							PORT_PCR_LK (config->lk) | 
						    PORT_PCR_IRQC(config->interrupt);

	// Enable or disable internal pull resistor
	if(config->pull == PORT_PullDisable)
		ports[n]->PCR[pin] &= ~PORT_PCR_PE_MASK;
	else
		ports[n]->PCR[pin] |= PORT_PCR_PE_MASK;

}

void PORT_MultiplePinsConfig (PORT_Instance n, uint32_t mask, PORT_Config *config)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);

	uint32_t PCR = 	PORT_PCR_SRE(config->sr) | 
					PORT_PCR_PFE(config->filter) | 
					PORT_PCR_ODE(config->od) | 
					PORT_PCR_DSE(config->ds) | 
					PORT_PCR_PS (config->pull) | 
					PORT_PCR_MUX(config->mux) | 
					PORT_PCR_LK (config->lk) | 
					PORT_PCR_IRQC(config->interrupt);

	ports[n]->GPCLR = PORT_GPCLR_GPWD(PCR) | PORT_GPCLR_GPWE(mask);
	ports[n]->GPCHR = PORT_GPCLR_GPWD(PCR) | PORT_GPCLR_GPWE(mask);

}

void 	PORT_PinMux (PORT_Instance n, uint32_t pin, PORT_Mux mux)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);

	ports[n]->PCR[pin] &= ~PORT_PCR_MUX_MASK;
	ports[n]->PCR[pin] |= PORT_PCR_MUX(mux);
}

void    PORT_PinInterruptConfig (PORT_Instance n, uint32_t pin, PORT_Interrupt interrupt)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);

	ports[n]->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
	ports[n]->PCR[pin] |= PORT_PCR_IRQC(interrupt);
}

uint32_t PORT_GetPinInterruptFlag (PORT_Instance n,uint32_t pin)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);
	return (ports[n]->ISFR>>pin)&1;
}

uint32_t PORT_GetPinsInterruptFlags (PORT_Instance n)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);
	return ports[n]->ISFR;
}

void 	PORT_ClearPinInterruptFlag (PORT_Instance n, uint32_t pin)
{
	ASSERT(n<FSL_FEATURE_SOC_PORT_COUNT);
	//W1C????

}
