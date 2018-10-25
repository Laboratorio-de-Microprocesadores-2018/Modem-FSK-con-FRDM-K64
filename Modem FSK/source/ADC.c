/*
 * ADC.c
 *
 *  Created on: Oct 17, 2018
 *      Author: sebas
 */


#include "ADC.h"
#include "Assert.h"
#include "hardware.h"
#include "GPIO.h"

static ADC_Type * ADCs[] = ADC_BASE_PTRS;

void ADC_defaultConfig(ADC_Instance n);
void ADC_defaultCallibration(ADC_Instance n);
void ADC_setCallibration(ADC_Instance n);

void ADC_Init(ADC_Instance n)
{
	// Enable clock gating and NVIC
	if(n==ADC_0)
	{
		SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
		NVIC_EnableIRQ(ADC0_IRQn);
	}
	else if (n==ADC_1)
	{
		SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;
		NVIC_EnableIRQ(ADC1_IRQn);
	}

	//Configure
	ADC_defaultConfig(n);

	//Callibrate
	ADC_defaultCallibration(n);

	// Enable continuous conversion
	//ADC_enableContinuousConv(n);


	// Enable ADC Interrupts
	//ADC_enableInterrupts(n);

	pinMode(PORTNUM2PIN(PC,5),OUTPUT);
}

uint32_t ADC_GetDataResultAddress(ADC_Instance n)
{
	ASSERT(n < FSL_FEATURE_SOC_ADC16_COUNT);
	return (uint32_t)&(ADCs[n]->R[0]);
}

void ADC_enableContinuousConv(ADC_Instance n)
{
	// Enable continuous conversion
	ADCs[n]->SC3 = ADC_SC3_ADCO(1);
}

void ADC_enableInterrupts(ADC_Instance n)
{
	// Enable ADC Interrupts
	ADCs[n]->SC1[0] |= ADC_SC1_AIEN(1);
}

void ADC_setHardwareTrigger(ADC_Instance n)
{
	ADCs[n]->SC2 |=  ADC_SC2_ADTRG(1);
}

void ADC0_IRQHandler(void)
{

	uint32_t test = ADCs[0]->SC2, test2 = ADCs[0]->SC3, test3 = ADCs[0]->R[0];
	digitalToggle(PORTNUM2PIN(PC,5));
/*
	static uint16_t result[100];
	static int i, prevResult;
	int newResult;

	newResult = ADC0->R[0];

	if(i == 0)
	{
		prevResult = ADC0->R[0];
		i++;
	}
	else if(((prevResult + 1) < newResult) || ((prevResult - 1) > newResult))
	{
		prevResult = newResult;
		result[i] = newResult;

		i++;

		if(i == 99)
			i=0;
	}

	//ASSERT(((ADC0->SC1[0]) & ADC_SC1_COCO_MASK) == ADC_SC1_COCO_MASK);

	/*Write to the SC1A register*/
	//ADC0->SC1[0] |= ADC_SC1_AIEN(1);

}

void ADC_defaultConfig(ADC_Instance n)
{

	/* Configuro para que la señal entre de forma single ended (DIFF = 0) y que el canal utilizado sea el DAP0 ADCH = (DAD0)*/
	ADCs[n]->SC1[0] = ADC_SC1_DIFF(0) | ADC_SC1_ADCH(DAD0);


	/*Configuración de CLK source (ADCLK) y (ADIV), de resolución MODE, power consumption (ADLPC) y sampling time (ADLSMP)*/
	/*Divido el ADCLK por dos ADIV(DIV2),  teniendo en cuenta que el source es el CLK_BUS/2 ADICLK(ADICLK_BUS2), la frecuencia del ADCLK queda
	 * ADCLK = (BUS_CLK/2)*(1/2) = (50 (MHz)/2) * (1/2) = 12.5 (MHz). Si verificamos el datasheet del cortex, este valor esta dentro del rango
	 * permitido para el correcto funcionamiento del ADC.
	 * Por otra parte la resolución del conversor sera de 10 bits MODE(HIGHRES). Teniendo en cuenta que la frecuencia máxima a muestrear será
	 * de 2200 (Hz), esto da como resultado que en un período de la misma habra 5681 (cycles) del ADCLK, luego chequeando que la cantidad de
	 * (cycles) del ADCLK para una conversión del ADC no superará los 50 (cycles), esto nos deja con una cantidad de al menos 113 muestras por
	 * período de señal. Luego corroboramos que la configuración elegida da un margen adecuado para la aplicación que se budca.*/
	ADCs[n]->CFG1 = ADC_CFG1_ADLPC(0) | ADC_CFG1_ADIV(DIV2) | ADC_CFG1_ADLSMP(0) | ADC_CFG1_MODE(HIGHRES) | ADC_CFG1_ADICLK(ADICLK_BUS2);

	/*Se configura para que la tensión de referencia sea VDDA REFSEL(AD_REFV), (Esta esta conectada a 3.3 (V) en la placa FRDM).
	 * Por otra parte se configra para que el inicio de la conversion sea triggereada por software ADTRG(0)*/
	ADCs[n]->SC2 = ADC_SC2_REFSEL(AD_REFV) | ADC_SC2_ADTRG(0);

	/*Activo los DMA requests*/
	ADCs[n]->SC2 |=  ADC_SC2_DMAEN(1);
}

void ADC_defaultCallibration(ADC_Instance n)
{

	ADCs[n]->SC3 |= ADC_SC3_CAL_MASK;

	while(((ADCs[n]->SC3) & ADC_SC3_CAL_MASK) == ADC_SC3_CAL_MASK){}

	ASSERT(((ADCs[n]->SC3) & ADC_SC3_CALF_MASK) != ADC_SC3_CALF_MASK);

	ADC_setCallibration(n);

	uint16_t readToClearCOCO = ADCs[n]->SC1;
}

void ADC_setCallibration(ADC_Instance n){

	uint16_t PGVar, MGVar;

	PGVar = ADCs[n]->CLP0 + ADCs[n]->CLP1 + ADCs[n]->CLP2 + ADCs[n]->CLP3 + ADCs[n]->CLP4 + ADCs[n]->CLPS;
	MGVar = ADCs[n]->CLM0 + ADCs[n]->CLM1 + ADCs[n]->CLM2 + ADCs[n]->CLM3 + ADCs[n]->CLM4 + ADCs[n]->CLMS;

	PGVar = PGVar >> 1;
	MGVar = MGVar >> 1;

	PGVar |= 0x80;
	MGVar |= 0x80;

	ADCs[n]->PG = PGVar;
	ADCs[n]->MG = MGVar;
}
