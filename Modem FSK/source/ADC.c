/*
 * ADC.c
 *
 *  Created on: Oct 17, 2018
 *      Author: sebas
 */


#include "ADC.h"
#include "Assert.h"
#include "hardware.h"


void ADC_config(void);
uint8_t ADC_callibrate(void);
void ADC_setCallibration(void);

void ADC_Init(void)
{
	// Enable clock gating
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	//Enable NVIC
	NVIC_EnableIRQ(ADC0_IRQn);

	//Configure
	ADC_config();

	//Callibrate
	ASSERT(ADC_callibrate());
	ADC_setCallibration();

	/*Enable continuous conversion*/
	ADC0->SC3 = ADC_SC3_ADCO(1);


	// Enable ADC Interrupts
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);
}

void ADC0_IRQHandler(void){

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

void ADC_config(void){

	/* Configuro para que la señal entre de forma single ended (DIFF = 0) y que el canal utilizado sea el DAP0 ADCH = (DAD0)*/
	ADC0->SC1[0] = ADC_SC1_DIFF(0) | ADC_SC1_ADCH(DAD0);


	/*Configuración de CLK source (ADCLK) y (ADIV), de resolución MODE, power consumption (ADLPC) y sampling time (ADLSMP)*/
	/*Divido el ADCLK por dos ADIV(DIV2),  teniendo en cuenta que el source es el CLK_BUS/2 ADICLK(ADICLK_BUS2), la frecuencia del ADCLK queda
	 * ADCLK = (BUS_CLK/2)*(1/2) = (50 (MHz)/2) * (1/2) = 12.5 (MHz). Si verificamos el datasheet del cortex, este valor esta dentro del rango
	 * permitido para el correcto funcionamiento del ADC.
	 * Por otra parte la resolución del conversor sera de 10 bits MODE(HIGHRES). Teniendo en cuenta que la frecuencia máxima a muestrear será
	 * de 2200 (Hz), esto da como resultado que en un período de la misma habra 5681 (cycles) del ADCLK, luego chequeando que la cantidad de
	 * (cycles) del ADCLK para una conversión del ADC no superará los 50 (cycles), esto nos deja con una cantidad de al menos 113 muestras por
	 * período de señal. Luego corroboramos que la configuración elegida da un margen adecuado para la aplicación que se budca.*/
	ADC0->CFG1 = ADC_CFG1_ADLPC(0) | ADC_CFG1_ADIV(DIV2) | ADC_CFG1_ADLSMP(0) | ADC_CFG1_MODE(HIGHRES) | ADC_CFG1_ADICLK(ADICLK_BUS2);

	/*Se configura para que la tensión de referencia sea VDDA REFSEL(AD_REFV), (Esta esta conectada a 3.3 (V) en la placa FRDM).
	 * Por otra parte se configra para que el inicio de la conversion sea triggereada por software ADTRG(0)*/
	ADC0->SC2 = ADC_SC2_REFSEL(AD_REFV) | ADC_SC2_ADTRG(0);
	/*Activo las DMA requests*/
	//ADC0->SC2 =  ADC_SC2_DMAEN(1);
}

uint8_t ADC_callibrate(void){

	ADC0->SC3 |= ADC_SC3_CAL_MASK;

	while(((ADC0->SC3) & ADC_SC3_CAL_MASK) == ADC_SC3_CAL_MASK){}

	if(((ADC0->SC3) & ADC_SC3_CALF_MASK) == ADC_SC3_CALF_MASK)
		return 0;
	else
		return 1;
}

void ADC_setCallibration(void){

	uint16_t PGVar, MGVar;

	PGVar = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
	MGVar = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;

	PGVar = PGVar >> 1;
	MGVar = MGVar >> 1;

	PGVar |= 0x80;
	MGVar |= 0x80;

	ADC0->PG = PGVar;
	ADC0->MG = MGVar;
}
