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

#include "CircularBuffer.h"

#define BUFFER_SIZE		(100)

static ADC_Type * ADCs[] = ADC_BASE_PTRS;

#define FS		13200;
#define DELTA	6 //(ceil((446e-6)/(1/FS)));
#define DATA_BUFFER_SIZE	(256)


//NEW_CIRCULAR_BUFFER(xBuffer,BUFFER_SIZE,sizeof(sampleWord));
//NEW_CIRCULAR_BUFFER(mBuffer,BUFFER_SIZE,sizeof(sampleWord));
//NEW_CIRCULAR_BUFFER(dBuffer,BUFFER_SIZE,sizeof(sampleWord));

double h[] = {0.000184258321387766,	-0.00221281271600225,	-0.00875721735248610,	-0.0157935638369741,	-0.0125404257819552,
				0.0140848855293968,	0.0690100446059607,	0.140515735542082,	0.202192975479381,	0.226632240418417,
				0.202192975479381,	0.140515735542082,	0.0690100446059607,	0.0140848855293968,	-0.0125404257819552,
				-0.0157935638369741,	-0.00875721735248610,	-0.00221281271600225,	0.000184258321387766};
uint16_t filterNum = sizeof(h)/sizeof(h[0]);


void ADC_DefaultCallibration(ADC_Instance n);
void ADC_SetCallibration(ADC_Instance n);




void ADC_Init(ADC_Instance n, ADC_Config * config){
	ASSERT(n<FSL_FEATURE_SOC_ADC16_COUNT);

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

	/* Configuro para que la señal entre de forma single ended (DIFF = 0) y que el canal utilizado sea el DAP0 ADCH = (DAD0)*/
	ADCs[n]->SC1[0] = (config->InterruptsEnable) | (config->SignalType) | (config->InputChannel);


	/*Configuración de CLK source (ADCLK) y (ADIV), de resolución MODE, power consumption (ADLPC) y sampling time (ADLSMP)*/
	ADCs[n]->CFG1 = (config->PowerConsumtion) | (config->ClkDivider) | (config->ConversionTime) | (config->Resolution) | (config->InternalClk);

	/*Se configura para que la tensión de referencia sea VDDA REFSEL(AD_REFV), (Esta esta conectada a 3.3 (V) en la placa FRDM).
	 * Por otra parte se configra para que el inicio de la conversion sea triggereada por software ADTRG(0).*/
	ADCs[n]->SC2 = (config->Trigger) | (config->VoltageReference) | (config->DMAEnable);

	//Callibrate
	ADC_DefaultCallibration(n);

//	pinMode(PORTNUM2PIN(PC,5),OUTPUT);
}


void ADC_EnableContinuousConv(ADC_Instance n){
	// Enable continuous conversion
	ADCs[n]->SC3 = ADC_SC3_ADCO(1);
}

void ADC_EnableInterrupts(ADC_Instance n){
	// Enable ADC Interrupts
	ADCs[n]->SC1[0] |= ADC_SC1_AIEN(1);
}

void ADC_SetHardwareTrigger(ADC_Instance n){
	ADCs[n]->SC2 |=  ADC_SC2_ADTRG(1);
}


uint32_t ADC_GetDataResultAddress(ADC_Instance n){
	ASSERT(n < FSL_FEATURE_SOC_ADC16_COUNT);
	return (uint32_t)&(ADCs[n]->R[0]);
}



void ADC0_IRQHandler(void){

	uint16_t clearInterruptFlag = ADCs[0]->R[0];


	static uint16_t xBuffer[DELTA];
	static uint16_t mBuffer[DATA_BUFFER_SIZE];
	static uint16_t dBuffer[DATA_BUFFER_SIZE];

//	digitalToggle(PORTNUM2PIN(PC,5));

/*
	ASSERT(k<);
	uint16_t xn = numel(&xBuffer);

	if(xn > DELTA)
	{
		uint16_t xdata;
		ASSERT(!pop(&xBuffer, &xdata));

		uint16_t mi = (*(uint16_t)get(&xdata, (DELTA-1))) * xdata; //Aca debería poder mirar los elmentos dentro del buffer circular.
		ASSERT(!push(&mBuffer, &mi));
		uint16_t mn = numel(&mBuffer);
		if(mn > filterNum)
		{
			uint16_t di;
			for(int i = 0; i < filterNum; i++)
			{
				di += (*(uint16_t)get(&mBuffer, i)) * h[filterNum - i];
			}
			ASSERT(!push(&dBuffer, &di));

			uint16_t mdata;
			ASSERT(!pop(&mBuffer, &mdata));


			if(isFull(&dBuffer))
			{
				uint16_t ddata;
				ASSERT(!pop(&dBuffer, &ddata));
			}
		}
	}
*/
//	digitalToggle(PORTNUM2PIN(PC,5));


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



void ADC_GetDefaultConfig(ADC_Config * config){

	/* Configuro para que la señal entre de forma single ended (DIFF = 0) y que el canal utilizado sea el DAP0 ADCH = (DAD0)*/
	config->InterruptsEnable = ADC_SC1_AIEN(ADC_INTERRUPTS_DISABLED);
	config->SignalType = ADC_SC1_DIFF(ADC_SINGLE_ENDED);
	config->InputChannel = ADC_SC1_ADCH(ADC_IN_DAD0);


	/*Configuración de CLK source (ADCLK) y (ADIV), de resolución MODE, power consumption (ADLPC) y sampling time (ADLSMP)*/
	/*Divido el ADCLK por dos ADIV(DIV2),  teniendo en cuenta que el source es el CLK_BUS/2 ADICLK(ADICLK_BUS2), la frecuencia del ADCLK queda
	 * ADCLK = (BUS_CLK/2)*(1/2) = (50 (MHz)/2) * (1/2) = 12.5 (MHz). Si verificamos el datasheet del cortex, este valor esta dentro del rango
	 * permitido para el correcto funcionamiento del ADC.
	 * Por otra parte la resolución del conversor sera de 10 bits MODE(HIGHRES). Teniendo en cuenta que la frecuencia máxima a muestrear será
	 * de 2200 (Hz), esto da como resultado que en un período de la misma habra 5681 (cycles) del ADCLK, luego chequeando que la cantidad de
	 * (cycles) del ADCLK para una conversión del ADC no superará los 50 (cycles), esto nos deja con una cantidad de al menos 113 muestras por
	 * período de señal. Luego corroboramos que la configuración elegida da un margen adecuado para la aplicación que se budca.*/
	config->PowerConsumtion = ADC_CFG1_ADLPC(ADC_NORMAL_POWER);
	config->ClkDivider = ADC_CFG1_ADIV(DIV2);
	config->ConversionTime = ADC_CFG1_ADLSMP(ADC_SHORT_TIME);
	config->Resolution = ADC_CFG1_MODE(ADC_10OR11_BITS);
	config->InternalClk = ADC_CFG1_ADICLK(ADC_HALF_BUS_CLK);


	/*Se configura para que la tensión de referencia sea VDDA REFSEL(AD_REFV), (Esta esta conectada a 3.3 (V) en la placa FRDM).
	 * Por otra parte se configra para que el inicio de la conversion sea triggereada por software ADTRG(0)*/
	config->VoltageReference = ADC_SC2_REFSEL(ADC_REFV);
	config->Trigger = ADC_SC2_ADTRG(ADC_SOFTWARE_TRIGGER);
	config->DMAEnable = ADC_SC2_DMAEN(ADC_DMA_DISABLED);
}

void ADC_DefaultCallibration(ADC_Instance n){

	ADCs[n]->SC3 |= ADC_SC3_CAL_MASK;

	while(((ADCs[n]->SC3) & ADC_SC3_CAL_MASK) == ADC_SC3_CAL_MASK){}

	ASSERT(((ADCs[n]->SC3) & ADC_SC3_CALF_MASK) != ADC_SC3_CALF_MASK);

	ADC_SetCallibration(n);

	uint16_t readToClearCOCO = ADCs[n]->SC1;
}
void ADC_SetCallibration(ADC_Instance n){

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
