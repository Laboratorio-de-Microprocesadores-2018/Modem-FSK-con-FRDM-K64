#include "ADC.h"
#include "Assert.h"
#include "hardware.h"
#include "GPIO.h"
#include "DAC.h"

//#include "CircularBuffer.h"

#define BUFFER_SIZE		(100)

static ADC_Type * ADCs[] = ADC_BASE_PTRS;

#define FS		13200;
#define DELTA	6 //(ceil((446e-6)/(1/FS)));
#define DATA_BUFFER_SIZE	(256)
typedef struct
{
	float data[DATA_BUFFER_SIZE];     /** Pointer to statically reserved memory array. */
//	char * const buffer_end; /** Pointer to end of the array. */
	uint16_t head;	         /** Pointer to the head of the buffer. */
	uint16_t tail;	         /** Pointer to the tail of the buffer. */
	uint16_t capacity;            /** Maximum number of elements in the buffer. */
	uint16_t count;               /** Number of elements in the buffer. */
//	uint16_t size;                /** Size of each element in the buffer. */
} MODEMCircularBuffer;



float h[] = {0.000184258321387766,	-0.00221281271600225,	-0.00875721735248610,	-0.0157935638369741,	-0.0125404257819552,
				0.0140848855293968,	0.0690100446059607,	0.140515735542082,	0.202192975479381,	0.226632240418417,
				0.202192975479381,	0.140515735542082,	0.0690100446059607,	0.0140848855293968,	-0.0125404257819552,
				-0.0157935638369741,	-0.00875721735248610,	-0.00221281271600225,	0.000184258321387766};

uint16_t filterNum = sizeof(h)/sizeof(h[0]);



/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////

static void ADC_DefaultCallibration(ADC_Instance n);
static void ADC_SetCallibration(ADC_Instance n);

bool MOEDM_BuffInit(MODEMCircularBuffer *this);
bool MODEM_BuffPush(MODEMCircularBuffer *this, float data);
bool MODEM_BuffPop(MODEMCircularBuffer *this,float *data);
void MODEM_BuffFlush(MODEMCircularBuffer * this);
uint16_t MODEM_BuffNumel(MODEMCircularBuffer *this);
bool MODEM_BuffIsEmpty(MODEMCircularBuffer *this);
bool MODEM_BuffIsFull(MODEMCircularBuffer *this);









/**
 *
 */

void ADC_Init(ADC_Instance n, ADC_Config * config)
{
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
	ADCs[n]->SC2 = ADC_SC2_ADTRG(config->Trigger) |
					ADC_SC2_REFSEL(config->VoltageReference) |
					ADC_SC2_DMAEN(config->DMAEnable);

	/*Se configura el Average por Hardware*/
	ADCs[n]->SC3 = ADC_SC3_AVGE(config->HardwareAverage) |
					ADC_SC3_AVGS(config->AverageResolution);


	// Configuración de CLK source (ADCLK) y (ADIV), de resolución MODE, power consumption (ADLPC) y sampling time (ADLSMP)*/
	ADCs[n]->CFG1 = ADC_CFG1_ADLPC(config->PowerConsumtion) |
					ADC_CFG1_ADIV(config->ClkDivider) |
					ADC_CFG1_ADLSMP(config->ConversionTime) |
					ADC_CFG1_MODE(config->Resolution) |
					ADC_CFG1_ADICLK(config->InternalClk);


	// Callibrate
	ADC_DefaultCallibration(n);


	// By default disable continuous conversion
	ADCs[n]->SC3 &= ~ADC_SC3_ADCO(1);

//	pinMode(PORTNUM2PIN(PC,5),OUTPUT);
}
void ADC_SetChannelConfig(ADC_Instance n,ADC_Channel m,ADC_ChannelConfig * config)
{
	ASSERT(m<FSL_FEATURE_ADC16_CONVERSION_CONTROL_COUNT);

	ADCs[n]->SC1[m] = ADC_SC1_AIEN(config->InterruptsEnable) |
					ADC_SC1_DIFF(config->SignalType) |
					ADC_SC1_ADCH(config->InputChannel);
}
uint16_t ADC_GetConversionResult(ADC_Instance n,ADC_Channel m)
{
	ASSERT(n < FSL_FEATURE_SOC_ADC16_COUNT);
	return ADCs[n]->R[m];
}
void ADC_EnableContinuousConv(ADC_Instance n)
{
	// Enable continuous conversion
	ADCs[n]->SC3 |= ADC_SC3_ADCO(1);
}
void ADC_EnableInterrupts(ADC_Instance n,ADC_Channel m)
{
	// Enable ADC Interrupts
	ADCs[n]->SC1[m] |= ADC_SC1_AIEN(1);
}
void ADC_SetHardwareTrigger(ADC_Instance n){
	ADCs[n]->SC2 |=  ADC_SC2_ADTRG(ADC_HARDWARE_TRIGGER);
}
void ADC_SetAverage(ADC_Instance n, ADC_AverageResolution res)
{
	ADCs[n]->SC3 = (ADC_SC3_AVGE_MASK) | res;
}
uint32_t ADC_GetDataResultAddress(ADC_Instance n,ADC_Channel m)
{
	ASSERT(n < FSL_FEATURE_SOC_ADC16_COUNT);
	return (uint32_t)&(ADCs[n]->R[m]);
}


void ADC0_IRQHandler(void){

	digitalToggle(PORTNUM2PIN(PC,5));
//
	static MODEMCircularBuffer xBuffer;
	static MODEMCircularBuffer mBuffer;
	static MODEMCircularBuffer dBuffer;
	if(MODEM_BuffIsEmpty(&xBuffer))
	{
		ASSERT(MOEDM_BuffInit(&xBuffer));
		ASSERT(MOEDM_BuffInit(&mBuffer));
		ASSERT(MOEDM_BuffInit(&dBuffer));
	}
//	if(mBuffer.count > 15)
//		digitalToggle(PORTNUM2PIN(PC,5));

	float xn = (float)ADCs[0]->R[0]/1024.0*3.3-1.65;

	ASSERT(MODEM_BuffPush(&xBuffer, xn));

	if(xBuffer.count > DELTA)
	{
		float mn = xBuffer.data[(xBuffer.tail + DELTA)%xBuffer.capacity] * xBuffer.data[xBuffer.tail]; //Aca debería poder mirar los elmentos dentro del buffer circular.
		ASSERT(MODEM_BuffPush(&mBuffer, mn));
		if((mBuffer.count) > filterNum)
		{
			float dn = 0;
			uint16_t firstSample = mBuffer.tail;

			for(uint16_t timeIndex = firstSample; timeIndex < firstSample + filterNum; timeIndex++)
				dn += ((mBuffer.data[(timeIndex)%xBuffer.capacity]) * (h[filterNum - (timeIndex - firstSample) - 1]));

			ASSERT(MODEM_BuffPush(&dBuffer, dn));

			float mdata;
			ASSERT(MODEM_BuffPop(&mBuffer, &mdata));
			if(MODEM_BuffIsFull(&dBuffer))
			{
				float ddata;
				ASSERT(MODEM_BuffPop(&dBuffer, &ddata));
			}
		}

		float xdata;
		ASSERT(MODEM_BuffPop(&xBuffer, &xdata));
	}

	float printVal = dBuffer.data[dBuffer.tail];
//	uint16_t printVal = xBuffer.data[xBuffer.tail];
	//DAC_WriteValue(DAC_0,(uint16_t)((printVal+1.65)*1024.0/3.3));
	if(printVal>0)
		DAC_WriteValue(DAC_0,0);
	else
		DAC_WriteValue(DAC_0,1024);
	digitalToggle(PORTNUM2PIN(PC,5));

}


void ADC_GetDefaultChannelConfig(ADC_ChannelConfig * config)
{
	config->InterruptsEnable = ADC_INTERRUPTS_DISABLED;
	config->SignalType = ADC_SINGLE_ENDED;
	config->InputChannel = ADC_IN_DADP0;

}
void ADC_GetDefaultConfig(ADC_Config * config)
{
	/*Configuración de CLK source (ADCLK) y (ADIV), de resolución MODE, power consumption (ADLPC) y sampling time (ADLSMP)*/
	/*Divido el ADCLK por dos ADIV(DIV2),  teniendo en cuenta que el source es el CLK_BUS/2 ADICLK(ADICLK_BUS2), la frecuencia del ADCLK queda
	 * ADCLK = (BUS_CLK/2)*(1/2) = (50 (MHz)/2) * (1/2) = 12.5 (MHz). Si verificamos el datasheet del cortex, este valor esta dentro del rango
	 * permitido para el correcto funcionamiento del ADC.
	 * Por otra parte la resolución del conversor sera de 10 bits MODE(HIGHRES). Teniendo en cuenta que la frecuencia máxima a muestrear será
	 * de 2200 (Hz), esto da como resultado que en un período de la misma habra 5681 (cycles) del ADCLK, luego chequeando que la cantidad de
	 * (cycles) del ADCLK para una conversión del ADC no superará los 50 (cycles), esto nos deja con una cantidad de al menos 113 muestras por
	 * período de señal. Luego corroboramos que la configuración elegida da un margen adecuado para la aplicación que se budca.*/
	config->PowerConsumtion = ADC_NORMAL_POWER;
	config->ClkDivider = DIV2;
	config->ConversionTime = ADC_SHORT_TIME;
	config->Resolution = ADC_10OR11_BITS;
	config->InternalClk = ADC_HALF_BUS_CLK;

	/*Se configura para que la tensión de referencia sea VDDA REFSEL(AD_REFV), (Esta esta conectada a 3.3 (V) en la placa FRDM).
	 * Por otra parte se configra para que el inicio de la conversion sea triggereada por software ADTRG(0)*/
	config->VoltageReference = ADC_REFV;
	config->Trigger = ADC_SOFTWARE_TRIGGER;
	config->DMAEnable = ADC_DMA_DISABLED;

	/*Configuro el Hardware Average*/
	config->HardwareAverage = ADC_HARDWARE_AVG_ON;
	config->AverageResolution = ADC_8SAMPLES_AVG;
}

void ADC_DefaultCallibration(ADC_Instance n)
{
	// Begin the callibration sequence
	ADCs[n]->SC3 |= ADC_SC3_CAL_MASK;

	// Wait until calibration ends
	while(((ADCs[n]->SC3) & ADC_SC3_CAL_MASK) == ADC_SC3_CAL_MASK);

	// Check callibration success
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



bool MOEDM_BuffInit(MODEMCircularBuffer *this)
{
	this->capacity = DATA_BUFFER_SIZE;
	this->count = 0;
	this->head = 0;
	this->tail = 0;

	return true;
}
bool MODEM_BuffPush(MODEMCircularBuffer *this, float data)
{
	if(this->count == this->capacity)
		return false;
	else
	{
		this->data[this->head] = data;
		this->head ++;
	    if(this->head == this->capacity)
	    	this->head = 0;
	    this->count++;
	    return true;
	}
}
bool MODEM_BuffPop(MODEMCircularBuffer *this,float *data)
{
	if(this->count == 0)
		return false;
	else
	{
		*data = this->data[this->tail];
		this->tail ++;
		if(this->tail == this->capacity)
			this->tail = 0;
		this->count--;
		return true;
	}
}
void MODEM_BuffFlush(MODEMCircularBuffer * this)
{
	this->head = this->tail;
	this->count = 0;
}
uint16_t MODEM_BuffNumel(MODEMCircularBuffer *this)
{
	return this->count;
}
bool MODEM_BuffIsEmpty(MODEMCircularBuffer *this)
{
	return (this->count == 0);
}
bool MODEM_BuffIsFull(MODEMCircularBuffer *this)
{
	return (this->count == this->capacity);
}
