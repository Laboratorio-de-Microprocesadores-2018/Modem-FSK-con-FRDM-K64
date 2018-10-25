/*
 * ADC.h
 *
 *  Created on: Oct 17, 2018
 *      Author: sebas
 */

#ifndef ADC_H_
#define ADC_H_

#include "stdint.h"

typedef enum {ADC_0,
			  ADC_1}ADC_Instance;

/* ADC_SC1 enums*/
typedef enum{DAD0, DAD1, DAD2, DAD3, AD4, AD5, AD6, AD7, AD8, AD9,
			AD10, AD11, AD12, AD13, AD14, AD15, AD16, AD17, AD18, AD19,
			AD20, AD21, AD22, AD23}ADC_Channel;

/* ADC_CFG1 enums*/
typedef enum{LOWRES, MIDRES, HIGHRES, MESSIRES}ADC_Resolution;
typedef enum{NODIV, DIV2, DIV4, DIV8}ClkDiv;
typedef enum{ADICLK_BUS, ADICLK_BUS2, ADICLK_ALT, ADICLK_ASYNC}ADIClk;

/* ADC_SC2 enums*/
typedef enum{AD_REFV, AD_ALTV}RefSel;

typedef struct
{
	ADC_Resolution

}ADC_Config;

void ADC_Init(ADC_Instance n);
uint32_t ADC_GetDataResultAddress(ADC_Instance n);
void ADC_enableContinuousConv(ADC_Instance n);
void ADC_enableInterrupts(ADC_Instance n);
void ADC_setHardwareTrigger(ADC_Instance n);

#endif /* ADC_H_ */
