#ifndef ADC_H_
#define ADC_H_

#include "stdint.h"


#define ADC_BIT_RESOLUTION 	(10)
#define ADC_RESOLUTION		(1024.0)
#define ADC_VCC				(3.3)
#define ADC_GND				(0.0)
#define ADC_OFFSET			(ADC_VCC/2.0)

typedef enum {ADC_0, ADC_1}ADC_Instance;
typedef enum {ADC_ChannelA,ADC_ChannelB} ADC_Channel;

/* ADC_SC1 enums*/
typedef enum{ADC_INTERRUPTS_DISABLED, ADC_INTERRUPTS_ENABLED}ADC_InterruptEnable;
typedef enum{ADC_SINGLE_ENDED, ADC_DIFFERENTIAL}ADC_SignalType;
typedef enum{ADC_IN_DADP0,
			ADC_IN_DADP1,
			ADC_IN_DADP2,
			ADC_IN_DADP3,
			ADC_IN_AD4,
			ADC_IN_AD5,
			ADC_IN_AD6,
			ADC_IN_AD7,
			ADC_IN_AD8,
			ADC_IN_AD9,
			ADC_IN_AD10,
			ADC_IN_AD11,
			ADC_IN_AD12,
			ADC_IN_AD13,
			ADC_IN_AD14,
			ADC_IN_AD15,
			ADC_IN_AD16,
			ADC_IN_AD17,
			ADC_IN_AD18,
			ADC_IN_AD19,
			ADC_IN_AD20,
			ADC_IN_AD21,
			ADC_IN_AD22,
			ADC_IN_AD23,
			ADC_reserved0,
			ADC_reserved1,
			ADC_TempSensor,
			ADC_BandGap,
			ADC_reserved2,
			ADC_Vrefsh,
			ADC_Vrefsl,
			ADC_Disabled}ADC_InputSignal;

/* ADC_SC2 enums*/
typedef enum{ADC_SOFTWARE_TRIGGER, ADC_HARDWARE_TRIGGER}ADC_Trigger;
typedef enum{ADC_DMA_DISABLED, ADC_DMA_ENABLED}ADC_DMAEnable;
typedef enum{ADC_REFV, AD_ALTV}ADC_VRefSel;


/* ADC_CFG1 enums*/
typedef enum{ADC_NORMAL_POWER, ADC_LOW_POWER}ADC_PowerConsumption;
typedef enum{NODIV, DIV2, DIV4, DIV8}ADC_ClkDiv;
typedef enum{ADC_SHORT_TIME, ADC_LONG_TIME}ADC_ConversionTime;
typedef enum{ADC_8OR9_BITS, ADC_12OR13_BITS, ADC_10OR11_BITS, ADC_16_BITS}ADC_Resolution;
typedef enum{ADC_BUS_CLK, ADC_HALF_BUS_CLK, ADC_ALTERNATE_CLK, ADC_ASYNCHRONOUS_CLK}ADC_InClk;


/* ADC_SC3 enums*/
typedef enum{ADC_HARDWARE_AVG_ON, ADC_HARDWARE_AVG_OFF}ADC_HardwareAverage;
typedef enum{ADC_4SAMPLES_AVG, ADC_8SAMPLES_AVG, ADC_16SAMPLES_AVG, ADC_32SAMPLES_AVG}ADC_AverageResolution;


typedef struct{
	/**< ADC_CFG1. */
	ADC_PowerConsumption PowerConsumtion;
	ADC_ClkDiv ClkDivider;
	ADC_ConversionTime ConversionTime;
	ADC_Resolution Resolution;
	ADC_InClk InternalClk;

	/**< ADC_SC2. */
	ADC_Trigger Trigger;
	ADC_DMAEnable DMAEnable;
	ADC_VRefSel VoltageReference;

	/**< ADC_SC3. */
	ADC_HardwareAverage HardwareAverage;
	ADC_AverageResolution AverageResolution;
}ADC_Config;


typedef struct
{
	/**< ADC_SC1. */
	ADC_InterruptEnable InterruptsEnable;
	ADC_SignalType SignalType;
	ADC_InputSignal InputChannel;
}ADC_ChannelConfig;

void ADC_GetDefaultConfig(ADC_Config * config);

void ADC_Init(ADC_Instance n, ADC_Config * config);

void ADC_GetDefaultChannelConfig(ADC_ChannelConfig * config);

void ADC_SetChannelConfig(ADC_Instance n,ADC_Channel m,ADC_ChannelConfig * config);

/**
 * @brief Return conversion value
 */
uint16_t ADC_GetConversionResult(ADC_Instance n,ADC_Channel m);

uint32_t ADC_GetDataResultAddress(ADC_Instance n,ADC_Channel m);

void ADC_EnableContinuousConv(ADC_Instance n);
void ADC_EnableInterrupts(ADC_Instance n,ADC_Channel m);
void ADC_SetHardwareTrigger(ADC_Instance n);


#endif /* ADC_H_ */
