#ifndef STARTUP_CMP_H_
#define STARTUP_CMP_H_
typedef enum{CMP_0,CMP_1,CMP_2}CMP_Instance;
typedef enum{CMP_IN0,CMP_IN1,CMP_IN2,CMP_IN3,CMP_IN4,CMP_IN5,CMP_IN6,CMP_IN7}CMP_Input;
typedef enum{CMP_OUT_FTM1_CH0,CMP_OUT_FTM2_CH0}CMP_Output;

/*
		   CMP0 						 CMP1 						CMP2
IN0		CMP0_IN0 					CMP1_IN0 					CMP2_IN0
IN1		CMP0_IN1 					CMP1_IN1 					CMP2_IN1
IN2		CMP0_IN2 					ADC0_SE16/CMP1_IN21 		ADC1_SE16/CMP2_IN21
IN3		CMP0_IN3 					12-bit DAC0_OUT/CMP1_IN3 	12-bit DAC1_OUT/CMP2_IN31
IN4		12-bit DAC1_OUT/CMP0_IN4 	— 							—
IN5		VREF Output/CMP0_IN5 		VREF Output/CMP1_IN5 —
IN6		Bandgap						Bandgap 					Bandgap
IN7		6b DAC0 Reference 			6b DAC1 Reference 			—
*/
typedef enum{CMP_HysteresisLevel0,CMP_HysteresisLevel1,CMP_HysteresisLevel2,CMP_HysteresisLevel3}CMP_HysteresisMode;
typedef enum {CMP_VrefSourceVin1 ,CMP_VrefSourceVin2} CMP_Vref;
typedef enum {CMP_OutputRising,CMP_OutputFalling,CMP_OutputAssert}CMP_Status;
typedef enum { CMP_InterruptOutputRising = 0x10,
			   CMP_InterruptOutputFalling = 0x07}CMP_Interrupt;

typedef struct
{
	bool enableModule;
	bool enableHighSpeed;
	bool invertOutput;
	bool useUnfilteredOutput;
	bool enableOutputPin;
	CMP_HysteresisMode hysteresisMode;
}CMP_Config;

typedef struct
{
	CMP_Vref vref;
	uint8_t dacValue; // Available range is 0-63. DACO = (Vin /64) * (dacValue + 1)
}CMP_DACConfig;

typedef struct
{
	uint8_t filterCount; // Available range is 1-7, 0 would cause the filter disabled.
	uint8_t filterPeriod; // The divider to bus clock. Available range is 0-255.
}CMP_FilterConfig;

void CMP_GetDefaultConfig(CMP_Config * config);

void CMP_Init(CMP_Instance n, CMP_Config * config);

void CMP_Enable(CMP_Instance n, bool enable);

void CMP_SetInputChannels (CMP_Instance n, CMP_Input positive, CMP_Input negative);

void CMP_SetFilterConfig (CMP_Instance n, CMP_FilterConfig *config);

void CMP_SetDACConfig (CMP_Instance n, CMP_DACConfig *config);

void CMP_SetOutputDestination(CMP_Output o);

uint8_t CMP_GetOutput(CMP_Instance n);

void CMP_EnableInterrupts (CMP_Instance n, uint32_t mask);

void CMP_DisableInterrupts (CMP_Instance n, uint32_t mask);


#endif /* STARTUP_CMP_H_ */
