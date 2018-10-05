
#include "DAC.h"
#include "MK64F12.h"
#include "MK64F12_features.h"


#define DAC_DATL_DATA0_WIDTH (8)
/*


@brief Define the size of hardware buffer
 FSL_FEATURE_DAC_BUFFER_SIZE (16)

@brief Define whether the buffer supports watermark event detection or not.
 FSL_FEATURE_DAC_HAS_WATERMARK_DETECTION (1)

@brief Define whether the buffer supports watermark selection detection or not.
 FSL_FEATURE_DAC_HAS_WATERMARK_SELECTION (1)

@brief Define whether the buffer supports watermark event 1 word before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_1_WORD (1)

@brief Define whether the buffer supports watermark event 2 words before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_2_WORDS (1)

@brief Define whether the buffer supports watermark event 3 words before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_3_WORDS (1)

@brief Define whether the buffer supports watermark event 4 words before buffer upper limit.
 FSL_FEATURE_DAC_HAS_WATERMARK_4_WORDS (1)

@brief Define whether FIFO buffer mode is available or not.
 FSL_FEATURE_DAC_HAS_BUFFER_FIFO_MODE (0)

@brief Define whether swing buffer mode is available or not..
 FSL_FEATURE_DAC_HAS_BUFFER_SWING_MODE (1)*/

void DAC_Init(DAC_VREF vref)
{
	// Enable clock gating
	SIM->SCGC2 |= SIM_SCGC2_DAC0_MASK;

	// Enable DAC and select the voltage reference
	if(vref == DAC_VREF_1)
		DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACTRGSEL_MASK;
	else if (vref == DAC_VREF_2)
		DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK | DAC_C0_DACTRGSEL_MASK;
}

void DAC_Deinit()
{
	// Diasble DAC
	DAC0->C0 &= ~DAC_C0_DACEN_MASK;

	// Disable clock gating
	SIM->SCGC2 &= ~SIM_SCGC2_DAC0_MASK;
}

void DAC_Enable()
{
	DAC0->C0 |= DAC_C0_DACEN_MASK;
}

void DAC_Disable()
{
	DAC0->C0 &= ~DAC_C0_DACEN_MASK;
}

void DAC_WriteValue(uint16_t data)
{
	if(~(DAC0->C1 & DAC_C1_DACBFEN_MASK))
	{
		DAC0->DAT[0].DATL = DAC_DATL_DATA0(data);
		DAC0->DAT[0].DATH = DAC_DATH_DATA1(data >> DAC_DATL_DATA0_WIDTH);
	}
}
