/////////////////////////////////////////////////////////////////////////////////
//                     				                                    	   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file     DAC.h
 * @brief    DAC driver
 * @date	 5 October 2018
 * @author   Tobías Lifschitz
 */

#ifndef _DAC_H_
#define _DAC_H_

#include "stdint.h"
#include "stdbool.h"

typedef enum {DAC_0,
			  DAC_1} DAC_Instance;

typedef enum {DAC_VREF_1,
			  DAC_VREF_2}DAC_Vref;

typedef enum {DAC_HARDWARE_TRIGGER,
			  DAC_SOFTWARE_TRIGGER,}DAC_TriggerMode;

typedef enum {DAC_MODE_NORMAL,
			  DAC_MODE_SWING,
			  DAC_MODE_ONE_TIME}DAC_WorkMode;

typedef enum {DAC_WATERMARK_1_WORD,
			  DAC_WATERMARK_2_WORD,
			  DAC_WATERMARK_3_WORD,
			  DAC_WATERMARK_4_WORD}DAC_Watermark;

typedef enum{DAC_INTERRUPT_WATERMARK = 0x4U,
			DAC_INTERRUPT_POINTER_TOP = 0x2U,
			DAC_INTERRUPT_POINTER_BOTTOM = 0x1U}DAC;


typedef enum{DAC_FLAG_WATERMARK = 0x4U,
			 DAC_FLAG_POINTER_TOP = 0x2U,
			 DAC_FLAG_POINTER_BOTTOM = 0x1U}DAC_Flag;

typedef struct
{
	DAC_TriggerMode triggerMode;
	DAC_WorkMode workMode;
	uint8_t upperLimit;  // 0-16
	DAC_Watermark watermark;
}DAC_BufferConfig;


/**
 *
 */
void DAC_Init(DAC_Instance n, DAC_Vref vref);

/**
 *
 */
void DAC_Deinit(DAC_Instance n);

/**
 *
 */
void DAC_Enable(DAC_Instance n);

/**
 *
 */
void DAC_Disable(DAC_Instance n);

/**
 *
 */
void DAC_WriteValue(DAC_Instance n,uint16_t value);


/**
 *
 */
void DAC_GetDefaultBufferConfig (DAC_BufferConfig * config);

/**
 *
 */
void DAC_SetBufferConfig(DAC_Instance n, DAC_BufferConfig * config);

/**
 *
 */
void DAC_SetBufferValue (DAC_Instance n, uint8_t index, uint16_t value);

/**
 *
 */
void DAC_EnableBufferDMA (DAC_Instance n, bool enable);

/**
 *
 */
void DAC_TriggerBuffer (DAC_Instance n);

/**
 *
 */
uint8_t DAC_GetBufferPointer (DAC_Instance n);

/**
 *
 */
void 	DAC_SetBufferPointer (DAC_Instance n, uint8_t index);


/**
 * void DAC0_IRQHandler(void) y void DAC1_IRQHandler(void);
 */
void 	DAC_EnableInterrupts (DAC_Instance n, uint32_t mask);

/**
 *
 */
void 	DAC_DisableInterrupts (DAC_Instance n, uint32_t mask);

/**
 *
 */
bool DAC_GetFlag(DAC_Instance n, DAC_Flag flag);

void DAC_ClearFlag(DAC_Instance n, DAC_Flag flag);

#endif
