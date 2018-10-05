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

typedef enum {DAC_VREF_1,DAC_VREF_2}DAC_VREF;
typedef enum {DAC_SOFTWARE_TRIGGER,DAC_HARDWARE_TRIGGER}DAC_TRIGGER_MODE;
typedef enum {DAC_MODE_NORMAL,DAC_MODE_ONETIMESCAN}DAC_WORK_MODE;
/**
 *
 */
void DAC_Init(DAC_VREF vref);

/**
 *
 */
void DAC_Deinit();

/**
 *
 */
void DAC_Enable();

/**
 *
 */
void DAC_Disable();

void DAC_WriteValue(uint16_t data);

#endif
