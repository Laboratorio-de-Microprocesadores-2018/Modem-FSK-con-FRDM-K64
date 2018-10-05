/////////////////////////////////////////////////////////////////////////////////
//                              CONTROL DE ACCESO                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file Systick.h
 * @brief Systick driver to register periodic callbacks calls or single calls.
 */

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
#include "hardware.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////
#define SYSTICK_MAX_CALLBACKS 10
#define MS2S (1/1000.0)
#define US2S (1/1000000.0)
#define SYSTICK_ISR_PERIOD_S (250*US2S)

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////
typedef void(*SysTickFnc)(void);

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Init systick driver
 * @return Init succeed
 */
uint32_t sysTickInit();

/**
 * @brief Register a periodic call to a function.
 * @param period Period of the function call.
 * @return Register succeed. If false, max. number of callbacks reached.
 */
bool sysTickAddCallback(SysTickFnc fnc,float period);

/**
 * @brief Register a single call to a function, after a certain delay.
 * @param time Time to wait before the function call.
 * @return Register succeed. If false, max. number of callbacks reached.
 */
bool sysTickAddDelayCall(SysTickFnc fnc,float time);

/**
 * @brief Return the number of milliseconds since the program execution started.
 * @return Time in milliseconds.
 */
uint64_t millis();


//*******************************************************************************

/////////////////////////////////////////////////////////////////////////////////
#endif // _SYSTICK_H_
