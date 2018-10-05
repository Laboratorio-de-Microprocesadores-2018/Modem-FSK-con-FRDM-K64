/////////////////////////////////////////////////////////////////////////////////
//                              CONTROL DE ACCESO                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file SysTick.c
 *
 */


#include "SysTick.h"
#include "Board.h"


// If defined, a GPIO pin is raised during sysTickHandler execution, in order
// to make measurements.
//#define DEBUG_SYSTICK

#ifdef DEBUG_SYSTICK
	#include "GPIO.h"
	#define SYSTICK_TEST_PIN PORTNUM2PIN(PD,1)
#endif




// Structure to store SysTick callbacks
typedef struct{
	SysTickFnc f;        // Function pointer
	uint32_t resetValue; // Reset value for the counter
	uint32_t count;      // Counter
}SysTickCallback;

// Static array to store callbacks
static SysTickCallback callbacks[SYSTICK_MAX_CALLBACKS];
static int callbacksSize;

static uint64_t _millisCount;

static void millisCount()
{
	_millisCount++;
}

uint64_t millis()
{
	return _millisCount;
}


uint32_t sysTickInit()
{
#ifdef DEBUG_SYSTICK
		pinMode(SYSTICK_TEST_PIN,OUTPUT);
#endif

	uint32_t ticks = SYSTICK_ISR_PERIOD_S*__CORE_CLOCK__;

	if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
	    return (0UL);

	SysTick->CTRL = 0x00;  // Disable the timer
	SysTick->LOAD = ticks -1;
	SysTick->VAL  = 0UL;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;

	sysTickAddCallback(&millisCount,1*MS2S);
	return (1UL);
}

bool sysTickAddCallback(SysTickFnc fnc,float period)
{
	// If the array is not full
	if(callbacksSize < SYSTICK_MAX_CALLBACKS)
	{
		callbacks[callbacksSize].f = fnc;
		callbacks[callbacksSize].resetValue = (uint32_t)(period/SYSTICK_ISR_PERIOD_S+0.5);
		callbacks[callbacksSize].count = callbacks[callbacksSize].resetValue;
		callbacksSize++;
		return true;
	}
	else return false;
}

bool sysTickAddDelayCall(SysTickFnc fnc,float time)
{
	// If the array is not full
	if(callbacksSize < SYSTICK_MAX_CALLBACKS)
	{
		callbacks[callbacksSize].f = fnc;
		callbacks[callbacksSize].resetValue = 0; // The reset counter is cero, call only once!
		callbacks[callbacksSize].count = time/SYSTICK_ISR_PERIOD_S;
		callbacksSize++;
		return true;
	}
	else return false;
}


__ISR__ SysTick_Handler(void)
{
#ifdef DEBUG_SYSTICK
		digitalWrite(SYSTICK_TEST_PIN,1);
#endif
	// Iterate all registered functions
	for(int i=0;i<callbacksSize;i++)
	{
		callbacks[i].count--;

		if(callbacks[i].count==0)
		{
			// If timeouts, execute
			(*callbacks[i].f)();
			// And either reset the counter, or unregister
			if(callbacks[i].resetValue!=0)
				callbacks[i].count = callbacks[i].resetValue;
			else
			{
				for(int j=i+1;j<callbacksSize;j++)
				{
					callbacks[j-1]=callbacks[j];
				}
				callbacksSize--;
				i--;
			}
		}
	}
#ifdef DEBUG_SYSTICK
		digitalWrite(SYSTICK_TEST_PIN,0);
#endif
}
