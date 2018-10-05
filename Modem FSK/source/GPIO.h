/////////////////////////////////////////////////////////////////////////////////
//                         														//
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file     GPIO.h
 * @brief    GPIO driver to handle pin configurations and IRQ setups
 * @author   Diego juarez
 * @author   Tobías Lifschitz
 */

#ifndef _GPIO_H_
#define _GPIO_H_

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <assert.h>
#include <Board.h>

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

typedef void (*pinIrqFun_t)(void);
/*
enum {
    GPIO_IRQ_MODE_DISABLE,
    GPIO_IRQ_MODE_RISING_EDGE,
    GPIO_IRQ_MODE_FALLING_EDGE,
    GPIO_IRQ_MODE_BOTH_EDGES,

    GPIO_IRQ_CANT_MODES
};*/

enum
{
	IRQC_DISABLE				= 0x00,
	IRQC_DMA_RISING				= 0x01,//Not developed
	IRQC_DMA_FALLING			= 0x02,//Not developed
	IRQC_DMA_EITHER				= 0x03,//Not developed
	IRQC_INTERRUPT_L0			= 0x08,//Not developed
	IRQC_INTERRUPT_RISING		= 0x09,
	IRQC_INTERRUPT_FALLING		= 0x0A,
	IRQC_INTERRUPT_EITHER		= 0x0B,
	IRQC_INTERRUPT_L1			= 0x0C,//Not developed
};

typedef enum
{
	PORT_mAnalog,
	PORT_mGPIO,
	PORT_mAlt2,
	PORT_mAlt3,
	PORT_mAlt4,
	PORT_mAlt5,
	PORT_mAlt6,
	PORT_mAlt7,

} PORTMux_t;

typedef enum
{
	PORT_eDisabled				= 0x00,
	PORT_eDMARising				= 0x01,
	PORT_eDMAFalling			= 0x02,
	PORT_eDMAEither				= 0x03,
	PORT_eInterruptDisasserted	= 0x08,
	PORT_eInterruptRising		= 0x09,
	PORT_eInterruptFalling		= 0x0A,
	PORT_eInterruptEither		= 0x0B,
	PORT_eInterruptAsserted		= 0x0C,
} PORTEvent_t;



/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Configures the specified pin to behave either as an input or an output
 * @param pin the pin whose mode you wish to set (according PORTNUM2PIN)
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void pinMode (uint8_t pin, uint8_t mode);

/**
 * @brief Configures how the pin reacts when an IRQ event ocurrs
 * @param pin the pin whose IRQ mode you wish to set (according PORTNUM2PIN)
 * @param irqMode disable, risingEdge, fallingEdge or bothEdges
 * @param irqFun function to call on pin event
 * @return Registration succeed(1=success)
 */
uint8_t pinConfigureIRQ (uint8_t pin, uint8_t irqMode, pinIrqFun_t irqFun);

/**
 * @brief Write a HIGH or a LOW value to a digital pin
 * @param id the pin to write (according PORTNUM2PIN)
 * @param val Desired value (HIGH or LOW)
 */
void digitalWrite (uint8_t pin, uint8_t value);

/**
 * @brief Toggle the value of a digital pin (HIGH<->LOW)
 * @param id the pin to toggle (according PORTNUM2PIN)
 */
void digitalToggle (uint8_t pin);

/**
 * @brief Reads the value from a specified digital pin, either HIGH or LOW.
 * @param id the pin to read (according PORTNUM2PIN)
 * @return HIGH or LOW. An inexistent pin number gives back 0xFF
 */
uint8_t digitalRead (uint8_t pin);

/////////////////////////////////////////////////////////////////////////////////
#endif // _GPIO_H_
