/////////////////////////////////////////////////////////////////////////////////
//                              CONTROL DE ACCESO                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file Board.h
 * @brief Kinetis K64 board definitions.
 * @author Nicolas Magliola
 */

#ifndef _BOARD_H_
#define _BOARD_H_

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

enum { PA, PB, PC, PD, PE };

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

/** GPIO pin definition */
#define PORTNUM2PIN(po, pi)  (((po)<<5) + (pi))

/** GPIO pin modes */
#define INPUT           0
#define OUTPUT          1
#define INPUT_PULLUP    2
#define INPUT_PULLDOWN  3

#define LOW     0
#define HIGH    1

/** On Board User LEDs */
#define PIN_LED_RED     PORTNUM2PIN(PB,22)
#define PIN_LED_GREEN   PORTNUM2PIN(PE,26)
#define PIN_LED_BLUE    PORTNUM2PIN(PB,21) 

#define LED_ACTIVE      LOW

// On Board User Switches
#define PIN_SW2         PORTNUM2PIN(PC,6)
#define PIN_SW3         PORTNUM2PIN(PA,4)


/////////////////////////////////////////////////////////////////////////////////
#endif // _BOARD_H_
