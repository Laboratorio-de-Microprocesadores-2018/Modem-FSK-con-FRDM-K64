/////////////////////////////////////////////////////////////////////////////////
//                    	   TP2 -  Comunicaciones Serie                         //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

#ifndef UART_H_
#define UART_H_


/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include "MK64F12.h"
#include "CircularBuffer.h"

//#define MEASURE_UART
#ifdef MEASURE_UART
	#define MEASURE_UART_PORT PORTC
	#define MEASURE_UART_GPIO GPIOC
	#define MEASURE_UART_PIN	9
#endif

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Initializes the module of UART in the nxp cortex.
 * @param Not developed.
 * @param Not developed.
 * @return Not developed.
 */
void UARTInit (void);

/**
 * @brief Sets the requested Baud Rate in the corresponding register of the UART module desired.
 * @param uart Is the pointer to the base of the map memory of the UART module where the Baud Rate is changed.
 * @param baudrate is the real Baud Rate you want to set.
 */
void UARTSetBaudRate (UART_Type *uart, uint32_t baudrate);

/**
 * @brief Service funcition to send the rquired data through the UART module.
 * @param tx_data Pointer to the begining of the chunk of data to transmit.
 * @param len Length of the chunk of data to transmit.
 * @return true if everything went fine, false if there was an error.
 */
bool UARTSendData( uint8_t * tx_data, uint8_t len);

/**
 * @brief Service funcition to get the recieved data through the UART module.
 * @param rx_data Pointer to the begining of the memory place where to save the data.
 * @param len Maximum amount of data words to be saved.
 * @return true if everything went fine, false if there was an error.
 */
bool UARTRecieveData( uint8_t * rx_data, uint8_t len);


#endif /* UART_H_ */
