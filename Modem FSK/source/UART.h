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


typedef enum
{
	UART_ParityEven,
	UART_ParityOdd,
	UART_ParityDisabled
}UART_ParityMode;

typedef enum
{
	UART_Baud_1200_Bps = 1200,
	//UART_Baud_2400_BPS,
	//UART_Baud_4800_BPS,
	UART_Baud_9600_Bps = 9600,
	//UART_Baud_19200_BPS,
	//UART_Baud_38400_BPS,
	//UART_Baud_57600_BPS,
	//UART_Baud_115200_BPS
}UART_Baudrate;



typedef struct
{
	UART_Baudrate baud;
	UART_ParityMode parityMode;
	bool enableTx;
	bool enableRx;
	bool TxFIFOEnable;
	bool RxFIFOEnable;
	bool loopBackEnable;
}UART_Config;


/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Initializes the module of UART in the nxp cortex.
 * @param Not developed.
 * @param Not developed.
 * @return Not developed.
 */
void UART_Init (UART_Config * config);


/**
 * @brief
 * @param
 * @return
 */
bool UART_SendByte( uint8_t byte);

/**
 * @brief
 * @param
 * @return
 */
bool UART_ReceiveByte(uint8_t * byte);


/**
 * @brief Sets the requested Baud Rate in the corresponding register of the UART module desired.
 * @param uart Is the pointer to the base of the map memory of the UART module where the Baud Rate is changed.
 * @param baudrate is the real Baud Rate you want to set.
 */
void UART_SetBaudRate ( uint32_t baudrate);

/**
 * @brief Service funcition to send the rquired data through the UART module.
 * @param tx_data Pointer to the begining of the chunk of data to transmit.
 * @param len Length of the chunk of data to transmit.
 * @return true if everything went fine, false if there was an error.
 */
bool UART_SendData( uint8_t * tx_data, uint8_t len);

/**
 * @brief Service funcition to get the recieved data through the UART module.
 * @param rx_data Pointer to the begining of the memory place where to save the data.
 * @param len Maximum amount of data words to be saved.
 * @return true if everything went fine, false if there was an error.
 */
bool UART_RecieveData( uint8_t * rx_data, uint8_t len);


#endif /* UART_H_ */
