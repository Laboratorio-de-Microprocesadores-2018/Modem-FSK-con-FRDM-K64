/////////////////////////////////////////////////////////////////////////////////
//                         													   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @file     Modem.h
 * @brief    FSK Modem
 */

#ifndef MODEM_H_
#define MODEM_H_

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>

#define MODEM_VERSION 1

#ifndef MODEM_VERSION
#error "Modem version undefined!"
#endif

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Modem initialization
 */
void MODEM_Init();
/**
 * @brief Modem function to send a byte
 * @param data byte to be sent
 */
void MODEM_SendByte(uint8_t data);


/**
 * @brief Modem  function to receive a byte
 * @param pointer to a variable to store received data
 * @return true when there is new data, false if no data has arrived
 */
bool MODEM_ReceiveByte(uint8_t * byte);

/**
 * @brief Process input data. Must call this function periodically
 */
void MODEM_Demodulate(void);


#endif //MODEM
