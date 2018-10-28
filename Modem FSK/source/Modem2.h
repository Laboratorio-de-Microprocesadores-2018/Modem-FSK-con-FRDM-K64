/////////////////////////////////////////////////////////////////////////////////
//                         													   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

#ifndef MODEM2_H_
#define MODEM2_H_
/**
 * @file     Modem2.h
 * @brief    FSK Digital Modem
 */
/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <stdbool.h>
/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Modem version 2 initialization
 */
void MODEM2_Init();
/**
 * @brief Modem version 2  function to send a byte
 * @param data byte to be sent
 */
void MODEM2_SendData(uint8_t data);
/**
 * @brief Modem version 2  function to receive a byte
 * @param byte to be sent
 * @return true when there is new data, false if no data has arrived
 */
bool MODEM2_ReceiveByte(uint8_t * byte);



#endif //MODEM2
