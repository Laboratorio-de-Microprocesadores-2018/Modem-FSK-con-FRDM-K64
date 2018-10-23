

#ifndef MODEM_H_
#define MODEM_H_
#include "stdint.h"

void MODEM_Init();

void MODEM_SendByte(uint8_t byte);

uint8_t MODEM_ReceiveByte();


#endif /* MODEM_H_ */
