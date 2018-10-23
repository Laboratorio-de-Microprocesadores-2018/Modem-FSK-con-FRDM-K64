#ifndef MODEM_H_
#define MODEM_H_

#include "stdint.h"
#include "stdbool.h"

void MODEM_Init();

void MODEM_SendData(uint8_t data);

bool MODEM_ReceiveData(uint8_t * buffer, uint8_t * length);


#endif /* MODEM_H_ */
