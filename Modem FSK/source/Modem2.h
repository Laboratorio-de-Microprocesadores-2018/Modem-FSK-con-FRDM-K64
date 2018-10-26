#ifndef MODEM2_H_
#define MODEM2_H_

#include <stdint.h>
#include <stdbool.h>


void MODEM2_Init();

void MODEM2_SendData(uint8_t data);

bool MODEM2_ReceiveByte(uint8_t * byte);



#endif //MODEM2
