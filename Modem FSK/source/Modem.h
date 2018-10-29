#ifndef MODEM_H_
#define MODEM_H_

#include "stdint.h"
#include "stdbool.h"


#define MODEM_VERSION 1

#if MODEM_VERSION == 1

#elif MODEM_VERSON == 2

#else
	#error ("Modem version not defined.")
#endif



typedef enum
{
	MODEM_1200_Bps,
	//MODEM_9600_Bps;
}MODEM_BaudRate;

typedef struct
{
	MODEM_BaudRate baudRate;
}MODEM_Config;


void MODEM_GetDefaultConfig();

void MODEM_Init();

// REEMPLAZAR EL MAIN POR UN:
void MODEM_Run(void);
void MODEM_demodulate(void);

void MODEM_SendByte(uint8_t byte);

bool MODEM_ReceiveByte(uint8_t * byte);


#endif /* MODEM_H_ */

