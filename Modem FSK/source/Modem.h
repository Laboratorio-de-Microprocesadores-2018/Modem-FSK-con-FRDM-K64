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

void MODEM_Init(MODEM_Config * config);

// REEMPLAZAR EL MAIN POR UN:
void MODEM_Run(void);

void MODEM_SendData(uint8_t data);

bool MODEM_ReceiveData(uint8_t * buffer, uint8_t * length);


#endif /* MODEM_H_ */

