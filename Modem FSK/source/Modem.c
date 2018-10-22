#include "Modem.h"
//#include "DAC.h"
//#include "PDB.h"
#include "math.h"
#include "SysTick.h"
#include "GPIO.h"
#include "DMAMUX.h"
#include "DMA.h"
#include "PIT.h"

#define SINE_FREQ (1100)
#define N_SAMPLE (64)


static uint16_t signal[N_SAMPLE];

void MODEM_Init()
{

}

void MODEM_SendByte(uint8_t byte)
{

}

uint8_t MODEM_ReceiveByte()
{

}
