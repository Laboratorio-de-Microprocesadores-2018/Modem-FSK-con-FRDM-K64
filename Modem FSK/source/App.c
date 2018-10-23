/////////////////////////////////////////////////////////////////////////////////
//                     				MODEM FSK                              	   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file App.c
 * @brief Trabajo Practico N°3.
 */

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "Modem.h"
#include "UART.h"
#include "GPIO.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////
static uint8_t UartRxBuffer[10];
static uint8_t UartRxLen;
static uint8_t ModemRxBuffer[10];
static uint8_t ModemRxLen;

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////


void DAC0_IRQHandler()
{
	//digitalToggle(PORTNUM2PIN(PC,10));

	//static uint8_t index = 0;

	//DAC_WriteValue(DAC_0,signal[index++]);

	/*if(DAC_GetFlag(DAC_0,DAC_INTERRUPT_POINTER_TOP)==true)
	{
		for(int n=DAC_BUFFER_SIZE-DAC_WATERMARK-1;n<DAC_BUFFER_SIZE; n++)
			DAC_SetBufferValue (DAC_0, n, signal[index+n]);

		index = (index + DAC_BUFFER_SIZE)%N_SAMPLE;

		DAC_ClearFlag(DAC_0,DAC_INTERRUPT_POINTER_TOP);
	}
	else if(DAC_GetFlag(DAC_0,DAC_INTERRUPT_WATERMARK)==true)
	{
		for(int n=0;n<DAC_BUFFER_SIZE-DAC_WATERMARK-1; n++)
				DAC_SetBufferValue (DAC_0, n, signal[index+n]);

		DAC_ClearFlag(DAC_0,DAC_INTERRUPT_WATERMARK);
	}
	digitalToggle(PORTNUM2PIN(PC,10));*/

	/*for(int n=0;n<DAC_BUFFER_SIZE; n++)
		DAC_SetBufferValue (DAC_0, n, signal[index+n]);

	index = (index + DAC_BUFFER_SIZE)%N_SAMPLE;

	// Reset pointer
	//DAC_SetBufferPointer (DAC_0,0);

	DAC_ClearFlag(DAC_0,DAC_INTERRUPT_POINTER_TOP);*/


}

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////


void App_Init (void)
{
	UARTInit();
	MODEM_Init();
}


void App_Run (void)
{
	UartRxLen = sizeof(UartRxBuffer);
	ModemRxLen = sizeof(ModemRxBuffer);

	if(UART_RecieveData(UartRxBuffer,&UartRxLen))
	{
		for(int i=0; i<UartRxLen; i++)
			MODEM_SendData(UartRxBuffer[i]);
	}
	if(MODEM_ReceiveData(ModemRxBuffer,&ModemRxLen))
		UART_SendData(ModemRxBuffer,ModemRxLen);
}

