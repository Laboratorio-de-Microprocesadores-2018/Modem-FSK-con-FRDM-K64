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
#include "SysTick.h"
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
	sysTickInit();
	pinMode(PIN_SW2,INPUT);
	pinMode(PIN_LED_GREEN,OUTPUT);

}

static int currState,lastState;
static uint64_t lastDebounceTime;

void App_Run (void)
{

	if((millis()-lastDebounceTime)>=500)
	{
		lastDebounceTime = millis();
		MODEM_SendData(0x01);
		uint64_t t = millis();
		while((millis()-t)<5);

		MODEM_SendData(0x02);
		t = millis();
		while((millis()-t)<9);

		MODEM_SendData(0x03);
		t = millis();
		while((millis()-t)<9);

		MODEM_SendData(0x04);
		digitalToggle(PIN_LED_GREEN);
	}

	/*
	static int debounced;
	currState = digitalRead(PIN_SW2);
	if(currState!=lastState)
	{
		debounced = 0;
		lastState=currState;
		lastDebounceTime = millis();
	}

	if((millis()-lastDebounceTime)>=40 && debounced == 0)
	{
		debounced = 1;
		if(lastState==0)
		{
			MODEM_SendData(0x10101010b);
			digitalWrite(PIN_LED_GREEN,0);
		}
		else
		{
			digitalWrite(PIN_LED_GREEN,1);
		}
	}*/




/*
// Mas o menos asi seria el main loop
	UartRxLen = sizeof(UartRxBuffer);
	ModemRxLen = sizeof(ModemRxBuffer);

	if(UART_RecieveData(UartRxBuffer,&UartRxLen))
	{
		for(int i=0; i<UartRxLen; i++)
		{
			MODEM_SendData(UartRxBuffer[i]);
			UART_SendData(UartRxBuffer,UartRxLen);
		}
	}*/
	/*if(MODEM_ReceiveData(ModemRxBuffer,&ModemRxLen))
		UART_SendData(ModemRxBuffer,ModemRxLen);*/
}

