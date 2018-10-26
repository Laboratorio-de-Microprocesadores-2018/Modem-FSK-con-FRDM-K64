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

#include "SysTick.h"
#include "DAC.h"


#include "PIT.h"
#include "stdlib.h"


#include "PORT.h"
#include "PWMGen.h"
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
//static uint8_t UartRxLen;
//static uint8_t ModemRxBuffer[10];
//static uint8_t ModemRxLen;

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////


#define BIT_FREC 1200
#define PWM_FREC 98400

void App_Init (void)
{

 PWMGen_Init();

 sysTickInit();

	UART_Config UARTconfig;
	UARTconfig.baud = UART_Baud_1200_Bps;
	UARTconfig.enableRx = true;
	UARTconfig.enableTx = true;
	UARTconfig.parityMode = UART_ParityOdd;//UART_ParityOdd;// UART_ParityDisabled;
	UARTconfig.RxFIFOEnable = true;
	UARTconfig.TxFIFOEnable = true;
	UARTconfig.loopBackEnable = false;
//	UART_Init(&UARTconfig);



	//sysTickInit();

	//pinMode(PIN_SW2,INPUT);
	//pinMode(PIN_LED_GREEN,OUTPUT);

}


static int currState,lastState;
static uint64_t lastDebounceTime;


void App_Run (void)
{

	static uint64_t time;
	if((millis()-time)>=1000)
	{
		time = millis();
		PWMGen_SendData('A');
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

	//static uint8_t rxByte;
	//if(UART_ReceiveByte(&rxByte))
	//	MODEM_SendData(rxByte);

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
