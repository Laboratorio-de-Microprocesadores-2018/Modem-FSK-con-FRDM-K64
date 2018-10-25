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
//static uint8_t UartRxLen;
//static uint8_t ModemRxBuffer[10];
//static uint8_t ModemRxLen;

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////



void App_Init (void)
{

	UART_Config UARTconfig;
	UARTconfig.baud = UART_Baud_1200_Bps;
	UARTconfig.enableRx = true;
	UARTconfig.enableTx = true;
	UARTconfig.parityMode = UART_ParityOdd;//UART_ParityOdd;// UART_ParityDisabled;
	UARTconfig.RxFIFOEnable = true;
	UARTconfig.TxFIFOEnable = true;
	UARTconfig.loopBackEnable = false;
	UART_Init(&UARTconfig);

	MODEM_Config MODEMconfig;

	MODEM_Init(&MODEMconfig);

	//sysTickInit();
	//pinMode(PIN_SW2,INPUT);
	//pinMode(PIN_LED_GREEN,OUTPUT);

}


//static int currState,lastState;
//static uint64_t lastDebounceTime;


void App_Run (void)
{
/*
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
*/

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

	static uint8_t rxByte;
	if(UART_ReceiveByte(&rxByte))
		MODEM_SendData(rxByte);


}

