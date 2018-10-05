/*
 * UART.c
 *
 *  Created on: 14 sep. 2018
 *      Author: sebas
 */
/////////////////////////////////////////////////////////////////////////////////
//                        Intertial Motion Unit (IMU)						   //
//																			   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include "hardware.h"
#include "UART.h"
#include "GPIO.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////
#define UART_HAL_DEFAULT_BAUDRATE	9600

#define UART_CALL_FREQUENCY			100

#define BUFFER_SIZE					100

#define UART0_TX_PIN 	17   //PTB17
#define UART0_RX_PIN 	16   //PTB16

#define UART0FIFOEXP	(((UART0->PFIFO) & UART_PFIFO_TXFIFOSIZE_MASK) >> (UART_PFIFO_TXFIFOSIZE_SHIFT))
#define UART0FIFOSIZE	(1 << (UART0FIFOEXP + 1))

#define TIESTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_C2_TIE_MASK)) == UART_C2_TIE_MASK)? true: false)
#define TDRESTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_S1_TDRE_MASK)) == UART_S1_TDRE_MASK)? true: false)
// #define TDRESTAT(x)		(((uint8_t)(((uint8_t)(x)) & UART_S1_TDRE_MASK)) >> UART_S1_TDRE_SHIFT)

#define RIESTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_C2_RIE_MASK)) == UART_C2_RIE_MASK)? true: false)
#define RDRFSTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_S1_RDRF_MASK)) == UART_S1_RDRF_MASK)? true: false)
// #define RDRFSTAT(x)		(((uint8_t)(((uint8_t)(x)) & UART_S1_RDRF_MASK)) >> UART_S1_RDRF_SHIFT)

#define TXOFStat(x)		(((uint8_t)(((uint8_t)(x)) & UART_SFIFO_TXOF_MASK)) >> UART_SFIFO_TXOF_SHIFT)
#define RXOFStat(x)		(((uint8_t)(((uint8_t)(x)) & UART_SFIFO_RXOF_MASK)) >> UART_SFIFO_RXOF_SHIFT)


/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////
typedef enum{TXOF_ERR = 0, RXOF_ERR, BUFFFULL_ERR, BUFFEMPTY_ERR, UART0IRQ_ERR, NO_ERR};


/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////
static uint8_t err;
static uint8_t transferWord;
static bool errFlag;
NEW_CIRCULAR_BUFFER(transmitBuffer,BUFFER_SIZE,sizeof(transferWord));
NEW_CIRCULAR_BUFFER(recieveBuffer,BUFFER_SIZE,sizeof(transferWord));

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////
static void transmitData(void);
static void recieveData(void);
static void loadBuffer2Register(void);
static void loadRegister2Buffer(void);


/**
 * @brief Initializes the module of UART in the nxp cortex.
 * @param Not developed.
 * @param Not developed.
 * @return Not developed.
 */
void UARTInit (void)
{

#ifdef MEASURE_UART
	MEASURE_UART_PORT->PCR[MEASURE_UART_PIN] = PORT_PCR_MUX(1);
	MEASURE_UART_GPIO->PDDR |= (1<<MEASURE_UART_PIN);
	MEASURE_UART_GPIO->PDOR &= ~(1<<MEASURE_UART_PIN);
#endif

// Note: 5.6 Clock Gating page 192
// Any bus access to a peripheral that has its clock disabled generates an error termination.
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
/*		SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
	SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
	SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;
*/

	NVIC_EnableIRQ(UART0_RX_TX_IRQn);
/*		NVIC_EnableIRQ(UART1_RX_TX_IRQn);
	NVIC_EnableIRQ(UART2_RX_TX_IRQn);
	NVIC_EnableIRQ(UART3_RX_TX_IRQn);
	NVIC_EnableIRQ(UART4_RX_TX_IRQn);
	NVIC_EnableIRQ(UART5_RX_TX_IRQn);
*/

	//UART0 Set UART Speed

	UARTSetBaudRate(UART0, UART_HAL_DEFAULT_BAUDRATE);

	//Configure UART0 TX and RX PINS

	PORTB->PCR[UART0_TX_PIN] = 0x0; //Clear all bits
	PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
	PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts
//----------------------------------------------------------------------------------
	PORTB->PCR[UART0_RX_PIN] = 0x0; //Clear all bits
	PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
	PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts



	UART0->PFIFO = UART_PFIFO_RXFE_MASK | UART_PFIFO_TXFE_MASK;

	//Enable UART0 Xmiter and Rcvr
	UART0->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

	err = NO_ERR;

}

/**
 * @brief Sets the requested Baud Rate in the corresponding register of the UART module desired.
 * @param uart Is the pointer to the base of the map memory of the UART module where the Baud Rate is changed.
 * @param baudrate is the real Baud Rate you want to set.
 */
void UARTSetBaudRate (UART_Type *uart, uint32_t baudrate)
{
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart == UART0) || (uart == UART1)) ?(__CORE_CLOCK__):(__CORE_CLOCK__ >> 1);

	baudrate = ((baudrate == 0)?(UART_HAL_DEFAULT_BAUDRATE):
			((baudrate > 0x1FFF)?(UART_HAL_DEFAULT_BAUDRATE):(baudrate)));

	sbr = clock / (baudrate << 4);               // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	uart->BDH = UART_BDH_SBR(sbr >> 8);
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}


/**
 * @brief Service funcition to send the rquired data through the UART module.
 * @param tx_data Pointer to the begining of the chunk of data to transmit.
 * @param len Length of the chunk of data to transmit.
 * @return true if everything went fine, false if there was an error.
 */
bool UARTSendData( uint8_t * tx_data, uint8_t len)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif
	if( len > 0)
	{
		for(int i = 0; (i < len) && (err == NO_ERR); i++)
		{
			if(!push(&transmitBuffer, &tx_data[i]))
			{
				err = BUFFFULL_ERR;
				return false;
			}
		}
		UART0->C2 |= UART_C2_TIE_MASK;
	}

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif
	return true;
}

/**
 * @brief Service funcition to get the recieved data through the UART module.
 * @param rx_data Pointer to the begining of the memory place where to save the data.
 * @param len Maximum amount of data words to be saved.
 * @return true if everything went fine, false if there was an error.
 */
bool UARTRecieveData( uint8_t * rx_data, uint8_t len)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif

	uint8_t lenRet = 0 ;
	while( (lenRet < len) && pop(&recieveBuffer, &rx_data[lenRet]))
		lenRet ++;

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif

	return lenRet;
}

/**
 * @brief Interrupt handler function. This will check which flag was the one that called the interrupt.
 */
__ISR__ UART0_RX_TX_IRQHandler(void)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif
	uint8_t debugSRegister, debugCRegister;
	debugSRegister = UART0->S1;
	debugCRegister = UART0->C2;

	if(TDRESTAT(UART0->S1) && TIESTAT(UART0->C2))
		transmitData();

	else if(RDRFSTAT(UART0->S1) && RIESTAT(UART0->C2))
		recieveData();

	else{}
		//err = UART0IRQ_ERR;

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif

}

/**
 * @brief It will send the data that is in the transmission buffer to the UART module. It uses local variables.
 */
void transmitData(void)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif
	uint8_t debugRegister;
	debugRegister = UART0->S1;

	int tBuffCount = numel(&transmitBuffer);
	int FIFOLeft = UART0FIFOSIZE - (UART0->TCFIFO);
	if( tBuffCount < FIFOLeft )
		UART0->C2 &= (~UART_C2_TIE_MASK);

	if(isEmpty(&transmitBuffer) == false)
	{
		for(int i = 0; (i < (tBuffCount - 1)) && (i < (FIFOLeft-1)) && (err == NO_ERR); i++)
		{
			loadBuffer2Register();
		}
		if(err == NO_ERR)
		{
			bool a = TDRESTAT(UART0->S1);
			bool b = !TXOFStat(UART0->SFIFO);
			//a = ( TDRESTAT(UART0->S1) || !TXOFStat(UART0->SFIFO) );
			if( a || b)
			{
				loadBuffer2Register();
			}else
			{
				err = TXOF_ERR;
				return;
			}
		}

	}

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif
}

/**
 * @brief Analogously to the transmitData functio, this will get the recieved in the UART module and stores it in the local reception buffer.
 */
void recieveData(void)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif
	if(isFull(&recieveBuffer) == false)
	{
		int FIFOCount = UART0->RCFIFO;
		for(int i = 0; (i < (FIFOCount - 1)) && (err == NO_ERR); i++)
		{
			loadRegister2Buffer();
		}
		if(err == NO_ERR)
		{
			if((RDRFSTAT(UART0->S1)) && (RXOFStat(UART0->SFIFO)))
			{
				loadRegister2Buffer();
			}else
			{
				err = RXOF_ERR;
				return;
			}
		}
	}else
	{
		err = BUFFFULL_ERR;
		return;
	}

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif
}

/**
 * @brief Actually pulls the next word of data to transmit and writes it to the data register in the UART module.
 */
void loadBuffer2Register(void)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif

	if(pop(&transmitBuffer, &transferWord))
		UART0->D = transferWord;
	else
	{
		err = BUFFEMPTY_ERR;
		return;
	}
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif
}

/**
 * @brief Actually reads the data register in the UART module and pushes the recieved data word to the reception buffer.
 */
void loadRegister2Buffer(void)
{
	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif

	transferWord = UART0->D;
	if(!push(&recieveBuffer, &transferWord))
	{
		err = BUFFFULL_ERR;
		return;
	}

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 0;
	#endif
}
