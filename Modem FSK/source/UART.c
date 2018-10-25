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
static bool errFlag;
static uint8_t transferWord;
static uint8_t UART_RX_FIFO_SIZE;
static uint8_t UART_TX_FIFO_SIZE;
NEW_CIRCULAR_BUFFER(transmitBuffer,BUFFER_SIZE,sizeof(uint8_t));
NEW_CIRCULAR_BUFFER(recieveBuffer,BUFFER_SIZE,sizeof(uint8_t));

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
void UART_Init (UART_Config * config)
{

#ifdef MEASURE_UART
	MEASURE_UART_PORT->PCR[MEASURE_UART_PIN] = PORT_PCR_MUX(1);
	MEASURE_UART_GPIO->PDDR |= (1<<MEASURE_UART_PIN);
	MEASURE_UART_GPIO->PDOR &= ~(1<<MEASURE_UART_PIN);
#endif

	// Enable clock
		SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

		NVIC_EnableIRQ(UART0_RX_TX_IRQn);

		// Configure baudrate
		UART_SetBaudRate(config->baud);

		//Configure UART0 TX and RX PINS
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

		PORTB->PCR[UART0_TX_PIN] = 0x0; //Clear all bits
		PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
		PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts

		PORTB->PCR[UART0_RX_PIN] = 0x0; //Clear all bits
		PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
		PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts

		if(config->loopBackEnable)
		{
			UART0->C1 |= UART_C1_LOOPS_MASK;
			// setear RSRC si se quiere elegir el modo
		}

		// Configure parity
		switch(config->parityMode)
		{
		case UART_ParityDisabled:
			UART0->C1 &= ~UART_C1_PE_MASK;
			break;
		case UART_ParityOdd: case UART_ParityEven:
			UART0->C1 |= UART_C1_PE_MASK;
			UART0->C1 |= UART_C1_M_MASK;
			UART0->C4 &= ~UART_C4_M10_MASK;

			UART0->C1 |= UART_C1_PT(config->parityMode);
			break;
		}

		// Configure FIFOs
		UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

		if(config->TxFIFOEnable)
			UART0->PFIFO |= UART_PFIFO_TXFE_MASK;
		else
			UART0->PFIFO &= ~UART_PFIFO_TXFE_MASK;

		if(config->RxFIFOEnable)
			UART0->PFIFO |= UART_PFIFO_RXFE_MASK;
		else
			UART0->PFIFO &= ~UART_PFIFO_RXFE_MASK;

		uint8_t TXFIFOSIZE = (((UART0->PFIFO) & UART_PFIFO_TXFIFOSIZE_MASK) >> (UART_PFIFO_TXFIFOSIZE_SHIFT));
		UART_TX_FIFO_SIZE =  1<<(TXFIFOSIZE+1);

		uint8_t RXFIFOSIZE = (((UART0->PFIFO) & UART_PFIFO_RXFIFOSIZE_MASK) >> (UART_PFIFO_RXFIFOSIZE_SHIFT));
		UART_RX_FIFO_SIZE =  1<<(RXFIFOSIZE+1);

		if(config->enableRx)
			UART0->C2 |=UART_C2_RE_MASK;
		if(config->enableTx)
			UART0->C2 |= UART_C2_TE_MASK;

		// Enable receive interrupts
		UART0->C2 |= UART_C2_RIE_MASK;

		err = NO_ERR;

}

/**
 * @brief Sets the requested Baud Rate in the corresponding register of the UART module desired.
 * @param uart Is the pointer to the base of the map memory of the UART module where the Baud Rate is changed.
 * @param baudrate is the real Baud Rate you want to set.
 */
void UART_SetBaudRate (uint32_t baudrate)
{
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = __CORE_CLOCK__;

	baudrate = ((baudrate == 0)?(UART_HAL_DEFAULT_BAUDRATE):
			((baudrate > 0x1FFF)?(UART_HAL_DEFAULT_BAUDRATE):(baudrate)));

	sbr = clock / (baudrate << 4);               // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	UART0->BDH = UART_BDH_SBR(sbr >> 8);
	UART0->BDL = UART_BDL_SBR(sbr);
	UART0->C4 = (UART0->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}

bool UART_SendByte( uint8_t byte)
{
	if(push(&transmitBuffer, &byte))
	{
		// Enable interrupts
		UART0->C2 |= UART_C2_TIE_MASK;
		return true;
	}
	else return false;
}

bool UART_ReceiveByte(uint8_t * byte)
{
	return pop(&recieveBuffer, byte);
}


/**
 * @brief Service funcition to send the rquired data through the UART module.
 * @param tx_data Pointer to the begining of the chunk of data to transmit.
 * @param len Length of the chunk of data to transmit.
 * @return true if everything went fine, false if there was an error.
 */
bool UART_SendData( uint8_t * tx_data, uint8_t len)
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
bool UART_RecieveData( uint8_t * rx_data, uint8_t len)
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
	ASSERT(err==NO_ERR);

	#ifdef MEASURE_UART
		BITBAND_REG(MEASURE_UART_GPIO->PDOR, MEASURE_UART_PIN) = 1;
	#endif

	uint8_t debugSRegister, debugCRegister;
	debugSRegister = UART0->S1;
	debugCRegister = UART0->C2;

	if(TDRESTAT(debugSRegister) && TIESTAT(debugCRegister))
	{
		int tBuffCount = numel(&transmitBuffer);
		int FIFOLeft = UART0FIFOSIZE - (UART0->TCFIFO);
		if( tBuffCount < FIFOLeft )
			UART0->C2 &= (~UART_C2_TIE_MASK);

		if(isEmpty(&transmitBuffer) == false)
		{
			for(int i = 0; (i < (tBuffCount - 1)) && (i < (FIFOLeft-1)) && (err == NO_ERR); i++)
				loadBuffer2Register();

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
	}
	else if(RDRFSTAT(debugSRegister) && RIESTAT(debugCRegister))
	{
		if(isFull(&recieveBuffer) == false)
		{
			int FIFOCount = UART0->RCFIFO;
			for(int i = 0; (i < (FIFOCount - 1)) && (err == NO_ERR); i++)
				loadRegister2Buffer();
			if(err == NO_ERR)
			{
				if((RDRFSTAT(debugSRegister)) )//&& (RXOFStat(UART0->SFIFO)))
					loadRegister2Buffer();
				else
					err = RXOF_ERR;
			}
		}
		else
			err = BUFFFULL_ERR;

	}

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

	uint8_t debugRegister;
	debugRegister = UART0->S1;
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
