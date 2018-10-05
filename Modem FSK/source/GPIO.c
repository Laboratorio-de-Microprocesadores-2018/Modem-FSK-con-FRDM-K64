/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "./gpio.h"
#include "hardware.h"
#include <string.h>
/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

#define PINS 32
#define PORTS 6
#define PIN_PORT(x) (x)/32UL /*gets the port number from the result of PORTNUMTOPIN*/
#define PIN_POS(x) (x)%32UL	/*gets the pin number,inside a specific port, from the result of PORTNUMTOPIN*/


/*
 * structure for a small buffer which also has it's size as a field
 */
typedef struct  {
	uint8_t *arr;
	uint8_t size;
}buffer;

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

static PORT_Type * const ports[] = PORT_BASE_PTRS;
static GPIO_Type * const gpio[] = GPIO_BASE_PTRS;


/*
 * Array of inturruption services for each pin of each port
 * */
static pinIrqFun_t portAIRQs[32];
static pinIrqFun_t portBIRQs[32];
static pinIrqFun_t portCIRQs[32];
static pinIrqFun_t portDIRQs[32];
static pinIrqFun_t portEIRQs[32];
static pinIrqFun_t  * const portsIRQs[5]={portAIRQs,portBIRQs,portCIRQs,portDIRQs,portEIRQs};//



/*
 * Array of buffers containing the pin number, in the given port [0,31], of each pin with an
 * enabled interruption
 * */
static uint8_t portAIndxs[32];
static uint8_t portBIndxs[32];
static uint8_t portCIndxs[32];
static uint8_t portDIndxs[32];
static uint8_t portEIndxs[32];

buffer buffA={portAIndxs,0};
buffer buffB={portBIndxs,0};
buffer buffC={portCIndxs,0};
buffer buffD={portDIndxs,0};
buffer buffE={portEIndxs,0};

static buffer *const portsIRQIndxs[5]={&buffA,&buffB,&buffC,&buffD,&buffE};
/*-----------------------------------------------------------------------------------------*/


/**
 * @brief Configures the specified pin to behave either as an input or an output
 * @param pin the pin whose mode you wish to set (according PORTNUM2PIN)
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void pinMode (uint8_t pin, uint8_t mode)
{
	if(pin >= PORTS*PINS)
		return;

	uint8_t portnum = PIN_PORT(pin); //Port Number (A,B,C,D,E)
	uint8_t pinpos = PIN_POS(pin); // Pin Number [0:31]


	// Pin configuration as GPIO
	ports[portnum]->PCR[pinpos] &= ~PORT_PCR_MUX_MASK;
	ports[portnum]->PCR[pinpos] |= PORT_PCR_MUX(1);

	switch(mode)
	{
	case INPUT:
		ports[portnum]->PCR[pinpos] &= ~PORT_PCR_PE(1); // Pull Enable=0
		gpio[portnum]->PDDR &= ~(1UL<<pinpos); // Set as 0 the n bit of the corresponding PDDR
		break;
	case INPUT_PULLUP:
		ports[portnum]->PCR[pinpos] |= PORT_PCR_PE(1); // Pull Enable=1
		ports[portnum]->PCR[pinpos] |= PORT_PCR_PS(1); // Pull Select=1
		gpio[portnum]->PDDR &= ~(1UL<<pinpos);
		break;
	case INPUT_PULLDOWN:
		ports[portnum]->PCR[pinpos] |= PORT_PCR_PE(1); // Pull Enable=1
		ports[portnum]->PCR[pinpos] &= ~PORT_PCR_PS(1); // Pull Select=0
		gpio[portnum]->PDDR &= ~(1UL<<pinpos);
		break;
	case OUTPUT:
		ports[portnum]->PCR[pinpos] &= ~PORT_PCR_PE(1); // Pull Enable=0
		gpio[portnum]->PDDR |= (1UL<<pinpos);
		break;
	}
}

/**
 * @brief Write a HIGH or a LOW value to a digital pin
 * @param id the pin to write (according PORTNUM2PIN)
 * @param val Desired value (HIGH or LOW)
 */
void digitalWrite (uint8_t pin, uint8_t v)
{
	if(pin >= PORTS*PINS)
		return;

	uint8_t value=v&1;


	uint8_t i = PIN_PORT(pin);//Port Number (A,B,C,D,E)
	uint8_t n = PIN_POS(pin); //Pin number [0,31]
	static GPIO_Type * const gpio[] = GPIO_BASE_PTRS;

	gpio[i]->PDOR = ((gpio[i]->PDOR & (~(1<<n))) | (value<<n));

}

/**
 * @brief Toggle the value of a digital pin (HIGH<->LOW)
 * @param id the pin to toggle (according PORTNUM2PIN)
 */
void digitalToggle (uint8_t pin)
{

	if(pin >= PORTS*PINS)
		return;

	uint8_t i = PIN_PORT(pin);//Port Number (A,B,C,D,E)
	uint8_t n = PIN_POS(pin); //Pin number [0,31]
	static GPIO_Type * const gpio[] = GPIO_BASE_PTRS;
	gpio[i]->PTOR |= 1<<n;
}

/**
 * @brief Reads the value from a specified digital pin, either HIGH or LOW.
 * @param id the pin to read (according PORTNUM2PIN)
 * @return HIGH or LOW
 */
uint8_t digitalRead (uint8_t pin)
{
	if(pin >= PORTS*PINS)
		return 0xFF;

	uint8_t i = PIN_PORT(pin);//Port Number (A,B,C,D,E)
	uint8_t n = PIN_POS(pin); //Pin number [0,31]
	static GPIO_Type * const gpio[] = GPIO_BASE_PTRS;
	return ((gpio[i]->PDIR>>n) & 1);
}

/**
 * @brief Configures how the pin reacts when an IRQ event ocurrs
 * @param pin the pin whose IRQ mode you wish to set (according PORTNUM2PIN)
 * @param irqMode disable, risingEdge, fallingEdge or bothEdges
 * @param irqFun function to call on pin event
 * @return Registration succeed
 */
uint8_t pinConfigureIRQ (uint8_t pin, uint8_t irqMode, pinIrqFun_t irqFun)
{


	uint8_t port=PIN_PORT(pin);
	uint8_t pos=PIN_POS(pin);

	uint8_t retVal=1;
	if(pin >= 6*32 || (irqMode!=IRQC_DISABLE && irqFun == NULL))
		retVal=0;//Error
	else
	{
		//Set the callback
		portsIRQs[port][pos]=irqFun;

		//IRQC Mode set
		ports[port]->PCR[pos] &= ~PORT_PCR_IRQC_MASK;//Sets IRQC part in 0000
		switch(irqMode)
		{
			case IRQC_DISABLE:
				ports[port]->PCR[pos] |= PORT_PCR_IRQC(IRQC_DISABLE);//Sets IRQC for disable mode

				//Searches for the index in the buffer and deletes it
				for(uint8_t i=0; i<portsIRQIndxs[port]->size ; i++)
				{
					if(portsIRQIndxs[port]->arr[i]==pos)
					{
						for(uint8_t j=i; j<portsIRQIndxs[port]->size-1; j++)
						{
							portsIRQIndxs[port]->arr[j]=portsIRQIndxs[port]->arr[j+1];
						}
						portsIRQIndxs[port]->size--;
						break;
					}
				}

				break;
			case IRQC_INTERRUPT_RISING: case IRQC_INTERRUPT_FALLING	: case IRQC_INTERRUPT_EITHER:
				NVIC_EnableIRQ(PORTA_IRQn+port);
				ports[port]->PCR[pos] |= PORT_PCR_IRQC(irqMode);//Sets IRQC for the specified mode
				uint8_t i=0;

				//Adds the index to the buffer if it was not there already
				while(i<portsIRQIndxs[port]->size)
				{
					if(portsIRQIndxs[port]->arr[i]!=pos)
						i++;
					else
						break;
				}
				if(i==portsIRQIndxs[port]->size)
				{	//The index was not in the buffer
					portsIRQIndxs[port]->arr[i]=pos;
					portsIRQIndxs[port]->size++;
				}

				break;
			default:
				retVal=0;//Error
				break;
		}
	}
	return retVal;
}



/*
 * All IRQ Handlers work the same way
 * */
void PORTA_IRQHandler(void)
{
	if(buffA.size!=0)//if there is any pin enabled for interrupts
	{
		uint8_t pin=0;
		for(uint8_t i=0; i<buffA.size;++i)//for pin in PORTA
		{
			pin=buffA.arr[i];
			if( (ports[PA]->PCR[pin]&PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)//checks every pins' ISF
			{
				portAIRQs[pin]();//callback
				ports[PA]->PCR[pin] |= PORT_PCR_ISF(1);//Sets to 0 the interrupt status flag
			}
		}
	}

}
void PORTB_IRQHandler(void)
{
	if(buffB.size!=0)
	{
		uint8_t pin=0;
		for(uint8_t i=0; i<buffB.size;++i)
		{
			pin=buffB.arr[i];
			if( (ports[PB]->PCR[pin]&PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
			{
				portBIRQs[pin]();
				ports[PB]->PCR[pin] |= PORT_PCR_ISF(1);//Sets to 0 the interrupt status flag
			}
		}
	}
}
void PORTC_IRQHandler(void)
{
	if(buffC.size!=0)
	{
		uint8_t pin=0;
		for(uint8_t i=0; i<buffC.size;++i)
		{
			pin=buffC.arr[i];
			if( (ports[PC]->PCR[pin]&PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
			{
				portCIRQs[pin]();
				ports[PC]->PCR[pin] |= PORT_PCR_ISF(1);//Sets to 0 the interrupt status flag
			}
		}
	}
}
void PORTD_IRQHandler(void)
{
	if(buffD.size!=0)
	{
		uint8_t pin=0;
		for(uint8_t i=0; i<buffD.size;++i)
		{
			pin=buffD.arr[i];
			if( (ports[PD]->PCR[pin]&PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
			{
				portDIRQs[pin]();
				ports[PD]->PCR[pin] |= PORT_PCR_ISF(1); //Sets to 0 the interrupt status flag by setting a 1
			}
		}
	}
}
void PORTE_IRQHandler(void)
{
	if(buffE.size!=0)
	{
		uint8_t pin=0;
		for(uint8_t i=0; i<buffE.size;++i)
		{
			pin=buffE.arr[i];
			if( (ports[PE]->PCR[pin]&PORT_PCR_ISF_MASK) == PORT_PCR_ISF_MASK)
			{
				portEIRQs[pin]();
				ports[PE]->PCR[pin] |= PORT_PCR_ISF(1);//Sets to 0 the interrupt status flag
			}
		}
	}
}
