

typedef enum{
	UART0_RX,
	UART0_TX,
	UART1_RX,
	UART1_TX,
	UART2_RX,
	UART2_TX,
	UART3_RX,
	UART3_TX,
	UART4_RX_TX,
	UART5_RX_TX,
	I2S0_RX,
	I2S0_TX,
	SPI0_RX,
	SPI0_TX,
	SPI1_RX_TX,
	SPI2_RX_TX,
	I2C0,
	I2C1,
	FTM0_CH0,
	FTM0_CH1,
	FTM0_CH2,
	FTM0_CH3,
	FTM0_CH4,
	FTM0_CH5,
	FTM0_CH6,
	FTM0_CH7,
	FTM1_CH0,
	FTM1_CH1,
	FTM2_CH0,
	FTM2_CH1,
	FTM3_CH0,
	FTM3_CH1,
	FTM3_CH2,
	FTM3_CH3,
	FTM3_CH4,
	FTM3_CH5,
	FTM3_CH6,
	FTM3_CH7,
	ADC0,
	ADC1,
	CMP0,
	CMP1,
	CMP2,
	DAC0,
	DAC1,
	CMT ,
	PDB ,
	PortA,
	PortB,
	PortC,
	PortD,
	PortE,
	IEEETimer0,
	IEEETimer1,
	IEEETimer2,
	IEEETimer3,
	AlwaysEnabled0,
	AlwaysEnabled1,
	AlwaysEnabled2,
	AlwaysEnabled3,
	AlwaysEnabled4,
	AlwaysEnabled5
} DMAMUX_Source;





void DMAMUX_Init ();
void DMAMUX_Deinit ();
static void DMAMUX_EnableChannel ( uint32_t channel);
static void DMAMUX_DisableChannel (uint32_t channel);
static void DMAMUX_SetSource ( uint32_t channel, DMAMUX_Source source);
