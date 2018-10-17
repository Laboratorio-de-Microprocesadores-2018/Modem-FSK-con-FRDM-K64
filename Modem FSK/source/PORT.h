#ifndef _PORT_H_
#define _PORT_H_

#include "stdint.h"

typedef enum {
	PORT_A,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E
}PORT_Instance;

typedef enum {
	PORT_MuxDisabled,
	PORT_MuxGPIO,
	PORT_MuxAlt2,
	PORT_MuxAlt3,
	PORT_MuxAlt4,
	PORT_MuxAlt5,
	PORT_MuxAlt6,
	PORT_MuxAlt7
}PORT_Mux;


typedef enum {
	PORT_PullDisable = 2U, 
	PORT_PullDown = 0U, 
	PORT_PullUp = 1U 
}PORT_Pull;

typedef enum { 
	PORT_FastSlewRate = 0U, 
	PORT_SlowSlewRate = 1U 
}PORT_SlewRate;

typedef enum {
  PORT_PassiveFilterDisable = 0U, 
  PORT_PassiveFilterEnable = 1U 
}PORT_PassiveFilter;

typedef enum {
  PORT_OpenDrainDisable = 0U, 
  PORT_OpenDrainEnable = 1U 
}PORT_OpenDrain;

typedef enum {
  PORT_LowDriveStrength = 0U, 
  PORT_HighDriveStrength = 1U 
}PORT_DriveStrength;

typedef enum {
  PORT_LockRegister = 0U, 
  PORT_UnlockRegister = 1U 
}PORT_Lock;


typedef enum { 
  PORT_InterruptOrDMADisabled = 0x0U, 
  PORT_InterruptRisingEdge = 0x9U, 
  PORT_InterruptFallingEdge = 0xAU, 
  PORT_InterruptEitherEdge = 0xBU, 
  PORT_InterruptLogicZero = 0x8U, 
  PORT_InterruptLogicOne = 0xCU 
}PORT_Interrupt;


typedef struct{
	PORT_Lock lk;
	PORT_Mux mux;
	PORT_Pull pull;
	PORT_SlewRate sr;
	PORT_OpenDrain od;
	PORT_PassiveFilter filter;
	PORT_DriveStrength ds;
	PORT_Interrupt interrupt;
}PORT_Config;

void 	PORT_PinConfig (PORT_Instance n, uint32_t pin, PORT_Config *config);

void 	PORT_MultiplePinsConfig (PORT_Instance n, uint32_t mask, PORT_Config *config);

void 	PORT_PinMux (PORT_Instance n, uint32_t pin, PORT_Mux mux);

void    PORT_PinInterruptConfig (PORT_Instance n, uint32_t pin, PORT_Interrupt interrupt);

uint32_t PORT_GetPinInterruptFlag (PORT_Instance n,uint32_t pin);

uint32_t PORT_GetPinsInterruptFlags (PORT_Instance n);

void 	PORT_ClearPinInterruptFlag (PORT_Instance n, uint32_t pin);



#endif //_PORT_H_
