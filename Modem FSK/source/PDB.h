#ifndef PDB_H_
#define PDB_H_

#include "stdbool.h"
#include "stdint.h"

typedef enum {
  PDB_IMMEDIATELY,
  PDB_ON_COUNTER_OVERFLOW,
  PDB_ON_TRIGGER_INPUT,
  PDB_ON_COUNTER_OVERFLOW_OR_TRIGGER_INPUT
}PDB_LoadValueMode;

typedef enum {
  PDB_PRESCALER_DIVIDER_1 = 0U,
  PDB_PRESCALER_DIVIDER_2 = 1U,
  PDB_PRESCALER_DIVIDER_4 = 2U,
  PDB_PRESCALER_DIVIDER_8 = 3U,
  PDB_PRESCALER_DIVIDER_16 = 4U,
  PDB_PRESCALER_DIVIDER_32 = 5U,
  PDB_PRESCALER_DIVIDER_64 = 6U,
  PDB_PRESCALER_DIVIDER_128 = 7U
}PDB_Prescaler;

typedef enum {
  PDB_MULT_FACTOR_1 = 0U,
  PDB_MULT_FACTOR_10 = 1U,
  PDB_MULT_FACTOR_20 = 2U,
  PDB_MULT_FACTOR_40 = 3U
}PDB_MultiplicationFactor;

typedef enum {
  PDB_TRIGGER_INPUT_0 = 0U,
  PDB_TRIGGER_INPUT_1 = 1U,
  PDB_TRIGGER_INPUT_2 = 2U,
  PDB_TRIGGER_INPUT_3 = 3U,
  PDB_TRIGGER_INPUT_4 = 4U,
  PDB_TRIGGER_INPUT_5 = 5U,
  PDB_TRIGGER_INPUT_6 = 6U,
  PDB_TRIGGER_INPUT_7 = 7U,
  PDB_TRIGGER_INPUT_8 = 8U,
  PDB_TRIGGER_INPUT_9 = 9U,
  PDB_TRIGGER_INPUT_10 = 10U,
  PDB_TRIGGER_INPUT_11 = 11U,
  PDB_TRIGGER_INPUT_12 = 12U,
  PDB_TRIGGER_INPUT_13 = 13U,
  PDB_TRIGGER_INPUT_14 = 14U,
  PDB_TRIGGER_SOFTWARE = 15U
}PDB_TriggerSource;

typedef enum{PDB_PreTrigger0, PDB_PreTrigger1}PDB_PreTrigger;
typedef enum{PDB_Channel0, PDB_Channel1}PDB_Channel;
typedef struct{
	PDB_LoadValueMode loadValueMode;
	PDB_Prescaler prescalerDivider;
	PDB_MultiplicationFactor multiplicationFactor;
	PDB_TriggerSource triggerInputSource;
	bool enableContinuousMode;
	uint16_t MODValue;
}PDB_Config;

void PDB_GetDefaultConfig(PDB_Config * config);

void PDB_Init(PDB_Config * config);

void PDB_Deinit(void);

void PDB_Enable(void);

void PDB_Disable(void);

void PDB_Trigger(void);

void PDB_LoadValues(void);

void PDB_EnableDMA(bool enable);

void PDB_EnableInterrupts(uint32_t mask);

void PDB_DisableInterrupts(uint32_t mask);

uint32_t PDB_GetStatusFlags(void);

void PDB_ClearStatusFlags(uint32_t mask);

void PDB_SetChannelDelay(PDB_PreTrigger m, PDB_Channel n,  uint32_t CHDelay);

/**
 * @brief Specifies the period of the counter.
 * When the counter reaches this value, it will be reset back to zero.
 * If the PDB is in Continuous mode, the count begins anew.
 */
void PDB_SetModulusValue(uint16_t value);

/**
 *
 */
uint32_t PDB_GetCounterValue(void);

/**
 *
 */
void PDB_SetCounterDelayValue(uint32_t value);

/**
 *
 */
void PDB_SetDacTriggerPeriod(uint16_t value);

/**
 *
 */
void PDB_EnableDACTrigger(bool enable);
void PDB_EnableADCTrigger(void);

#endif /* PDB_H_ */
