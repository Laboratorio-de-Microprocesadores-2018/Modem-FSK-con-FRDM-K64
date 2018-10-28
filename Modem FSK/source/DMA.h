/////////////////////////////////////////////////////////////////////////////////
//                         													   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

#ifndef DMA_H_
#define DMA_H_
/**
 * @file     DMA.h
 * @brief    DMA driver to configure and set automatic data transfers
 */
/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "stdint.h"
#include "stdbool.h"
/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

typedef void (*DMAIrqFun_t)(void);

typedef enum
{
    DMA_TransferSize1Bytes,
    DMA_TransferSize2Bytes,
    DMA_TransferSize4Bytes,
    DMA_TransferSize16Bytes,
    DMA_TransferSize32Bytes
} DMA_TransferSize;

typedef enum
{
    DMA_ModuloDisable,
    DMA_Modulo2bytes,
    DMA_Modulo4bytes,
    DMA_Modulo8bytes,
    DMA_Modulo16bytes,
    DMA_Modulo32bytes,
    DMA_Modulo64bytes,
    DMA_Modulo128bytes,
    DMA_Modulo256bytes,
    DMA_Modulo512bytes,
    DMA_Modulo1Kbytes,
    DMA_Modulo2Kbytes,
    DMA_Modulo4Kbytes,
    DMA_Modulo8Kbytes,
    DMA_Modulo16Kbytes,
    DMA_Modulo32Kbytes,
    DMA_Modulo64Kbytes,
    DMA_Modulo128Kbytes,
    DMA_Modulo256Kbytes,
    DMA_Modulo512Kbytes,
    DMA_Modulo1Mbytes,
    DMA_Modulo2Mbytes,
    DMA_Modulo4Mbytes,
    DMA_Modulo8Mbytes,
    DMA_Modulo16Mbytes,
    DMA_Modulo32Mbytes,
    DMA_Modulo64Mbytes,
    DMA_Modulo128Mbytes,
    DMA_Modulo256Mbytes,
    DMA_Modulo512Mbytes,
    DMA_Modulo1Gbytes,
    DMA_Modulo2Gbytes
} DMA_Modulo;

typedef enum
{
    DMA_BandwidthStallNone = 0,
    DMA_BandwidthStall4Cycle = 2,
    DMA_BandwidthStall8Cycle = 3
} DMA_Bandwidth;

typedef enum
{
    DMA_LinkNone,
    DMA_MinorLink,
    DMA_MajorLink
} DMA_ChannelLinkType;

typedef enum
{
    DMA_DoneFlag = 1,
    DMA_ErrorFlag = 2,
    DMA_InterruptFlag = 4
}DMA_ChannelStatusFlag;

typedef enum
{
    DMA_ErrorInterruptEnable = 1,
    DMA_MajorInterruptEnable = 2,
    DMA_HalfInterruptEnable = 4,
} DMA_InterruptEnable;


typedef enum
{
    DMA_Status_QueueFull,
    DMA_Status_Busy
}DMA_TransferStatus;

typedef struct
{
    bool enableContinuousLinkMode;
    bool enableHaltOnError;
    bool enableRoundRobinArbitration;
    bool enableDebugMode;
} DMA_Config;


typedef struct
{
    uint32_t sourceAddress;
    uint32_t destinationAddress;
    DMA_TransferSize sourceTransferSize;
    DMA_TransferSize destinationTransferSize;
    int16_t sourceOffset;
    int16_t destinationOffset;
    uint16_t minorLoopBytes;
    uint32_t majorLoopCounts;
    uint32_t sourceLastAdjust;
    uint32_t destinationLastAdjust;
} DMA_TransferConfig;

typedef struct
{
    bool enableSrcMinorOffset;
    bool enableDestMinorOffset;
    uint32_t minorOffset;
} DMA_MinorOffsetConfig;


struct DMA_Handle;

typedef void (*DMA_Callback)(struct DMA_Handle *handle, void *userData, bool transferDone, uint32_t tcds);

typedef struct
{
    DMA_Callback callback;
    void *userData;
    uint8_t channel;
    uint8_t flags;
} DMA_Handle;
/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Gives a default configuration for the DMA module
 * @param config struct where the function will return the default configuration
 */
void DMA_GetDefaultConfig(DMA_Config * config);

/**
 * @brief Initializes DMA module
 *
 * Ungate the DMA clock and configure DMA peripheral.
 */
void DMA_Init(DMA_Config *config);

/**
 *	@brief Sets a a transfer configuration for a certain DMA channel
 *	@param channel used
 *	@param config , struct containing the configuration wished for the transfer
 */
void DMA_SetTransferConfig	(uint32_t channel,DMA_TransferConfig * 	config);

/**
 * @brief enables interrupts in a desired channel
 * @param channel
 */
void DMA_EnableInterrupts (uint32_t channel);
/**
 * @brief disables interrupts in a desired channel
 * @param channel
 */
void DMA_DisableInterrupts (uint32_t channel);
/**
 * @brief software triggers the data transfer configured in a desired channel
 * @param channel to trigger
 */
void DMA_TriggerChannelStart (uint32_t channel);
/**
 * @brief Enables DMA requests for the selected channel
 * @param channel
 */
void DMA_EnableChannelRequest (uint32_t channel);
/**
 * @brief Disables DMA requests for the selected channel
 * @param channel
 */
void DMA_DisableChannelRequest (uint32_t channel);
/**
 * @brief Sets a new source address for the DMA transfer for the selected channel
 * @param channel
 * @param newAddress
 */
void DMA_ModifySourceAddress(uint32_t channel, uint32_t newAddress);
/**
 * @brief Sets a callback to be executed when a transfer ended. The callback should
 * be as quick as posible as it is called inside an interruption
 * @param channel
 * @param majorIntCallback   callback
 */
void DMA_SetCallback(uint32_t channel,DMAIrqFun_t majorIntCallback);

#endif /* DMA_H_ */
