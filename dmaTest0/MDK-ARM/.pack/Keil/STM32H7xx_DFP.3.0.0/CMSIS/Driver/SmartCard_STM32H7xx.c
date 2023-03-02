/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2022 Arm Limited (or its affiliates). All 
 * rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        11. February 2022
 * $Revision:    V2.0
 *
 * Driver:       Driver_USART1, Driver_USART2, Driver_USART3, Driver_USART6
 *               Driver_USART10
 *
 * Configured:   via STM32CubeMx configuration tool
 * Project:      USART Driver for ST STM32H7xx
 * --------------------------------------------------------------------------
 * Define used for driver configuration (at compile time):
 *
 *   SMARTCARD_DCACHE_MAINTENANCE:       0U  disable SMARTCARD DCache maintenance operations
 *                                       1U   enable SMARTCARD DCache maintenance operations
 *     - default value:                  1U
 *   SMARTCARD_DCACHE_DATA_RX_ALIGNMENT: 0U  disable SMARTCARD DCache data Rx alignment check
 *                                       1U   enable SMARTCARD DCache data Rx alignment check
 *     - default value:                  1U
 *   SMARTCARD_DCACHE_DATA_RX_SIZE:      0U  disable SMARTCARD DCache data Rx size check
 *                                       1U   enable SMARTCARD DCache data Rx size check
 *     - default value:                  1U
 *   SMARTCARD_DCACHE_DATA_TX_ALIGNMENT: 0U  disable SMARTCARD DCache data Tx alignment check
 *                                       1U   enable SMARTCARD DCache data Tx alignment check
 *     - default value:                  1U
 *   SMARTCARD_DCACHE_DATA_TX_SIZE:      0U  disable SMARTCARD DCache data Tx size check
 *                                       1U   enable SMARTCARD DCache data Tx size check
 *     - default value:                  1U
 *
 * Notes:
 *  SMARTCARD DMA Transfers, with DCache enabled:
 *    The size of DCache line on Cortex M7 is 32 Bytes. To safely perform
 *    DCache maintenance operations, data must be aligned to a 32 Byte boundary
 *    and data size must be n*32 Bytes. By default, the alignment and size of
 *    provided data are checked in SMARTCARD transfer functions. If data is not
 *    aligned to a 32 Byte boundary or data size is not n*32 Bytes, DMA is
 *    not used and IRQ transfer is performed.
 *
 *    Advanced user can disable data alignment and/or data size checking by
 *    setting SMARTCARD_DCACHE_DATA_xX_ALIGNMENT and/or SMARTCARD_DCACHE_DATA_xX_SIZE to 0.
 *    The user must be aware that performing DCache maintenance operations on
 *    data that is not aligned to a 32 Byte boundary or that is not n*32 Bytes
 *    in size can be dangerous and can corrupt other data that is also maintained
 *    by the same 32 Byte DCache line.
 *
 *  Get Count (SMARTCARD_GetTxCount, SMARTCARD_GetRxCount) functionality is supported
 *  only in interrupt mode. When DMA is used, SMARTCARD_GetTxCount and
 *  SMARTCARD_GetRxCount will return 0.
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.4
 *    Added USART10 interface accessed by Driver_USART10
 *  Version 1.3
 *    Corrected Data Cache handling (checking of data alignment and size is configurable)
 *  Version 1.2
 *    Updated Get Count functionality
 *  Version 1.1
 *    Corrected UARTx/USARTx related typing mistakes
 *  Version 1.0
 *    Initial release
 */
 
 /*! \page stm32h7_usart_smart CMSIS-Driver USART Setup in SmartCard mode

The CMSIS-Driver USART requires:
  - Setup of USART/UART peripheral in SmartCard mode

Valid settings for various evaluation boards are listed in the table below:

Peripheral Resource | STM32H743I-EVAL     |
:-------------------|:--------------------|
USART Peripheral    | USART1              |
USART Mode          | <b>SmartCard</b>    |
TX Pin              | PB14                |

For different boards, refer to the hardware schematics to reflect correct setup values.

The STM32CubeMX configuration for STM32H743I-EVAL with steps for Pinout, Clock, and System Configuration are 
listed below. Enter the values that are marked \b bold.
   
Pinout tab
----------
  1. Configure USART1 mode
     - Peripherals \b USART1: Mode=<b>SmartCard</b>
          
Clock Configuration tab
-----------------------
  1. Configure USART1 Clock.
  
Configuration tab
-----------------
  1. Under Connectivity open \b USART1 Configuration:
     - \e optional <b>DMA Settings</b>: setup DMA transfers for Rx and Tx (DMA is optional)\n
       \b Add - Select \b USART1_RX: Stream=DMA1 Channel 3, DMA Request Settings: not used\n
       \b Add - Select \b USART1_TX: Stream=DMA1 Channel 4, DMA Request Settings: not used

     - <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin | GPIO mode | GPIO Pull-up/Pull..| Maximum out | User Label
          :--------|:--------------|:----------|:-------------------|:------------|:----------
          PB14     | USART1_TX     | Alternate | Pull-up            | Very High   |
     - <b>NVIC Settings</b>: enable interrupts
          Interrupt Table                      | Enable | Preemption Priority | Sub Priority
          :------------------------------------|:-------|:--------------------|:--------------
          USART1 global interrupt              |\b ON   | 0                   | 0
          DMA1 stream5 global interrupt        |   ON   | 0                   | 0
          DMA1 stream4 global interrupt        |   ON   | 0                   | 0
     - Parameter Settings: not used
     - User Constants: not used
     - Click \b OK to close the USART1 Configuration dialog

  2. Open <b>Project - Settings - Advanced Settings</b> from the menu and enable "Not Generate Function call" for 
     MX_USART1_SMARTCARD_Init 
*/

#include "SmartCard_STM32H7xx.h"
#ifdef USARTx_MODE_SMARTCARD

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,0)

#ifndef SMARTCARD_DCACHE_MAINTENANCE
#define SMARTCARD_DCACHE_MAINTENANCE       (1U)
#endif
#ifndef SMARTCARD_DCACHE_DATA_RX_ALIGNMENT
#define SMARTCARD_DCACHE_DATA_RX_ALIGNMENT (1U)
#endif 
#ifndef SMARTCARD_DCACHE_DATA_RX_SIZE
#define SMARTCARD_DCACHE_DATA_RX_SIZE      (1U)
#endif
#ifndef SMARTCARD_DCACHE_DATA_TX_ALIGNMENT
#define SMARTCARD_DCACHE_DATA_TX_ALIGNMENT (1U)
#endif 
#ifndef SMARTCARD_DCACHE_DATA_TX_SIZE
#define SMARTCARD_DCACHE_DATA_TX_SIZE      (1U)
#endif

#define SMARTCARD_DCACHE_DATA_SIZE(rxtx)       rxtx ? SMARTCARD_DCACHE_DATA_TX_SIZE     : SMARTCARD_DCACHE_DATA_RX_SIZE
#define SMARTCARD_DCACHE_DATA_ALIGNMENT(rxtx)  rxtx ? SMARTCARD_DCACHE_DATA_TX_ALIGNMENT: SMARTCARD_DCACHE_DATA_RX_ALIGNMENT

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };

static const ARM_USART_CAPABILITIES Capabilities = {
    0,      ///< supports UART (Asynchronous) mode 
    0,      ///< supports Synchronous Master mode
    0,      ///< supports Synchronous Slave mode
    0,      ///< supports UART Single-wire mode
    0,      ///< supports UART IrDA mode
    1,      ///< supports UART Smart Card mode
    0,      ///< Smart Card Clock generator available
    0,      ///< RTS Flow Control available
    0,      ///< CTS Flow Control available
    1,      ///< Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,      ///< Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
    0,      ///< RTS Line: 0=not available, 1=available
    0,      ///< CTS Line: 0=not available, 1=available
    0,      ///< DTR Line: 0=not available, 1=available
    0,      ///< DSR Line: 0=not available, 1=available
    0,      ///< DCD Line: 0=not available, 1=available
    0,      ///< RI Line: 0=not available, 1=available
    0,      ///< Signal CTS change event: \ref ARM_USART_EVENT_CTS
    0,      ///< Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,      ///< Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0       ///< Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  };

// USART1
#ifdef USART1_MODE_SMARTCARD
SMARTCARDx_RESOURCE_ALLOC(1);
#endif

// USART2
#ifdef USART2_MODE_SMARTCARD
SMARTCARDx_RESOURCE_ALLOC(2);
#endif
  
// USART3
#ifdef USART3_MODE_SMARTCARD
SMARTCARDx_RESOURCE_ALLOC(3);
#endif

// USART6
#ifdef USART6_MODE_SMARTCARD
SMARTCARDx_RESOURCE_ALLOC(6);
#endif

// USART10
#ifdef USART10_MODE_SMARTCARD
SMARTCARDx_RESOURCE_ALLOC(10);
#endif

/**
  \fn          const SMARTCARD_RESOURCES SMARTCARD_Resources (SMARTCARD_HandleTypeDef *hsmartcard)
  \brief       Get UART_RESOURCES strusture from SMARTCARD_HandleTypeDef
*/
static const SMARTCARD_RESOURCES * SMARTCARD_Resources (SMARTCARD_HandleTypeDef *hsmartcard) {
  const SMARTCARD_RESOURCES *smartcard = NULL;

#ifdef USART1_MODE_SMARTCARD
  if (hsmartcard->Instance == USART1) { smartcard = &SMARTCARD1_Resources; }
#endif
#ifdef USART2_MODE_SMARTCARD
  if (hsmartcard->Instance == USART2) { smartcard = &SMARTCARD2_Resources; }
#endif
#ifdef USART3_MODE_SMARTCARD
  if (hsmartcard->Instance == USART3) { smartcard = &SMARTCARD3_Resources; }
#endif
#ifdef USART6_MODE_SMARTCARD
  if (hsmartcard->Instance == USART6) { smartcard = &SMARTCARD6_Resources; }
#endif
#ifdef USART10_MODE_SMARTCARD
  if (hsmartcard->Instance == USART10) { smartcard = &SMARTCARD10_Resources; }
#endif

  return smartcard;
}

/**
  \fn          void SMARTCARD_PeripheralReset (USART_TypeDef *usart)
  \brief       SMARTCARD Reset
*/
static void SMARTCARD_PeripheralReset (USART_TypeDef *usart) {

#ifdef USART1_MODE_SMARTCARD
  if (usart == USART1) { __HAL_RCC_USART1_FORCE_RESET(); }
#endif
#ifdef USART2_MODE_SMARTCARD
  if (usart == USART2) { __HAL_RCC_USART2_FORCE_RESET(); }
#endif
#ifdef USART3_MODE_SMARTCARD
  if (usart == USART3) { __HAL_RCC_USART3_FORCE_RESET(); }
#endif
#ifdef USART6_MODE_SMARTCARD
  if (usart == USART6) { __HAL_RCC_USART6_FORCE_RESET(); }
#endif
#ifdef USART10_MODE_SMARTCARD
  if (usart == USART10) { __HAL_RCC_USART10_FORCE_RESET(); }
#endif

      __NOP(); __NOP(); __NOP(); __NOP(); 
#ifdef USART1_MODE_SMARTCARD
  if (usart == USART1) { __HAL_RCC_USART1_RELEASE_RESET(); }
#endif
#ifdef USART2_MODE_SMARTCARD
  if (usart == USART2) { __HAL_RCC_USART2_RELEASE_RESET(); }
#endif
#ifdef USART3_MODE_SMARTCARD
  if (usart == USART3) { __HAL_RCC_USART3_RELEASE_RESET(); }
#endif
#ifdef USART6_MODE_SMARTCARD
  if (usart == USART6) { __HAL_RCC_USART6_RELEASE_RESET(); }
#endif
#ifdef USART10_MODE_SMARTCARD
  if (usart == USART10) { __HAL_RCC_USART10_RELEASE_RESET(); }
#endif
}

#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
/**
  \fn          int32_t SMARTCARD_GetDCacheMemBlockSize (uint32_t rxtx, uint32_t *data, uint32_t size)
  \brief       Get DCache memory block size
  \param[in]   rxtx  0 = rx, 1 = tx
  \param[in]   data  Data address
  \param[in]   size  Data size
  \return      DCache memory block size (n *32) or 0 = invalid size
*/
__STATIC_INLINE int32_t SMARTCARD_GetDCacheMemBlockSize (uint32_t rxtx, uint32_t *data, uint32_t size) {
  int32_t sz = 0;
  
#if (SMARTCARD_DCACHE_DATA_SIZE(rxtx) == 1U)
  if ((size & 0x1FU) == 0U) {
    sz = (int32_t)size;
  }
#else
  sz = (int32_t)((((uint32_t)data + size + 0x1FU) & ~0x1FU) - ((uint32_t)data & ~0x1FU));
#endif

  return sz;
}

/**
  \fn          uint32_t * SMARTCARD_GetDCacheMemBlockAddr (uint32_t rxtx, uint32_t *data)
  \brief       Get DCache memory block address
  \param[in]   rxtx  0 = rx, 1 = tx
  \param[in]   data  Data address
  \param[in]   size  Data size
  \return      DCache memory block address (aligned to 32-byte boundary) or 0 = invalid data address
*/
__STATIC_INLINE uint32_t * SMARTCARD_GetDCacheMemBlockAddr (uint32_t rxtx, uint32_t *data) {
  uint32_t *addr = NULL;
  
#if (SMARTCARD_DCACHE_DATA_ALIGNMENT(rxtx) == 1U)
  if (((uint32_t)data & 0x1FU) == 0U) {
    addr = data;
  }
#else
  addr = (uint32_t *)((uint32_t)data & ~0x1FU);
#endif

  return addr;
}
#endif

// USART Driver functions

/**
  \fn          ARM_DRIVER_VERSION SMARTCARD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION SMARTCARD_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES SMARTCARD_GetCapabilities (void)
  \brief       Get driver capabilities
  \param[in]   usart     Pointer to USART resources
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES SMARTCARD_GetCapabilities (void) {
  return Capabilities;
}

/**
  \fn          int32_t SMARTCARD_Initialize (      ARM_USART_SignalEvent_t   cb_event
                                             const SMARTCARD_RESOURCES      *smartcard)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   smartcard      Pointer to SMARTCARD resources
  \return      \ref execution_status
*/
static int32_t SMARTCARD_Initialize (      ARM_USART_SignalEvent_t   cb_event,
                                     const SMARTCARD_RESOURCES      *smartcard) {

  if (smartcard->info->flags & SMARTCARD_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  smartcard->h->Init.Mode = 0;

  // Initialize SMARTCARD Run-time Resources
  smartcard->info->cb_event = cb_event;

  // Clear transfer information
  memset(smartcard->xfer, 0, sizeof(SMARTCARD_TRANSFER_INFO));
  
  smartcard->h->Instance = smartcard->reg;

  smartcard->info->flags = SMARTCARD_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SMARTCARD_Uninitialize (const SMARTCARD_RESOURCES *smartcard)
  \brief       De-initialize SMARTCARD Interface.
  \param[in]   usart     Pointer to SMARTCARD resources
  \return      \ref execution_status
*/
static int32_t SMARTCARD_Uninitialize (const SMARTCARD_RESOURCES *smartcard) {

  // Reset SMARTCARD status flags
  smartcard->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SMARTCARD_PowerControl (ARM_POWER_STATE state)
  \brief       Control SMARTCARD Interface Power.
  \param[in]   state  Power state
  \param[in]   usart  Pointer to SMARTCARD resources
  \return      \ref execution_status
*/
static int32_t SMARTCARD_PowerControl (      ARM_POWER_STATE       state,
                                       const SMARTCARD_RESOURCES  *smartcard) {

  switch (state) {
    case ARM_POWER_OFF:

      // SMARTCARD peripheral reset
      SMARTCARD_PeripheralReset (smartcard->reg);

      if (smartcard->h->Instance != NULL) {
        HAL_SMARTCARD_MspDeInit (smartcard->h);
      }

      smartcard->info->flags &= ~SMARTCARD_FLAG_POWERED;
      break;
    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((smartcard->info->flags & SMARTCARD_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((smartcard->info->flags & SMARTCARD_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      smartcard->xfer->def_val    = 0U;

      smartcard->info->flags = SMARTCARD_FLAG_POWERED | SMARTCARD_FLAG_INITIALIZED;

      HAL_SMARTCARD_MspInit (smartcard->h);

      // SMARTCARD peripheral reset
      SMARTCARD_PeripheralReset (smartcard->reg);

    break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SMARTCARD_Send (const void                *data,
                                             uint32_t             num,
                                       const SMARTCARD_RESOURCES *smartcard)
  \brief       Start sending data to SMARTCARD transmitter.
  \param[in]   data  Pointer to buffer with data to send to SMARTCARD transmitter
  \param[in]   num   Number of data items to send
  \param[in]   usart Pointer to SMARTCARD resources
  \return      \ref execution_status
*/
static int32_t SMARTCARD_Send (const void                 *data,
                                     uint32_t              num,
                               const SMARTCARD_RESOURCES  *smartcard) {

  HAL_StatusTypeDef stat;
#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
  int32_t   mem_sz;
  uint32_t *mem_addr;
#endif

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((smartcard->info->flags & SMARTCARD_FLAG_CONFIGURED) == 0U) {
    // SMARTCARD is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  switch (HAL_SMARTCARD_GetState (smartcard->h)) {
    case HAL_SMARTCARD_STATE_RESET:
    case HAL_SMARTCARD_STATE_ERROR:
      return ARM_DRIVER_ERROR;

    case HAL_SMARTCARD_STATE_TIMEOUT:
      return ARM_DRIVER_ERROR_TIMEOUT;

    case HAL_SMARTCARD_STATE_BUSY:
    case HAL_SMARTCARD_STATE_BUSY_TX:
    case HAL_SMARTCARD_STATE_BUSY_RX:
    case HAL_SMARTCARD_STATE_BUSY_TX_RX:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_SMARTCARD_STATE_READY:
      break;
  }

  smartcard->xfer->tx_dma_flag = 0U;
  if (smartcard->dma_use_tx != 0U) {
#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      mem_addr = SMARTCARD_GetDCacheMemBlockAddr(1U, (uint32_t *)data);
      mem_sz   = SMARTCARD_GetDCacheMemBlockSize(1U, (uint32_t *)data, num);
      if ((mem_addr != NULL) && (mem_sz != 0U)) {
        smartcard->xfer->tx_dma_flag = 1U;
        // Clean data cache: ensure data coherency between DMA and CPU
        SCB_CleanDCache_by_Addr (mem_addr, mem_sz);
      }
    } else {
      smartcard->xfer->tx_dma_flag = 1U;
    }
#else
    smartcard->xfer->tx_dma_flag = 1U;
#endif
  }

  if (smartcard->xfer->tx_dma_flag != 0U) {
    // DMA mode
    stat = HAL_SMARTCARD_Transmit_DMA (smartcard->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  } else {
    // Interrupt mode
    stat = HAL_SMARTCARD_Transmit_IT (smartcard->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  }

  switch (stat) {
    case HAL_ERROR:
    case HAL_TIMEOUT:
      return ARM_DRIVER_ERROR;

    case HAL_BUSY:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_OK:
      break;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SMARTCARD_Receive (       void                 *data,
                                                 uint32_t              num,
                                           const SMARTCARD_RESOURCES  *smartcard)
  \brief       Start receiving data from SMARTCARD receiver.
  \param[out]  data  Pointer to buffer for data to receive from SMARTCARD receiver
  \param[in]   num   Number of data items to receive
  \param[in]   usart Pointer to SMARTCARD resources
  \return      \ref execution_status
*/
static int32_t SMARTCARD_Receive (      void                *data,
                                        uint32_t             num,
                                  const SMARTCARD_RESOURCES *smartcard) {

  HAL_StatusTypeDef stat;
#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
  int32_t   mem_sz;
  uint32_t *mem_addr;
#endif

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((smartcard->info->flags & SMARTCARD_FLAG_CONFIGURED) == 0U) {
    // SMARTCARD is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  switch (HAL_SMARTCARD_GetState (smartcard->h)) {
    case HAL_SMARTCARD_STATE_RESET:
    case HAL_SMARTCARD_STATE_ERROR:
      return ARM_DRIVER_ERROR;

    case HAL_SMARTCARD_STATE_TIMEOUT:
      return ARM_DRIVER_ERROR_TIMEOUT;

    case HAL_SMARTCARD_STATE_BUSY:
    case HAL_SMARTCARD_STATE_BUSY_TX:
    case HAL_SMARTCARD_STATE_BUSY_RX:
    case HAL_SMARTCARD_STATE_BUSY_TX_RX:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_SMARTCARD_STATE_READY:
      break;
  }

  // Save buffer info
  smartcard->xfer->rx_data = data;

  smartcard->xfer->rx_dma_flag = 0U;
  if (smartcard->dma_use_rx != 0U) {
#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      mem_addr = SMARTCARD_GetDCacheMemBlockAddr(0U, (uint32_t *)data);
      mem_sz   = SMARTCARD_GetDCacheMemBlockSize(0U, (uint32_t *)data, num);
      if ((mem_addr != NULL) && (mem_sz != 0U)) {
        smartcard->xfer->rx_dma_flag = 1U;
      }
    } else {
      smartcard->xfer->rx_dma_flag = 1U;
    }
#else
    smartcard->xfer->rx_dma_flag = 1U;
#endif
  }

  if (smartcard->xfer->rx_dma_flag != 0U) {
    // DMA mode
    stat = HAL_SMARTCARD_Receive_DMA (smartcard->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  } else {
    // Interrupt mode
    stat = HAL_SMARTCARD_Receive_IT (smartcard->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  }

  switch (stat) {
    case HAL_ERROR:
    case HAL_TIMEOUT:
      return ARM_DRIVER_ERROR;

    case HAL_BUSY:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_OK:
      break;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SMARTCARD_Transfer (const void                 *data_out,
                                                 void                 *data_in,
                                                 uint32_t              num,
                                           const SMARTCARD_RESOURCES  *smartcard)
  \brief       Start sending/receiving data to/from SMARTCARD transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to SMARTCARD transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SMARTCARD receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   smartcard      Pointer to SMARTCARD resources
  \return      \ref execution_status
*/
static int32_t SMARTCARD_Transfer (const void                 *data_out,
                                         void                 *data_in,
                                         uint32_t              num,
                                   const SMARTCARD_RESOURCES  *smartcard) {

  UNUSED(data_out);
  UNUSED(data_in);
  UNUSED(num);
  UNUSED(smartcard);

  // Supported only in Synchronous mode
  return ARM_DRIVER_ERROR;
}

/**
  \fn          uint32_t SMARTCARD_GetTxCount (const SMARTCARD_RESOURCES *smartcard)
  \brief       Get transmitted data count.
  \param[in]   smartcard     Pointer to SMARTCARD resources
  \return      number of data items transmitted
*/
static uint32_t SMARTCARD_GetTxCount (const SMARTCARD_RESOURCES *smartcard) {
  uint32_t cnt;

  if (smartcard->xfer->tx_dma_flag == 0U) {
    cnt = smartcard->h->TxXferSize - smartcard->h->TxXferCount;
  } else {
    cnt = 0U;
  }
  return cnt;
}

/**
  \fn          uint32_t SMARTCARD_GetRxCount (const SMARTCARD_RESOURCES *smartcard)
  \brief       Get received data count.
  \param[in]   smartcard     Pointer to SMARTCARD resources
  \return      number of data items received
*/
static uint32_t SMARTCARD_GetRxCount (const SMARTCARD_RESOURCES *smartcard) {
  uint32_t cnt;

  if (smartcard->xfer->rx_dma_flag == 0U) {
    cnt = smartcard->h->RxXferSize - smartcard->h->RxXferCount;
  } else {
    cnt = 0U;
  }
  return cnt;
}

/**
  \fn          int32_t SMARTCARD_Control (      uint32_t              control,
                                                uint32_t              arg,
                                          const SMARTCARD_RESOURCES  *smartcard)
  \brief       Control SMARTCARD Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   smartcard    Pointer to SMARTCARD resources
  \return      common \ref execution_status and driver specific \ref smartcard_execution_status
*/
static int32_t SMARTCARD_Control (     uint32_t              control,
                                       uint32_t              arg,
                                 const SMARTCARD_RESOURCES  *smartcard) {

  PLL2_ClocksTypeDef pll2_clocks;
  PLL3_ClocksTypeDef pll3_clocks;
  HAL_StatusTypeDef  status;
  uint32_t           clksrc, val = 0U;
  uint8_t            i;

  if ((smartcard->info->flags & SMARTCARD_FLAG_POWERED) == 0U) {
    // SmartCard not powered
    return ARM_DRIVER_ERROR;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
     // Control break
    case ARM_USART_CONTROL_BREAK:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    // Abort
    case ARM_USART_ABORT_SEND:
      HAL_SMARTCARD_AbortTransmit(smartcard->h);
      return ARM_DRIVER_OK;
    case ARM_USART_ABORT_RECEIVE:
      HAL_SMARTCARD_AbortReceive(smartcard->h);
      return ARM_DRIVER_OK;
    case ARM_USART_ABORT_TRANSFER:
      HAL_SMARTCARD_Abort(smartcard->h);
      return ARM_DRIVER_OK;

    // Control TX
    case ARM_USART_CONTROL_TX:
      if (arg) {
        // Transmitter enable
        smartcard->h->Init.Mode |=  SMARTCARD_MODE_TX;
      } else {
        // Transmitter disable
        smartcard->h->Init.Mode &= ~SMARTCARD_MODE_TX;
      }
      status = HAL_SMARTCARD_Init(smartcard->h);
      return SMARTCARD_HAL_STATUS(status);

    // Control RX
    case ARM_USART_CONTROL_RX:
      if (arg) {
        smartcard->h->Init.Mode |= SMARTCARD_MODE_RX;
      } else {
        // Receiver disable
        smartcard->h->Init.Mode &= ~SMARTCARD_MODE_RX;
      }
      status = HAL_SMARTCARD_Init(smartcard->h);
      return SMARTCARD_HAL_STATUS(status);
    default: break;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_MODE_IRDA:
    case ARM_USART_MODE_ASYNCHRONOUS:
    case ARM_USART_MODE_SYNCHRONOUS_MASTER:
    case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
    case ARM_USART_MODE_SINGLE_WIRE:
      return ARM_USART_ERROR_MODE;

    case ARM_USART_MODE_SMART_CARD:
      break;

    // Default TX value
    case ARM_USART_SET_DEFAULT_TX_VALUE:
      smartcard->xfer->def_val = (uint16_t)arg;
      return ARM_DRIVER_OK;

    // SmartCard guard time
    case ARM_USART_SET_SMART_CARD_GUARD_TIME:
      if (arg > 255U) return ARM_DRIVER_ERROR;
      smartcard->h->Init.GuardTime = arg & 0xFF;
      HAL_SMARTCARD_Init(smartcard->h);
      return ARM_DRIVER_OK;

    // SmartCard clock
    case ARM_USART_SET_SMART_CARD_CLOCK:
      SMARTCARD_GETCLOCKSOURCE(smartcard->h, clksrc);
      switch (clksrc)
      {
        case SMARTCARD_CLOCKSOURCE_D2PCLK1:
          val = HAL_RCC_GetPCLK1Freq();
          break;
        case SMARTCARD_CLOCKSOURCE_D2PCLK2:
          val = HAL_RCC_GetPCLK2Freq();
          break;
        case SMARTCARD_CLOCKSOURCE_PLL2Q:
          HAL_RCCEx_GetPLL2ClockFreq(&pll2_clocks);
          val = pll2_clocks.PLL2_Q_Frequency;
          break;
        case SMARTCARD_CLOCKSOURCE_PLL3Q:
          HAL_RCCEx_GetPLL3ClockFreq(&pll3_clocks);
          val = pll3_clocks.PLL3_Q_Frequency;
          break;
        case SMARTCARD_CLOCKSOURCE_HSI:
         if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U) {
          val = (HSI_VALUE >> (__HAL_RCC_GET_HSI_DIVIDER()>> 3));
         }
         else {
           val = HSI_VALUE;
         }
        break;
        case SMARTCARD_CLOCKSOURCE_CSI:
          val = CSI_VALUE;
          break;
        case SMARTCARD_CLOCKSOURCE_LSE:
          val = LSE_VALUE;
          break;
        case SMARTCARD_CLOCKSOURCE_UNDEFINED:
          return ARM_DRIVER_ERROR;
      }

      // Calculate period in ns
      val = 1000000000U / val;
      for (i = 1U; i <64U; i++) {
        // if in +-2% tolerance
        if (((val * i * 2U * 100U) < (arg * 102U)) &&
            ((val * i * 2U * 100U) > (arg * 98U))    ) {
          break;
        }
      }
      if (i == 64U) { return ARM_DRIVER_ERROR; }
      smartcard->h->Init.Prescaler = i;
      HAL_SMARTCARD_Init(smartcard->h);
      return ARM_DRIVER_OK;

    // SmartCard NACK
    case ARM_USART_CONTROL_SMART_CARD_NACK:
      if (arg != 0U) {
        smartcard->h->Init.NACKEnable = SMARTCARD_NACK_ENABLE;
      } else {
        smartcard->h->Init.NACKEnable = SMARTCARD_NACK_DISABLE;
      }
      HAL_SMARTCARD_Init(smartcard->h);
      return ARM_DRIVER_OK;

    // Unsupported command
    default: { return ARM_DRIVER_ERROR_UNSUPPORTED; }
  }

  // SmartCard Data bits
  if ((control & ARM_USART_DATA_BITS_Msk) == ARM_USART_DATA_BITS_8) {
    smartcard->h->Init.WordLength = SMARTCARD_WORDLENGTH_9B;
  }
  else{
    return ARM_USART_ERROR_DATA_BITS;
  }

  // SmartCard Parity
  switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_EVEN:
      smartcard->h->Init.Parity = SMARTCARD_PARITY_EVEN;
      break;
    case ARM_USART_PARITY_ODD:
      smartcard->h->Init.Parity = SMARTCARD_PARITY_ODD;
      break;
    case ARM_USART_PARITY_NONE:
    default: return ARM_USART_ERROR_PARITY;
  }

  // SmartCard Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:
      break;
    case ARM_USART_STOP_BITS_2:
    case ARM_USART_STOP_BITS_1_5:
    case ARM_USART_STOP_BITS_0_5:
    default: return ARM_USART_ERROR_STOP_BITS;
  }

  // SmartCard Flow control
  switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
      break;
    case ARM_USART_FLOW_CONTROL_RTS:
    case ARM_USART_FLOW_CONTROL_CTS:
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
    default: return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // SmartCard Baudrate
  smartcard->h->Init.BaudRate = arg;

  // Set configured flag
  smartcard->info->flags |= SMARTCARD_FLAG_CONFIGURED;

  // Initialize SmartCard
  status = HAL_SMARTCARD_Init(smartcard->h);
  return SMARTCARD_HAL_STATUS(status);
}

/**
  \fn          ARM_USART_STATUS SMARTCARD_GetStatus (const SMARTCARD_RESOURCES *smartcard)
  \brief       Get SMARTCARD status.
  \param[in]   smartcard     Pointer to SMARTCARD resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS SMARTCARD_GetStatus (const SMARTCARD_RESOURCES *smartcard) {
  ARM_USART_STATUS  status;
  uint32_t          error;

  status.rx_busy = 0;
  status.tx_busy = 0;
  switch (HAL_SMARTCARD_GetState (smartcard->h)) {
    case HAL_SMARTCARD_STATE_BUSY:
    case HAL_SMARTCARD_STATE_BUSY_TX_RX:
      status.rx_busy = 1;
      status.tx_busy = 1;
      break;
    case HAL_SMARTCARD_STATE_BUSY_TX:
      status.tx_busy = 1;
      break;
    case HAL_SMARTCARD_STATE_BUSY_RX:
      status.rx_busy = 1;
      break;

    case HAL_SMARTCARD_STATE_TIMEOUT:
    case HAL_SMARTCARD_STATE_READY:
    case HAL_SMARTCARD_STATE_RESET:
    case HAL_SMARTCARD_STATE_ERROR:
      break;
  }

  error = HAL_SMARTCARD_GetError (smartcard->h);

  if (error & HAL_SMARTCARD_ERROR_PE) {
    status.rx_parity_error = 1;
  } else {
    status.rx_parity_error = 0;
  }

  if (error & HAL_SMARTCARD_ERROR_FE) {
    status.rx_framing_error = 1;
  } else {
    status.rx_framing_error = 0;
  }

  if (error & HAL_SMARTCARD_ERROR_ORE) {
    status.rx_overflow = 1;
  } else {
    status.rx_overflow = 0;
  }

  status.tx_underflow  = 0;

  return status;
}

/**
  \fn          int32_t SMARTCARD_SetModemControl (ARM_USART_MODEM_CONTROL  control
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t SMARTCARD_SetModemControl (ARM_USART_MODEM_CONTROL  control) {

  (void) control;

  // No modem control in synchronous mode
  return ARM_DRIVER_ERROR;
}

/**
  \fn          ARM_USART_MODEM_STATUS SMARTCARD_GetModemStatus (void)
  \brief       Get UART Modem Status lines state.
  \return      modem status \ref ARM_UART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS SMARTCARD_GetModemStatus (void) {
  ARM_USART_MODEM_STATUS modem_status;

  modem_status.cts = 0U;
  modem_status.dsr = 0U;
  modem_status.ri  = 0U;
  modem_status.dcd = 0U;

  return modem_status;
}

/**
  * @brief Tx Transfer completed callback.
  * @param husart: SMARTCARD handle.
  * @retval None
  */
void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard) {
  const SMARTCARD_RESOURCES * smartcard;

  smartcard = SMARTCARD_Resources (hsmartcard);

  if (smartcard->info->cb_event != NULL) {
    smartcard->info->cb_event(ARM_USART_EVENT_TX_COMPLETE | ARM_USART_EVENT_SEND_COMPLETE);
  }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param husart: SMARTCARD handle.
  * @retval None
  */
void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard) {
  const SMARTCARD_RESOURCES * smartcard;
#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
  int32_t   mem_sz;
  uint32_t *mem_addr;
#endif

  smartcard = SMARTCARD_Resources (hsmartcard);

#if (SMARTCARD_DCACHE_MAINTENANCE == 1U)
  if (smartcard->xfer->rx_dma_flag != 0U) {
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      mem_addr = SMARTCARD_GetDCacheMemBlockAddr(0U, (uint32_t *)smartcard->xfer->rx_data);
      mem_sz   = SMARTCARD_GetDCacheMemBlockSize(0U, (uint32_t *)smartcard->xfer->rx_data, smartcard->h->RxXferSize);
      if ((mem_addr != NULL) && (mem_sz != 0U)) {
        // Invalidate data cache: ensure data coherency between DMA and CPU
        SCB_InvalidateDCache_by_Addr (mem_addr, mem_sz);
      }
    }
  }
#endif

  if (smartcard->info->cb_event != NULL) {
    smartcard->info->cb_event(ARM_USART_EVENT_RECEIVE_COMPLETE);
  }
}

/**
  * @brief SMARTCARD error callback.
  * @param hsmartcard: SMARTCARD handle.
  * @retval None
  */
void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsmartcard) {
  const SMARTCARD_RESOURCES * smartcard;
        uint32_t         error;
        uint32_t         event;

  smartcard = SMARTCARD_Resources (hsmartcard);

  error = HAL_SMARTCARD_GetError (smartcard->h);
  event = 0;

  if (error & HAL_SMARTCARD_ERROR_PE) {
    event |= ARM_USART_EVENT_RX_PARITY_ERROR;
  }
  if (error & HAL_SMARTCARD_ERROR_FE) {
    event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
  }
  if (error & HAL_SMARTCARD_ERROR_ORE) {
    event |= ARM_USART_EVENT_RX_OVERFLOW;
  }

  if ((event != 0) && (smartcard->info->cb_event != NULL)) {
    smartcard->info->cb_event(event);
  }
}

#ifdef USART1_MODE_SMARTCARD
SMARTCARDx_EXPORT_DRIVER(1);
#endif

#ifdef USART2_MODE_SMARTCARD
SMARTCARDx_EXPORT_DRIVER(2);
#endif

#ifdef USART3_MODE_SMARTCARD
SMARTCARDx_EXPORT_DRIVER(3);
#endif

#ifdef USART6_MODE_SMARTCARD
SMARTCARDx_EXPORT_DRIVER(6);
#endif

#ifdef USART10_MODE_SMARTCARD
SMARTCARDx_EXPORT_DRIVER(10);
#endif

#endif

/*! \endcond */
