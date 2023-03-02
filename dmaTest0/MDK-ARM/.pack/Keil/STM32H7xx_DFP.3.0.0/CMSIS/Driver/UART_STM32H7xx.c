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
 * Driver:       Driver_USART1, Driver_USART2, Driver_USART3, Driver_USART4,
 *               Driver_USART5, Driver_USART6, Driver_USART7, Driver_USART8,
 *               Driver_USART9, Driver_USART10, Driver_USART21
 *
 * Configured:   via STM32CubeMx configuration tool
 * Project:      USART Driver for ST STM32H7xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value   UART Interface
 *   ---------------------                   -----   --------------
 *   Connect to hardware via Driver_USART# = 1       use USART1
 *   Connect to hardware via Driver_USART# = 2       use USART2
 *   Connect to hardware via Driver_USART# = 3       use USART3
 *   Connect to hardware via Driver_USART# = 4       use UART4
 *   Connect to hardware via Driver_USART# = 5       use UART5
 *   Connect to hardware via Driver_USART# = 6       use USART6
 *   Connect to hardware via Driver_USART# = 7       use UART7
 *   Connect to hardware via Driver_USART# = 8       use UART8
 *   Connect to hardware via Driver_USART# = 9       use UART9
 *   Connect to hardware via Driver_USART# = 10      use USART10
 *   Connect to hardware via Driver_USART# = 21      use LPUART1
 *
 * Note:
 * LPUART1 interface accessed by Driver_USART21 (previously by Driver_USART9).
 * --------------------------------------------------------------------------
 * Define used for driver configuration (at compile time):
 *
 *   UART_DCACHE_MAINTENANCE:       0U  disable UART DCache maintenance operations
 *                                  1U   enable UART DCache maintenance operations
 *     - default value:             1U
 *   UART_DCACHE_DATA_RX_ALIGNMENT: 0U  disable UART DCache data Rx alignment check
 *                                  1U   enable UART DCache data Rx alignment check
 *     - default value:             1U
 *   UART_DCACHE_DATA_RX_SIZE:      0U  disable UART DCache data Rx size check
 *                                  1U   enable UART DCache data Rx size check
 *     - default value:             1U
 *   UART_DCACHE_DATA_TX_ALIGNMENT: 0U  disable UART DCache data Tx alignment check
 *                                  1U   enable UART DCache data Tx alignment check
 *     - default value:             1U
 *   UART_DCACHE_DATA_TX_SIZE:      0U  disable UART DCache data Tx size check
 *                                  1U   enable UART DCache data Tx size check
 *     - default value:             1U
 *
 * Notes:
 *  UART DMA Transfers, with DCache enabled:
 *    The size of DCache line on Cortex M7 is 32 Bytes. To safely perform
 *    DCache maintenance operations, data must be aligned to a 32 Byte boundary
 *    and data size must be n*32 Bytes. By default, the alignment and size of
 *    provided data are checked in UART transfer functions. If data is not
 *    aligned to a 32 Byte boundary or data size is not n*32 Bytes, DMA is
 *    not used and IRQ transfer is performed.
 *
 *    Advanced user can disable data alignment and/or data size checking by
 *    setting UART_DCACHE_DATA_xX_ALIGNMENT and/or UART_DCACHE_DATA_xX_SIZE to 0.
 *    The user must be aware that performing DCache maintenance operations on
 *    data that is not aligned to a 32 Byte boundary or that is not n*32 Bytes
 *    in size can be dangerous and can corrupt other data that is also maintained
 *    by the same 32 Byte DCache line.
 *
 *  Get Count (UART_GetTxCount, UART_GetRxCount) functionality is supported
 *  only in interrupt mode. When DMA is used, UART_GetTxCount and
 *  UART_GetRxCount will return 0.
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.0
 *    LPUART1 interface accessed by Driver_USART21 (previously by Driver_USART9)
 *    Added USART10 interface accessed by Driver_USART10
 *    Added LPUART1 interface accessed by Driver_USART21
 *  Version 1.3
 *    Added UART interface for LPUART1 (Driver_USART9)
 *    Corrected Data Cache handling (checking of data alignment and size is configurable)
 *  Version 1.2
 *    Updated Get Count functionality
 *  Version 1.1
 *    Corrected UARTx/USARTx related typing mistakes
 *  Version 1.0
 *    Initial release
 */
 
  /*! \page stm32h7_uart CMSIS-Driver UART Setup

The CMSIS-Driver USART requires:
  - Setup of USART/UART mode (Asynchronous, Synchronous, Single wire, IrDA or SmartCard)
  - Optional configuration of Hardware Flow Control (only in Asynchronous mode).
 
Valid settings for various evaluation boards are listed in the table below:

Peripheral Resource | STM32H743I-EVAL     |
:-------------------|:--------------------|
USART Peripheral    | USART1              |
USART Mode          | <b>Asynchronous</b> |
TX Pin              | PB14                |
RX Pin              | PB15                |

For different boards, refer to the hardware schematics to reflect correct setup values.

The STM32CubeMX configuration for STM32H743I-EVAL with steps for Pinout, Clock, and System Configuration are 
listed below. Enter the values that are marked \b bold.
   
Pinout tab
----------
  1. Configure USART1 mode
     - Peripherals \b USART1: Mode=<b>Asynchronous</b>, Hardware Flow Control=<b>Disable</b>
          
Clock Configuration tab
-----------------------
  1. Configure USART1 Clock.
  
Configuration tab
-----------------
  1. Under Connectivity open \b USART1 Configuration:
     - \e optional <b>DMA Settings</b>: setup DMA transfers for Rx and Tx (DMA is optional)\n
       \b Add - Select \b USART1_RX: Stream=DMA1 Channel 3, DMA Request Settings: not used\n
       \b Add - Select \b USART1_TX: Stream=DMA1 Channel 4, DMA Request Settings: not used

     \n\note
     LPUART1 transfers with BDMA:\n
         BDMA can access only SRAM4 memory. Ensure that Tx and Rx buffers are be positioned in SRAM4 memory, when LPUART1 is used in DMA mode (BDMA).

     \n
     - <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin | GPIO mode | GPIO Pull-up/Pull..| Maximum out | User Label
          :--------|:--------------|:----------|:-------------------|:------------|:----------
          PB14     | USART1_TX     | Alternate | Pull-up            | Very High   |.
          PB15     | USART1_RX     | Alternate | Pull-up            | Very High   |.
     - <b>NVIC Settings</b>: enable interrupts
          Interrupt Table                      | Enable | Preemption Priority | Sub Priority
          :------------------------------------|:-------|:--------------------|:--------------
          USART1 global interrupt              |\b ON   | 0                   | 0
          DMA1 stream3 global interrupt        |   ON   | 0                   | 0
          DMA1 stream4 global interrupt        |   ON   | 0                   | 0
     - Parameter Settings: not used
     - User Constants: not used
     - Click \b OK to close the USART1 Configuration dialog

  2. Open <b>Project - Settings - Advanced Settings</b> from the menu and enable "Not Generate Function call" for 
     MX_USART1_UART_Init 
*/
 
/*! \cond */
#include "UART_STM32H7xx.h"
#ifdef USARTx_MODE_ASYNC

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,0)

#ifndef UART_DCACHE_MAINTENANCE
#define UART_DCACHE_MAINTENANCE       (1U)
#endif
#ifndef UART_DCACHE_DATA_RX_ALIGNMENT
#define UART_DCACHE_DATA_RX_ALIGNMENT (1U)
#endif 
#ifndef UART_DCACHE_DATA_RX_SIZE
#define UART_DCACHE_DATA_RX_SIZE      (1U)
#endif
#ifndef UART_DCACHE_DATA_TX_ALIGNMENT
#define UART_DCACHE_DATA_TX_ALIGNMENT (1U)
#endif 
#ifndef UART_DCACHE_DATA_TX_SIZE
#define UART_DCACHE_DATA_TX_SIZE      (1U)
#endif

#define UART_DCACHE_DATA_SIZE(rxtx)       rxtx ? UART_DCACHE_DATA_TX_SIZE     : UART_DCACHE_DATA_RX_SIZE
#define UART_DCACHE_DATA_ALIGNMENT(rxtx)  rxtx ? UART_DCACHE_DATA_TX_ALIGNMENT: UART_DCACHE_DATA_RX_ALIGNMENT

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };

static const ARM_USART_CAPABILITIES Capabilities = {
    1,      ///< supports UART (Asynchronous) mode 
    0,      ///< supports Synchronous Master mode
    0,      ///< supports Synchronous Slave mode
    0,      ///< supports UART Single-wire mode
    0,      ///< supports UART IrDA mode
    0,      ///< supports UART Smart Card mode
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
#ifdef USART1_MODE_ASYNC
UARTx_RESOURCE_ALLOC(1);
#endif

// USART2
#ifdef USART2_MODE_ASYNC
UARTx_RESOURCE_ALLOC(2);
#endif

// USART3
#ifdef USART3_MODE_ASYNC
UARTx_RESOURCE_ALLOC(3);
#endif

// UART4
#ifdef USART4_MODE_ASYNC
UARTx_RESOURCE_ALLOC(4);
#endif

// UART5
#ifdef USART5_MODE_ASYNC
UARTx_RESOURCE_ALLOC(5);
#endif

// UART6
#ifdef USART6_MODE_ASYNC
UARTx_RESOURCE_ALLOC(6);
#endif

// UART7
#ifdef USART7_MODE_ASYNC
UARTx_RESOURCE_ALLOC(7);
#endif

// UART8
#ifdef USART8_MODE_ASYNC
UARTx_RESOURCE_ALLOC(8);
#endif

// UART9
#ifdef USART9_MODE_ASYNC
UARTx_RESOURCE_ALLOC(9);
#endif

// UART10
#ifdef USART10_MODE_ASYNC
UARTx_RESOURCE_ALLOC(10);
#endif

// UART21
#ifdef USART21_MODE_ASYNC
UARTx_RESOURCE_ALLOC(21);
#endif

/**
  \fn          const UART_RESOURCES UART_Resources (UART_HandleTypeDef *huart)
  \brief       Get UART_RESOURCES strusture from UART_HandleTypeDef
*/
static const UART_RESOURCES * UART_Resources (UART_HandleTypeDef *huart) {
  const UART_RESOURCES *uart = NULL;

#ifdef USART1_MODE_ASYNC
  if (huart->Instance == USART1)  { uart = &UART1_Resources; }
#endif
#ifdef USART2_MODE_ASYNC
  if (huart->Instance == USART2)  { uart = &UART2_Resources; }
#endif
#ifdef USART3_MODE_ASYNC
  if (huart->Instance == USART3)  { uart = &UART3_Resources; }
#endif
#ifdef USART4_MODE_ASYNC
  if (huart->Instance == UART4)   { uart = &UART4_Resources; }
#endif
#ifdef USART5_MODE_ASYNC
  if (huart->Instance == UART5)   { uart = &UART5_Resources; }
#endif
#ifdef USART6_MODE_ASYNC
  if (huart->Instance == USART6)  { uart = &UART6_Resources; }
#endif
#ifdef USART7_MODE_ASYNC
  if (huart->Instance == UART7)   { uart = &UART7_Resources; }
#endif
#ifdef USART8_MODE_ASYNC
  if (huart->Instance == UART8)   { uart = &UART8_Resources; }
#endif
#ifdef USART9_MODE_ASYNC
  if (huart->Instance == UART9) { uart = &UART9_Resources; }
#endif
#ifdef USART10_MODE_ASYNC
  if (huart->Instance == USART10) { uart = &UART10_Resources; }
#endif
#ifdef USART21_MODE_ASYNC
  if (huart->Instance == LPUART1) { uart = &UART21_Resources; }
#endif

  return uart;
}

/**
  \fn          void UART_PeripheralReset (USART_TypeDef *usart)
  \brief       UART Reset
*/
static void UART_PeripheralReset (USART_TypeDef *usart) {

#ifdef USART1_MODE_ASYNC
  if (usart == USART1) { __HAL_RCC_USART1_FORCE_RESET(); }
#endif
#ifdef USART2_MODE_ASYNC
  if (usart == USART2) { __HAL_RCC_USART2_FORCE_RESET(); }
#endif
#ifdef USART3_MODE_ASYNC
  if (usart == USART3) { __HAL_RCC_USART3_FORCE_RESET(); }
#endif
#ifdef USART4_MODE_ASYNC
  if (usart == UART4)  { __HAL_RCC_UART4_FORCE_RESET(); }
#endif
#ifdef USART5_MODE_ASYNC
  if (usart == UART5)  { __HAL_RCC_UART5_FORCE_RESET(); }
#endif
#ifdef USART6_MODE_ASYNC
  if (usart == USART6) { __HAL_RCC_USART6_FORCE_RESET(); }
#endif
#ifdef USART7_MODE_ASYNC
  if (usart == UART7)  { __HAL_RCC_UART7_FORCE_RESET(); }
#endif
#ifdef USART8_MODE_ASYNC
  if (usart == UART8)  { __HAL_RCC_UART8_FORCE_RESET(); }
#endif
#ifdef USART9_MODE_ASYNC
  if (usart == LPUART1)  { __HAL_RCC_LPUART1_FORCE_RESET(); }
#endif
      __NOP(); __NOP(); __NOP(); __NOP(); 
#ifdef USART1_MODE_ASYNC
  if (usart == USART1) { __HAL_RCC_USART1_RELEASE_RESET(); }
#endif
#ifdef USART2_MODE_ASYNC
  if (usart == USART2) { __HAL_RCC_USART2_RELEASE_RESET(); }
#endif
#ifdef USART3_MODE_ASYNC
  if (usart == USART3) { __HAL_RCC_USART3_RELEASE_RESET(); }
#endif
#ifdef USART4_MODE_ASYNC
  if (usart == UART4)  { __HAL_RCC_UART4_RELEASE_RESET(); }
#endif
#ifdef USART5_MODE_ASYNC
  if (usart == UART5)  { __HAL_RCC_UART5_RELEASE_RESET(); }
#endif
#ifdef USART6_MODE_ASYNC
  if (usart == USART6) { __HAL_RCC_USART6_RELEASE_RESET(); }
#endif
#ifdef USART7_MODE_ASYNC
  if (usart == UART7)  { __HAL_RCC_UART7_RELEASE_RESET(); }
#endif
#ifdef USART8_MODE_ASYNC
  if (usart == UART8)  { __HAL_RCC_UART8_RELEASE_RESET(); }
#endif
#ifdef USART9_MODE_ASYNC
  if (usart == UART9)  { __HAL_RCC_UART9_RELEASE_RESET(); }
#endif
#ifdef USART10_MODE_ASYNC
  if (usart == USART10)  { __HAL_RCC_USART10_RELEASE_RESET(); }
#endif
#ifdef USART21_MODE_ASYNC
  if (usart == LPUART1)  { __HAL_RCC_LPUART1_RELEASE_RESET(); }
#endif
}

#if (UART_DCACHE_MAINTENANCE == 1U)
/**
  \fn          int32_t UART_GetDCacheMemBlockSize (uint32_t rxtx, uint32_t *data, uint32_t size)
  \brief       Get DCache memory block size
  \param[in]   rxtx  0 = rx, 1 = tx
  \param[in]   data  Data address
  \param[in]   size  Data size
  \return      DCache memory block size (n *32) or 0 = invalid size
*/
__STATIC_INLINE int32_t UART_GetDCacheMemBlockSize (uint32_t rxtx, uint32_t *data, uint32_t size) {
  int32_t sz = 0;
  
#if (UART_DCACHE_DATA_SIZE(rxtx) == 1U)
  if ((size & 0x1FU) == 0U) {
    sz = (int32_t)size;
  }
#else
  sz = (int32_t)((((uint32_t)data + size + 0x1FU) & ~0x1FU) - ((uint32_t)data & ~0x1FU));
#endif

  return sz;
}

/**
  \fn          uint32_t * UART_GetDCacheMemBlockAddr (uint32_t rxtx, uint32_t *data)
  \brief       Get DCache memory block address
  \param[in]   rxtx  0 = rx, 1 = tx
  \param[in]   data  Data address
  \param[in]   size  Data size
  \return      DCache memory block address (aligned to 32-byte boundary) or 0 = invalid data address
*/
__STATIC_INLINE uint32_t * UART_GetDCacheMemBlockAddr (uint32_t rxtx, uint32_t *data) {
  uint32_t *addr = NULL;
  
#if (UART_DCACHE_DATA_ALIGNMENT(rxtx) == 1U)
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
  \fn          ARM_DRIVER_VERSION UART_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION UART_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES UART_GetCapabilities (void)
  \brief       Get driver capabilities
  \param[in]   usart     Pointer to USART resources
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES UART_GetCapabilities (void) {
  return Capabilities;
}

/**
  \fn          int32_t UART_Initialize (      ARM_USART_SignalEvent_t  cb_event
                                         const UART_RESOURCES          *uart)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   uart      Pointer to UART resources
  \return      \ref execution_status
*/
static int32_t UART_Initialize (      ARM_USART_SignalEvent_t  cb_event,
                                 const UART_RESOURCES          *uart) {

  if (uart->info->flags & UART_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  uart->h->Init.Mode = 0;

  // Initialize UART Run-time Resources
  uart->info->cb_event = cb_event;

  // Clear transfer information
  memset(uart->xfer, 0, sizeof(UART_TRANSFER_INFO));
  
  uart->h->Instance = (USART_TypeDef *)uart->reg;

  uart->info->flags = UART_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t UART_Uninitialize (const UART_RESOURCES *uart)
  \brief       De-initialize UART Interface.
  \param[in]   usart     Pointer to UART resources
  \return      \ref execution_status
*/
static int32_t UART_Uninitialize (const UART_RESOURCES *uart) {

  // Reset UART status flags
  uart->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t UART_PowerControl (ARM_POWER_STATE state)
  \brief       Control UART Interface Power.
  \param[in]   state  Power state
  \param[in]   usart  Pointer to UART resources
  \return      \ref execution_status
*/
static int32_t UART_PowerControl (      ARM_POWER_STATE  state,
                                   const UART_RESOURCES  *uart) {

  switch (state) {
    case ARM_POWER_OFF:

      // UART peripheral reset
      UART_PeripheralReset (uart->reg);

      if (uart->h->Instance != NULL) {
        HAL_UART_MspDeInit (uart->h);
      }

      uart->info->flags &= ~UART_FLAG_POWERED;
      break;
    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((uart->info->flags & UART_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((uart->info->flags & UART_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      uart->xfer->def_val    = 0U;

      uart->info->flags = UART_FLAG_POWERED | UART_FLAG_INITIALIZED;

      HAL_UART_MspInit (uart->h);

      // UART peripheral reset
      UART_PeripheralReset (uart->reg);

    break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t UART_Send (const void            *data,
                                        uint32_t         num,
                                  const UART_RESOURCES *uart)
  \brief       Start sending data to UART transmitter.
  \param[in]   data  Pointer to buffer with data to send to UART transmitter
  \param[in]   num   Number of data items to send
  \param[in]   usart Pointer to UART resources
  \return      \ref execution_status
*/
static int32_t UART_Send (const void            *data,
                                uint32_t         num,
                          const UART_RESOURCES  *uart) {

  HAL_StatusTypeDef stat;
#if (UART_DCACHE_MAINTENANCE == 1U)
  int32_t   mem_sz;
  uint32_t *mem_addr;
#endif

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((uart->info->flags & UART_FLAG_CONFIGURED) == 0U) {
    // UART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  switch (HAL_UART_GetState (uart->h)) {
    case HAL_UART_STATE_RESET:
    case HAL_UART_STATE_ERROR:
      return ARM_DRIVER_ERROR;

    case HAL_UART_STATE_TIMEOUT:
      return ARM_DRIVER_ERROR_TIMEOUT;

    case HAL_UART_STATE_BUSY:
    case HAL_UART_STATE_BUSY_TX:
    case HAL_UART_STATE_BUSY_TX_RX:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_UART_STATE_BUSY_RX:
    case HAL_UART_STATE_READY:
      break;
  }

  uart->xfer->tx_dma_flag = 0U;
  if (uart->dma_use_tx != 0U) {
#if (UART_DCACHE_MAINTENANCE == 1U)
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      mem_addr = UART_GetDCacheMemBlockAddr(1U, (uint32_t *)data);
      mem_sz   = UART_GetDCacheMemBlockSize(1U, (uint32_t *)data, num);
      if ((mem_addr != NULL) && (mem_sz != 0U)) {
        uart->xfer->tx_dma_flag = 1U;
        // Clean data cache: ensure data coherency between DMA and CPU
        SCB_CleanDCache_by_Addr (mem_addr, mem_sz);
      }
    } else {
      uart->xfer->tx_dma_flag = 1U;
    }
#else
    uart->xfer->tx_dma_flag = 1U;
#endif
  }

  if (uart->xfer->tx_dma_flag != 0U) {
    // DMA mode
    stat = HAL_UART_Transmit_DMA (uart->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  } else {
    // Interrupt mode
    stat = HAL_UART_Transmit_IT (uart->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
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
  \fn          int32_t UART_Receive (      void            *data,
                                           uint32_t         num,
                                     const UART_RESOURCES  *uart)
  \brief       Start receiving data from UART receiver.
  \param[out]  data  Pointer to buffer for data to receive from UART receiver
  \param[in]   num   Number of data items to receive
  \param[in]   usart Pointer to UART resources
  \return      \ref execution_status
*/
static int32_t UART_Receive (      void           *data,
                                   uint32_t        num,
                             const UART_RESOURCES *uart) {

  HAL_StatusTypeDef stat;
#if (UART_DCACHE_MAINTENANCE == 1U)
  int32_t   mem_sz;
  uint32_t *mem_addr;
#endif

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((uart->info->flags & UART_FLAG_CONFIGURED) == 0U) {
    // UART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  switch (HAL_UART_GetState (uart->h)) {
    case HAL_UART_STATE_RESET:
    case HAL_UART_STATE_ERROR:
      return ARM_DRIVER_ERROR;

    case HAL_UART_STATE_TIMEOUT:
      return ARM_DRIVER_ERROR_TIMEOUT;

    case HAL_UART_STATE_BUSY:
    case HAL_UART_STATE_BUSY_RX:
    case HAL_UART_STATE_BUSY_TX_RX:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_UART_STATE_BUSY_TX:
    case HAL_UART_STATE_READY:
      break;
  }

  // Save buffer info
  uart->xfer->rx_data = data;

  uart->xfer->rx_dma_flag = 0U;
  if (uart->dma_use_rx != 0U) {
#if (UART_DCACHE_MAINTENANCE == 1U)
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      mem_addr = UART_GetDCacheMemBlockAddr(0U, (uint32_t *)data);
      mem_sz   = UART_GetDCacheMemBlockSize(0U, (uint32_t *)data, num);
      if ((mem_addr != NULL) && (mem_sz != 0U)) {
        uart->xfer->rx_dma_flag = 1U;
      }
    } else {
      uart->xfer->rx_dma_flag = 1U;
    }
#else
    uart->xfer->rx_dma_flag = 1U;
#endif
  }

  if (uart->xfer->rx_dma_flag != 0U) {
    // DMA mode
    stat = HAL_UART_Receive_DMA (uart->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  } else {
    // Interrupt mode
    stat = HAL_UART_Receive_IT (uart->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
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
  \fn          int32_t UART_Transfer (const void             *data_out,
                                             void            *data_in,
                                             uint32_t         num,
                                       const UART_RESOURCES  *uart)
  \brief       Start sending/receiving data to/from UART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to UART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from UART receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   uart      Pointer to UART resources
  \return      \ref execution_status
*/
static int32_t UART_Transfer (const void             *data_out,
                                    void             *data_in,
                                    uint32_t          num,
                              const UART_RESOURCES   *uart) {

  UNUSED(data_out);
  UNUSED(data_in);
  UNUSED(num);
  UNUSED(uart);

  // Supported only in Synchronous mode
  return ARM_DRIVER_ERROR;
}

/**
  \fn          uint32_t UART_GetTxCount (const UART_RESOURCES *uart)
  \brief       Get transmitted data count.
  \param[in]   uart     Pointer to UART resources
  \return      number of data items transmitted
*/
static uint32_t UART_GetTxCount (const UART_RESOURCES *uart) {
  uint32_t cnt;

  if (uart->xfer->tx_dma_flag == 0U) {
    cnt = uart->h->TxXferSize - uart->h->TxXferCount;
  } else {
    cnt = 0U;
  }
  return cnt;
}

/**
  \fn          uint32_t UART_GetRxCount (const UART_RESOURCES *uart)
  \brief       Get received data count.
  \param[in]   uart     Pointer to UART resources
  \return      number of data items received
*/
static uint32_t UART_GetRxCount (const UART_RESOURCES *uart) {
  uint32_t cnt;

  if (uart->xfer->rx_dma_flag == 0U) {
    cnt = uart->h->RxXferSize - uart->h->RxXferCount;
  } else {
    cnt = 0U;
  }
  return cnt;
}

/**
  \fn          int32_t UART_Control (      uint32_t          control,
                                           uint32_t          arg,
                                      const UART_RESOURCES  *uart)
  \brief       Control UART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   uart    Pointer to UART resources
  \return      common \ref execution_status and driver specific \ref uart_execution_status
*/
static int32_t UART_Control (      uint32_t         control,
                                   uint32_t         arg,
                             const UART_RESOURCES  *uart) {

  HAL_StatusTypeDef status;

  if ((uart->info->flags & UART_FLAG_POWERED) == 0U) {
    // UART not powered
    return ARM_DRIVER_ERROR;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
     // Control break
    case ARM_USART_CONTROL_BREAK:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    // Abort
    case ARM_USART_ABORT_SEND:
      HAL_UART_AbortTransmit(uart->h);
      return ARM_DRIVER_OK;
    case ARM_USART_ABORT_RECEIVE:
      HAL_UART_AbortReceive(uart->h);
      return ARM_DRIVER_OK;
    case ARM_USART_ABORT_TRANSFER:
      HAL_UART_Abort(uart->h);
      return ARM_DRIVER_OK;

    // Control TX
    case ARM_USART_CONTROL_TX:
      if (arg) {
        // Transmitter enable
        uart->h->Init.Mode |=  UART_MODE_TX;
      } else {
        // Transmitter disable
        uart->h->Init.Mode &= ~UART_MODE_TX;
      }
      status = HAL_UART_Init(uart->h);
      return UART_HAL_STATUS(status);

    // Control RX
    case ARM_USART_CONTROL_RX:
      if (arg) {
        uart->h->Init.Mode |= UART_MODE_RX;
      } else {
        // Receiver disable
        uart->h->Init.Mode &= ~UART_MODE_RX;
      }
      status = HAL_UART_Init(uart->h);
      return UART_HAL_STATUS(status);
    default: break;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_MODE_IRDA:
    case ARM_USART_MODE_SMART_CARD:
    case ARM_USART_MODE_SYNCHRONOUS_MASTER:
    case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
    case ARM_USART_MODE_SINGLE_WIRE:
      return ARM_USART_ERROR_MODE;

    case ARM_USART_MODE_ASYNCHRONOUS:
      break;

    // Default TX value
    case ARM_USART_SET_DEFAULT_TX_VALUE:
      uart->xfer->def_val = (uint16_t)arg;
      return ARM_DRIVER_OK;

    default: { return ARM_DRIVER_ERROR_UNSUPPORTED; }
  }

  // USART Data bits
  switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_6:
      if ((control & ARM_USART_PARITY_Msk) != ARM_USART_PARITY_NONE) {
        uart->h->Init.WordLength = UART_WORDLENGTH_7B;
      } else {
        return ARM_USART_ERROR_DATA_BITS;
      }
      break;
    case ARM_USART_DATA_BITS_7:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        uart->h->Init.WordLength = UART_WORDLENGTH_7B;
      } else {
        uart->h->Init.WordLength = UART_WORDLENGTH_8B;
      }
      break;
    case ARM_USART_DATA_BITS_8:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        uart->h->Init.WordLength = UART_WORDLENGTH_8B;
      } else {
        uart->h->Init.WordLength = UART_WORDLENGTH_9B;
      }
      break;
    case ARM_USART_DATA_BITS_9:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        uart->h->Init.WordLength = UART_WORDLENGTH_9B;
      } else {
        return ARM_USART_ERROR_DATA_BITS;
      }
      break;
    default: return ARM_USART_ERROR_DATA_BITS;
  }

  // USART Parity
  switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE:
      uart->h->Init.Parity = UART_PARITY_NONE;
      break;
    case ARM_USART_PARITY_EVEN:
      uart->h->Init.Parity = UART_PARITY_EVEN;
      break;
    case ARM_USART_PARITY_ODD:
      uart->h->Init.Parity = UART_PARITY_ODD;
      break;
    default: return ARM_USART_ERROR_PARITY;
  }

  // USART Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:
      uart->h->Init.StopBits = UART_STOPBITS_1;
      break;
    case ARM_USART_STOP_BITS_2:
      uart->h->Init.StopBits = UART_STOPBITS_2;
      break;
    case ARM_USART_STOP_BITS_1_5:
      uart->h->Init.StopBits = UART_STOPBITS_1_5;
      break;
    case ARM_USART_STOP_BITS_0_5:
      uart->h->Init.StopBits = UART_STOPBITS_0_5;
      break;
    default: return ARM_USART_ERROR_STOP_BITS;
  }

  // USART Flow control
  switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
      uart->h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
      break;
    case ARM_USART_FLOW_CONTROL_RTS:
      uart->h->Init.HwFlowCtl = UART_HWCONTROL_RTS;
      break;
    case ARM_USART_FLOW_CONTROL_CTS:
      uart->h->Init.HwFlowCtl = UART_HWCONTROL_CTS;
      break;
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
      uart->h->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
      break;
    default: return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // USART Baudrate
  uart->h->Init.BaudRate = arg;

  // Set configured flag
  uart->info->flags |= UART_FLAG_CONFIGURED;

  // Initialize UART
  status = HAL_UART_Init(uart->h);

  // Reconfigure DMA
  if ((uart->dma_use_tx != 0) && (uart->h->hdmatx !=NULL)) {
    if ((control & ARM_USART_DATA_BITS_Msk) > ARM_USART_DATA_BITS_8) {
      uart->h->hdmatx->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      uart->h->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    } else {
      uart->h->hdmatx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      uart->h->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    }
    HAL_DMA_Init(uart->h->hdmatx);
  }
  
  if ((uart->dma_use_rx != 0) && (uart->h->hdmarx !=NULL)) {
    if ((control & ARM_USART_DATA_BITS_Msk) > ARM_USART_DATA_BITS_8) {
      uart->h->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      uart->h->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    } else {
      uart->h->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      uart->h->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    }
    HAL_DMA_Init(uart->h->hdmarx);
  }
  return UART_HAL_STATUS(status);
}

/**
  \fn          ARM_USART_STATUS UART_GetStatus (const UART_RESOURCES *uart)
  \brief       Get UART status.
  \param[in]   uart     Pointer to UART resources
  \return      UART status \ref ARM_UART_STATUS
*/
static ARM_USART_STATUS UART_GetStatus (const UART_RESOURCES *uart) {
  ARM_USART_STATUS  status;
  uint32_t          error;

  status.rx_busy = 0;
  status.tx_busy = 0;
  switch (HAL_UART_GetState (uart->h)) {
    case HAL_UART_STATE_BUSY:
    case HAL_UART_STATE_BUSY_TX_RX:
      status.rx_busy = 1;
      status.tx_busy = 1;
      break;
    case HAL_UART_STATE_BUSY_TX:
      status.tx_busy = 1;
      break;
    case HAL_UART_STATE_BUSY_RX:
      status.rx_busy = 1;
      break;

    case HAL_UART_STATE_TIMEOUT:
    case HAL_UART_STATE_READY:
    case HAL_UART_STATE_RESET:
    case HAL_UART_STATE_ERROR:
      break;
  }

  error = HAL_UART_GetError (uart->h);

  if (error & HAL_UART_ERROR_PE) {
    status.rx_parity_error = 1;
  } else {
    status.rx_parity_error = 0;
  }

  if (error & HAL_UART_ERROR_FE) {
    status.rx_framing_error = 1;
  } else {
    status.rx_framing_error = 0;
  }

  if (error & HAL_UART_ERROR_ORE) {
    status.rx_overflow = 1;
  } else {
    status.rx_overflow = 0;
  }

  status.tx_underflow  = 0;

  return status;
}

/**
  \fn          int32_t UART_SetModemControl (       ARM_USART_MODEM_CONTROL  control)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t UART_SetModemControl (       ARM_USART_MODEM_CONTROL  control) {

  (void) control;

  // No modem control in synchronous mode
  return ARM_DRIVER_ERROR;
}

/**
  \fn          ARM_USART_MODEM_STATUS UART_GetModemStatus (void)
  \brief       Get UART Modem Status lines state.
  \return      modem status \ref ARM_UART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS UART_GetModemStatus (void) {
  ARM_USART_MODEM_STATUS modem_status;

  modem_status.cts = 0U;
  modem_status.dsr = 0U;
  modem_status.ri  = 0U;
  modem_status.dcd = 0U;

  return modem_status;
}

/**
  * @brief Tx Transfer completed callback.
  * @param husart: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  const UART_RESOURCES * uart;

  uart = UART_Resources (huart);

  if (uart->info->cb_event != NULL) {
    uart->info->cb_event(ARM_USART_EVENT_TX_COMPLETE | ARM_USART_EVENT_SEND_COMPLETE);
  }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param husart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  const UART_RESOURCES * uart;
#if (UART_DCACHE_MAINTENANCE == 1U)
  int32_t   mem_sz;
  uint32_t *mem_addr;
#endif

  uart = UART_Resources (huart);

#if (UART_DCACHE_MAINTENANCE == 1U)
  if (uart->xfer->rx_dma_flag != 0U) {
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      mem_addr = UART_GetDCacheMemBlockAddr(0U, (uint32_t *)(uint32_t)uart->xfer->rx_data);
      mem_sz   = UART_GetDCacheMemBlockSize(0U, (uint32_t *)(uint32_t)uart->xfer->rx_data, uart->h->RxXferSize);
      if ((mem_addr != NULL) && (mem_sz != 0U)) {
        // Invalidate data cache: ensure data coherency between DMA and CPU
        SCB_InvalidateDCache_by_Addr (mem_addr, mem_sz);
      }
    }
  }
#endif

  if (uart->info->cb_event != NULL) {
    uart->info->cb_event(ARM_USART_EVENT_RECEIVE_COMPLETE);
  }
}

/**
  * @brief UART error callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  const UART_RESOURCES * uart;
        uint32_t         error;
        uint32_t         event;

  uart = UART_Resources (huart);

  error = HAL_UART_GetError (uart->h);
  event = 0;

  if (error & HAL_UART_ERROR_PE) {
    event |= ARM_USART_EVENT_RX_PARITY_ERROR;
  }
  if (error & HAL_UART_ERROR_FE) {
    event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
  }
  if (error & HAL_UART_ERROR_ORE) {
    event |= ARM_USART_EVENT_RX_OVERFLOW;
  }

  if ((event != 0) && (uart->info->cb_event != NULL)) {
    uart->info->cb_event(event);
  }
}

#ifdef USART1_MODE_ASYNC
UARTx_EXPORT_DRIVER(1);
#endif

#ifdef USART2_MODE_ASYNC
UARTx_EXPORT_DRIVER(2);
#endif

#ifdef USART3_MODE_ASYNC
UARTx_EXPORT_DRIVER(3);
#endif

#ifdef USART4_MODE_ASYNC
UARTx_EXPORT_DRIVER(4);
#endif

#ifdef USART5_MODE_ASYNC
UARTx_EXPORT_DRIVER(5);
#endif

#ifdef USART6_MODE_ASYNC
UARTx_EXPORT_DRIVER(6);
#endif

#ifdef USART7_MODE_ASYNC
UARTx_EXPORT_DRIVER(7);
#endif

#ifdef USART8_MODE_ASYNC
UARTx_EXPORT_DRIVER(8);
#endif

#ifdef USART9_MODE_ASYNC
UARTx_EXPORT_DRIVER(9);
#endif

#ifdef USART10_MODE_ASYNC
UARTx_EXPORT_DRIVER(10);
#endif

#ifdef USART21_MODE_ASYNC
UARTx_EXPORT_DRIVER(21);
#endif

#endif /*! \endcond */

