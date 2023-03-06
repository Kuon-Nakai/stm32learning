/* -----------------------------------------------------------------------------
 * Copyright (c) 2021 Arm Limited (or its affiliates).
 * All rights reserved.
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
 * $Date:        27. October 2021
 * $Revision:    V1.0
 * Project:      USART Driver definitions for ST STM32G4xx
 * -------------------------------------------------------------------------- */

#ifndef __USART_STM32G4XX_H
#define __USART_STM32G4XX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_USART.h"
#include "stm32g4xx_hal.h"

#include "RTE_Components.h"
#include "MX_Device.h"

#define  VM_ASYNC                      (1UL)
#define  VM_SYNC                       (2UL)
#define  VM_IRDA                       (3UL)
#define  VM_SMARTCARD                  (4UL)
#define  Asynchronous                  VM_ASYNC
#define  IrDA                          VM_IRDA

#define USART_HAL_STATUS(stat)       ((stat == HAL_OK)      ? ARM_DRIVER_OK :            \
                                     ((stat == HAL_BUSY)    ? ARM_DRIVER_ERROR_BUSY :    \
                                     ((stat == HAL_TIMEOUT) ? ARM_DRIVER_ERROR_TIMEOUT : \
                                                              ARM_DRIVER_ERROR)))



#define USARTx_RESOURCE_ALLOC(x)     extern USART_HandleTypeDef husart##x;                          \
                                     static USART_INFO            USART##x##_Info;                  \
                                     static USART_TRANSFER_INFO   USART##x##_TransferInfo;          \
                                     static const USART_RESOURCES USART##x##_Resources =  {         \
                                               &husart##x,                                          \
                                               USART##x,                                            \
                                               &USART##x##_Info,                                    \
                                               &USART##x##_TransferInfo,                            \
                                               USART##x##_DMA_USE,                                  \
                                               {0, 0, 0}                                            \
                                             }

#define USARTx_EXPORT_DRIVER(x)                                                                                                                                                           \
static int32_t                USART##x##_Initialize      (ARM_USART_SignalEvent_t cb_event)                  { return USART_Initialize (cb_event, &USART##x##_Resources); }               \
static int32_t                USART##x##_Uninitialize    (void)                                              { return USART_Uninitialize (&USART##x##_Resources); }                       \
static int32_t                USART##x##_PowerControl    (ARM_POWER_STATE state)                             { return USART_PowerControl (state, &USART##x##_Resources); }                \
static int32_t                USART##x##_Send            (const void *data, uint32_t num)                    { return USART_Send (data, num, &USART##x##_Resources); }                    \
static int32_t                USART##x##_Receive         (void *data, uint32_t num)                          { return USART_Receive (data, num, &USART##x##_Resources); }                 \
static int32_t                USART##x##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return USART_Transfer (data_out, data_in, num, &USART##x##_Resources); }   \
static uint32_t               USART##x##_GetGetTxCount   (void)                                              { return USART_GetTxCount (&USART##x##_Resources); }                         \
static uint32_t               USART##x##_GetGetRxCount   (void)                                              { return USART_GetRxCount (&USART##x##_Resources); }                         \
static int32_t                USART##x##_Control         (uint32_t control, uint32_t arg)                    { return USART_Control (control, arg, &USART##x##_Resources); }              \
static ARM_USART_STATUS       USART##x##_GetStatus       (void)                                              { return USART_GetStatus (&USART##x##_Resources); }                          \
                                                                                                                                                                                          \
ARM_DRIVER_USART Driver_USART##x = {    \
  USART_GetVersion,                     \
  USART_GetCapabilities,                \
  USART##x##_Initialize,                \
  USART##x##_Uninitialize,              \
  USART##x##_PowerControl,              \
  USART##x##_Send,                      \
  USART##x##_Receive,                   \
  USART##x##_Transfer,                  \
  USART##x##_GetGetTxCount,             \
  USART##x##_GetGetRxCount,             \
  USART##x##_Control,                   \
  USART##x##_GetStatus,                 \
  USART_SetModemControl,                \
  USART_GetModemStatus                  \
}

// DMA Use
#define USART_DMA_USE_TX           (1U << 0)
#define USART_DMA_USE_RX           (1U << 1)
#define USART_DMA_USE_TX_RX        (USART_DMA_USE_TX | USART_DMA_USE_RX)

// USART1 Configuration
#ifdef MX_USART1
#if   (MX_USART1_VM == VM_SYNC)

// USART1 used in Synchronous mode
#define USART1_MODE_SYNC           1

// USART1 DMA USE
#ifdef MX_USART1_TX_DMA_Instance
  #define USART1_DMA_USE_TX        USART_DMA_USE_TX
#else
  #define USART1_DMA_USE_TX        0
#endif
#ifdef MX_USART1_RX_DMA_Instance
  #define USART1_DMA_USE_RX        USART_DMA_USE_RX
#else
  #define USART1_DMA_USE_RX        0
#endif
#define USART1_DMA_USE            (USART1_DMA_USE_TX | USART1_DMA_USE_RX)
#endif
#endif

// USART2 Configuration
#ifdef MX_USART2
#if   (MX_USART2_VM == VM_SYNC)

// USART2 used in Synchronous mode
#define USART2_MODE_SYNC           1

// USART2 DMA USE
#ifdef MX_USART2_TX_DMA_Instance
  #define USART2_DMA_USE_TX        USART_DMA_USE_TX
#else
  #define USART2_DMA_USE_TX        0
#endif
#ifdef MX_USART2_RX_DMA_Instance
  #define USART2_DMA_USE_RX        USART_DMA_USE_RX
#else
  #define USART2_DMA_USE_RX        0
#endif
#define USART2_DMA_USE            (USART2_DMA_USE_TX | USART2_DMA_USE_RX)
#endif
#endif

// USART3 Configuration
#ifdef MX_USART3
#if   (MX_USART3_VM == VM_SYNC)

// USART3 used in Synchronous mode
#define USART3_MODE_SYNC           1

// USART3 DMA USE
#ifdef MX_USART3_TX_DMA_Instance
  #define USART3_DMA_USE_TX        USART_DMA_USE_TX
#else
  #define USART3_DMA_USE_TX        0
#endif
#ifdef MX_USART3_RX_DMA_Instance
  #define USART3_DMA_USE_RX        USART_DMA_USE_RX
#else
  #define USART3_DMA_USE_RX        0
#endif
#define USART3_DMA_USE            (USART3_DMA_USE_TX | USART3_DMA_USE_RX)
#endif
#endif

#if defined(USART1_MODE_SYNC) ||     \
    defined(USART2_MODE_SYNC) ||     \
    defined(USART3_MODE_SYNC)

#define USARTx_MODE_SYNC             1


// USART flags
#define USART_FLAG_INITIALIZED      ((uint8_t)(1U))
#define USART_FLAG_POWERED          ((uint8_t)(1U << 1))
#define USART_FLAG_CONFIGURED       ((uint8_t)(1U << 2))
#define USART_FLAG_TX_ENABLED       ((uint8_t)(1U << 3))
#define USART_FLAG_RX_ENABLED       ((uint8_t)(1U << 4))

// USART Transfer Information (Run-Time)
typedef struct {
  uint32_t              rx_num;         // Total number of receive data
  uint32_t              tx_num;         // Total number of transmit data
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint16_t              def_val;        // Default transfer value
  uint16_t              reserved;
} USART_TRANSFER_INFO;

typedef struct {
  uint8_t tx_busy;                      // Transmitter busy flag
  uint8_t rx_busy;                      // Receiver busy flag
  uint8_t tx_underflow;                 // Transmit data underflow detected (cleared on start of next send operation)
  uint8_t rx_overflow;                  // Receive data overflow detected (cleared on start of next receive operation)
  uint8_t rx_break;                     // Break detected on receive (cleared on start of next receive operation)
  uint8_t rx_framing_error;             // Framing error detected on receive (cleared on start of next receive operation)
  uint8_t rx_parity_error;              // Parity error detected on receive (cleared on start of next receive operation)
  uint8_t reserved;
} USART_STATUS;

// USART Information (Run-time)
typedef struct {
  ARM_USART_SignalEvent_t cb_event;     // Event Callback
  USART_STATUS            status;       // Status flags
  uint8_t                 flags;        // Current USART flags
  uint8_t                 reserved[3];
} USART_INFO;

// USART Resources definition
typedef const struct {
  USART_HandleTypeDef    *h;
  USART_TypeDef          *reg;          // USART peripheral pointer
  USART_INFO             *info;         // Run-Time Information
  USART_TRANSFER_INFO    *xfer;         // USART transfer information
  uint8_t                 dma_use;
  uint8_t                 reserved[3];
} USART_RESOURCES;

// Global functions and variables exported by driver .c module
#ifdef USART1_MODE_SYNC
extern ARM_DRIVER_USART Driver_USART1;
#endif

#ifdef USART2_MODE_SYNC
extern ARM_DRIVER_USART Driver_USART2;
#endif

#ifdef USART3_MODE_SYNC
extern ARM_DRIVER_USART Driver_USART3;
#endif

#endif
#endif /* __USART_STM32G4XX_H */
