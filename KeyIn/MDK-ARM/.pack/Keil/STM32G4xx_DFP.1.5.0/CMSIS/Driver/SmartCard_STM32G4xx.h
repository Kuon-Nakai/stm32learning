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
 * Project:      SMARTCARD Driver definitions for ST STM32G4xx
 * -------------------------------------------------------------------------- */

#ifndef __SMARTCARD_STM32G4XX_H
#define __SMARTCARD_STM32G4XX_H

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

#define SMARTCARD_HAL_STATUS(stat)     ((stat == HAL_OK)      ? ARM_DRIVER_OK :            \
                                       ((stat == HAL_BUSY)    ? ARM_DRIVER_ERROR_BUSY :    \
                                       ((stat == HAL_TIMEOUT) ? ARM_DRIVER_ERROR_TIMEOUT : \
                                                                  ARM_DRIVER_ERROR)))



#define SMARTCARDx_RESOURCE_ALLOC(x)   extern SMARTCARD_HandleTypeDef    hsmartcard##x;                        \
                                       static SMARTCARD_INFO             SMARTCARD##x##_Info;                  \
                                       static SMARTCARD_TRANSFER_INFO    SMARTCARD##x##_TransferInfo;          \
                                       static const SMARTCARD_RESOURCES  SMARTCARD##x##_Resources =  {         \
                                                   &hsmartcard##x,                                             \
                                                   SMARTCARD##x##_PERIPHERAL,                                  \
                                                   &SMARTCARD##x##_Info,                                       \
                                                   &SMARTCARD##x##_TransferInfo,                               \
                                                   SMARTCARD##x##_DMA_USE_TX,                                  \
                                                   SMARTCARD##x##_DMA_USE_RX,                                  \
                                                   0                                                           \
                                                 }

#define SMARTCARDx_EXPORT_DRIVER(x)                                                                                                                                                          \
static int32_t                USART##x##_Initialize      (ARM_USART_SignalEvent_t cb_event)                  { return SMARTCARD_Initialize (cb_event, &SMARTCARD##x##_Resources); }               \
static int32_t                USART##x##_Uninitialize    (void)                                              { return SMARTCARD_Uninitialize (&SMARTCARD##x##_Resources); }                       \
static int32_t                USART##x##_PowerControl    (ARM_POWER_STATE state)                             { return SMARTCARD_PowerControl (state, &SMARTCARD##x##_Resources); }                \
static int32_t                USART##x##_Send            (const void *data, uint32_t num)                    { return SMARTCARD_Send (data, num, &SMARTCARD##x##_Resources); }                    \
static int32_t                USART##x##_Receive         (void *data, uint32_t num)                          { return SMARTCARD_Receive (data, num, &SMARTCARD##x##_Resources); }                 \
static int32_t                USART##x##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return SMARTCARD_Transfer (data_out, data_in, num, &SMARTCARD##x##_Resources); }   \
static uint32_t               USART##x##_GetGetTxCount   (void)                                              { return SMARTCARD_GetTxCount (&SMARTCARD##x##_Resources); }                         \
static uint32_t               USART##x##_GetGetRxCount   (void)                                              { return SMARTCARD_GetRxCount (&SMARTCARD##x##_Resources); }                         \
static int32_t                USART##x##_Control         (uint32_t control, uint32_t arg)                    { return SMARTCARD_Control (control, arg, &SMARTCARD##x##_Resources); }              \
static ARM_USART_STATUS       USART##x##_GetStatus       (void)                                              { return SMARTCARD_GetStatus (&SMARTCARD##x##_Resources); }                          \
                                                                                                                                                                                                  \
ARM_DRIVER_USART Driver_USART##x = {    \
  SMARTCARD_GetVersion,                 \
  SMARTCARD_GetCapabilities,            \
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
  SMARTCARD_SetModemControl,            \
  SMARTCARD_GetModemStatus              \
}

// DMA Use
#define SMARTCARD_DMA_USE_TX           (1U << 0)
#define SMARTCARD_DMA_USE_RX           (1U << 1)
#define SMARTCARD_DMA_USE_TX_RX        (USART_DMA_USE_TX | USART_DMA_USE_RX)

// SMARTCARD1 Configuration
#ifdef MX_USART1
#if   (MX_USART1_VM == VM_SMARTCARD)

// Peripheral: USART1
#define SMARTCARD1_PERIPHERAL          USART1

// USART1 used in SmartCard mode
#define USART1_MODE_SMARTCARD          1

// SMARTCARD1 DMA USE
#ifdef MX_USART1_TX_DMA_Instance
  #define SMARTCARD1_DMA_USE_TX        SMARTCARD_DMA_USE_TX
#else
  #define SMARTCARD1_DMA_USE_TX        0
#endif
#ifdef MX_USART1_RX_DMA_Instance
  #define SMARTCARD1_DMA_USE_RX        SMARTCARD_DMA_USE_RX
#else
  #define SMARTCARD1_DMA_USE_RX        0
#endif
#define SMARTCARD1_DMA_USE            (SMARTCARD1_DMA_USE_TX | SMARTCARD1_DMA_USE_RX)
#endif
#endif

// SMARTCARD2 Configuration
#ifdef MX_USART2
#if   (MX_USART2_VM == VM_SMARTCARD)

// Peripheral: USART2
#define SMARTCARD2_PERIPHERAL          USART2

// USART2 used in SmartCard mode
#define USART2_MODE_SMARTCARD          1

// SMARTCARD2 DMA USE
#ifdef MX_USART2_TX_DMA_Instance
  #define SMARTCARD2_DMA_USE_TX        SMARTCARD_DMA_USE_TX
#else
  #define SMARTCARD2_DMA_USE_TX        0
#endif
#ifdef MX_USART2_RX_DMA_Instance
  #define SMARTCARD2_DMA_USE_RX        SMARTCARD_DMA_USE_RX
#else
  #define SMARTCARD2_DMA_USE_RX        0
#endif
#define SMARTCARD2_DMA_USE            (SMARTCARD2_DMA_USE_TX | SMARTCARD2_DMA_USE_RX)
#endif
#endif

// SMARTCARD3 Configuration
#ifdef MX_USART3
#if   (MX_USART3_VM == VM_SMARTCARD)

// Peripheral: USART3
#define SMARTCARD3_PERIPHERAL          USART3

// USART3 used in SmartCard mode
#define USART3_MODE_SMARTCARD          1

// SMARTCARD3 DMA USE
#ifdef MX_USART3_TX_DMA_Instance
  #define SMARTCARD3_DMA_USE_TX        SMARTCARD_DMA_USE_TX
#else
  #define SMARTCARD3_DMA_USE_TX        0
#endif
#ifdef MX_USART3_RX_DMA_Instance
  #define SMARTCARD3_DMA_USE_RX        SMARTCARD_DMA_USE_RX
#else
  #define SMARTCARD3_DMA_USE_RX        0
#endif
#define SMARTCARD3_DMA_USE            (SMARTCARD3_DMA_USE_TX | SMARTCARD3_DMA_USE_RX)
#endif
#endif

#if defined(USART1_MODE_SMARTCARD) ||     \
    defined(USART2_MODE_SMARTCARD) ||     \
    defined(USART3_MODE_SMARTCARD)

#define USARTx_MODE_SMARTCARD            1

// SMARTCARD flags
#define SMARTCARD_FLAG_INITIALIZED      ((uint8_t)(1U))
#define SMARTCARD_FLAG_POWERED          ((uint8_t)(1U << 1))
#define SMARTCARD_FLAG_CONFIGURED       ((uint8_t)(1U << 2))
#define SMARTCARD_FLAG_TX_ENABLED       ((uint8_t)(1U << 3))
#define SMARTCARD_FLAG_RX_ENABLED       ((uint8_t)(1U << 4))

// SMARTCARD Transfer Information (Run-Time)
typedef struct {
  uint32_t              rx_num;         // Total number of receive data
  uint32_t              tx_num;         // Total number of transmit data
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint16_t              def_val;        // Default transfer value
  uint16_t              reserved;
} SMARTCARD_TRANSFER_INFO;

typedef struct {
  uint8_t tx_busy;                      // Transmitter busy flag
  uint8_t rx_busy;                      // Receiver busy flag
  uint8_t tx_underflow;                 // Transmit data underflow detected (cleared on start of next send operation)
  uint8_t rx_overflow;                  // Receive data overflow detected (cleared on start of next receive operation)
  uint8_t rx_break;                     // Break detected on receive (cleared on start of next receive operation)
  uint8_t rx_framing_error;             // Framing error detected on receive (cleared on start of next receive operation)
  uint8_t rx_parity_error;              // Parity error detected on receive (cleared on start of next receive operation)
  uint8_t reserved;
} SMARTCARD_STATUS;

// SMARTCARD Information (Run-time)
typedef struct {
  ARM_USART_SignalEvent_t   cb_event;   // Event Callback
  SMARTCARD_STATUS          status;     // Status flags
  uint8_t                   flags;      // Current USART flags
  uint8_t                   reserved[3];
} SMARTCARD_INFO;

// SMARTCARD Resources definition
typedef const struct {
  SMARTCARD_HandleTypeDef    *h;
  void                       *reg;          // SMARTCARD peripheral pointer
  SMARTCARD_INFO             *info;         // Run-Time Information
  SMARTCARD_TRANSFER_INFO    *xfer;         // SMARTCARD transfer information
  uint8_t                    dma_use_tx;
  uint8_t                    dma_use_rx;
  uint16_t                   reserved;
} SMARTCARD_RESOURCES;

// Global functions and variables exported by driver .c module
#ifdef USART1_MODE_SMARTCARD
extern ARM_DRIVER_USART Driver_USART1;
#endif

#ifdef USART2_MODE_SMARTCARD
extern ARM_DRIVER_USART Driver_USART2;
#endif

#ifdef USART3_MODE_SMARTCARD
extern ARM_DRIVER_USART Driver_USART3;
#endif

#endif
#endif /* __SMARTCARD_STM32G4XX_H */
