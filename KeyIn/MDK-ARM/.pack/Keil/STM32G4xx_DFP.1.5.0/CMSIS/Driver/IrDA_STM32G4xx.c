/* -----------------------------------------------------------------------------
 * Copyright (c) 2021-2022 Arm Limited (or its affiliates).
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
 * $Date:        11. May 2022
 * $Revision:    V1.0
 *
 * Driver:       Driver_USART1, Driver_USART2, Driver_USART3
 *               Driver_USART4, Driver_USART5
 *
 * Configured:   via CubeMX
 * Project:      USART Driver for ST STM32G4xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value   IrDA Interface
 *   ---------------------                   -----   --------------
 *   Connect to hardware via Driver_USART# = 1       use USART1
 *   Connect to hardware via Driver_USART# = 2       use USART2
 *   Connect to hardware via Driver_USART# = 3       use USART3
 *   Connect to hardware via Driver_USART# = 4       use UART4
 *   Connect to hardware via Driver_USART# = 5       use UART5
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0
 *    Initial release
 */

/*! \page stm32g4_usart_irda CMSIS-Driver USART in IrDA mode

\section stm32g4_usart_irda_limitations Limitations

STM32 HAL limitations:
 - Rx Timeout event can be detected only when send/receive/transfer operation is active
 - Manual control of modem lines is not supported
 - Break character is handled as framing error

\section stm32g4_usart_irda_setup STM32CubeMX Setup

The CMSIS-Driver USART requires:
  - Setup of USART/UART peripheral in IrDA mode

Valid settings for various evaluation boards are listed in the table below:

Peripheral Resource | NUCLEO-G474RE
:-------------------|:-------------
USART Peripheral    | <b>USART1</b>
USART Mode          | IrDA
TX Pin              | PC4
RX Pin              | PC5

For different boards, refer to the hardware schematics to reflect correct setup values.

The STM32CubeMX configuration for NUCLEO-G474RE with steps for Pinout, Clock, and System Configuration are
listed below. Enter the values that are marked \b bold.

Pinout view
-----------
  1. Configure USART1 pins
    - PC4: USART1_TX
    - PC5: USART1_RX

Pinout & Configuration tab
--------------------------
  1. Under <em>Connectivity</em> open \b USART1:
    - Section <em>Mode</em>
      - Mode: <b>IrDA</b>
    - Section <em>Configuration</em>
      - Parameter Settings: not used
      - User Constants: not used
      - NVIC Settings: configured later
      - DMA Settings: configured with DMA
      - GPIO Settings: check used pins

  2. Under <em>System Core</em> open \b GPIO:
    - <b>Group By Peripherals</b> - \b USART:
         Pin Name | Signal on Pin | GPIO mode    | GPIO Pull-up/Pull..| Maximum out..| User Label
         :--------|:--------------|:-------------|:-------------------|:-------------|:----------
         PC4      | USART1_TX     | Alternate F..| No pull-up and no..| Very High    |.
         PC5      | USART1_RX     | Alternate F..| No pull-up and no..| Very High    |.

  3. Under <em>System Core</em> open \b DMA:
    - Section <em>Configuration</em>
      - <b>DMA1, DMA2</b>
        - Click on \b Add button and add requests as in table below:
          DMA Request | Channel        | Direction            | Priority
          :-----------|:---------------|:---------------------|:--------
          USART1_RX   | DMA1 Channel 1 | Peripheral to Memory | Low
          USART1_TX   | DMA1 Channel 2 | Memory to Peripheral | Low

  4. Under <em>System Core</em> open \b NVIC:
    - Section <em>Configuration</em>
      - <b>NVIC</b>
           NVIC Interrupt Table                 | Enabled | Preemption Priority | Sub Priority
           :------------------------------------|:--------|:--------------------|:------------
           DMA1 channel1 global interrupt       |\b ON    | 0                   | 0
           DMA1 channel2 global interrupt       |\b ON    | 0                   | 0
           USART1 global interrupt              |\b ON    | 0                   | 0

Clock Configuration tab
-----------------------
  1. Configure USART1 Clock.

Project Manager tab
-------------------
  1. Open <b>Advanced Settings</b>:
     - enable <b>Do Not Generate Function Call</b> for MX_USART1_IRDA_Init.

\note
The <b>CMSIS Driver:USART (API):USART</b> component in the <b>Manage Run-Time Environment</b> dialog adds multiple C source
files to the project. The interface selection in STM32CubeMX selects the actual implementation that is compiled: IrDA,
SmartCard, UART, or USART.
*/
/*! \cond */

#include "IrDA_STM32G4xx.h"
#ifdef USARTx_MODE_IRDA

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };

static const ARM_USART_CAPABILITIES Capabilities = {
    0,      ///< supports UART (Asynchronous) mode
    0,      ///< supports Synchronous Master mode
    0,      ///< supports Synchronous Slave mode
    0,      ///< supports UART Single-wire mode
    1,      ///< supports UART IrDA mode
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

// IrDA1
#ifdef USART1_MODE_IRDA
IRDAx_RESOURCE_ALLOC(1);
#endif

// IrDA2
#ifdef USART2_MODE_IRDA
IRDAx_RESOURCE_ALLOC(2);
#endif

// IrDA3
#ifdef USART3_MODE_IRDA
IRDAx_RESOURCE_ALLOC(3);
#endif

// IrDA4
#ifdef USART4_MODE_IRDA
IRDAx_RESOURCE_ALLOC(4);
#endif

// IrDA5
#ifdef USART5_MODE_IRDA
IRDAx_RESOURCE_ALLOC(5);
#endif


/**
  \fn          const IRDA_RESOURCES IRDA_Resources (IRDA_HandleTypeDef *hirda)
  \brief       Get UART_RESOURCES structure from IRDA_HandleTypeDef
*/
static const IRDA_RESOURCES * IRDA_Resources (IRDA_HandleTypeDef *hirda) {
  const IRDA_RESOURCES *irda = NULL;

#ifdef USART1_MODE_IRDA
  if (hirda->Instance == USART1) { irda = &IRDA1_Resources; }
#endif
#ifdef USART2_MODE_IRDA
  if (hirda->Instance == USART2) { irda = &IRDA2_Resources; }
#endif
#ifdef USART3_MODE_IRDA
  if (hirda->Instance == USART3) { irda = &IRDA3_Resources; }
#endif
#ifdef USART4_MODE_IRDA
  if (hirda->Instance == UART4)  { irda = &IRDA4_Resources; }
#endif
#ifdef USART5_MODE_IRDA
  if (hirda->Instance == UART5)  { irda = &IRDA5_Resources; }
#endif

  return irda;
}

/**
  \fn          void IRDA_PeripheralReset (USART_TypeDef *usart)
  \brief       IRDA Reset
*/
static void IRDA_PeripheralReset (USART_TypeDef *usart) {

#ifdef USART1_MODE_IRDA
  if (usart == USART1) { __HAL_RCC_USART1_FORCE_RESET(); }
#endif
#ifdef USART2_MODE_IRDA
  if (usart == USART2) { __HAL_RCC_USART2_FORCE_RESET(); }
#endif
#ifdef USART3_MODE_IRDA
  if (usart == USART3) { __HAL_RCC_USART3_FORCE_RESET(); }
#endif
#ifdef USART4_MODE_IRDA
  if (usart == UART4)  { __HAL_RCC_UART4_FORCE_RESET(); }
#endif
#ifdef USART5_MODE_IRDA
  if (usart == UART5)  { __HAL_RCC_UART5_FORCE_RESET(); }
#endif

      __NOP(); __NOP(); __NOP(); __NOP();

#ifdef USART1_MODE_IRDA
  if (usart == USART1) { __HAL_RCC_USART1_RELEASE_RESET(); }
#endif
#ifdef USART2_MODE_IRDA
  if (usart == USART2) { __HAL_RCC_USART2_RELEASE_RESET(); }
#endif
#ifdef USART3_MODE_IRDA
  if (usart == USART3) { __HAL_RCC_USART3_RELEASE_RESET(); }
#endif
#ifdef USART4_MODE_IRDA
  if (usart == UART4)  { __HAL_RCC_UART4_RELEASE_RESET(); }
#endif
#ifdef USART5_MODE_IRDA
  if (usart == UART5)  { __HAL_RCC_UART5_RELEASE_RESET(); }
#endif
}

// USART Driver functions

/**
  \fn          ARM_DRIVER_VERSION IRDA_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION IRDA_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES IRDA_GetCapabilities (void)
  \brief       Get driver capabilities
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES IRDA_GetCapabilities (void) {
  return Capabilities;
}

/**
  \fn          int32_t IRDA_Initialize (      ARM_USART_SignalEvent_t  cb_event
                                         const IRDA_RESOURCES          *irda)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   irda      Pointer to IRDA resources
  \return      \ref execution_status
*/
static int32_t IRDA_Initialize (      ARM_USART_SignalEvent_t  cb_event,
                                 const IRDA_RESOURCES          *irda) {

  if (irda->info->flags & IRDA_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  irda->h->Init.Mode = 0;

  // Initialize IRDA Run-time Resources
  irda->info->cb_event = cb_event;

  // Clear Status flags
  irda->info->status.tx_busy          = 0;
  irda->info->status.rx_busy          = 0;
  irda->info->status.tx_underflow     = 0;
  irda->info->status.rx_overflow      = 0;
  irda->info->status.rx_break         = 0;
  irda->info->status.rx_framing_error = 0;
  irda->info->status.rx_parity_error  = 0;

  // Clear transfer information
  memset(irda->xfer, 0, sizeof(IRDA_TRANSFER_INFO));

  irda->h->Instance = irda->reg;

  irda->info->flags = IRDA_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IRDA_Uninitialize (const IRDA_RESOURCES *irda)
  \brief       De-initialize IRDA Interface.
  \param[in]   usart     Pointer to IRDA resources
  \return      \ref execution_status
*/
static int32_t IRDA_Uninitialize (const IRDA_RESOURCES *irda) {

  // Reset IRDA status flags
  irda->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IRDA_PowerControl (ARM_POWER_STATE state)
  \brief       Control IRDA Interface Power.
  \param[in]   state  Power state
  \param[in]   usart  Pointer to IRDA resources
  \return      \ref execution_status
*/
static int32_t IRDA_PowerControl (      ARM_POWER_STATE  state,
                                   const IRDA_RESOURCES  *irda) {

  if ((irda->info->flags & IRDA_FLAG_INITIALIZED) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:

      // IrDA peripheral reset
      IRDA_PeripheralReset (irda->reg);

      if (irda->h->Instance != NULL) {
        HAL_IRDA_MspDeInit (irda->h);
      }

      // Clear Status flags
      irda->info->status.tx_busy          = 0;
      irda->info->status.rx_busy          = 0;
      irda->info->status.tx_underflow     = 0;
      irda->info->status.rx_overflow      = 0;
      irda->info->status.rx_break         = 0;
      irda->info->status.rx_framing_error = 0;
      irda->info->status.rx_parity_error  = 0;

      irda->info->flags &= ~IRDA_FLAG_POWERED;
      break;
    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((irda->info->flags & IRDA_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((irda->info->flags & IRDA_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      irda->xfer->def_val = 0U;

      // Clear Status flags
      irda->info->status.tx_busy          = 0;
      irda->info->status.rx_busy          = 0;
      irda->info->status.tx_underflow     = 0;
      irda->info->status.rx_overflow      = 0;
      irda->info->status.rx_break         = 0;
      irda->info->status.rx_framing_error = 0;
      irda->info->status.rx_parity_error  = 0;

      irda->info->flags = IRDA_FLAG_POWERED | IRDA_FLAG_INITIALIZED;

      HAL_IRDA_MspInit (irda->h);

      // IrDA peripheral reset
      IRDA_PeripheralReset (irda->reg);

    break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IRDA_Send (const void            *data,
                                        uint32_t         num,
                                  const IRDA_RESOURCES *irda)
  \brief       Start sending data to IRDA transmitter.
  \param[in]   data  Pointer to buffer with data to send to IRDA transmitter
  \param[in]   num   Number of data items to send
  \param[in]   usart Pointer to IRDA resources
  \return      \ref execution_status
*/
static int32_t IRDA_Send (const void            *data,
                                uint32_t         num,
                          const IRDA_RESOURCES  *irda) {

  HAL_StatusTypeDef stat;

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((irda->info->flags & IRDA_FLAG_CONFIGURED) == 0U) {
    // IrDA is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  switch (HAL_IRDA_GetState (irda->h)) {
    case HAL_IRDA_STATE_RESET:
    case HAL_IRDA_STATE_ERROR:
      return ARM_DRIVER_ERROR;

    case HAL_IRDA_STATE_TIMEOUT:
      return ARM_DRIVER_ERROR_TIMEOUT;

    case HAL_IRDA_STATE_BUSY:
    case HAL_IRDA_STATE_BUSY_TX:
    case HAL_IRDA_STATE_BUSY_TX_RX:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_IRDA_STATE_BUSY_RX:
    case HAL_IRDA_STATE_READY:
      break;
  }

  // Clear ARM UART STATUS flags
  irda->info->status.tx_underflow = 0;

  // Save buffer info
  irda->xfer->tx_num = num;
  irda->xfer->tx_cnt = 0U;

  if (irda->dma_use_tx != 0U) {
    // DMA mode
    stat = HAL_IRDA_Transmit_DMA (irda->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  } else {
    // Interrupt mode
    stat = HAL_IRDA_Transmit_IT (irda->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
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
  \fn          int32_t IRDA_Receive (      void            *data,
                                           uint32_t         num,
                                     const IRDA_RESOURCES  *irda)
  \brief       Start receiving data from IRDA receiver.
  \param[out]  data  Pointer to buffer for data to receive from IRDA receiver
  \param[in]   num   Number of data items to receive
  \param[in]   usart Pointer to IRDA resources
  \return      \ref execution_status
*/
static int32_t IRDA_Receive (      void           *data,
                                   uint32_t        num,
                             const IRDA_RESOURCES *irda) {

  HAL_StatusTypeDef stat;

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((irda->info->flags & IRDA_FLAG_CONFIGURED) == 0U) {
    // IrDA is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  switch (HAL_IRDA_GetState (irda->h)) {
    case HAL_IRDA_STATE_RESET:
    case HAL_IRDA_STATE_ERROR:
      return ARM_DRIVER_ERROR;

    case HAL_IRDA_STATE_TIMEOUT:
      return ARM_DRIVER_ERROR_TIMEOUT;

    case HAL_IRDA_STATE_BUSY:
    case HAL_IRDA_STATE_BUSY_RX:
    case HAL_IRDA_STATE_BUSY_TX_RX:
      return ARM_DRIVER_ERROR_BUSY;

    case HAL_IRDA_STATE_BUSY_TX:
    case HAL_IRDA_STATE_READY:
      break;
  }

  // Clear ARM UART STATUS flags
  irda->info->status.rx_overflow = 0;
  irda->info->status.rx_break = 0;
  irda->info->status.rx_framing_error = 0;
  irda->info->status.rx_parity_error = 0;

  // Save buffer info
  irda->xfer->rx_num = num;
  irda->xfer->rx_cnt = 0U;

  if (irda->dma_use_rx != 0U) {
    // DMA mode
    stat = HAL_IRDA_Receive_DMA (irda->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
  } else {
    // Interrupt mode
    stat = HAL_IRDA_Receive_IT (irda->h, (uint8_t *)(uint32_t)data, (uint16_t)num);
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
  \fn          int32_t IRDA_Transfer (const void             *data_out,
                                             void            *data_in,
                                             uint32_t         num,
                                       const IRDA_RESOURCES  *irda)
  \brief       Start sending/receiving data to/from IRDA transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to IRDA transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from IRDA receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   irda      Pointer to IRDA resources
  \return      \ref execution_status
*/
static int32_t IRDA_Transfer (const void             *data_out,
                                    void             *data_in,
                                    uint32_t          num,
                              const IRDA_RESOURCES   *irda) {

  UNUSED(data_out);
  UNUSED(data_in);
  UNUSED(num);
  UNUSED(irda);

  // Supported only in Synchronous mode
  return ARM_DRIVER_ERROR;
}

/**
  \fn          uint32_t IRDA_GetTxCount (const IRDA_RESOURCES *irda)
  \brief       Get transmitted data count.
  \param[in]   irda     Pointer to IRDA resources
  \return      number of data items transmitted
*/
static uint32_t IRDA_GetTxCount (const IRDA_RESOURCES *irda) {
  uint32_t cnt;

  if ((irda->info->flags & IRDA_FLAG_POWERED) == 0U) {
    return 0U;
  }

  if (irda->dma_use_tx != 0U) {
    cnt = irda->xfer->tx_num - __HAL_DMA_GET_COUNTER(irda->h->hdmatx);
  } else {
    cnt = irda->h->TxXferSize - irda->h->TxXferCount;
  }

  return cnt;
}

/**
  \fn          uint32_t IRDA_GetRxCount (const IRDA_RESOURCES *irda)
  \brief       Get received data count.
  \param[in]   irda     Pointer to IRDA resources
  \return      number of data items received
*/
static uint32_t IRDA_GetRxCount (const IRDA_RESOURCES *irda) {
  uint32_t cnt;

  if ((irda->info->flags & IRDA_FLAG_POWERED) == 0U) {
    return 0U;
  }

  if (irda->dma_use_rx != 0U) {
    cnt = irda->xfer->rx_num - __HAL_DMA_GET_COUNTER(irda->h->hdmarx);
  } else {
    cnt = irda->h->RxXferSize - irda->h->RxXferCount;
  }

  return cnt;
}

/**
  \fn          int32_t IRDA_Control (      uint32_t          control,
                                           uint32_t          arg,
                                      const IRDA_RESOURCES  *irda)
  \brief       Control IRDA Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   irda    Pointer to IRDA resources
  \return      common \ref execution_status and driver specific \ref irda_execution_status
*/
static int32_t IRDA_Control (      uint32_t         control,
                                   uint32_t         arg,
                             const IRDA_RESOURCES  *irda) {

  HAL_StatusTypeDef status;

  if ((irda->info->flags & IRDA_FLAG_POWERED) == 0U) {
    // IrDA not powered
    return ARM_DRIVER_ERROR;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
     // Control break
    case ARM_USART_CONTROL_BREAK:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    // Abort
    case ARM_USART_ABORT_SEND:
      HAL_IRDA_AbortTransmit(irda->h);
      irda->h->TxXferSize = 0U;
      return ARM_DRIVER_OK;
    case ARM_USART_ABORT_RECEIVE:
      HAL_IRDA_AbortReceive(irda->h);
      irda->h->RxXferSize = 0U;
      return ARM_DRIVER_OK;
    case ARM_USART_ABORT_TRANSFER:
      HAL_IRDA_Abort(irda->h);
      irda->h->RxXferSize = 0U;
      irda->h->TxXferSize = 0U;
      return ARM_DRIVER_OK;

    // Control TX
    case ARM_USART_CONTROL_TX:
      if (arg) {
        // Transmitter enable
        irda->h->Init.Mode |=  IRDA_MODE_TX;
      } else {
        // Transmitter disable
        irda->h->Init.Mode &= ~IRDA_MODE_TX;
      }
      status = HAL_IRDA_Init(irda->h);
      return IRDA_HAL_STATUS(status);

    // Control RX
    case ARM_USART_CONTROL_RX:
      if (arg) {
        // Receiver enable
        irda->h->Init.Mode |= IRDA_MODE_RX;
      } else {
        // Receiver disable
        irda->h->Init.Mode &= ~IRDA_MODE_RX;
      }
      status = HAL_IRDA_Init(irda->h);
      return IRDA_HAL_STATUS(status);
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
      irda->xfer->def_val = (uint16_t)arg;
      return ARM_DRIVER_OK;

    case ARM_USART_SET_IRDA_PULSE:
      if (arg == 0) {
        return ARM_DRIVER_OK;
      } else {
        return ARM_DRIVER_ERROR;
      }

    default: { return ARM_DRIVER_ERROR_UNSUPPORTED; }
  }

  // IrDA Data bits
  switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_6:
      if ((control & ARM_USART_PARITY_Msk) != ARM_USART_PARITY_NONE) {
        irda->h->Init.WordLength = IRDA_WORDLENGTH_7B;
      } else {
        return ARM_USART_ERROR_DATA_BITS;
      }
      break;
    case ARM_USART_DATA_BITS_7:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        irda->h->Init.WordLength = IRDA_WORDLENGTH_7B;
      } else {
        irda->h->Init.WordLength = IRDA_WORDLENGTH_8B;
      }
      break;
    case ARM_USART_DATA_BITS_8:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        irda->h->Init.WordLength = IRDA_WORDLENGTH_8B;
      } else {
        irda->h->Init.WordLength = IRDA_WORDLENGTH_9B;
      }
      break;
    case ARM_USART_DATA_BITS_9:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        irda->h->Init.WordLength = IRDA_WORDLENGTH_9B;
      } else {
        return ARM_USART_ERROR_DATA_BITS;
      }
      break;
    default: return ARM_USART_ERROR_DATA_BITS;
  }

  // IrDA Parity
  switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE:
      irda->h->Init.Parity = IRDA_PARITY_NONE;
      break;
    case ARM_USART_PARITY_EVEN:
      irda->h->Init.Parity = IRDA_PARITY_EVEN;
      break;
    case ARM_USART_PARITY_ODD:
      irda->h->Init.Parity = IRDA_PARITY_ODD;
      break;
    default: return ARM_USART_ERROR_PARITY;
  }

  // IrDA Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:
      break;
    case ARM_USART_STOP_BITS_2:
    case ARM_USART_STOP_BITS_1_5:
    case ARM_USART_STOP_BITS_0_5:
    default: return ARM_USART_ERROR_STOP_BITS;
  }

  // IrDA Flow control
  switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
      break;
    case ARM_USART_FLOW_CONTROL_RTS:
    case ARM_USART_FLOW_CONTROL_CTS:
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
    default: return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // IrDA Baudrate
  irda->h->Init.BaudRate = arg;

  // Set configured flag
  irda->info->flags |= IRDA_FLAG_CONFIGURED;

  // Initialize IrDA
  status = HAL_IRDA_Init(irda->h);

  // Reconfigure DMA
  if ((irda->dma_use_tx != 0U) && (irda->h->hdmatx != NULL)) {
    if ((control & ARM_USART_DATA_BITS_Msk) > ARM_USART_DATA_BITS_8) {
      irda->h->hdmatx->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      irda->h->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    } else {
      irda->h->hdmatx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      irda->h->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    }
    __HAL_DMA_DISABLE(irda->h->hdmatx);
    HAL_DMA_Init(irda->h->hdmatx);
    __HAL_DMA_ENABLE(irda->h->hdmatx);
  }

  if ((irda->dma_use_rx != 0U) && (irda->h->hdmarx != NULL)) {
    if ((control & ARM_USART_DATA_BITS_Msk) > ARM_USART_DATA_BITS_8) {
      irda->h->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      irda->h->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    } else {
      irda->h->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      irda->h->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    }
    __HAL_DMA_DISABLE(irda->h->hdmarx);
    HAL_DMA_Init(irda->h->hdmarx);
    __HAL_DMA_ENABLE(irda->h->hdmarx);
  }

  return IRDA_HAL_STATUS(status);
}

/**
  \fn          ARM_USART_STATUS IRDA_GetStatus (const IRDA_RESOURCES *irda)
  \brief       Get IRDA status.
  \param[in]   irda     Pointer to IRDA resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS IRDA_GetStatus (const IRDA_RESOURCES *irda) {
  ARM_USART_STATUS  status = {0};

  switch (HAL_IRDA_GetState (irda->h)) {
    case HAL_IRDA_STATE_BUSY:
    case HAL_IRDA_STATE_BUSY_TX_RX:
      status.rx_busy = 1;
      status.tx_busy = 1;
      break;
    case HAL_IRDA_STATE_BUSY_TX:
      status.tx_busy = 1;
      break;
    case HAL_IRDA_STATE_BUSY_RX:
      status.rx_busy = 1;
      break;

    case HAL_IRDA_STATE_TIMEOUT:
    case HAL_IRDA_STATE_READY:
    case HAL_IRDA_STATE_RESET:
    case HAL_IRDA_STATE_ERROR:
      break;
  }

  status.tx_underflow     = irda->info->status.tx_underflow;
  status.rx_overflow      = irda->info->status.rx_overflow;
  status.rx_break         = irda->info->status.rx_break;
  status.rx_framing_error = irda->info->status.rx_framing_error;
  status.rx_parity_error  = irda->info->status.rx_parity_error;

  return status;
}

/**
  \fn          int32_t IRDA_SetModemControl (ARM_USART_MODEM_CONTROL control)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t IRDA_SetModemControl (ARM_USART_MODEM_CONTROL control) {

  (void) control;

  // No modem control in synchronous mode
  return ARM_DRIVER_ERROR;
}

/**
  \fn          ARM_USART_MODEM_STATUS IRDA_GetModemStatus (void)
  \brief       Get UART Modem Status lines state.
  \return      modem status \ref ARM_UART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS IRDA_GetModemStatus (void) {
  ARM_USART_MODEM_STATUS modem_status;

  modem_status.cts = 0U;
  modem_status.dsr = 0U;
  modem_status.ri  = 0U;
  modem_status.dcd = 0U;

  return modem_status;
}

/**
  * @brief Tx Transfer completed callback.
  * @param hirda: IRDA handle.
  * @retval None
  */
void HAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda) {
  const IRDA_RESOURCES * irda;

  irda = IRDA_Resources (hirda);
  irda->xfer->tx_cnt = irda->xfer->tx_num;

  if (irda->info->cb_event != NULL) {
    irda->info->cb_event(ARM_USART_EVENT_TX_COMPLETE | ARM_USART_EVENT_SEND_COMPLETE);
  }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param hirda: IRDA handle.
  * @retval None
  */
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda) {
  const IRDA_RESOURCES * irda;

  irda = IRDA_Resources (hirda);
  irda->xfer->rx_cnt = irda->xfer->rx_num;

  if (irda->info->cb_event != NULL) {
    irda->info->cb_event(ARM_USART_EVENT_RECEIVE_COMPLETE);
  }
}

/**
  * @brief IRDA error callback.
  * @param hirda: IRDA handle.
  * @retval None
  */
void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda) {
  const IRDA_RESOURCES * irda;
        uint32_t         error;
        uint32_t         event;

  irda = IRDA_Resources (hirda);

  error = HAL_IRDA_GetError (irda->h);
  event = 0;

  if (error & HAL_IRDA_ERROR_PE) {
    event |= ARM_USART_EVENT_RX_PARITY_ERROR;
    irda->info->status.rx_parity_error = 1;
  }
  if (error & HAL_IRDA_ERROR_FE) {
    event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
    irda->info->status.rx_framing_error = 1;
  }
  if (error & HAL_IRDA_ERROR_ORE) {
    event |= ARM_USART_EVENT_RX_OVERFLOW;
    irda->info->status.rx_overflow = 1;
  }

  if ((event != 0) && (irda->info->cb_event != NULL)) {
    irda->info->cb_event(event);
  }
}

#ifdef USART1_MODE_IRDA
IRDAx_EXPORT_DRIVER(1);
#endif

#ifdef USART2_MODE_IRDA
IRDAx_EXPORT_DRIVER(2);
#endif

#ifdef USART3_MODE_IRDA
IRDAx_EXPORT_DRIVER(3);
#endif

#ifdef USART4_MODE_IRDA
IRDAx_EXPORT_DRIVER(4);
#endif

#ifdef USART5_MODE_IRDA
IRDAx_EXPORT_DRIVER(5);
#endif

#endif

/*! \endcond */
