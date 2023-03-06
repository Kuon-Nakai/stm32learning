/* -----------------------------------------------------------------------------
 * Copyright (c) 2020 Arm Limited (or its affiliates).
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
 * $Date:        13. October 2020
 * $Revision:    V1.0
 * Project:      SPI Driver definitions for ST STM32G4xx
 * -------------------------------------------------------------------------- */

#ifndef __SPI_STM32G4XX_H
#define __SPI_STM32G4XX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_SPI.h"
#include "stm32g4xx_hal.h"

#include "RTE_Components.h"
#include "MX_Device.h"


#define SPIx_PIN_STRUCT(x, pin)      static const SPI_PIN  SPI##x##_##pin = {MX_SPI##x##_##pin##_GPIOx, MX_SPI##x##_##pin##_GPIO_Pin, MX_SPI##x##_##pin##_GPIO_AF}
#define SPIx_PIN_NSS_STRUCT_ALLOC(x) SPIx_PIN_STRUCT(x, NSS)

#define SPIx_PIN_STRUCT_PTR(x, pin)  SPI##x##_PIN_##pin##_STRUCT_PTR
#define SPIx_PIN_NSS_STRUCT_PTR(x)   SPIx_PIN_STRUCT_PTR(x, NSS)

#define SPIx_RESOURCE_ALLOC(x)       extern SPI_HandleTypeDef hspi##x;                        \
                                     static SPI_INFO          SPI##x##_Info;                  \
                                     static SPI_TRANSFER_INFO SPI##x##_TransferInfo;          \
                                     static const SPI_RESOURCES SPI##x##_Resources =  {       \
                                               &hspi##x,                                      \
                                               SPI##x,                                        \
                                               SPI##x##_GET_CLK,                              \
                                               SPIx_PIN_NSS_STRUCT_PTR(x),                    \
                                               &SPI##x##_Info,                                \
                                               &SPI##x##_TransferInfo,                        \
                                                SPI##x##_DMA_USE                              \
                                     }

#define SPIx_EXPORT_DRIVER(x)                                                                                                                                               \
static int32_t        SPI##x##_Initialize      (ARM_SPI_SignalEvent_t pSignalEvent)                { return SPI_Initialize (pSignalEvent, &SPI##x##_Resources); }           \
static int32_t        SPI##x##_Uninitialize    (void)                                              { return SPI_Uninitialize (&SPI##x##_Resources); }                       \
static int32_t        SPI##x##_PowerControl    (ARM_POWER_STATE state)                             { return SPI_PowerControl (state, &SPI##x##_Resources); }                \
static int32_t        SPI##x##_Send            (const void *data, uint32_t num)                    { return SPI_Send (data, num, &SPI##x##_Resources); }                    \
static int32_t        SPI##x##_Receive         (void *data, uint32_t num)                          { return SPI_Receive (data, num, &SPI##x##_Resources); }                 \
static int32_t        SPI##x##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return SPI_Transfer (data_out, data_in, num, &SPI##x##_Resources); }   \
static uint32_t       SPI##x##_GetDataCount    (void)                                              { return SPI_GetDataCount (&SPI##x##_Resources); }                       \
static int32_t        SPI##x##_Control         (uint32_t control, uint32_t arg)                    { return SPI_Control (control, arg, &SPI##x##_Resources); }              \
static ARM_SPI_STATUS SPI##x##_GetStatus       (void)                                              { return SPI_GetStatus (&SPI##x##_Resources); }                          \
                                                                                                                                                                            \
ARM_DRIVER_SPI Driver_SPI##x = {                                                                                                                                            \
  SPIX_GetVersion,                                                                                                                                                          \
  SPIX_GetCapabilities,                                                                                                                                                     \
  SPI##x##_Initialize,                                                                                                                                                      \
  SPI##x##_Uninitialize,                                                                                                                                                    \
  SPI##x##_PowerControl,                                                                                                                                                    \
  SPI##x##_Send,                                                                                                                                                            \
  SPI##x##_Receive,                                                                                                                                                         \
  SPI##x##_Transfer,                                                                                                                                                        \
  SPI##x##_GetDataCount,                                                                                                                                                    \
  SPI##x##_Control,                                                                                                                                                         \
  SPI##x##_GetStatus                                                                                                                                                        \
}

// DMA Use
#define SPI_DMA_USE_TX             (1U << 0)
#define SPI_DMA_USE_RX             (1U << 1)
#define SPI_DMA_USE_TX_RX          (SPI_DMA_USE_TX | SPI_DMA_USE_RX)

// Current driver status flag definition
#define SPI_INITIALIZED            ((uint8_t)(1U))          // SPI initialized
#define SPI_POWERED                ((uint8_t)(1U << 1))     // SPI powered on
#define SPI_CONFIGURED             ((uint8_t)(1U << 2))     // SPI configured
#define SPI_DATA_LOST              ((uint8_t)(1U << 3))     // SPI data lost occurred
#define SPI_MODE_FAULT             ((uint8_t)(1U << 4))     // SPI mode fault occurred

// SPI1 Configuration
#ifdef MX_SPI1
// SPI1 DMA USE
#ifdef MX_SPI1_TX_DMA_Instance
  #define SPI1_DMA_USE_TX          SPI_DMA_USE_TX
#else
  #define SPI1_DMA_USE_TX          0
#endif
#ifdef MX_SPI1_RX_DMA_Instance
  #define SPI1_DMA_USE_RX          SPI_DMA_USE_RX
#else
  #define SPI1_DMA_USE_RX          0
#endif
#define SPI1_DMA_USE               (SPI1_DMA_USE_TX | SPI1_DMA_USE_RX)

// SPI GET CLOCK Function pointer
#define SPI1_GET_CLK               HAL_RCC_GetPCLK2Freq

// SPI1 NSS pin
#ifdef  MX_SPI1_NSS_Pin
  #define SPI1_PIN_NSS_STRUCT_PTR  &SPI1_NSS
#else
  #define SPI1_PIN_NSS_STRUCT_PTR  NULL
#endif
#endif

//SPI2 Configuration
#ifdef MX_SPI2
// SPI2 DMA USE
#ifdef MX_SPI2_TX_DMA_Instance
  #define SPI2_DMA_USE_TX          SPI_DMA_USE_TX
#else
  #define SPI2_DMA_USE_TX          0
#endif
#ifdef MX_SPI2_RX_DMA_Instance
  #define SPI2_DMA_USE_RX          SPI_DMA_USE_RX
#else
  #define SPI2_DMA_USE_RX          0
#endif
#define SPI2_DMA_USE               (SPI2_DMA_USE_TX | SPI2_DMA_USE_RX)

// SPI2 GET CLOCK Function pointer
#define SPI2_GET_CLK               HAL_RCC_GetPCLK1Freq

// SPI2 NSS pin
#ifdef  MX_SPI2_NSS_Pin
  #define SPI2_PIN_NSS_STRUCT_PTR  &SPI2_NSS
#else
  #define SPI2_PIN_NSS_STRUCT_PTR  NULL
#endif
#endif

// SPI3 Configuration
#ifdef MX_SPI3
// SPI3 DMA USE
#ifdef MX_SPI3_TX_DMA_Instance
  #define SPI3_DMA_USE_TX          SPI_DMA_USE_TX
#else
  #define SPI3_DMA_USE_TX          0
#endif
#ifdef MX_SPI3_RX_DMA_Instance
  #define SPI3_DMA_USE_RX          SPI_DMA_USE_RX
#else
  #define SPI3_DMA_USE_RX          0
#endif
#define SPI3_DMA_USE               (SPI3_DMA_USE_TX | SPI3_DMA_USE_RX)

// SPI3 GET CLOCK Function pointer
#define SPI3_GET_CLK               HAL_RCC_GetPCLK1Freq

// SPI3 NSS pin
#ifdef  MX_SPI3_NSS_Pin
  #define SPI3_PIN_NSS_STRUCT_PTR  &SPI3_NSS
#else
  #define SPI3_PIN_NSS_STRUCT_PTR  NULL
#endif
#endif

//SPI4 Configuration
#ifdef MX_SPI4
// SPI4 DMA USE
#ifdef MX_SPI4_TX_DMA_Instance
  #define SPI4_DMA_USE_TX          SPI_DMA_USE_TX
#else
  #define SPI4_DMA_USE_TX          0
#endif
#ifdef MX_SPI4_RX_DMA_Instance
  #define SPI4_DMA_USE_RX          SPI_DMA_USE_RX
#else
  #define SPI4_DMA_USE_RX          0
#endif
#define SPI4_DMA_USE               (SPI4_DMA_USE_TX | SPI4_DMA_USE_RX)

// SPI4 GET CLOCK Function pointer
#define SPI4_GET_CLK               HAL_RCC_GetPCLK1Freq

// SPI4 NSS pin
#ifdef  MX_SPI4_NSS_Pin
  #define SPI4_PIN_NSS_STRUCT_PTR  &SPI4_NSS
#else
  #define SPI4_PIN_NSS_STRUCT_PTR  NULL
#endif
#endif

#if ((defined(MX_SPI1) && defined(MX_SPI1_RX_DMA_Instance)) || \
     (defined(MX_SPI2) && defined(MX_SPI2_RX_DMA_Instance)) || \
     (defined(MX_SPI3) && defined(MX_SPI3_RX_DMA_Instance)) || \
     (defined(MX_SPI4) && defined(MX_SPI4_RX_DMA_Instance)))
#define __SPI_DMA_RX
#endif
#if ((defined(MX_SPI1) && defined(MX_SPI1_TX_DMA_Instance)) || \
     (defined(MX_SPI2) && defined(MX_SPI2_TX_DMA_Instance)) || \
     (defined(MX_SPI3) && defined(MX_SPI3_TX_DMA_Instance)) || \
     (defined(MX_SPI4) && defined(MX_SPI4_TX_DMA_Instance)))
#define __SPI_DMA_TX
#endif
#if (defined(__SPI_DMA_RX) && defined(__SPI_DMA_TX))
#define __SPI_DMA
#endif

// SPI pin
typedef const struct _SPI_PIN {
  GPIO_TypeDef         *port;           // Port
  uint32_t              pin;            // Pin
  uint32_t              af;             // Alternate function
} SPI_PIN;

// SPI Information (Run-time)
typedef struct _SPI_INFO {
  ARM_SPI_SignalEvent_t cb_event;       // Event Callback
  uint32_t              mode;           // Current SPI mode
  uint8_t               state;          // Current SPI state
  uint8_t               reserved[3];
} SPI_INFO;

// SPI Transfer Information (Run-Time)
typedef struct _SPI_TRANSFER_INFO {
  uint16_t              def_val;        // Default transfer value
  uint16_t              reserved;
} SPI_TRANSFER_INFO;


// SPI Resources definition
typedef struct {
  SPI_HandleTypeDef    *h;                    // SPI Handle
  SPI_TypeDef          *reg;                  // SPI peripheral pointer
  uint32_t            (*periph_clock)(void);  // Peripheral bus clock
  SPI_PIN              *nss;                  // SPI NSS pin
  SPI_INFO             *info;                 // Run-Time Information
  SPI_TRANSFER_INFO    *xfer;                 // SPI transfer information
  uint32_t              dma_use;              // DMA use: bit0 - DMA_TX, bit1 -DMA_RX
} SPI_RESOURCES;

// Global functions and variables exported by driver .c module
#ifdef MX_SPI1
extern ARM_DRIVER_SPI Driver_SPI1;
#endif

#ifdef MX_SPI2
extern ARM_DRIVER_SPI Driver_SPI2;
#endif

#ifdef MX_SPI3
extern ARM_DRIVER_SPI Driver_SPI3;
#endif

#ifdef MX_SPI4
extern ARM_DRIVER_SPI Driver_SPI4;
#endif

#endif /* __SPI_STM32G4XX_H */
