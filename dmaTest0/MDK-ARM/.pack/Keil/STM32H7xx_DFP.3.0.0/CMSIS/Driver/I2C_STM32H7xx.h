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
 * $Date:        10. February 2022
 * $Revision:    V1.4
 *
 * Project:      I2C Driver definitions for ST STM32H7xx
 * -------------------------------------------------------------------------- */

#ifndef __I2C_STM32H7XX_H
#define __I2C_STM32H7XX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_I2C.h"
#include "stm32h7xx_hal.h"

#include "MX_Device.h"

#if ((defined(RTE_Drivers_I2C1) || \
      defined(RTE_Drivers_I2C2) || \
      defined(RTE_Drivers_I2C3) || \
      defined(RTE_Drivers_I2C4) || \
      defined(RTE_Drivers_I2C5))   \
      && (!defined (MX_I2C1))      \
      && (!defined (MX_I2C2))      \
      && (!defined (MX_I2C3))      \
      && (!defined (MX_I2C4))      \
      && (!defined (MX_I2C5)))
  #error "I2C not configured in STM32CubeMX!"
#endif

#if (defined(ARM_I2C_API_VERSION) && (ARM_I2C_API_VERSION >= 0x203U))
  /* This is used to add init values to padding bytes in API structures */
  #define I2C_API_PADDING_EN            (1)
#else
  #define I2C_API_PADDING_EN            (0)
#endif

#if defined(MX_I2C1)
#if (!defined(MX_I2C1_SCL_GPIO_Pu) && defined(MX_I2C1_SCL_GPIO_PuPd))
#define MX_I2C1_SCL_GPIO_Pu MX_I2C1_SCL_GPIO_PuPd
#endif
#if (!defined(MX_I2C1_SDA_GPIO_Pu) && defined(MX_I2C1_SDA_GPIO_PuPd))
#define MX_I2C1_SDA_GPIO_Pu MX_I2C1_SDA_GPIO_PuPd
#endif
#endif

#if defined(MX_I2C2)
#if (!defined(MX_I2C2_SCL_GPIO_Pu) && defined(MX_I2C2_SCL_GPIO_PuPd))
#define MX_I2C2_SCL_GPIO_Pu MX_I2C2_SCL_GPIO_PuPd
#endif
#if (!defined(MX_I2C2_SDA_GPIO_Pu) && defined(MX_I2C2_SDA_GPIO_PuPd))
#define MX_I2C2_SDA_GPIO_Pu MX_I2C2_SDA_GPIO_PuPd
#endif
#endif

#if defined(MX_I2C3)
#if (!defined(MX_I2C3_SCL_GPIO_Pu) && defined(MX_I2C3_SCL_GPIO_PuPd))
#define MX_I2C3_SCL_GPIO_Pu MX_I2C3_SCL_GPIO_PuPd
#endif
#if (!defined(MX_I2C3_SDA_GPIO_Pu) && defined(MX_I2C3_SDA_GPIO_PuPd))
#define MX_I2C3_SDA_GPIO_Pu MX_I2C3_SDA_GPIO_PuPd
#endif
#endif

#if defined(MX_I2C4)
#if (!defined(MX_I2C4_SCL_GPIO_Pu) && defined(MX_I2C4_SCL_GPIO_PuPd))
#define MX_I2C4_SCL_GPIO_Pu MX_I2C4_SCL_GPIO_PuPd
#endif
#if (!defined(MX_I2C4_SDA_GPIO_Pu) && defined(MX_I2C4_SDA_GPIO_PuPd))
#define MX_I2C4_SDA_GPIO_Pu MX_I2C4_SDA_GPIO_PuPd
#endif
#endif

#if defined(MX_I2C5)
#if (!defined(MX_I2C5_SCL_GPIO_Pu) && defined(MX_I2C5_SCL_GPIO_PuPd))
#define MX_I2C5_SCL_GPIO_Pu MX_I2C5_SCL_GPIO_PuPd
#endif
#if (!defined(MX_I2C5_SDA_GPIO_Pu) && defined(MX_I2C5_SDA_GPIO_PuPd))
#define MX_I2C5_SDA_GPIO_Pu MX_I2C5_SDA_GPIO_PuPd
#endif
#endif

/* Bus Clear clock period definition */
#define I2C_BUS_CLEAR_CLOCK_PERIOD      (2)   // I2C bus clock period (ms)

/* Current driver status flag definition */
#define I2C_INIT            ((uint8_t)0x01)   // I2C initialized
#define I2C_POWER           ((uint8_t)0x02)   // I2C powered on
#define I2C_SETUP           ((uint8_t)0x04)   // I2C configured
#define I2C_DMA_TX          ((uint8_t)0x08)   // Tx DMA enabled
#define I2C_DMA_RX          ((uint8_t)0x10)   // Rx DMA enabled
#define I2C_XFER_SET        ((uint8_t)0x20)   // Transfer buffer registered
#define I2C_XFER_FIRST      ((uint8_t)0x40)
#define I2C_XFER_NEXT       ((uint8_t)0x80)

/* Transfer status flags definitions */
#define XFER_CTRL_RESTART   ((uint8_t)0x01)   // Transfer pending
#define XFER_CTRL_STOP      ((uint8_t)0x02)   // Generate repeated start and readdress
#define XFER_CTRL_ADDR_NACK ((uint8_t)0x04)   // Slave address not acknowledged
#define XFER_CTRL_ADDR_DONE ((uint8_t)0x08)   // Addressing done

/* TIMING register limit values */
#define I2C_TIMINGR_PRESC_MAX    (1UL << 4)
#define I2C_TIMINGR_SCLDEL_MAX   (1UL << 4)
#define I2C_TIMINGR_SDADEL_MAX   (1UL << 4)
#define I2C_TIMINGR_SCLH_MAX     (1UL << 8)
#define I2C_TIMINGR_SCLL_MAX     (1UL << 8)

/* Pulse width of spikes suppressed by the analog filter */
#define I2C_ANALOG_FILTER_DELAY_MIN     50    // Min delay added by the analog filter (ns)
#define I2C_ANALOG_FILTER_DELAY_MAX    260    // Max delay added by the analog filter (ns)


/* I2C standard timing specification */
typedef const struct {
  uint32_t clk_max;                           // Max I2C speed (Hz)
  uint32_t tf_max;                            // Max SDA and SCL fall time (ns)
  uint32_t tr_max;                            // Max SDA and SCL rise time (ns)
  uint32_t hddat_min;                         // Min data hold time (ns)
  uint32_t vddat_max;                         // Max data valid time (ns)
  uint32_t sudat_min;                         // Min data setup time (ns)
  uint32_t scll_min;                          // Min SCL low period (ns)
  uint32_t sclh_min;                          // Min SCL high period (ns)
} I2C_STD_TIME;

/* I2C clock setup auxiliary structure */
typedef struct {
  uint16_t i2cclk;                            // Kernel clock period (ns)
  uint16_t busclk;                            // Bus clock period (ns)
  uint16_t dfd;                               // Digital filter delay (ns)
  uint16_t afd_min;                           // Analog filter min delay (ns)
  uint16_t afd_max;                           // Analog filter max delay (ns)
  uint16_t error;                             // Clock setup error (ns)
} I2C_CLK_SETUP;

/* I2C peripheral TIMING register values */
typedef struct {
  uint8_t presc;                              // Timing register value PRESC[3:0]
  uint8_t scldel;                             // Timing register value SCLDEL[3:0]
  uint8_t sdadel;                             // Timing register value SDADEL[3:0]
  uint8_t sclh;                               // Timing register value SCLH[7:0]
  uint8_t scll;                               // Timing register value SCLL[7:0]
} I2C_TIMING;

/* I2C Input/Output Configuration */
typedef const struct {
  GPIO_TypeDef         *scl_port;             // SCL IO Port
  GPIO_TypeDef         *sda_port;             // SDA IO Port
  uint16_t              scl_pin;              // SCL IO Pin
  uint16_t              sda_pin;              // SDA IO Pin
  uint32_t              scl_pull;             // SCL IO Pin Pull Definition
  uint32_t              sda_pull;             // SDA IO Pin Pull Definition
  uint32_t              scl_af;
  uint32_t              sda_af;
} I2C_IO;

/* I2C Information (Run-Time) */
typedef struct {
  ARM_I2C_SignalEvent_t cb_event;             // Event Callback
  ARM_I2C_STATUS        status;               // Status flags
  uint8_t volatile      flags;                // Current I2C state flags
  uint8_t volatile      abort;                // Abort completed flag
  uint8_t volatile      rx_dma;               // DMA receive pending flag
  uint8_t               reserved;
  uint8_t              *rx_data;              // Pointer to receive data buffer
  uint32_t              xfer_sz;              // Transfer size (bytes)
} I2C_INFO;

/* I2C Resources definition */
typedef struct {
  I2C_HandleTypeDef    *h;                    // I2C handle
  I2C_TypeDef          *reg;                  // I2C peripheral register interface
  I2C_IO                io;                   // I2C Input/Output pins
  IRQn_Type             ev_irq_num;           // I2C Event IRQ Number
  IRQn_Type             er_irq_num;           // I2C Error IRQ Number
  uint16_t              anf_enable;           // Analog noise filter enable/disable flag
  uint16_t              dnf_coef;             // Digital noise filter coefficient
  I2C_INFO             *info;                 // Run-Time information
} const I2C_RESOURCES;


/* Global functions and variables */
#ifdef MX_I2C1
extern void I2C1_EV_IRQHandler  (void);
extern void I2C1_ER_IRQHandler  (void);
extern void I2C1_RX_DMA_Handler (void);
extern void I2C1_TX_DMA_Handler (void);

#if defined(MX_I2C1_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c1_rx;
#endif
#if defined(MX_I2C1_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c1_tx;
#endif

extern ARM_DRIVER_I2C Driver_I2C1;
#endif

#ifdef MX_I2C2
extern void I2C2_EV_IRQHandler  (void);
extern void I2C2_ER_IRQHandler  (void);
extern void I2C2_RX_DMA_Handler (void);
extern void I2C2_TX_DMA_Handler (void);

#if defined(MX_I2C2_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c2_rx;
#endif
#if defined(MX_I2C2_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c2_tx;
#endif

extern ARM_DRIVER_I2C Driver_I2C2;
#endif

#ifdef MX_I2C3
extern void I2C3_EV_IRQHandler  (void);
extern void I2C3_ER_IRQHandler  (void);
extern void I2C3_RX_DMA_Handler (void);
extern void I2C3_TX_DMA_Handler (void);

#if defined(MX_I2C3_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c3_rx;
#endif
#if defined(MX_I2C3_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c3_tx;
#endif

extern ARM_DRIVER_I2C Driver_I2C3;
#endif

#ifdef MX_I2C4
extern void I2C4_EV_IRQHandler  (void);
extern void I2C4_ER_IRQHandler  (void);
extern void I2C4_RX_DMA_Handler (void);
extern void I2C4_TX_DMA_Handler (void);

#if defined(MX_I2C4_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c4_rx;
#endif
#if defined(MX_I2C4_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c4_tx;
#endif

extern ARM_DRIVER_I2C Driver_I2C4;
#endif

#ifdef MX_I2C5
extern void I2C5_EV_IRQHandler  (void);
extern void I2C5_ER_IRQHandler  (void);
extern void I2C5_RX_DMA_Handler (void);
extern void I2C5_TX_DMA_Handler (void);

#if defined(MX_I2C5_RX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c5_rx;
#endif
#if defined(MX_I2C5_TX_DMA_Instance)
extern DMA_HandleTypeDef hdma_i2c5_tx;
#endif

extern ARM_DRIVER_I2C Driver_I2C5;
#endif

#endif /* __I2C_STM32H7XX_H */
