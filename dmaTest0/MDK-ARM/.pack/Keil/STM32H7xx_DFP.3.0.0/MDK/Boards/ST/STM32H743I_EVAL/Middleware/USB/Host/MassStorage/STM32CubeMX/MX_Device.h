/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 22/02/2022 14:37:12
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            64000000
#define MX_HSE_VALUE                            25000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLDSIFreq_Value                     500000000
#define MX_SYSCLKFreq_VALUE                     400000000
#define MX_HCLKFreq_Value                       200000000
#define MX_CortexFreq_Value                     400000000
#define MX_APB1Freq_Value                       100000000
#define MX_APB2Freq_Value                       100000000
#define MX_CECFreq_Value                        32000
#define MX_RTCFreq_Value                        32000
#define MX_USBFreq_Value                        48000000
#define MX_WatchDogFreq_Value                   32000
#define MX_DSIFreq_Value                        96000000
#define MX_DSIPHYCLKFreq_Value                  96000000
#define MX_DSITXEscFreq_Value                   20000000
#define MX_SPDIFRXFreq_Value                    200000000
#define MX_MCO1PinFreq_Value                    64000000
#define MX_MCO2PinFreq_Value                    400000000

/*-------------------------------- ADC1       --------------------------------*/

#define MX_ADC1                                 1

/* GPIO Configuration */

/* Pin PA0_C */
#define MX_ADCx_INP0_SYSCFG_AnalogSwitch        SYSCFG_SWITCH_PA0
#define MX_ADCx_INP0_Pin                        PA0_C
#define MX_ADCx_INP0_GPIOx                      GPIOA
#define MX_ADCx_INP0_GPIO_PuPd                  GPIO_NOPULL
#define MX_ADCx_INP0_GPIO_Pin                   GPIO_PIN_0
#define MX_ADCx_INP0_GPIO_Mode                  GPIO_MODE_ANALOG

/*-------------------------------- CORTEX_M7  --------------------------------*/

#define MX_CORTEX_M7                            1

/* GPIO Configuration */

/*-------------------------------- FMC        --------------------------------*/

#define MX_FMC                                  1

/* GPIO Configuration */

/* Pin PE11 */
#define MX_FMC_D8_DA8_Pin                       PE11
#define MX_FMC_D8_DA8_GPIOx                     GPIOE
#define MX_FMC_D8_DA8_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D8_DA8_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D8_DA8_GPIO_Pin                  GPIO_PIN_11
#define MX_FMC_D8_DA8_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D8_DA8_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PH7 */
#define MX_FMC_SDCKE1_Pin                       PH7
#define MX_FMC_SDCKE1_GPIOx                     GPIOH
#define MX_FMC_SDCKE1_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_SDCKE1_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_SDCKE1_GPIO_Pin                  GPIO_PIN_7
#define MX_FMC_SDCKE1_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_SDCKE1_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PF0 */
#define MX_FMC_A0_Pin                           PF0
#define MX_FMC_A0_GPIOx                         GPIOF
#define MX_FMC_A0_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A0_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A0_GPIO_Pin                      GPIO_PIN_0
#define MX_FMC_A0_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A0_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PF1 */
#define MX_FMC_A1_Pin                           PF1
#define MX_FMC_A1_GPIOx                         GPIOF
#define MX_FMC_A1_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A1_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A1_GPIO_Pin                      GPIO_PIN_1
#define MX_FMC_A1_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A1_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PF2 */
#define MX_FMC_A2_Pin                           PF2
#define MX_FMC_A2_GPIOx                         GPIOF
#define MX_FMC_A2_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A2_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A2_GPIO_Pin                      GPIO_PIN_2
#define MX_FMC_A2_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A2_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PF11 */
#define MX_FMC_SDNRAS_Pin                       PF11
#define MX_FMC_SDNRAS_GPIOx                     GPIOF
#define MX_FMC_SDNRAS_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_SDNRAS_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_SDNRAS_GPIO_Pin                  GPIO_PIN_11
#define MX_FMC_SDNRAS_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_SDNRAS_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PF3 */
#define MX_FMC_A3_Pin                           PF3
#define MX_FMC_A3_GPIOx                         GPIOF
#define MX_FMC_A3_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A3_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A3_GPIO_Pin                      GPIO_PIN_3
#define MX_FMC_A3_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A3_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PF4 */
#define MX_FMC_A4_Pin                           PF4
#define MX_FMC_A4_GPIOx                         GPIOF
#define MX_FMC_A4_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A4_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A4_GPIO_Pin                      GPIO_PIN_4
#define MX_FMC_A4_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A4_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PF5 */
#define MX_FMC_A5_Pin                           PF5
#define MX_FMC_A5_GPIOx                         GPIOF
#define MX_FMC_A5_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A5_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A5_GPIO_Pin                      GPIO_PIN_5
#define MX_FMC_A5_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A5_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PH5 */
#define MX_FMC_SDNWE_Pin                        PH5
#define MX_FMC_SDNWE_GPIOx                      GPIOH
#define MX_FMC_SDNWE_GPIO_PuPd                  GPIO_NOPULL
#define MX_FMC_SDNWE_GPIO_Speed_High_Default    GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_SDNWE_GPIO_Pin                   GPIO_PIN_5
#define MX_FMC_SDNWE_GPIO_AF                    GPIO_AF12_FMC
#define MX_FMC_SDNWE_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PF12 */
#define MX_FMC_A6_Pin                           PF12
#define MX_FMC_A6_GPIOx                         GPIOF
#define MX_FMC_A6_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A6_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A6_GPIO_Pin                      GPIO_PIN_12
#define MX_FMC_A6_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A6_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PG0 */
#define MX_FMC_A10_Pin                          PG0
#define MX_FMC_A10_GPIOx                        GPIOG
#define MX_FMC_A10_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_A10_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A10_GPIO_Pin                     GPIO_PIN_0
#define MX_FMC_A10_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_A10_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PF13 */
#define MX_FMC_A7_Pin                           PF13
#define MX_FMC_A7_GPIOx                         GPIOF
#define MX_FMC_A7_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A7_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A7_GPIO_Pin                      GPIO_PIN_13
#define MX_FMC_A7_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A7_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PI10 */
#define MX_FMC_D31_Pin                          PI10
#define MX_FMC_D31_GPIOx                        GPIOI
#define MX_FMC_D31_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D31_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D31_GPIO_Pin                     GPIO_PIN_10
#define MX_FMC_D31_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D31_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PF14 */
#define MX_FMC_A8_Pin                           PF14
#define MX_FMC_A8_GPIOx                         GPIOF
#define MX_FMC_A8_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A8_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A8_GPIO_Pin                      GPIO_PIN_14
#define MX_FMC_A8_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A8_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PI9 */
#define MX_FMC_D30_Pin                          PI9
#define MX_FMC_D30_GPIOx                        GPIOI
#define MX_FMC_D30_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D30_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D30_GPIO_Pin                     GPIO_PIN_9
#define MX_FMC_D30_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D30_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PF15 */
#define MX_FMC_A9_Pin                           PF15
#define MX_FMC_A9_GPIOx                         GPIOF
#define MX_FMC_A9_GPIO_PuPd                     GPIO_NOPULL
#define MX_FMC_A9_GPIO_Speed_High_Default       GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A9_GPIO_Pin                      GPIO_PIN_15
#define MX_FMC_A9_GPIO_AF                       GPIO_AF12_FMC
#define MX_FMC_A9_GPIO_Mode                     GPIO_MODE_AF_PP

/* Pin PG1 */
#define MX_FMC_A11_Pin                          PG1
#define MX_FMC_A11_GPIOx                        GPIOG
#define MX_FMC_A11_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_A11_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A11_GPIO_Pin                     GPIO_PIN_1
#define MX_FMC_A11_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_A11_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PE14 */
#define MX_FMC_D11_DA11_Pin                     PE14
#define MX_FMC_D11_DA11_GPIOx                   GPIOE
#define MX_FMC_D11_DA11_GPIO_PuPd               GPIO_NOPULL
#define MX_FMC_D11_DA11_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D11_DA11_GPIO_Pin                GPIO_PIN_14
#define MX_FMC_D11_DA11_GPIO_AF                 GPIO_AF12_FMC
#define MX_FMC_D11_DA11_GPIO_Mode               GPIO_MODE_AF_PP

/* Pin PI2 */
#define MX_FMC_D26_Pin                          PI2
#define MX_FMC_D26_GPIOx                        GPIOI
#define MX_FMC_D26_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D26_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D26_GPIO_Pin                     GPIO_PIN_2
#define MX_FMC_D26_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D26_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PI1 */
#define MX_FMC_D25_Pin                          PI1
#define MX_FMC_D25_GPIOx                        GPIOI
#define MX_FMC_D25_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D25_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D25_GPIO_Pin                     GPIO_PIN_1
#define MX_FMC_D25_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D25_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PI6 */
#define MX_FMC_D28_Pin                          PI6
#define MX_FMC_D28_GPIOx                        GPIOI
#define MX_FMC_D28_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D28_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D28_GPIO_Pin                     GPIO_PIN_6
#define MX_FMC_D28_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D28_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PI3 */
#define MX_FMC_D27_Pin                          PI3
#define MX_FMC_D27_GPIOx                        GPIOI
#define MX_FMC_D27_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D27_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D27_GPIO_Pin                     GPIO_PIN_3
#define MX_FMC_D27_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D27_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PH14 */
#define MX_FMC_D22_Pin                          PH14
#define MX_FMC_D22_GPIOx                        GPIOH
#define MX_FMC_D22_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D22_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D22_GPIO_Pin                     GPIO_PIN_14
#define MX_FMC_D22_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D22_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PE10 */
#define MX_FMC_D7_DA7_Pin                       PE10
#define MX_FMC_D7_DA7_GPIOx                     GPIOE
#define MX_FMC_D7_DA7_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D7_DA7_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D7_DA7_GPIO_Pin                  GPIO_PIN_10
#define MX_FMC_D7_DA7_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D7_DA7_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PH13 */
#define MX_FMC_D21_Pin                          PH13
#define MX_FMC_D21_GPIOx                        GPIOH
#define MX_FMC_D21_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D21_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D21_GPIO_Pin                     GPIO_PIN_13
#define MX_FMC_D21_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D21_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PI0 */
#define MX_FMC_D24_Pin                          PI0
#define MX_FMC_D24_GPIOx                        GPIOI
#define MX_FMC_D24_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D24_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D24_GPIO_Pin                     GPIO_PIN_0
#define MX_FMC_D24_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D24_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PH15 */
#define MX_FMC_D23_Pin                          PH15
#define MX_FMC_D23_GPIOx                        GPIOH
#define MX_FMC_D23_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D23_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D23_GPIO_Pin                     GPIO_PIN_15
#define MX_FMC_D23_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D23_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PI4 */
#define MX_FMC_NBL2_Pin                         PI4
#define MX_FMC_NBL2_GPIOx                       GPIOI
#define MX_FMC_NBL2_GPIO_PuPd                   GPIO_NOPULL
#define MX_FMC_NBL2_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_NBL2_GPIO_Pin                    GPIO_PIN_4
#define MX_FMC_NBL2_GPIO_AF                     GPIO_AF12_FMC
#define MX_FMC_NBL2_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PE1 */
#define MX_FMC_NBL1_Pin                         PE1
#define MX_FMC_NBL1_GPIOx                       GPIOE
#define MX_FMC_NBL1_GPIO_PuPd                   GPIO_NOPULL
#define MX_FMC_NBL1_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_NBL1_GPIO_Pin                    GPIO_PIN_1
#define MX_FMC_NBL1_GPIO_AF                     GPIO_AF12_FMC
#define MX_FMC_NBL1_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PI5 */
#define MX_FMC_NBL3_Pin                         PI5
#define MX_FMC_NBL3_GPIOx                       GPIOI
#define MX_FMC_NBL3_GPIO_PuPd                   GPIO_NOPULL
#define MX_FMC_NBL3_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_NBL3_GPIO_Pin                    GPIO_PIN_5
#define MX_FMC_NBL3_GPIO_AF                     GPIO_AF12_FMC
#define MX_FMC_NBL3_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PI7 */
#define MX_FMC_D29_Pin                          PI7
#define MX_FMC_D29_GPIOx                        GPIOI
#define MX_FMC_D29_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D29_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D29_GPIO_Pin                     GPIO_PIN_7
#define MX_FMC_D29_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D29_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PE0 */
#define MX_FMC_NBL0_Pin                         PE0
#define MX_FMC_NBL0_GPIOx                       GPIOE
#define MX_FMC_NBL0_GPIO_PuPd                   GPIO_NOPULL
#define MX_FMC_NBL0_GPIO_Speed_High_Default     GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_NBL0_GPIO_Pin                    GPIO_PIN_0
#define MX_FMC_NBL0_GPIO_AF                     GPIO_AF12_FMC
#define MX_FMC_NBL0_GPIO_Mode                   GPIO_MODE_AF_PP

/* Pin PD14 */
#define MX_FMC_D0_DA0_Pin                       PD14
#define MX_FMC_D0_DA0_GPIOx                     GPIOD
#define MX_FMC_D0_DA0_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D0_DA0_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D0_DA0_GPIO_Pin                  GPIO_PIN_14
#define MX_FMC_D0_DA0_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D0_DA0_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PE13 */
#define MX_FMC_D10_DA10_Pin                     PE13
#define MX_FMC_D10_DA10_GPIOx                   GPIOE
#define MX_FMC_D10_DA10_GPIO_PuPd               GPIO_NOPULL
#define MX_FMC_D10_DA10_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D10_DA10_GPIO_Pin                GPIO_PIN_13
#define MX_FMC_D10_DA10_GPIO_AF                 GPIO_AF12_FMC
#define MX_FMC_D10_DA10_GPIO_Mode               GPIO_MODE_AF_PP

/* Pin PD8 */
#define MX_FMC_D13_DA13_Pin                     PD8
#define MX_FMC_D13_DA13_GPIOx                   GPIOD
#define MX_FMC_D13_DA13_GPIO_PuPd               GPIO_NOPULL
#define MX_FMC_D13_DA13_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D13_DA13_GPIO_Pin                GPIO_PIN_8
#define MX_FMC_D13_DA13_GPIO_AF                 GPIO_AF12_FMC
#define MX_FMC_D13_DA13_GPIO_Mode               GPIO_MODE_AF_PP

/* Pin PH12 */
#define MX_FMC_D20_Pin                          PH12
#define MX_FMC_D20_GPIOx                        GPIOH
#define MX_FMC_D20_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D20_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D20_GPIO_Pin                     GPIO_PIN_12
#define MX_FMC_D20_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D20_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PH9 */
#define MX_FMC_D17_Pin                          PH9
#define MX_FMC_D17_GPIOx                        GPIOH
#define MX_FMC_D17_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D17_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D17_GPIO_Pin                     GPIO_PIN_9
#define MX_FMC_D17_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D17_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PH8 */
#define MX_FMC_D16_Pin                          PH8
#define MX_FMC_D16_GPIOx                        GPIOH
#define MX_FMC_D16_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D16_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D16_GPIO_Pin                     GPIO_PIN_8
#define MX_FMC_D16_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D16_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PD15 */
#define MX_FMC_D1_DA1_Pin                       PD15
#define MX_FMC_D1_DA1_GPIOx                     GPIOD
#define MX_FMC_D1_DA1_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D1_DA1_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D1_DA1_GPIO_Pin                  GPIO_PIN_15
#define MX_FMC_D1_DA1_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D1_DA1_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PG15 */
#define MX_FMC_SDNCAS_Pin                       PG15
#define MX_FMC_SDNCAS_GPIOx                     GPIOG
#define MX_FMC_SDNCAS_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_SDNCAS_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_SDNCAS_GPIO_Pin                  GPIO_PIN_15
#define MX_FMC_SDNCAS_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_SDNCAS_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PE12 */
#define MX_FMC_D9_DA9_Pin                       PE12
#define MX_FMC_D9_DA9_GPIOx                     GPIOE
#define MX_FMC_D9_DA9_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D9_DA9_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D9_DA9_GPIO_Pin                  GPIO_PIN_12
#define MX_FMC_D9_DA9_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D9_DA9_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PD0 */
#define MX_FMC_D2_DA2_Pin                       PD0
#define MX_FMC_D2_DA2_GPIOx                     GPIOD
#define MX_FMC_D2_DA2_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D2_DA2_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D2_DA2_GPIO_Pin                  GPIO_PIN_0
#define MX_FMC_D2_DA2_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D2_DA2_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PH11 */
#define MX_FMC_D19_Pin                          PH11
#define MX_FMC_D19_GPIOx                        GPIOH
#define MX_FMC_D19_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D19_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D19_GPIO_Pin                     GPIO_PIN_11
#define MX_FMC_D19_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D19_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PH10 */
#define MX_FMC_D18_Pin                          PH10
#define MX_FMC_D18_GPIOx                        GPIOH
#define MX_FMC_D18_GPIO_PuPd                    GPIO_NOPULL
#define MX_FMC_D18_GPIO_Speed_High_Default      GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D18_GPIO_Pin                     GPIO_PIN_10
#define MX_FMC_D18_GPIO_AF                      GPIO_AF12_FMC
#define MX_FMC_D18_GPIO_Mode                    GPIO_MODE_AF_PP

/* Pin PD1 */
#define MX_FMC_D3_DA3_Pin                       PD1
#define MX_FMC_D3_DA3_GPIOx                     GPIOD
#define MX_FMC_D3_DA3_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D3_DA3_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D3_DA3_GPIO_Pin                  GPIO_PIN_1
#define MX_FMC_D3_DA3_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D3_DA3_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PH6 */
#define MX_FMC_SDNE1_Pin                        PH6
#define MX_FMC_SDNE1_GPIOx                      GPIOH
#define MX_FMC_SDNE1_GPIO_PuPd                  GPIO_NOPULL
#define MX_FMC_SDNE1_GPIO_Speed_High_Default    GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_SDNE1_GPIO_Pin                   GPIO_PIN_6
#define MX_FMC_SDNE1_GPIO_AF                    GPIO_AF12_FMC
#define MX_FMC_SDNE1_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PD9 */
#define MX_FMC_D14_DA14_Pin                     PD9
#define MX_FMC_D14_DA14_GPIOx                   GPIOD
#define MX_FMC_D14_DA14_GPIO_PuPd               GPIO_NOPULL
#define MX_FMC_D14_DA14_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D14_DA14_GPIO_Pin                GPIO_PIN_9
#define MX_FMC_D14_DA14_GPIO_AF                 GPIO_AF12_FMC
#define MX_FMC_D14_DA14_GPIO_Mode               GPIO_MODE_AF_PP

/* Pin PE7 */
#define MX_FMC_D4_DA4_Pin                       PE7
#define MX_FMC_D4_DA4_GPIOx                     GPIOE
#define MX_FMC_D4_DA4_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D4_DA4_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D4_DA4_GPIO_Pin                  GPIO_PIN_7
#define MX_FMC_D4_DA4_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D4_DA4_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PE8 */
#define MX_FMC_D5_DA5_Pin                       PE8
#define MX_FMC_D5_DA5_GPIOx                     GPIOE
#define MX_FMC_D5_DA5_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D5_DA5_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D5_DA5_GPIO_Pin                  GPIO_PIN_8
#define MX_FMC_D5_DA5_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D5_DA5_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PG4 */
#define MX_FMC_A14_BA0_Pin                      PG4
#define MX_FMC_A14_BA0_GPIOx                    GPIOG
#define MX_FMC_A14_BA0_GPIO_PuPd                GPIO_NOPULL
#define MX_FMC_A14_BA0_GPIO_Speed_High_Default  GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_A14_BA0_GPIO_Pin                 GPIO_PIN_4
#define MX_FMC_A14_BA0_GPIO_AF                  GPIO_AF12_FMC
#define MX_FMC_A14_BA0_GPIO_Mode                GPIO_MODE_AF_PP

/* Pin PG8 */
#define MX_FMC_SDCLK_Pin                        PG8
#define MX_FMC_SDCLK_GPIOx                      GPIOG
#define MX_FMC_SDCLK_GPIO_PuPd                  GPIO_NOPULL
#define MX_FMC_SDCLK_GPIO_Speed_High_Default    GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_SDCLK_GPIO_Pin                   GPIO_PIN_8
#define MX_FMC_SDCLK_GPIO_AF                    GPIO_AF12_FMC
#define MX_FMC_SDCLK_GPIO_Mode                  GPIO_MODE_AF_PP

/* Pin PE15 */
#define MX_FMC_D12_DA12_Pin                     PE15
#define MX_FMC_D12_DA12_GPIOx                   GPIOE
#define MX_FMC_D12_DA12_GPIO_PuPd               GPIO_NOPULL
#define MX_FMC_D12_DA12_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D12_DA12_GPIO_Pin                GPIO_PIN_15
#define MX_FMC_D12_DA12_GPIO_AF                 GPIO_AF12_FMC
#define MX_FMC_D12_DA12_GPIO_Mode               GPIO_MODE_AF_PP

/* Pin PE9 */
#define MX_FMC_D6_DA6_Pin                       PE9
#define MX_FMC_D6_DA6_GPIOx                     GPIOE
#define MX_FMC_D6_DA6_GPIO_PuPd                 GPIO_NOPULL
#define MX_FMC_D6_DA6_GPIO_Speed_High_Default   GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D6_DA6_GPIO_Pin                  GPIO_PIN_9
#define MX_FMC_D6_DA6_GPIO_AF                   GPIO_AF12_FMC
#define MX_FMC_D6_DA6_GPIO_Mode                 GPIO_MODE_AF_PP

/* Pin PD10 */
#define MX_FMC_D15_DA15_Pin                     PD10
#define MX_FMC_D15_DA15_GPIOx                   GPIOD
#define MX_FMC_D15_DA15_GPIO_PuPd               GPIO_NOPULL
#define MX_FMC_D15_DA15_GPIO_Speed_High_Default GPIO_SPEED_FREQ_VERY_HIGH
#define MX_FMC_D15_DA15_GPIO_Pin                GPIO_PIN_10
#define MX_FMC_D15_DA15_GPIO_AF                 GPIO_AF12_FMC
#define MX_FMC_D15_DA15_GPIO_Mode               GPIO_MODE_AF_PP

/*-------------------------------- I2C1       --------------------------------*/

#define MX_I2C1                                 1

/* GPIO Configuration */

/* Pin PB6 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Pu                     GPIO_PULLUP
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB7 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SDA_Pin                         PB7
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_7
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_Pu                     GPIO_PULLUP
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- USART1     --------------------------------*/

#define MX_USART1                               1

#define MX_USART1_VM                            VM_ASYNC

/* GPIO Configuration */

/* Pin PB14 */
#define MX_USART1_TX_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_USART1_TX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART1_TX_Pin                        PB14
#define MX_USART1_TX_GPIOx                      GPIOB
#define MX_USART1_TX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART1_TX_GPIO_Pin                   GPIO_PIN_14
#define MX_USART1_TX_GPIO_AF                    GPIO_AF4_USART1

/* Pin PB15 */
#define MX_USART1_RX_GPIO_ModeDefaultPP         GPIO_MODE_AF_PP
#define MX_USART1_RX_GPIO_Speed                 GPIO_SPEED_FREQ_LOW
#define MX_USART1_RX_Pin                        PB15
#define MX_USART1_RX_GPIOx                      GPIOB
#define MX_USART1_RX_GPIO_PuPd                  GPIO_NOPULL
#define MX_USART1_RX_GPIO_Pin                   GPIO_PIN_15
#define MX_USART1_RX_GPIO_AF                    GPIO_AF4_USART1

/*-------------------------------- USB_OTG_FS --------------------------------*/

#define MX_USB_OTG_FS                           1

#define MX_USB_OTG_FS_HOST                      1

/* GPIO Configuration */

/* Pin PA9 */
#define MX_USB_OTG_FS_VBUS_Pin                  PA9
#define MX_USB_OTG_FS_VBUS_GPIOx                GPIOA
#define MX_USB_OTG_FS_VBUS_GPIO_PuPd            GPIO_NOPULL
#define MX_USB_OTG_FS_VBUS_GPIO_Pin             GPIO_PIN_9
#define MX_USB_OTG_FS_VBUS_GPIO_Mode            GPIO_MODE_INPUT

/* Pin PA11 */
#define MX_USB_OTG_FS_DM_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_USB_OTG_FS_DM_Pin                    PA11
#define MX_USB_OTG_FS_DM_GPIOx                  GPIOA
#define MX_USB_OTG_FS_DM_GPIO_PuPd              GPIO_NOPULL
#define MX_USB_OTG_FS_DM_GPIO_Pin               GPIO_PIN_11
#define MX_USB_OTG_FS_DM_GPIO_AF                GPIO_AF10_OTG1_FS
#define MX_USB_OTG_FS_DM_GPIO_Mode              GPIO_MODE_AF_PP

/* Pin PA12 */
#define MX_USB_OTG_FS_DP_GPIO_Speed             GPIO_SPEED_FREQ_LOW
#define MX_USB_OTG_FS_DP_Pin                    PA12
#define MX_USB_OTG_FS_DP_GPIOx                  GPIOA
#define MX_USB_OTG_FS_DP_GPIO_PuPd              GPIO_NOPULL
#define MX_USB_OTG_FS_DP_GPIO_Pin               GPIO_PIN_12
#define MX_USB_OTG_FS_DP_GPIO_AF                GPIO_AF10_OTG1_FS
#define MX_USB_OTG_FS_DP_GPIO_Mode              GPIO_MODE_AF_PP

/* NVIC Configuration */

/* NVIC OTG_FS_IRQn */
#define MX_OTG_FS_IRQn_interruptPremptionPriority 0
#define MX_OTG_FS_IRQn_PriorityGroup            NVIC_PRIORITYGROUP_4
#define MX_OTG_FS_IRQn_Subriority               0

/*-------------------------------- USB_OTG_HS --------------------------------*/

#define MX_USB_OTG_HS                           1

#define MX_USB_OTG_HS_HOST                      1

/* GPIO Configuration */

/* Pin PB0 */
#define MX_USB_OTG_HS_ULPI_D1_Pin               PB0
#define MX_USB_OTG_HS_ULPI_D1_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D1_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D1_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D1_GPIO_Pin          GPIO_PIN_0
#define MX_USB_OTG_HS_ULPI_D1_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D1_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PB1 */
#define MX_USB_OTG_HS_ULPI_D2_Pin               PB1
#define MX_USB_OTG_HS_ULPI_D2_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D2_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D2_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D2_GPIO_Pin          GPIO_PIN_1
#define MX_USB_OTG_HS_ULPI_D2_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D2_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PB10 */
#define MX_USB_OTG_HS_ULPI_D3_Pin               PB10
#define MX_USB_OTG_HS_ULPI_D3_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D3_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D3_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D3_GPIO_Pin          GPIO_PIN_10
#define MX_USB_OTG_HS_ULPI_D3_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D3_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PB11 */
#define MX_USB_OTG_HS_ULPI_D4_Pin               PB11
#define MX_USB_OTG_HS_ULPI_D4_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D4_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D4_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D4_GPIO_Pin          GPIO_PIN_11
#define MX_USB_OTG_HS_ULPI_D4_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D4_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PH4 */
#define MX_USB_OTG_HS_ULPI_NXT_Pin              PH4
#define MX_USB_OTG_HS_ULPI_NXT_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_NXT_GPIOx            GPIOH
#define MX_USB_OTG_HS_ULPI_NXT_GPIO_PuPd        GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_NXT_GPIO_Pin         GPIO_PIN_4
#define MX_USB_OTG_HS_ULPI_NXT_GPIO_AF          GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_NXT_GPIO_Mode        GPIO_MODE_AF_PP

/* Pin PB12 */
#define MX_USB_OTG_HS_ULPI_D5_Pin               PB12
#define MX_USB_OTG_HS_ULPI_D5_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D5_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D5_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D5_GPIO_Pin          GPIO_PIN_12
#define MX_USB_OTG_HS_ULPI_D5_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D5_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PB13 */
#define MX_USB_OTG_HS_ULPI_D6_Pin               PB13
#define MX_USB_OTG_HS_ULPI_D6_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D6_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D6_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D6_GPIO_Pin          GPIO_PIN_13
#define MX_USB_OTG_HS_ULPI_D6_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D6_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PB5 */
#define MX_USB_OTG_HS_ULPI_D7_Pin               PB5
#define MX_USB_OTG_HS_ULPI_D7_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D7_GPIOx             GPIOB
#define MX_USB_OTG_HS_ULPI_D7_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D7_GPIO_Pin          GPIO_PIN_5
#define MX_USB_OTG_HS_ULPI_D7_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D7_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PI11 */
#define MX_USB_OTG_HS_ULPI_DIR_Pin              PI11
#define MX_USB_OTG_HS_ULPI_DIR_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_DIR_GPIOx            GPIOI
#define MX_USB_OTG_HS_ULPI_DIR_GPIO_PuPd        GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_DIR_GPIO_Pin         GPIO_PIN_11
#define MX_USB_OTG_HS_ULPI_DIR_GPIO_AF          GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_DIR_GPIO_Mode        GPIO_MODE_AF_PP

/* Pin PA5 */
#define MX_USB_OTG_HS_ULPI_CK_Pin               PA5
#define MX_USB_OTG_HS_ULPI_CK_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_CK_GPIOx             GPIOA
#define MX_USB_OTG_HS_ULPI_CK_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_CK_GPIO_Pin          GPIO_PIN_5
#define MX_USB_OTG_HS_ULPI_CK_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_CK_GPIO_Mode         GPIO_MODE_AF_PP

/* Pin PC0 */
#define MX_USB_OTG_HS_ULPI_STP_Pin              PC0
#define MX_USB_OTG_HS_ULPI_STP_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_STP_GPIOx            GPIOC
#define MX_USB_OTG_HS_ULPI_STP_GPIO_PuPd        GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_STP_GPIO_Pin         GPIO_PIN_0
#define MX_USB_OTG_HS_ULPI_STP_GPIO_AF          GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_STP_GPIO_Mode        GPIO_MODE_AF_PP

/* Pin PA3 */
#define MX_USB_OTG_HS_ULPI_D0_Pin               PA3
#define MX_USB_OTG_HS_ULPI_D0_GPIO_Speed_Only_High GPIO_SPEED_FREQ_HIGH
#define MX_USB_OTG_HS_ULPI_D0_GPIOx             GPIOA
#define MX_USB_OTG_HS_ULPI_D0_GPIO_PuPd         GPIO_NOPULL
#define MX_USB_OTG_HS_ULPI_D0_GPIO_Pin          GPIO_PIN_3
#define MX_USB_OTG_HS_ULPI_D0_GPIO_AF           GPIO_AF10_OTG2_HS
#define MX_USB_OTG_HS_ULPI_D0_GPIO_Mode         GPIO_MODE_AF_PP

/* NVIC Configuration */

/* NVIC OTG_HS_IRQn */
#define MX_OTG_HS_IRQn_interruptPremptionPriority 0
#define MX_OTG_HS_IRQn_PriorityGroup            NVIC_PRIORITYGROUP_4
#define MX_OTG_HS_IRQn_Subriority               0

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

#endif  /* __MX_DEVICE_H */

