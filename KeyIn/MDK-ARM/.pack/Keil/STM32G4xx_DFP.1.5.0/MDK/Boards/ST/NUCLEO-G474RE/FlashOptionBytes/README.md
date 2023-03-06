# FlashOptionBytes project

## Overview

The **FlashOptionBytes** project demonstrates how to set the **FLASH option bytes** for a STM32G4xx device.

## Description

An assembler file containing the **FLASH option bytes** is used to change the settings.\
The Device Family pack Keil.STM32G4xx_DFP contains two different template files:
  * STM32G4xx_SB_OPT.s: for STM32G4 Flash categories 2, 4 (single bank)
  * STM32G4xx_DB_OPT.s: for STM32G4 Flash categories 3 (dual bank)

For each template file exist an extra Flash Programming Algorithm
  * STM32G4xx_SB_OPT.FLM: STM32G4xx single bank Flash Options
  * STM32G4xx_DB_OPT.FLM: STM32G4xx dual bank Flash Options

## How it works

 * Create an uVision project with the appropriate assembler template file and the corresponding 
   [Flash Programming Algorithm](https://www.keil.com/support/man/docs/uv4/uv4_fl_dlconfiguration.htm).
 * Set the **FLASH option bytes** values using the uVision 
   [Configuration Wizard](https://www.keil.com/support/man/docs/uv4/uv4_ut_configwizard.htm).
 * Build the project and flash the output.
 * When flashing the output no Flash content is changed but the **FLASH option bytes** values 
   are written to the corresponding Flash registers.
 * Additional **FLASH option bytes** loading can be configured in the 
   [dbgconf file](https://www.keil.com/support/man/docs/ulink2/ulink2_ctx_pack.htm).
 
## Note:

Select all **FLASH option bytes** values very carefully and double check them before flashing.\
Wrong **FLASH option bytes** values can lock the device!



