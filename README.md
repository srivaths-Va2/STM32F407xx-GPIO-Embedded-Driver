# STM32F407xx-GPIO-Embedded-Driver
This repository contains all code that provide driver APIs and a Hardware Abstraction Layer (HAL) for GPIO ports on STM32F407xx MCUs, which are developed by STMicroelectronics. These driver APIs can be used to configure and program GPIO ports as per user's needs.

## Project organization
```
STM32F407xx-GPIO-Embedded-Driver
|___ .settings
|    |___ language.settings.xml
|    |___ org.eclipse.cdt.core.prefs
|    |___ org.eclipse.cdt.resources.prefs
|___ Debug
|    |___ Src
|    |    |___ main.cyclo
|    |    |___ main.d
|    |    |___ main.o
|    |    |___ main.su
|    |    |___ subdir.mk
|    |    |___ syscalls.cyclo
|    |    |___ syscalls.d
|    |    |___ syscalls.o
|    |    |___ syscalls.su
|    |    |___ sysmem.cyclo
|    |    |___ sysmem.d
|    |    |___ sysmem.o
|    |    |___ sysmem.su
|    |___ Startup
|    |    |___startup_stm32f407vgtx.d
|    |    |___ startup_stm32f407vgtx.o
|    |    |___ subdir.mk
|    |___ STM32F407xx-GPIO-Driver-v2.elf
|    |___ STM32F407xx-GPIO-Driver-v2.list
|    |___ STM32F407xx-GPIO-Driver-v2.map
|    |___ makefile
|    |___ objects.list
|    |___ objects.mk
|    |___ sources.mk
|___ Src
|    |___ main.c                                                  - Example code
|    |___ syscalls.c
|    |___ sysmem.c
|___ Startup
|    |___ startup_stm32f407vgtx.s
|___ drivers
|    |___ Inc
|    |    |___ stm32f407xx.h                                      - MCU header file
|    |    |___ stm32f407xx_gpio_driver.h                          - GPIO port specific header file
|    |___ Src
|         |___ stm32f407xx_gpio_driver.c                          - Driver source file where driver API definitions are contained
|___ .cproject
|___ .project
|___ README.md
|___ STM32F407VGTX_FLASH.Id
|___STM32F407VGTX_RAM.Id
```

## Installing the Project
To install the project, create an empty folder and follow the steps given below - 
1. Create a STM32CubeIDE workspace for the new folder created
2. Open Git-Bash and type the following command
   `git clone https://github.com/srivaths-Va2/STM32F407xx-GPIO-Embedded-Driver.git`
   



