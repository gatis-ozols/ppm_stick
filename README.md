# ppm_stick
STM32 HID Joystick with PPM input.

The STM32 Workbench project is generated with ST Microelectronics CubeMX (http://www.st.com/en/development-tools/stm32cubemx.html) configuration tool

# Hardware
* STM32F103C8T6 "Blue pill" development board (http://wiki.stm32duino.com/index.php?title=Blue_Pill);
* ST-LINK/V2 clone USB programmer;
* FlySky/iRangeX A8S receiver;
* FlySky/iRangeX I6X transmitter;

PPM signal wire must be connected to **B9** pin of development board.

# How to build
I use "System Workbench for STM32" - http://www.openstm32.org/System+Workbench+for+STM32

# Uploading
* If using ST-LINK/V2 System workbench or ST-LINK Utility can be used (http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link004.html);
* If using UART "STM32 Flash loader demonstrator" can be used (http://www.st.com/en/development-tools/flasher-stm32.html);
