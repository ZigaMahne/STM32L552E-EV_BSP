# STM32L552E-EV Evaluation board

## Overview

The STM32L552E-EV Evaluation board is designed as a complete demonstration and development platform forSTMicroelectronics Arm® Cortex®-M33 core with Arm® TrustZone® and the ARMv8-M mainline security extension.

The STM32L552E-EV Evaluation board is based on an ultra-low-power STM32L552ZET6QU microcontroller with 512 Kbytes of Flash memory and 256 Kbytes of SRAM, one external memory interface supporting an LCD interface, one Octo-SPI memory interface, one USB Type-C® FS with Power Delivery controller, two SAI ports, four I2C buses, six USART ports, three SPI interfaces, one CAN‑FD controller, one SDMMC interface, two 12-bit ADC, two 12-bit DAC, two operational amplifiers, two ultra-low comparators, four digital filters for sigma-delta modulator, up to 16 timers, touch-sensing capability, and debugging supported by SWD, JTAG and ETM interface.

The full range of hardware features on the board helps the user to evaluate all the peripherals (USB FS, USART, digital microphones, ADC and DAC, dot-matrix TFT LCD, LDR, SRAM, octal Flash memory device, microSD™ card, sigma-delta modulators, smartcard, CAN‑FD transceiver, I2C, EEPROM), and to develop applications. Extension headers allow easy connection of a daughterboard or wrapping board for a specific application.

An ST-LINK/V2-1 is integrated on the board, as embedded in-circuit debugger and programmer for the STM32 MCU and the USB Virtual COM port bridge.

## Getting started

- [User manual](https://www.st.com/resource/en/user_manual/um2597-evaluation-board-with-stm32l552ze-mcu-stmicroelectronics.pdf)

### ST-LINK driver installation and firmware upgrade (on Microsoft Windows)

1. Download the latest [ST-LINK driver](https://www.st.com/en/development-tools/stsw-link009.html).
2. Extract the archive and run `dpinst_amd64.exe`. Follow the displayed instructions.
3. Download the latest [ST-LINK firmware upgrade](https://www.st.com/en/development-tools/stsw-link007.html).
4. Extract the archive and run the `ST-LinkUpgrade.exe` program.
5. Connect the board to your PC using a USB cable and wait until the USB enumeration is completed.
6. In the **ST-Link Upgrade** program, press the **Device Connect** button.
7. When the ST-LINK driver is correctly installed, the current ST-LINK version is displayed.
8. Press the **Yes >>>>** button to start the firmware upgrade process.

## Technical reference

- [STM32L552ZE microcontroller](https://www.st.com/en/microcontrollers-microprocessors/stm32l552ze.html)
- [STM32L552E-EV board](https://www.st.com/en/evaluation-tools/stm32l552e-ev.html)
- [User manual](https://www.st.com/resource/en/user_manual/um2597-evaluation-board-with-stm32l552ze-mcu-stmicroelectronics.pdf)
- [Data brief](https://www.st.com/resource/en/data_brief/stm32l552e-ev.pdf)
- [Schematic](https://www.st.com/resource/en/schematic_pack/mb1372-l552zeq-c02_schematic.pdf)
