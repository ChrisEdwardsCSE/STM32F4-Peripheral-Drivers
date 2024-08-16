# STM32F4 Drivers
Hello, I wrote these drivers by dissecting and imitating STM's HAL API to build myself a lightweight alternative for using the peripherals on the STM32F4 boards. The motivation for writing these is to help myself become more comfortable with register-level/bare metal microcontroller programming and give myself a more comprehensive understanding of microcontrollers. I plan to continuously build and practice on these and hope to improve my code to be more robust, safe, and versatile. -Chris

## GPIO
Driver supports GPIO pin configuration, GPIO manipulation, and GPIO Interrupt capabilities.

## SPI
Driver supports Master SPI configuration, transaction using polling, and transaction using interrupts. Working on DMA & slave mode next!
