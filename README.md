# STM32F407_MODBUS_LwIP_FreeRTOS
STM32F407ZGT6 
1. Porting the modbus support, modbus packets vary length. Need use timer to sync according to modbus specification.
2. The flash section vary with section number from 16k to 128k
3. Enabled the usart interrupt, but not enabled the DMA.
