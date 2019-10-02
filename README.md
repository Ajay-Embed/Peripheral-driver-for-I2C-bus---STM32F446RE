# Peripheral-driver-for-I2C-bus---STM32F446RE
Here  I have included the peripheral driver for the I2C bus for the STM32f446re MCU, along witht the MCU specific header file. There are various API's that include Polling as well as Interrupt based.


1. Make sure you add the rcc.h and rcc.c files also in the driver file
2. The API's in this driver are well documented.
3. I have also attached the datasheet, reference manual nad the pin schematic for STM32F446RE MCU
4. There is an example code for testing the Driver, this code i2c_master_rx_testing.c sends data to Serial monitor of an arduino.
5. I'll be further adding some sensor interface examples also.
