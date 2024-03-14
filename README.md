# TP_Sys_uC_3DN

This project is an learning project of using STM32L021K4T6 with KiCAD.
There is three part : 
	- Datasheets : All datashetts needed for the project
	- Firmware : STM32 firmware using STM32CubeIDE
	- Hardware : Conception with KiCAD.

# Hardware

#STM32L021K4T6
PB9 pin is connected to reset. L1,C4 and C5 is used to filter power supply.

#POWER SUPPLY
For the power supply, the datasheet indicated that all C's need to be more than 0.47uF.

#DAC
For DAC, in page 25, datasheet of MCP4801 indicates that C9 has to be 100n and C10 10u. 
not(CS) is the Chip Select input pin, which requires an active low to enable serial clock and data functions. 
not(LDAC) (latch DAC synchronization input) pin is used to transfer the input latch register to the DAC register (output latches, VOUT). 
When this pin is low, VOUT is updated with input register content. 
This pin can be tied to low (VSS) if the VOUT update is desired at the rising edge of the CS pin. 
This pin can be driven by an external control device such as an MCU I/O pin. 
MISO pin of SDI not used because STM32 is used Transmit only.

# Firmware

#LL drivers
Using LL drivers instead of using HAL drivers allows to use low-level registers to have beter optimisation.
LL drivers are specific to uC and HAL drivers are more portable.

#__STATIC_INLINE
__STATIC_INLINE is used to declare a function statically and the compiler will generate function only where necessary.