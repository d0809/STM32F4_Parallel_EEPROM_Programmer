# STM32F4_Parallel_EEPROM_Programmer
<h1>Description:</h1>

This is a simple STM32F429 program to program and verify AT28C64B parallel EEPROMs. It reads values from an array(in values.h) and writes them to the EEPROM, this is possible because the F429 has 2MB of Flash. The program uses data polling(described in the datasheet) to detect the end of write cycles instead of waiting for the maximum duration of a write cycle(10ms) which greatly increases programming speed. No shift registers are used as the F429 has a lot of GPIO pins.

Since EEPROM chips are not as fast as SRAM chips, delays have been inserted into necessary locations. Empty for loops with volatile indices(to prevent the compiler from optimizing them out) were used to generate these delays.

Since this program depends on the data polling function, it can be used to only program 28C62Bs or other EEPROMs with the same feature. The program can be easily modified to program other parallel EEPROMs simply by removing the polling code and replacing it with a fixed delay. It will be slow but it is a better alternative to buying a dedicated programmer.

<h1>Connection Table:</h1>

*STM32F429* | *AT28C64B*
------------ | -------------
PB[0-7] | IO[0-7]
PB[8-15] | A[0-7]
PE[2-6] | A[8-12]
PC8 | WE
PC9 | OE
PC10 | CE 

Make sure to use a sufficient decoupling capacitor if you are implementing this circuit on a breadboard. The 28C64B has a protection features which blocks all writes if VCC is lower than 3.8V and waits for 5ms before enabling writes once VCC goes above 3.8V. I had problems writing to the device until I used a decoupling capacitor.

Also make sure to connect the VCC pin of the EEPROM to 5V to be able to program it and connect its pins to only 5V capable GPIO pins. The pins I used were 5V capable but this may not be the case if you are using another MCU.


<h1>TODO:</h1>

- Add UART or SD card support and use that instead of reading data from an array stored in flash memory
- Add support for other EEPROMs and/or make the process controllable over UART


