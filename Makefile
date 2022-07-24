all: binfile hexfile

binfile :
	avr-gcc -Os -mmcu=atmega328p -ffunction-sections -fdata-sections -Wl,--gc-sections app.o -o app.out

hexfile :
	avr-objcopy -O ihex app.out app.hex

.PHONY : all binfile hexfile