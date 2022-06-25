build:
	hascal app.has
	avr-gcc -Os -mmcu=atmega328p -ffunction-sections -fdata-sections -Wl,--gc-sections app.o -o app.out
	avr-objcopy -O ihex app.out app.hex