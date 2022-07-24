# arduino_example
Trying to use Hascal in arduino boards

## Perquisites
- Latest Hascal Compiler(from git repo)
- `avr-gcc`>7.5.x
- Arduino IDE
- `make`
- `avrdude`(to uploading hex file to board)
- Arduino Uno board

## Getting Started
- At first clone the repo :
```
git clone https://github.com/hascal/arduino_example
cd arduino_example
```
- Open `config.json` file in your code editor and replace `{arduino_path}` with your arduino installation path.
- generate hex file :
```
make
```
- Upload generated hex file to board :
```
avrdude -C avrdude.conf -v -patmega328p -carduino -P/dev/ttyUSB4 -b115200 -D -Uflash:w: app.hex 
```
> Replace `/dev/ttyUSB4` with your COM port.
**NOTE**: This example only works with Arduino Uno, but you can edit codes to run this example on your device.
