# arduino_example
Trying to use Hascal in arduino boards

## Perquisites
- Latest Hascal Compiler(from git repo)
- `avr-gcc`>7.5.x
- Arduino IDE
- `make`
- `avrdude`(to uploading hex file to board)
- Arduino Uno

## How to compile ?
- At first clone the repo :
```
git clone https://github.com/hascal/arduino_example
cd arduino_example
```
- Open `config.json` file in your code editor and replace `{arduino_path}` with your arduino installation path.
- Enter following command in your terminal :
```
make
```
Now you can upload `app.hex` to your arduino board with `avrdude`.

**NOTE**: This example only works with arduino uno, but you can edit codes to run this example on your device.
