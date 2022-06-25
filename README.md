# arduino_example
Trying to use Hascal in arduino boards

## How to compile ?
- At first clone the repo :
```
git clone https://github.com/hascal/arduino_example
cd arduino_example
```
- Open `config.json` file in your code editor and replace `{arduino_path}` with your arduino installation path.
- Enter following commands in your terminal :
```
hascal app.has
make
```
Now you can upload `app.hex` to your arduino board with `avrdude`.

**NOTE**: This example only works with arduino uno, but you can edit codes to run this example on your device.
