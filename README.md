# fetcher

A process for fetching data from sensors using the I2C protocol.

## Usage

See the help documentation with the command: `use fetcher`.

The output of fetcher looks like:

```console
Time: 1295 ms
Temperature: 25.02 C
Pressure: 99.94 kPa
...
```

All sensor data output is printed in plain-text, with its type specifier (i.e. temperature), its value and its unit.

## Board ID EEPROM Encoding

In order for fetcher to recognize the sensors on the board, the EEPROM must encode the ID in this format:

```
CU InSpace <board name> REV <x>: <manufacture date>
<sensor 1 id> <address>
<sensor 2 id> <address>
```

**Fields:**

- `<board name>`: The name of the board. Ex: `Sensor board`
- `REV <x>`: The revision of the board. Ex: `REV B`
- `<sensor id>`: The identifier of the sensor. Ex: `MS5611`
- `<address>`: The 7 bit I2C address of the sensor in hexadecimal (without the leading 0x). Ex: `77`

<!--- Links --->

[packager]: https://github.com/CarletonURocketry/packager
