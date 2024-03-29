# MLX90640 access library

This repository contains a simple access library for the MLX90640
FIR sensor manufactured by Melexis (this repository is in no way
associated with the manufacturer). It uses a simple I2C wrapper to
access the sensor and can be used on platforms like the RaspberryPi.

## Required I2C library

The I2C library is not contained in the requirements since it follows
a modular approach. The modules author personally uses his own
[pyfbsdi2c-tspspi](https://github.com/tspspi/pyfbsdi2c) wrapper that 
can be installed using

```
pip install pyfbsdi2c-tspspi
```

This wrapper provides ```ioctl``` abstractions on FreeBSD and is mainly
used on a RaspberryPi to talk to different I2C devices. Any other library
exposing the [pylabdevs](https://github.com/tspspi/pylabdevs) [I2CBus](https://github.com/tspspi/pylabdevs/blob/master/src/labdevices/i2cbus.py)
interface can be used to make this library as modular as possible.

## Installation

```
pip install pymlx90640-tspspi
```

## Usage




