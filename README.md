# MLX90640 access library

This repository contains a simple access library for the MLX90640
FIR sensor manufactured by Melexis (this repository is in no way
associated with the manufacturer). It uses a simple I2C wrapper to
access the sensor and can be used on platforms like the RaspberryPi.
Details are described [in the accompanying blog article](https://www.tspi.at/2024/03/31/pimlx90640py.html)

![](https://raw.githubusercontent.com/tspspi/pymlx90640/master/examples/capture.png)

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

### Fetch-Frame API

```
from fbsdi2c import FbsdI2C
from mlx90640 import MLX90640

import matplotlib.pyplot as plt
#import numpy as np

with FbsdI2C() as i2c:
    with MLX90640(i2c) as mlx:
        framedata = mlx.fetch_frame()

        fig, ax = plt.subplots(figsize=(6.4*2, 4.8*2))

        im = ax.imshow(framedata, cmap="plasma", interpolation="gaussian")
        ax.set_title("Temperatures")
        cbar = fig.colorbar(im, ax=ax, extend="both")
        cbar.minorticks_on()

        plt.show()
```

### pysimplecam API

```
from fbsdi2c import FbsdI2C
from mlx90640 import MLX90640

import matplotlib.pyplot as plt

plt.ion()
fig, ax = plt.subplots()
axim = None

framectr = 0

def updateFrame(cam, frame):
    global axim, framectr, fig

    if axim is None:
        axim = ax.imshow(frame, cmap = "plasma", interpolation="gaussian")
    else:
        axim.set_data(frame)
        axim.autoscale()
    fig.canvas.flush_events()

    plt.savefig(f"simplecam01_cap/{framectr:08}.png")

    framectr = framectr + 1
    if framectr > 60:
        return False
    else:
        return True


with FbsdI2C() as i2c:
    with MLX90640(i2c) as mlx:
        mlx._set_refresh_rate(2)
        mlx.frameCallback = [ updateFrame ]
        mlx.stream()
```


