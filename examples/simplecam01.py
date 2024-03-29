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
