from fbsdi2c import FbsdI2C
from mlx90640 import MLX90640

import matplotlib.pyplot as plt

framectr = 0
frames = []

def updateFrame(cam, frame):
    global framectr, frames

    frames.append(frame)
    print(".")

    framectr = framectr + 1
    if framectr > 60:
        return False
    else:
        return True


with FbsdI2C() as i2c:
    with MLX90640(i2c) as mlx:
        mlx._set_refresh_rate(4)
        mlx.frameCallback = [ updateFrame ]
        mlx.stream()

for iframe, frame in enumerate(frames):
    fig, ax = plt.subplots()
    im = ax.imshow(frame, cmap = "plasma", interpolation="gaussian")
    cbar = fig.colorbar(im, ax=ax, extend="both")
    cbar.minorticks_on()
    plt.savefig(f"simplecam02_cap/{iframe:08}.png")
    plt.close("all")
