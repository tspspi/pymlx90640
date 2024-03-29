from fbsdi2c import FbsdI2C
from mlx90640 import MLX90640

import matplotlib.pyplot as plt

with FbsdI2C() as i2c:
    with MLX90640(i2c) as mlx:
        framedata = mlx._fetch_frame_sync()

        fig, ax = plt.subplots(figsize=(6.4*2, 4.8*2))
        im = ax.imshow(framedata, cmap="plasma", interpolation="gaussian")
        ax.set_title("Temperatures")
        cbar = fig.colorbar(im, ax=ax, extend="both")
        cbar.minorticks_on()

        plt.savefig("capture2.png")
        plt.show()
