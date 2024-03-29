from fbsdi2c import FbsdI2C
from mlx90640 import MLX90640

import matplotlib.pyplot as plt

with FbsdI2C() as i2c:
    with MLX90640(i2c) as mlx:
        rawframe = mlx._fetch_raw_frame_sync()
        framedata = mlx._process_frame(rawframe)
        fig, ax = plt.subplots(1, 2, figsize=(6.4*2, 4.8))
        ax[0].imshow(rawframe, cmap="plasma", interpolation="gaussian")
        ax[0].set_title("Raw pixel data")

        im = ax[1].imshow(framedata, cmap="plasma", interpolation="gaussian")
        ax[1].set_title("Temperatures")
        cbar = fig.colorbar(im, ax=ax[1], extend="both")
        cbar.minorticks_on()

        plt.savefig("capture.png")
        plt.show()
