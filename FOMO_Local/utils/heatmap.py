import numpy as np
from configs.config import (
    GRID_SIZE,
    NUM_CLASSES,
)



class HeatmapGenerator:
    def __init__(
            self,
            grid_size=GRID_SIZE,
            num_classes=NUM_CLASSES
            # grid_size=(12, 12),
            # num_classes=1
    ):
        self.grid_h = grid_size[0]
        self.grid_w = grid_size[1]
        self.num_classes = num_classes

    def generate(self, labels):
        """
        labels:
        [
            {
                class_id,
                x,
                y,
                w,
                h
            }
        ]
        return

        heatmap
        shape
        (12,12,2)

        channel0 : background
        channel1 : drone
        """

        heatmap = np.zeros(
            (
                self.grid_h,
                self.grid_w,
                self.num_classes + 1
            ),
            dtype=np.float32
        )

        #
        # 처음에는 모두 background
        #
        heatmap[:, :, 0] = 1.0

        for obj in labels:
            cls = obj["class_id"]
            x = obj["x"]
            y = obj["y"]

            #
            # normalized -> grid
            #
            # gx = int(round(x * (self.grid_w - 1)))
            # gy = int(round(y * (self.grid_h - 1)))
            gx = int(x * self.grid_w)
            gy = int(y * self.grid_h)

            #
            # boundary
            #
            gx = max(
                0,
                min(
                    gx,
                    self.grid_w - 1
                )
            )
            gy = max(
                0,
                min(
                    gy,
                    self.grid_h - 1
                )
            )

            #
            # background 제거
            #
            heatmap[gy, gx, 0] = 0.0

            #
            # class 활성화
            #
            radius = 1

            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):

                    ny = gy + dy
                    nx = gx + dx

                    if (
                            0 <= ny < self.grid_h and
                            0 <= nx < self.grid_w
                    ):
                        heatmap[ny, nx, 0] = 0.0

                        heatmap[
                            ny,
                            nx,
                            cls + 1
                        ] = 1.0
            # heatmap[
            #     gy,
            #     gx,
            #     cls + 1
            # ] = 1.0

        return heatmap