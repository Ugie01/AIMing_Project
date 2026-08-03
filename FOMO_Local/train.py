import tensorflow as tf
import cv2
import numpy as np

from models.fomo import build_fomo

from utils.dataset import YOLODataset
from utils.heatmap import HeatmapGenerator

from configs.config import (
    INPUT_SIZE,
    GRID_SIZE,
    NUM_CLASSES,
    BATCH_SIZE,
    EPOCHS,
    LEARNING_RATE
)

import math


# =====================================================
# Image Loader
# =====================================================
def load_image(image_path):
    image = cv2.imread(
        str(image_path)
    )
    if image is None:
        raise ValueError(
            f"Image load failed : {image_path}"
        )

    # BGR -> RGB
    image = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2RGB
    )

    # 96x96
    image = cv2.resize(
        image,
        INPUT_SIZE
    )
    image = image.astype(
        np.float32
    )

    # normalize
    image /= 255.0

    return image


# =====================================================
# Tensorflow Dataset 생성
# =====================================================
def create_tf_dataset(
        yolo_dataset,
        shuffle=True
):
    heatmap_generator = HeatmapGenerator(
        grid_size=GRID_SIZE,
        num_classes=NUM_CLASSES
    )

    def generator():
        for idx in range(
            len(yolo_dataset)
        ):
            sample = yolo_dataset[idx]
            image = load_image(
                sample["image_path"]
            )
            heatmap = heatmap_generator.generate(
                sample["labels"]
            )
            yield (
                image,
                heatmap
            )

    dataset = tf.data.Dataset.from_generator(
        generator,
        output_signature=(
            tf.TensorSpec(
                shape=(
                    INPUT_SIZE[0],
                    INPUT_SIZE[1],
                    3
                ),
                dtype=tf.float32
            ),
            tf.TensorSpec(
                shape=(
                    GRID_SIZE[0],
                    GRID_SIZE[1],
                    NUM_CLASSES + 1
                ),
                dtype=tf.float32
            )
        )
    )

    if shuffle:
        dataset = dataset.shuffle(
            buffer_size=1000,
            reshuffle_each_iteration=True
        )
    dataset = dataset.batch(
        BATCH_SIZE,
        drop_remainder=False
    )
    dataset = dataset.repeat()
    dataset = dataset.prefetch(
        tf.data.AUTOTUNE
    )

    return dataset




# =====================================================
# Main
# =====================================================
def main():
    # ----------------------------------
    # Dataset
    # ----------------------------------
    train_dataset = YOLODataset(
        image_dir="dataset/train/images",
        label_dir="dataset/train/labels"
    )
    val_dataset = YOLODataset(
        image_dir="dataset/validation/images",
        label_dir="dataset/validation/labels"
    )

    train_ds = create_tf_dataset(
        train_dataset,
        shuffle=True
    )
    val_ds = create_tf_dataset(
        val_dataset,
        shuffle=False
    )


    # ----------------------------------
    # Model
    # ----------------------------------
    model = build_fomo()
    model.summary()


    # ----------------------------------
    # Compile
    # ----------------------------------
    model.compile(
        optimizer=tf.keras.optimizers.Adam(
            learning_rate=LEARNING_RATE
        ),
        # loss=tf.keras.losses.CategoricalFocalCrossentropy(
        #     gamma=2.0
        # ),
        loss=tf.keras.losses.CategoricalCrossentropy(),
        metrics=[
            "accuracy"
        ]
    )


    # ----------------------------------
    # Callback
    # ----------------------------------
    checkpoint = tf.keras.callbacks.ModelCheckpoint(
        filepath="checkpoints/best_fomo.keras",
        monitor="val_loss",
        save_best_only=True,
        mode="min",
        verbose=1
    )

    # early_stop = tf.keras.callbacks.EarlyStopping(
    #     monitor="val_loss",
    #     patience=15,
    #     restore_best_weights=True
    # )


    # ----------------------------------
    # Train
    # ----------------------------------
    history = model.fit(
        train_ds,
        validation_data= val_ds,
        epochs= EPOCHS,
        steps_per_epoch=math.ceil(len(train_dataset) / BATCH_SIZE),
        validation_steps=math.ceil(len(val_dataset) / BATCH_SIZE),
        # steps_per_epoch= len(train_dataset)//BATCH_SIZE,
        # validation_steps= len(val_dataset)//BATCH_SIZE,
        callbacks=[
            checkpoint,
            # early_stop
        ]
    )
    print(
        "Training Finished"
    )


if __name__ == "__main__":
    main()









# import tensorflow as tf
# import cv2
# import numpy as np
# from pathlib import Path
#
# from models.fomo import build_fomo
# from utils.dataset import YOLODataset
# from utils.heatmap import HeatmapGenerator
#
# from configs.config import (
#     INPUT_SIZE,
#     GRID_SIZE,
#     NUM_CLASSES,
#     BATCH_SIZE,
#     EPOCHS,
#     LEARNING_RATE
# )
#
#
#
# # =====================================================
# # Image Loader
# # =====================================================
# def load_image(image_path):
#     """
#     image_path
#         jpg path
#
#     return
#         (96,96,3)
#         float32
#         0~1
#     """
#     image = cv2.imread(
#         str(image_path)
#     )
#
#     if image is None:
#         raise ValueError(
#             f"Image load failed : {image_path}"
#         )
#
#     # BGR -> RGB
#     image = cv2.cvtColor(
#         image,
#         cv2.COLOR_BGR2RGB
#     )
#     image = cv2.resize(
#         image,
#         INPUT_SIZE
#     )
#     image = image.astype(
#         np.float32
#     )
#     image /= 255.0
#
#     return image
#
#
# # =====================================================
# # Dataset Generator
# # =====================================================
# def create_tf_dataset(
#         yolo_dataset,
#         shuffle=True
# ):
#     heatmap_generator = HeatmapGenerator(
#         grid_size=GRID_SIZE,
#         num_classes=NUM_CLASSES
#     )
#
#     def generator():
#         for idx in range(
#                 len(yolo_dataset)
#         ):
#             sample = yolo_dataset[idx]
#             image = load_image(
#                 sample["image_path"]
#             )
#             heatmap = heatmap_generator.generate(
#                 sample["labels"]
#             )
#             yield (
#                 image,
#                 heatmap
#             )
#
#     dataset = tf.data.Dataset.from_generator(
#         generator,
#         output_signature=(
#             tf.TensorSpec(
#                 shape=(
#                     INPUT_SIZE[0],
#                     INPUT_SIZE[1],
#                     3
#                 ),
#                 dtype=tf.float32
#             ),
#
#             tf.TensorSpec(
#                 shape=(
#                     GRID_SIZE[0],
#                     GRID_SIZE[1],
#                     NUM_CLASSES + 1
#                 ),
#                 dtype=tf.float32
#             )
#         )
#     )
#
#     if shuffle:
#         dataset = dataset.shuffle(
#             1000,
#             reshuffle_each_iteration=True
#         )
#
#     dataset = dataset.batch(
#         BATCH_SIZE,
#         drop_remainder=False
#     )
#     dataset = dataset.prefetch(
#         tf.data.AUTOTUNE
#     )
#
#     return dataset
#
#
#
# # =====================================================
# # Main
# # =====================================================
# def main():
#     # -----------------------------
#     # Dataset
#     # -----------------------------
#     train_dataset = YOLODataset(
#         image_dir= "dataset/train/images",
#         label_dir= "dataset/train/labels"
#     )
#     val_dataset = YOLODataset(
#         image_dir= "dataset/validation/images",
#         label_dir= "dataset/validation/labels"
#     )
#     train_ds = create_tf_dataset(
#         train_dataset,
#         shuffle=True
#     )
#     val_ds = create_tf_dataset(
#         val_dataset,
#         shuffle=False
#     )
#
#
#     # -----------------------------
#     # Model
#     # -----------------------------
#     model = build_fomo()
#     model.summary()
#
#
#     # -----------------------------
#     # Compile
#     # -----------------------------
#     model.compile(
#         optimizer=tf.keras.optimizers.Adam(
#             learning_rate=LEARNING_RATE
#         ),
#         loss=
#         tf.keras.losses.CategoricalCrossentropy(),
#         metrics=[
#             "accuracy"
#         ]
#     )
#
#
#     # -----------------------------
#     # Callback
#     # -----------------------------
#     checkpoint = tf.keras.callbacks.ModelCheckpoint(
#         filepath= "checkpoints/best_fomo.keras",
#         monitor= "val_loss",
#         save_best_only= True,
#         mode= "min",
#         verbose= 1
#     )
#
#     early_stop = tf.keras.callbacks.EarlyStopping(
#         monitor= "val_loss",
#         patience= 15,
#         restore_best_weights= True
#     )
#
#
#     # -----------------------------
#     # Train
#     # -----------------------------
#     history = model.fit(
#         train_ds,
#         validation_data= val_ds,
#         epochs= EPOCHS,
#         steps_per_epoch=len(train_dataset) // BATCH_SIZE,
#         validation_steps=len(val_dataset) // BATCH_SIZE,
#         callbacks=[
#             checkpoint,
#             early_stop
#         ]
#     )
#     print(
#         "Training Finished"
#     )
#
#
#
# if __name__ == "__main__":
#     main()