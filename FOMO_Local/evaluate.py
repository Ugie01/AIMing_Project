import os
import time
import cv2
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

from train import load_image
from utils.dataset import YOLODataset
from utils.heatmap import HeatmapGenerator

from configs.config import (
    INPUT_SIZE,
    GRID_SIZE,
    NUM_CLASSES
)


# ============================
# Paths
# ============================
MODEL_PATH = "checkpoints/best_fomo.keras"

TEST_IMAGE_DIR = "dataset/test/images"
TEST_LABEL_DIR = "dataset/test/labels"

SAVE_DIR = "evaluation"

PRED_DIR = os.path.join(SAVE_DIR, "prediction")
HEAT_DIR = os.path.join(SAVE_DIR, "heatmap")

os.makedirs(PRED_DIR, exist_ok=True)
os.makedirs(HEAT_DIR, exist_ok=True)


# ============================
# Load
# ============================
dataset = YOLODataset(
    TEST_IMAGE_DIR,
    TEST_LABEL_DIR
)

heatmap_generator = HeatmapGenerator(
    GRID_SIZE,
    NUM_CLASSES
)

model = tf.keras.models.load_model(
    MODEL_PATH,
    compile=False
)

print("Model Loaded")


# ============================
# Detection Point
# ============================
def get_detection_points(
        prob_map,
        threshold=0.5
):

    binary = (
        prob_map > threshold
    ).astype(np.uint8)


    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8
    )


    points = []


    for i in range(1, num_labels):

        cx = centroids[i][0]
        cy = centroids[i][1]


        px = int(
            (cx + 0.5)
            *
            INPUT_SIZE[0]
            /
            GRID_SIZE[0]
        )

        py = int(
            (cy + 0.5)
            *
            INPUT_SIZE[1]
            /
            GRID_SIZE[1]
        )

        points.append(
            (
                px,
                py
            )
        )

    return points



# ============================
# Metrics
# ============================
TP = 0
FP = 0
FN = 0

times = []


# ============================
# Evaluation
# ============================
MATCH_DISTANCE = 8


for idx in range(len(dataset)):

    sample = dataset[idx]


    image = load_image(
        sample["image_path"]
    )


    start = time.perf_counter()


    pred = model.predict(
        image[np.newaxis],
        verbose=0
    )[0]


    end = time.perf_counter()


    times.append(
        end-start
    )


    # ========================
    # GT points
    # ========================

    gt_points = []

    for obj in sample["labels"]:

        gx = int(
            obj["x"]
            *
            INPUT_SIZE[0]
        )

        gy = int(
            obj["y"]
            *
            INPUT_SIZE[1]
        )

        gt_points.append(
            (
                gx,
                gy
            )
        )



    # ========================
    # Prediction points
    # ========================

    points = get_detection_points(
        pred[...,1],
        # threshold=0.5
        threshold=0.3
    )



    # ========================
    # Matching
    # ========================

    matched_pred = set()


    for gt in gt_points:

        matched = False


        for i, pred_point in enumerate(points):

            if i in matched_pred:
                continue


            dist = np.sqrt(
                (gt[0]-pred_point[0])**2
                +
                (gt[1]-pred_point[1])**2
            )


            if dist <= MATCH_DISTANCE:

                TP += 1

                matched_pred.add(i)

                matched = True

                break


        if not matched:

            FN += 1



    FP += (
        len(points)
        -
        len(matched_pred)
    )



    # ========================
    # Heatmap
    # ========================

    heat = (
        pred[...,1]
        *
        255
    ).astype(np.uint8)


    heat = cv2.resize(
        heat,
        INPUT_SIZE
    )


    heat = cv2.applyColorMap(
        heat,
        cv2.COLORMAP_JET
    )


    cv2.imwrite(
        os.path.join(
            HEAT_DIR,
            f"{idx:04d}.png"
        ),
        heat
    )



    # ========================
    # Prediction Image
    # ========================

    rgb = (
        image*255
    ).astype(np.uint8)


    for cx, cy in points:

        cv2.circle(
            rgb,
            (
                cx,
                cy
            ),
            4,
            (255,0,0),
            2
        )


    cv2.imwrite(
        os.path.join(
            PRED_DIR,
            f"{idx:04d}.png"
        ),
        cv2.cvtColor(
            rgb,
            cv2.COLOR_RGB2BGR
        )
    )



# ============================
# Final Metrics
# ============================

precision = TP / (
    TP + FP + 1e-8
)

recall = TP / (
    TP + FN + 1e-8
)

f1 = (
    2 *
    precision *
    recall
    /
    (
        precision
        +
        recall
        +
        1e-8
    )
)


avg_time = np.mean(times)

fps = 1.0 / avg_time


model_size = (
    os.path.getsize(
        MODEL_PATH
    )
    /
    1024
)



# ============================
# Save
# ============================

with open(
        os.path.join(
            SAVE_DIR,
            "metrics.txt"
        ),
        "w"
) as f:

    f.write(
        f"TP       : {TP}\n"
    )

    f.write(
        f"FP       : {FP}\n"
    )

    f.write(
        f"FN       : {FN}\n"
    )

    f.write(
        f"Precision: {precision:.4f}\n"
    )

    f.write(
        f"Recall   : {recall:.4f}\n"
    )

    f.write(
        f"F1 Score : {f1:.4f}\n"
    )

    f.write(
        f"Inference(ms): {avg_time*1000:.2f}\n"
    )

    f.write(
        f"FPS      : {fps:.2f}\n"
    )

    f.write(
        f"Model(KB): {model_size:.2f}\n"
    )



print("="*40)

print(
    f"TP       : {TP}"
)

print(
    f"FP       : {FP}"
)

print(
    f"FN       : {FN}"
)

print(
    f"Precision: {precision:.4f}"
)

print(
    f"Recall   : {recall:.4f}"
)

print(
    f"F1       : {f1:.4f}"
)

print(
    f"Inference: {avg_time*1000:.2f} ms"
)

print(
    f"FPS      : {fps:.2f}"
)

print(
    f"Model    : {model_size:.2f} KB"
)

print("="*40)