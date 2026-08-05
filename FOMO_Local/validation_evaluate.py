import os
import time
import cv2
import numpy as np
import tensorflow as tf

from train import load_image
from utils.dataset import YOLODataset

from configs.config import (
    INPUT_SIZE,
    GRID_SIZE,
    NUM_CLASSES
)


# =====================================================
# Path
# =====================================================
MODEL_PATH = (
    "checkpoints/best_fomo.keras"
)
VAL_IMAGE_DIR = (
    "dataset/validation/images"
)
VAL_LABEL_DIR = (
    "dataset/validation/labels"
)
SAVE_DIR = (
    "validation_results_float32"
)
os.makedirs(
    SAVE_DIR,
    exist_ok=True
)


# =====================================================
# Config
# =====================================================
CONF_THRESHOLD = 0.9


# =====================================================
# Load Model
# =====================================================
dataset = YOLODataset(
    VAL_IMAGE_DIR,
    VAL_LABEL_DIR
)
model = tf.keras.models.load_model(
    MODEL_PATH,
    compile=False
)
print(
    "Float32 Model Loaded"
)
print(
    f"Validation Images : {len(dataset)}"
)



# =====================================================
# Detection Point Extraction
# =====================================================
def get_detection_points(
        prob_map,
        # threshold=0.5
        threshold=CONF_THRESHOLD
):
    binary = (
        prob_map > threshold
    ).astype(
        np.uint8
    )

    num_labels, labels, stats, centroids = (
        cv2.connectedComponentsWithStats(
            binary,
            connectivity=8
        )
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



# =====================================================
# Evaluation
# =====================================================
MATCH_DISTANCE = 8
TP = 0
FP = 0
FN = 0
inference_times = []

for idx in range(
        len(dataset)
):
    sample = dataset[idx]
    image = load_image(
        sample["image_path"]
    )

    # -------------------------
    # Inference
    # -------------------------
    start = time.perf_counter()
    prediction = model.predict(
        image[np.newaxis],
        verbose=0
    )[0]
    end = time.perf_counter()
    inference_times.append(
        end-start
    )


    # -------------------------
    # Prediction points
    # -------------------------
    pred_points = get_detection_points(
        prediction[...,1],
        # threshold=0.5
        threshold=CONF_THRESHOLD
    )


    # -------------------------
    # Ground truth points
    # -------------------------
    gt_points = []

    for obj in sample["labels"]:
        x = int(
            obj["x"]
            *
            INPUT_SIZE[0]
        )
        y = int(
            obj["y"]
            *
            INPUT_SIZE[1]
        )
        gt_points.append(
            (
                x,
                y
            )
        )


    # -------------------------
    # Matching
    # -------------------------
    matched_pred = set()

    for gt in gt_points:
        matched = False
        for i, pred in enumerate(pred_points):
            if i in matched_pred:
                continue

            distance = np.sqrt(
                (gt[0]-pred[0])**2
                +
                (gt[1]-pred[1])**2
            )
            if distance <= MATCH_DISTANCE:
                matched = True
                matched_pred.add(i)
                break

        if matched:
            TP += 1
        else:
            FN += 1

    FP += (
        len(pred_points)
        -
        len(matched_pred)
    )



# =====================================================
# Metrics
# =====================================================
precision = TP / (
    TP + FP + 1e-8
)
recall = TP / (
    TP + FN + 1e-8
)
f1 = (
    2
    *
    precision
    *
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
avg_time = np.mean(
    inference_times
)
fps = (
    1.0
    /
    avg_time
)
model_size = (
    os.path.getsize(
        MODEL_PATH
    )
    /
    1024
)


# =====================================================
# Print
# =====================================================
print("="*40)
print(
    " Float32 Validation Result"
)
print("="*40)
print(
    f"Images     : {len(dataset)}"
)
print(
    f"TP         : {TP}"
)
print(
    f"FP         : {FP}"
)
print(
    f"FN         : {FN}"
)
print(
    f"Precision  : {precision:.4f}"
)
print(
    f"Recall     : {recall:.4f}"
)
print(
    f"F1 Score   : {f1:.4f}"
)
print(
    f"Inference  : {avg_time*1000:.2f} ms"
)
print(
    f"FPS        : {fps:.2f}"
)
print(
    f"Model Size : {model_size:.2f} KB"
)
print("="*40)


# =====================================================
# Save Result
# =====================================================
result_path = os.path.join(
    SAVE_DIR,
    "validation_metrics.txt"
)
with open(
        result_path,
        "w"
) as f:
    f.write(
        "========================================\n"
    )
    f.write(
        " Float32 Validation Result\n"
    )
    f.write(
        "========================================\n"
    )
    f.write(
        f"Images     : {len(dataset)}\n"
    )
    f.write(
        f"TP         : {TP}\n"
    )
    f.write(
        f"FP         : {FP}\n"
    )
    f.write(
        f"FN         : {FN}\n"
    )
    f.write(
        f"Precision  : {precision:.4f}\n"
    )
    f.write(
        f"Recall     : {recall:.4f}\n"
    )
    f.write(
        f"F1 Score   : {f1:.4f}\n"
    )
    f.write(
        f"Inference  : {avg_time*1000:.2f} ms\n"
    )
    f.write(
        f"FPS        : {fps:.2f}\n"
    )
    f.write(
        f"Model Size : {model_size:.2f} KB\n"
    )

print(
    "\nValidation result saved"
)
print(
    result_path
)
