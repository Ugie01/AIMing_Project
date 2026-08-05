import os
import time
import cv2
import numpy as np
import tensorflow as tf

from train import load_image
from utils.dataset import YOLODataset

from configs.config import (
    INPUT_SIZE,
    GRID_SIZE
)


# =====================================================
# Path
# =====================================================
MODEL_PATH = (
    "models/quantized/PTQ_INT8/fomo_ptq_int8.tflite"
)
TEST_IMAGE_DIR = (
    "dataset/test/images"
)
TEST_LABEL_DIR = (
    "dataset/test/labels"
)
SAVE_DIR = (
    "test_results_quantized"
)
PRED_DIR = os.path.join(
    SAVE_DIR,
    "prediction"
)
HEAT_DIR = os.path.join(
    SAVE_DIR,
    "heatmap"
)
os.makedirs(
    PRED_DIR,
    exist_ok=True
)
os.makedirs(
    HEAT_DIR,
    exist_ok=True
)


# =====================================================
# Config
# =====================================================
CONF_THRESHOLD = 0.4


# =====================================================
# Dataset
# =====================================================
dataset = YOLODataset(
    TEST_IMAGE_DIR,
    TEST_LABEL_DIR
)
print(
    f"[Dataset] {len(dataset)} images loaded"
)



# =====================================================
# Load INT8 TFLite Model
# =====================================================
interpreter = tf.lite.Interpreter(
    model_path=MODEL_PATH
)
interpreter.allocate_tensors()
input_detail = (
    interpreter
    .get_input_details()[0]
)
output_detail = (
    interpreter
    .get_output_details()[0]
)
input_index = (
    input_detail["index"]
)
output_index = (
    output_detail["index"]
)
input_scale, input_zero = (
    input_detail["quantization"]
)
output_scale, output_zero = (
    output_detail["quantization"]
)
print(
    "Quantized Model Loaded"
)
print(
    "Input:",
    input_detail["dtype"],
    input_detail["shape"]
)
print(
    "Output:",
    output_detail["dtype"],
    output_detail["shape"]
)



# =====================================================
# Detection extraction
# =====================================================
def get_detection_points(
        prob_map,
        threshold=0.5
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
times = []

for idx in range(len(dataset)):
    sample = dataset[idx]
    image = load_image(
        sample["image_path"]
    )


    # -----------------------------
    # INT8 Input
    # -----------------------------
    input_data = (
        image[np.newaxis]
    )
    input_data = (
        input_data
        /
        input_scale
        +
        input_zero
    )
    input_data = np.clip(
        input_data,
        -128,
        127
    )
    input_data = (
        input_data
        .astype(
            np.int8
        )
    )
    start = time.perf_counter()
    interpreter.set_tensor(
        input_index,
        input_data
    )
    interpreter.invoke()
    output = interpreter.get_tensor(
        output_index
    )
    end = time.perf_counter()
    times.append(
        end-start
    )


    # -----------------------------
    # Dequantize Output
    # -----------------------------
    pred = (
        output.astype(
            np.float32
        )
        -
        output_zero
    )
    pred *= output_scale
    pred = pred[0]


    # -----------------------------
    # Prediction
    # -----------------------------
    pred_points = get_detection_points(
        pred[...,1],
        # threshold=0.5
        threshold=CONF_THRESHOLD
    )

    # -----------------------------
    # Ground Truth
    # -----------------------------
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


    # -----------------------------
    # Matching
    # -----------------------------
    matched_pred = set()

    for gt in gt_points:
        matched = False
        for i, pred_point in enumerate(pred_points):
            if i in matched_pred:
                continue

            distance = np.sqrt(
                (gt[0]-pred_point[0])**2
                +
                (gt[1]-pred_point[1])**2
            )

            if distance <= MATCH_DISTANCE:
                matched = True
                matched_pred.add(
                    i
                )
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


    # -----------------------------
    # Heatmap save
    # -----------------------------
    heat = (
        pred[...,1]
        *
        255
    ).clip(
        0,
        255
    ).astype(
        np.uint8
    )

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


    # -----------------------------
    # Prediction image save
    # -----------------------------
    rgb = (
        image
        *
        255
    ).astype(
        np.uint8
    )

    for x,y in pred_points:
        cv2.circle(
            rgb,
            (
                x,
                y
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
    times
)
fps = 1.0 / avg_time




# =====================================================
# Print
# =====================================================
print("="*40)
print(
    " Quantized Test Result"
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
print("="*40)



# =====================================================
# Save
# =====================================================
with open(
    os.path.join(
        SAVE_DIR,
        "test_metrics.txt"
    ),
    "w"
) as f:
    f.write(
        "========================================\n"
    )
    f.write(
        " Quantized Test Result\n"
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

print(
    "Quantized test result saved"
)