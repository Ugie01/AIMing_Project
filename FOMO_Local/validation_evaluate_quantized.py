import os
import time
import cv2
import numpy as np
import tensorflow as tf

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
VAL_IMAGE_DIR = (
    "dataset/validation/images"
)
VAL_LABEL_DIR = (
    "dataset/validation/labels"
)
SAVE_DIR = (
    "validation_results_quantized"
)
os.makedirs(
    SAVE_DIR,
    exist_ok=True
)


# =====================================================
# Config
# =====================================================
MATCH_DISTANCE = 8
# CONF_THRESHOLD = 0.5
CONF_THRESHOLD = 0.4


# =====================================================
# Image Loader
# =====================================================
def load_image(path):
    image = cv2.imread(
        str(path)
    )
    if image is None:
        raise ValueError(
            f"Image load failed : {path}"
        )
    image = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2RGB
    )
    image = cv2.resize(
        image,
        INPUT_SIZE
    )
    image = image.astype(
        np.float32
    )
    image /= 255.0


    return image


# =====================================================
# Dataset
# =====================================================
dataset = YOLODataset(
    VAL_IMAGE_DIR,
    VAL_LABEL_DIR
)
print(
    f"Validation Images : {len(dataset)}"
)


# =====================================================
# Load INT8 Model
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
input_scale, input_zero = (
    input_detail["quantization"]
)
output_scale, output_zero = (
    output_detail["quantization"]
)

print("Quantized Model Loaded")
print(
    "Input:",
    input_detail["dtype"],
    input_detail["shape"]
)
print(
    "Input scale:",
    input_scale,
    "zero:",
    input_zero
)
print(
    "Output:",
    output_detail["dtype"],
    output_detail["shape"]
)
print(
    "Output scale:",
    output_scale,
    "zero:",
    output_zero
)


# =====================================================
# Detection
# =====================================================
def get_detection_points(
        prob_map,
        # threshold=0.5
        threshold = CONF_THRESHOLD
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
    points=[]

    for i in range(1,num_labels):
        cx = centroids[i][0]
        cy = centroids[i][1]
        px = int(
            (cx+0.5)
            *
            INPUT_SIZE[0]
            /
            GRID_SIZE[0]
        )
        py = int(
            (cy+0.5)
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
TP=0
FP=0
FN=0
times=[]
debug=True

for idx in range(
        len(dataset)
):
    sample = dataset[idx]
    image = load_image(
        sample["image_path"]
    )

    # -------------------------
    # Quantize Input
    # -------------------------
    input_data = (
        image / input_scale
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
    input_data = np.expand_dims(
        input_data,
        axis=0
    )


    # -------------------------
    # Inference
    # -------------------------
    start=time.perf_counter()
    interpreter.set_tensor(
        input_detail["index"],
        input_data
    )
    interpreter.invoke()
    output = interpreter.get_tensor(
        output_detail["index"]
    )
    end=time.perf_counter()
    times.append(
        end-start
    )


    # -------------------------
    # Dequantize Output
    # -------------------------
    output_float = (
        output.astype(
            np.float32
        )
        -
        output_zero
    )
    output_float *= (
        output_scale
    )
    pred = output_float[0]

    if debug and idx == 0:
        print(
            "Output range:",
            pred.min(),
            pred.max()
        )
        print(
            "Object channel range:",
            pred[...,1].min(),
            pred[...,1].max()
        )


    # -------------------------
    # Prediction
    # -------------------------
    pred_points = get_detection_points(
        pred[...,1],
        CONF_THRESHOLD
    )


    # -------------------------
    # Ground Truth
    # ------------------------
    gt_points=[]

    for obj in sample["labels"]:
        x=int(
            obj["x"]
            *
            INPUT_SIZE[0]
        )
        y=int(
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
    matched=set()

    for gt in gt_points:
        found=False

        for i,p in enumerate(pred_points):
            if i in matched:
                continue

            dist=np.sqrt(
                (gt[0]-p[0])**2
                +
                (gt[1]-p[1])**2
            )

            if dist <= MATCH_DISTANCE:
                found=True
                matched.add(i)
                break

        if found:
            TP+=1
        else:
            FN+=1

    FP += (
        len(pred_points)
        -
        len(matched)
    )



# =====================================================
# Metrics
# =====================================================
precision = TP/(TP+FP+1e-8)
recall = TP/(TP+FN+1e-8)
f1 = (
    2*precision*recall
    /
    (
        precision+recall+1e-8
    )
)
avg_time=np.mean(times)
fps=1/avg_time


# =====================================================
# Result
# =====================================================
print("="*40)
print(
    " INT8 Validation Result"
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

with open(
    os.path.join(
        SAVE_DIR,
        "validation_metrics.txt"
    ),
    "w"
) as f:
    f.write(
        f"TP : {TP}\n"
    )
    f.write(
        f"FP : {FP}\n"
    )
    f.write(
        f"FN : {FN}\n"
    )
    f.write(
        f"Precision : {precision:.4f}\n"
    )
    f.write(
        f"Recall : {recall:.4f}\n"
    )
    f.write(
        f"F1 : {f1:.4f}\n"
    )
    f.write(
        f"FPS : {fps:.2f}\n"
    )

print(
    "Saved"
)