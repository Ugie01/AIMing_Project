import os
import tensorflow as tf
import cv2
import numpy as np

from pathlib import Path
from configs.config import INPUT_SIZE


# =====================================================
# Configuration
# =====================================================
MODEL_PATH = (
    "checkpoints/best_fomo.keras"
)
OUTPUT_DIR = (
    "models/quantized/PTQ_INT8"
)
OUTPUT_MODEL_NAME = (
    "fomo_ptq_int8.tflite"
)
CALIB_IMAGE_DIR = (
    "dataset/train/images"
)

CALIBRATION_SIZE = 200



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
# Representative Dataset
# =====================================================
def representative_dataset():
    image_list = sorted(
        Path(CALIB_IMAGE_DIR).glob("*.jpg")
    )

    if len(image_list) == 0:
        image_list = sorted(
            Path(CALIB_IMAGE_DIR).glob("*.png")
        )
    print(
        f"Calibration Images : {len(image_list[:CALIBRATION_SIZE])}"
    )

    for image_path in image_list[:CALIBRATION_SIZE]:
        image = load_image(
            image_path
        )
        image = np.expand_dims(
            image,
            axis=0
        )
        yield [
            image
        ]



# =====================================================
# Tensor Information
# =====================================================
def get_tensor_info(
        model_path
):
    interpreter = tf.lite.Interpreter(
        model_path=model_path
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

    return (
        input_detail,
        output_detail
    )



# =====================================================
# Report
# =====================================================
def save_report(
        save_path,
        src_size,
        dst_size,
        input_detail,
        output_detail
):
    reduction = (
        1 -
        dst_size/src_size
    ) * 100

    with open(
        save_path,
        "w"
    ) as f:
        f.write(
            "========================================\n"
        )
        f.write(
            " PTQ INT8 Quantization Report\n"
        )
        f.write(
            "========================================\n\n"
        )
        f.write(
            "[Quantization Method]\n"
        )
        f.write(
            "Type : PTQ\n"
        )
        f.write(
            "Mode : Full Integer Quantization\n\n"
        )
        f.write(
            "[Source Model]\n"
        )
        f.write(
            f"Path : {MODEL_PATH}\n"
        )
        f.write(
            f"Size : {src_size:.2f} KB\n\n"
        )
        f.write(
            "[Calibration Dataset]\n"
        )
        f.write(
            f"Path : {CALIB_IMAGE_DIR}\n"
        )
        f.write(
            f"Images : {CALIBRATION_SIZE}\n\n"
        )
        f.write(
            "[Input Tensor]\n"
        )
        f.write(
            f"Dtype : {input_detail['dtype']}\n"
        )
        f.write(
            f"Shape : {input_detail['shape']}\n"
        )
        f.write(
            f"Scale : {input_detail['quantization'][0]}\n"
        )
        f.write(
            f"Zero Point : {input_detail['quantization'][1]}\n\n"
        )
        f.write(
            "[Output Tensor]\n"
        )
        f.write(
            f"Dtype : {output_detail['dtype']}\n"
        )
        f.write(
            f"Shape : {output_detail['shape']}\n"
        )
        f.write(
            f"Scale : {output_detail['quantization'][0]}\n"
        )
        f.write(
            f"Zero Point : {output_detail['quantization'][1]}\n\n"
        )
        f.write(
            "[Generated Model]\n"
        )
        f.write(
            f"Path : {OUTPUT_MODEL_NAME}\n"
        )
        f.write(
            f"Size : {dst_size:.2f} KB\n\n"
        )
        f.write(
            "[Compression]\n"
        )
        f.write(
            f"Reduction : {reduction:.2f}%\n"
        )
        f.write(
            "\n========================================\n"
        )



# =====================================================
# PTQ INT8 Conversion
# =====================================================
def convert_ptq_int8():
    print(
        "Loading float32 model"
    )
    model = tf.keras.models.load_model(
        MODEL_PATH,
        compile=False
    )
    converter = (
        tf.lite.TFLiteConverter
        .from_keras_model(model)
    )
    converter.optimizations = [
        tf.lite.Optimize.DEFAULT
    ]
    converter.representative_dataset = (
        representative_dataset
    )
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8
    ]
    converter.inference_input_type = (
        tf.int8
    )
    converter.inference_output_type = (
        tf.int8
    )
    print(
        "Converting PTQ INT8..."
    )

    tflite_model = (
        converter.convert()
    )
    os.makedirs(
        OUTPUT_DIR,
        exist_ok=True
    )
    model_path = os.path.join(
        OUTPUT_DIR,
        OUTPUT_MODEL_NAME
    )

    with open(
        model_path,
        "wb"
    ) as f:
        f.write(
            tflite_model
        )


    # -------------------------------
    # Size
    # -------------------------------
    src_size = (
        os.path.getsize(
            MODEL_PATH
        )
        /
        1024
    )
    dst_size = (
        os.path.getsize(
            model_path
        )
        /
        1024
    )


    # -------------------------------
    # Tensor info
    # -------------------------------
    input_detail, output_detail = (
        get_tensor_info(
            model_path
        )
    )

    # -------------------------------
    # Report
    # -------------------------------
    report_path = os.path.join(
        OUTPUT_DIR,
        "quantization_report.txt"
    )
    save_report(
        report_path,
        src_size,
        dst_size,
        input_detail,
        output_detail
    )
    print(
        "========================================"
    )
    print(
        "PTQ INT8 Finished"
    )
    print(
        f"Original : {src_size:.2f} KB"
    )
    print(
        f"INT8     : {dst_size:.2f} KB"
    )
    print(
        f"Report   : {report_path}"
    )
    print(
        "========================================"
    )




# =====================================================
# Main
# =====================================================
if __name__ == "__main__":
    convert_ptq_int8()