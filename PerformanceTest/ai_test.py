# -*- coding: utf-8 -*-
"""
AI 검출 성능 시험 - 합성 이미지 자동 평가
============================================================
drone/  : 누끼 딴 드론 PNG (투명 배경).  예: A.png, B.png ...
bg/     : 같은 ID의 배경 이미지.         예: A.png(또는 .jpg), B.png ...

흐름:
  drone + bg 랜덤 위치 합성 -> 중앙 1:1 크롭 -> 96x96 -> 모델 추론
  -> 검출 결과를 '정답 위치'와 비교해서 적중/오검출/미검출 자동 판정

poc_test.py 와 동일한 합성 규칙(크기, 스폰 범위)을 사용하므로
실사 시험 결과와 직접 비교할 수 있습니다.

설치:
    pip install numpy opencv-python tensorflow
"""

import os

# TensorFlow 로그/oneDNN 메시지 억제 (반드시 tensorflow import 보다 먼저)
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")

import csv
import math
import random
import sys
import time
import warnings
from datetime import datetime

import cv2
import numpy as np

warnings.filterwarnings("ignore", message=r".*tf\.lite\.Interpreter is deprecated.*")


# ============================================================
# [1] 설정  <- 여기만 수정하세요
# ============================================================
MODEL_PATH = r"ei-detect_drone_ver2_exp_rgb-object-detection-tensorflow-lite-int8-quantized-model.3.lite"
DRONE_DIR  = r"drone"
BG_DIR     = r"bg"
OUT_DIR    = r"AI_Test_Result"

TRIALS_PER_IMAGE = 10          # 이미지 쌍당 합성 횟수 (10쌍 x 10회 = 100장)
RANDOM_SEED      = 42          # 재현성. None 이면 매번 다른 결과

# --- 합성 규칙 (poc_test.py 와 동일하게 유지) ---
CANVAS_W, CANVAS_H = 1920, 1080   # 실사 시험에 쓴 모니터 해상도
DRONE_TARGET_WIDTH = 135           # 드론 가로폭(px). None이면 원본 크기
SPAWN_HALF_W = 291                # 화면 중심 기준 스폰 허용 범위
SPAWN_HALF_H = 203

# --- 판정 기준 ---
THRESHOLD     = 0.5            # 검출 임계값 (모델 기본 0.5)
MERGE_MODE    = "cube"         # "cube" = 인접 셀 병합(EI 방식) / "cell" = 셀 단위
HIT_RADIUS_96 = 12.0           # 96x96 기준, 정답에서 이 거리 안이면 '적중' (셀 1.5개)

# --- 저장 옵션 ---
SAVE_RAW       = True          # 합성 원본 + 정답/검출 표시 이미지
SAVE_96        = True          # 96x96 결과 (5배 확대)
SAVE_GRID      = "miss"        # "always" | "miss"(미적중일 때만) | "never"
SAVE_COMPOSITE = False         # 주석 없는 순수 합성 이미지도 따로 저장

EXTS   = (".png", ".jpg", ".jpeg", ".bmp", ".webp")
LABELS = ["background", "drone"]

# 판정 코드 (poc_test.py 와 동일한 체계)
CODE_HIT    = 1   # 적중   : 정답 위치에서 검출
CODE_OFF    = 2   # 미적중 : 검출은 했으나 위치가 벗어남
CODE_MISS   = 3   # 미검출 : 아무것도 검출 못함
CODE_FALSE  = 4   # 오검출 : 정답 위치는 놓치고 엉뚱한 곳만 검출
RESULT_LABELS = {CODE_HIT: "적중", CODE_OFF: "미적중",
                 CODE_MISS: "미검출", CODE_FALSE: "오검출"}


# ============================================================
# [2] 모델 백엔드
# ============================================================
def _find_interpreter():
    """설치된 패키지에 따라 Interpreter 클래스를 찾아옴."""
    tried = []
    try:
        from tflite_runtime.interpreter import Interpreter
        return Interpreter, "tflite-runtime"
    except Exception as e:
        tried.append(f"  - tflite_runtime : {type(e).__name__}")
    try:
        from ai_edge_litert.interpreter import Interpreter
        return Interpreter, "ai-edge-litert"
    except Exception as e:
        tried.append(f"  - ai_edge_litert : {type(e).__name__}")
    try:
        # 주의: 'from tensorflow.lite import Interpreter' 는 실패합니다.
        # tensorflow/lite/ 실제 폴더와 tf.lite API 별칭이 다른 객체이기 때문.
        import tensorflow as tf
        return tf.lite.Interpreter, f"tensorflow {tf.__version__}"
    except Exception as e:
        tried.append(f"  - tensorflow     : {type(e).__name__}")

    raise ImportError(
        "TFLite Interpreter를 찾을 수 없습니다.\n" + "\n".join(tried) +
        "\n\n  pip install tensorflow   (Windows 권장)"
    )


class TFLiteBackend:
    def __init__(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f"모델 파일이 없습니다: {path}")
        Interpreter, src = _find_interpreter()
        print(f"백엔드: TensorFlow Lite  (via {src})")
        self.itp = Interpreter(model_path=path)
        self.itp.allocate_tensors()
        self.inp = self.itp.get_input_details()[0]
        self.out = self.itp.get_output_details()[0]
        print(f"  입력 {[int(v) for v in self.inp['shape']]} "
              f"{np.dtype(self.inp['dtype']).name} quant={self.inp['quantization']}")
        print(f"  출력 {[int(v) for v in self.out['shape']]} "
              f"{np.dtype(self.out['dtype']).name} quant={self.out['quantization']}")

    @property
    def input_shape(self):
        return [int(v) for v in self.inp["shape"]]

    @property
    def output_shape(self):
        return [int(v) for v in self.out["shape"]]

    def invoke(self, x_int8):
        """x_int8 : (H, W, 3) int8 값 -> (OH, OW, C) 확률"""
        if self.inp["dtype"] == np.float32:
            x = ((x_int8.astype(np.float32) + 128.0) / 255.0)[None, ...]
        else:
            x = x_int8.astype(np.int8)[None, ...]
        self.itp.set_tensor(self.inp["index"], x)
        self.itp.invoke()
        raw = self.itp.get_tensor(self.out["index"])[0]
        if self.out["dtype"] == np.int8:
            scale, zp = self.out["quantization"]
            return (raw.astype(np.float32) - zp) * scale
        return raw.astype(np.float32)


def load_backend(path):
    if path.lower().endswith(".cpp"):
        from eon_runtime import EonInterpreter
        print("백엔드: EON .cpp 파서 (NumPy)")
        return EonInterpreter(path)
    return TFLiteBackend(path)


# ============================================================
# [3] 이미지 쌍 로드 & 합성 (poc_test.py 규칙과 동일)
# ============================================================
def load_pairs():
    """drone/와 bg/에서 같은 stem을 가진 쌍을 찾아 (id, drone_path, bg_path) 반환."""
    def stems(d):
        if not os.path.isdir(d):
            print(f"[오류] 폴더가 없습니다: {d}")
            sys.exit(1)
        out = {}
        for f in sorted(os.listdir(d)):
            stem, ext = os.path.splitext(f)
            if ext.lower() in EXTS:
                out[stem] = os.path.join(d, f)
        return out

    drones, bgs = stems(DRONE_DIR), stems(BG_DIR)
    ids = sorted(set(drones) & set(bgs))
    missing = sorted(set(drones) ^ set(bgs))
    if missing:
        print(f"[경고] 짝이 없는 ID는 제외됨: {missing}")
    if not ids:
        print("[오류] drone/와 bg/에서 매칭되는 이미지 쌍이 없습니다.")
        sys.exit(1)
    return [(i, drones[i], bgs[i]) for i in ids]


def alpha_blend(canvas, fg_bgra, cx, cy):
    """
    투명 배경 드론(fg_bgra)을 canvas의 (cx, cy) 중심에 알파 합성.
    화면 밖으로 나가는 부분은 잘라냅니다.
    """
    fh, fw = fg_bgra.shape[:2]
    x1, y1 = cx - fw // 2, cy - fh // 2
    x2, y2 = x1 + fw, y1 + fh

    # 캔버스 밖 잘라내기
    sx1, sy1 = max(0, -x1), max(0, -y1)
    dx1, dy1 = max(0, x1), max(0, y1)
    dx2, dy2 = min(canvas.shape[1], x2), min(canvas.shape[0], y2)
    if dx2 <= dx1 or dy2 <= dy1:
        return canvas

    fg = fg_bgra[sy1:sy1 + (dy2 - dy1), sx1:sx1 + (dx2 - dx1)]
    roi = canvas[dy1:dy2, dx1:dx2].astype(np.float32)

    if fg.shape[2] == 4:
        a = (fg[:, :, 3:4].astype(np.float32)) / 255.0
        blended = fg[:, :, :3].astype(np.float32) * a + roi * (1.0 - a)
    else:
        blended = fg[:, :, :3].astype(np.float32)

    canvas[dy1:dy2, dx1:dx2] = np.clip(blended, 0, 255).astype(np.uint8)
    return canvas


def compose(bg_img, drone_img, rng):
    """
    배경을 캔버스 크기로 맞추고 드론을 스폰 범위 내 랜덤 위치에 합성.
    반환: (합성 이미지, 드론 중심 (cx, cy), 드론 크기 (w, h))
    """
    canvas = cv2.resize(bg_img, (CANVAS_W, CANVAS_H), interpolation=cv2.INTER_AREA)
    if canvas.ndim == 2:
        canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    canvas = canvas[:, :, :3].copy()

    d = drone_img
    if DRONE_TARGET_WIDTH is not None and d.shape[1] != DRONE_TARGET_WIDTH:
        ratio = DRONE_TARGET_WIDTH / d.shape[1]
        nh = max(1, int(round(d.shape[0] * ratio)))
        d = cv2.resize(d, (DRONE_TARGET_WIDTH, nh), interpolation=cv2.INTER_AREA)

    dh, dw = d.shape[:2]
    cx0, cy0 = CANVAS_W // 2, CANVAS_H // 2
    half_w = min(SPAWN_HALF_W, CANVAS_W // 2 - dw // 2 - 5)
    half_h = min(SPAWN_HALF_H, CANVAS_H // 2 - dh // 2 - 5)
    cx = cx0 + rng.randint(-half_w, half_w)
    cy = cy0 + rng.randint(-half_h, half_h)

    return alpha_blend(canvas, d, cx, cy), (cx, cy), (dw, dh)


# ============================================================
# [4] 전처리 (Edge Impulse FIT_SHORTEST 방식과 동일)
# ============================================================
def preprocess(img_bgr, in_w, in_h):
    """중앙 1:1 크롭 -> 리사이즈 -> RGB -> int8 (= 픽셀 - 128)"""
    h, w = img_bgr.shape[:2]
    size = min(h, w)
    x0, y0 = (w - size) // 2, (h - size) // 2
    crop = img_bgr[y0:y0 + size, x0:x0 + size]
    img96_bgr = cv2.resize(crop, (in_w, in_h), interpolation=cv2.INTER_AREA)
    img96_rgb = cv2.cvtColor(img96_bgr, cv2.COLOR_BGR2RGB)
    return img96_rgb.astype(np.int32) - 128, img96_bgr, x0, y0, size


def raw_to_96(pt, x0, y0, crop_size, in_w):
    """원본(합성 이미지) 좌표 -> 96x96 좌표"""
    s = in_w / crop_size
    return ((pt[0] - x0) * s, (pt[1] - y0) * s)


def to_raw(box, x0, y0, crop_size, in_w):
    """96x96 좌표 -> 원본 좌표"""
    s = crop_size / in_w
    x1, y1, x2, y2 = box
    return (x1 * s + x0, y1 * s + y0, x2 * s + x0, y2 * s + y0)


# ============================================================
# [5] FOMO 후처리
# ============================================================
def _overlap(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return (ax <= bx + bw) and (ax + aw >= bx) and (ay <= by + bh) and (ay + ah >= by)


def postprocess(prob, cell, threshold=THRESHOLD, mode=MERGE_MODE):
    """12x12 확률맵 -> 96x96 기준 박스 리스트."""
    out_h, out_w, n_cls = prob.shape
    dets = []

    for cls in range(1, n_cls):
        hits = [(gx, gy, float(prob[gy, gx, cls]))
                for gy in range(out_h) for gx in range(out_w)
                if prob[gy, gx, cls] >= threshold]

        if mode == "cell":
            cubes = [{"x": gx, "y": gy, "w": 1, "h": 1, "score": v, "cells": [(gx, gy)]}
                     for gx, gy, v in hits]
        else:
            cubes = []
            for gx, gy, v in hits:
                merged = False
                for c in cubes:
                    if _overlap((c["x"], c["y"], c["w"], c["h"]), (gx, gy, 1, 1)):
                        ex = max(c["x"] + c["w"], gx + 1)
                        ey = max(c["y"] + c["h"], gy + 1)
                        c["x"], c["y"] = min(c["x"], gx), min(c["y"], gy)
                        c["w"], c["h"] = ex - c["x"], ey - c["y"]
                        c["score"] = max(c["score"], v)
                        c["cells"].append((gx, gy))
                        merged = True
                        break
                if not merged:
                    cubes.append({"x": gx, "y": gy, "w": 1, "h": 1,
                                  "score": v, "cells": [(gx, gy)]})

            changed = True
            while changed:
                changed = False
                for i in range(len(cubes)):
                    for j in range(i + 1, len(cubes)):
                        a, b = cubes[i], cubes[j]
                        if _overlap((a["x"], a["y"], a["w"], a["h"]),
                                    (b["x"], b["y"], b["w"], b["h"])):
                            ex = max(a["x"] + a["w"], b["x"] + b["w"])
                            ey = max(a["y"] + a["h"], b["y"] + b["h"])
                            a["x"], a["y"] = min(a["x"], b["x"]), min(a["y"], b["y"])
                            a["w"], a["h"] = ex - a["x"], ey - a["y"]
                            a["score"] = max(a["score"], b["score"])
                            a["cells"] += b["cells"]
                            cubes.pop(j)
                            changed = True
                            break
                    if changed:
                        break

        for c in cubes:
            x1, y1 = c["x"] * cell, c["y"] * cell
            x2, y2 = (c["x"] + c["w"]) * cell, (c["y"] + c["h"]) * cell
            dets.append({
                "box": (x1, y1, x2, y2),
                "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                "score": c["score"],
                "label": LABELS[cls] if cls < len(LABELS) else str(cls),
                "cells": sorted(set(c["cells"])),
            })

    dets.sort(key=lambda d: -d["score"])
    return dets


# ============================================================
# [6] 정답 대조 판정
# ============================================================
def judge(dets, gt_96, gt_in_crop):
    """
    검출 결과를 정답 위치와 비교해 판정.
    반환: (판정코드, 가장가까운검출 index 또는 None, 거리(96기준))
    """
    if not gt_in_crop:
        # 드론이 크롭 영역 밖 -> 모델이 볼 수 없었음. 판정에서 제외 대상
        return None, None, None

    if not dets:
        return CODE_MISS, None, None

    # 정답에서 가장 가까운 검출 찾기
    best_i, best_d = None, float("inf")
    for i, d in enumerate(dets):
        dist = math.hypot(d["center"][0] - gt_96[0], d["center"][1] - gt_96[1])
        if dist < best_d:
            best_i, best_d = i, dist

    if best_d <= HIT_RADIUS_96:
        return CODE_HIT, best_i, best_d
    # 검출은 있는데 전부 정답에서 멀다
    if len(dets) == 1:
        return CODE_OFF, best_i, best_d
    return CODE_FALSE, best_i, best_d


# ============================================================
# [7] 시각화
# ============================================================
def draw_result(img, dets, gt, hit_idx, thick=1, fs=0.32, gt_r=10):
    """
    노란 원 = 정답 위치(GT)
    초록 박스 + 빨간 십자 = 모델 검출
    적중한 검출은 굵게, 정답과 선으로 연결
    """
    vis = img.copy()
    if gt is not None:
        gx, gy = int(round(gt[0])), int(round(gt[1]))
        cv2.circle(vis, (gx, gy), gt_r, (0, 255, 255), thick)
        cv2.drawMarker(vis, (gx, gy), (0, 255, 255), cv2.MARKER_TILTED_CROSS,
                       gt_r, thick)

    for i, d in enumerate(dets):
        x1, y1, x2, y2 = [int(round(v)) for v in d["box"]]
        cx, cy = [int(round(v)) for v in d["center"]]
        w = thick + 1 if i == hit_idx else thick
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), w)
        cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS,
                       max(8, thick * 7), thick)
        cv2.putText(vis, f"{d['score']:.2f}", (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 255, 255), thick, cv2.LINE_AA)
        if i == hit_idx and gt is not None:
            cv2.line(vis, (int(round(gt[0])), int(round(gt[1]))), (cx, cy),
                     (255, 0, 255), thick)
    return vis


def draw_grid(img96, prob, cell, cls=1):
    """12x12 셀별 확률 히트맵 (미검출 원인 분석용)."""
    up = 6
    h, w = img96.shape[:2]
    vis = cv2.resize(img96, (w * up, h * up), interpolation=cv2.INTER_NEAREST)
    heat = cv2.resize((prob[:, :, cls] * 255).clip(0, 255).astype(np.uint8),
                      (w * up, h * up), interpolation=cv2.INTER_NEAREST)
    vis = cv2.addWeighted(vis, 0.6, cv2.applyColorMap(heat, cv2.COLORMAP_JET), 0.4, 0)

    step = cell * up
    for i in range(prob.shape[1] + 1):
        cv2.line(vis, (i * step, 0), (i * step, h * up), (90, 90, 90), 1)
        cv2.line(vis, (0, i * step), (w * up, i * step), (90, 90, 90), 1)
    for gy in range(prob.shape[0]):
        for gx in range(prob.shape[1]):
            v = prob[gy, gx, cls]
            if v >= 0.05:
                cv2.putText(vis, f"{v:.2f}", (gx * step + 2, gy * step + step - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1,
                            cv2.LINE_AA)
    return vis


# ============================================================
# [8] 메인
# ============================================================
def main():
    img_dir = os.path.join(OUT_DIR, "images")
    os.makedirs(img_dir, exist_ok=True)
    if SAVE_COMPOSITE:
        comp_dir = os.path.join(OUT_DIR, "composite")
        os.makedirs(comp_dir, exist_ok=True)

    rng = random.Random(RANDOM_SEED)

    # --- 모델 ---
    print("모델 로딩 중...")
    net = load_backend(MODEL_PATH)
    _, IN_H, IN_W, _ = net.input_shape
    _, OUT_H, OUT_W, _ = net.output_shape
    CELL = IN_W // OUT_W
    print(f"입력 {IN_W}x{IN_H} / 출력 그리드 {OUT_W}x{OUT_H} / 셀 {CELL}px\n")

    # --- 이미지 쌍 ---
    pairs = load_pairs()
    print(f"이미지 쌍 {len(pairs)}개: {', '.join(i for i, _, _ in pairs)}")

    # 프리로드 (드론은 알파 채널 유지)
    cache = {}
    for image_id, dp, bp in pairs:
        d = cv2.imread(dp, cv2.IMREAD_UNCHANGED)
        b = cv2.imread(bp, cv2.IMREAD_COLOR)
        if d is None or b is None:
            print(f"[경고] 읽기 실패로 제외: {image_id}")
            continue
        if d.ndim == 2:
            d = cv2.cvtColor(d, cv2.COLOR_GRAY2BGR)
        cache[image_id] = (d, b)
    pairs = [p for p in pairs if p[0] in cache]

    # --- 시행 큐 (poc_test.py 와 동일하게 셔플) ---
    queue = []
    for image_id, dp, bp in pairs:
        queue += [image_id] * TRIALS_PER_IMAGE
    rng.shuffle(queue)
    total = len(queue)

    # 크롭 범위 미리 계산 (캔버스 크기가 고정이므로 동일)
    crop_size = min(CANVAS_W, CANVAS_H)
    crop_x0 = (CANVAS_W - crop_size) // 2
    crop_y0 = (CANVAS_H - crop_size) // 2
    scale = crop_size / IN_W
    print(f"캔버스 {CANVAS_W}x{CANVAS_H} -> 크롭 x={crop_x0} y={crop_y0} "
          f"size={crop_size} -> {IN_W}x{IN_H}")
    print(f"배율: 96 기준 1px = 합성이미지 {scale:.3f}px")
    print(f"적중 판정 반경: 96 기준 {HIT_RADIUS_96}px "
          f"(= 합성이미지 {HIT_RADIUS_96 * scale:.1f}px)\n")
    print(f"총 {total}회 시행 시작")
    print("-" * 78)

    summary_rows, det_rows = [], []
    counts = {c: 0 for c in RESULT_LABELS}
    n_skip = 0
    dists = []
    t_start = time.time()

    for trial, image_id in enumerate(queue, 1):
        drone_img, bg_img = cache[image_id]

        # --- 합성 ---
        comp, (gx, gy), (dw, dh) = compose(bg_img, drone_img, rng)

        # --- 전처리 & 추론 ---
        x, img96, x0, y0, csz = preprocess(comp, IN_W, IN_H)
        t0 = time.time()
        prob = net.invoke(x)
        ms = (time.time() - t0) * 1000

        dets = postprocess(prob, CELL)
        max_p = float(prob[:, :, 1].max())

        # --- 정답 좌표 변환 & 판정 ---
        gt96 = raw_to_96((gx, gy), x0, y0, csz, IN_W)
        gt_in_crop = (0 <= gt96[0] < IN_W) and (0 <= gt96[1] < IN_H)
        code, hit_i, dist96 = judge(dets, gt96, gt_in_crop)

        stem = f"{trial:04d}_{image_id}"
        if code is None:
            n_skip += 1
            label = "범위밖"
        else:
            counts[code] += 1
            label = RESULT_LABELS[code]
            if code == CODE_HIT and dist96 is not None:
                dists.append(dist96)

        mark = {CODE_HIT: "O", CODE_OFF: "~", CODE_MISS: ".",
                CODE_FALSE: "X", None: "-"}[code]
        print(f"[{trial:>4}/{total}] {mark} {image_id:<10} {label:<6} "
              f"검출 {len(dets)}개  최대 {max_p:.3f}  "
              + (f"오차 {dist96:.1f}px  " if dist96 is not None else "            ")
              + f"{ms:>5.1f}ms")

        # --- CSV 행 ---
        summary_rows.append({
            "trial": trial, "image_id": image_id,
            "gt_x_raw": gx, "gt_y_raw": gy,
            "gt_x_96": round(gt96[0], 2), "gt_y_96": round(gt96[1], 2),
            "drone_w": dw, "drone_h": dh,
            "gt_in_crop": int(gt_in_crop),
            "num_detections": len(dets),
            "max_prob": round(max_p, 4),
            "result_code": "" if code is None else code,
            "result_label": label,
            "dist_96": "" if dist96 is None else round(dist96, 2),
            "dist_raw": "" if dist96 is None else round(dist96 * scale, 1),
            "infer_ms": round(ms, 2),
        })

        for k, d in enumerate(dets):
            r = to_raw(d["box"], x0, y0, csz, IN_W)
            det_rows.append({
                "trial": trial, "image_id": image_id, "det_idx": k,
                "is_matched": int(k == hit_i and code == CODE_HIT),
                "score": round(d["score"], 4),
                "cx_96": round(d["center"][0], 1), "cy_96": round(d["center"][1], 1),
                "x1_96": d["box"][0], "y1_96": d["box"][1],
                "x2_96": d["box"][2], "y2_96": d["box"][3],
                "cx_raw": round((r[0] + r[2]) / 2, 1),
                "cy_raw": round((r[1] + r[3]) / 2, 1),
                "x1_raw": round(r[0], 1), "y1_raw": round(r[1], 1),
                "x2_raw": round(r[2], 1), "y2_raw": round(r[3], 1),
                "cells": " ".join(f"({a},{b})" for a, b in d["cells"]),
            })

        # --- 이미지 저장 ---
        if SAVE_COMPOSITE:
            cv2.imwrite(os.path.join(comp_dir, f"{stem}.png"), comp)

        if SAVE_96:
            v = draw_result(img96, dets, gt96, hit_i, thick=1, fs=0.30, gt_r=6)
            v = cv2.resize(v, (IN_W * 5, IN_H * 5), interpolation=cv2.INTER_NEAREST)
            cv2.putText(v, f"{label}  max={max_p:.2f}", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(img_dir, f"{stem}_96.png"), v)

        if SAVE_RAW:
            dr = []
            for d in dets:
                r = to_raw(d["box"], x0, y0, csz, IN_W)
                dr.append({**d, "box": r,
                           "center": ((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)})
            v = draw_result(comp, dr, (gx, gy), hit_i, thick=2, fs=0.7, gt_r=30)
            cv2.rectangle(v, (crop_x0, crop_y0),
                          (crop_x0 + crop_size, crop_y0 + crop_size), (255, 0, 0), 2)
            cv2.putText(v, f"[{trial}] {image_id}  {label}  max={max_p:.2f}",
                        (crop_x0 + 10, crop_y0 + 34), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(img_dir, f"{stem}_raw.png"), v)

        want_grid = (SAVE_GRID == "always") or \
                    (SAVE_GRID == "miss" and code != CODE_HIT)
        if want_grid:
            cv2.imwrite(os.path.join(img_dir, f"{stem}_grid.png"),
                        draw_grid(img96, prob, CELL))

    # --- CSV 저장 ---
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sp = os.path.join(OUT_DIR, f"summary_{stamp}.csv")
    with open(sp, "w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        wr.writeheader()
        wr.writerows(summary_rows)

    dp_ = os.path.join(OUT_DIR, f"detections_{stamp}.csv")
    det_fields = ["trial", "image_id", "det_idx", "is_matched", "score",
                  "cx_96", "cy_96", "x1_96", "y1_96", "x2_96", "y2_96",
                  "cx_raw", "cy_raw", "x1_raw", "y1_raw", "x2_raw", "y2_raw", "cells"]
    with open(dp_, "w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=det_fields)
        wr.writeheader()
        wr.writerows(det_rows)

    # --- 요약 ---
    n = sum(counts.values())
    elapsed = time.time() - t_start
    print("-" * 78)
    print(f"총 시행: {total}  (판정 {n}, 범위밖 제외 {n_skip})")
    if n:
        for code in (CODE_HIT, CODE_OFF, CODE_MISS, CODE_FALSE):
            c = counts[code]
            print(f"  {RESULT_LABELS[code]:4s}: {c:4d}  ({c / n * 100:5.1f}%)")
        print(f"\n검출률(적중+미적중+오검출): "
              f"{(n - counts[CODE_MISS]) / n * 100:.1f}%")
        print(f"적중률(정답 위치 검출)    : {counts[CODE_HIT] / n * 100:.1f}%")
        if dists:
            ds = sorted(dists)
            print(f"\n중심 오차 (적중 {len(ds)}건, 96 기준):")
            print(f"  중앙값 {ds[len(ds)//2]:.2f}px / 평균 {sum(ds)/len(ds):.2f}px "
                  f"/ 최대 {ds[-1]:.2f}px")
            print(f"  합성이미지 환산: 중앙값 {ds[len(ds)//2]*scale:.1f}px "
                  f"/ 평균 {sum(ds)/len(ds)*scale:.1f}px")

        print("\n이미지별 적중률:")
        for image_id, _, _ in pairs:
            sub = [r for r in summary_rows
                   if r["image_id"] == image_id and r["result_code"] != ""]
            if not sub:
                continue
            h = sum(1 for r in sub if r["result_code"] == CODE_HIT)
            bar = "#" * int(h / len(sub) * 20)
            print(f"  {image_id:<12} {h:>3}/{len(sub):<3} "
                  f"({h/len(sub)*100:5.1f}%) {bar}")

    print(f"\n소요시간: {elapsed:.1f}초 (장당 {elapsed/max(1,total)*1000:.1f}ms)")
    print(f"\n저장 완료")
    print(f"  {sp}")
    print(f"  {dp_}")
    print(f"  {img_dir}")


if __name__ == "__main__":
    main()
