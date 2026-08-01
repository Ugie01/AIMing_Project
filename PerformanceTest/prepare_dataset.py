# -*- coding: utf-8 -*-
"""
시험 2 (PoC) 데이터셋 준비 스크립트
===================================
raw/ 폴더의 원본 이미지(드론 + 배경)에서:
  1) 드론 누끼 (rembg / U2-Net)         → drone/{ID}.png  (투명 PNG, 크기 정규화)
  2) 드론 자리 인페인팅으로 배경 복원     → bg/{ID}.png
  3) 검수용 시트 생성                    → review/{ID}.png (원본|마스크|드론|배경 나란히)

사용:
  pip install rembg opencv-python pillow numpy
  python prepare_dataset.py            # raw/ 전체 처리
  python prepare_dataset.py A C        # 특정 ID만 재처리 (검수 후 수정용)

반드시 review/ 시트를 눈으로 확인할 것. 누끼가 날개·로터를 잘라먹거나
그림자를 포함하는 경우가 있으며, 그런 ID만 골라 재처리하거나 수동 보정한다.
"""

import os
import sys

import cv2
import numpy as np
from PIL import Image
from rembg import remove, new_session

# ============================================================
# 설정
# ============================================================
RAW_DIR = "raw"        # 원본 (드론이 포함된 배경 이미지)
DRONE_DIR = "drone"    # 출력: 누끼 딴 드론
BG_DIR = "bg"          # 출력: 드론 제거된 배경
REVIEW_DIR = "review"  # 출력: 검수 시트

# 드론 크기 정규화: 잘라낸 드론의 "가로폭"을 이 픽셀로 통일.
# poc_test.py에서 다시 스케일하므로 여기선 여유 있게 크게 저장 (다운스케일이 화질 유리)
NORMALIZED_WIDTH = 300
# poc_test.py의 DRONE_TARGET_WIDTH와 같은 값을 적을 것 (하한 보호용)
DISPLAY_WIDTH = 73

# 누끼 마스크 이진화 임계 (0~255). 알파가 이 값 미만이면 완전 투명 처리
ALPHA_THRESHOLD = 40

# 인페인팅 전 마스크 팽창(px). 드론 경계의 잔상·그림자까지 지우기 위해 여유를 둠
MASK_DILATE_PX = 15

# 인페인팅 반경 (cv2.inpaint)
INPAINT_RADIUS = 7

REMBG_MODEL = "u2net"   # 결과가 나쁘면 "isnet-general-use" 시도

# ---- 경계 품질 (누끼 외곽선이 지저분할 때 여기를 조정) ----
USE_ALPHA_MATTING = True   # rembg 정밀 매팅. 느리지만 경계가 훨씬 깨끗함
EDGE_ERODE = 2             # 알파를 안쪽으로 깎을 픽셀 수. 후광이 남으면 3~4로 올림
EDGE_FEATHER = 1.0         # 경계 부드럽게(안티에일리어싱). 0이면 끔
DECONTAMINATE = True       # 반투명 경계의 배경색 오염 제거
# ============================================================


def list_raw():
    out = {}
    for f in sorted(os.listdir(RAW_DIR)):
        stem, ext = os.path.splitext(f)
        if ext.lower() in (".png", ".jpg", ".jpeg", ".bmp"):
            out[stem] = os.path.join(RAW_DIR, f)
    return out


def refine_edges(rgba):
    """누끼 경계 정리: 배경색 오염 제거 → 알파 침식 → 페더링.

    반투명 경계 픽셀은 드론색과 원래 배경색이 섞여 있어, 다른 위치에 합성하면
    후광으로 보인다. 그 픽셀들의 RGB를 불투명 영역 색으로 덮어쓴 뒤(오염 제거),
    알파를 살짝 깎아 남은 테두리를 잘라내고, 마지막에 부드럽게 만든다."""
    rgb = rgba[:, :, :3].copy()
    a = rgba[:, :, 3].copy()

    if DECONTAMINATE:
        fringe = ((a > 0) & (a < 250)).astype(np.uint8) * 255
        if fringe.any():
            # 경계 영역의 색을 내부 불투명 픽셀 색으로 채움
            rgb = cv2.inpaint(rgb, fringe, 3, cv2.INPAINT_TELEA)

    if EDGE_ERODE > 0:
        k = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (EDGE_ERODE * 2 + 1, EDGE_ERODE * 2 + 1))
        a = cv2.erode(a, k)

    if EDGE_FEATHER > 0:
        a = cv2.GaussianBlur(a, (0, 0), EDGE_FEATHER)

    out = np.dstack([rgb, a])
    return out


def largest_component(mask):
    """마스크에서 가장 큰 연결 성분만 남긴다 (드론 외 오검출 조각 제거)."""
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if num <= 1:
        return mask
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == largest, 255, 0).astype(np.uint8)


def process_one(image_id, path, session):
    print(f"[{image_id}] 처리 중...")
    raw_bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if raw_bgr is None:
        print(f"  [오류] 읽기 실패: {path}")
        return False
    raw_rgb = cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2RGB)

    # ---------- 1) 누끼 ----------
    kw = {}
    if USE_ALPHA_MATTING:
        kw = dict(alpha_matting=True,
                  alpha_matting_foreground_threshold=250,
                  alpha_matting_background_threshold=15,
                  alpha_matting_erode_size=11)
    try:
        rgba = remove(Image.fromarray(raw_rgb), session=session, **kw)
    except Exception as e:
        print(f"  [알림] 정밀 매팅 실패({type(e).__name__}) — 기본 모드로 진행. "
              f"pip install pymatting 후 재시도 권장")
        rgba = remove(Image.fromarray(raw_rgb), session=session)
    rgba = np.array(rgba)                       # H×W×4
    alpha = rgba[:, :, 3].copy()

    mask = np.where(alpha >= ALPHA_THRESHOLD, 255, 0).astype(np.uint8)
    mask = largest_component(mask)
    if mask.sum() == 0:
        print("  [오류] 전경을 찾지 못함 — REMBG_MODEL 변경 또는 수동 처리 필요")
        return False

    # 정리된 마스크를 알파에 반영 (조각 제거 부분 투명화)
    alpha[mask == 0] = 0
    rgba[:, :, 3] = alpha
    rgba = refine_edges(rgba)

    # ---------- 2) 드론 크롭 + 크기 정규화 ----------
    ys, xs = np.where(mask > 0)
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    pad = 4
    y0, x0 = max(0, y0 - pad), max(0, x0 - pad)
    y1 = min(rgba.shape[0], y1 + pad)
    x1 = min(rgba.shape[1], x1 + pad)
    drone = rgba[y0:y1, x0:x1]
    raw_w = drone.shape[1]

    # 원본보다 크게 확대하지 않는다 (확대하면 화질만 나빠지고 정보는 안 늘어남).
    # 최종 표시 크기(DISPLAY_WIDTH)보다는 반드시 크게 유지.
    target_w = min(NORMALIZED_WIDTH, max(drone.shape[1], DISPLAY_WIDTH))
    scale = target_w / drone.shape[1]
    new_h = max(1, round(drone.shape[0] * scale))
    interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC
    drone = cv2.resize(drone, (target_w, new_h), interpolation=interp)
    if drone.shape[1] < DISPLAY_WIDTH * 1.2:
        print(f"  [주의] 원본 드론이 작습니다({raw_w}px). "
              f"표시 크기 {DISPLAY_WIDTH}px에 가까워 선명도 확보가 어려움 — "
              f"드론이 더 크게 나온 프레임으로 교체 권장")
    Image.fromarray(drone).save(os.path.join(DRONE_DIR, f"{image_id}.png"))

    # ---------- 3) 배경 인페인팅 ----------
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (MASK_DILATE_PX * 2 + 1, MASK_DILATE_PX * 2 + 1))
    inpaint_mask = cv2.dilate(mask, kernel)
    bg = cv2.inpaint(raw_bgr, inpaint_mask, INPAINT_RADIUS, cv2.INPAINT_TELEA)
    cv2.imwrite(os.path.join(BG_DIR, f"{image_id}.png"), bg)

    # ---------- 4) 검수 시트 ----------
    h = 260
    def fit(img_bgr):
        s = h / img_bgr.shape[0]
        return cv2.resize(img_bgr, (max(1, int(img_bgr.shape[1] * s)), h))

    mask_vis = cv2.cvtColor(inpaint_mask, cv2.COLOR_GRAY2BGR)
    dh, dw = drone.shape[:2]
    a = drone[:, :, 3:4].astype(np.float32) / 255
    drone_rgb = drone[:, :, :3][:, :, ::-1]      # RGBA(RGB순) → BGR
    checker = np.full((dh, dw, 3), 200, np.uint8)
    checker[::2, ::2] = 150
    drone_vis = (drone_rgb * a + checker * (1 - a)).astype(np.uint8)
    # 후광 확인용: 대비 강한 마젠타 배경에 얹기
    mag = np.full((dh, dw, 3), (200, 0, 200), np.uint8)
    halo_vis = (drone_rgb * a + mag * (1 - a)).astype(np.uint8)

    panels = [fit(raw_bgr), fit(mask_vis), fit(drone_vis), fit(halo_vis), fit(bg)]
    gap = np.full((h, 6, 3), 255, np.uint8)
    sheet = panels[0]
    for pnl in panels[1:]:
        sheet = np.hstack([sheet, gap, pnl])
    for i, label in enumerate(["RAW", "MASK", "DRONE", "HALO CHECK", "BG(INPAINT)"]):
        x = sum(p.shape[1] + 6 for p in panels[:i]) + 8
        cv2.putText(sheet, label, (x, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(REVIEW_DIR, f"{image_id}.png"), sheet)

    print(f"  drone {drone.shape[1]}x{drone.shape[0]}px (원본 {raw_w}px)  /  bg 인페인팅 완료  "
          f"/  review/{image_id}.png 확인 요망")
    return True


def main():
    for d in (DRONE_DIR, BG_DIR, REVIEW_DIR):
        os.makedirs(d, exist_ok=True)
    raws = list_raw()
    if not raws:
        print(f"[오류] {RAW_DIR}/ 에 이미지가 없습니다.")
        sys.exit(1)

    targets = sys.argv[1:] if len(sys.argv) > 1 else list(raws.keys())
    unknown = [t for t in targets if t not in raws]
    if unknown:
        print(f"[오류] raw/에 없는 ID: {unknown}")
        sys.exit(1)

    print(f"모델 로딩 ({REMBG_MODEL})...")
    session = new_session(REMBG_MODEL)

    ok = 0
    for image_id in targets:
        if process_one(image_id, raws[image_id], session):
            ok += 1
    print(f"\n완료: {ok}/{len(targets)}  →  review/ 폴더를 반드시 눈으로 검수하세요.")
    print("문제 있는 ID는  python prepare_dataset.py <ID>  로 재처리하거나,")
    print("설정(ALPHA_THRESHOLD, MASK_DILATE_PX, REMBG_MODEL)을 바꿔 다시 시도.")


if __name__ == "__main__":
    main()
