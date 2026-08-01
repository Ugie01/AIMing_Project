# -*- coding: utf-8 -*-
"""
시험 2 (PoC 검증) 실행 스크립트
================================
drone/  폴더: 누끼 딴 드론 PNG (투명 배경).  예: A.png, B.png ...
bg/     폴더: 같은 ID의 배경 이미지.        예: A.png(또는 .jpg), B.png ...

흐름:
  블랙 화면(터렛 탐색 복귀 대기) → 합성 이미지 표시 + t0 기록
  → UART에서 FIRE 수신 시 시간 기록 → 육안 판정 키 입력 → 다음 시행

판정 키:
  1 = 적중        2 = 미적중(추적했으나 빗나감)
  3 = 미검출      4 = 오검출(다른 물체를 드론으로 인식)
  R = 무효(재시행 큐에 다시 넣음)   ESC = 시험 중단(저장 후 종료)

결과: results_YYYYMMDD_HHMMSS.csv + 요약 출력
"""

import csv
import os
import random
import sys
import threading
import time
from datetime import datetime

import pygame

# ============================================================
# 설정 — 실측일 전에 여기만 손대면 됨
# ============================================================
DRONE_DIR = "drone"
BG_DIR = "bg"
OUT_DIR = "results"

TRIALS_PER_IMAGE = 10          # 이미지당 시행 수 (10장 × 10회 = 100회)
TIMEOUT_S = 10.0               # 이 시간 안에 FIRE 없으면 타임아웃 표시 (판정은 사람이)
INTER_TRIAL_BLANK_S = 3.0      # 시행 사이 블랙 화면(터렛이 탐색 상태로 복귀할 시간)

# 드론 표시 크기 (모니터 px 기준 가로폭). None이면 원본 크기 그대로.
# d=20cm 기준 모델 12px ≈ 모니터 73px
DRONE_TARGET_WIDTH = 250

# 스폰 허용 범위: 화면 중심 기준. 이전 계산값 (가로 ±291px, 세로 ±203px)
SPAWN_HALF_W = 291
SPAWN_HALF_H = 203

# UART
SERIAL_PORT = "COM13"           # Linux/Mac이면 "/dev/ttyUSB0" 등
BAUD_RATE = 115200
FIRE_KEYWORD = "FIRE"          # 보드 로그 라인에 이 문자열이 포함되면 발사로 간주
SIMULATE_SERIAL = False        # True면 시리얼 없이 스페이스바로 FIRE 대용 (리허설용)

FULLSCREEN = True
BG_FILL = (0, 0, 0)

RESULT_LABELS = {1: "적중", 2: "미적중", 3: "미검출", 4: "오검출"}
# ============================================================


class SerialFireListener:
    """백그라운드 스레드에서 UART 라인을 읽고 FIRE 이벤트 시각을 기록한다."""

    def __init__(self, port, baud, keyword):
        self.keyword = keyword
        self.fire_time = None          # time.monotonic() 기준
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._armed = threading.Event()
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    text = line.decode(errors="ignore")
                except Exception:
                    continue
                if self._armed.is_set() and self.keyword in text:
                    with self._lock:
                        if self.fire_time is None:   # 시행당 첫 FIRE만
                            self.fire_time = time.monotonic()

    def arm(self):
        """시행 시작: 이전 FIRE 기록을 지우고 감지 시작."""
        with self._lock:
            self.fire_time = None
        # 시행 시작 전 버퍼에 남은 잔여 데이터 제거
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._armed.set()

    def disarm(self):
        self._armed.clear()

    def get_fire_time(self):
        with self._lock:
            return self.fire_time

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


class SimulatedListener:
    """리허설용: 스페이스바를 FIRE로 취급. 인터페이스만 맞춘 더미."""

    def __init__(self):
        self.fire_time = None

    def arm(self):
        self.fire_time = None

    def disarm(self):
        pass

    def get_fire_time(self):
        return self.fire_time

    def trigger(self):
        if self.fire_time is None:
            self.fire_time = time.monotonic()

    def close(self):
        pass


def load_pairs():
    """drone/와 bg/에서 같은 stem을 가진 쌍을 찾아 (id, drone_path, bg_path) 목록 반환."""
    def stems(d):
        out = {}
        for f in os.listdir(d):
            stem, ext = os.path.splitext(f)
            if ext.lower() in (".png", ".jpg", ".jpeg", ".bmp"):
                out[stem] = os.path.join(d, f)
        return out

    drones = stems(DRONE_DIR)
    bgs = stems(BG_DIR)
    ids = sorted(set(drones) & set(bgs))
    missing = sorted(set(drones) ^ set(bgs))
    if missing:
        print(f"[경고] 짝이 없는 ID는 제외됨: {missing}")
    if not ids:
        print("[오류] drone/와 bg/에서 매칭되는 이미지 쌍이 없습니다.")
        sys.exit(1)
    return [(i, drones[i], bgs[i]) for i in ids]


def build_trial_queue(pairs):
    """이미지당 TRIALS_PER_IMAGE회, 전체를 셔플한 시행 큐."""
    q = []
    for image_id, dp, bp in pairs:
        q += [(image_id, dp, bp)] * TRIALS_PER_IMAGE
    random.shuffle(q)
    return q


def compose(screen, bg_img, drone_img):
    """배경을 화면 크기로 맞추고, 드론을 스폰 범위 내 랜덤 위치에 합성.
    반환: (합성된 Surface, 드론 중심 좌표(화면 기준))"""
    sw, sh = screen.get_size()
    bg = pygame.transform.smoothscale(bg_img, (sw, sh))

    d = drone_img
    if DRONE_TARGET_WIDTH is not None:
        ratio = DRONE_TARGET_WIDTH / d.get_width()
        d = pygame.transform.smoothscale(
            d, (DRONE_TARGET_WIDTH, max(1, int(d.get_height() * ratio))))

    cx0, cy0 = sw // 2, sh // 2
    half_w = min(SPAWN_HALF_W, sw // 2 - d.get_width() // 2 - 5)
    half_h = min(SPAWN_HALF_H, sh // 2 - d.get_height() // 2 - 5)
    cx = cx0 + random.randint(-half_w, half_w)
    cy = cy0 + random.randint(-half_h, half_h)

    surf = bg.copy()
    surf.blit(d, (cx - d.get_width() // 2, cy - d.get_height() // 2))
    return surf, (cx, cy)


def draw_status(screen, font, lines, color=(255, 255, 255)):
    """블랙 화면에 상태 텍스트(진행자용) 표시."""
    screen.fill(BG_FILL)
    y = screen.get_height() // 2 - len(lines) * 22
    for ln in lines:
        img = font.render(ln, True, color)
        screen.blit(img, (screen.get_width() // 2 - img.get_width() // 2, y))
        y += 44
    pygame.display.flip()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    pairs = load_pairs()
    queue = build_trial_queue(pairs)
    total_planned = len(queue)

    pygame.init()
    flags = pygame.FULLSCREEN if FULLSCREEN else 0
    screen = pygame.display.set_mode((0, 0), flags)
    pygame.display.set_caption("PoC Test")
    pygame.mouse.set_visible(False)
    font = pygame.font.SysFont("malgungothic,applesdgothicneo,nanumgothic", 28)

    # 이미지 프리로드
    cache = {}
    for image_id, dp, bp in pairs:
        cache[image_id] = (
            pygame.image.load(dp).convert_alpha(),
            pygame.image.load(bp).convert(),
        )

    # 시리얼
    if SIMULATE_SERIAL:
        listener = SimulatedListener()
        print("[모드] 시리얼 시뮬레이션 — 스페이스바가 FIRE 역할")
    else:
        try:
            listener = SerialFireListener(SERIAL_PORT, BAUD_RATE, FIRE_KEYWORD)
            print(f"[시리얼] {SERIAL_PORT} @ {BAUD_RATE} 연결됨")
        except Exception as e:
            print(f"[오류] 시리얼 연결 실패: {e}")
            print("리허설이면 SIMULATE_SERIAL = True 로 바꿔서 실행하세요.")
            pygame.quit()
            sys.exit(1)

    out_path = os.path.join(
        OUT_DIR, f"results_{datetime.now():%Y%m%d_%H%M%S}.csv")
    fcsv = open(out_path, "w", newline="", encoding="utf-8-sig")
    writer = csv.writer(fcsv)
    writer.writerow(["trial", "image_id", "pos_x", "pos_y",
                     "shown_at", "fire_received", "time_to_fire_s",
                     "result_code", "result_label"])

    results = []
    trial_no = 0
    aborted = False

    try:
        while queue:
            image_id, dp, bp = queue.pop(0)
            trial_no += 1

            # ---- 시행 간 블랙 화면 (터렛 탐색 복귀) ----
            t_end = time.monotonic() + INTER_TRIAL_BLANK_S
            while time.monotonic() < t_end:
                draw_status(screen, font, [
                    f"시행 {trial_no} / {total_planned}   준비 중...",
                    f"다음: {image_id}",
                ], (120, 120, 120))
                for ev in pygame.event.get():
                    if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt
                time.sleep(0.05)

            # ---- 합성 및 표시 ----
            drone_img, bg_img = cache[image_id]
            surf, (px, py) = compose(screen, bg_img, drone_img)
            listener.arm()
            screen.blit(surf, (0, 0))
            pygame.display.flip()
            t0 = time.monotonic()
            shown_at = datetime.now().isoformat(timespec="milliseconds")

            # ---- FIRE 대기 + 판정 입력 대기 ----
            fire_t = None
            result = None
            timeout_flagged = False
            while result is None:
                if fire_t is None:
                    fire_t = listener.get_fire_time()
                    if fire_t is not None:
                        # FIRE 수신을 화면 구석에 작게 표시 (진행자 확인용)
                        tt = fire_t - t0
                        tag = font.render(
                            f"FIRE  {tt:.2f}s   판정 입력 (1적중 2미적중 3미검출 4오검출 R무효)",
                            True, (255, 80, 80))
                        screen.blit(surf, (0, 0))
                        screen.blit(tag, (20, 16))
                        pygame.display.flip()

                if fire_t is None and not timeout_flagged and \
                        time.monotonic() - t0 > TIMEOUT_S:
                    timeout_flagged = True
                    tag = font.render(
                        f"TIMEOUT {TIMEOUT_S:.0f}s   판정 입력 (1적중 2미적중 3미검출 4오검출 R무효)",
                        True, (255, 200, 60))
                    screen.blit(surf, (0, 0))
                    screen.blit(tag, (20, 16))
                    pygame.display.flip()

                for ev in pygame.event.get():
                    if ev.type != pygame.KEYDOWN:
                        continue
                    if ev.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt
                    if ev.key == pygame.K_SPACE and SIMULATE_SERIAL:
                        listener.trigger()
                    if ev.key in (pygame.K_1, pygame.K_KP1):
                        result = 1
                    elif ev.key in (pygame.K_2, pygame.K_KP2):
                        result = 2
                    elif ev.key in (pygame.K_3, pygame.K_KP3):
                        result = 3
                    elif ev.key in (pygame.K_4, pygame.K_KP4):
                        result = 4
                    elif ev.key == pygame.K_r:
                        result = "R"
                time.sleep(0.02)

            listener.disarm()

            # ---- 기록 ----
            if result == "R":
                queue.append((image_id, dp, bp))   # 큐 끝에 재삽입
                random.shuffle(queue)
                trial_no -= 1
                total_planned = trial_no + len(queue)
                print(f"  [무효] {image_id} 재시행 큐에 추가")
                continue

            ttf = round(fire_t - t0, 3) if fire_t is not None else ""
            writer.writerow([trial_no, image_id, px, py, shown_at,
                             fire_t is not None, ttf,
                             result, RESULT_LABELS[result]])
            fcsv.flush()
            results.append((image_id, result,
                            fire_t - t0 if fire_t is not None else None))
            print(f"  [{trial_no}/{total_planned}] {image_id}  "
                  f"{RESULT_LABELS[result]}"
                  + (f"  {ttf}s" if ttf != "" else ""))

    except KeyboardInterrupt:
        aborted = True
        print("\n[중단] ESC 입력 — 여기까지 저장됨")
    finally:
        listener.close()
        fcsv.close()
        pygame.quit()

    # ---- 요약 ----
    n = len(results)
    print("\n" + "=" * 50)
    print(f"결과 파일: {out_path}")
    print(f"총 시행: {n}" + ("  (중단됨)" if aborted else ""))
    if n:
        for code, label in RESULT_LABELS.items():
            c = sum(1 for _, r, _ in results if r == code)
            print(f"  {label:4s}: {c:3d}  ({c / n * 100:.1f}%)")
        hits = sum(1 for _, r, _ in results if r == 1)
        print(f"\nPoC 성공률(적중): {hits}/{n} = {hits / n * 100:.1f}%")
        times = sorted(t for _, r, t in results if r == 1 and t is not None)
        if times:
            med = times[len(times) // 2]
            print(f"Time-to-Fire (적중만): 중앙값 {med:.2f}s, "
                  f"평균 {sum(times) / len(times):.2f}s, "
                  f"최소 {times[0]:.2f}s, 최대 {times[-1]:.2f}s")
        print("\n이미지별:")
        for image_id, _, _ in pairs:
            sub = [(r, t) for i, r, t in results if i == image_id]
            if not sub:
                continue
            h = sum(1 for r, _ in sub if r == 1)
            print(f"  {image_id}: {h}/{len(sub)} 적중")
    print("=" * 50)


if __name__ == "__main__":
    main()
