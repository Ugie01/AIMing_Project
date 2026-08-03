# PerformanceTest — PC 기반 AI / 하드웨어 성능 시험 도구

STM32H743(`cortex_m_project`)에 FOMO 드론 검출 모델을 올려 실사 시험을 하기 전에,
**PC에서 먼저 검증**하기 위한 파이썬 스크립트 모음입니다.

두 종류의 시험을 지원합니다.

| 구분 | 스크립트 | 확인 대상 |
|---|---|---|
| 시험 1 — AI 성능 | `ai_test.py` | 모델 자체의 검출률·적중률·중심 오차 (사람 개입 없음) |
| 시험 2 — 하드웨어 PoC | `poc_test.py` | 모니터에 띄운 표적을 보드가 실제로 조준·발사하는지 (실물 터렛 포함) |

두 스크립트는 **동일한 합성 규칙**(드론 크기·스폰 범위·랜덤 위치)을 쓰도록 만들어져,
"모델은 맞췄는데 하드웨어에서 못 맞춘 것"과 "모델부터 못 찾은 것"을 분리해서 볼 수 있습니다.

---

## 왜 이런 구조가 필요했나

실제 드론을 띄워 시험하는 것은 장소·안전·기상 제약이 크고, 매번 같은 조건을 재현할 수 없습니다.
그래서 **모니터에 드론 영상을 띄우고 터렛이 그것을 조준하게 하는 방식**으로 시험을 대체했는데,
여기서 두 가지 문제가 생깁니다.

1. **정답 좌표를 알 수 없다** — 실사 영상은 드론이 어디에 있었는지 사람이 일일이 라벨링해야 합니다.
2. **표본이 부족하다** — 촬영한 원본 이미지 몇 장으로는 통계적으로 의미 있는 시행 횟수가 안 나옵니다.

`prepare_dataset.py`가 원본 사진에서 드론과 배경을 **분리**해 두면,
이후 스크립트가 드론을 임의 위치에 다시 합성하면서 **정답 좌표(GT)를 스스로 알고 있는 무한한 시험 장면**을
만들어낼 수 있습니다. 사진 2장으로 100회 시행이 가능해지고, 판정도 자동화됩니다.

---

## 폴더 구조

```
PerformanceTest/
├── prepare_dataset.py      # ① 데이터셋 준비 (드론/배경 분리)
├── ai_test.py              # ② AI 검출 성능 자동 평가
├── poc_test.py             # ③ 실물 터렛 PoC 시험 (화면 출력 + UART)
├── ei-detect_drone_ver2_...int8-quantized-model.3.lite   # Edge Impulse FOMO 모델
│
├── raw/                    # [입력] 드론이 찍힌 원본 사진
├── drone/                  # [생성] 누끼 딴 드론 PNG (투명 배경)
├── bg/                     # [생성] 드론을 지운 배경 이미지
├── review/                 # [생성] 누끼 품질 검수용 시트
│
├── AI_Test_Result/         # ai_test.py 결과 (CSV + 시각화 이미지)
└── results/                # poc_test.py 결과 (CSV)
```

`raw`, `drone`, `bg`는 같은 파일명(stem)을 ID로 사용해 짝을 맞춥니다. (예: `F_2.jpg` → `F_2.png`)

---

## ① prepare_dataset.py — 데이터셋 준비

### 구현 내용
`raw/`의 원본 사진 한 장에서 세 가지 산출물을 만듭니다.

1. **드론 누끼** — `rembg`(U2-Net)로 전경을 추출하고, 가장 큰 연결 성분만 남겨 잔조각을 제거한 뒤
   경계를 정리(배경색 오염 제거 → 알파 침식 → 페더링)해 `drone/{ID}.png`로 저장합니다.
   경계 처리를 넣은 이유는, 반투명 테두리 픽셀에 원래 배경색이 섞여 있어 다른 배경에 합성하면
   **후광(halo)** 이 생기고 이것이 검출 결과를 왜곡하기 때문입니다.
2. **배경 복원** — 드론 마스크를 15px 팽창시켜 그림자·잔상까지 덮은 뒤 `cv2.inpaint`로 메워
   드론이 없던 것처럼 만든 배경을 `bg/{ID}.png`로 저장합니다.
3. **검수 시트** — `RAW | MASK | DRONE | HALO CHECK | BG(INPAINT)`를 가로로 붙인 이미지를
   `review/{ID}.png`로 저장합니다. HALO CHECK 패널은 마젠타 배경 위에 드론을 얹어
   후광이 남았는지 눈으로 바로 확인할 수 있게 한 것입니다.

드론은 가로 300px(`NORMALIZED_WIDTH`)로 정규화해 저장하되, **원본보다 확대하지는 않습니다.**
확대는 화질만 나빠지고 정보량은 늘지 않기 때문이며, 최종 표시 크기보다 원본이 작으면 경고를 출력합니다.

### 사용법

```bash
pip install rembg opencv-python pillow numpy pymatting
```

```bash
python prepare_dataset.py
```

특정 ID만 재처리 (검수에서 문제가 발견된 경우):

```bash
python prepare_dataset.py F_2 F_3
```

> 실행 후 **반드시 `review/`를 눈으로 확인**하세요. 누끼가 날개·로터를 잘라먹거나 그림자를 포함하면
> 이후 모든 시험 결과가 오염됩니다. 문제가 있으면 `ALPHA_THRESHOLD`, `MASK_DILATE_PX`,
> `REMBG_MODEL`(`u2net` → `isnet-general-use`)을 조정해 재실행합니다.

---

## ② ai_test.py — AI 검출 성능 자동 평가

### 구현 내용
사람 개입 없이 모델만 평가합니다. 시행 1회의 흐름은 다음과 같습니다.

```
bg를 1920x1080으로 리사이즈 → drone을 스폰 범위 내 랜덤 위치에 알파 합성 (이때 GT 좌표 확보)
→ 중앙 1:1 크롭 → 96x96 리사이즈 → int8 변환 → TFLite 추론
→ 12x12 확률맵 후처리 → GT와 거리 비교로 자동 판정
```

- **전처리**는 Edge Impulse의 `FIT_SHORTEST`(중앙 정사각 크롭 후 리사이즈)와 동일하게 맞췄습니다.
  보드에서 도는 전처리와 다르면 PC 결과를 신뢰할 수 없기 때문입니다.
- **후처리**는 FOMO 방식으로, 임계값(0.5)을 넘은 셀을 인접끼리 병합(`MERGE_MODE="cube"`)해
  하나의 검출 박스로 만듭니다. `"cell"`로 바꾸면 셀 단위로 볼 수 있습니다.
- **백엔드**는 `tflite_runtime` → `ai_edge_litert` → `tensorflow.lite` 순으로 자동 탐색합니다.
  (Windows에서는 `tensorflow` 설치가 가장 무난합니다.)
- **판정 코드는 `poc_test.py`와 동일한 체계**를 씁니다.

| 코드 | 라벨 | 의미 |
|---|---|---|
| 1 | 적중 | GT에서 `HIT_RADIUS_96`(12px, 96 기준) 이내에 검출 |
| 2 | 미적중 | 검출 1개인데 GT에서 벗어남 |
| 3 | 미검출 | 아무것도 검출하지 못함 |
| 4 | 오검출 | GT는 놓치고 엉뚱한 곳만 여러 개 검출 |
| — | 범위밖 | 드론이 크롭 영역 밖에 있어 모델이 볼 수 없었음 → 통계에서 제외 |

### 출력물
- `AI_Test_Result/summary_YYYYMMDD_HHMMSS.csv` — 시행별 GT 좌표, 판정, 중심 오차, 추론 시간
- `AI_Test_Result/detections_YYYYMMDD_HHMMSS.csv` — 검출 박스별 상세(점수, 96/원본 좌표, 활성 셀)
- `AI_Test_Result/images/*_raw.png` — 합성 원본에 GT(노란 원)·검출(초록 박스)·크롭 영역(파란 사각) 표시
- `AI_Test_Result/images/*_96.png` — 실제 모델이 본 96x96 화면 (5배 확대)
- `AI_Test_Result/images/*_grid.png` — 12x12 셀 확률 히트맵. 기본값(`SAVE_GRID="miss"`)에서는
  **적중하지 못한 시행에만** 생성되며, 미검출 원인을 셀 단위로 추적할 때 사용합니다.

콘솔에는 적중률·검출률·중심 오차 통계와 이미지별 적중률 막대가 함께 출력됩니다.

### 사용법

```bash
pip install numpy opencv-python tensorflow
```

```bash
python ai_test.py
```

주요 설정은 파일 상단 `[1] 설정` 블록에 모여 있습니다.
`TRIALS_PER_IMAGE`(이미지당 시행 수), `RANDOM_SEED`(재현성), `THRESHOLD`, `HIT_RADIUS_96`,
`DRONE_TARGET_WIDTH`를 여기서 조정합니다.

---

## ③ poc_test.py — 실물 터렛 PoC 시험

### 구현 내용
모니터를 전체화면으로 점유해 합성 표적을 띄우고, 보드가 조준·발사할 때까지의 시간을 재는 시험 진행 도구입니다.
시행 1회의 흐름은 다음과 같습니다.

```
블랙 화면 3초(터렛이 탐색 상태로 복귀할 시간)
→ 합성 이미지 표시 + t0 기록
→ 백그라운드 스레드가 UART에서 "FIRE" 수신 → 시각 기록
→ 진행자가 육안으로 판정 키 입력 → 다음 시행
```

- **UART 리스너**는 별도 스레드에서 시리얼 라인을 읽습니다. 시행 시작 시 `arm()`으로 입력 버퍼를 비우고
  이전 기록을 지우기 때문에, 앞 시행에서 늦게 도착한 로그가 다음 시행의 측정값을 오염시키지 않습니다.
  **시행당 첫 FIRE만** 기록합니다.
  보드 쪽에서는 `cortex_m_project/Core/Src/Tasks/task_motor.c`가 발사 상태 진입 시 1회 `FIRE\r\n`을 보냅니다.
- **판정은 사람이** 합니다. 표적을 맞췄는지 여부는 물리적 결과이므로 자동화할 수 없기 때문입니다.
  `TIMEOUT_S`(10초)가 지나면 화면에 TIMEOUT 표시만 하고, 판정 자체는 그대로 진행자에게 맡깁니다.
- **무효(R) 처리** — 시행 도중 문제가 생기면 R을 눌러 해당 시행을 기록하지 않고 큐 뒤에 다시 넣습니다.
  전체 시행 수가 줄어들지 않도록 카운터도 되돌립니다.
- **시행 큐는 셔플**되어, 같은 이미지가 연속으로 나와 진행자가 위치를 예측하는 것을 막습니다.
- `SIMULATE_SERIAL = True`로 두면 보드 없이 **스페이스바를 FIRE 대용**으로 써서 리허설할 수 있습니다.

### 판정 키

| 키 | 의미 |
|---|---|
| `1` | 적중 |
| `2` | 미적중 (추적은 했으나 빗나감) |
| `3` | 미검출 |
| `4` | 오검출 (다른 물체를 드론으로 인식) |
| `R` | 무효 — 재시행 큐에 다시 넣음 |
| `ESC` | 시험 중단 (여기까지 저장 후 종료) |

### 출력물
`results/results_YYYYMMDD_HHMMSS.csv` — 시행별 표적 좌표, 표시 시각, FIRE 수신 여부,
Time-to-Fire(초), 판정 코드/라벨. 매 시행마다 `flush()`하므로 중간에 종료해도 기록이 남습니다.

종료 시 콘솔에 판정별 비율, PoC 성공률(적중), Time-to-Fire 통계(적중 건만: 중앙값/평균/최소/최대),
이미지별 적중률이 출력됩니다.

`results/150.csv`, `results/250.csv`는 `DRONE_TARGET_WIDTH`를 각각 150px, 250px로 바꿔 진행한
지난 시험 기록입니다. (표적 크기에 따른 검출 성능 비교용)

### 사용법

```bash
pip install pygame pyserial
```

```bash
python poc_test.py
```

실행 전 파일 상단 설정에서 다음을 확인하세요.

- `SERIAL_PORT` — 보드가 연결된 COM 포트 (Linux/Mac은 `/dev/ttyUSB0` 등), `BAUD_RATE`는 115200
- `DRONE_TARGET_WIDTH` — 모니터에 표시할 드론 가로폭(px). 시험 조건의 핵심 변수
- `TRIALS_PER_IMAGE` — 이미지당 시행 수 (기본 10)
- `INTER_TRIAL_BLANK_S` — 시행 간 블랙 화면 시간. 터렛이 탐색 상태로 확실히 돌아갈 만큼 줄 것

---

## 전체 진행 순서

```bash
python prepare_dataset.py    # 1. raw/ → drone/, bg/, review/
```
```bash
python ai_test.py            # 2. 모델 단독 성능 확인 (보드 불필요)
```
```bash
python poc_test.py           # 3. 보드 + 터렛 연결 후 실물 PoC 시험
```

1단계 후에는 `review/`를 검수하고, 2단계에서 적중률이 낮으면 3단계로 넘어가기 전에
모델이나 표적 크기부터 다시 봐야 합니다. 하드웨어 시험은 준비 비용이 크기 때문입니다.

---

## 주의: 스크립트 간 설정 동기화

세 스크립트에 **같은 의미의 값이 각각 따로 정의**되어 있습니다. 비교 가능한 결과를 얻으려면 손으로 맞춰야 합니다.

| 값 | prepare_dataset.py | ai_test.py | poc_test.py |
|---|---|---|---|
| 드론 표시 가로폭 | `DISPLAY_WIDTH = 73` | `DRONE_TARGET_WIDTH = 135` | `DRONE_TARGET_WIDTH = 250` |
| 스폰 범위 | — | `SPAWN_HALF_W/H = 291/203` | `SPAWN_HALF_W/H = 291/203` |
| 이미지당 시행 | — | `TRIALS_PER_IMAGE = 10` | `TRIALS_PER_IMAGE = 10` |

현재 커밋 기준으로 **드론 표시 가로폭이 세 파일에서 서로 다릅니다.**
`ai_test.py`와 `poc_test.py`의 결과를 직접 비교할 목적이라면 두 값을 같게 맞추고,
`prepare_dataset.py`의 `DISPLAY_WIDTH`는 그중 가장 작은 값(하한 보호용)으로 설정하세요.
