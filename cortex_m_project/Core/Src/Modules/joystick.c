#include "joystick.h"
#include "adc.h"

// ==============================================================================
// 매크로 및 상수 정의
// ==============================================================================

// ADC 변환 타임아웃
#define JOYSTICK_ADC_TIMEOUT_MS   10U
// 중립 오프셋 계산용 초기 샘플링 횟수
#define JOYSTICK_CALIB_SAMPLES    32U
// 조이스틱 데드존 (오차 및 미세 떨림 무시 영역, 약 4.9%)
#define JOYSTICK_DEADZONE         200
// 버튼 채터링 방지를 위한 디바운스 연속 일치 횟수
#define JOYSTICK_DEBOUNCE_COUNT   2U

// ==============================================================================
// 모듈 정적(Static) 상태 변수 모음
// ==============================================================================

// 초기 중립(Center) ADC 값 보관용 변수 (초기화 실패 시 기본값 2048)
static uint16_t joystick_center_x = 2048U;
static uint16_t joystick_center_y = 2048U;

// 버튼 디바운스 처리용 상태 변수
static bool joystick_button_stable = false; // 확정된 버튼 상태
static bool joystick_button_last = false;   // 직전 버튼 원시 입력
static uint8_t joystick_button_count = 0U;  // 연속 일치 카운트
static volatile uint16_t adc_dma_buffer[2]; // X, Y축 DMA 수신 버퍼
static bool is_adc_dma_running = false;

// ==============================================================================
// 내부 데이터 처리 함수
// ==============================================================================

// ADC1을 폴링하여 X(Rank1), Y(Rank2) 아날로그 원시값 측정
static bool Joystick_SampleRaw(uint16_t *raw_x, uint16_t *raw_y) {
    SCB_InvalidateDCache_by_Addr((uint32_t*) adc_dma_buffer, sizeof(adc_dma_buffer));
    // DMA가 백그라운드에서 실시간으로 버퍼를 갱신하므로 대기 없이 즉시 읽음
    *raw_x = adc_dma_buffer[0]; // Rank 1 (X축)
    *raw_y = adc_dma_buffer[1]; // Rank 2 (Y축)
    return true;
}

// ADC 원시값을 중립값과 데드존을 적용하여 ±JOYSTICK_AXIS_MAX 로 정규화
static int16_t Joystick_Normalize(uint16_t raw, uint16_t center) {
    int32_t value = (int32_t) raw - (int32_t) center;
    int32_t span;
    int32_t magnitude;

    // 데드존 이내면 0 반환
    if ((value > -JOYSTICK_DEADZONE) && (value < JOYSTICK_DEADZONE)) {
        return 0;
    }

    // 데드존 경계부터 새 원점으로 스케일링
    if (value > 0) {
        span = (int32_t) 4095 - (int32_t) center - JOYSTICK_DEADZONE;
        magnitude = value - JOYSTICK_DEADZONE;
    } else {
        span = (int32_t) center - JOYSTICK_DEADZONE;
        magnitude = -value - JOYSTICK_DEADZONE;
    }

    // 캘리브레이션 오류 방어
    if (span <= 0)
        return 0;
    if (magnitude > span)
        magnitude = span; 

    // ±2048 스케일로 매핑
    magnitude = (magnitude * JOYSTICK_AXIS_MAX) / span;
    if (magnitude > JOYSTICK_AXIS_MAX)
        magnitude = JOYSTICK_AXIS_MAX;

    return (value > 0) ? (int16_t) magnitude : (int16_t) (-magnitude);
}

// ==============================================================================
// 외부 공개 API 함수
// ==============================================================================

void Joystick_Init(void) {
    joystick_button_stable = (HAL_GPIO_ReadPin(JOY_SW_GPIO_Port, JOY_SW_Pin) == GPIO_PIN_RESET);
    joystick_button_last = joystick_button_stable;
    joystick_button_count = 0U;
}

// 수동 제어 시작 시 호출할 DMA 시작 함수 (기존 Init의 캘리브레이션 내용을 가져옴)
void Joystick_Start_DMA(void) {
    uint32_t sum_x = 0U, sum_y = 0U;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_dma_buffer, 2);
    HAL_Delay(10); // DMA 첫 데이터 대기

    for (uint32_t i = 0U; i < JOYSTICK_CALIB_SAMPLES; i++) {
        uint16_t raw_x, raw_y;
        if (Joystick_SampleRaw(&raw_x, &raw_y)) {
            sum_x += raw_x;
            sum_y += raw_y;
        }
        HAL_Delay(1);
    }
    joystick_center_x = (uint16_t) (sum_x / JOYSTICK_CALIB_SAMPLES);
    joystick_center_y = (uint16_t) (sum_y / JOYSTICK_CALIB_SAMPLES);

    is_adc_dma_running = true; // 플래그 ON
}

// 수동 제어 종료 시 호출할 DMA 정지 함수
void Joystick_Stop_DMA(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    is_adc_dma_running = false; // 플래그 OFF
}

// 현재 조이스틱의 X/Y 좌표 및 스위치 상태를 읽어 out 구조체에 반환
void Joystick_Read(JoystickInput_t *out) {
    uint16_t raw_x = joystick_center_x;
    uint16_t raw_y = joystick_center_y;
    bool raw_button;
    bool was_down;

    if (out == NULL)
        return;

    // 아날로그 축 읽기 및 정규화
    if (is_adc_dma_running) {
        if (Joystick_SampleRaw(&raw_x, &raw_y)) {
            out->x = Joystick_Normalize(raw_x, joystick_center_x);
            out->y = Joystick_Normalize(raw_y, joystick_center_y);
        }
    } else {
        // 수동 모드가 아닐 때는 조이스틱 값이 튀지 않도록 0으로 고정
        out->x = 0;
        out->y = 0;
    }

    // 버튼 상태 폴링 (Pull-up 설정이므로 누르면 LOW)
    raw_button = (HAL_GPIO_ReadPin(JOY_SW_GPIO_Port, JOY_SW_Pin) == GPIO_PIN_RESET);

    // 디바운스 로직 (지정된 횟수만큼 연속 일치 시 상태 확정)
    if (raw_button == joystick_button_last) {
        if (joystick_button_count < JOYSTICK_DEBOUNCE_COUNT) {
            joystick_button_count++;
        }
    } else {
        // 입력 변화 감지 시 카운트 리셋 (반응 속도를 위해 1부터 카운트 시작)
        joystick_button_last = raw_button;
        joystick_button_count = 1U;
    }

    was_down = joystick_button_stable;

    // 디바운스 횟수를 충족하면 상태 업데이트
    if (joystick_button_count >= JOYSTICK_DEBOUNCE_COUNT) {
        joystick_button_stable = joystick_button_last;
    }

    out->button_down = joystick_button_stable;
    out->button_pressed = (joystick_button_stable && !was_down); // 상승 엣지(새로 눌림) 판별
}
