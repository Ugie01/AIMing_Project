/*
 * joystick.c
 *
 * 2축 아날로그 조이스틱 + 푸시버튼 입력.
 *
 *   PA0  JOY_X   ADC1_INP16  Rank 1
 *   PA1  JOY_Y   ADC1_INP17  Rank 2
 *   PA2  JOY_SW  GPIO_Input + PULLUP  ->  눌림 = LOW (active-low)
 *
 * ADC1 은 DMA/인터럽트 없이 폴링 전용으로 설정되어 있다
 * (ConversionDataManagement = ADC_CONVERSIONDATA_DR, multimode = INDEPENDENT).
 * 이 조합에서는 HAL_ADC_PollForConversion() 이 rank 마다 EOC 를 기다리므로
 * 순서대로 두 번 읽으면 Rank1(X), Rank2(Y) 를 얻는다.
 *
 * 2채널 변환에 약 7.8us 밖에 걸리지 않지만 폴링은 블로킹이므로,
 * AI 추론이 도는 VisionTask(osPriorityHigh) 가 아니라
 * MotorTask(osPriorityNormal) 에서 호출해야 한다.
 */

#include "joystick.h"
#include "adc.h"

/* ADC 변환 타임아웃. 정상이면 수 us 안에 끝나므로 넉넉한 값이다. */
#define JOYSTICK_ADC_TIMEOUT_MS   10U

/* 중립값 산출에 쓸 부팅 시 샘플 수 */
#define JOYSTICK_CALIB_SAMPLES    32U

/*
 * 데드존. ADC LSB 단위이며 12bit 풀스케일 4096 기준 약 4.9% 다.
 *
 * 스틱을 놓았을 때의 미세한 떨림과 캘리브레이션 잔차를 흡수한다.
 * 이 값을 넘어서면 데드존 경계부터 다시 0 으로 시작하도록 재정규화하므로
 * 데드존을 벗어나는 순간 값이 튀지 않는다.
 */
#define JOYSTICK_DEADZONE         200

/*
 * 버튼 디바운스. Joystick_Read() 호출 횟수 기준이다.
 *
 * 같은 값이 연속 2회 읽혀야 상태를 확정한다. 권장 주기 20ms 기준으로
 * 처음 변화를 본 시점에서 약 20ms 뒤에 확정되며, 탁틱 스위치의 통상
 * 바운스(1~10ms)보다 길어 채터링을 걸러낸다.
 */
#define JOYSTICK_DEBOUNCE_COUNT   2U

/* 부팅 시 측정한 중립 ADC 값. 캘리브레이션 실패 시 이론값으로 둔다. */
static uint16_t joystick_center_x = 2048U;
static uint16_t joystick_center_y = 2048U;

/* 디바운스 상태 */
static bool joystick_button_stable = false; /* 확정된 눌림 상태 */
static bool joystick_button_last = false; /* 직전 원시 입력   */
static uint8_t joystick_button_count = 0U; /* 연속 일치 횟수   */

/**
 * @brief ADC1 을 1회 구동해 Rank1(X), Rank2(Y) 를 읽는다.
 *
 * @return 두 채널 모두 정상 변환되면 true.
 */
static bool Joystick_SampleRaw(uint16_t *raw_x, uint16_t *raw_y) {
	bool ok = false;

	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		return false;
	}

	/* Rank 1 : JOY_X (ADC_CHANNEL_16) */
	if (HAL_ADC_PollForConversion(&hadc1, JOYSTICK_ADC_TIMEOUT_MS) == HAL_OK) {
		*raw_x = (uint16_t) HAL_ADC_GetValue(&hadc1);

		/* Rank 2 : JOY_Y (ADC_CHANNEL_17) */
		if (HAL_ADC_PollForConversion(&hadc1, JOYSTICK_ADC_TIMEOUT_MS)
				== HAL_OK) {
			*raw_y = (uint16_t) HAL_ADC_GetValue(&hadc1);
			ok = true;
		}
	}

	/*
	 * 성공/실패와 무관하게 정지시킨다.
	 * 중간에 실패한 경우 시퀀서가 Rank2 에 멈춰 있을 수 있는데,
	 * 그대로 두면 다음 호출에서 X 와 Y 가 뒤바뀐 채로 읽힌다.
	 */
	(void) HAL_ADC_Stop(&hadc1);

	return ok;
}

/**
 * @brief 원시 ADC 값을 중립 기준 ±JOYSTICK_AXIS_MAX 로 정규화한다.
 *
 * 데드존 안이면 0 을 돌려주고, 벗어나면 데드존 경계를 새 원점으로 삼아
 * 다시 0 부터 올라가도록 재정규화한다.
 */
static int16_t Joystick_Normalize(uint16_t raw, uint16_t center) {
	int32_t value = (int32_t) raw - (int32_t) center;
	int32_t span;
	int32_t magnitude;

	if ((value > -JOYSTICK_DEADZONE) && (value < JOYSTICK_DEADZONE)) {
		return 0;
	}

	if (value > 0) {
		/* 중립에서 위쪽으로 남은 거리 */
		span = (int32_t) 4095 - (int32_t) center - JOYSTICK_DEADZONE;
		magnitude = value - JOYSTICK_DEADZONE;
	} else {
		/* 중립에서 아래쪽으로 남은 거리 */
		span = (int32_t) center - JOYSTICK_DEADZONE;
		magnitude = -value - JOYSTICK_DEADZONE;
	}

	if (span <= 0) {
		/* 중립이 한쪽 끝에 붙어 있는 비정상 캘리브레이션 방어 */
		return 0;
	}

	if (magnitude > span) {
		magnitude = span;
	}

	magnitude = (magnitude * JOYSTICK_AXIS_MAX) / span;

	if (magnitude > JOYSTICK_AXIS_MAX) {
		magnitude = JOYSTICK_AXIS_MAX;
	}

	return (value > 0) ? (int16_t) magnitude : (int16_t) (-magnitude);
}

void Joystick_Init(void) {
	uint32_t sum_x = 0U;
	uint32_t sum_y = 0U;
	uint32_t taken = 0U;
	uint32_t i;

	/*
	 * ADC 오프셋 캘리브레이션.
	 * H7 은 이걸 하지 않으면 오프셋 오차가 수십~수백 LSB 에 달한다.
	 */
	if (HAL_ADCEx_Calibration_Start(&hadc1,
	ADC_CALIB_OFFSET,
	ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}

	/* 스틱을 놓은 상태라고 가정하고 중립값을 측정한다 */
	for (i = 0U; i < JOYSTICK_CALIB_SAMPLES; i++) {
		uint16_t raw_x;
		uint16_t raw_y;

		if (Joystick_SampleRaw(&raw_x, &raw_y)) {
			sum_x += raw_x;
			sum_y += raw_y;
			taken++;
		}
	}

	if (taken > 0U) {
		joystick_center_x = (uint16_t) (sum_x / taken);
		joystick_center_y = (uint16_t) (sum_y / taken);
	}
	/* 한 번도 못 읽었으면 이론 중앙값(2048)을 그대로 쓴다 */

	/* 버튼 상태를 현재 입력으로 초기화해 부팅 직후 헛 엣지를 막는다 */
	joystick_button_stable = (HAL_GPIO_ReadPin(JOY_SW_GPIO_Port, JOY_SW_Pin)
			== GPIO_PIN_RESET);
	joystick_button_last = joystick_button_stable;
	joystick_button_count = 0U;
}

void Joystick_Read(JoystickInput_t *out) {
	uint16_t raw_x = joystick_center_x;
	uint16_t raw_y = joystick_center_y;
	bool raw_button;
	bool was_down;

	if (out == NULL) {
		return;
	}

	/* --- 축 --- */
	if (Joystick_SampleRaw(&raw_x, &raw_y)) {
		out->x = Joystick_Normalize(raw_x, joystick_center_x);
		out->y = Joystick_Normalize(raw_y, joystick_center_y);
	} else {
		/* 변환 실패 시 중립으로 취급해 모터가 제멋대로 움직이지 않게 한다 */
		out->x = 0;
		out->y = 0;
	}

	/* --- 버튼 (active-low) --- */
	raw_button = (HAL_GPIO_ReadPin(JOY_SW_GPIO_Port, JOY_SW_Pin)
			== GPIO_PIN_RESET);

	if (raw_button == joystick_button_last) {
		if (joystick_button_count < JOYSTICK_DEBOUNCE_COUNT) {
			joystick_button_count++;
		}
	} else {
		/*
		 * 입력이 바뀌었다. 이 샘플 자체가 새 상태의 첫 번째 증거이므로
		 * 0 이 아니라 1 에서 다시 센다. 0 으로 두면 확정이 한 샘플씩
		 * 밀려 버튼 반응이 느려진다.
		 */
		joystick_button_last = raw_button;
		joystick_button_count = 1U;
	}

	was_down = joystick_button_stable;

	if (joystick_button_count >= JOYSTICK_DEBOUNCE_COUNT) {
		joystick_button_stable = joystick_button_last;
	}

	out->button_down = joystick_button_stable;
	out->button_pressed = (joystick_button_stable && !was_down);
}
