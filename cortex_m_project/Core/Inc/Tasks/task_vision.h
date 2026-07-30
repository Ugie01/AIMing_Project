#ifndef TASK_VISION_H_
#define TASK_VISION_H_

#ifdef __cplusplus
extern "C" {
#endif

// ==============================================================================
// 전처리 모드 설정 매크로 및 변수
// ==============================================================================
typedef enum {
    PREPROCESS_NONE = 0, // 원본 RGB888 그대로 사용 (전처리 없음, 속도 최상)
    PREPROCESS_SIMPLE = 1, // 기존 단순 RGB Threshold 방식
    PREPROCESS_HSV = 2  // HSV 기반 정밀 Red 마스킹 (RGB 포맷으로 재구성)
} PreprocessMode_t;

void VisionTask(void);
float Vision_GetCurrentFPS(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_VISION_H_ */
