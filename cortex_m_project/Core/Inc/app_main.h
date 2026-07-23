#ifndef APP_MAIN_H_
#define APP_MAIN_H_

// C++ 컴파일러가 이 헤더를 읽을 때만 extern "C"를 적용하여
// C 언어인 main.c에서도 이 함수를 찾을 수 있게 해줍니다.
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h" // HAL 라이브러리 및 GPIO/UART 정의 사용

/**
 * @brief C++ 애플리케이션 진입점 함수
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H_ */
