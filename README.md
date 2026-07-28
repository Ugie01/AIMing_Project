
## 💻 3. `main.c` 활용 예시 코드

분리된 모듈을 STM32 메인 루프에서 어떻게 호출하는지 보여주는 예시입니다.

```c
int main(void)
{
 

  while (1)
  {
    // 1. 랜덤한 타이밍(2~6틱 = 40ms~120ms)마다 새로운 AI(더미) 타겟 좌표를 가져옴
    if (ai_delay_counter == 0)
    {
        Get_Dummy_Target_Coords(&target_x, &target_y);
        random_num = rand() % (6 - 2 + 1) + 2; // 다음 갱신까지 대기할 틱 수 (2~6)
    }

    ai_delay_counter++;
    if (ai_delay_counter >= random_num)
    {
        ai_delay_counter = 0;
    }

    // 2. 현재 위치와 타겟 간의 거리 계산
    float distance = sqrtf(powf(target_x - current_x, 2) + powf(target_y - current_y, 2));

    // 3. 거리에 따른 동적 smoothing_alpha 설정
    float smoothing_alpha;
    if (distance > 50.0f) {
        smoothing_alpha = 0.4f;  // 멀리 떨어져 있을 때는 빠르게 추종
    } else {
        smoothing_alpha = 0.15f; // 가까워지면 부드럽게 안착 (진동 방지)
    }

    // 4. EMA 필터 적용
    current_x = current_x + smoothing_alpha * (target_x - current_x);
    current_y = current_y + smoothing_alpha * (target_y - current_y);

    // 5. 모니터링 출력
    printf("Target(%.1f, %.1f) | EMA(%.1f, %.1f) | Alpha: %.2f | Pan: %d | Tilt: %d\r\n", 
           target_x, target_y, 
           current_x, current_y, 
           smoothing_alpha,
           (int)pan_pid.current, (int)tilt_pid.current);

    // 6. PID 제어기로 투입
    Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, (int)current_x, (int)current_y, &htim2);

    // 7. 20ms 주기 유지
    HAL_Delay(20);
  }
}

```

---


