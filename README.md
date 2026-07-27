
## 💻 3. `main.c` 활용 예시 코드

분리된 모듈을 STM32 메인 루프에서 어떻게 호출하는지 보여주는 예시입니다.

```c
int main(void)
{
 

  while (1)
  {
    // 테스트를 위해 95x95 안에서 움직이는 가상의 타겟 좌표 생성 (예: 원 형태로 움직임)
    Get_Dummy_Target_Coords(&target_x, &target_y);
    
    // 프레임 좌표 기반 직접 반영 함수 호출
    Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, target_x, target_y, &htim2);

    // 디버깅 출력
    // printf("Target(%.1f, %.1f) | Pan_Cur: %d | Tilt_Cur: %d\r\n", 
           target_x, target_y, (int)pan_pid.current, (int)tilt_pid.current);

      // 서보모터 표준 제어 주기 (20ms)
      HAL_Delay(20);
  }
}

```

---


