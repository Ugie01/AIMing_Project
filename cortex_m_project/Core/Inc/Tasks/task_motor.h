#ifndef TASK_MOTOR_H_
#define TASK_MOTOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MotorTask(void);
uint8_t Motor_GetTrackState(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MOTOR_H_ */
