#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void MotorControl_TaskFunc(void *argument);
void MotorControl_SetTarget(float linear_mps, float angular_radps);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_TASK_H */
