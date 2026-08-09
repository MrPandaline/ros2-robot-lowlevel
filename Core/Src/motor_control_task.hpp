#ifndef MOTOR_CONTROL_TASK_HPP_
#define MOTOR_CONTROL_TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif

void MotorControl_Init(void);
void MotorControl_SetTarget(float linear_mps, float angular_radps);
void MotorControl_TaskFunc(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class MotorController {
public:
    static MotorController& instance();

    void init();
    void setTarget(float linear_mps, float angular_radps);
    void run();

private:
    MotorController() = default;

    void *cmd_queue_ = nullptr;
};

#endif // __cplusplus

#endif // MOTOR_CONTROL_TASK_HPP_
