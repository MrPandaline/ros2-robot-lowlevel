extern "C" {
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "peripherals.h"
}

#include "motor_control_task.hpp"

#include <cmath>
#include <cstdint>

/* TIM2 = right encoder (32-bit counter), TIM5 = left encoder (32-bit
 * counter) — same mapping as Core/Src/app_main.cpp's joint_state
 * publisher. */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

namespace {

struct CmdVelTarget {
    float linear;
    float angular;
    TickType_t timestamp;
};

// kTrackWidthM needs calibration for different robots
constexpr float kTrackWidthM = 0.1967f;
constexpr uint32_t kCmdVelTimeoutMs = 350;
constexpr uint32_t kControlPeriodMs = 20;  // 50 Hz
constexpr float kControlPeriodS = kControlPeriodMs / 1000.0f;

/* wheel_diameter_m/ticks_per_rev must be
 * kept in sync wheel_odometry.py*/
constexpr float kWheelDiameterM = 0.065f;
constexpr float kTicksPerRev = 32.0f;
constexpr float kWheelCircumferenceM = kWheelDiameterM * 3.14159265f;

/* PID gains — starting point, needs empirical tuning on the real robot */
constexpr float kPidKp = 30.0f;
constexpr float kPidKi = 103.0f;
constexpr float kPidKd = 0.0f;

constexpr float kPidOutputMax = 100.0f;
constexpr float kIntegralMax = kPidKi > 1e-6f ? (kPidOutputMax / kPidKi) : 0.0f;

struct PidState {
    float integral = 0.0f;
    float prev_error = 0.0f;
};

/* Linear approximation of duty to motor radial speed relationship —
 * needs empirical tuning for different drives */
constexpr float kFeedforwardOffsetPercent = 3.75f;
constexpr float kFeedforwardSlopePercentPerMps = 195.2f;

float feedforwardDuty(float target_mps)
{
    if (std::fabs(target_mps) < 1e-4f) {
        return 0.0f;
    }

    const float magnitude =
        kFeedforwardOffsetPercent + kFeedforwardSlopePercentPerMps * std::fabs(target_mps);

    return (target_mps > 0.0f) ? magnitude : -magnitude;
}

float pidStep(PidState &state, float target_mps, float measured_mps, float dt_s)
{
    const float error = target_mps - measured_mps;

    state.integral += error * dt_s;
    if (state.integral > kIntegralMax) {
        state.integral = kIntegralMax;
    } else if (state.integral < -kIntegralMax) {
        state.integral = -kIntegralMax;
    }

    const float derivative = (dt_s > 0.0f) ? (error - state.prev_error) / dt_s : 0.0f;
    state.prev_error = error;

    float output = feedforwardDuty(target_mps) +
                    kPidKp * error + kPidKi * state.integral + kPidKd * derivative;

    if (output > kPidOutputMax) {
        output = kPidOutputMax;
    } else if (output < -kPidOutputMax) {
        output = -kPidOutputMax;
    }

    return output;
}

int8_t speedToDir(float signed_duty)
{
    if (!std::isfinite(signed_duty)) {
        return 0;
    }

    if (signed_duty > 0.5f) {
        return 1;
    }

    if (signed_duty < -0.5f) {
        return -1;
    }

    return 0;
}

uint16_t signedDutyToMagnitude(float signed_duty)
{
    if (!std::isfinite(signed_duty)) {
        return 0;
    }

    float duty = std::fabs(signed_duty);
    if (duty > 100.0f) {
        duty = 100.0f;
    }

    return static_cast<uint16_t>(duty);
}

/* output speed smoothing sliding window length*/
constexpr int kSpeedFilterPeriods = 3;

struct EncoderHistory {
    uint32_t left_raw[kSpeedFilterPeriods] = {};
    uint32_t right_raw[kSpeedFilterPeriods] = {};
    int next_slot = 0;
};

}

MotorController& MotorController::instance()
{
    static MotorController inst;
    return inst;
}

void MotorController::init()
{
    if (cmd_queue_ == nullptr) {
        cmd_queue_ = xQueueCreate(1, sizeof(CmdVelTarget));
    }
}

void MotorController::setTarget(float linear_mps, float angular_radps)
{
    QueueHandle_t q = static_cast<QueueHandle_t>(cmd_queue_);

    if (q == nullptr) {
        return;
    }

    CmdVelTarget t{linear_mps, angular_radps, xTaskGetTickCount()};
    xQueueOverwrite(q, &t);}

void MotorController::run()
{
    init();

    QueueHandle_t q = static_cast<QueueHandle_t>(cmd_queue_);

    if (q == nullptr) {
        for (;;) {
        	osDelay(1000);
        }
    }

    CmdVelTarget target{0.0f, 0.0f, 0};
    CmdVelTarget incoming;

    TickType_t last_cmd_tick = xTaskGetTickCount();
    TickType_t last_wake_tick = xTaskGetTickCount();

    const TickType_t period_ticks_raw = pdMS_TO_TICKS(kControlPeriodMs);
    const TickType_t period_ticks = (period_ticks_raw == 0) ? 1 : period_ticks_raw;

    PidState left_pid;
    PidState right_pid;

    EncoderHistory history;
    const uint32_t initial_left_raw = __HAL_TIM_GET_COUNTER(&htim5);
    const uint32_t initial_right_raw = __HAL_TIM_GET_COUNTER(&htim2);
    for (int i = 0; i < kSpeedFilterPeriods; i++) {
        history.left_raw[i] = initial_left_raw;
        history.right_raw[i] = initial_right_raw;
    }

    constexpr float kFilterWindowS = kControlPeriodS * kSpeedFilterPeriods;

    for (;;) {
        if (xQueueReceive(q, &incoming, 0) == pdTRUE) {
            target = incoming;
            last_cmd_tick = xTaskGetTickCount();
        }

        if ((xTaskGetTickCount() - last_cmd_tick) >= pdMS_TO_TICKS(kCmdVelTimeoutMs)) {
            target.linear = 0.0f;
            target.angular = 0.0f;
        }

        const float left_speed =
            target.linear - target.angular * kTrackWidthM * 0.5f;

        const float right_speed =
            target.linear + target.angular * kTrackWidthM * 0.5f;

        const uint32_t left_raw = __HAL_TIM_GET_COUNTER(&htim5);
        const uint32_t right_raw = __HAL_TIM_GET_COUNTER(&htim2);

        /* Compare against the reading from kSpeedFilterPeriods periods ago */
        const uint32_t left_raw_oldest = history.left_raw[history.next_slot];
        const uint32_t right_raw_oldest = history.right_raw[history.next_slot];

        const int32_t d_left_ticks = (int32_t)(left_raw - left_raw_oldest);
        /* TIM2's count direction matches the motor's "forward" direction as
         * currently wired */
        const int32_t d_right_ticks = (int32_t)(right_raw - right_raw_oldest);

        history.left_raw[history.next_slot] = left_raw;
        history.right_raw[history.next_slot] = right_raw;
        history.next_slot = (history.next_slot + 1) % kSpeedFilterPeriods;

        const float left_measured_mps =
            (float)d_left_ticks / kTicksPerRev * kWheelCircumferenceM / kFilterWindowS;
        const float right_measured_mps =
            (float)d_right_ticks / kTicksPerRev * kWheelCircumferenceM / kFilterWindowS;

        const float left_output = pidStep(left_pid, left_speed, left_measured_mps, kControlPeriodS);
        const float right_output = pidStep(right_pid, right_speed, right_measured_mps, kControlPeriodS);

        const int8_t left_dir = speedToDir(left_output);
        const int8_t right_dir = speedToDir(right_output);

        const uint16_t left_duty = signedDutyToMagnitude(left_output);
        const uint16_t right_duty = signedDutyToMagnitude(right_output);

        Periph_MotorSet(0, left_dir, left_duty);
        Periph_MotorSet(1, right_dir, right_duty);

        vTaskDelayUntil(&last_wake_tick, period_ticks);
    }
}

extern "C" void MotorControl_TaskFunc(void *argument)
{
    (void)argument;

    MotorController::instance().init();
    MotorController::instance().run();

}

extern "C" void MotorControl_SetTarget(float linear_mps, float angular_radps)
{
    MotorController::instance().setTarget(linear_mps, angular_radps);
}

extern "C" void MotorControl_Init(void)
{
    MotorController::instance().init();
}
