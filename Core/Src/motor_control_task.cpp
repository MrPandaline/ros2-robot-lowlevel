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

namespace {

struct CmdVelTarget {
    float linear;
    float angular;
    TickType_t timestamp;
};

constexpr float kTrackWidthM = 0.1825f;
constexpr float kMaxLinearMps = 0.5f;
constexpr uint32_t kCmdVelTimeoutMs = 500;
constexpr uint32_t kControlPeriodMs = 20;  // 50 Hz

int8_t speedToDir(float speed_mps)
{
    if (!std::isfinite(speed_mps)) {
        return 0;
    }

    if (speed_mps > 0.001f) {
        return 1;
    }

    if (speed_mps < -0.001f) {
        return -1;
    }

    return 0;
}

uint16_t speedToDuty(float speed_mps)
{
    if (!std::isfinite(speed_mps)) {
        return 0;
    }

    float duty = std::fabs(speed_mps) / kMaxLinearMps * 100.0f;

    if (duty > 100.0f) {
        duty = 100.0f;
    }

    return static_cast<uint16_t>(duty);
}

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

        const int8_t left_dir = speedToDir(left_speed);
        const int8_t right_dir = speedToDir(right_speed);

        const uint16_t left_duty = speedToDuty(left_speed);
        const uint16_t right_duty = speedToDuty(right_speed);

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
