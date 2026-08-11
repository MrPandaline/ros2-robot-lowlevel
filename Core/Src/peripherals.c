/*
 * peripherals.c
 *
 * Pins and timers match the STM32F411CEUx pinout:
 *   LEFT_AIN_1  = PC13   LEFT_AIN_2  = PC14
 *   RIGHT_AIN1  = PC15   RIGHT_AIN2  = PA3
 *   STBY        = PA4
 *   TIM1_CH1_PWM_LEFT  = PA8   TIM1_CH2_PWM_RIGHT = PA9
 */

#include "peripherals.h"

extern TIM_HandleTypeDef htim1;   /* motor PWM */
extern TIM_HandleTypeDef htim4;   /* servos */
extern TIM_HandleTypeDef htim5;   /* servos */

/* ---------- TB6612FNG driver control ---------- */
static void motor_stby(uint8_t enable)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Periph_MotorSet(uint8_t is_left, int8_t direction, uint16_t duty_percent)
{
	motor_stby(1);

	if (duty_percent > 100) {
		duty_percent = 100;
	}

    GPIO_TypeDef *ain1_port = is_left ? LEFT_AIN_1_GPIO_Port : RIGHT_AIN1_GPIO_Port;
    uint16_t      ain1_pin  = is_left ? LEFT_AIN_1_Pin       : RIGHT_AIN1_Pin;
    GPIO_TypeDef *ain2_port = is_left ? LEFT_AIN_2_GPIO_Port : RIGHT_AIN2_GPIO_Port;
    uint16_t      ain2_pin  = is_left ? LEFT_AIN_2_Pin       : RIGHT_AIN2_Pin;

    if (direction > 0) {
        HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_RESET);
    } else if (direction < 0) {
        HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_RESET);
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t ccr = (arr * duty_percent) / 100;
    if (is_left) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
    }
}

/* ---------- Servos ---------- */

/* PWM range stretched to cover the full angle range: 500-2500us. */
#define SERVO_PULSE_0_US    500u
#define SERVO_PULSE_180_US  2500u
#define SERVO_RAW_MIN_US    500u
#define SERVO_RAW_MAX_US    2500u

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} servo_ch_t;

static const servo_ch_t k_servos[] = {
    { &htim4, TIM_CHANNEL_1 },
    { &htim4, TIM_CHANNEL_2 },
    { &htim5, TIM_CHANNEL_1 },
    { &htim5, TIM_CHANNEL_2 },
    { &htim5, TIM_CHANNEL_3 },
};
#define SERVO_COUNT (sizeof(k_servos) / sizeof(k_servos[0]))

static void servo_write_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

static void servo_write_deg(TIM_HandleTypeDef *htim, uint32_t channel, int deg)
{
    if (deg < -110) deg = -110;
    if (deg > 110)  deg = 110;
    int pulse_us = (int)SERVO_PULSE_0_US +
                   (deg + 90) * (int)(SERVO_PULSE_180_US - SERVO_PULSE_0_US) / 180;
    if (pulse_us < (int)SERVO_RAW_MIN_US) pulse_us = (int)SERVO_RAW_MIN_US;
    if (pulse_us > (int)SERVO_RAW_MAX_US) pulse_us = (int)SERVO_RAW_MAX_US;
    servo_write_us(htim, channel, (uint32_t)pulse_us);
}

void Periph_ServoSetAngle(uint8_t servo_index, float angle_deg)
{
    if (servo_index >= SERVO_COUNT) return;
    servo_write_deg(k_servos[servo_index].htim, k_servos[servo_index].channel, (int)angle_deg);
}

/* Manipulator start angles (S1..S5), tuned via arm_teleop.py. */
static const int k_servo_start_deg[SERVO_COUNT] = { -20, 30, 0, -90, 0 };

void Periph_ServoStartAll(void)
{
    for (size_t i = 0; i < SERVO_COUNT; i++) {
        /* Valid pulse first, then PWM — otherwise Pulse=0 before the first
         * servo_write_deg(), out of the servo's range. */
        servo_write_deg(k_servos[i].htim, k_servos[i].channel, k_servo_start_deg[i]);
        HAL_TIM_PWM_Start(k_servos[i].htim, k_servos[i].channel);
    }
}
