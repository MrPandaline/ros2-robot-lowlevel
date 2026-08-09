#ifndef PERIPHERALS_H
#define PERIPHERALS_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Periph_MotorSet(uint8_t is_left, int8_t direction, uint16_t duty_percent);
void Periph_ServoSetAngle(uint8_t servo_index, float angle_deg);
void Periph_ServoStartAll(void);

/* ---------- INA226 registers and constants ---------- */
#define INA226_ADDR          (0x40 << 1)
#define INA226_REG_CONFIG    0x00
#define INA226_REG_SHUNTV    0x01
#define INA226_REG_BUSV      0x02
#define INA226_REG_POWER     0x03
#define INA226_REG_CURRENT   0x04
#define INA226_REG_CAL       0x05
#define INA226_REG_MANUF_ID  0xFE
#define INA226_REG_DIE_ID    0xFF

#define INA226_CAL_VALUE     5120
#define INA226_CURRENT_LSB_A 0.0001f
#define INA226_POWER_LSB_W   (25.0f * INA226_CURRENT_LSB_A)

#define INA226_CONFIG_VALUE  0x4927

#define MPU6050_ADDR          (0x68 << 1)
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_PWR_MGMT1    0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

#define MPU6050_ACCEL_LSB_PER_G   16384.0f  /* at default sensitivity +-2g */
#define MPU6050_GYRO_LSB_PER_DPS   131.0f   /* at default sensitivity +-250 dps */

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERALS_H */
