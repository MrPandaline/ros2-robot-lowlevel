extern "C" {
#include "main.h"
#include "cmsis_os.h"
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/time_sync.h"

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
}

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/battery_state.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "peripherals.h"
#include <string.h>

/* MPU6050_ADDR, INA226_ADDR etc. — reusing the register/constant defines */

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;   /* motor PWM */
extern TIM_HandleTypeDef htim2;   /* right encoder */
extern TIM_HandleTypeDef htim3;   /* left encoder */
extern I2C_HandleTypeDef hi2c1;

/* ---------- micro-ROS entities ---------- */
static rclc_support_t support;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_executor_t executor;

static rcl_publisher_t imu_pub;
static rcl_publisher_t joint_state_pub;
static rcl_publisher_t battery_pub;
static rcl_subscription_t cmd_vel_sub;
static rcl_subscription_t servo_sub;

static sensor_msgs__msg__Imu           imu_msg;
static sensor_msgs__msg__JointState    joint_state_msg;
static sensor_msgs__msg__BatteryState  battery_msg;
static geometry_msgs__msg__Twist       cmd_vel_msg;
static std_msgs__msg__Float32MultiArray servo_msg;

static rcl_timer_t imu_timer;
static rcl_timer_t joint_state_timer;
static rcl_timer_t battery_timer;

/* Gyro zero-rate offset, in raw LSB,measured once at boot
 * in app_calibrate_gyro() and subtracted in imu_timer_callback(). */
static float g_gyro_bias_x = 0.0f;
static float g_gyro_bias_y = 0.0f;
static float g_gyro_bias_z = 0.0f;

/* Buffers for joint names */
static rosidl_runtime_c__String joint_names[2];
static char joint_name_left[]  = "left_wheel";
static char joint_name_right[] = "right_wheel";
static double joint_pos[2];
static double joint_vel[2];

/* Buffer for the array of 5 servo angles */
static float servo_data_buf[5];

/* Buffers for frame_id */
static char frame_id_imu[] = "imu_link";
static char frame_id_base_link[] = "base_link";
static char frame_id_battery[] = "battery";

/* ---------- Publish error counter, used to detect a lost session ---------- */
static volatile uint32_t g_publish_errors = 0;

static void app_publisher_result(rcl_ret_t ret)
{
    if (ret == RCL_RET_OK) {
        g_publish_errors = 0;
    } else {
        if (g_publish_errors < 1000000u) {
            g_publish_errors++;
        }
    }
}

/* last_call_time_ns passed into rclc timer callbacks is elapsed time since
 * the PREVIOUS call (jittery, ~timer period), not an absolute timestamp —
 * using it for header.stamp made message timestamps non-monotonic. Real
 * wall-clock time comes from rmw_uros_epoch_nanos() instead, valid once
 * rmw_uros_sync_session() has synced with the agent's clock. */
static void app_stamp_now(builtin_interfaces__msg__Time *stamp)
{
    int64_t now_ns = rmw_uros_epoch_nanos();
    stamp->sec = (int32_t)(now_ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(now_ns % 1000000000LL);
}

/* ---------- Timer: IMU ---------- */
static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time_ns)
{
    (void)timer;
    (void)last_call_time_ns;
    if (timer == NULL) return;

    /* Set the timestamp */
    app_stamp_now(&imu_msg.header.stamp);

    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                          I2C_MEMADD_SIZE_8BIT, buf, 14, 20) != HAL_OK) {
        return;
    }

    int16_t ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    float gx = (float)(int16_t)(((uint16_t)buf[8] << 8) | buf[9]) - g_gyro_bias_x;
    float gy = (float)(int16_t)(((uint16_t)buf[10] << 8) | buf[11]) - g_gyro_bias_y;
    float gz = (float)(int16_t)(((uint16_t)buf[12] << 8) | buf[13]) - g_gyro_bias_z;

    /* g to m/s^2, dps to rad/s */
    imu_msg.linear_acceleration.x = (ax / MPU6050_ACCEL_LSB_PER_G) * 9.80665;
    imu_msg.linear_acceleration.y = (ay / MPU6050_ACCEL_LSB_PER_G) * 9.80665;
    imu_msg.linear_acceleration.z = (az / MPU6050_ACCEL_LSB_PER_G) * 9.80665;
    imu_msg.angular_velocity.x = (gx / MPU6050_GYRO_LSB_PER_DPS) * (3.14159265f / 180.0f);
    imu_msg.angular_velocity.y = (gy / MPU6050_GYRO_LSB_PER_DPS) * (3.14159265f / 180.0f);
    imu_msg.angular_velocity.z = (gz / MPU6050_GYRO_LSB_PER_DPS) * (3.14159265f / 180.0f);

    imu_msg.orientation_covariance[0] = -1.0;

    app_publisher_result(rcl_publish(&imu_pub, &imu_msg, NULL));
}

/* ---------- Timer: encoders ---------- */
static int32_t prev_right_cnt = 0, prev_left_cnt = 0;

static void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time_ns)
{
    (void)timer;

    /* Set the timestamp */
    app_stamp_now(&joint_state_msg.header.stamp);

    int32_t right_cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t left_cnt  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

    int32_t d_right = right_cnt - prev_right_cnt;
    int32_t d_left  = left_cnt  - prev_left_cnt;
    prev_right_cnt = right_cnt;
    prev_left_cnt  = left_cnt;

    /* last_call_time_ns is the elapsed interval since the previous call —
     * correct for a dt, unlike using it as an absolute timestamp above. */
    double dt_s = last_call_time_ns / 1e9;
    if (dt_s <= 0.0) dt_s = 0.05;

    joint_pos[0] = (double)right_cnt;
    joint_pos[1] = (double)left_cnt;
    joint_vel[0] = d_right / dt_s;
    joint_vel[1] = d_left  / dt_s;

    app_publisher_result(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
}

/* ---------- Timer: battery ---------- */

/* RCL_MS_TO_NS(1000), ...) must match this constant. */
#define BATTERY_PERIOD_S 1.0f

static float g_consumed_wh = 0.0f;

static void battery_timer_callback(rcl_timer_t *timer, int64_t last_call_time_ns)
{
    (void)timer;
    (void)last_call_time_ns;

    /* Set the timestamp */
    app_stamp_now(&battery_msg.header.stamp);

    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c1, INA226_ADDR, INA226_REG_BUSV,
                          I2C_MEMADD_SIZE_8BIT, buf, 2, 20) != HAL_OK) {
        battery_msg.voltage = 0.0;
    } else {
        uint16_t bus_raw = ((uint16_t)buf[0] << 8) | buf[1];
        battery_msg.voltage = bus_raw * 1.25f / 1000.0f;
    }

    bool have_current = false;
    if (HAL_I2C_Mem_Read(&hi2c1, INA226_ADDR, INA226_REG_CURRENT,
                          I2C_MEMADD_SIZE_8BIT, buf, 2, 20) != HAL_OK) {
        battery_msg.current = 0.0;
    } else {
        int16_t cur_raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        battery_msg.current = (cur_raw * INA226_CURRENT_LSB_A);
        have_current = true;
    }

    if (have_current) {
        if (HAL_I2C_Mem_Read(&hi2c1, INA226_ADDR, INA226_REG_POWER,
                              I2C_MEMADD_SIZE_8BIT, buf, 2, 20) == HAL_OK) {
            uint16_t power_raw = ((uint16_t)buf[0] << 8) | buf[1];
            float power_w = power_raw * INA226_POWER_LSB_W;
            if (battery_msg.current < 0.0f) {
                power_w = -power_w;
            }

            g_consumed_wh += power_w * (BATTERY_PERIOD_S / 3600.0f);
            if (g_consumed_wh < 0.0f) {
                g_consumed_wh = 0.0f;
            }
            /* Approximates consumed charge in Ah using the instantaneous voltage,
             * which introduces a persistent systematic error. */
            if (battery_msg.voltage > 1.0) {
                float consumed_ah = g_consumed_wh / (float)battery_msg.voltage;
                double charge = battery_msg.design_capacity - consumed_ah;
                if (charge < 0.0) {
                    charge = 0.0;
                }
                battery_msg.charge = charge;
                battery_msg.percentage = (battery_msg.design_capacity > 0.0)
                    ? (charge / battery_msg.design_capacity)
                    : 0.0;
            }
        }
    }

    app_publisher_result(rcl_publish(&battery_pub, &battery_msg, NULL));
}

#include "motor_control_task.hpp"

/* ---------- Subscription: /cmd_vel (motors) ---------- */
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *tw = (const geometry_msgs__msg__Twist *)msgin;
    MotorController::instance().setTarget((float)tw->linear.x, (float)tw->angular.z);
}

/* ---------- Subscription: /servo_cmd (servos) ---------- */
static void servo_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *arr = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (arr->data.size != 5) {
        return;
    }

    for (size_t i = 0; i < 5; i++) {
        Periph_ServoSetAngle((uint8_t)i, arr->data.data[i]);
    }
}

/* ============================================================
 * AUTO RECONNECT MICRO-ROS
 * ============================================================ */

enum AppInitFlags {
    APP_INIT_SUPPORT       = (1u << 0),
    APP_INIT_NODE          = (1u << 1),
    APP_INIT_IMU_PUB       = (1u << 2),
    APP_INIT_JOINT_PUB     = (1u << 3),
    APP_INIT_BATTERY_PUB   = (1u << 4),
    APP_INIT_CMD_VEL_SUB   = (1u << 5),
    APP_INIT_SERVO_SUB     = (1u << 6),
    APP_INIT_IMU_TIMER     = (1u << 7),
    APP_INIT_JOINT_TIMER   = (1u << 8),
    APP_INIT_BATTERY_TIMER = (1u << 9),
    APP_INIT_EXECUTOR      = (1u << 10),
};

static uint32_t g_init_flags = 0;

#define GYRO_CAL_SAMPLES 200

/* Robot must be stationary for the ~1s this takes (200 samples * 5ms) —
 * called once at boot, right after MPU6050 wake-up. */
static void app_calibrate_gyro(void)
{
    //Settle time
	osDelay(50);

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint32_t good_samples = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        uint8_t buf[6];
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT, buf, 6, 20) == HAL_OK) {
            sum_x += (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
            sum_y += (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
            sum_z += (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
            good_samples++;
        }
        osDelay(5);
    }

    if (good_samples > 0) {
        g_gyro_bias_x = (float)sum_x / (float)good_samples;
        g_gyro_bias_y = (float)sum_y / (float)good_samples;
        g_gyro_bias_z = (float)sum_z / (float)good_samples;
    }
}

static void app_low_level_sensors_init(void)
{
    /* MPU6050 wake-up */
    uint8_t pwr_mgmt1_wake = 0x00;
    (void)HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        MPU6050_REG_PWR_MGMT1,
        I2C_MEMADD_SIZE_8BIT,
        &pwr_mgmt1_wake,
        1,
        100
    );

    app_calibrate_gyro();

    // Enable hardware averaging over 128 samples by writing the config register
    uint8_t config_buf[2] = {
        (uint8_t)(INA226_CONFIG_VALUE >> 8),
        (uint8_t)(INA226_CONFIG_VALUE & 0xFF)
    };

    (void)HAL_I2C_Mem_Write(
        &hi2c1,
        INA226_ADDR,
        INA226_REG_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        config_buf,
        2,
        100
    );

    /* INA226 calibration */
    uint8_t cal_buf[2] = {
        (uint8_t)(INA226_CAL_VALUE >> 8),
        (uint8_t)(INA226_CAL_VALUE & 0xFF)
    };

    (void)HAL_I2C_Mem_Write(
        &hi2c1,
        INA226_ADDR,
        INA226_REG_CAL,
        I2C_MEMADD_SIZE_8BIT,
        cal_buf,
        2,
        100
    );
}

static void app_init_static_messages(void)
{
    /* ---- JointState ---- */
    joint_names[0].data = joint_name_right;
    joint_names[0].size = strlen(joint_name_right);
    joint_names[0].capacity = sizeof(joint_name_right);

    joint_names[1].data = joint_name_left;
    joint_names[1].size = strlen(joint_name_left);
    joint_names[1].capacity = sizeof(joint_name_left);

    joint_state_msg.header.frame_id.data = frame_id_base_link;
    joint_state_msg.header.frame_id.size = strlen(frame_id_base_link);
    joint_state_msg.header.frame_id.capacity = sizeof(frame_id_base_link);

    joint_state_msg.header.stamp.sec = 0;
    joint_state_msg.header.stamp.nanosec = 0;

    joint_state_msg.name.data = joint_names;
    joint_state_msg.name.size = 2;
    joint_state_msg.name.capacity = 2;

    joint_state_msg.position.data = joint_pos;
    joint_state_msg.position.size = 2;
    joint_state_msg.position.capacity = 2;

    joint_state_msg.velocity.data = joint_vel;
    joint_state_msg.velocity.size = 2;
    joint_state_msg.velocity.capacity = 2;

    joint_state_msg.effort.data = NULL;
    joint_state_msg.effort.size = 0;
    joint_state_msg.effort.capacity = 0;

    /* Reset the previous encoder values so a reconnect doesn't cause a huge
     * jump in joint_vel */
    prev_right_cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    prev_left_cnt  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* ---- IMU ---- */
    imu_msg.header.frame_id.data = frame_id_imu;
    imu_msg.header.frame_id.size = strlen(frame_id_imu);
    imu_msg.header.frame_id.capacity = sizeof(frame_id_imu);

    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nanosec = 0;

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }

    imu_msg.orientation_covariance[0] = -1.0;

    /* ---- BatteryState ---- */
    battery_msg.header.frame_id.data = frame_id_battery;
    battery_msg.header.frame_id.size = strlen(frame_id_battery);
    battery_msg.header.frame_id.capacity = sizeof(frame_id_battery);

    battery_msg.header.stamp.sec = 0;
    battery_msg.header.stamp.nanosec = 0;

    battery_msg.voltage = 0.0;
    battery_msg.current = 0.0;
    /* charge/percentage are recomputed every tick in battery_timer_callback
     * from the accumulated g_consumed_wh. This is just the starting value
     * before the first real reading. */
    battery_msg.capacity = 2.0;
    battery_msg.design_capacity = 2.0;
    battery_msg.charge = battery_msg.design_capacity;
    battery_msg.percentage = 1.0;
    battery_msg.power_supply_status = 2;
    battery_msg.power_supply_health = 2;
    battery_msg.power_supply_technology = 3;
    battery_msg.present = true;

    battery_msg.serial_number.data = (char *)"LiIon_01";
    battery_msg.serial_number.size = strlen("LiIon_01");
    battery_msg.serial_number.capacity = 8;

    /* ---- Servo message ---- */
    servo_msg.data.data = servo_data_buf;
    servo_msg.data.size = 5;
    servo_msg.data.capacity = 5;

    /* Buffer for incoming servo_cmd values (before the first message) —
     * doesn't affect the actual servo start pose, */
    for (int i = 0; i < 5; i++) {
        servo_data_buf[i] = 0.0f;
    }
}

static bool app_micro_ros_init(void)
{
    rcl_ret_t rc = RCL_RET_OK;

    g_init_flags = 0;
    g_publish_errors = 0;

    app_init_static_messages();

    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_SUPPORT;

    rc = rclc_node_init_default(&node, "stm32_node", "", &support);
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_NODE;

    /* ===== PUBLISHERS ===== */

    rc = rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_IMU_PUB;

    rc = rclc_publisher_init_default(
        &joint_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_JOINT_PUB;

    rc = rclc_publisher_init_default(
        &battery_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery_state"
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_BATTERY_PUB;

    /* ===== SUBSCRIBERS ===== */

    rc = rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_CMD_VEL_SUB;

    rc = rclc_subscription_init_default(
        &servo_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_cmd"
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_SERVO_SUB;

    /* ===== TIMERS ===== */

    rc = rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(100),
        imu_timer_callback
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_IMU_TIMER;

    rc = rclc_timer_init_default(
        &joint_state_timer,
        &support,
        RCL_MS_TO_NS(100),
        joint_state_timer_callback
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_JOINT_TIMER;

    rc = rclc_timer_init_default(
        &battery_timer,
        &support,
        RCL_MS_TO_NS(1000),
        battery_timer_callback
    );
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_BATTERY_TIMER;

    /* ===== EXECUTOR ===== */

    rc = rclc_executor_init(&executor, &support.context, 5, &allocator);
    if (rc != RCL_RET_OK) {
        return false;
    }
    g_init_flags |= APP_INIT_EXECUTOR;

    rc = rclc_executor_add_timer(&executor, &imu_timer);
    if (rc != RCL_RET_OK) {
        return false;
    }

    rc = rclc_executor_add_timer(&executor, &joint_state_timer);
    if (rc != RCL_RET_OK) {
        return false;
    }

    rc = rclc_executor_add_timer(&executor, &battery_timer);
    if (rc != RCL_RET_OK) {
        return false;
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        return false;
    }

    rc = rclc_executor_add_subscription(
        &executor,
        &servo_sub,
        &servo_msg,
        &servo_callback,
        ON_NEW_DATA
    );
    if (rc != RCL_RET_OK) {
        return false;
    }

    return true;
}

static void app_ignore_rcl_ret(rcl_ret_t ret)
{
    (void)ret;
}

static void app_micro_ros_fini(void)
{
    if (g_init_flags & APP_INIT_EXECUTOR) {
        app_ignore_rcl_ret(rclc_executor_fini(&executor));
    }

    if (g_init_flags & APP_INIT_BATTERY_TIMER) {
        app_ignore_rcl_ret(rcl_timer_fini(&battery_timer));
    }

    if (g_init_flags & APP_INIT_JOINT_TIMER) {
        app_ignore_rcl_ret(rcl_timer_fini(&joint_state_timer));
    }

    if (g_init_flags & APP_INIT_IMU_TIMER) {
        app_ignore_rcl_ret(rcl_timer_fini(&imu_timer));
    }

    if (g_init_flags & APP_INIT_SERVO_SUB) {
        app_ignore_rcl_ret(rcl_subscription_fini(&servo_sub, &node));
    }

    if (g_init_flags & APP_INIT_CMD_VEL_SUB) {
        app_ignore_rcl_ret(rcl_subscription_fini(&cmd_vel_sub, &node));
    }

    if (g_init_flags & APP_INIT_BATTERY_PUB) {
        app_ignore_rcl_ret(rcl_publisher_fini(&battery_pub, &node));
    }

    if (g_init_flags & APP_INIT_JOINT_PUB) {
        app_ignore_rcl_ret(rcl_publisher_fini(&joint_state_pub, &node));
    }

    if (g_init_flags & APP_INIT_IMU_PUB) {
        app_ignore_rcl_ret(rcl_publisher_fini(&imu_pub, &node));
    }

    if (g_init_flags & APP_INIT_NODE) {
        app_ignore_rcl_ret(rcl_node_fini(&node));
    }

    if (g_init_flags & APP_INIT_SUPPORT) {
        app_ignore_rcl_ret(rclc_support_fini(&support));
    }

    g_init_flags = 0;
    g_publish_errors = 0;
}

static bool app_agent_available(void)
{
    return rmw_uros_ping_agent(300, 1) == RMW_RET_OK;
}

static void app_wait_for_agent(void)
{
    for (;;) {
        if (app_agent_available()) {
            return;
        }

        osDelay(50);
    }
}

static bool app_micro_ros_spin_until_lost(void)
{
    uint32_t loop_counter = 0;
    uint32_t ping_fail_counter = 0;

    for (;;) {
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        /* RCL_RET_TIMEOUT is fine if there are no new events */
        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
            return false;
        }

        if (g_publish_errors > 10u) {
            return false;
        }

        /* ping the agent roughly every 200ms */
        if (++loop_counter >= 10u) {
            loop_counter = 0;

            if (app_agent_available()) {
                ping_fail_counter = 0;
            } else {
                ping_fail_counter++;

                /* 2 consecutive failed pings counts as a drop (~400-500ms) */
                if (ping_fail_counter >= 2u) {
                    return false;
                }
            }
        }

        osDelay(10);
    }
}

/* ---------- Entry point ---------- */
extern "C" void cpp_entry_point(void)
{
    MotorControl_Init();
    app_low_level_sensors_init();

    allocator = rcl_get_default_allocator();

    if (rmw_uros_set_custom_transport(
            true,
            (void *)&huart1,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read) != RMW_RET_OK) {
        for (;;) { osDelay(1000); }
    }

    for (;;) {
        app_wait_for_agent();

        if (app_micro_ros_init()) {
            /* Sync epoch time with the agent so app_stamp_now() gives real
             * wall-clock time, not just time-since-boot. Best-effort — if it
             * fails, header.stamp will read as time since 1970 without the
             * offset, but stays monotonic either way. */
            (void)rmw_uros_sync_session(1000);
            app_micro_ros_spin_until_lost();
        }

        app_micro_ros_fini();
    }
}
