#ifndef IMU_H_
#define IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern uint32_t previous_tick;

#define MADGWICK_BETA 0.1f // Filtre kazanç katsayısı

#define ICM42670P_I2C_ADDR         (0x68 << 1)
#define ICM42670P_WHO_AM_I         0x75
#define ICM42670P_WHO_AM_I_VALUE   0x67
#define ICM42670P_PWR_MGMT0        0x1F
#define ICM42670P_GYRO_CONFIG0     0x20
#define ICM42670P_ACCEL_CONFIG0    0x21

#define ICM42670P_TEMP_DATA1       0x09
#define ICM42670P_TEMP_DATA0       0x0A
#define ICM42670P_ACCEL_DATA_X1    0x0B
#define ICM42670P_ACCEL_DATA_X0    0x0C
#define ICM42670P_ACCEL_DATA_Y1    0x0D
#define ICM42670P_ACCEL_DATA_Y0    0x0E
#define ICM42670P_ACCEL_DATA_Z1    0x0F
#define ICM42670P_ACCEL_DATA_Z0    0x10
#define ICM42670P_GYRO_DATA_X1     0x11 // MSB
#define ICM42670P_GYRO_DATA_X0     0x12 // LSB
#define ICM42670P_GYRO_DATA_Y1     0x13 // MSB
#define ICM42670P_GYRO_DATA_Y0     0x14 // LSB
#define ICM42670P_GYRO_DATA_Z1     0x15 // MSB
#define ICM42670P_GYRO_DATA_Z0     0x16 // LSB
#define ICM42670P_REG_BANK_SEL     0x76 // Bank seçme register'ı
#define ACCEL_SENSITIVITY_16G      (32768.0f / 16.0f)  // LSB/g
#define ACCEL_SCALE_FACTOR_16G     (1.0f / ACCEL_SENSITIVITY_16G) // g/LSB
// Gyro ölçek faktörü (±2000 dps için)
#define GYRO_SENSITIVITY_2000DPS   (32768.0f / 2000.0f) // LSB/dps
#define GYRO_SCALE_FACTOR_2000DPS  (1.0f / GYRO_SENSITIVITY_2000DPS) // dps/LSB
#define COMPLEMENTARY_FILTER_ALPHA 0.98f



HAL_StatusTypeDef ICM42670P_ReadRegister(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ICM42670P_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ICM42670P_Init(void);
HAL_StatusTypeDef ICM42670P_ReadBurst(uint8_t start_reg, uint8_t *data, uint16_t length);
HAL_StatusTypeDef ICM42670P_ReadAccelGyroData(ImuData *imu_data);
float calculate_angle_from_quaternion(float q_w, float q_x, float q_y, float q_z);
void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void calibrate_gyro_offsets(ImuData *imu_data);
void set_initial_orientation(ImuData *imu_data);
void quaternion_multiply(float q1[4], float q2[4], float qr[4]);
void imu_get_sensor_data (ImuData *imu_data);
void kalman_filter_reset(void);
void kalman_filter_euler_reset(void);
void apply_filter_euler(float measured_pitch, float measured_roll, float measured_yaw, ImuData *imu_data);
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif
