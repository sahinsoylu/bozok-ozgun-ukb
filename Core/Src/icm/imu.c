#include "icm/imu.h"
#include "math.h"
#include "stdio.h"
#include "main.h"

static const float Q_diag[4] = {1e-4f, 1e-4f, 1e-4f, 1e-4f};

static const float R_diag[4] = {1e-2f, 1e-2f, 1e-2f, 1e-2f};

static float x_hat[4];

static float P_diag[4];

// Pitch, Roll, Yaw için Kalman Filtresi değişkenleri
static float pitch_x_hat;   // Pitch için durum tahmini
static float pitch_P_diag;  // Pitch için hata kovaryansı
static const float PITCH_Q_diag = 1e-4f; // Pitch için süreç gürültüsü
static const float PITCH_R_diag = 1e-2f; // Pitch için ölçüm gürültüsü

static float roll_x_hat;    // Roll için durum tahmini
static float roll_P_diag;   // Roll için hata kovaryansı
static const float ROLL_Q_diag = 1e-4f;  // Roll için süreç gürültüsü
static const float ROLL_R_diag = 1e-2f;  // Roll için ölçüm gürültüsü

static float yaw_x_hat;     // Yaw için durum tahmini
static float yaw_P_diag;    // Yaw için hata kovaryansı
static const float YAW_Q_diag = 1e-4f;   // Yaw için süreç gürültüsü
static const float YAW_R_diag = 1e-2f;   // Yaw için ölçüm gürültüsü

void kalman_filter_reset(void) {
    // Durum quaternion'unu kimlik quaternion'una ayarla [1, 0, 0, 0]
    x_hat[0] = 1.0f;
    x_hat[1] = 0.0f;
    x_hat[2] = 0.0f;
    x_hat[3] = 0.0f;

    // Hata kovaryansını (belirsizliği) yüksek bir değere ayarla
    for (int i = 0; i < 4; i++) {
        P_diag[i] = 1.0f;
    }
}

void kalman_filter_euler_reset(void) {
    pitch_x_hat = 0.0f;
    pitch_P_diag = 1.0f; // Yüksek belirsizlik ile başla

    roll_x_hat = 0.0f;
    roll_P_diag = 1.0f; // Yüksek belirsizlik ile başla

    yaw_x_hat = 0.0f;
    yaw_P_diag = 1.0f; // Yüksek belirsizlik ile başla
}

// Genelleştirilmiş tek boyutlu Kalman filtre güncellemesi
void update_single_kalman_filter(float *x_hat_ptr, float *P_diag_ptr, float Q, float R, float measured_value) {
    // 1. TAHMİN ADIMI
    *P_diag_ptr = *P_diag_ptr + Q; // P_minus = P + Q

    // 2. GÜNCELLEME ADIMI
    float K = *P_diag_ptr / (*P_diag_ptr + R); // Kalman Kazancı
    *x_hat_ptr = *x_hat_ptr + K * (measured_value - *x_hat_ptr); // Durum tahmini güncellemesi
    *P_diag_ptr = (1.0f - K) * (*P_diag_ptr); // Hata kovaryansı güncellemesi
}

void quaternion_to_euler(float q_w, float q_x, float q_y, float q_z, float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q_w * q_x + q_y * q_z);
    float cosr_cosp = 1.0f - 2.0f * (q_x * q_x + q_y * q_y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q_w * q_y - q_z * q_x);
    if (fabsf(sinp) >= 1) {
        *pitch = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI); // Use 90 degrees if out of range
    } else {
        *pitch = asinf(sinp) * (180.0f / M_PI);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q_w * q_z + q_x * q_y);
    float cosy_cosp = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}

void kalman_filter_update(float measured_q[4], ImuData *imu_data) {
    // --- 1. TAHMİN (PREDICTION) ADIMI ---

    float P_minus_diag[4];
    for (int i = 0; i < 4; i++) {
        P_minus_diag[i] = P_diag[i] + Q_diag[i];
    }

    // --- 2. GÜNCELLEME (UPDATE) ADIMI ---

    // Kalman Kazancı Hesabı: K = P_minus / (P_minus + R)
    float K_diag[4];
    for (int i = 0; i < 4; i++) {
        K_diag[i] = P_minus_diag[i] / (P_minus_diag[i] + R_diag[i]);
    }

    // Durum tahminini güncelle: x_hat = x_hat + K * (ölçüm - x_hat)
    for (int i = 0; i < 4; i++) {
        x_hat[i] = x_hat[i] + K_diag[i] * (measured_q[i] - x_hat[i]);
    }

    // Hata kovaryansını güncelle: P = (1 - K) * P_minus
    for (int i = 0; i < 4; i++) {
        P_diag[i] = (1.0f - K_diag[i]) * P_minus_diag[i];
    }

    // --- 3. NORMALİZASYON (EN KRİTİK ADIM) ---
    // Quaternion'un birim uzunluğunu korumak için normalize et.
    float norm = sqrtf(x_hat[0] * x_hat[0] + x_hat[1] * x_hat[1] + x_hat[2] * x_hat[2] + x_hat[3] * x_hat[3]);
    if (norm > 1e-9f) { // Sıfıra bölme hatasını önle
        for (int i = 0; i < 4; i++) {
            x_hat[i] /= norm;
        }
    }

    imu_data->q_filtered.w = x_hat[0];
    imu_data->q_filtered.x = x_hat[1];
    imu_data->q_filtered.y = x_hat[2];
    imu_data->q_filtered.z = x_hat[3];
}

static float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;

static float q_current[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float q_initial_inverse[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float q_relative[4] = {1.0f, 0.0f, 0.0f, 0.0f};

uint32_t previous_tick = 0; // Zaman farkı hesaplaması için

void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx *= M_PI / 180.0f; gy *= M_PI / 180.0f; gz *= M_PI / 180.0f;

    qDot1 = 0.5f * (-q_current[1] * gx - q_current[2] * gy - q_current[3] * gz);
    qDot2 = 0.5f * (q_current[0] * gx + q_current[2] * gz - q_current[3] * gy);
    qDot3 = 0.5f * (q_current[0] * gy - q_current[1] * gz + q_current[3] * gx);
    qDot4 = 0.5f * (q_current[0] * gz + q_current[1] * gy - q_current[2] * gx);

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * q_current[0]; _2q1 = 2.0f * q_current[1]; _2q2 = 2.0f * q_current[2]; _2q3 = 2.0f * q_current[3];
        _4q0 = 4.0f * q_current[0]; _4q1 = 4.0f * q_current[1]; _4q2 = 4.0f * q_current[2];
        _8q1 = 8.0f * q_current[1]; _8q2 = 8.0f * q_current[2];
        q0q0 = q_current[0] * q_current[0]; q1q1 = q_current[1] * q_current[1];
        q2q2 = q_current[2] * q_current[2]; q3q3 = q_current[3] * q_current[3];

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q_current[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q_current[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q_current[3] - _2q1 * ax + 4.0f * q2q2 * q_current[3] - _2q2 * ay;
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= MADGWICK_BETA * s0; qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2; qDot4 -= MADGWICK_BETA * s3;
    }
    q_current[0] += qDot1 * dt; q_current[1] += qDot2 * dt;
    q_current[2] += qDot3 * dt; q_current[3] += qDot4 * dt;
    recipNorm = 1.0f / sqrtf(q_current[0] * q_current[0] + q_current[1] * q_current[1] + q_current[2] * q_current[2] + q_current[3] * q_current[3]);
    q_current[0] *= recipNorm; q_current[1] *= recipNorm;
    q_current[2] *= recipNorm; q_current[3] *= recipNorm;
}

void calibrate_gyro_offsets(ImuData *imu_data) {
    const int num_samples = 500;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    send_usb_message("1. Gyro Calibration: Keep sensor stationary...\r\n");
    HAL_Delay(1000);
    for (int i = 0; i < num_samples; i++) {
        if (ICM42670P_ReadAccelGyroData(imu_data) == HAL_OK) {
            sum_gx += imu_data->gx; sum_gy += imu_data->gy; sum_gz += imu_data->gz;
        }
        HAL_Delay(3);
    }
    gyro_offset_x = sum_gx / num_samples;
    gyro_offset_y = sum_gy / num_samples;
    gyro_offset_z = sum_gz / num_samples;
    send_usb_message("Gyro calibration done.\r\n");
}

void set_initial_orientation(ImuData *imu_data) {
    send_usb_message("2. Zeroing Position: Calculating initial orientation...\r\n");
    HAL_Delay(100);

    float ax_avg = 0, ay_avg = 0, az_avg = 0;
    const int avg_count = 100;
    for(int i=0; i<avg_count; i++) {
        ICM42670P_ReadAccelGyroData(imu_data);
        ax_avg += imu_data->ax;
        ay_avg +=  imu_data->ay;
        az_avg += imu_data->az;
        HAL_Delay(3);
    }

    ax_avg /= avg_count; ay_avg /= avg_count; az_avg /= avg_count;

    float norm = sqrtf(ax_avg*ax_avg + ay_avg*ay_avg + az_avg*az_avg);
    ax_avg /= norm; ay_avg /= norm; az_avg /= norm;

    float v_from[3] = { ax_avg, ay_avg, az_avg };
    float v_to[3]   = { 0.0f, 0.0f, 1.0f };

    float cross[3] = {
        v_from[1]*v_to[2] - v_from[2]*v_to[1],
        v_from[2]*v_to[0] - v_from[0]*v_to[2],
        v_from[0]*v_to[1] - v_from[1]*v_to[0]
    };

    float dot = v_from[0]*v_to[0] + v_from[1]*v_to[1] + v_from[2]*v_to[2];
    float s = sqrtf((1.0f + dot) * 2.0f);
    float inv_s = 1.0f / s;

    float q_init[4] = {
        s * 0.5f,
        cross[0] * inv_s,
        cross[1] * inv_s,
        cross[2] * inv_s
    };

    float qnorm = sqrtf(q_init[0]*q_init[0] + q_init[1]*q_init[1] + q_init[2]*q_init[2] + q_init[3]*q_init[3]);
    for (int i = 0; i < 4; i++) q_init[i] /= qnorm;

    q_current[0] = q_init[0]; q_current[1] = q_init[1];
    q_current[2] = q_init[2]; q_current[3] = q_init[3];

    q_initial_inverse[0] =  q_current[0];
    q_initial_inverse[1] = -q_current[1];
    q_initial_inverse[2] = -q_current[2];
    q_initial_inverse[3] = -q_current[3];

    send_usb_message("Zero point set.\r\n");
}
void quaternion_multiply(float q1[4], float q2[4], float qr[4]) {
    qr[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    qr[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    qr[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    qr[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}


HAL_StatusTypeDef ICM42670P_ReadRegister(uint8_t reg, uint8_t *data) {
return HAL_I2C_Mem_Read(&hi2c2, ICM42670P_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef ICM42670P_WriteRegister(uint8_t reg, uint8_t value) {
return HAL_I2C_Mem_Write(&hi2c2, ICM42670P_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

uint8_t ICM42670P_Init(void) {
	HAL_StatusTypeDef status;
	uint8_t read_val = 0;
	char msg[60];

	uint8_t target_pwr_mgmt0 = 0x0F;

	snprintf(msg, sizeof(msg), "Attempting to activate sensor (PWR_MGMT0[0x%02X]=0x%02X)...\r\n", ICM42670P_PWR_MGMT0, target_pwr_mgmt0);
	send_usb_message(msg);
	status = ICM42670P_WriteRegister(ICM42670P_PWR_MGMT0, target_pwr_mgmt0);
	if (status != HAL_OK) {
		snprintf(msg, sizeof(msg), "Error: Writing PWR_MGMT0 failed! Status: %d\r\n", status);
		send_usb_message(msg);
		return 0;
	}

	send_usb_message("Waiting 100ms for mode switch...\r\n");
	HAL_Delay(100);
	send_usb_message("Reading back PWR_MGMT0 to verify...\r\n");
	status = ICM42670P_ReadRegister(ICM42670P_PWR_MGMT0, &read_val);
	if (status != HAL_OK) {
		snprintf(msg, sizeof(msg), "Error: Reading back PWR_MGMT0 failed! Status: %d\r\n", status);
		send_usb_message(msg);
	}

	snprintf(msg, sizeof(msg), "Verification -> PWR_MGMT0: Should be=0x%02X, Read=0x%02X\r\n", target_pwr_mgmt0, read_val);
	send_usb_message(msg);

	if (read_val != target_pwr_mgmt0) {
		send_usb_message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		send_usb_message("FATAL: Sensor did NOT activate! PWR_MGMT0 mismatch.\r\n");
		send_usb_message("Check I2C Write reliability, Power, Wiring, Pull-ups!\r\n");
		send_usb_message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		return 0;
	}
	send_usb_message("Sensor successfully activated (PWR_MGMT0 confirmed).\r\n");
	HAL_Delay(100);
	send_usb_message("ICM42670P Initialization Sequence Complete.\r\n");

	return 1;
}

HAL_StatusTypeDef ICM42670P_ReadBurst(uint8_t start_reg, uint8_t *data, uint16_t length) {
	return HAL_I2C_Mem_Read(&hi2c2, ICM42670P_I2C_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT, data, length, 200);
}

HAL_StatusTypeDef ICM42670P_ReadAccelGyroData(ImuData *imu_data) {
	uint8_t sensor_data_raw[14];
	HAL_StatusTypeDef status;

	status = ICM42670P_ReadBurst(ICM42670P_TEMP_DATA1, sensor_data_raw, 14);

	if (status == HAL_OK) {
		int16_t ax_raw = (int16_t)((sensor_data_raw[2] << 8) | sensor_data_raw[3]); // ACCEL_X1, ACCEL_X0
		int16_t ay_raw = (int16_t)((sensor_data_raw[4] << 8) | sensor_data_raw[5]); // ACCEL_Y1, ACCEL_Y0
		int16_t az_raw = (int16_t)((sensor_data_raw[6] << 8) | sensor_data_raw[7]); // ACCEL_Z1, ACCEL_Z0

		int16_t gx_raw = (int16_t)((sensor_data_raw[8] << 8) | sensor_data_raw[9]);   // GYRO_X1, GYRO_X0
		int16_t gy_raw = (int16_t)((sensor_data_raw[10] << 8) | sensor_data_raw[11]); // GYRO_Y1, GYRO_Y0
		int16_t gz_raw = (int16_t)((sensor_data_raw[12] << 8) | sensor_data_raw[13]); // GYRO_Z1, GYRO_Z0

		imu_data->ax = (float)ax_raw * ACCEL_SCALE_FACTOR_16G;
		imu_data->ay = (float)ay_raw * ACCEL_SCALE_FACTOR_16G;
		imu_data->az = (float)az_raw * ACCEL_SCALE_FACTOR_16G;

		imu_data->gx = (float)gx_raw * GYRO_SCALE_FACTOR_2000DPS;
		imu_data->gy = (float)gy_raw * GYRO_SCALE_FACTOR_2000DPS;
		imu_data->gz = (float)gz_raw * GYRO_SCALE_FACTOR_2000DPS;
	} else {
		imu_data->ax = 0.0f;
		imu_data->ay = 0.0f;
		imu_data->az = 0.0f;
		imu_data->gx = 0.0f;
		imu_data->gy = 0.0f;
		imu_data->gz = 0.0f;
	}

	return status;
}



float calculate_angle_from_quaternion(float q_w, float q_x, float q_y, float q_z) {
    float cos_theta = 1.0f - 2.0f * (q_x * q_x + q_y * q_y);

    if (cos_theta > 1.0f) {
        cos_theta = 1.0f;
    }
    if (cos_theta < -1.0f) {
        cos_theta = -1.0f;
    }

    float angle_rad = acosf(cos_theta);

    float angle_deg = angle_rad * (180.0f / M_PI);

    return angle_deg;
}

void apply_filter_euler(float measured_pitch, float measured_roll, float measured_yaw, ImuData *imu_data){
    update_single_kalman_filter(&pitch_x_hat, &pitch_P_diag, PITCH_Q_diag, PITCH_R_diag, measured_pitch);
    imu_data->pitch_filtered = pitch_x_hat; // Filtrelenmiş Pitch'i kaydet

    // Roll için Kalman filtresini güncelle
    update_single_kalman_filter(&roll_x_hat, &roll_P_diag, ROLL_Q_diag, ROLL_R_diag, measured_roll);
    imu_data->roll_filtered = roll_x_hat; // Filtrelenmiş Roll'u kaydet

    // Yaw için Kalman filtresini güncelle
    update_single_kalman_filter(&yaw_x_hat, &yaw_P_diag, YAW_Q_diag, YAW_R_diag, measured_yaw);
    imu_data->yaw_filtered = yaw_x_hat;   // Filtrelenmiş Yaw'ı kaydet

    float pitch_rad = imu_data->pitch_filtered * M_PI / 180.0f;
    float roll_rad = imu_data->roll_filtered * M_PI / 180.0f;
    imu_data->angle_ground_normal_pr = asinf(sqrtf(sinf(pitch_rad) * sinf(pitch_rad) + sinf(roll_rad) * sinf(roll_rad))) * (180.0f / M_PI);
}


void imu_get_sensor_data (ImuData *imu_data){
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - previous_tick) / 1000.0f;
    previous_tick = current_tick;

    if (dt > 0.001f) {

        float cal_gx = imu_data->gx - gyro_offset_x;
        float cal_gy = imu_data->gy - gyro_offset_y;
        float cal_gz = imu_data->gz - gyro_offset_z;

        MadgwickUpdate(cal_gx, cal_gy, cal_gz,
                       imu_data->ax, imu_data->ay, imu_data->az, dt);

        quaternion_multiply(q_current, q_initial_inverse, q_relative);

        imu_data->q.w = q_relative[0];
        imu_data->q.x = q_relative[1];
        imu_data->q.y = q_relative[2];
        imu_data->q.z = q_relative[3];

        kalman_filter_update(q_relative, imu_data);

        float measured_roll, measured_pitch, measured_yaw;
        quaternion_to_euler(imu_data->q_filtered.w, imu_data->q_filtered.x, imu_data->q_filtered.y, imu_data->q_filtered.z, &measured_roll, &measured_pitch, &measured_yaw);
        imu_data->pitch = measured_pitch;
        imu_data->roll = measured_roll;
        imu_data->yaw = measured_yaw;
        apply_filter_euler(measured_pitch, measured_roll, measured_yaw, imu_data);

        imu_data->angle_ground_normal_q = calculate_angle_from_quaternion(q_relative[0], q_relative[1], q_relative[2], q_relative[3]);
    }
}


