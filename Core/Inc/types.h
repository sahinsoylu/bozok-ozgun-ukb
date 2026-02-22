/*
 * types.h
 *
 *  Created on: Sep 2, 2025
 *      Author: HUAWEl
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

enum UKB_MODE{
	NORMAL,
	SIT,
	SUT
};
enum FLIGHT_FLAG{
	ON_GROUND = 0,
	PROTECTED_FLIGHT = 1,
	ALTITUDE_PROTECTED_FLIGHT = 2,
	FREE_FLIGHT = 3,
	APOGEE = 4,
	TRIGGER = 5,
};
typedef struct {
    enum UKB_MODE ukb_mode;
    bool i2c_ok;
    bool isDelayed; // Bu flag'in mantığını tekrar gözden geçirmek iyi olabilir
    volatile bool reduceFrequencyFlag; // Hız düşürme flag'i
} SystemControl_t;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    Quaternion q;
    Quaternion q_filtered;

    float angle_ground_normal_q;

    float pitch;
	float roll;
	float yaw;

    float pitch_filtered;
    float roll_filtered;
    float yaw_filtered;

    float angle_ground_normal_pr;

} ImuData;

typedef struct {
    // Uçuş Durum Makinesi
    enum FLIGHT_FLAG flight_state;
    uint32_t takeoff_time;

    // YENİ EKLENEN DURUM BAYRAKLARI
    uint8_t stateData_h;
    uint8_t stateData_l;

    // Sensör Verileri
    struct PressureData pressure_data;
    ImuData imu_data;
    struct GpsData gps_data;

    // Hesaplanan Değerler
    float vertical_velocity;

    bool isDelayed;

} FlightData_t;




#endif /* INC_TYPES_H_ */
