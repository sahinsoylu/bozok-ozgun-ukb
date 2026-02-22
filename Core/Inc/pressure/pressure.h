#ifndef PRESSURE_H
#define PRESSURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmp3.h"

#define MEDIAN_WINDOW_SIZE 7


struct PressureData {
	float pressure;
	float filtered_pressure;
	float altitude_sea;
	float temperature;
	float ground_altitude;
	float altitude;
	float press_calb;
	float temp_calb;
};


void bmp390_sensor_init();
void bmp390_process_data(struct PressureData *pressure_data);
void bmp390_get_sensor_data(struct PressureData *pressure_data);
void median_filter_reset(void);
float median_filter_apply(float new_pressure);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif
