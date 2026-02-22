#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif


struct GpsData {
	int16_t hour;
	int16_t min;
	int16_t sec;
	float latitude;
	float longitude;
	float altitude;
};

void parse_gpgga(char *nmea_sentence, struct GpsData *gps_data);



#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif
