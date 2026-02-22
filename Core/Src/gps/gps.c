#include "main.h"
#include "gps/gps.h"
#include <string.h>
#include <stdlib.h>


void parse_gpgga(char *nmea_sentence, struct GpsData *gps_data) {
    char *token;
    char *utc_time_str;
    char *latitude_str;
    char *longitude_str;
    char *altitude_str;  // Yeni: Altitude bilgisi için
    char lat_dir, lon_dir;

    // Sadece GPGGA cümlesini işliyoruz
    if (strncmp(nmea_sentence, "$GPGGA", 6) != 0) return;

    token = strtok(nmea_sentence, ",");      // $GPGGA
    token = strtok(NULL, ",");               // UTC Time
    utc_time_str = token;

    token = strtok(NULL, ",");               // Latitude
    latitude_str = token;

    token = strtok(NULL, ",");               // N/S
    lat_dir = token[0];

    token = strtok(NULL, ",");               // Longitude
    longitude_str = token;

    token = strtok(NULL, ",");               // E/W
    lon_dir = token[0];

    token = strtok(NULL, ",");               // Position Fix Indicator (0-1-2)
    token = strtok(NULL, ",");               // Satellites Used
    token = strtok(NULL, ",");               // HDOP
    token = strtok(NULL, ",");               // MSL Altitude (GPS yüksekliği)
    altitude_str = token;                    // Örnek: "545.4"

    token = strtok(NULL, ",");               // Altitude birimi (M: metre)
    // token şu an "M" veya başka bir birim olabilir (kontrol edilebilir)

    // UTC zamanını saat:dakika:saniye formatına çevir
    if (strlen(utc_time_str) >= 6) {
        char h_str[3] = {utc_time_str[0], utc_time_str[1], '\0'};
        char m_str[3] = {utc_time_str[2], utc_time_str[3], '\0'};
        char s_str[3] = {utc_time_str[4], utc_time_str[5], '\0'};
        gps_data->hour = atoi(h_str);
        gps_data->min = atoi(m_str);
        gps_data->sec = atoi(s_str);
    }

    // Latitude dönüşümü
    double lat_val = atof(latitude_str);
    double lat_deg = (int)(lat_val / 100);
    double lat_min = lat_val - (lat_deg * 100);
    double latitude = lat_deg + lat_min / 60.0;
    if (lat_dir == 'S') latitude = -latitude;
    gps_data->latitude = latitude;

    // Longitude dönüşümü
    double lon_val = atof(longitude_str);
    double lon_deg = (int)(lon_val / 100);
    double lon_min = lon_val - (lon_deg * 100);
    double longitude = lon_deg + lon_min / 60.0;
    if (lon_dir == 'W') longitude = -longitude;
    gps_data->longitude = longitude;

    // Altitude (yükseklik) dönüşümü (metre cinsinden)
    gps_data->altitude = atof(altitude_str);
}
