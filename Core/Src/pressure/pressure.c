#include "pressure/bmp3.h"
#include "pressure/pressure.h"
#include "main.h"
#include <math.h>
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"

struct bmp3_dev bmp3;
struct bmp3_data data = {0};
struct bmp3_status status = {0};

static const float seaLevelPressure = 101325.0f; //Pa

static float window_buffer[MEDIAN_WINDOW_SIZE] = {seaLevelPressure};
static int current_index = 0;
static bool is_filled = false;


const float altitude_lut[][2] = {
    { 105000.0f, -305.45f },
    { 104500.0f, -263.85f },
    { 104000.0f, -222.14f },
    { 103500.0f, -180.33f },
    { 103000.0f, -138.42f },
    { 102500.0f, -96.41f },
    { 102000.0f, -54.30f },
    { 101500.0f, -12.08f },
    { 101325.0f, 0.00f }, // Deniz Seviyesi Referansı
    { 101000.0f, 27.24f },
    { 100500.0f, 69.57f },
    { 100000.0f, 111.99f },
    { 99500.0f, 154.52f },
    { 99000.0f, 197.14f },
    { 98500.0f, 239.86f },
    { 98000.0f, 282.68f },
    { 97500.0f, 325.60f },
    { 97000.0f, 368.62f },
    { 96500.0f, 411.75f },
    { 96000.0f, 454.98f },
    { 95500.0f, 498.31f },
    { 95000.0f, 541.75f },
    { 94500.0f, 585.30f },
    { 94000.0f, 628.95f },
    { 93500.0f, 672.71f },
    { 93000.0f, 716.57f },
    { 92500.0f, 760.54f },
    { 92000.0f, 804.62f },
    { 91500.0f, 848.81f },
    { 91000.0f, 893.11f },
    { 90500.0f, 937.51f },
    { 90000.0f, 982.03f },
    { 89500.0f, 1026.66f },
    { 89000.0f, 1071.40f },
    { 88500.0f, 1116.25f },
    { 88000.0f, 1161.21f },
    { 87500.0f, 1206.29f },
    { 87000.0f, 1251.48f },
    { 86500.0f, 1296.79f },
    { 86000.0f, 1342.21f },
    { 85500.0f, 1387.75f },
    { 85000.0f, 1433.41f },
    { 84500.0f, 1479.18f },
    { 84000.0f, 1525.08f },
    { 83500.0f, 1571.09f },
    { 83000.0f, 1617.22f },
    { 82500.0f, 1663.48f },
    { 82000.0f, 1709.85f },
    { 81500.0f, 1756.35f },
    { 81000.0f, 1802.97f },
    { 80500.0f, 1849.71f },
    { 80000.0f, 1896.58f },
    { 79500.0f, 1943.58f },
    { 79000.0f, 1990.70f },
    { 78500.0f, 2037.95f },
    { 78000.0f, 2085.33f },
    { 77500.0f, 2132.84f },
    { 77000.0f, 2180.48f },
    { 76500.0f, 2228.25f },
    { 76000.0f, 2276.15f },
    { 75500.0f, 2324.18f },
    { 75000.0f, 2372.35f },
    { 74500.0f, 2420.65f },
    { 74000.0f, 2469.09f },
    { 73500.0f, 2517.66f },
    { 73000.0f, 2566.38f },
    { 72500.0f, 2615.23f },
    { 72000.0f, 2664.23f },
    { 71500.0f, 2713.37f },
    { 71000.0f, 2762.66f },
    { 70500.0f, 2812.09f },
    { 70000.0f, 2861.67f },
    { 69500.0f, 2911.40f },
    { 69000.0f, 2961.28f },
    { 68500.0f, 3011.31f },
    { 68000.0f, 3061.50f },
    { 67500.0f, 3111.85f },
    { 67000.0f, 3162.35f },
    { 66500.0f, 3213.01f },
    { 66000.0f, 3263.83f },
    { 65500.0f, 3314.81f },
    { 65000.0f, 3365.95f },
    { 64500.0f, 3417.26f },
    { 64000.0f, 3468.73f },
    { 63500.0f, 3520.37f },
    { 63000.0f, 3572.18f },
    { 62500.0f, 3624.16f },
    { 62000.0f, 3676.32f },
    { 61500.0f, 3728.65f },
    { 61000.0f, 3781.16f },
    { 60500.0f, 3833.85f },
    { 60000.0f, 3886.72f },
    { 59500.0f, 3939.78f },
    { 59000.0f, 3993.03f },
    { 58500.0f, 4046.46f },
    { 58000.0f, 4100.09f },
    { 57500.0f, 4153.91f },
    { 57000.0f, 4207.92f },
    { 56500.0f, 4262.13f },
    { 56000.0f, 4316.53f },
    { 55500.0f, 4371.14f },
    { 55000.0f, 4425.95f },
    { 54500.0f, 4480.97f },
    { 54000.0f, 4536.20f },
    { 53500.0f, 4591.64f },
    { 53000.0f, 4647.30f },
    { 52500.0f, 4703.17f },
    { 52000.0f, 4759.26f },
    { 51500.0f, 4815.57f },
    { 51000.0f, 4872.11f },
    { 50500.0f, 4928.87f },
    { 50000.0f, 4985.86f },
    { 49500.0f, 5043.08f },
    { 49000.0f, 5100.53f },
    { 48500.0f, 5158.23f },
    { 48000.0f, 5216.16f },
    { 47500.0f, 5274.34f },
    { 47000.0f, 5332.77f },
    { 46500.0f, 5391.45f },
    { 46000.0f, 5450.39f },
    { 45500.0f, 5509.58f },
    { 45000.0f, 5569.04f },
    { 44500.0f, 5628.77f },
    { 44000.0f, 5688.77f },
    { 43500.0f, 5749.05f },
    { 43000.0f, 5809.61f },
    { 42500.0f, 5870.45f },
    { 42000.0f, 5931.59f },
    { 41500.0f, 5993.02f },
    { 41000.0f, 6054.76f },
    { 40500.0f, 6116.80f },
    { 40000.0f, 6179.14f },
    { 39500.0f, 6241.79f },
    { 39000.0f, 6304.76f },
    { 38500.0f, 6368.05f },
    { 38000.0f, 6431.66f },
    { 37500.0f, 6495.60f },
    { 37000.0f, 6559.88f },
    { 36500.0f, 6624.50f },
    { 36000.0f, 6689.46f },
    { 35500.0f, 6754.78f },
    { 35000.0f, 6820.46f },
    { 34500.0f, 6886.50f },
    { 34000.0f, 6952.92f },
    { 33500.0f, 7019.71f },
    { 33000.0f, 7086.89f },
    { 32500.0f, 7154.46f },
    { 32000.0f, 7222.44f },
    { 31500.0f, 7290.82f },
    { 31000.0f, 7359.62f },
    { 30500.0f, 7428.84f },
    { 30000.0f, 7498.49f },
    { 29500.0f, 7568.58f },
    { 29000.0f, 7639.12f },
    { 28500.0f, 7710.12f },
    { 28000.0f, 7781.59f },
    { 27500.0f, 7853.53f },
    { 27000.0f, 7925.96f },
    { 26500.0f, 7998.88f },
    { 26000.0f, 8072.31f },
    { 25500.0f, 8146.26f },
    { 25000.0f, 8220.73f },
    { 24500.0f, 8295.74f },
    { 24000.0f, 8371.30f },
    { 23500.0f, 8447.42f },
    { 23000.0f, 8524.12f },
    { 22500.0f, 8601.41f },
    { 22000.0f, 8679.31f },
    { 21500.0f, 8757.82f },
    { 21000.0f, 8836.97f },
    { 20500.0f, 8916.78f },
    { 20000.0f, 8997.26f },
    { 19500.0f, 9078.43f },
    { 19000.0f, 9160.31f },
    { 18500.0f, 9242.92f },
    { 18000.0f, 9326.28f },
    { 17500.0f, 9410.41f },
    { 17000.0f, 9495.34f },
    { 16500.0f, 9581.08f },
    { 16000.0f, 9667.67f },
    { 15500.0f, 9755.13f },
    { 15000.0f, 9843.47f },
    { 14500.0f, 9932.74f },
    { 14000.0f, 10022.96f },
    { 13500.0f, 10114.16f },
    { 13000.0f, 10206.37f },
    { 12500.0f, 10299.63f },
    { 12000.0f, 10393.97f },
    { 11500.0f, 10489.42f },
    { 11000.0f, 10586.03f },
    { 10500.0f, 10683.83f },
    { 10000.0f, 10782.86f },
};

const int LUT_SIZE = sizeof(altitude_lut) / sizeof(altitude_lut[0]);


int8_t bmp3_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    return (HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) == HAL_OK) ? BMP3_OK : BMP3_E_COMM_FAIL;
}

int8_t bmp3_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    return (HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, HAL_MAX_DELAY) == HAL_OK) ? BMP3_OK : BMP3_E_COMM_FAIL;
}

void bmp3_delay_us(uint32_t us, void *intf_ptr)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}


int8_t bmp3_interface_init(struct bmp3_dev *dev, uint8_t intf) {

    static uint8_t dev_addr = 0x76 << 1;

    dev->intf_ptr = &dev_addr;
    dev->intf = BMP3_I2C_INTF;
    dev->read = bmp3_i2c_read;
    dev->write = bmp3_i2c_write;
    dev->delay_us = bmp3_delay_us;
    return BMP3_OK;

}

void bmp390_sensor_init(){

	int8_t rslt;
	struct bmp3_settings settings = {0};
	uint16_t settings_sel;

	rslt = bmp3_interface_init(&bmp3, BMP3_I2C_INTF);
	if (rslt != BMP3_OK) send_usb_message("Interface Init Error\r\n");

	rslt = bmp3_init(&bmp3);
	if (rslt != BMP3_OK) send_usb_message("BMP Init Error\r\n");


	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;
	settings.odr_filter.odr = BMP3_ODR_100_HZ;
	settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
	settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
	settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER | BMP3_SEL_TEMP_OS | BMP3_SEL_PRESS_OS;

	rslt = bmp3_set_sensor_settings(settings_sel, &settings, &bmp3);

	if (rslt != BMP3_OK) send_usb_message("Set Sensor Settings Failed\r\n");


	settings.op_mode = BMP3_MODE_NORMAL;
	rslt = bmp3_set_op_mode(&settings, &bmp3);
	if (rslt != BMP3_OK) send_usb_message("Set Op Mode Failed\r\n");

}


float calculate_altitude_icao(float current_pressure_pa)
{
    // ICAO Standart Atmosfer sabitleri
    const float P0 = 101325.0f; // Deniz seviyesi standart basıncı (Pa)

    // Bu sabitler, formülün basitleştirilmiş halinden gelir:
    // Formül: h = 44330 * (1 - (P / P0)^(1/5.255))
    // 1/5.255 ≈ 0.190295
    const float EXPONENT = 0.190295f;
    const float ALTITUDE_FACTOR = 44330.0f;

    // Hata kontrolü: Negatif veya sıfır basınç anlamsızdır.
    if (current_pressure_pa <= 0) {
        return 0.0f;
    }

    // Formülü uygula
    return ALTITUDE_FACTOR * (1.0f - powf(current_pressure_pa / P0, EXPONENT));
}

void bmp390_get_sensor_data(struct PressureData *pressure_data){
	bmp3_get_status(&status, &bmp3);
	if (status.intr.drdy) {
	  bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &bmp3);
	  pressure_data->pressure = data.pressure;
	  pressure_data->temperature = data.temperature;
	}
}


void bmp390_calibrate(struct PressureData *pressure_data){
    uint8_t i;
    int num_samples = 30;
    float pressure_sum;
    float temp_sum;

    for (i = 0; i < num_samples; i++)
    {
        // Sensörden veri oku (basınç ve sıcaklık)
        bmp390_get_sensor_data(pressure_data);
        pressure_sum += data.pressure;
        temp_sum += data.temperature;

        HAL_Delay(20);
    }

    (*pressure_data).temp_calb = temp_sum / 30;
    (*pressure_data).press_calb = pressure_sum / 30;

}

void bmp390_process_data(struct PressureData *pressure_data){
	  pressure_data->filtered_pressure = median_filter_apply(data.pressure);
	  pressure_data->altitude_sea = 44330.0f * (1.0f - powf(pressure_data->filtered_pressure / seaLevelPressure, 0.1903f));
	  if(pressure_data->ground_altitude == 0){
		  pressure_data->ground_altitude = pressure_data->altitude_sea;
	  }
	  pressure_data->altitude = calculate_altitude_icao(data.pressure);
}



//=====================================================================================
// Filtrenin dahili durum değişkenleri
//=====================================================================================


// Sıralama için karşılaştırma fonksiyonu (değişiklik yok)
static int float_compare(const void *a, const void *b) {
    float fa = *(const float*) a;
    float fb = *(const float*) b;
    return (fa > fb) - (fa < fb);
}

/**
 * @brief Medyan filtresini standart varsayılan bir değere sıfırlar.
 */
void median_filter_reset(void) {
    // Tüm pencereyi varsayılan basınç değeriyle doldur
    for (int i = 0; i < MEDIAN_WINDOW_SIZE; i++) {
        window_buffer[i] = seaLevelPressure;
    }
    // İndeksi ve doluluk durumunu sıfırla
    current_index = 0;
    is_filled = false;
}

/**
 * @brief Gelen yeni basınç değerine medyan filtresi uygular. (Değişiklik yok)
 */
float median_filter_apply(float new_pressure) {
    // Yeni değeri pencereye (buffer) ekle
    window_buffer[current_index] = new_pressure;

    current_index++;

    if (current_index >= MEDIAN_WINDOW_SIZE) {
        current_index = 0; // Başa dön
        is_filled = true;
    }

    if (!is_filled) {
        return new_pressure;
    }

    float temp_buffer[MEDIAN_WINDOW_SIZE];
    memcpy(temp_buffer, window_buffer, sizeof(window_buffer));

    qsort(temp_buffer, MEDIAN_WINDOW_SIZE, sizeof(float), float_compare);

    return temp_buffer[MEDIAN_WINDOW_SIZE / 2];
}




