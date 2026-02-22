/* USER CODE BEGIN Header */
/**


@file           : main.c

@brief          : Main program body



@attention


Copyright (c) 2025 STMicroelectronics.

All rights reserved.


This software is licensed under terms that can be found in the LICENSE file

in the root directory of this software component.

If no LICENSE file comes with this software, it is provided AS-IS.



*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "pressure/pressure.h"
#include "gps/gps.h"
#include "icm/imu.h"
#include "math.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Sensor data variables
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;

/* Definitions for defaultTask */
/*osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};*/
/* USER CODE BEGIN PV */



#define TELEMETRY_PACKET_SIZE       48 // Toplam 41 bayt oldu
#define TELEMETRY_PAYLOAD_LENGTH    46 // Checksum hariç, header ve tüm veriler (0'dan 37. indekse kadar)
#define SENSOR_TASK_PERIOD_MS_80HZ 12
#define SENSOR_TASK_PERIOD_MS_10HZ 100
#define TELEMETRY_INTERVAL_MS 500

#define FIFO_SIZE 10          // Pencere boyutu, kaç ölçüm saklanacağı


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void ProcessReceivedPacket(uint8_t* data, uint8_t length, uint8_t packet_type);
typedef struct {
    uint8_t bytes[4];
} FloatBytes_t;



uint8_t gps_rx_data;
char gps_buffer[512];
uint16_t gps_index = 0;
struct GpsData gps_data;
uint16_t received_data_len;

uint8_t huart4_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];

typedef struct {
  float altitude;
  unsigned long timestamp_ms;
} DataPoint;
const float gravity = 9.80665;

TaskHandle_t sensorTaskHandle;
TaskHandle_t loraTaskHandle;
SemaphoreHandle_t sensorSemaphore;
SemaphoreHandle_t sutDataReadySemaphore;


// FreeRTOS nesnelerinin handle'larını bir arada tutan yapı (sadece organizasyon için).

volatile FlightData_t g_flightData = {
		.flight_state = ON_GROUND,
		.stateData_h = 0,
		.stateData_l = 0,
		.vertical_velocity = 0.0f,
		.isDelayed = false,
};

// 2. SİSTEM KONTROL FLAG'LERİ
SystemControl_t g_systemControl = {
		.ukb_mode = NORMAL,
};

/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void addToFifo(float new_altitude, DataPoint altitude_fifo[], int *buffer_index, bool *is_buffer_full) {
  // Yeni veri noktasını oluştur
  altitude_fifo[*buffer_index].altitude = new_altitude;
  altitude_fifo[*buffer_index].timestamp_ms = HAL_GetTick(); // STM32'de geçen zamanı al

  // İndeksi bir sonraki pozisyona taşı
  (*buffer_index)++;

  // Buffer doldu mu kontrol et ve indeksi sıfırla
  if ((*buffer_index) >= FIFO_SIZE) {
    (*buffer_index) = 0;
    *is_buffer_full = true; // Buffer ilk kez doldu, artık hesaplama yapabiliriz.
  }
}


float calculate_velocity_from_fifo(DataPoint altitude_fifo[]) {
  // Hesaplamalar için değişkenler
  double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

  // Referans noktası olarak ilk zaman damgasını alalım ki sayılar çok büyümesin
  double t0 = altitude_fifo[0].timestamp_ms / 1000.0; // Saniye cinsinden

  for (int i = 0; i < FIFO_SIZE; ++i) {
    double x = (altitude_fifo[i].timestamp_ms / 1000.0) - t0; // Geçen saniye
    double y = altitude_fifo[i].altitude;

    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_x2 += x * x;
  }

  // Eğim (hız) formülünü uygula
  double N = FIFO_SIZE;
  double numerator = (N * sum_xy) - (sum_x * sum_y);
  double denominator = (N * sum_x2) - (sum_x * sum_x);

  // Sıfıra bölünmeyi engelle
  if (denominator == 0) {
    return 0.0f;
  }

  return (float)(numerator / denominator); // m/s cinsinden hız
}

void send_usb_message(const char* message) {
if (message != NULL) {
CDC_Transmit_FS((uint8_t*)message, strlen(message));
}}

uint8_t calculate_checksum_8bit(const uint8_t *data_buffer, uint16_t length) {
    uint8_t checksum = 0;

    for (uint16_t i = 0; i < length; i++) {
        checksum += data_buffer[i];
    }

    return checksum;
}

FloatBytes_t convert_float_to_big_endian_bytes_return(float float_value) {
    union {
        float f;
        uint8_t b[4];
    } u;

    FloatBytes_t result;

    u.f = float_value;
    result.bytes[0] = u.b[3];
    result.bytes[1] = u.b[2];
    result.bytes[2] = u.b[1];
    result.bytes[3] = u.b[0];

    return result;
}
void convert_int16_to_big_endian_bytes(int16_t value, uint8_t *output_buffer) {
    if (output_buffer == NULL) {
        return;
    }

    output_buffer[0] = (uint8_t)(value >> 8);

    output_buffer[1] = (uint8_t)(value & 0xFF);
}

void convert_int32_to_big_endian_bytes(int32_t value, uint8_t *output_buffer) {
    // MSB
    output_buffer[0] = (uint8_t)((value >> 24) & 0xFF);
    output_buffer[1] = (uint8_t)((value >> 16) & 0xFF);
    output_buffer[2] = (uint8_t)((value >> 8) & 0xFF);
    // LSB
    output_buffer[3] = (uint8_t)(value & 0xFF);
}
void create_telemetry_packet_no_id(
    uint8_t *packet_buffer,
    float accel_x_g, float accel_y_g, float accel_z_g,float gyro_x, float gyro_y, float gyro_z,
    float quat_w, float quat_x, float quat_y, float quat_z,
    double gps_latitude_deg, double gps_longitude_deg, float gps_altitude_m,
	float altitude,
    int16_t surface_angle,
    uint8_t status_value
) {
    if (packet_buffer == NULL) {
        return; // Geçersiz tampon
    }

    uint16_t current_index = 0;
    FloatBytes_t float_bytes; // Float dönüşümü için geçici yapı
    uint8_t scaled_bytes[4]; // int16 veya int32 için geçici dizi

    // 0. Başlangıç Baytı (Header)
    packet_buffer[current_index++] = 0xAA;

    // 1. İvme X, Y, Z (G cinsinden, int16_t olarak skalalı)
    convert_int16_to_big_endian_bytes((int16_t)(accel_x_g * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    convert_int16_to_big_endian_bytes((int16_t)(accel_y_g * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    convert_int16_to_big_endian_bytes((int16_t)(accel_z_g * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    convert_int16_to_big_endian_bytes((int16_t)(gyro_x * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    convert_int16_to_big_endian_bytes((int16_t)(gyro_y * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    convert_int16_to_big_endian_bytes((int16_t)(gyro_z * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    // 2. Quaternion W, X, Y, Z (float olarak kalır)
    float_bytes = convert_float_to_big_endian_bytes_return(quat_w);
    memcpy(&packet_buffer[current_index], float_bytes.bytes, 4);
    current_index += 4;

    float_bytes = convert_float_to_big_endian_bytes_return(quat_x);
    memcpy(&packet_buffer[current_index], float_bytes.bytes, 4);
    current_index += 4;

    float_bytes = convert_float_to_big_endian_bytes_return(quat_y);
    memcpy(&packet_buffer[current_index], float_bytes.bytes, 4);
    current_index += 4;

    float_bytes = convert_float_to_big_endian_bytes_return(quat_z);
    memcpy(&packet_buffer[current_index], float_bytes.bytes, 4);
    current_index += 4;

    // 3. GPS Enlem (int32_t olarak skalalı)
    convert_int32_to_big_endian_bytes((int32_t)(gps_latitude_deg * 1000000.0), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 4);
    current_index += 4;

    // 4. GPS Boylam (int32_t olarak skalalı)
    convert_int32_to_big_endian_bytes((int32_t)(gps_longitude_deg * 1000000.0), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 4);
    current_index += 4;

    // 5. GPS Yükseklik (int32_t olarak skalalı, 0.01m hassasiyet)
    convert_int32_to_big_endian_bytes((int32_t)(gps_altitude_m * 100.0f), scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 4);
    current_index += 4;

    convert_int16_to_big_endian_bytes(altitude * 100.0f, scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;

    // 6. Yüzey Açısı (int16_t)
    convert_int16_to_big_endian_bytes(surface_angle, scaled_bytes);
    memcpy(&packet_buffer[current_index], scaled_bytes, 2);
    current_index += 2;


    // 7. Durum Değeri (uint8_t)
    packet_buffer[current_index++] = status_value;

    // 8. Checksum Hesaplama ve Ekleme
    // Checksum, başlangıç baytından durum değerine kadar olan TÜM veriyi kapsar.
    uint8_t calculated_checksum = calculate_checksum_8bit(packet_buffer, TELEMETRY_PAYLOAD_LENGTH);
    packet_buffer[current_index++] = calculated_checksum;

    // 9. Footer Değerleri
    packet_buffer[current_index++] = 0x0A; // LF (Line Feed)

    // current_index artık 41 olmalı (TELEMETRY_PACKET_SIZE)
}

void set_bit_u8(uint8_t *value_ptr, uint8_t bit_position) {
    if (value_ptr == NULL) {
        return;
    }
    if (bit_position < 8) {
        *value_ptr |= (1 << bit_position);
    }
}


float convert_big_endian_float(uint8_t *data) {
    union {
        float f;
        uint8_t b[4];
    } u;

    u.b[0] = data[3];
    u.b[1] = data[2];
    u.b[2] = data[1];
    u.b[3] = data[0];

    return u.f;
}

void reset_state()
{
    memset((void*)&g_flightData, 0, sizeof(FlightData_t));
    g_flightData.flight_state = ON_GROUND;
    g_flightData.takeoff_time = 0;
    g_flightData.vertical_velocity = 0.0f;
    g_flightData.stateData_h = 0;
    g_flightData.stateData_l = 0;
    median_filter_reset();
    kalman_filter_reset();
    kalman_filter_euler_reset();
}

void sensorTask(void *pvParameters)
{

  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS_80HZ);

  static enum UKB_MODE previous_mode;
  previous_mode = g_systemControl.ukb_mode; // Başlangıç değerini ata


  static DataPoint altitude_fifo[FIFO_SIZE];

  static int apogee_confirmation_counter = 0;
  uint8_t local_dma_buffer[UART_RX_DMA_BUFFER_SIZE];
  static uint8_t statePacket[6];
  static uint8_t sit_data[36] = {0xAB};
  static float priv_alt = 0;
  static float priv_press = 0;
  static float min_pressure_pa = 200000.0f;
  static int cooldown = 9000;
  static int sut_conf = 0;

  for (;;)
  {

	  vTaskDelayUntil(&xLastWakeTime, xFrequency);

	  // Frekansı ayarla
	  if (g_systemControl.ukb_mode == SIT || g_systemControl.ukb_mode == SUT) {
		  xFrequency = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS_10HZ);
	  } else if (g_systemControl.ukb_mode == NORMAL ){ // NORMAL mod
		  xFrequency = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS_80HZ);
	  }

    FlightData_t local_data;
	taskENTER_CRITICAL();
	local_data = g_flightData;
    memcpy(local_dma_buffer, huart4_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);
	taskEXIT_CRITICAL();

	if (g_systemControl.i2c_ok) {
       if(g_systemControl.ukb_mode != SUT){
    	   ICM42670P_ReadAccelGyroData(&local_data.imu_data);
    	   imu_get_sensor_data(&local_data.imu_data);

       }
       if(g_systemControl.ukb_mode == SUT && received_data_len == 36){
    	   local_data.imu_data.ax = convert_big_endian_float(&local_dma_buffer[17] ) / gravity;
    	   local_data.imu_data.ay = convert_big_endian_float(&local_dma_buffer[13]) / gravity;
    	   local_data.imu_data.az = convert_big_endian_float(&local_dma_buffer[9]) / gravity;
    	   apply_filter_euler(convert_big_endian_float(&local_dma_buffer[25]), convert_big_endian_float(&local_dma_buffer[21]), convert_big_endian_float(&local_dma_buffer[29]), &local_data.imu_data);
       }
	}
    if(g_systemControl.ukb_mode != SUT){
    	bmp390_get_sensor_data(&local_data.pressure_data);
    	bmp390_process_data(&local_data.pressure_data);

    }
    if(g_systemControl.ukb_mode == SUT && received_data_len == 36){
    	local_data.pressure_data.altitude = convert_big_endian_float(&local_dma_buffer[1]);
    }

	if(local_data.flight_state == ON_GROUND){
		if(g_systemControl.ukb_mode == SUT){
			if(local_data.imu_data.ax > 4){
				set_bit_u8(&local_data.stateData_l, 0);
				local_data.flight_state = PROTECTED_FLIGHT;
				local_data.takeoff_time = HAL_GetTick();
			}
		}else{
			if(local_data.pressure_data.pressure < local_data.pressure_data.press_calb - 200.0f){
				local_data.flight_state = PROTECTED_FLIGHT;
				local_data.takeoff_time = HAL_GetTick();
			}
		}
	}

	if(g_systemControl.ukb_mode == NORMAL){
		cooldown = 9000;
	}else{
		cooldown = 4000;
	}
	if(local_data.flight_state == PROTECTED_FLIGHT && HAL_GetTick() - local_data.takeoff_time > cooldown){
		set_bit_u8(&local_data.stateData_l, 1);
		if(g_systemControl.ukb_mode == NORMAL){
			local_data.flight_state = FREE_FLIGHT;
		}else{
			local_data.flight_state = ALTITUDE_PROTECTED_FLIGHT;
		}
	}


	if(local_data.flight_state == ALTITUDE_PROTECTED_FLIGHT && local_data.pressure_data.altitude > 950){
		set_bit_u8(&local_data.stateData_l, 2);//TEST DEVICE FLAGS

		local_data.flight_state = FREE_FLIGHT;
	}


	if(local_data.flight_state == FREE_FLIGHT){
		if(g_systemControl.ukb_mode == SUT){
			if(local_data.imu_data.angle_ground_normal_pr > 30){
				set_bit_u8(&local_data.stateData_l, 3);
				local_data.flight_state = APOGEE;
			}
		}else{
			local_data.flight_state = APOGEE;
		}
	}

	if(local_data.flight_state == APOGEE  && local_data.flight_state != TRIGGER){
	    local_data.vertical_velocity = calculate_velocity_from_fifo(altitude_fifo);
	    if(g_systemControl.ukb_mode == SUT){
	    	if(local_data.pressure_data.altitude < (priv_alt - 2)){
	    		sut_conf++;
	    	}else{
	    		sut_conf = 0;
	    	}

	    	if(sut_conf > 3){
	    		set_bit_u8(&local_data.stateData_l, 4);//TEST DEVICE FLAGS
				set_bit_u8(&local_data.stateData_l, 7);//TEST DEVICE FLAGS
				HAL_GPIO_TogglePin(LED_34BYTE_GPIO_Port, LED_34BYTE_Pin);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				local_data.flight_state = TRIGGER;
	    	}

	    	priv_alt = local_data.pressure_data.altitude;
	    }else{
	    	if (local_data.pressure_data.pressure < min_pressure_pa) {
	    	    min_pressure_pa = local_data.pressure_data.pressure;
	    	    apogee_confirmation_counter = 0;
	    	}

	    	if(local_data.pressure_data.pressure > min_pressure_pa + 60.0f){
				apogee_confirmation_counter++;
			}else{
				apogee_confirmation_counter = 0;
			}

			if (apogee_confirmation_counter >= 40) {
				set_bit_u8(&local_data.stateData_l, 4);//TEST DEVICE FLAGS
				set_bit_u8(&local_data.stateData_l, 7);//TEST DEVICE FLAGS
				HAL_GPIO_TogglePin(LED_34BYTE_GPIO_Port, LED_34BYTE_Pin);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				local_data.flight_state = TRIGGER;
			}
	    }

	}

	if(g_systemControl.ukb_mode == SUT){
		statePacket[0] = 0xAA;
		statePacket[1] = local_data.stateData_l;
		statePacket[2] = local_data.stateData_h;
		statePacket[3] = calculate_checksum_8bit(statePacket, 3);
		statePacket[4] = 0x0D;
		statePacket[5] = 0x0A;
		HAL_UART_Transmit(&huart4, statePacket, sizeof(statePacket), HAL_MAX_DELAY);

	}

	if(local_data.isDelayed){
	    vTaskDelay(pdMS_TO_TICKS(1000));
	    local_data.isDelayed = false;
	}
	if(g_systemControl.ukb_mode == SIT){
		memcpy(&sit_data[1], (convert_float_to_big_endian_bytes_return(local_data.pressure_data.altitude_sea).bytes) , 4);
		memcpy(&sit_data[5], (convert_float_to_big_endian_bytes_return(local_data.pressure_data.pressure / 100).bytes) , 4);
		memcpy(&sit_data[9],  (convert_float_to_big_endian_bytes_return(local_data.imu_data.ax * gravity).bytes) , 4);
		memcpy(&sit_data[13],  (convert_float_to_big_endian_bytes_return(local_data.imu_data.ay * gravity).bytes) , 4);
		memcpy(&sit_data[17],  (convert_float_to_big_endian_bytes_return(local_data.imu_data.az * gravity).bytes) , 4);
		memcpy(&sit_data[21],  (convert_float_to_big_endian_bytes_return(local_data.imu_data.pitch_filtered).bytes) , 4);
		memcpy(&sit_data[25],  (convert_float_to_big_endian_bytes_return(local_data.imu_data.roll_filtered).bytes) , 4);
		memcpy(&sit_data[29],  (convert_float_to_big_endian_bytes_return(local_data.imu_data.yaw_filtered).bytes) , 4);
		sit_data[33] = calculate_checksum_8bit(sit_data, 33);
		sit_data[34] = 0x0D;
		sit_data[35] = 0x0A;

		HAL_UART_Transmit(&huart4, sit_data, sizeof(sit_data), 100);
	}
	taskENTER_CRITICAL();
	g_flightData = local_data;
	taskEXIT_CRITICAL();


  }
}

void loraTask(void *pvParameters)
{

    static uint8_t LoraBuffer[TELEMETRY_PACKET_SIZE];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS); // 500ms

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        FlightData_t data_to_send;

        taskENTER_CRITICAL();
        data_to_send = g_flightData;
        taskEXIT_CRITICAL();

        if (g_systemControl.ukb_mode == NORMAL)
        {

            create_telemetry_packet_no_id(
                LoraBuffer,
                data_to_send.imu_data.ax,
                data_to_send.imu_data.ay,
                data_to_send.imu_data.az,
                data_to_send.imu_data.gx,
                data_to_send.imu_data.gy,
                data_to_send.imu_data.gz,
                data_to_send.imu_data.q_filtered.w,
                data_to_send.imu_data.q_filtered.x,
                data_to_send.imu_data.q_filtered.y,
                data_to_send.imu_data.q_filtered.z,
                data_to_send.gps_data.latitude,
                data_to_send.gps_data.longitude,
                data_to_send.gps_data.altitude, // GPS irtifası yerine hız hesabını kullandık
                data_to_send.pressure_data.altitude_sea,
                data_to_send.imu_data.angle_ground_normal_q,//angle_ground_normal_q
                data_to_send.flight_state
            );


            HAL_UART_Transmit(&huart3, LoraBuffer, sizeof(LoraBuffer), 100);

            HAL_GPIO_TogglePin(LED_34BYTE_GPIO_Port, LED_34BYTE_Pin);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance == USART2)
  {
    if (gps_rx_data == '\n')
    {
      gps_buffer[gps_index++] = gps_rx_data;
      gps_buffer[gps_index] = '\0';
      parse_gpgga(gps_buffer, &gps_data);
      g_flightData.gps_data = gps_data;
      gps_index = 0;
    }
    else
    {
      if (gps_index < sizeof(gps_buffer) - 1)
      {
        gps_buffer[gps_index++] = gps_rx_data;
      }
    }

    HAL_UART_Receive_IT(&huart2, &gps_rx_data, 1);
  }

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  HAL_UART_Receive_IT(&huart2, &gps_rx_data, 1);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart4, huart4_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);

  HAL_TIM_Base_Start(&htim2);

  HAL_Delay(1000);
  bmp390_sensor_init();
  HAL_Delay(100);
  bmp390_calibrate(&g_flightData.pressure_data);


  median_filter_reset();
  kalman_filter_reset();
  kalman_filter_euler_reset();


  g_systemControl.i2c_ok = ICM42670P_Init();
  if (g_systemControl.i2c_ok) {
	calibrate_gyro_offsets((ImuData *)&g_flightData.imu_data);
	set_initial_orientation((ImuData *)&g_flightData.imu_data);
	previous_tick = HAL_GetTick();
	send_usb_message("Normal olcum basliyor...\r\n");
  } else {
	send_usb_message("ICM Init Failed! Halting.\r\n");
  }


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  sutDataReadySemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  #define SENSOR_TASK_STACK_SIZE 1024
  #define LORA_TASK_STACK_SIZE   512

  BaseType_t sensorStatus = xTaskCreate(sensorTask, "SensorTask", SENSOR_TASK_STACK_SIZE, NULL, 3, &sensorTaskHandle);
  BaseType_t loraStatus = xTaskCreate(loraTask, "LoraTask", LORA_TASK_STACK_SIZE, NULL, 2, &loraTaskHandle);

  vTaskStartScheduler();

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB2 M1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : M0_Pin */
  GPIO_InitStruct.Pin = M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Diğer yardımcı fonksiyonlar buraya eklenebilir.
// scan_i2c2_with_led_error gibi debug fonksiyonları geliştirme sırasında faydalıdır.
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
send_usb_message("HAL Error Occurred! System Halted.\r\n");
__disable_irq();
while (1)
{
// Hata durumunda belirgin bir LED yak/söndür
// blink_led(GPIOB, GPIO_PIN_14, 1, 50); // Kırmızı LED varsa
}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
char msg[100];
snprintf(msg, sizeof(msg), "Assert Failed: file %s on line %lu\r\n", file, line);
send_usb_message(msg);
while (1) {
// blink_led(GPIOB, GPIO_PIN_14, 1, 25); // Kırmızı LED varsa hızlı yanıp sönsün
blink_led(GPIOB, D5_Pin, 1, 25); // Veya D5 pinini kullan
}
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
