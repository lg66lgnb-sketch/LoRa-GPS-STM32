/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_vehicle.c
  * @brief          : Main program body for Vehicle role
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6371000.0 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// --- GPS 与传感器变量 ---
uint32_t adc_value = 0;       
float temperature = 0.0f;     
volatile uint8_t rx_byte;              
char gps_buffer[100];         
volatile uint16_t rx_index = 0;        

// ！！！关键修复：给在中断中修改的变量加上 volatile ！！！
volatile double my_latitude = 0.0;     
volatile double my_longitude = 0.0;    
volatile uint8_t fix_quality = 0;      
volatile float hdop = 99.9f;           
volatile uint8_t satellites = 0;       
volatile float accuracy_meters = 0.0f; 
volatile float speed_kmh = 0.0f;       

double saved_latitude = 0.0;  
double saved_longitude = 0.0; 
double offset_distance = 0.0; 
double offset_bearing = 0.0;  
uint8_t point_is_saved = 0;   
uint8_t cmd_save_point = 0; 

// --- LoRa 通信专用变量 ---
UART_HandleTypeDef huart1;
volatile uint8_t lora_rx_byte;                 // 接收单个字符的缓存
volatile char lora_recv_msg[100] = {0}; // 在 Live Watch 观察收到的完整消息
volatile uint16_t lora_rx_index = 0;           // 接收消息的索引
uint32_t lora_send_count = 0;         // 发送心跳包的计数器

// --- 新增的 LoRa 调试标志位 ---
volatile uint8_t lora_receive_flag = 0;  // 1: 刚刚收到并解析了对端的有效定位数据
volatile uint8_t lora_ok_flag = 0;       // 1: 最近一次 HAL_UART_Transmit 成功执行
volatile uint32_t lora_rx_total_count = 0;  // 记录进入 LoRa 接收中断的总字节数
volatile uint32_t lora_rx_zero_count = 0;   // 记录收到 0x00 字节的次数
volatile uint32_t lora_last_rx_time = 0;    // 记录上次收到消息的时间戳

// --- 对端 (Peer) 节点信息变量 (Live Watch 监视) ---
volatile double peer_latitude = 0.0;     // 收到的对方纬度
volatile double peer_longitude = 0.0;    // 收到的对方经度
volatile float peer_speed_kmh = 0.0f;    // 收到的对方速度
volatile double peer_distance = 0.0;     // 相对对方的距离 (米)
volatile double peer_bearing = 0.0;      // 相对对方的方位角 (度)

// === V2X 专用状态变量 (Live Watch 监视) ===
volatile char received_warning[50] = "safe"; // 收到的控制指令
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double to_radians(double degree) {
    return degree * M_PI / 180.0;
}

double to_degrees(double rad) {
    return rad * 180.0 / M_PI;
}

void calculate_offset(double lat1, double lon1, double lat2, double lon2, double *dist, double *brng) {
    double phi1 = to_radians(lat1);
    double phi2 = to_radians(lat2);
    double delta_phi = to_radians(lat2 - lat1);
    double delta_lambda = to_radians(lon2 - lon1);

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    *dist = EARTH_RADIUS * c;

    double y = sin(delta_lambda) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);
    double b = atan2(y, x);
    *brng = fmod((to_degrees(b) + 360.0), 360.0);
}

void LoRa_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600; 
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    // 开启中断支持
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    HAL_UART_Init(&huart1);
}

// 必须添加 USART1 的硬件中断处理函数
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  
  /* USER CODE BEGIN 2 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_ADC_Start(&hadc1);
  
  // 启动 GPS 的 USART3 接收中断
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  
  // 初始化并启动 LoRa 的 USART1 接收中断
  LoRa_USART1_Init();
  HAL_UART_Receive_IT(&huart1, &lora_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) 
    {
        adc_value = HAL_ADC_GetValue(&hadc1);
        float vsense = (adc_value * 3.3f) / 4096.0f;
        temperature = ((1.43f - vsense) / 0.0043f) + 25.0f;
    }

    if (cmd_save_point == 1) 
    {
        if (fix_quality > 0) {
            saved_latitude = my_latitude;
            saved_longitude = my_longitude;
            point_is_saved = 1; 
        }
        cmd_save_point = 0; 
    }

    if (point_is_saved == 1 && fix_quality > 0) {
        calculate_offset(saved_latitude, saved_longitude, 
                         my_latitude, my_longitude, 
                         &offset_distance, &offset_bearing);
    }
    
    // --- 定时发送本机 GPS 数据给另一个 LoRa 模块 ---
    lora_receive_flag = 0; // 清零，方便观察下一个接收中断是否触发
    char tx_msg[80];
    if (fix_quality > 0) {
        int lat_int = (int)my_latitude;
        int lat_frac = (int)(fabs(my_latitude - lat_int) * 1000000.0);
        int lon_int = (int)my_longitude;
        int lon_frac = (int)(fabs(my_longitude - lon_int) * 1000000.0);
        int spd_int = (int)speed_kmh;
        int spd_frac = (int)(fabs(speed_kmh - spd_int) * 100.0);

        sprintf(tx_msg, "LOC:%d.%06d,%d.%06d,%d.%02d\n",
                lat_int, lat_frac,
                lon_int, lon_frac,
                spd_int, spd_frac);
    } else {
        sprintf(tx_msg, "LOC:0.000000,0.000000,0.00\n");
    }
    if (HAL_UART_Transmit(&huart1, (uint8_t*)tx_msg, strlen(tx_msg), 100) == HAL_OK) {
        lora_ok_flag = 1;
    } else {
        lora_ok_flag = 0;
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  */
static void MX_USART3_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
double NMEA_to_Decimal(double nmea_val) {
    int degrees = (int)(nmea_val / 100.0);
    double minutes = nmea_val - (degrees * 100.0);
    return (double)degrees + (minutes / 60.0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // --- 1. 处理来自 GPS 的 USART3 数据 ---
  if(huart->Instance == USART3)
  {
    gps_buffer[rx_index++] = (char)rx_byte;
    
    if (rx_byte == '\n' || rx_index >= 99) 
    {
      gps_buffer[rx_index] = '\0'; 
      
      if (strstr(gps_buffer, "GGA") != NULL) 
      {
        char *fields[15];
        int f = 0;
        char *ptr = gps_buffer;
        
        fields[f++] = ptr;
        while(*ptr && f < 15) {
            if(*ptr == ',') {
                *ptr = '\0'; 
                fields[f++] = ptr + 1; 
            }
            ptr++;
        }
        
        if (f >= 10) {
            fix_quality = atoi(fields[6]); 
            satellites = atoi(fields[7]);     
            hdop = atof(fields[8]);           
            
            if(hdop > 0.1f) {
                accuracy_meters = hdop * 1.5f; 
            }

            if (fix_quality > 0) {
                my_latitude = NMEA_to_Decimal(atof(fields[2]));
                my_longitude = NMEA_to_Decimal(atof(fields[4]));
                
                if (fields[3][0] == 'S') my_latitude = -my_latitude;
                if (fields[5][0] == 'W') my_longitude = -my_longitude;
            }
        }
      }
      else if (strstr(gps_buffer, "RMC") != NULL) 
      {
        char *fields[15];
        int f = 0;
        char *ptr = gps_buffer;
        
        fields[f++] = ptr;
        while(*ptr && f < 15) {
            if(*ptr == ',') {
                *ptr = '\0'; 
                fields[f++] = ptr + 1; 
            }
            ptr++;
        }
        
        if (f >= 8 && fields[2][0] == 'A') {
            float knots = atof(fields[7]);
            speed_kmh = knots * 1.852f;
        } else if (fields[2][0] == 'V') {
            speed_kmh = 0.0f; 
        }
      }
      rx_index = 0; 
    }
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
  
  // --- 2. 处理来自 LoRa 的 USART1 数据 ---
  else if(huart->Instance == USART1)
  {
    lora_rx_total_count++;
    if (lora_rx_byte == 0x00) lora_rx_zero_count++;
    else lora_ok_flag = 1;

    lora_recv_msg[lora_rx_index++] = (char)lora_rx_byte;
    
    if (lora_rx_byte == '\n' || lora_rx_index >= 99) 
    {
      lora_recv_msg[lora_rx_index] = '\0'; 
      
      if (strncmp((char*)lora_recv_msg, "CMD:", 4) == 0) 
      {
          strncpy(received_warning, (char*)lora_recv_msg + 4, sizeof(received_warning) - 1);
          received_warning[sizeof(received_warning) - 1] = '\0';

          char *newline = strchr(received_warning, '\n');
          if (newline != NULL) *newline = '\0';
          
          lora_receive_flag = 1; 
      }
      
      lora_rx_index = 0; 
    }
    HAL_UART_Receive_IT(&huart1, &lora_rx_byte, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
