/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>

extern bool cubemx_transport_open(struct uxrCustomTransport *transport);
extern bool cubemx_transport_close(struct uxrCustomTransport *transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport *transport,
    const uint8_t *buf, size_t len, uint8_t *err);
extern size_t cubemx_transport_read(struct uxrCustomTransport *transport,
    uint8_t *buf, size_t len, int timeout, uint8_t *err);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for microROSTask */
osThreadId_t microROSTaskHandle;
const osThreadAttr_t microROSTask_attributes = {
  .name = "microROSTask",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32      rover_status_msg;
geometry_msgs__msg__Twist wheel_odom_msg;
static uint32_t last_cmd_vel_time = 0;
#define CMD_VEL_TIMEOUT_MS  500  /* 500ms 타임아웃 */

/* 엔코더 상수 */
#define ENCODER_PPR        3087 /*1300   /* 26 × 50 (감속비) */
#define WHEEL_DIAMETER_M   0.08f /* 85mm */
#define WHEEL_CIRCUMFERENCE (3.14159f * WHEEL_DIAMETER_M)
/* 엔코더 변수 */
static int32_t enc_left_prev  = 0;
static int32_t enc_right_prev = 0;
static float   vel_left  = 0.0f;  /* m/s */
static float   vel_right = 0.0f;  /* m/s */

/* Ackermann 기하학 파라미터 */
#define WHEELBASE_M      0.175f  /* 축거 L */
#define TRACK_WIDTH_M    0.200f  /* 바퀴 간 거리 T */
#define MAX_STEER_ANGLE_RAD  0.5236f  /* 30도 = 0.5236 rad, 실측 후 조정 필요 */


/* ESP32로 전송할 텔레메트리 구조체 */
typedef struct __attribute__((packed)) {
    uint8_t  header;      /* 0xAA 고정 */
    uint16_t battery_mv;  /* 배터리 전압 */
    int16_t  motor_l_pwm; /* 왼쪽 모터 PWM */
    int16_t  motor_r_pwm; /* 오른쪽 모터 PWM */
    float    heading;     /* 방향각 */
    uint8_t  status;      /* 0:대기 1:주행 2:비상정지 */
    uint8_t  checksum;    /* XOR 체크섬 */
} rover_telem_t;

/* PID 게인 (초기값, 튜닝 필요) */
#define PID_KP   0.6f
#define PID_KI   0.6f
#define PID_KD   0.02f
#define PID_MAX_INTEGRAL  0.6f  /* 적분 와인드업 방지 */

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} PID_t;

static PID_t pid_left  = {PID_KP, PID_KI, PID_KD, 0.0f, 0.0f};
static PID_t pid_right = {PID_KP, PID_KI, PID_KD, 0.0f, 0.0f};

/* 가속도 제한 상태 변수 */
static float smoothed_left  = 0.0f;  /* 현재 실제 적용 중인 목표속도 */
static float smoothed_right = 0.0f;
#define MAX_ACCEL  0.6f   /* m/s^2, 튜닝 필요 */

/* 데드존 보상 */
#define BREAKAWAY_PWM   15000   /* 정지 → 출발 시 필요한 PWM (12V 기준 실측) */
#define HOLDING_PWM     500   /* 이미 주행 중일 때 유지 최소 PWM (12V 기준 실측) */
#define MOVING_THRESHOLD 0.02f /* 이 속도(m/s) 이상이면 "이미 돌고 있다"고 판단 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void micro_ros_task(void *argument);

/* USER CODE BEGIN PFP */
void cmd_vel_callback(const void *msgin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* PWM 범위: 0 ~ 19999 (Period) */
#define SERVO_NEUTRAL   1500
#define SERVO_MAX       2000
#define SERVO_MIN       1000
#define MOTOR_MAX       17999   /* 최대 80% duty (처음엔 안전하게) */

void set_servo_from_ackermann(float linear_x, float angular_z)
{
    float delta;
    const float V_MIN_FOR_STEERING = 0.05f;
    const float V_FULL_STEERING = 0.15f;   /* 이 속도 이상에서만 완전한 atan 계산 신뢰 */

    float abs_v = fabsf(linear_x);

    if (abs_v < V_MIN_FOR_STEERING) {
        delta = 0.0f;
    } else {
        /* atan 계산 시 분모를 V_FULL_STEERING 이상으로
           강제해서 분모가 너무 작아지는 걸 방지 */
        float effective_v = (abs_v < V_FULL_STEERING) ? V_FULL_STEERING : abs_v;
        float sign = (linear_x >= 0) ? 1.0f : -1.0f;
        delta = atanf(WHEELBASE_M * angular_z / (sign * effective_v));

        /* 저속 구간에서는 delta를 선형으로 줄여서 부드럽게 전환 */
        if (abs_v < V_FULL_STEERING) {
            float blend = (abs_v - V_MIN_FOR_STEERING)
                        / (V_FULL_STEERING - V_MIN_FOR_STEERING);
            delta *= blend;  /* 0~1 사이로 부드럽게 감쇠 */
        }
    }

    if (delta >  MAX_STEER_ANGLE_RAD) delta =  MAX_STEER_ANGLE_RAD;
    if (delta < -MAX_STEER_ANGLE_RAD) delta = -MAX_STEER_ANGLE_RAD;

    float angular_z_normalized = delta / MAX_STEER_ANGLE_RAD;
    int pulse = SERVO_NEUTRAL + (int)(-angular_z_normalized * 500.0f);
    if (pulse > SERVO_MAX) pulse = SERVO_MAX;
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

void set_servo_neutral(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_NEUTRAL);
}

void set_servo(float angular_z)
{
    /* angular_z: -1.0(최대 좌) ~ +1.0(최대 우) */
    int pulse = SERVO_NEUTRAL + (int)(-angular_z * 500.0f);
    if (pulse > SERVO_MAX) pulse = SERVO_MAX;
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}


void apply_motor_left_pwm(int pwm)
{
    if (pwm > 0) {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_SET);
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
        pwm = -pwm;
    } else {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
    }
    if (pwm > MOTOR_MAX) pwm = MOTOR_MAX;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
}

void apply_motor_right_pwm(int pwm)
{
    if (pwm > 0) {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_SET);
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);
        pwm = -pwm;
    } else {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);
    }
    if (pwm > MOTOR_MAX) pwm = MOTOR_MAX;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);
}

static void test_deadzone_sweep(void)
{
    static int test_pwm = 17000;   /* 큰 값에서 시작해서 확실히 출발 */
    static uint32_t last_step_time = 0;
    static bool started = false;
    uint32_t now = osKernelGetTickCount();

    if (!started) {
        apply_motor_left_pwm(test_pwm);
        apply_motor_right_pwm(test_pwm);
        last_step_time = now;
        started = true;
        rover_status_msg.data = test_pwm;
        return;
    }

    if (now - last_step_time > 3000) {   /* 3초씩 유지하며 관찰 */
        test_pwm -= 500;   /* 점진적으로 감소 */
        if (test_pwm < 500) test_pwm = 17000;  /* 다시 리셋 */
        apply_motor_left_pwm(test_pwm);
        apply_motor_right_pwm(test_pwm);
        last_step_time = now;
        rover_status_msg.data = test_pwm;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  /* 서보 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  /* 왼쪽 모터 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  /* 오른쪽 모터 */

  /* 엔코더 시작 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  /* 왼쪽 엔코더 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  /* 오른쪽 엔코더 */

  /* 엔코더 카운터 초기화 */
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of microROSTask */
  microROSTaskHandle = osThreadNew(micro_ros_task, NULL, &microROSTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MOTOR_L_IN1_Pin|MOTOR_L_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MOTOR_L_IN1_Pin MOTOR_L_IN2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MOTOR_L_IN1_Pin|MOTOR_L_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_R_IN3_Pin */
  GPIO_InitStruct.Pin = MOTOR_R_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_R_IN3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_R_IN4_Pin */
  GPIO_InitStruct.Pin = MOTOR_R_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_R_IN4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* 전역 목표 변수 추가 */
static volatile float target_linear_x  = 0.0f;
static volatile float target_angular_z = 0.0f;

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    /* 마지막 수신 시간 업데이트 */
    last_cmd_vel_time = osKernelGetTickCount();

    target_linear_x  = msg->linear.x;   /* -1.0 ~ +1.0 */
    target_angular_z = msg->angular.z;  /* -1.0 ~ +1.0 */

    /* 서보 조향 */
    set_servo_from_ackermann(target_linear_x, target_angular_z);
}

/* Failsafe 함수 */
void emergency_stop(void)
{
	apply_motor_left_pwm(0);
	apply_motor_right_pwm(0);
    set_servo_neutral();   /* set_servo(0.0f) 대신 */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); /* LED OFF */

    /* PID/가속도 제한 상태 리셋 - 재출발 시 이전 값 영향 방지 */
	smoothed_left  = 0.0f;
	smoothed_right = 0.0f;
	pid_left.integral   = 0.0f;
	pid_left.prev_error = 0.0f;
	pid_right.integral   = 0.0f;
	pid_right.prev_error = 0.0f;

	/* 목표값도 0으로 (재출발 시 갑자기 튀는 것 방지) */
	target_linear_x  = 0.0f;
	target_angular_z = 0.0f;
}

/* 체크섬 계산 */
static uint8_t calc_checksum(uint8_t *data, uint8_t len)
{
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

/* ESP32로 텔레메트리 전송 */
void send_telemetry_to_esp32(void)
{
    rover_telem_t telem;
    telem.header     = 0xAA;
    telem.battery_mv = 7400;  /* TODO: 실제 ADC 측정값 */

    /* 현재 모터 PWM값 읽기 */
    telem.motor_l_pwm = (int16_t)__HAL_TIM_GET_COMPARE(
                            &htim2, TIM_CHANNEL_2);
    telem.motor_r_pwm = (int16_t)__HAL_TIM_GET_COMPARE(
                            &htim2, TIM_CHANNEL_3);
    telem.heading     = 0.0f;  /* TODO: BNO055 연동 후 */
    telem.status      = 1;

    /* 체크섬 계산 (header 제외) */
    telem.checksum = calc_checksum(
        (uint8_t*)&telem + 1,
        sizeof(rover_telem_t) - 2
    );

    /* USART6으로 전송 */
    HAL_UART_Transmit(&huart6,
        (uint8_t*)&telem,
        sizeof(rover_telem_t),
        100);
}

void read_encoder(float dt)
{
    /* 현재 카운터 값 읽기 */
    int32_t enc_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t enc_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    /* 오버플로우 처리 */
    int32_t delta_left  = enc_left  - enc_left_prev;
    int32_t delta_right = enc_right - enc_right_prev;

    /* 오버플로우 보정 */
    if (delta_left  > 32767)  delta_left  -= 65536;
    if (delta_left  < -32767) delta_left  += 65536;
    if (delta_right > 32767)  delta_right -= 65536;
    if (delta_right < -32767) delta_right += 65536;

    /* 속도 계산 (m/s) */
    vel_left  = (float)delta_left  / ENCODER_PPR
                * WHEEL_CIRCUMFERENCE / dt;
    vel_right = (float)delta_right / ENCODER_PPR
                * WHEEL_CIRCUMFERENCE / dt;

    enc_left_prev  = enc_left;
    enc_right_prev = enc_right;
}

/* 서보 pulse → 실제 조향각(라디안) 역산 */
static float get_steering_angle_rad(void)
{
    /* set_servo에서: pulse = 1500 - angular_z*500 (angular_z: -1~1) */
    /* 최대 조향각을 실측 또는 가정값으로 설정 (예: ±30도) */
    int current_pulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    float angular_z_est = -(float)(current_pulse - SERVO_NEUTRAL) / 500.0f;
    return angular_z_est * MAX_STEER_ANGLE_RAD;
}

/* 오른쪽 바퀴 속도로부터 중심 속도, 왼쪽 바퀴 속도 계산 */
static void estimate_wheel_speeds(float v_right_meas, float *v_center_out, float *v_left_out)
{
    float delta = get_steering_angle_rad();

    if (fabsf(delta) < 0.01f) {
        /* 직진: 좌우 동일 */
        *v_center_out = v_right_meas;
        *v_left_out   = v_right_meas;
        return;
    }

    float R = WHEELBASE_M / tanf(delta);  /* 회전 반경 (부호 있음) */

    /* v_right = v_center * (R + T/2) / R  →  v_center 역산 */
    float v_center = v_right_meas * R / (R + TRACK_WIDTH_M / 2.0f);
    float v_left   = v_center * (R - TRACK_WIDTH_M / 2.0f) / R;

    *v_center_out = v_center;
    *v_left_out   = v_left;
}

/* Ackermann 모델 기반 yaw_rate 계산 */
static float compute_yaw_rate(float v_center)
{
    float delta = get_steering_angle_rad();
    if (fabsf(delta) < 0.001f) return 0.0f;
    return v_center * tanf(delta) / WHEELBASE_M;
}

/* PID 계산: target(m/s), actual(m/s), dt(s) → PWM 보정값(-1~1 범위 duty) */
static float pid_update(PID_t *pid, float target, float actual, float dt)
{
    float error = target - actual;

    pid->integral += error * dt;
    if (pid->integral >  PID_MAX_INTEGRAL) pid->integral =  PID_MAX_INTEGRAL;
    if (pid->integral < -PID_MAX_INTEGRAL) pid->integral = -PID_MAX_INTEGRAL;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    return output;  /* duty 보정값, -1.0~1.0 근처로 clamp 필요 */
}

/* 가속도 제한: 목표속도를 부드럽게 추종 */
static float apply_accel_limit(float *smoothed, float target, float dt)
{
    float max_delta = MAX_ACCEL * dt;
    float diff = target - *smoothed;

    if (diff >  max_delta) diff =  max_delta;
    if (diff < -max_delta) diff = -max_delta;

    *smoothed += diff;
    return *smoothed;
}
#define BREAKAWAY_HOLD_MS  300

static bool     in_breakaway = false;
static uint32_t breakaway_start_time = 0;
/* 데드존 보상: 목표 duty가 있는데 너무 작으면 최소 구동력 보장 */
static int apply_deadzone(float duty_normalized, float current_actual_speed)
{
    //uint32_t now = osKernelGetTickCount();

	if (fabsf(duty_normalized) < 0.02f) {
		return 0;  /* 완전 정지 목표는 그대로 0 */
	}
    /* duty_normalized: -1.0 ~ 1.0 */
    int pwm = (int)(duty_normalized * MOTOR_MAX);
    /*bool already_moving = fabsf(current_actual_speed) > MOVING_THRESHOLD;
    int threshold = already_moving ? HOLDING_PWM : BREAKAWAY_PWM;

    if (pwm > 0 && pwm < threshold) pwm = threshold;
    if (pwm < 0 && pwm > -threshold) pwm = -threshold;*/
    /*if (!already_moving && !in_breakaway) {
            in_breakaway = true;
            breakaway_start_time = now;
        }

        if (in_breakaway) {
            if ((now - breakaway_start_time) < BREAKAWAY_HOLD_MS) {
                if (pwm > 0 && pwm < BREAKAWAY_PWM) pwm = BREAKAWAY_PWM;
                if (pwm < 0 && pwm > -BREAKAWAY_PWM) pwm = -BREAKAWAY_PWM;
                return pwm;
            } else {
                in_breakaway = false;
            }
        }

        if (pwm > 0 && pwm < HOLDING_PWM) pwm = HOLDING_PWM;
        if (pwm < 0 && pwm > -HOLDING_PWM) pwm = -HOLDING_PWM;*/

    return pwm;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_micro_ros_task */
/**
  * @brief  Function implementing the microROSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_micro_ros_task */
void micro_ros_task(void *argument)
{
  /* USER CODE BEGIN 5 */
	rmw_uros_set_custom_transport(
	      true, (void *)&huart2,
	      cubemx_transport_open,
	      cubemx_transport_close,
	      cubemx_transport_write,
	      cubemx_transport_read
	  );

	  rcl_allocator_t    allocator = rcl_get_default_allocator();
	  rclc_support_t     support;
	  rcl_node_t         node;
	  rcl_subscription_t subscriber;
	  rcl_publisher_t    publisher;
	  rcl_publisher_t wheel_odom_publisher;
	  rclc_executor_t    executor;

	  /* Agent 연결 대기 */
	  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
	      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	      osDelay(100);
	  }
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); /* 연결 성공 LED ON */

	  rclc_support_init(&support, 0, NULL, &allocator);
	  rclc_node_init_default(&node, "stm32_rover_node", "", &support);

	  rclc_subscription_init_default(
	      &subscriber, &node,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	      "/cmd_vel"
	  );

	  rclc_publisher_init_default(
	      &publisher, &node,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	      "/rover_status"
	  );

	  rclc_publisher_init_default(
	      &wheel_odom_publisher, &node,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	      "/wheel_odom"
	  );

	  rclc_executor_init(&executor, &support.context, 1, &allocator);
	  rclc_executor_add_subscription(
	      &executor, &subscriber,
	      &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA
	  );

	  //rover_status_msg.data = 0;
	  static uint32_t telem_counter = 0;
#define TEST_MODE_SWEEP  0   /* 1이면 스윕 테스트, 0이면 정상 주행 */

	  for (;;) {
	      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

	      /* 엔코더 읽기 (100ms마다) */
	      read_encoder(0.1f);  /* dt = 100ms */
#if TEST_MODE_SWEEP
	      test_deadzone_sweep();
#else
	      /* 목표 좌/우 바퀴 속도 계산 (기존 기하학 함수 재사용)*/
		  float target_v_center = target_linear_x;
		  float target_correction = target_angular_z * 0.3f;
		  float target_left  = target_v_center;
		  float target_right = target_v_center;

		  /* 가속도 제한 적용*/
		  float smooth_left_target  = apply_accel_limit(&smoothed_left,  target_left,  0.1f);
		  float smooth_right_target = apply_accel_limit(&smoothed_right, target_right, 0.1f);

		  /* PID로 실제 속도 추종 (vel_left, vel_right는 read_encoder에서 갱신)*/
		  float pid_out_left  = pid_update(&pid_left,  smooth_left_target,  vel_right,  0.1f);
		  float pid_out_right = pid_update(&pid_right, smooth_right_target, vel_right, 0.1f);

		  /* PID 출력을 duty(-1~1)로 clamp*/
		  if (pid_out_left  >  1.0f) pid_out_left  =  1.0f;
		  if (pid_out_left  < -1.0f) pid_out_left  = -1.0f;
		  if (pid_out_right >  1.0f) pid_out_right =  1.0f;
		  if (pid_out_right < -1.0f) pid_out_right = -1.0f;

		  /* 데드존 보상 + PWM 적용*/
		  if (fabsf(target_left) < 0.01f && fabsf(vel_right) < 0.01f) {
			  apply_motor_left_pwm(0);
			  pid_left.integral = 0;  /* 정지 시 적분 리셋*/
		  } else {
			  apply_motor_left_pwm(apply_deadzone(pid_out_right, vel_right));
		  }

		  if (fabsf(target_right) < 0.01f && fabsf(vel_right) < 0.01f) {
			  apply_motor_right_pwm(0);
			  pid_right.integral = 0;
		  } else {
			  apply_motor_right_pwm(apply_deadzone(pid_out_right, vel_right));
		  }
#endif
	      float v_center, v_left_est;
	      estimate_wheel_speeds(vel_right, &v_center, &v_left_est);
	      float yaw_rate = compute_yaw_rate(v_center);

	      /* wheel_odom 발행 (Twist) */
		  wheel_odom_msg.linear.x  = v_center;
		  wheel_odom_msg.linear.y  = 0.0;
		  wheel_odom_msg.linear.z  = 0.0;
		  wheel_odom_msg.angular.x = 0.0;
		  wheel_odom_msg.angular.y = 0.0;
		  wheel_odom_msg.angular.z = yaw_rate;
		  rcl_publish(&wheel_odom_publisher, &wheel_odom_msg, NULL);

	      /* Failsafe 체크*/
		  uint32_t current_time = osKernelGetTickCount();
		  if ((current_time - last_cmd_vel_time) > CMD_VEL_TIMEOUT_MS) {
			  emergency_stop();
		  }
	      rover_status_msg.data = (int32_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);//++;
	      (void)rcl_publish(&publisher, &rover_status_msg, NULL);

	      /* ESP32로 텔레메트리 전송 (1초마다) */
	      if(++telem_counter >= 10){
	    	  send_telemetry_to_esp32();
	    	  telem_counter = 0;
	      }
	      osDelay(100);
	  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
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
  __disable_irq();
  while (1)
  {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
