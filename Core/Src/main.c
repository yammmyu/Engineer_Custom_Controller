/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dvc_timebase.h"
#include "dvc_serialplot.h"
#include "dvc_motor.h"
#include "alg_pid.h"
#include "dvc_motor_config.h"
#include "drv_can.h"
#include "math.h"
#include "drv_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
UART_Manage_t uart2_mgr;   // one per UART

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
Serialplot_t plot;
volatile int16_t debug_cmd_motor7 = 0;
#define SEND_INTERVAL_MS 100 // Send data every 100ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float var1, var2, var3;

AnglesAndButtons_t sensor_data = {0};
ControllerFrame_t tx_frame = {0};
static uint32_t last_send_time = 0; // Track last send time for periodic transmission
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void collect_and_send_data(void); // Function to collect and send angle data
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();                    // init TIM6 first
  HAL_TIM_Base_Start_IT(&htim6);     // then start it with interrupt enabled
  protocol_init();

  /* USER CODE BEGIN 2 */
  Enable_CAN2();
  Motors_Init();
  Motor_Set_Target_Angle(&motors[3], -1.8f);
  Motor_Set_Target_Angle(&motors[4], -2.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check if it's time to send data (every 100ms)
    if (HAL_GetTick() - last_send_time >= SEND_INTERVAL_MS) {
        collect_and_send_data();
        last_send_time = HAL_GetTick();
    }
  }
  /* USER CODE END 3 */
}


void send_to_vtm(const ControllerFrame_t* frame) {
    // Transmit via UART (blocking)
    HAL_UART_Transmit(&huart2, (uint8_t*)frame, DATA_FRAME_LENGTH, HAL_MAX_DELAY);
}
/**
  * @brief Collect angle data and send via UART
  * @retval None
  *
  *
  */
void collect_and_send_data(void) {
    // Populate sensor_data with motor angles and button statuses
    for (int i = 0; i < 6; i++) {
        sensor_data.angle_data[i] = motors[i].Now_Angle; // Assuming motors[i].Angle exists
    }
    // Set button statuses (example: all buttons off; modify as needed)
    sensor_data.button_status[0] = 0;
    sensor_data.button_status[1] = 0;
    sensor_data.button_status[2] = 0;

    // Prepare the frame
    protocol_concatenate_data(&sensor_data, &tx_frame);

    // Send the frame via UART
    send_to_vtm(&tx_frame);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            Motor_TIM_PID_PeriodElapsedCallback(&motors[i]);
        }

        Set_C620_Current((int16_t)motors[0].Out,
                         (int16_t)motors[1].Out,
                         (int16_t)motors[2].Out);

        Set_GM6020_Voltage((int16_t)motors[3].Out,
                           (int16_t)motors[4].Out,
                           (int16_t)motors[5].Out);
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
      HAL_Delay(250); // blink every 250ms
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
