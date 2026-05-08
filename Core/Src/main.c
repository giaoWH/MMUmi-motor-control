/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"

// 新增：引入USB虚拟串口通信接口和标准输入输出库
#include "usbd_cdc_if.h"
#include <stdio.h>
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

/* USER CODE BEGIN PV */
// 【修改点】增加用于软件零点对齐的全局变量
volatile uint8_t zero_calibrated = 0; // 0:未校准, 1:已校准
float offset_motor1 = 0.0f;           // A电机初始绝对位置
float offset_motor2 = 0.0f;           // B电机初始绝对位置
extern FDCAN_HandleTypeDef hfdcan2;   // 引入FDCAN2句柄

// 新增：USB发送相关的定时变量和缓冲区
uint32_t last_usb_tx_time = 0;
char usb_tx_buf[128];                 // USB发送缓冲区
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {
		
        // 【修改点】双向力反馈交叉赋值控制逻辑
        if (zero_calibrated) {
            // 1. 计算以各自上电时刻为基准的【相对位置】
            float rel_pos1 = motor[Motor1].para.pos - offset_motor1;
            float rel_pos2 = motor[Motor2].para.pos - offset_motor2;

            // 2. 交叉追踪逻辑 (目标位置 = 对方的相对位置 + 自己的零点偏移)
            motor[Motor1].ctrl.pos_set = rel_pos2 + offset_motor1;
            motor[Motor1].ctrl.vel_set = motor[Motor2].para.vel;
            
            // 【新增】给Motor1（FDCAN1）增加恒定前馈偏置扭矩 0.3Nm
            // 注意：达妙电机的正负号决定顺逆时针，如果你发现 0.3f 是顺时针，请改为 -0.3f
            motor[Motor1].ctrl.tor_set = 0.3f; 

            motor[Motor2].ctrl.pos_set = rel_pos1 + offset_motor2;
            motor[Motor2].ctrl.vel_set = motor[Motor1].para.vel;
            
            // 【新增】确保Motor2没有前馈扭矩
            motor[Motor2].ctrl.tor_set = 0.0f; 
        } else {
            // 在未校准前，强制目标位置等于当前位置，保持完全静止，防止意外运动
            motor[Motor1].ctrl.pos_set = motor[Motor1].para.pos;
            motor[Motor1].ctrl.vel_set = 0.0f;
            motor[Motor1].ctrl.tor_set = 0.0f; // 【新增】未校准前，绝对不能给前馈扭矩
            
            motor[Motor2].ctrl.pos_set = motor[Motor2].para.pos;
            motor[Motor2].ctrl.vel_set = 0.0f;
            motor[Motor2].ctrl.tor_set = 0.0f; // 【新增】未校准前，绝对不能给前馈扭矩
        }

        // 分别通过 FDCAN1 和 FDCAN2 向两个电机发送控制指令
        dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
        dm_motor_ctrl_send(&hfdcan2, &motor[Motor2]);
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
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_FDCAN2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	power(1);
	HAL_Delay(1000);
	
    // 【修改点】同时设置两个CAN的波特率
	bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
    bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);

	bsp_can_init();
	dm_motor_init();
	
	HAL_Delay(100);
	
    // (注释掉你原有保存参数的这部分测试代码，直接进入控制逻辑)
	/*
	write_motor_data(motor[Motor1].id, 10, mit_mode, 0, 0, 0);
	HAL_Delay(100);
	read_motor_data(motor[Motor1].id, RID_CAN_BR); 
	dm_motor_disable(&hfdcan1, &motor[Motor1]);
	HAL_Delay(100);
	save_motor_data(motor[Motor1].id, 10);
	HAL_Delay(100);
    */

    // 【修改点】使能两个电机 (此时 Kp=0, Kd=0，电机处于自由状态)
	dm_motor_enable(&hfdcan1, &motor[Motor1]);
    dm_motor_enable(&hfdcan2, &motor[Motor2]);
	
    // 开启控制循环定时器，触发电机开始不断回传当前位置
	HAL_TIM_Base_Start_IT(&htim3);
    
    // 【修改点】等待500ms，确保接收到两台电机的反馈帧数据
	HAL_Delay(500);

    // 【修改点】记录此时两台电机的绝对位置，作为各自的“软件零点”
    offset_motor1 = motor[Motor1].para.pos;
    offset_motor2 = motor[Motor2].para.pos;
    
    // 【修改点】平滑恢复刚度与阻尼，此时因为相对误差绝对为0，瞬间施加力矩也完全不会跳跃！
    // 注意：Kp、Kd的数值请根据实际手感微调，1.0 和 0.1 只是安全起步值
    // （修复潜在数据竞争：先设置控制参数，再解除校准锁定标志）
    motor[Motor1].ctrl.kp_set = 2.0f; 
    motor[Motor1].ctrl.kd_set = 0.008f; 
    motor[Motor2].ctrl.kp_set = 2.0f; 
    motor[Motor2].ctrl.kd_set = 0.008f; 

    // 标志位置1，控制循环会开始进行相对位置耦合运算
    zero_calibrated = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // 设定 10ms (100Hz) 的频率发送数据，避免高频发包阻塞主循环
      if (HAL_GetTick() - last_usb_tx_time >= 10) 
      {
          last_usb_tx_time = HAL_GetTick();
          
          // 确保电机已经初始化并对齐零点，避免输出全 0 的无效数据
          if (zero_calibrated) 
          {
              // 安全的格式化字符串，防止内存溢出
              int len = snprintf(usb_tx_buf, sizeof(usb_tx_buf), 
                               "M1: P:%.3f V:%.2f T:%.2f | M2: P:%.3f V:%.2f T:%.2f\r\n",
                               motor[Motor1].para.pos, motor[Motor1].para.vel, motor[Motor1].para.tor,
                               motor[Motor2].para.pos, motor[Motor2].para.vel, motor[Motor2].para.tor);
              
              if (len > 0) {
                  // 通过 USB HS 虚拟串口接口发送数据
                  // 如果返回 BUSY 就直接丢弃本次数据，保证控制系统的实时性
                  if (CDC_Transmit_HS((uint8_t *)usb_tx_buf, (uint16_t)len) == USBD_BUSY) {
                      // 丢弃策略：繁忙时不等待，直接进入下一次循环
                  }
              }
          }
      }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
