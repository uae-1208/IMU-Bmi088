/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define Time_Up(time)  (time == 0)
#include <stdio.h>
#include "math.h"
#include "bmi088.h"
#include "bmi088reg.h"
#include "MahonyAHRS.h"
extern uint8_t Task_Time;
extern volatile float q0,q1,q2,q3;					// quaternion of sensor frame relative to auxiliary frame
extern float gx_offset,gy_offset,gz_offset;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  kp  0.005f
#define  ki  0.0f
float tem = 26.73f,target = 40.0f,error = 0;
double p_out,i_out = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	bmi088_data_t bmi088_data;
	Euler_Angle Mahony_angle,KF_angle;
	int i;
	float Acc_angle = 0;
	float q_gyro = 0;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	bmi088_data.bmi088_error = BMI088_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */  


    //0.001848, 0.001401, -0.002787   
	
//	for(i=0;i<30000;i++)
//	{
//		ReadGyroData(&(bmi088_data.gyro_data.gyro_raw_data));
//	    gx_offset += bmi088_data.gyro_data.gyro_raw_data.roll;
//	    gy_offset += bmi088_data.gyro_data.gyro_raw_data.pitch;
//	    gz_offset += bmi088_data.gyro_data.gyro_raw_data.yaw;
//        HAL_Delay(1);                                               
//	}                                                                 
//	gx_offset = gx_offset / 30000;                                                              
//	gy_offset = gy_offset / 30000;                                                              
//	gz_offset = gz_offset / 30000;                                                              

	
	while (1)
	{                                 
    /* USER CODE END WHILE */
		if(Task_Time == 0)         		//256us
		{
			Task_Time = 3;
//			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10); //示波器测代码运行时间

            //128us
			ReadAccData(&(bmi088_data.acc_data.acc_raw_data));
			ReadGyroData(&(bmi088_data.gyro_data.gyro_raw_data));
			ReadAccTemperature(&(bmi088_data.acc_data.temperature));  
			
//			printf("channels:%.3f\n",bmi088_data.acc_data.temperature);	
            			
			//5.1us			
			MahonyAHRSupdateIMU(bmi088_data.gyro_data.gyro_raw_data.roll,         //gx
								bmi088_data.gyro_data.gyro_raw_data.pitch,        //gy
								bmi088_data.gyro_data.gyro_raw_data.yaw,          //gz
								bmi088_data.acc_data.acc_raw_data.x,
								bmi088_data.acc_data.acc_raw_data.y,
								bmi088_data.acc_data.acc_raw_data.z);
			//125us
			Mahony_angle.pitch = asin(2.0f*(q0*q2 - q1*q3))*180.0f/PI;            
			Mahony_angle.yaw = atan((2.0f*(q0*q3+q1*q2))/(1.0f-2*(q2*q2+q3*q3)))*180.0f/PI;
			Mahony_angle.roll = atan((2.0f*(q0*q1+q2*q3))/(1.0f-2.0f*(q1*q1+q2*q2)))*180.0f/PI;
			
//			//加了这段代码后定时不准，运行周期从3ms变为3.58ms，但是为什么呢？
			printf("channels:%f, %f, %f\n",Mahony_angle.roll,Mahony_angle.pitch,Mahony_angle.yaw);			
		
		
//			{
//				Data_Ope_Before_KF(bmi088_data.acc_data.acc_raw_data.x,   			//41us
//								   bmi088_data.acc_data.acc_raw_data.z,
//								   bmi088_data.gyro_data.gyro_raw_data.pitch,
//								   &Acc_angle,&q_gyro);
//				KF_Pitch(Acc_angle,q_gyro,&(KF_angle.pitch));
//				Data_Ope_Before_KF(bmi088_data.acc_data.acc_raw_data.y,
//								   bmi088_data.acc_data.acc_raw_data.z,
//								   bmi088_data.gyro_data.gyro_raw_data.roll,
//								   &Acc_angle,&q_gyro);
//				KF_Roll(Acc_angle,q_gyro,&(KF_angle.roll)); //2.3us
//			}
			
//			printf("channels:%f, %f, %f, %f\n",Mahony_angle.pitch,KF_angle.pitch,Mahony_angle.roll,-KF_angle.roll);			
//			printf("channels:%f, %f\n",Mahony_angle.roll,-KF_angle.roll);			

		}


//		
//			printf("channels:%f, %f, %f\n",bmi088_data.gyro_data.gyro_raw_data.yaw,
//										   bmi088_data.gyro_data.gyro_raw_data.pitch,
//										   bmi088_data.gyro_data.gyro_raw_data.roll);

//			printf("channels:%f, %f, %f\n",bmi088_data.acc_data.acc_raw_data.x,
//									   bmi088_data.acc_data.acc_raw_data.y,
//								       bmi088_data.acc_data.acc_raw_data.z);
		
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
