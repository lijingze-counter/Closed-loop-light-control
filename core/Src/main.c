/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "LCD1602.h"
#include "key.h"
#include "math.h"
#define  LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define  LED_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
uint16_t ADC_value=0;
uint16_t ADC_volt=0;
uint16_t LIGHT=0;
uint16_t LIGHT_SHOW=0;
uint16_t SET_value=1000;
uint8_t strr_buff[64];
uint8_t strr_buff1[64];
uint16_t delay_pwm=0;

double a;
double R;
uint16_t pwm_cycle;
double Position_Kp=0.04;
double Position_Ki=0.002;
double Position_Kd=0;
int k;


void UR1_SEND_info()
{
	sprintf((char*)strr_buff1,"light:%d%d%d%d LUX",LIGHT/1000,(LIGHT%1000)/100,(LIGHT%100)/10,LIGHT%10);
	sprintf((char*)strr_buff," pwm:%d%d.%d%%%\r\n",pwm_cycle/100,(pwm_cycle%100)/10,pwm_cycle%10);
	
		HAL_UART_Transmit(&huart1,strr_buff1,sizeof(strr_buff1),1000);
	HAL_UART_Transmit(&huart1,strr_buff,sizeof(strr_buff),1000);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		ADC_value=HAL_ADC_GetValue(&hadc1);
		ADC_volt=ADC_value*3300/4096;
		R=ADC_volt/(3300.0-ADC_volt);
//		UR1_SEND_info();
		LED_OFF();
}	

void PWM_duty_ratio(int duty_ratio)
		{
			while(delay_pwm<duty_ratio)
				{
			delay_pwm++;
			TIM3->CCR1=delay_pwm;
			
				}
		
			while(delay_pwm>duty_ratio)
				{
			delay_pwm--;
			TIM3->CCR1=delay_pwm;
				}
  }

	void LCD_SHOW() 
	{
			  lcd_put_cur(0, 0);
		//lcd_send_string("light: ");
    
		lcd_send_string("cyc");
		lcd_send_data(pwm_cycle/100+48);
		lcd_send_data((pwm_cycle%100)/10+48);
		lcd_send_string(".");
		lcd_send_data((pwm_cycle%10)+48);
		lcd_send_string("%");
		lcd_send_string(" ");
		//lcd_send_data (((LIGHT%10000)/1000)+48)
		lcd_send_data ((LIGHT/1000)+48);
		lcd_send_data (((LIGHT%1000)/100)+48);
		lcd_send_data ((LIGHT%100)/10+48);
		lcd_send_data ((LIGHT%10)+48);
		lcd_send_string("Lux");
		lcd_put_cur(1, 0);
//		lcd_send_string(".");



		lcd_send_string("set: ");
		lcd_send_data ((SET_value/1000)+48);
		lcd_send_data ((SET_value%1000)/100+48);
		lcd_send_data ((SET_value%100)/10+48);
		lcd_send_data ((SET_value%10)+48);
/*		
		lcd_send_string("SET:");
		lcd_send_data ((SET_value/100)+48);
		lcd_send_data (((SET_value%100)/10)+48);
		lcd_send_data ((SET_value%10)+48);
		lcd_send_string("%");
	*/
			HAL_Delay(200);
			lcd_clear ();
	}
	
	int Position_PID(int Encoder, int Target)
{
 static float Bias, PWM_out, Integral_bias, Last_bias,increment;
 Bias = Encoder-Target;  // 计算偏差
 Integral_bias += Bias;  // 计算偏差的积分
 // 位置式PID控制器公式
 increment = Position_Kp*Bias+Position_Ki* Integral_bias+ Position_Kd*(Bias- Last_bias);
 Last_bias = Bias;   // 保存上一次偏差
	PWM_out+=increment;
	if (1<PWM_out& PWM_out< 900)
	{
 return PWM_out;    // 输出
}
else if(PWM_out<1)
{
return PWM_out=1;
}
else 
{
return PWM_out=999;
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

//	initLCD(0,0,GPIOB);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	 HAL_TIM_Base_Start(&htim1);//开启定时器
   lcd_init ();

//makeCursor(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		key();
		HAL_ADC_Start_IT(&hadc1);	
		//LIGHT=(int)(40.1902*pow(R,-0.6021));
		LIGHT=pow(2.7183,(log(R/40.1902))/-0.602);
		LIGHT=85.3*pow(LIGHT,0.4807);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
		LED_ON();
		pwm_cycle=(int)Position_PID(SET_value, LIGHT);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1000-pwm_cycle);
//		PWM_duty_ratio(a);
		LCD_SHOW();
		UR1_SEND_info();




	//	writeText(strr_buff);
  //	writeData(0x21);
  //	placeText(0,1,strr_buff);
  //	writeData(0x21);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
