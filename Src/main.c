/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "BSP\tim.h"
#include "BSP\usart.h"
#include "BSP\gpio.h"
#include "BSP\Led&Key.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
__IO uint32_t uwTick_Led_Set_Point=0;
__IO uint32_t uwTick_Key_Set_Point=0;
__IO uint32_t uwTick_Lcd_Set_Point=0;

uint8_t ucLed;
uint8_t Key_Val,Key_Down,Key_Up,Key_Old;
uint8_t Lcd_Disp_String[21];
uint8_t count=0;
uint8_t str[40];
uint8_t rx_Buffer;
uint8_t Rx_Buffer[7];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t interface_mode=0;
uint16_t PA1_F=1;
uint16_t PA7_F=1;
uint16_t PA1_Fre=999;
uint16_t PA7_Fre=999;
uint16_t PA1_P=1;
uint16_t PA7_P=1;
uint16_t PA1_Plu=0;
uint16_t PA7_Plu=0;
uint8_t Mode=0;
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
void Led_Proc(void);
void Key_Proc(void);
void Lcd_Proc(void);
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
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart1,(uint8_t *)(&rx_Buffer),1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Led_Proc();
		Key_Proc();
		Lcd_Proc();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Led_Proc(void)
{
	if((uwTick-uwTick_Led_Set_Point)<100)return;
	uwTick_Led_Set_Point=uwTick;
	if(PA1_F==PA7_F)
	{
		if(interface_mode==0)
		{
			ucLed &=0x00;
			ucLed |=0x01;
		}
		else
		{
			ucLed &=0x00;
			ucLed |=0x02;
		}
	}
	else if(PA1_F>PA7_F)
	{
		ucLed ^=0x01;
	}
	else
	{
		ucLed ^=0x02;
	}
	if(Mode==0)
	{
		ucLed &=0x03;
		ucLed |=0x04;
	}
	else
	{
		ucLed &=0x03;
		ucLed |=0x00;
	}
	Led_Disp(ucLed);
}

void Key_Proc(void)
{
	if((uwTick-uwTick_Key_Set_Point)<100)return;
	uwTick_Key_Set_Point=uwTick;
	
	Key_Val=Key_Scan();
	Key_Down=Key_Val&(Key_Old^Key_Val);
	Key_Up=~Key_Val&(Key_Old^Key_Val);
	Key_Old=Key_Val;
	
	switch(Key_Down)
	{
		case 1:
			if(interface_mode==0)
			{
				PA1_F+=1;
				PA1_Fre=((1000000.0/(PA1_F*1000))-1);
				__HAL_TIM_SET_AUTORELOAD(&htim2,PA1_Fre);
				if(PA1_F==11)
				{
					PA1_F=1;
					__HAL_TIM_SET_AUTORELOAD(&htim2,999);
				}
			}
			else
			{
				PA7_F+=1;
				PA7_Fre=((1000000.0/(PA7_F*1000))-1);
				__HAL_TIM_SET_AUTORELOAD(&htim17,PA7_Fre);
				if(PA7_F==11)
				{
					PA7_F=1;
					__HAL_TIM_SET_AUTORELOAD(&htim17,999);
				}
			}
		break;
		case 2:
			if(interface_mode==0)
			{
				PA1_P+=1;
				PA1_Plu=((PA1_Fre+1)*(PA1_P*0.1));
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,PA1_Plu);
				if(PA1_P>9)
				{
					PA1_P=1;
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,100);
				}
			}
			else
			{
				PA7_P+=1;
				PA7_Plu=((PA7_Fre+1)*(PA7_P*0.1));
				__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,PA7_Plu);
				if(PA7_P>9)
				{
					PA7_P=1;
					__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,100);
				}
			}
		break;
		case 3:
			if(Mode==0)
			{
				interface_mode^=1;
				LCD_Clear(Black);
			}
		break;
		case 4:
			Mode^=1;
		  
//			else
//			{
//				ucLed
//			}
		break;
	}
	
}

void Lcd_Proc(void)
{
	if((uwTick-uwTick_Lcd_Set_Point)<100)return;
	uwTick_Lcd_Set_Point=uwTick;
	
	if(interface_mode==0)
	{
		sprintf(Lcd_Disp_String,"        PA1");
		LCD_DisplayStringLine(Line3,Lcd_Disp_String);
		sprintf(Lcd_Disp_String,"      F:%4dHz ",(PA1_F*1000));
		LCD_DisplayStringLine(Line5,Lcd_Disp_String);
		sprintf(Lcd_Disp_String,"      D:%2d%%",(PA1_P*10));
		LCD_DisplayStringLine(Line7,Lcd_Disp_String);
	}
	else
	{
		sprintf(Lcd_Disp_String,"        PA7");
		LCD_DisplayStringLine(Line3,Lcd_Disp_String);
		sprintf(Lcd_Disp_String,"      F:%4dHz ",(PA7_F*1000));
		LCD_DisplayStringLine(Line5,Lcd_Disp_String);
		sprintf(Lcd_Disp_String,"      D:%2d%%",(PA7_P*10));
		LCD_DisplayStringLine(Line7,Lcd_Disp_String);
	}
	
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Rx_Buffer[count]=rx_Buffer;
	count++;
	if(count==1)
	{
		if(Mode==1)
		{
			if((Rx_Buffer[0]=='#')||(Rx_Buffer[0]=='@'))
			{
				if(Rx_Buffer[0]=='@')
				{
					interface_mode=0;
				}
				if(Rx_Buffer[0]=='#')
				{
					interface_mode=1;
				}
			}
			else
			{
				sprintf(str,"ERROR\r\n");
			}
		}
		else
		{
			sprintf(str,"KEY CONTROL\r\n");
		}
		HAL_UART_Transmit(&huart1,(uint8_t *)str,strlen(str),50);
		count=0;
  }
	HAL_UART_Receive_IT(&huart1,(uint8_t *)(&rx_Buffer),1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
