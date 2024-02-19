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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M0_VPWM TIM2->CCR1
#define M1_VPWM TIM2->CCR2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buff_SRF_serial[23];
uint8_t uart2_terima;
uint8_t uart2_status;

float distanceLeft, distanceRight, distanceAll =0;
float SRF_Distance[5];

uint8_t buff_gyro_kirim;
uint8_t buff_gyro_terima[8];
float GyroData;

//uint16_t Encoder[2];
int8_t Encoder[2];
int pulseRight, pulseLeft, pulseAll = 0;
int16_t MotorPWM[2];
int lastTimeTick = 0;

uint8_t tombol[4];

// line sensor init
uint8_t linesensor[8];
uint8_t pvlineSensor=0; //present value of line sensor


//PID init
int8_t P=0;
int8_t I=0;
int8_t D=0;
int8_t PD=0;
int8_t PID=0;
int8_t lastError=0;
uint8_t Kp=10;
uint8_t Ki=0;
uint8_t Kd=80;
int8_t error=0;

Lcd_PortType ports[] = {lcd_db4_GPIO_Port,lcd_db5_GPIO_Port,lcd_db6_GPIO_Port,lcd_db7_GPIO_Port};
Lcd_PinType pins[] = {lcd_db4_Pin, lcd_db5_Pin,lcd_db6_Pin,lcd_db7_Pin};

Lcd_HandleTypeDef lcd;

char buff_lcd[16];

uint8_t PC_Terima[24];
uint8_t status_serialPC;
uint8_t buff_PCSerial;

uint8_t Buzz;
uint8_t taskNumber;
uint16_t radoftrack;
uint16_t numofpict;
int steps=0;
int16_t speedTerima;

uint8_t PC_Kirim[128];
uint8_t TombolKirim = 0xff;
uint8_t captState=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MotorSetPWM(int16_t pwm[2]);
void Reset_Enc1();
void Reset_Enc2();
void Reset_Enc();
int8_t errorMapping(uint8_t sensorState);
void calibrateLineSensor();
void runCapture();



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
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, &uart2_terima, 1);
  HAL_UART_Receive_DMA(&huart4, buff_gyro_terima,8);
  HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
//  HAL_UART_Receive_DMA(&huart1, PC_Terima, 8);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim6);


  for(int i = 0;i<20;i++)
  {
	  HAL_GPIO_TogglePin(buzzer_GPIO_Port, buzzer_Pin);
	  HAL_Delay(20);
  }
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  lcd = Lcd_create(ports, pins, lcd_rs_GPIO_Port, lcd_rs_Pin, lcd_e_GPIO_Port, lcd_e_Pin, LCD_4_BIT_MODE);

  Lcd_string(&lcd, "--Mobile Robot--");
  Reset_Enc();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(taskNumber == 1)
	  {
		  calibrateLineSensor();
	  }
	  if(taskNumber == 2)
	  {
		  runCapture();
	  }

	  /*
	  tombol[0] = HAL_GPIO_ReadPin(s0_GPIO_Port, s0_Pin);
	  tombol[1] = HAL_GPIO_ReadPin(s1_GPIO_Port, s1_Pin);
	  tombol[2] = HAL_GPIO_ReadPin(s2_GPIO_Port, s2_Pin);
	  tombol[3] = HAL_GPIO_ReadPin(s3_GPIO_Port, s3_Pin);

	  TombolKirim = tombol[0] | tombol[1]<<1 | tombol[2]<<2 | tombol[3]<<3;
	   */

	  sprintf(buff_lcd,"L:%.2f R:%.2f",distanceLeft,distanceRight);
	  Lcd_cursor(&lcd, 1, 0);
	  Lcd_string(&lcd, buff_lcd);


	  //sprintf(PC_Kirim,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d\n",SRF_Distance[0],SRF_Distance[1],SRF_Distance[2],
	  //			SRF_Distance[3],SRF_Distance[4],GyroData,Encoder[0],Encoder[1],tombol[0],tombol[1],tombol[2],tombol[3]);


	  //HAL_UART_Transmit(&huart1, PC_Kirim, strlen(PC_Kirim),100);
	  //HAL_Delay(10);



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{

		if(uart2_status == 0 && uart2_terima == 'm')
		{
			uart2_status = 1;
			HAL_UART_Receive_DMA(&huart2, &uart2_terima, 1);
		}
		else if(uart2_status == 1 && uart2_terima == 'r')
		{
			uart2_status =2;
			HAL_UART_Receive_DMA(&huart2, &uart2_terima, 1);
		}
		else if(uart2_status == 2 && uart2_terima == 'i')
		{
			uart2_status=3;
			HAL_UART_Receive_DMA(&huart2, buff_SRF_serial+3, 20);
		}
		else if(uart2_status == 3)
		{
			uart2_status = 0;
			memcpy(&SRF_Distance,buff_SRF_serial+3,20);

			HAL_UART_Receive_DMA(&huart2, &uart2_terima, 1);


		}
		else
		{
			uart2_status = 0;
			HAL_UART_Receive_DMA(&huart2, &uart2_terima, 1);
		}



	}
	else if(huart->Instance == UART4)
	{

		if(buff_gyro_terima[0] == 0xAA && buff_gyro_terima[7]==0x55)
		{
			volatile int16_t gyroRaw = buff_gyro_terima[1]<<8 | buff_gyro_terima[2];
			GyroData = gyroRaw * 0.01;
		}

		HAL_UART_Receive_DMA(&huart4, buff_gyro_terima,8);
	}
	else if(huart->Instance == USART1)
	{
		/*
		if(status_serialPC==0 && buff_PCSerial == 'm')
		{
			status_serialPC = 1;
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		else if(status_serialPC == 1 && buff_PCSerial == 'r')
		{
			status_serialPC = 2;
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		else if(status_serialPC == 2 && buff_PCSerial == 'i')
		{
			status_serialPC = 3;
			HAL_UART_Receive_DMA(&huart1, PC_Terima,5);
		}
		else if(status_serialPC == 3)
		{
			status_serialPC = 0;
			memcpy(MotorPWM,PC_Terima,4);
			memcpy(&Buzz,PC_Terima+4,1);
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		else
		{	status_serialPC = 0;
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		*/
		// command

		if(status_serialPC==0 && buff_PCSerial == '3')
		{
			status_serialPC = 1;
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		else if(status_serialPC == 1 && buff_PCSerial == 'd')
		{
			status_serialPC = 2;
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		else if(status_serialPC == 2 && buff_PCSerial == 'p')
		{
			status_serialPC = 3;
			HAL_UART_Receive_DMA(&huart1, PC_Terima,12);
		}
		else if(status_serialPC == 3)
		{
			status_serialPC = 0;
			memcpy(&taskNumber,PC_Terima,1);
			memcpy(&numofpict,PC_Terima+1,2);
			memcpy(&radoftrack,PC_Terima+3,2);
			memcpy(MotorPWM,PC_Terima+5,4);
			memcpy(&Kp,PC_Terima+9,1);
			memcpy(&Ki,PC_Terima+10,1);
			memcpy(&Kd,PC_Terima+11,1);
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}
		else
		{	status_serialPC = 0;
			HAL_UART_Receive_DMA(&huart1, &buff_PCSerial, 1);
		}

		HAL_UART_Receive_DMA(&huart1, PC_Terima, 15);

	}


}
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(HAL_GetTick()-lastTimeTick>20){
		sprintf(PC_Kirim,"%.2f,%.2f,%d\n",distanceAll, GyroData,captState);
		HAL_UART_Transmit_IT(&huart1, PC_Kirim, strlen(PC_Kirim));
		lastTimeTick=HAL_GetTick();
	}
}
*/
void MotorSetPWM(int16_t pwm[2])
{

	if(pwm[0]>0)
	{
		HAL_GPIO_WritePin(M0_D1_GPIO_Port, M0_D1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M0_D2_GPIO_Port, M0_D2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(M0_D1_GPIO_Port, M0_D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M0_D2_GPIO_Port, M0_D2_Pin, GPIO_PIN_RESET);
	}

	if(pwm[1]<0)
	{
		HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, GPIO_PIN_RESET);
	}


	M0_VPWM = abs(pwm[0]);
	M1_VPWM = abs(pwm[1]);

}

void Reset_Enc1(){
	Encoder[0]=0;
	TIM3->CNT=65535;
	TIM3->CNT=0;
	distanceLeft=0;
	pulseLeft=0;
}
void Reset_Enc2(){
	Encoder[1]=0;
	TIM1->CNT=65535;
	TIM1->CNT=0;
	distanceRight=0;
	pulseRight=0;
}

void Reset_Enc(){
	Reset_Enc1();
	Reset_Enc2();
	distanceAll=0;
	pulseAll=0;
}


int8_t errorMapping(uint8_t sensorState)
{
	int8_t errorState;
	if(sensorState == 0b10000000){errorState = 7;}
	if(sensorState == 0b11000000){errorState = 6;}
	if(sensorState == 0b11100000){errorState = 6;}
	if(sensorState == 0b01000000){errorState = 5;}
	if(sensorState == 0b01100000){errorState = 4;}
	if(sensorState == 0b01110000){errorState = 4;}
	if(sensorState == 0b00100000){errorState = 3;}
	if(sensorState == 0b00110000){errorState = 2;}
	if(sensorState == 0b00111000){errorState = 2;}
	if(sensorState == 0b00010000){errorState = 1;}
	if(sensorState == 0b00011000){errorState = 0;}
	if(sensorState == 0b00001000){errorState = -1;}
	if(sensorState == 0b00011100){errorState = -2;}
	if(sensorState == 0b00001100){errorState = -2;}
	if(sensorState == 0b00000100){errorState = -3;}
	if(sensorState == 0b00001110){errorState = -4;}
	if(sensorState == 0b00000110){errorState = -4;}
	if(sensorState == 0b00000010){errorState = -5;}
	if(sensorState == 0b00000111){errorState = -6;}
	if(sensorState == 0b00000011){errorState = -6;}
	if(sensorState == 0b00000001){errorState = -7;}
	return errorState;
}

void calibrateLineSensor()
{
	MotorPWM[0]=0; MotorPWM[1]=0;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);

	MotorPWM[0]=100; MotorPWM[1]=-100;
	HAL_Delay(500);
	MotorPWM[0]=0; MotorPWM[1]=0;
	HAL_Delay(500);
	MotorPWM[0]=-100; MotorPWM[1]=100;
	HAL_Delay(1000);
	MotorPWM[0]=0; MotorPWM[1]=0;
	HAL_Delay(500);
	MotorPWM[0]=100; MotorPWM[1]=-100;
	HAL_Delay(500);
	MotorPWM[0]=0; MotorPWM[1]=0;
	HAL_Delay(500);

	taskNumber=0;
}

void runCapture()
{
	  Reset_Enc();
	  steps=0;
	  int timeSampling = 1;
	  uint16_t pwm0 = MotorPWM[0];
	  uint16_t pwm1 = MotorPWM[1];
	  while(steps<numofpict){
		  while(1){
			  linesensor[0] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
			  linesensor[1] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
			  linesensor[2] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
			  linesensor[3] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);
			  linesensor[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
			  linesensor[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
			  linesensor[6] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);
			  linesensor[7] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);

			  pvlineSensor = linesensor[0] | linesensor[1]<<1 | linesensor[2]<<2 | linesensor[3]<<3 | linesensor[4]<<4 | linesensor[5]<<5 | linesensor[6]<<6 | linesensor[7]<<7;

			  error   =   errorMapping(pvlineSensor);
			  P       =   Kp*error;
			  I		  =   Ki*(error+lastError);
			  D       =   Kd*(error-lastError);//(Kd/timeSampling)*(error-lastError);
			  //PD      =   P+D;
			  PID	  = P+I+D;
			  MotorPWM[0]=pwm0-PID;//100-PID;
			  MotorPWM[1]=pwm1+PID;//100+PID;


			  lastError=error;
			  timeSampling=HAL_GetTick()-timeSampling;
			  distanceAll=(distanceLeft+distanceRight)/2;

			  if(HAL_GetTick()-lastTimeTick>30){
			  		//sprintf(PC_Kirim,"3dp,%.2f,%.2f,%d,%d\n",distanceAll, GyroData,captState,error);
				    sprintf(PC_Kirim,"p,%.2f,%.2f,%d,d\n",distanceAll,GyroData,error);
			  		HAL_UART_Transmit_IT(&huart1, PC_Kirim, strlen(PC_Kirim));
			  		lastTimeTick=HAL_GetTick();
			  	}

			  if(taskNumber == 0){
				  break;
			  }
			  if((distanceAll/(steps+1))>=(2*3.14*radoftrack)/numofpict){
				  MotorPWM[0]=0; MotorPWM[1]=0;
				  break;
			  }
			  //sprintf(PC_Kirim,"%.2f,%.2f,%d\n",distanceAll, GyroData,captState);
			  //HAL_UART_Transmit_IT(&huart1, PC_Kirim, strlen(PC_Kirim));
			  //HAL_UART_Transmit_DMA(&huart1, PC_Kirim, strlen(PC_Kirim));
		  }
		  if(taskNumber == 0){
		  	 break;
		  }
		  MotorPWM[0]=0; MotorPWM[1]=0;
		  HAL_Delay(500);
		  captState = 1;
		  //sprintf(PC_Kirim,"%.2f,%.2f,%d\n",distanceAll, GyroData,captState);
		  //HAL_UART_Transmit(&huart1, PC_Kirim, strlen(PC_Kirim),100);
		  HAL_UART_Transmit(&huart1, "c", strlen("c"),100);
		  //sprintf(PC_Kirim,"3dp,%.2f,%.2f,%d,%d\n",distanceAll, GyroData,captState,error);
		  sprintf(PC_Kirim,"p,%.2f,%d,%d\n",distanceAll, error,captState);
		  //HAL_UART_Transmit(&huart1, PC_Kirim, strlen(PC_Kirim),10);
		  //HAL_UART_Transmit_IT(&huart1, PC_Kirim, strlen(PC_Kirim));//,100);
		  /*
		  for(int n=0; n<3; n++){
			  HAL_UART_Transmit(&huart1, PC_Kirim, strlen(PC_Kirim),100);
		  }
			*/

		  //HAL_UART_Transmit_IT(&huart1, PC_Kirim, strlen(PC_Kirim));
		  //HAL_Delay(8000);
		  captState = 0;
		  //while(taskNumber != 4){}
		  HAL_Delay(6000);
		  //taskNumber=2;
		  steps=steps+1;
	  }
	  taskNumber=0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		HAL_GPIO_TogglePin(brd_led_GPIO_Port, brd_led_Pin);

		if(MotorPWM[0]>1000)
			MotorPWM[0] = 1000;
		else if(MotorPWM[0]<-1000)
			MotorPWM[0] = -1000;


		if(MotorPWM[1]>1000)
			MotorPWM[1] = 1000;
		else if(MotorPWM[1]<-1000)
			MotorPWM[1] = -1000;

		MotorSetPWM(MotorPWM);

		Encoder[0]=TIM3->CNT - 65535;
		TIM3->CNT=65535;
		pulseLeft+=(float)(Encoder[0]);  //* Clicks1;
		distanceLeft = ((float)(pulseLeft-1)/28000)*3.14*150;
		Encoder[0]=0;

		Encoder[1]=TIM1->CNT - 65535;
		TIM1->CNT=65535;
		pulseRight+=(float)(Encoder[1]); //* Clicks1;
		distanceRight = ((float)(pulseRight-1)/28000)*3.14*150;
		Encoder[1]=0;

		//Encoder[0] = __HAL_TIM_GET_COUNTER(&htim3);
		//Encoder[1] = __HAL_TIM_GET_COUNTER(&htim1);
		/*
		if(Buzz==1)
		{
			__HAL_TIM_SET_COUNTER(&htim3,0);
			__HAL_TIM_SET_COUNTER(&htim1,0);
			Buzz=0;
		}
		 */


		//if(Buzz>1)
		//	Buzz = 1;

		//HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, Buzz);





	}
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
