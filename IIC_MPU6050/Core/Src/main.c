/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
 //�ض���c�⺯��printf������USARTx���ض�����ʹ��printf����
 int fputc(int ch, FILE *f)
 {
     /* ����һ���ֽ����ݵ�����USARTx */
     HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
     return (ch);
 }
 
 void ShowHex(uint8_t *buf,uint8_t len)
{
    uint8_t i;
    printf("hex = ");
    for( i = 0; i < len; i++){
        printf(" %02X",buf[i]); //ʹ��ǰ��0����
        //printf(" %2X",buf[i]);  //ʹ��ǰ���ո���
        //printf(" %X",buf[i]);  //�����̵�16���Ƹ�ʽ
    }
    printf( "\r\n");
}




void MPU6050_INIT()
{
    int i = 0, j = 0;
    
    //��ʱ
    HAL_Delay(100);
    uint8_t SendAddress = 0x6b;
    uint8_t SendData  = 0x00; //�������״̬
    HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
    
    SendAddress = 0x19;//�����ʷ�Ƶ��
    SendData = 0x07;
    HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
    
    SendAddress = 0x1A;//��ͨ�˲���
    SendData = 0x06;
    HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
    
    SendAddress = 0x1B;//������
    SendData = 0x08;   //�� 500 ��/s
    HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
    
    SendAddress = 0x1C;//���ٶȼ�
    SendData = 0x00;   //�� 2g
    HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
    
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    uint8_t MPU_Data[] = {0X00, 0X00,0X00, 0X00,0X00, 0X00,0X00, 0X00,0X00, 0X00,0X00, 0X00,0X00, 0X00};
    uint8_t preg1_Data  = 0x3B;
    double ACCEL_XOUT = 0.0,ACCEL_YOUT =0.0,ACCEL_ZOUT=0.0;
    double GYRO_XOUT = 0.0,GYRO_YOUT =0.0,GYRO_ZOUT=0.0;
    
    
    
    MPU6050_INIT();
    
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      
      HAL_I2C_Mem_Read(&hi2c1, 0xD1,preg1_Data,I2C_MEMADD_SIZE_8BIT,MPU_Data,14,50);
      //ShowHex(MPU_Data,14);
      float Temp = (MPU_Data[6]<<8)|MPU_Data[7];
      if(Temp>32768) Temp-=65536;
      Temp=(36.53+Temp/340);
      
      short int ACCEL_XOUT1 = ((MPU_Data[0]<<8)|MPU_Data[1]);ACCEL_XOUT = (double)ACCEL_XOUT1/16384;
      short int ACCEL_YOUT1 = ((MPU_Data[2]<<8)|MPU_Data[3]);ACCEL_YOUT = (double)ACCEL_YOUT1/16384;
      short int ACCEL_ZOUT1 = ((MPU_Data[4]<<8)|MPU_Data[5]);ACCEL_ZOUT = (double)ACCEL_ZOUT1/16384;
      
      
      short int GYRO_XOUT1 = ((MPU_Data[8]<<8)|MPU_Data[9]);GYRO_XOUT = (double)GYRO_XOUT1/65.5;
      short int GYRO_YOUT1 = ((MPU_Data[10]<<8)|MPU_Data[11]);GYRO_YOUT = (double)GYRO_YOUT1/65.5;
      short int GYRO_ZOUT1 = ((MPU_Data[12]<<8)|MPU_Data[13]);GYRO_ZOUT = (double)GYRO_ZOUT1/65.5;
      
      //printf("temp = %0.2f",Temp);
      printf("x= %0.2f,y= %0.2f,z= %0.2f",GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT);//���ٶȼ�
      //printf("x= %0.2f,y= %0.2f,z= %0.2f",ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT);
      printf("\r\n");
      HAL_Delay(200);
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
