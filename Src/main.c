/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gps.h"
#include "bmi160.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_UART 	huart1	//UART for GPS
#define DEVICE_UART 	huart3	//UART for message receiving device
#define ACCEL_I2C 	hi2c1	//I2C for accelerometer

//CAN base address
#define CAN_ADDRESS (0x1A0)	//CAN base address
#define CAN_MSG_ACCEL_ID (0x1)	//CAN accel message id
#define CAN_MSG_GYRO_ID (0x2)	//CAN gyro message id
#define CAN_MSG_GPS1_ID (0x3)	//CAN gps1 message id
#define CAN_MSG_GPS2_ID (0x4)	//CAN gps2 message id

#define ACCEL_ADDRESS (0x68<<1)	//Accelerometer I2C address

//status LEDs
#define GPS_LED GPIOB,GPIO_PIN_1	//GPS status LED pin
#define ACCEL_LED GPIOB,GPIO_PIN_2	//Accelerometer status LED pin

#define FLAG_BMI160_STATUS	0x0080

#define TIM1_INT		0x0080

#define SET_FLAG(flag)		(statusFlags |= flag)
#define RESET_FLAG(flag)	(statusFlags &= ~flag)
#define READ_FLAG(flag)		(statusFlags & flag)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t timeoutCounter=0;
uint8_t timeoutCycles = 5;
uint8_t timeoutCounterUntilSelfTest=0;
uint8_t timeoutCyclesUntilSelfTest = 10;

uint64_t statusFlags = 0x0000;

struct bmi160_dev sensor;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;

uint32_t txMailbox;		// CAN Tx-Mailbox
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

__IO ITStatus GPSUartReady = SET;
/*
Bits

-- GPS flags --
00: Status		0: no connection	1: connected
01: data Status		0: data invalid		1: data valid
02: message receive	0: no reception		1: receive ongoing
03: message pending	0: buffer empty		1: message in buffer
04:
05:
06:
07:

-- BMI160 flags --
08: Status
09: CAN message pending
10:
11:
12:
13:
14:
15:

-- UART flags --
16:
17:
18:
19:

-- CAN flags --
14: CAN message pending
15:
16:
17:

-- general flags --
18: timer interrupt
19:
1A:
1B:
1C:
1D:
1E:
1F:
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DeviceStatus(void); //set status LEDs
void SendMsgToUart(char[]);
uint32_t SendCANMsg(uint8_t*, uint8_t, uint8_t); //set CAN header and send data to CAN

void CAN_Config(void); //configure CAN and CAN filter

void InitBMI160();
void BMI160SelfTest();
void ReadAndSendAccelData();
void SendGPSData();

// Methods passed to bosch driver methods
int8_t bmi_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void bmi_delay_i2c(uint32_t period);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  SendMsgToUart("\n\n****************\n\rSoftware starting up\n\r****************\n");

  //initialize BMI160
  InitBMI160();
  BMI160SelfTest();

  // Enable the Analog I2C Filter
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1,I2C_ANALOGFILTER_ENABLE);

  //configure CAN and CAN filter
  CAN_Config();

  //start CAN communication
  if(HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();

  //activate CAN message interrupt --> interrupts are currently deactivated through cubeMx as no CAN messages need to be received at this point in time!
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  // Initialize empty message buffer for gps messages
  InitMsgBuffer();

  HAL_GPIO_WritePin(ACCEL_LED,1);
  HAL_GPIO_WritePin(GPS_LED,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive_IT(&GPS_UART, (uint8_t*)&gps_CharBuffer, 1);

      if(READ_FLAG(TIM1_INT))
      {
    	  DeviceStatus();
    	  RESET_FLAG(TIM1_INT);	//reset status flag
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * Check device status
 */
void DeviceStatus(void)
{

  //set acceleration module status
  if(READ_FLAG(FLAG_BMI160_STATUS))
  {
      HAL_GPIO_WritePin(ACCEL_LED,0);
  }
  else
  {
      HAL_GPIO_WritePin(ACCEL_LED,1);
  }

  //set GPS status
  if(timeoutCounter >=  timeoutCycles)
  {
    HAL_GPIO_WritePin(GPS_LED,1);
    RESET_FLAG(FLAG_GPS_STATUS);
  }
  else
  {
    char* gpsStat = GetGpsData(gps_MsgBuffer,GPS_RMC_CHECK);
    gpsStat = GetGpsData(gps_MsgBuffer,GPS_RMC_STATUS);
    if(*gpsStat == 'A')
    {
      HAL_GPIO_WritePin(GPS_LED,0);
      SET_FLAG(FLAG_GPS_DATA);	//set status 'TRUE'
    }
    else if(*gpsStat == 'V')
    {
      HAL_GPIO_TogglePin(GPS_LED);
      RESET_FLAG(FLAG_GPS_DATA);	//set status 'FALSE'
    }
    else
    {
      HAL_GPIO_WritePin(GPS_LED,1);
      RESET_FLAG(FLAG_GPS_DATA);	//set status 'FALSE'
    }
  }
}

/**
 * Create and send CAN message
 */
uint32_t SendCANMsg(uint8_t* dataAddress, uint8_t id, uint8_t messageSize)
{
  //create CAN TxHeader
  CAN_TxHeaderTypeDef txHeader;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.StdId = CAN_ADDRESS + id;
  txHeader.DLC = messageSize;
  txHeader.TransmitGlobalTime = DISABLE;

  //wait while message is pending
  while(HAL_CAN_IsTxMessagePending(&hcan, txMailbox)==1)
  {
  }

  //send data to CAN
  if(HAL_CAN_AddTxMessage(&hcan, &txHeader, dataAddress, &txMailbox)!=HAL_OK)
	  return(HAL_CAN_GetError(&hcan));
  else
  {
	  return(HAL_CAN_ERROR_NONE);
  }

}

/**
 * UART interrupt handler.
 * Builds gps message after GPS-UART buffer is empty.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &GPS_UART)
  {
	GPSUartReady = RESET;
    if(gps_CharBuffer == '$')
    {
      SET_FLAG(FLAG_GPS_MSG_RECEIVE);

      //clear message array
      memset(gps_ReceiveMsg,'\0',strlen(gps_ReceiveMsg)*sizeof(char));
    }
    else if(gps_CharBuffer == '\r' || gps_CharBuffer == '\n' || gps_CharBuffer == EOF)
    {
      RESET_FLAG(FLAG_GPS_MSG_RECEIVE);
    }

    if(READ_FLAG(FLAG_GPS_MSG_RECEIVE))
    {
      strcat(gps_ReceiveMsg, &gps_CharBuffer);

      HAL_UART_Receive_IT(&GPS_UART, (uint8_t*)&gps_CharBuffer, 1);
    }
    else
    {
      timeoutCounter = 0;

      if(READ_FLAG(FLAG_GPS_STATUS))
      {
    	  //HAL_UART_Transmit(&DEVICE_UART, gps_ReceiveMsg, strlen(gps_ReceiveMsg),100);
    	  SortGpsData(gps_MsgBuffer,gps_ReceiveMsg);
      }
      else
    	  SET_FLAG(FLAG_GPS_STATUS);
    }

    GPSUartReady = SET;
  }
}

/**
 * UART error handler
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t error = HAL_UART_GetError(&GPS_UART);

  if(error != HAL_UART_ERROR_NONE && huart == &GPS_UART)
  {
	  // Most likely overrun error, if implementation doesn't change then error handling should be applied here
  }
}

/**
 * Configures CAN filter
 */
void CAN_Config(void)
{
  CAN_FilterTypeDef  CanFilter;

  CanFilter.FilterBank = 0;
  CanFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  CanFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  CanFilter.FilterIdHigh = 0x0000;
  CanFilter.FilterIdLow = 0x0000;
  CanFilter.FilterMaskIdHigh = 0x0000;
  CanFilter.FilterMaskIdLow = 0x0000;
  CanFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  CanFilter.FilterActivation = ENABLE;
  CanFilter.SlaveStartFilterBank = 14;

  //activate CAN filter
  if (HAL_CAN_ConfigFilter(&hcan, &CanFilter) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * CAN rx interrupt callback.
 * Just stores the received data. Currently no functionality to receive messages is implemented.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
	/* Reception Error */
	Error_Handler();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_IT_PIN)
{
}

/**
 * Timer interrupt callback.
 * Send CAN messages containing the measured data.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM1) {
		  if(timeoutCounter < timeoutCycles)
		  {
		      timeoutCounter++;
		  }

		  SET_FLAG(TIM1_INT);	//set TIM1_INT status flag
	}
	if (htim->Instance == TIM2)
	{
		SendGPSData();
	}
	if (htim->Instance == TIM3)
	{
		ReadAndSendAccelData();
	}
}


/**
 * Send current GPS data to CAN
 */
void SendGPSData()
{
	uint16_t lat[3] = { 0 };
	getLatOrLon(RESET, &lat[0]);

	uint16_t lon[3] = { 0 };
	getLatOrLon(SET, &lon[0]);

	uint8_t gpsMsg1[8] = { 0 };
	gpsMsg1[0] = (uint8_t) lat[0];
	gpsMsg1[1] = (uint8_t) lat[1];
	gpsMsg1[2] = (lat[2] & 0x00FF);
	gpsMsg1[3] = (lat[2] & 0xFF00) >> 8;
	gpsMsg1[4] = (uint8_t) lon[0];
	gpsMsg1[5] = (uint8_t) lon[1];
	gpsMsg1[6] = (lon[2] & 0x00FF);
	gpsMsg1[7] = (lon[2] & 0xFF00) >> 8;

	char* status = GetGpsData(gps_MsgBuffer,GPS_RMC_STATUS);
	if (status[0] == 'A')
		gpsMsg1[0] = gpsMsg1[0] | 0x80; // set MSB to one if status = A

	char* nsIndicator = GetGpsData(gps_MsgBuffer,GPS_GGA_NSINDICATOR);
	if (nsIndicator[0] == 'N')
		gpsMsg1[1] = gpsMsg1[1] | 0x80; // set MSB to one if north

	char* ewIndicator = GetGpsData(gps_MsgBuffer,GPS_GGA_EWINDICATOR);
	if (ewIndicator[0] == 'E')
		gpsMsg1[5] = gpsMsg1[5] | 0x80; // set MSB to one if east


	uint32_t speed = getSpeed();
	int32_t altitude = getAltitude();
	uint8_t satellites = getSatellites();

	uint8_t gpsMsg2[8] = { 0 };
	gpsMsg2[0] = (speed & 0x000000FF);
	gpsMsg2[1] = (speed & 0x0000FF00) >> 8;
	gpsMsg2[2] = (speed & 0x00FF0000) >> 16;
	gpsMsg2[3] = (speed & 0xFF000000) >> 24;
	gpsMsg2[4] = (altitude & 0x000000FF);
	gpsMsg2[5] = (altitude & 0x0000FF00) >> 8;
	gpsMsg2[6] = (altitude & 0x00FF0000) >> 16;
	gpsMsg2[7] = (altitude & 0xFF000000) >> 24;

	gpsMsg2[3] = gpsMsg2[3] | (satellites << 4);


	SendCANMsg(&gpsMsg1[0], CAN_MSG_GPS1_ID, 8);
	SendCANMsg(&gpsMsg2[0], CAN_MSG_GPS2_ID, 8);
}

/**
 * Initializes and configures the BMI160
 * When accel or gyro range is changed you have to change factor in dbc files as you will receive differently calculated values.
 */
void InitBMI160()
{
	/* Basic Config */
	sensor.id = BMI160_I2C_ADDR;
	sensor.interface = BMI160_I2C_INTF;
	sensor.read = &bmi_read_i2c;
	sensor.write = &bmi_write_i2c;
	sensor.delay_ms = &bmi_delay_i2c;

	int8_t initResult = bmi160_init(&sensor);

	/* Select the Output data rate, range of accelerometer sensor */
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

	/* Select the power mode of accelerometer sensor */
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	/* Select the Output data rate, range of Gyroscope sensor */
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

	/* Select the power mode of Gyroscope sensor */
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/* Set the sensor configuration */
	int8_t sensorConfigResult = bmi160_set_sens_conf(&sensor);
	char initMsg[100];
	sprintf(initMsg, "\rInit BMI160: \n\rInitRslt: %d \n\rConfRslt: %d \n",initResult, sensorConfigResult );

	SendMsgToUart(initMsg);
}

/**
 * Self tests the BMI160
 * bmi160_init must have been called once before self test.
 * Self test resets config, that's why InitBMI160 is called again in this function.
 *
 */
void BMI160SelfTest() {
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
	bmi160_set_sens_conf(&sensor);

  	int8_t selfTestAccel = bmi160_perform_self_test(BMI160_ACCEL_ONLY, &sensor);

  	InitBMI160();

  	int8_t selfTestGyro = bmi160_perform_self_test(BMI160_GYRO_ONLY, &sensor);

  	char selfTestMsg[50];
  	sprintf(selfTestMsg, "\r\n\rSelfTestResult - Accel: %d Gyro: %d \n",selfTestAccel, selfTestGyro);
  	SendMsgToUart(selfTestMsg);

  	if (selfTestAccel == BMI160_OK && selfTestGyro == BMI160_OK) {
  		SET_FLAG(FLAG_BMI160_STATUS);
  	} else {
  		RESET_FLAG(FLAG_BMI160_STATUS);
  	}

  	InitBMI160();
}

void SendMsgToUart(char msgText[]) {
	  char msgOutUART[100];	//message for UART
	  //create UART message
	  memset(msgOutUART,'\0',sizeof(msgOutUART));
	  strcat(msgOutUART,msgText);
	  strcat(msgOutUART,"\r\n");
	  //send UART message
	  if (HAL_UART_Transmit(&DEVICE_UART,(uint8_t*)&msgOutUART,(uint16_t)sizeof(msgOutUART),1000) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  memset(msgOutUART,'\0',sizeof(msgOutUART));
}

/**
 * Reads and sends current accel and gyro data to CAN bus
 */
void ReadAndSendAccelData()
{
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);

    int16_t accelMsg[] = { accel.x, accel.y, accel.z };
    SendCANMsg((uint8_t*)&accelMsg[0], CAN_MSG_ACCEL_ID, 6);

    int16_t gyroMsg[] = { gyro.x, gyro.y, gyro.z };
    SendCANMsg((uint8_t*)&gyroMsg[0], CAN_MSG_GYRO_ID, 6);
}

/**
 * Read function to configure bmi160
 */
int8_t bmi_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int checkvar;
	int shifted_adress = dev_addr << 1;
	checkvar = HAL_I2C_Mem_Read(&ACCEL_I2C, (uint16_t) shifted_adress, (uint16_t) reg_addr, sizeof(reg_addr), data, len, 100);

	return checkvar;
}

/**
 * Write function to configure bmi160;
 */
int8_t bmi_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{

	int shifted_adress = dev_addr << 1;
	int checkvar = HAL_I2C_Mem_Write(&ACCEL_I2C, (uint16_t) shifted_adress, (uint16_t) reg_addr, sizeof(reg_addr), data, len, 100);

	return checkvar;
}

/**
 * Delay function to configure bmi160
 */
void bmi_delay_i2c(uint32_t period)
{
	HAL_Delay(period);
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
