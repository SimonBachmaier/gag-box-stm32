//#include "stdio.h"
#include "i2c.h"
#include "bmi160_reg.h"

uint8_t bmi160FifoBuffer[1024];		//buffer for acceleration module data

#ifndef ACCEL_I2C
#define ACCEL_I2C hi2c1
#endif

#ifndef ACCEL_ADDRESS
#define ACCEL_ADDRESS (0x68<<1)
#endif
//I2C_HandleTypeDef *ACCEL_I2C = &hi2c1;
//uint16_t ACCEL_ADDRESS = (0x68<<1);

enum BMI160_CMD
{//see BMI160 data sheet
  START_FOC	=0x03,

  ACC_SUS	=0x10,
  ACC_NORMAL	=0x11,
  ACC_LOW_POWER	=0x12,

  GYR_SUS	=0x14,
  GYR_NORMAL	=0x15,
  GYR_FAST	=0x17,

  MAG_SUS	=0x18,
  MAG_NORMAL	=0x19,
  MAG_LOW_POWER	=0x1A,

  PROG_NVM	=0xA0,

  FIFO_FLUSH	=0xB0,

  INT_RESET	=0xB1,

  SOFTRESET	=0xB6,

  STEP_CNT_CLR	=0xB2
};


//read register value | BMI160 - I2C
uint8_t BMI160_Read(uint8_t reg)
{
  uint8_t data = 0;

  HAL_I2C_Master_Transmit(&ACCEL_I2C, ACCEL_ADDRESS,&reg, 1, 1000);
  HAL_I2C_Master_Receive(&ACCEL_I2C, ACCEL_ADDRESS,&data, 1, 1000);

  return(data);
}

//write register value | BMI160 - I2C
void BMI160_Write(uint8_t reg, uint8_t data)
{
  uint8_t initData[2] = {reg, data};
  HAL_I2C_Master_Transmit(&ACCEL_I2C, ACCEL_ADDRESS,initData, 2, 1000);
}

uint8_t BMI160_SelfTest(void)
{
  char selfTest[30];
  BMI160_Write(BMI160_REG_SELF_TEST,0b00001101);
  //sprintf(selfTest,"self test:\t%#02x\n\r",BMI160_Read(BMI160_REG_STATUS));
  //HAL_UART_Transmit(&huart, (uint8_t*)selfTest, 15, 500);

  char status[30];
  //sprintf(status,"status:\t%#02x\n\r",BMI160_Read(BMI160_REG_PMU_STATUS));
  //HAL_UART_Transmit(&huart, (uint8_t*)status, 15, 500);

  BMI160_Write(BMI160_REG_SELF_TEST,0b00010001);	//start gyroscope and acceleration self test
  HAL_Delay(50);

  return(BMI160_Read(BMI160_REG_PMU_STATUS));
}

//initialize BMI160 - I2C
void BMI160_Init()
{
  BMI160_Write(BMI160_REG_CMD, SOFTRESET);	//reset and reboot BMI160
  HAL_Delay(50);
  BMI160_Write(BMI160_REG_NV_CONF,0b00000110);	//enable i2c

  BMI160_Write(BMI160_REG_CMD,ACC_NORMAL);	//set accelerometer normal power mode
  HAL_Delay(50);
  BMI160_Write(BMI160_REG_CMD,GYR_NORMAL);	//set gyroscope normal power mode
  HAL_Delay(50);
  BMI160_Write(BMI160_REG_CMD,MAG_NORMAL);	//set magnetometer normal power mode
  HAL_Delay(50);

  BMI160_Write(BMI160_REG_GYR_RANGE,0b00000010);//±500°/s

  BMI160_Write(BMI160_REG_MAG_CONF,0b00001011);	//magnetometer 800Hz
  BMI160_Write(BMI160_REG_ACC_CONF,0b00101100);	//accelerometer 1600Hz
  BMI160_Write(BMI160_REG_GYR_CONF,0b00101100);	//gyroscope 1600Hz

  BMI160_Write(BMI160_REG_FIFO_CONFIG_0,0x80);	//128 FIFO entries

  BMI160_Write(BMI160_REG_FIFO_CONFIG_1,0b11100000);	//enable FIFO for: gyroscope, accelerometer, magnetometer
}



//get data block from acceleration module FIFO
void BMI160_getFifoData(uint8_t *fifoDataPointer)
{
  uint16_t fifoLength=0;
  memset(fifoDataPointer,'\0',1024);

  fifoLength+=BMI160_Read(BMI160_REG_FIFO_LENGTH_0);
  fifoLength+=(BMI160_Read(BMI160_REG_FIFO_LENGTH_1)&0x07);

  HAL_I2C_Master_Transmit(&ACCEL_I2C, ACCEL_ADDRESS,BMI160_REG_FIFO_DATA, 1, 1000);
  HAL_I2C_Master_Receive(&ACCEL_I2C, ACCEL_ADDRESS,fifoDataPointer, fifoLength, 1000);
}
