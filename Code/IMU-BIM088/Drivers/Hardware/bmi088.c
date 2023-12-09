/**
 * @Author         : Minghang Li
 * @Date           : 2022-11-25 22:54
 * @LastEditTime   : 2022-11-28 16:32
 * @Note           :
 * @Copyright(c)   : Minghang Li Copyright
 */
#include "bmi088.h"

#include <math.h>
#include <stdio.h>

#include "bmi088reg.h"
#include "gpio.h"
#include "spi.h"

bmi088_error_e BMI088_INIT(void) 
{
    bmi088_error_e error = NO_ERROR;
    BMI088_CONF_INIT();

    error |= VerifyAccChipID();
    error |= VerifyGyroChipID();
	error |= VerifyAccSelfTest();
    error |= VerifyGyroSelfTest();

    return error;
}

void WriteDataToAcc(uint8_t addr, uint8_t data) 
{
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void WriteDataToGyro(uint8_t addr, uint8_t data) 
{
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data) 
{
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);          //dummy read,舍弃
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX);
	
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX);
	
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data) 
{
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX);
	
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data) 
{
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);             //dummy read,舍弃
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX);
	
    for (int i = 0; i < len; i++) 
	{
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX);
        data[i] = pRxData;
    }
	
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data) 
{
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX);
	
    for (int i = 0; i < len; i++) 
	{
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX);
        data[i] = pRxData;
    }
	
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void BMI088_CONF_INIT(void) 
{
	HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);      //CSB2上升沿
	HAL_Delay(10);
	
    //加速度计初始化
    //先软件重启
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    //开启加速度计电源
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    //开启加速度计normol mode
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    //加速度计设置
    //+-3g
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    //加速度计输出频率1600Hz，滤波器设置为normal模式
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    //陀螺仪初始化
    //先软件重启
    WriteDataToGyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    //开启陀螺仪normol mode
    WriteDataToGyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);
    //陀螺仪设置
    //+-500°/s
    WriteDataToGyro(GYRO_RANGE_ADDR, GYRO_RANGE_2000_DEG_S);
    //陀螺仪输出频率2000Hz,滤波器带宽532Hz
    WriteDataToGyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_1000Hz_BANDWIDTH_116Hz);
}

bmi088_error_e VerifyAccChipID(void) 
{
    uint8_t chip_id;
    ReadSingleDataFromAcc(ACC_CHIP_ID_ADDR, &chip_id);
	
    if (chip_id != ACC_CHIP_ID_VAL) 
        return ACC_CHIP_ID_ERR;
    
    return NO_ERROR;
}

bmi088_error_e VerifyGyroChipID(void) 
{
    uint8_t chip_id;
    ReadSingleDataFromGyro(GYRO_CHIP_ID_ADDR, &chip_id);
	
    if (chip_id != GYRO_CHIP_ID_VAL) 
        return GYRO_CHIP_ID_ERR;

    return NO_ERROR;
}

bmi088_error_e VerifyAccSelfTest(void) 
{
    acc_raw_data_t pos_data, neg_data;
	
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_24G);
    WriteDataToAcc(ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    ReadAccData(&pos_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    ReadAccData(&neg_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f)) 
        return ACC_DATA_ERR;

	
	//加速度计初始化
    //先软件重启
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    //开启加速度计电源
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    //开启加速度计normol mode
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    //加速度计设置
    //+-3g
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    //加速度计输出频率1600Hz，滤波器设置为normal模式
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));
	
    return NO_ERROR;
}

bmi088_error_e VerifyGyroSelfTest(void) 
{
	uint8_t bist_rdy = 0x00, bist_fail;
	WriteDataToGyro(GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
	
    while (bist_rdy == 0) 
	{
        ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
	
    ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
	
	
    if (bist_fail) 
		return GYRO_DATA_ERR;
	

	//陀螺仪初始化
    //先软件重启
    WriteDataToGyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    //开启陀螺仪normol mode
    WriteDataToGyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);
    //陀螺仪设置
    //+-500°/s
    WriteDataToGyro(GYRO_RANGE_ADDR, GYRO_RANGE_2000_DEG_S);
    //陀螺仪输出频率2000Hz,滤波器带宽532Hz
    WriteDataToGyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_1000Hz_BANDWIDTH_116Hz);
	return NO_ERROR;
}

void ReadAccData(acc_raw_data_t *data) 
{
    uint8_t buf[ACC_XYZ_LEN], range;
    int16_t acc[3];
    //ReadSingleDataFromAcc(ACC_RANGE_ADDR, &range);    
    ReadMultiDataFromAcc(ACC_X_LSB_ADDR, ACC_XYZ_LEN, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->x = (float)acc[0] * BMI088_ACCEL_3G_SEN;     //最终转换成m/s
    data->y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    data->z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
}

void ReadGyroData(gyro_raw_data_t *data) 
{
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
//    ReadSingleDataFromGyro(GYRO_RANGE_ADDR, &range);
//    switch (range) 
//	{
//        case 0x00:
//            unit = 16.384;
//            break;
//        case 0x01:
//            unit = 32.768;
//            break;
//        case 0x02:
//            unit = 65.536;
//            break;
//        case 0x03:
//            unit = 131.072;
//            break;
//        case 0x04:
//            unit = 262.144;
//            break;
//        default:
//            unit = 16.384;
//            break;
//    }

	unit = 16.384;
    ReadMultiDataFromGyro(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];

    data->roll = (float)gyro[0] / unit * DEG2SEC;         //最终转换成弧度制
    data->pitch = (float)gyro[1] / unit * DEG2SEC;
    data->yaw = (float)gyro[2] / unit * DEG2SEC;
}

void ReadAccSensorTime(float *time) 
{
    uint8_t buf[SENSORTIME_LEN];
    ReadMultiDataFromAcc(SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    *time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
}

void ReadAccTemperature(float *temp) 
{
    uint8_t buf[TEMP_LEN];
    ReadMultiDataFromAcc(TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023) 
        temp_int11 = (int16_t)temp_uint11 - 2048;
     else 
        temp_int11 = (int16_t)temp_uint11;
    
    *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}

