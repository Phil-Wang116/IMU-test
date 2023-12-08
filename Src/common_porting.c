#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "stm32l4xx_hal.h"
#include "common_porting.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
//extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern volatile uint16_t bhi260_fifo_ready;

#if defined(FIFO_WM_INT)
extern volatile uint16_t bma425_fifo_ready;
extern volatile uint16_t bma400_fifo_ready;
extern volatile uint16_t bma456an_fifo_ready;
extern volatile uint16_t bma456h_fifo_ready;
extern volatile uint16_t bma456mm_fifo_ready;
extern volatile uint16_t bma456w_fifo_ready;
extern volatile uint16_t bmi160_fifo_ready;
extern volatile uint16_t bmi270_fifo_ready;
extern volatile uint16_t bmi323_fifo_ready;
#endif


uint8_t GTXBuffer[512], GRXBuffer[2048];

volatile uint8_t int1_flag = 0;
volatile uint8_t int2_flag = 0;
volatile uint8_t int3_flag = 0;


#if defined(USE_SHUTTLE20)
void Enable_MCU_INT_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	/*Configure GPIO pin : INT1_Pin */
	GPIO_InitStruct.Pin = INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
}

int Get_MCU_INT_Pin_Status(void)
{
	int rslt;
	if(HAL_GPIO_ReadPin(INT1_GPIO_Port, INT1_Pin) == GPIO_PIN_SET)
	{
		rslt = BHI260_GPIO_PIN_HIGH;
	}
	else
	{
		rslt = BHI260_GPIO_PIN_LOW;
	}

	return rslt;
}
#elif defined(USE_SHUTTLE30)
//void Enable_MCU_INT_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin */
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//}
//
//int Get_MCU_INT_Pin_Status(void)
//{
//	int rslt;
//	if(HAL_GPIO_ReadPin(INT2_GPIO_Port, INT2_Pin) == GPIO_PIN_SET)
//	{
//		rslt = BHI260_GPIO_PIN_HIGH;
//	}
//	else
//	{
//		rslt = BHI260_GPIO_PIN_LOW;
//	}
//
//	return rslt;
//}

#endif

//void Enable_MCU_INT1_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//#if defined(USE_SHUTTLE20)
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT1_Pin PA10*/
//	GPIO_InitStruct.Pin = INT1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//#elif defined(USE_SHUTTLE30)
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin PB3*/
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	//GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
//  	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
//#endif
//}

//void Disable_MCU_INT1_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//#if defined(USE_SHUTTLE20)
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT1_Pin PA10*/
//	GPIO_InitStruct.Pin = INT1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//#elif defined(USE_SHUTTLE30)
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin PB3*/
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
//  	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
//#endif
//}
//
//void Enable_MCU_INT2_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//#if defined(USE_SHUTTLE20)
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin PB3*/
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
//  	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
//#elif defined(USE_SHUTTLE30)
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT1_Pin PA10*/
//	GPIO_InitStruct.Pin = INT1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//#endif
//}
//
//void Disable_MCU_INT2_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//#if defined(USE_SHUTTLE20)
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT2_Pin PB3*/
//	GPIO_InitStruct.Pin = INT2_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
//  	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
//#elif defined(USE_SHUTTLE30)
//	/*Configure GPIO pin : INT1_Pin PA10*/
//	GPIO_InitStruct.Pin = INT1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//
//#endif
//}
//
//void Enable_MCU_INT3_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//#if defined(USE_SHUTTLE30)
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//
//	/*Configure GPIO pin : INT3_Pin PA9*/
//	GPIO_InitStruct.Pin = INT3_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT3_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
//  	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//
//#endif
//}
//
//void Disable_MCU_INT3_Pin(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//#if defined(USE_SHUTTLE30)
//	/*Configure GPIO pin : INT3_Pin PA9*/
//	GPIO_InitStruct.Pin = INT3_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(INT3_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
//	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
//
//#endif
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_9)//PA9
	{
		#if defined(USE_SHUTTLE30)
		int3_flag = 1;
		PDEBUG("INT3 Triggered\r\n");
		#endif
	}
	else if(GPIO_Pin == GPIO_PIN_10)//PA10
	{
		#if defined(USE_SHUTTLE20)
		int1_flag = 1;
		PDEBUG("INT1 Triggered\r\n");
		#elif defined(USE_SHUTTLE30)
		int2_flag = 1;
		PDEBUG("INT2 Triggered\r\n");
			#if defined(FIFO_WM_INT)
				#if defined(USE_BMA400)
				bma400_fifo_ready = 1;
				#elif defined(USE_BMA456AN)
				bma456an_fifo_ready = 1;
				#elif defined(USE_BMI160)
				bmi160_fifo_ready = 1;
				#elif defined(USE_BMI270)
				bmi270_fifo_ready = 1;
				#elif defined(USE_BMI323)
				bmi323_fifo_ready = 1;
				#endif
			#endif
		#endif
	}
	else if(GPIO_Pin == GPIO_PIN_3)//PB3
	{
		#if defined(USE_SHUTTLE20)
		int2_flag = 1;
		PDEBUG("INT2 Triggered\r\n");
			#if defined(FIFO_WM_INT)
				#if defined(USE_BMA400)
				bma400_fifo_ready = 1;
				#elif defined(USE_BMA456AN)
				bma456an_fifo_ready = 1;
				#elif defined(USE_BMI160)
				bmi160_fifo_ready = 1;
				#elif defined(USE_BMI270)
				bmi270_fifo_ready = 1;
				#elif defined(USE_BMI323)
				bmi323_fifo_ready = 1;
				#endif
			#endif
		#elif defined(USE_SHUTTLE30)
		int1_flag = 1;
		#if defined(BHI260_FIFO_INTERRUPT_MODE)
		bhi260_fifo_ready = BHY2_INTERRUPT_READY;
		#endif
		//PDEBUG("INT1 Triggered\r\n");
		#endif

	}
	else if(GPIO_Pin == GPIO_PIN_5)//DRDY_BMM150
	{
		PDEBUG("DRDY_BMM150 Triggered\r\n");
	}
}

void bst_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}

void UART_Printf(uint8_t* buff, uint16_t size)
{
    //HAL_UART_Transmit_DMA(&huart2, buff, size);
    HAL_UART_Transmit(&UART_HANDLE, buff, size, BUS_TIMEOUT);
}

char chBuffer[512];
void PDEBUG(char *format, ...)
{
#if defined(DEBUG_EN)
    va_list ap;
    //char timestamp[16];
    va_start(ap, format);
    vsnprintf(chBuffer, sizeof(chBuffer), format, ap);
    //sprintf(timestamp, "[%d]", xTaskGetTickCount()); //xTaskGetTickCountFromISR()
    //Printf((uint8_t *)timestamp, strlen(timestamp));
    UART_Printf((uint8_t *)chBuffer,strlen(chBuffer));
    va_end(ap);
#endif
}

#if defined(USE_BOSCH_SENSOR_API)

//int8_t SensorAPI_I2Cx_Read_Nonptr(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
//{
//	uint16_t DevAddress = dev_addr << 1;
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, &reg_addr, 1, BUS_TIMEOUT);
//	HAL_I2C_Master_Receive(&I2C_HANDLE, DevAddress, reg_data, length, BUS_TIMEOUT);
//	return 0;
//}
//
//int8_t SensorAPI_I2Cx_Write_Nonptr(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length)
//{
//	uint16_t DevAddress = dev_addr << 1;
//
//	GTXBuffer[0] = reg_addr;
//	memcpy(&GTXBuffer[1], reg_data, length);
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, GTXBuffer, length+1, BUS_TIMEOUT);
//	return 0;
//}
//
//
//int8_t SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//	uint8_t dev_addr = *(uint8_t*)intf_ptr;
//	uint16_t DevAddress = dev_addr << 1;
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, &reg_addr, 1, BUS_TIMEOUT);
//	HAL_I2C_Master_Receive(&I2C_HANDLE, DevAddress, reg_data, length, BUS_TIMEOUT);
//	return 0;
//}
//
//int8_t SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//	uint8_t dev_addr = *(uint8_t*)intf_ptr;
//	uint16_t DevAddress = dev_addr << 1;
//
//	GTXBuffer[0] = reg_addr;
//	memcpy(&GTXBuffer[1], reg_data, length);
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, GTXBuffer, length+1, BUS_TIMEOUT);
//	return 0;
//}
//
//int8_t BMI323_SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//	uint8_t dev_addr = *(uint8_t*)intf_ptr;
//	uint16_t DevAddress = dev_addr << 1;
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, &reg_addr, 1, BUS_TIMEOUT);
//	HAL_I2C_Master_Receive(&I2C_HANDLE, DevAddress, GRXBuffer, length+2, BUS_TIMEOUT);
//	memcpy(reg_data, GRXBuffer+2, length);
//	return 0;
//}
//
//int8_t BMI323_SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//	uint8_t dev_addr = *(uint8_t*)intf_ptr;
//	uint16_t DevAddress = dev_addr << 1;
//
//	GTXBuffer[0] = reg_addr;
//	memcpy(&GTXBuffer[1], reg_data, length);
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, GTXBuffer, length+1, BUS_TIMEOUT);
//	return 0;
//}

int8_t SensorAPI_SPIx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}

int8_t SensorAPI_SPIx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // NSS high

    return 0;
}

int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	GTXBuffer[0] = reg_addr | 0x80;

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // NSS low

	//HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
	HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
	while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // NSS high
	memcpy(reg_data, GRXBuffer+1, length);

	return 0;
}

int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	GTXBuffer[0] = reg_addr & 0x7F;
	memcpy(&GTXBuffer[1], reg_data, length);

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // NSS low

	//HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
	HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
	while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // NSS high

	return 0;
}

//int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//	uint8_t dev_addr = 0x29 << 1;
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, dev_addr, &reg_addr, 1, BUS_TIMEOUT);
//	HAL_I2C_Master_Receive(&I2C_HANDLE, dev_addr, reg_data, length, BUS_TIMEOUT);
//
//	return 0;
//}
//
//int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
//{
//	uint8_t dev_addr = 0x29 << 1;
//	GTXBuffer[0] = reg_addr;
//	memcpy(&GTXBuffer[1], reg_data, length);
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, dev_addr, GTXBuffer, length+1, BUS_TIMEOUT);
//
//	return 0;
//}
//
//int8_t bhy1_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
//{
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, addr, &reg, 1, BUS_TIMEOUT);
//	HAL_I2C_Master_Receive(&I2C_HANDLE, addr, p_buf, size, BUS_TIMEOUT);
//
//	return 0;
//}
//
//int8_t bhy1_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
//{
//	GTXBuffer[0] = reg;
//	memcpy(&GTXBuffer[1], p_buf, size);
//
//	// send register address
//	HAL_I2C_Master_Transmit(&I2C_HANDLE, addr, GTXBuffer, size+1, BUS_TIMEOUT);
//
//	return 0;
//}
//
//signed char BNO055_SensorAPI_I2Cx_Read(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char ReadNumbr)
//{
//    uint16_t DevAddress = slave_address7 << 1;
//
//    // send register address
//    HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, &subaddress, 1, BUS_TIMEOUT);
//    HAL_I2C_Master_Receive(&I2C_HANDLE, DevAddress, pBuffer, ReadNumbr, BUS_TIMEOUT);
//    return 0;
//}
//
//signed char BNO055_SensorAPI_I2Cx_Write(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char WriteNumbr)
//{
//    uint16_t DevAddress = slave_address7 << 1;
//
//    GTXBuffer[0] = subaddress;
//    memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);
//
//    // send register address
//    HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, GTXBuffer, WriteNumbr+1, BUS_TIMEOUT);
//    return 0;
//}


#endif


