#ifndef COMMON_PORTING_H__
#define COMMON_PORTING_H__

#include <stdint.h>
#include "user_define.h"

//#define I2C_HANDLE	(hi2c1)
#define SPI_HANDLE	(hspi1)
#define UART_HANDLE	(huart2)

#define BUS_TIMEOUT             1000

typedef enum
{
 BHI260_GPIO_PIN_LOW = 0,
BHI260_GPIO_PIN_HIGH
}BHI260_GPIO_PinState;


typedef enum 
{
BHY2_INTERRUPT_IDLE = 0,
BHY2_INTERRUPT_READY = 1
};


void Enable_MCU_INT_Pin(void);
int Get_MCU_INT_Pin_Status(void);
void Enable_MCU_INT1_Pin(void);
void Disable_MCU_INT1_Pin(void);
void Enable_MCU_INT2_Pin(void);
void Disable_MCU_INT2_Pin(void);
void Enable_MCU_INT3_Pin(void);
void Disable_MCU_INT3_Pin(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


void bst_delay_us(uint32_t period, void *intf_ptr);
void UART_Printf(uint8_t* buff, uint16_t size);
void PDEBUG(char *format, ...);


#if defined(USE_BOSCH_SENSOR_API)
int8_t SensorAPI_I2Cx_Read_Nonptr(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length);
int8_t SensorAPI_I2Cx_Write_Nonptr(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length);
int8_t SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t BMI323_SensorAPI_I2Cx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t BMI323_SensorAPI_I2Cx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy1_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t bhy1_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
signed char BNO055_SensorAPI_I2Cx_Read(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char ReadNumbr);
signed char BNO055_SensorAPI_I2Cx_Write(unsigned char slave_address7, unsigned char subaddress, unsigned char *pBuffer, unsigned char WriteNumbr);


#endif


#endif
