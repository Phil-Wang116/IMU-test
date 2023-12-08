#ifndef BMI270_H
#define BMI270_H

#include <stdint.h>
#include "user_define.h"
#include "bmi3_defs.h"


/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(32)
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)


void bmi3_error_codes_print_result(const char api_name[], int8_t rslt);

int8_t Open_BMI323_ACC(struct bmi3_dev *dev);
int8_t Close_BMI323_ACC(struct bmi3_dev *dev);
int8_t Open_BMI323_GYRO(struct bmi3_dev *dev);
int8_t Close_BMI323_GYRO(struct bmi3_dev *dev);

#if defined(RDRY_INT)
int8_t Open_BMI323_DRDY_INT(struct bmi3_dev *dev);
int8_t Close_BMI323_DRDY_INT(struct bmi3_dev *dev);
#endif

#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
int8_t Open_BMI323_FIFO(struct bmi3_dev *dev);
int8_t Close_BMI323_FIFO(struct bmi3_dev *dev);
#endif

void BMI323_Print_ALLRegs(struct bmi3_dev *dev);
int8_t Init_BMI323(struct bmi3_dev *dev);
void StartBMI323Task(void const * argument);

#if defined(FIFO_POLL) || defined(FIFO_WM_INT) || defined(STEP_COUNTER)
void BMI323_Timer_Callback(TimerHandle_t xTimer);
#endif

uint8_t read_sensor(uint8_t index, uint8_t gyrdata_x, uint8_t gyrdata_y, uint8_t gyrdata_z);

#if defined(STEP_COUNTER)
int8_t Open_BMI323_STEP_COUNTER(struct bmi3_dev *dev);
int8_t Close_BMI323_STEP_COUNTER(struct bmi3_dev *dev);
#endif

#if defined(ACC_RDRY_INT) || defined(GYRO_RDRY_INT)
void StartBMI323InterruptTask(void const * argument);
#endif


#endif
