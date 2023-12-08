#ifndef USER_DEFINE_H__
#define USER_DEFINE_H__

#define DEBUG_EN

#define USE_BOSCH_SENSOR_API

//#define USE_SHUTTLE20
#define USE_SHUTTLE30

//#define USE_I2C_INTERFACE
#define USE_SPI_INTERFACE

//#define USE_BMA253
//#define USE_BMA400
//#define USE_BMA456AN
//#define USE_BMA456MM
//#define USE_BMA456H
//#define USE_BMA456W
//#define USE_BMA456H
//#define USE_BMI08X
//#define USE_BMI160
//#define USE_BMX160
//#define USE_BMI270
#define USE_BMI323
//#define USE_BNO055
//#define USE_BHI160
//#define USE_BHI260
//#define USE_BNO055
//#define USE_BME280
//#define USE_BME68X
//#define USE_BMP3XX
//#define USE_BMP58X
//#define USE_BMM150

#if defined(USE_BMA253)

#elif defined(USE_BMA400)

#elif defined(USE_BMA456AN)
	//#define ACC_RDRY_INT
	//#define ANY_MOTION
	//#define NO_MOTION
	//#define FIFO_POLL
	#define FIFO_WM_INT
	//#define SUPPORT_LOWPOWER
#elif defined(USE_BMA456H)

#elif defined(USE_BMA456W)

#elif defined(USE_BMA456MM)

#elif defined(USE_BMI08X)

#elif defined(USE_BMI160) || defined(USE_BMX160)
//#define ACC_ONLY
//#define GYRO_ONLY
#define ACC_GYRO
//#define SELFTEST
//#define FOC
//#define TAP
//#define NO_MOTION
//#define STEP_COUNTER
//#define DATA_POLL
//#define FIFO_POLL
#define FIFO_WM_INT
#elif defined(USE_BMI270)
	#define ACC_SELFTEST
	#define GYRO_SELFTEST
	#define ACC_GYRO_SELFTEST
	#define CRT
	#define ACC_ONLY
	#define GYRO_ONLY
	#define ACC_GYRO
		#if defined(ACC_ONLY) || defined(ACC_GYRO)
		#define STEP_COUNTER
		#define WRIST_WEAR_WAKE_UP
		#endif
	/*Only support to select one*/
	#define RDRY_INT
	#define FIFO_POLL
	#define FIFO_WM_INT
	#define SUPPORT_LOWPOWER
#elif defined(USE_BMI323)
	//#define ACC_SELFTEST
	//#define GYRO_SELFTEST
	//#define ACC_GYRO_SELFTEST
	//#define FOC
	//#define CRT
	//#define ACC_ONLY
	#define GYRO_ONLY
//	#define ACC_GYRO
//		#if defined(ACC_ONLY) || defined(ACC_GYRO)
//		#define STEP_COUNTER
//		#endif
	/*Only support to select one*/
	//#define ACC_RDRY_INT
	//#define GYRO_RDRY_INT

	#define FIFO_POLL
//	#define FIFO_WM_INT
//	#define SUPPORT_LOWPOWER

#elif defined(USE_BNO055)


#elif defined(USE_BHI160)
	//#define BHI160_6DOF
	#define BHI160_BMM150_9DOF

	#define BHI160_FIFO_INTERRUPT_MODE
	//#define BHI160_FIFO_POLL_MODE
#elif defined(USE_BHI260)
	//#define BHI260_6DOF
	//#define BHI260_BMM150_9DOF
	//#define BHI260_SELF_LEARNING
	#define BHI260_PDR

	#define BHI260_FIFO_INTERRUPT_MODE
	//#define BHI260_FIFO_POLL_MODE
#elif defined(USE_BNO055)


#elif defined(USE_BME280)

#elif defined(USE_BME68X)
#define BME68X_SELF_TEST
//#define USE_BME68X_FORCE_MODE
//#define USE_BME68X_PARALLEL_MODE
//#define USE_BME68X_SEQUENTIAL_MODE

#elif defined(USE_BMP3XX)

#elif defined(USE_BMP58X)

#elif defined(USE_BMM150)
	//#define LOW_THRESHOLD
	//#define HIGH_THRESHOLD
	//#define DATA_POLL
	#define DATA_READY_INT
#endif

#endif
