/**
  ******************************************************************************
  * File Name          : MPU9255.h
  * Description        : MPU9255 函式庫標頭檔
  ******************************************************************************
  * MIAT 實驗室製作
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU9255_H
#define __MPU9255_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "MPU_9255_ref.h"
/* Exported types ------------------------------------------------------------*/
typedef uint8_t (*MPU_9255_i2cTransiver_t)(uint16_t DevAddress, uint8_t *pData, uint16_t size);

// Set initial input parameters
typedef enum  {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
}ASCALE_t;

typedef enum {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
}GSCALE_t;

typedef enum {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
}MSCALE_T;

typedef enum {
	MGT_8HZ = 0x02,		//8 Hz
	MGT_100HZ = 0x06	//100 Hz
}MMODE_t;

typedef enum {
	SMPLRT_DIV_1000Hz = 0x00,
	SMPLRT_DIV_500Hz, // = 0x01,
	SMPLRT_DIV_333Hz, // = 0x02,
	SMPLRT_DIV_250Hz, // = 0x03,
	SMPLRT_DIV_200Hz, // = 0x04,
	SMPLRT_DIV_166Hz, // = 0x05,
	SMPLRT_DIV_142Hz, // = 0x06,
	SMPLRT_DIV_125Hz 	//= 0x07
}SMPLRT_DIV_t;

typedef struct{
	int           sample_rate;
  ASCALE_t      Ascale;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
  GSCALE_t      Gscale; 		// GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
  MSCALE_T      Mscale; 		// MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
  MMODE_t       Mmode;   		// Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR 
  SMPLRT_DIV_t  Smplrt_div;	//sample_rate = Internal_Sample_Rate / (1 + SMPLRT_DIV)

  float aRes;
  float gRes;
  float mRes; // scale resolutions per LSB for the sensors

  // parameters for 6 DoF sensor fusion calculations
  float magCalibration[3];
  float magbias[3];  // Factory mag calibration and mag bias
  float gyroBias[3];
  float accelBias[3]; // Bias corrections for gyro and accelerometer
  float SelfTest[6];

  //quaternion
  float q[4];           // vector to hold quaternion
  float eInt[3];              // vector to hold integral error for Mahony method
  float SEq_1;
  float SEq_2;
  float SEq_3;
  float SEq_4; // estimated orientation quaternion elements with initial conditions
  MPU_9255_i2cTransiver_t writer;
  MPU_9255_i2cTransiver_t reader;
  uint8_t addr;
} MPU_9255_t;

MPU_9255_t *MPU_9255_new(int addr, uint8_t smplrt_div, uint8_t mmode, uint8_t afs, uint8_t gfs, uint8_t mfs, MPU_9255_i2cTransiver_t reader, MPU_9255_i2cTransiver_t writer);
void        MPU_9255_destory(MPU_9255_t *nmpu);
void        MPU_9255_SetTransiver(MPU_9255_t *hmpu, MPU_9255_i2cTransiver_t reader, MPU_9255_i2cTransiver_t writer);
uint8_t     MPU_9255_whoami(MPU_9255_t *hmpu);
void        MPU_9255_initSensor(MPU_9255_t *hmpu);
void        MPU_9255_printInfo(MPU_9255_t *hmpu);
uint8_t     MPU_9255_isDataReady(MPU_9255_t *hmpu);
void        MPU_9255_readAccelData(MPU_9255_t *hmpu, float *ax, float *ay, float *az);
void        MPU_9255_readGyroData(MPU_9255_t *hmpu, float *gx, float *gy, float *gz);
void        MPU_9255_readMagData(MPU_9255_t *hmpu, float *mx, float *my, float *mz);
void        MPU_9255_filterUpdate(MPU_9255_t *hmpu, float gx, float gy, float gz, 
                                          float ax, float ay, float az, float deltat);
void        MPU_9255_getEulerDegreeFilter(MPU_9255_t *hmpu, float *yaw, float *pitch, float *roll);
float       MPU_9255_readTempData(MPU_9255_t *hmpu);
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* __MPU9255_H */

/*********************************END OF FILE**********************************/
