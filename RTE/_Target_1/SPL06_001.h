#ifndef SPL06_01_H_
#define SPL06_01_H_

#ifdef __cplusplus
extern "C" {
#endif
	
//#include "spi.h"

/****************************************************************************//**
 *
 *         Port Definition
 *
 *******************************************************************************/
#define SPL06_CS_PIN		SENSOR_NSS3_Pin
#define SPL06_CS_GPIO		SENSOR_NSS3_GPIO_Port
#define SPL06_CS_RCC		__HAL_RCC_GPIOB_CLK_ENABLE()


/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
#define SPL06_SPI hspi1
#define SPI3_ReadWrite_Byte    SPI2_ReadWriteByte
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

#define USE_IIC_FOR_SPL06  0
#define USE_SPI_FOR_SPL06  1
/****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/


/****************************************************************************//**
 *
 *         Types definitions
 *
 *******************************************************************************/
#include "stdio.h"
#include "stdint.h"
#ifndef USE_USER_TYPES 
#define USE_USER_TYPES 1
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef uint32_t    uint32;
typedef int8_t      int8;
typedef int16_t     int16;
typedef int32_t     int32;
typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;
typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;
typedef signed long long int64;
typedef unsigned long long uint64;
#endif


struct spl0601_calib_param_t 
{	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

struct spl0601_t 
{	
    struct spl0601_calib_param_t calib_param;/**<calibration data*/	
    u8 			chip_id; /**<chip id*/	
    int32_t 	i32rawPressure;
    int32_t 	i32rawTemperature;
    int32_t 	i32kP;    
    int32_t 	i32kT;
};

/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
extern s32 baro_height;
extern s32 baro_pa;
extern float baro_temperature;


/****************************************************************************//**
 *
 *         Function prototypes
 *
 *******************************************************************************/
#if USE_IIC_FOR_SPL06
unsigned char spl0601_init(void);

float user_spl0601_get_presure(void);
float user_spl0601_get_temperature(void);
#elif  USE_SPI_FOR_SPL06

u8 Drv_Spl0601_Init(void);
void Drv_spl06_soft_reset(void);
float Drv_Spl0601_Read(void);
unsigned char Drv_Spl0601_Offset(unsigned char _times);
#endif

#ifdef __cplusplus
}
#endif


#endif

