#ifndef POSITION_ALGORITHM_H_
#define POSITION_ALGORITHM_H_

#ifdef __cplusplus
extern "C" {
#endif

#define USE_DEVICE "stm32f4xx.h"
#define USE_DRIVER "stm32f4xx_hal.h"
#include "main.h"
#define USE_OS_UCOS_III 1
#include <stdint.h>
#include <string.h>
#include USE_DEVICE
#include USE_DRIVER
#if USE_OS_UCOS_III
#include  <UCOS_includes.h>
#endif
/****************************************************************************//**
 *
 *         Port Definition
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
#define THREED_TRILATERATION_LEAST_ANCHOR_NUM 4
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

typedef enum
{
	Anchor_NO1 = 0,
	Anchor_NO2,
	Anchor_NO3,
	Anchor_NO4,
	All_anchors
}Anchor_order;

typedef struct
{
	float x;
	float y;
	float z;
}Location_Cartesian_coordinate;

typedef struct
{
	Location_Cartesian_coordinate coord;
	float distance;
}TOF_position;

typedef struct
{
	TOF_position anchor[THREED_TRILATERATION_LEAST_ANCHOR_NUM];
	Location_Cartesian_coordinate position;
}Position_calcalation;
/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
 
 
 /****************************************************************************//**
 *
 *         Function prototypes
 *
 *******************************************************************************/
uint8 Positioning_initialization(void);

uint8 Update_threeD_trilaterationpositioning(float _x, float _y, float _z, double _distance, uint16 _no);

uint8 ThreeD_trilaterationpositioning_run(void);

 
 #ifdef __cplusplus
}
#endif


#endif 


