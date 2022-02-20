#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************//**
 *
 *         Port Definition
 *
 *******************************************************************************/
#define LED1_L_Pin     LED1_Pin
#define LED1_L_Port    LED1_GPIO_Port
/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
 
/****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/
#define LED1_ON  HAL_GPIO_WritePin(LED1_L_Port, LED1_L_Pin, GPIO_PIN_RESET)
#define LED1_OFF HAL_GPIO_WritePin(LED1_L_Port, LED1_L_Pin, GPIO_PIN_SET)

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

uint8 Led_initialization(void);
 
#ifdef __cplusplus
}
#endif

#endif



