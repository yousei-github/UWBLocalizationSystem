#include "led.h"

#define USE_DEVICE "stm32f4xx.h"
#define USE_DRIVER "stm32f4xx_hal.h"
#include "main.h"
#define USE_OS_UCOS_III 1
#include USE_DEVICE
#include USE_DRIVER
#if USE_OS_UCOS_III
#include  <UCOS_includes.h>
#endif
/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
 
/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif
/****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/
#define LED_Delay Led_delay
/****************************************************************************//**
 *
 *         Private variables and function prototypes
 *
 *******************************************************************************/
static void Led_delay(uint32 _t);

/****************************************************************************//**
 *
 *         LED function section
 *
 *******************************************************************************/

uint8 Led_initialization(void)
{
	LED1_ON;
	LED_Delay(500);
	LED1_OFF;
	LED_Delay(500);
	
	return TRUE;
}


#if USE_OS_UCOS_III
static void Led_delay(uint32 _t)
{
	OS_ERR  err;
	OSTimeDlyHMSM(0u, 0u, 0u, _t,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
}
#endif


