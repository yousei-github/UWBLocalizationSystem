#ifndef DWM1000_PERIPHERAL_H_
#define DWM1000_PERIPHERAL_H_

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
// RESET PIN
#define DW_RESET0_Pin          KEY2_Pin
#define DW_RESET0_GPIO_Port    KEY2_GPIO_Port
#define DW_RESET0_EXTI_IRQn    EXTI0_IRQn
#define DW_RESET1_Pin          0
#define DW_RESET1_GPIO_Port    0
#define DW_RESET1_EXTI_IRQn    EXTI0_IRQn
	
// SPI NSS PIN
#define DW_NSS0_Pin          DWM1000_CSN_Pin
#define DW_NSS0_GPIO_Port    DWM1000_CSN_GPIO_Port
#define DW_NSS1_Pin          0
#define DW_NSS1_GPIO_Port    0
// SPI SCK PIN
#define DW_SCK_Pin           DWM1000_SCK_Pin
#define DW_SCK_GPIO_Port     DWM1000_SCK_GPIO_Port
// SPI MISO PIN
#define DW_MISO_Pin          DWM1000_MISO_Pin
#define DW_MISO_GPIO_Port    DWM1000_MISO_GPIO_Port
// SPI MOSI PIN
#define DW_MOSI_Pin          DWM1000_MOSI_Pin
#define DW_MOSI_GPIO_Port    DWM1000_MOSI_GPIO_Port
// WAKE UP PIN
#define DW_WUP0_Pin          GPIO_PIN_0
#define DW_WUP0_GPIO_Port    GPIOB
#define DW_WUP1_Pin          GPIO_PIN_0
#define DW_WUP1_GPIO_Port    GPIOB
// IRQ PIN
#define DW_IRQn0_Pin          DWM1000_IRQ_Pin
#define DW_IRQn0_GPIO_Port    DWM1000_IRQ_GPIO_Port
#define DW_IRQn0_EXTI_IRQn    DWM1000_IRQ_EXTI_IRQn
#define DW_IRQn1_Pin          0
#define DW_IRQn1_GPIO_Port    0
#define DW_IRQn1_EXTI_IRQn    0

#define DECAIRQ	              DW_IRQn0_Pin
#define DECAIRQ_GPIO          DW_IRQn0_GPIO_Port
#define DECAIRQ_EXTI_IRQn     DWM1000_IRQ_EXTI_IRQn
/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
// SPI Handle
#define DWM1000_SPI hspi2
#define hspi1 DWM1000_SPI
// SPI clock
#define APB_CLOCK_MHZ 45
#define FAST_SPI_SPEED_MHZ 20
#define SLOW_SPI_SPEED_MHZ 3
// Internal use, Don't touch!
#if ((APB_CLOCK_MHZ/2) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_2
#elif ((APB_CLOCK_MHZ/4) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_4
#elif ((APB_CLOCK_MHZ/8) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_8
#elif ((APB_CLOCK_MHZ/16) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_16
#elif ((APB_CLOCK_MHZ/32) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_32
#elif ((APB_CLOCK_MHZ/64) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_64
#elif ((APB_CLOCK_MHZ/128) < FAST_SPI_SPEED_MHZ)
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_128
#else
#define SPI_FAST_BANDPRESCALER SPI_BAUDRATEPRESCALER_256
#endif

#if ((APB_CLOCK_MHZ/2) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_2
#elif ((APB_CLOCK_MHZ/4) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_4
#elif ((APB_CLOCK_MHZ/8) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_8
#elif ((APB_CLOCK_MHZ/16) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_16
#elif ((APB_CLOCK_MHZ/32) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_32
#elif ((APB_CLOCK_MHZ/64) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_64
#elif ((APB_CLOCK_MHZ/128) < SLOW_SPI_SPEED_MHZ)
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_128
#else
#define SPI_SLOW_BANDPRESCALER SPI_BAUDRATEPRESCALER_256
#endif
// Miscellaneous
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
// Delay function
#define Get_tick HAL_GetTick
#if USE_OS_UCOS_III
#define Delay_ms DWM1000_delay
#else
#define Delay_ms HAL_Delay
#endif


#define GPIO_ResetBits(x,y)             HAL_GPIO_WritePin(x,y, GPIO_PIN_RESET)
#define GPIO_SetBits(x,y)               HAL_GPIO_WritePin(x,y, GPIO_PIN_SET)
#define GPIO_ReadInputDataBit(x,y)      HAL_GPIO_ReadPin (x,y)

/* NSS pin is SW controllable */
#define port_SPIx_set_chip_select()     HAL_GPIO_WritePin(DW_NSS0_GPIO_Port, DW_NSS0_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()   HAL_GPIO_WritePin(DW_NSS0_GPIO_Port, DW_NSS0_Pin, GPIO_PIN_RESET)

#define DWM1000_EXTI_Callback           HAL_GPIO_EXTI_Callback
/****************************************************************************//**
 *
 *         Types definitions
 *
 *******************************************************************************/
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

/* DW1000 IRQ (EXTI9_5_IRQ) handler type. */
typedef void(*port_deca_isr_t)(void);
/****************************************************************************//**
 *
 *         Function prototypes
 *
 *******************************************************************************/
#if USE_OS_UCOS_III
void DWM1000_delay(uint32 _t);
#endif
/* DW1000 IRQ handler declaration. */
extern port_deca_isr_t port_deca_isr;

uint32_t portGetTickCnt(void);
void Sleep(uint32_t Delay);

ITStatus EXTI_GetITEnStatus(uint32_t x);

uint8 reset_DW1000(uint8 _module_nb);

uint8 setup_DW1000RSTnIRQ(int _enable, uint8 _module_nb);

void port_wakeup_dw1000(void);
void port_wakeup_dw1000_fast(void);

void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);

u8 dwm_spi_readwritebyte(u8 TxData);
u8 dwm_readbyte(unsigned char regadr, uint8 _nb);
u8 dwm_writebyte(unsigned char regadr, unsigned char val, uint8 _nb);

void process_deca_irq(void);

void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr);



#ifdef __cplusplus
}
#endif

#endif /* DWM1000_PERIPHERAL_H_ */



