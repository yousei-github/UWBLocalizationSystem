/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                            (c) Copyright 2007-2008; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : bsp_periph.c
* Version       : V1.00
* Programmer(s) : EHS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_PERIPH_MODULE
#include <bsp.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

//#define  BSP_PERIPH_REG_RCC_BASE                  0x40023800
//#define  BSP_PERIPH_REG_RCC_CFGR                  (*(CPU_REG32 *)(BSP_PERIPH_REG_RCC_BASE + 0x008))
//#define  BSP_PERIPH_REG_RCC_AHB1ENR               (*(CPU_REG32 *)(BSP_PERIPH_REG_RCC_BASE + 0x030))
//#define  BSP_PERIPH_REG_RCC_AHB2ENR               (*(CPU_REG32 *)(BSP_PERIPH_REG_RCC_BASE + 0x034))
//#define  BSP_PERIPH_REG_RCC_AHB3ENR               (*(CPU_REG32 *)(BSP_PERIPH_REG_RCC_BASE + 0x038))
//#define  BSP_PERIPH_REG_RCC_APB1ENR               (*(CPU_REG32 *)(BSP_PERIPH_REG_RCC_BASE + 0x040))
//#define  BSP_PERIPH_REG_RCC_APB2ENR               (*(CPU_REG32 *)(BSP_PERIPH_REG_RCC_BASE + 0x044))


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         BSP_PeriphClkFreqGet()
*
* Description : Get clock frequency of a peripheral.
*
* Argument(s) : pwr_clk_id      Power/clock ID.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

//CPU_INT32U  BSP_PeriphClkFreqGet (CPU_DATA  pwr_clk_id)
//{
//    CPU_INT32U  clk_freq;
//    CPU_INT32U  clk_div;


//    clk_freq = BSP_CPU_ClkFreq();

//    switch (pwr_clk_id) {
//        case BSP_PERIPH_ID_GPIOA:
//        case BSP_PERIPH_ID_GPIOB:
//        case BSP_PERIPH_ID_GPIOC:
//        case BSP_PERIPH_ID_GPIOD:
//        case BSP_PERIPH_ID_GPIOE:
//        case BSP_PERIPH_ID_GPIOF:
//        case BSP_PERIPH_ID_GPIOG:
//        case BSP_PERIPH_ID_GPIOH:
//        case BSP_PERIPH_ID_GPIOI:
//        case BSP_PERIPH_ID_GPIOJ:
//        case BSP_PERIPH_ID_GPIOK:
//        case BSP_PERIPH_ID_CRC:
//        case BSP_PERIPH_ID_BKPSRAM:
//        case BSP_PERIPH_ID_CCMDATARAM:
//        case BSP_PERIPH_ID_DMA1:
//        case BSP_PERIPH_ID_DMA2:
//        case BSP_PERIPH_ID_DMA2D:
//        case BSP_PERIPH_ID_ETHMAC:
//        case BSP_PERIPH_ID_ETHMACPTP:
//        case BSP_PERIPH_ID_ETHMACTX:
//        case BSP_PERIPH_ID_ETHMACRX:
//        case BSP_PERIPH_ID_OTGHS:
//        case BSP_PERIPH_ID_OTGHSULPI:
//        case BSP_PERIPH_ID_DCMI:
//        case BSP_PERIPH_ID_CRYP:
//        case BSP_PERIPH_ID_HASH:
//        case BSP_PERIPH_ID_RNG:
//        case BSP_PERIPH_ID_FSMC:
//             return (clk_freq);

//        case BSP_PERIPH_ID_TIM2:
//        case BSP_PERIPH_ID_TIM3:
//        case BSP_PERIPH_ID_TIM4:
//        case BSP_PERIPH_ID_TIM5:
//        case BSP_PERIPH_ID_TIM6:
//        case BSP_PERIPH_ID_TIM7:
//        case BSP_PERIPH_ID_TIM12:
//        case BSP_PERIPH_ID_TIM13:
//        case BSP_PERIPH_ID_TIM14:
//        case BSP_PERIPH_ID_WWDG:
//        case BSP_PERIPH_ID_SPI2:
//        case BSP_PERIPH_ID_SPI3:
//        case BSP_PERIPH_ID_USART2:
//        case BSP_PERIPH_ID_USART3:
//        case BSP_PERIPH_ID_UART4:
//        case BSP_PERIPH_ID_UART5:
//        case BSP_PERIPH_ID_I2C1:
//        case BSP_PERIPH_ID_I2C2:
//        case BSP_PERIPH_ID_I2C3:
//        case BSP_PERIPH_ID_CAN1:
//        case BSP_PERIPH_ID_CAN2:
//        case BSP_PERIPH_ID_PWR:
//        case BSP_PERIPH_ID_DAC:
//        case BSP_PERIPH_ID_UART7:
//        case BSP_PERIPH_ID_UART8:
//             clk_div = (BSP_PERIPH_REG_RCC_CFGR & 0x00001C00) >> 10;
//             if (clk_div < 4) {
//                 return (clk_freq);
//             }
//             clk_div   = (clk_div - 3) << 1;
//             clk_freq /= clk_div;
//             return (clk_freq);

//        case BSP_PERIPH_ID_TIM1:
//        case BSP_PERIPH_ID_TIM8:
//        case BSP_PERIPH_ID_USART1:
//        case BSP_PERIPH_ID_USART6:
//        case BSP_PERIPH_ID_ADC1:
//        case BSP_PERIPH_ID_ADC2:
//        case BSP_PERIPH_ID_ADC3:
//        case BSP_PERIPH_ID_SDIO:
//        case BSP_PERIPH_ID_SPI1:
//        case BSP_PERIPH_ID_SPI4:
//        case BSP_PERIPH_ID_SYSCFG:
//        case BSP_PERIPH_ID_TIM9:
//        case BSP_PERIPH_ID_TIM10:
//        case BSP_PERIPH_ID_TIM11:
//        case BSP_PERIPH_ID_SPI5:
//        case BSP_PERIPH_ID_SPI6:
//        case BSP_PERIPH_ID_SAI1:
//        case BSP_PERIPH_ID_LTDC:
//             clk_div = (BSP_PERIPH_REG_RCC_CFGR & 0x000E000) >> 13;
//             if (clk_div < 4) {
//                 return (clk_freq);
//             }
//             clk_div   = (clk_div - 3) << 1;
//             clk_freq /= clk_div;
//             return (clk_freq);
//    }

//    return ((CPU_INT32U)0);
//}


/*
*********************************************************************************************************
*                                             BSP_PeriphEn()
*
* Description : Enable clock for peripheral.
*
* Argument(s) : pwr_clk_id      Power/clock ID.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

//void  BSP_PeriphEn (CPU_DATA  pwr_clk_id)
//{
//    switch (pwr_clk_id) {
//        case BSP_PERIPH_ID_GPIOA:
//        case BSP_PERIPH_ID_GPIOB:
//        case BSP_PERIPH_ID_GPIOC:
//        case BSP_PERIPH_ID_GPIOD:
//        case BSP_PERIPH_ID_GPIOE:
//        case BSP_PERIPH_ID_GPIOF:
//        case BSP_PERIPH_ID_GPIOG:
//        case BSP_PERIPH_ID_GPIOH:
//        case BSP_PERIPH_ID_GPIOI:
//        case BSP_PERIPH_ID_GPIOJ:
//        case BSP_PERIPH_ID_GPIOK:
//        case BSP_PERIPH_ID_CRC:
//        case BSP_PERIPH_ID_BKPSRAM:
//        case BSP_PERIPH_ID_CCMDATARAM:
//        case BSP_PERIPH_ID_DMA1:
//        case BSP_PERIPH_ID_DMA2:
//        case BSP_PERIPH_ID_DMA2D:
//        case BSP_PERIPH_ID_ETHMAC:
//        case BSP_PERIPH_ID_ETHMACPTP:
//        case BSP_PERIPH_ID_ETHMACTX:
//        case BSP_PERIPH_ID_ETHMACRX:
//        case BSP_PERIPH_ID_OTGHS:
//        case BSP_PERIPH_ID_OTGHSULPI:
//             BSP_PERIPH_REG_RCC_AHB1ENR |= DEF_BIT(pwr_clk_id);
//             break;

//        case BSP_PERIPH_ID_DCMI:
//        case BSP_PERIPH_ID_CRYP:
//        case BSP_PERIPH_ID_HASH:
//        case BSP_PERIPH_ID_RNG:
//        case BSP_PERIPH_ID_OTGFS:
//             BSP_PERIPH_REG_RCC_AHB2ENR |= DEF_BIT(pwr_clk_id - 32);
//             break;

//        case BSP_PERIPH_ID_FSMC:
//             BSP_PERIPH_REG_RCC_AHB3ENR |= DEF_BIT(pwr_clk_id - 64);
//             break;

//        case BSP_PERIPH_ID_TIM2:
//        case BSP_PERIPH_ID_TIM3:
//        case BSP_PERIPH_ID_TIM4:
//        case BSP_PERIPH_ID_TIM5:
//        case BSP_PERIPH_ID_TIM6:
//        case BSP_PERIPH_ID_TIM7:
//        case BSP_PERIPH_ID_TIM12:
//        case BSP_PERIPH_ID_TIM13:
//        case BSP_PERIPH_ID_TIM14:
//        case BSP_PERIPH_ID_WWDG:
//        case BSP_PERIPH_ID_SPI2:
//        case BSP_PERIPH_ID_SPI3:
//        case BSP_PERIPH_ID_USART2:
//        case BSP_PERIPH_ID_USART3:
//        case BSP_PERIPH_ID_UART4:
//        case BSP_PERIPH_ID_UART5:
//        case BSP_PERIPH_ID_I2C1:
//        case BSP_PERIPH_ID_I2C2:
//        case BSP_PERIPH_ID_I2C3:
//        case BSP_PERIPH_ID_CAN1:
//        case BSP_PERIPH_ID_CAN2:
//        case BSP_PERIPH_ID_PWR:
//        case BSP_PERIPH_ID_DAC:
//        case BSP_PERIPH_ID_UART7:
//        case BSP_PERIPH_ID_UART8:
//             BSP_PERIPH_REG_RCC_APB1ENR |= DEF_BIT(pwr_clk_id - 96);
//             break;

//        case BSP_PERIPH_ID_TIM1:
//        case BSP_PERIPH_ID_TIM8:
//        case BSP_PERIPH_ID_USART1:
//        case BSP_PERIPH_ID_USART6:
//        case BSP_PERIPH_ID_ADC1:
//        case BSP_PERIPH_ID_ADC2:
//        case BSP_PERIPH_ID_ADC3:
//        case BSP_PERIPH_ID_SDIO:
//        case BSP_PERIPH_ID_SPI1:
//        case BSP_PERIPH_ID_SPI4:
//        case BSP_PERIPH_ID_SYSCFG:
//        case BSP_PERIPH_ID_TIM9:
//        case BSP_PERIPH_ID_TIM10:
//        case BSP_PERIPH_ID_TIM11:
//        case BSP_PERIPH_ID_SPI5:
//        case BSP_PERIPH_ID_SPI6:
//        case BSP_PERIPH_ID_SAI1:
//        case BSP_PERIPH_ID_LTDC:
//             BSP_PERIPH_REG_RCC_APB2ENR |= DEF_BIT(pwr_clk_id - 128);
//             break;
//    }
//}


/*
*********************************************************************************************************
*                                             BSP_PeriphDis()
*
* Description : Disable clock for peripheral.
*
* Argument(s) : pwr_clk_id      Power/clock ID.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

//void  BSP_PeriphDis (CPU_DATA  pwr_clk_id)
//{
//    switch (pwr_clk_id) {
//        case BSP_PERIPH_ID_GPIOA:
//        case BSP_PERIPH_ID_GPIOB:
//        case BSP_PERIPH_ID_GPIOC:
//        case BSP_PERIPH_ID_GPIOD:
//        case BSP_PERIPH_ID_GPIOE:
//        case BSP_PERIPH_ID_GPIOF:
//        case BSP_PERIPH_ID_GPIOG:
//        case BSP_PERIPH_ID_GPIOH:
//        case BSP_PERIPH_ID_GPIOI:
//        case BSP_PERIPH_ID_GPIOJ:
//        case BSP_PERIPH_ID_GPIOK:
//        case BSP_PERIPH_ID_CRC:
//        case BSP_PERIPH_ID_BKPSRAM:
//        case BSP_PERIPH_ID_CCMDATARAM:
//        case BSP_PERIPH_ID_DMA1:
//        case BSP_PERIPH_ID_DMA2:
//        case BSP_PERIPH_ID_DMA2D:
//        case BSP_PERIPH_ID_ETHMAC:
//        case BSP_PERIPH_ID_ETHMACPTP:
//        case BSP_PERIPH_ID_ETHMACTX:
//        case BSP_PERIPH_ID_ETHMACRX:
//        case BSP_PERIPH_ID_OTGHS:
//        case BSP_PERIPH_ID_OTGHSULPI:
//             BSP_PERIPH_REG_RCC_AHB1ENR &= ~DEF_BIT(pwr_clk_id);
//             break;


//        case BSP_PERIPH_ID_DCMI:
//        case BSP_PERIPH_ID_CRYP:
//        case BSP_PERIPH_ID_HASH:
//        case BSP_PERIPH_ID_RNG:
//        case BSP_PERIPH_ID_OTGFS:
//             BSP_PERIPH_REG_RCC_AHB2ENR &= ~DEF_BIT(pwr_clk_id - 32);
//             break;

//        case BSP_PERIPH_ID_FSMC:
//             BSP_PERIPH_REG_RCC_AHB3ENR &= ~DEF_BIT(pwr_clk_id - 64);
//             break;


//        case BSP_PERIPH_ID_TIM2:
//        case BSP_PERIPH_ID_TIM3:
//        case BSP_PERIPH_ID_TIM4:
//        case BSP_PERIPH_ID_TIM5:
//        case BSP_PERIPH_ID_TIM6:
//        case BSP_PERIPH_ID_TIM7:
//        case BSP_PERIPH_ID_TIM12:
//        case BSP_PERIPH_ID_TIM13:
//        case BSP_PERIPH_ID_TIM14:
//        case BSP_PERIPH_ID_WWDG:
//        case BSP_PERIPH_ID_SPI2:
//        case BSP_PERIPH_ID_SPI3:
//        case BSP_PERIPH_ID_USART2:
//        case BSP_PERIPH_ID_USART3:
//        case BSP_PERIPH_ID_UART4:
//        case BSP_PERIPH_ID_UART5:
//        case BSP_PERIPH_ID_I2C1:
//        case BSP_PERIPH_ID_I2C2:
//        case BSP_PERIPH_ID_I2C3:
//        case BSP_PERIPH_ID_CAN1:
//        case BSP_PERIPH_ID_CAN2:
//        case BSP_PERIPH_ID_PWR:
//        case BSP_PERIPH_ID_DAC:
//        case BSP_PERIPH_ID_UART7:
//        case BSP_PERIPH_ID_UART8:
//             BSP_PERIPH_REG_RCC_APB1ENR &= ~DEF_BIT(pwr_clk_id - 96);
//             break;


//        case BSP_PERIPH_ID_TIM1:
//        case BSP_PERIPH_ID_TIM8:
//        case BSP_PERIPH_ID_USART1:
//        case BSP_PERIPH_ID_USART6:
//        case BSP_PERIPH_ID_ADC1:
//        case BSP_PERIPH_ID_ADC2:
//        case BSP_PERIPH_ID_ADC3:
//        case BSP_PERIPH_ID_SDIO:
//        case BSP_PERIPH_ID_SPI1:
//        case BSP_PERIPH_ID_SPI4:
//        case BSP_PERIPH_ID_SYSCFG:
//        case BSP_PERIPH_ID_TIM9:
//        case BSP_PERIPH_ID_TIM10:
//        case BSP_PERIPH_ID_TIM11:
//        case BSP_PERIPH_ID_SPI5:
//        case BSP_PERIPH_ID_SPI6:
//        case BSP_PERIPH_ID_SAI1:
//        case BSP_PERIPH_ID_LTDC:
//             BSP_PERIPH_REG_RCC_APB2ENR &= ~DEF_BIT(pwr_clk_id - 128);
//             break;
//    }
//}
