/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
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
*                                            EXAMPLE CODE
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : YS
*                 DC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <UCOS_includes.h>
#include "DWM1000.h"
#include "led.h"
#include "spl06_001.h"
#include "Position_algorithm.h"
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/




/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/* Thread 1 mission */ 
static  OS_TCB   Thread_1_TCB;
static  CPU_STK  Thread_1_Stk[Thread_1_STK_SIZE];
/* Thread 2 mission */ 
static  OS_TCB   Thread_2_TCB;
static  CPU_STK  Thread_2_Stk[Thread_2_STK_SIZE];
/* Thread 3 mission */
static  OS_TCB   Thread_3_TCB;
static  CPU_STK  Thread_3_Stk[Thread_3_STK_SIZE];

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void Thread_1 (void *p_arg);          
static void Thread_2 (void *p_arg);   
static void Thread_3 (void *p_arg);



uint8 uCOS_III_initialization(void)
{
	OS_ERR  err;

    BSP_IntDisAll();                                            /* Disable all interrupts.                              */
    
    CPU_Init();                                                 /* Initialize the uC/CPU Services                       */
    Mem_Init();                                                 /* Initialize Memory Management Module                  */
    Math_Init();                                                /* Initialize Mathematical Module                       */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */	
	
	if(err != OS_ERR_NONE)
	{
		return DEF_FAIL;
	}
	

	HAL_SuspendTick();
//	BSP_Init();                                                 /* Initialize BSP functions                             */
	BSP_Tick_Init();                                            /* Initialize Tick Services.                            */
	
	
	return DEF_OK;
}

uint8 uCOS_III_preparemission(void)
{
	OS_ERR  os_err;
	
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&os_err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif
	
	/* Do not initialize peripheral here which need delay function! */
	/* Please create missions here*/
	OSTaskCreate((OS_TCB      *)&Thread_1_TCB,
				(CPU_CHAR    *)"Thread 1",
				(OS_TASK_PTR  ) Thread_1, 
				(void        *) 0,
				(OS_PRIO      ) TASK_THREAD1_PRIO,
				(CPU_STK     *)&Thread_1_Stk[0],
				(CPU_STK_SIZE ) Thread_1_Stk[Thread_1_STK_SIZE / 10u],
				(CPU_STK_SIZE ) Thread_1_STK_SIZE,
				(OS_MSG_QTY   ) 0u,
				(OS_TICK      ) 0u,
				(void        *) 0,
				(OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
				(OS_ERR      *)&os_err);
	OSTaskCreate((OS_TCB      *)&Thread_2_TCB,
                (CPU_CHAR    *)"Thread 2",
                (OS_TASK_PTR  ) Thread_2, 
                (void        *) 0,
                (OS_PRIO      ) TASK_THREAD2_PRIO,
                (CPU_STK     *)&Thread_2_Stk[0],
                (CPU_STK_SIZE ) Thread_2_Stk[Thread_2_STK_SIZE / 10u],
                (CPU_STK_SIZE ) Thread_2_STK_SIZE,
                (OS_MSG_QTY   ) 0u,
                (OS_TICK      ) 0u,
                (void        *) 0,
                (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
                (OS_ERR      *)&os_err);
	OSTaskCreate((OS_TCB      *)&Thread_3_TCB,
				(CPU_CHAR    *)"Thread 3",
				(OS_TASK_PTR)Thread_3,
				(void        *)0,
				(OS_PRIO)TASK_THREAD3_PRIO,
				(CPU_STK     *)&Thread_3_Stk[0],
				(CPU_STK_SIZE)Thread_3_Stk[Thread_3_STK_SIZE / 10u],
				(CPU_STK_SIZE)Thread_3_STK_SIZE,
				(OS_MSG_QTY)0u,
				(OS_TICK)0u,
				(void        *)0,
				(OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
				(OS_ERR      *)&os_err);

	return DEF_OK;
}

uint8 uCOS_III_run(void)
{
	OS_ERR  err;
	
	OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
	
	/* OSStart() is not supposed to return.  If it does, that would be considered a fatal error */
	return DEF_FAIL;
}

static void Thread_1 (void *p_arg)
{
	OS_ERR  err;

	(void)p_arg;
	Led_initialization();
	/* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) 
	{                                          

		LED1_ON;
        OSTimeDlyHMSM(0u, 0u, 0u, 500u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
		LED1_OFF;
		OSTimeDlyHMSM(0u, 0u, 0u, 500u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

    }
}

static void Thread_2 (void *p_arg)
{
	OS_ERR  err;

	(void)p_arg;
	DWM1000_initialization();
	//Drv_Spl0601_Init();
	/* Task body, always written as an infinite loop.       */
    while (DEF_TRUE) 
	{                                          
#if EN_ANCHOR
		DWM1000_run_anchor(&responder);
#endif
#if EN_TAG
		DWM1000_run_tag(&initiator);
		OSTimeDlyHMSM(0u, 0u, 0u, TWR_MISSION_MS, OS_OPT_TIME_HMSM_STRICT, &err);
#endif
    }
}

static void Thread_3(void *p_arg)
{
	OS_ERR  err;
	(void)p_arg;

	Positioning_initialization();

	while (DEF_TRUE)
	{
		ThreeD_trilaterationpositioning_run();

		OSTimeDlyHMSM(0u, 0u, 0u, 50u, OS_OPT_TIME_HMSM_STRICT, &err);
	}
}

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

//int main(void)
//{
//    OS_ERR  err;


//    BSP_IntDisAll();                                            /* Disable all interrupts.                              */
//    
//    CPU_Init();                                                 /* Initialize the uC/CPU Services                       */
//    Mem_Init();                                                 /* Initialize Memory Management Module                  */
//    Math_Init();                                                /* Initialize Mathematical Module                       */

//    OSInit(&err);                                               /* Init uC/OS-III.                                      */

//    OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
//                 (CPU_CHAR     *)"App Task Start",
//                 (OS_TASK_PTR   )AppTaskStart,
//                 (void         *)0u,
//                 (OS_PRIO       )APP_CFG_TASK_START_PRIO,
//                 (CPU_STK      *)&AppTaskStartStk[0u],
//                 (CPU_STK_SIZE  )AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
//                 (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
//                 (OS_MSG_QTY    )0u,
//                 (OS_TICK       )0u,
//                 (void         *)0u,
//                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//                 (OS_ERR       *)&err);

//    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

//    (void)&err;

//    return (0u);
//}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

//static  void  AppTaskStart (void *p_arg)
//{
//    OS_ERR  err;


//   (void)p_arg;

//    BSP_Init();                                                 /* Initialize BSP functions                             */
//    BSP_Tick_Init();                                            /* Initialize Tick Services.                            */


//#if OS_CFG_STAT_TASK_EN > 0u
//    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
//#endif

//#ifdef CPU_CFG_INT_DIS_MEAS_EN
//    CPU_IntDisMeasMaxCurReset();
//#endif

//    BSP_LED_Off(0u);                                            /* Turn Off LEDs after initialization                   */

//    APP_TRACE_DBG(("Creating Application Kernel Objects\n\r"));
//    AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

//    APP_TRACE_DBG(("Creating Application Tasks\n\r"));
//    AppTaskCreate();                                            /* Create Application tasks                             */


//    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */

//        BSP_LED_Toggle(1u);
//        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
//                      OS_OPT_TIME_HMSM_STRICT,
//                      &err);

//        BSP_LED_Toggle(2u);
//        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
//                      OS_OPT_TIME_HMSM_STRICT,
//                      &err);

//        BSP_LED_Toggle(3u);
//        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
//                      OS_OPT_TIME_HMSM_STRICT,
//                      &err);

//        BSP_LED_Toggle(4u);
//        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
//                      OS_OPT_TIME_HMSM_STRICT,
//                      &err);

//    }
//}


/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create application tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

//static  void  AppTaskCreate (void)
//{
//    OS_ERR  os_err;
//    
//                                                                /* ------------- CREATE FLOATING POINT TASK ----------- */
//    OSTaskCreate((OS_TCB      *)&App_TaskEq0FpTCB,
//                 (CPU_CHAR    *)"FP  Equation 1",
//                 (OS_TASK_PTR  ) App_TaskEq0Fp, 
//                 (void        *) 0,
//                 (OS_PRIO      ) APP_CFG_TASK_EQ_PRIO,
//                 (CPU_STK     *)&App_TaskEq0FpStk[0],
//                 (CPU_STK_SIZE ) App_TaskEq0FpStk[APP_CFG_TASK_EQ_STK_SIZE / 10u],
//                 (CPU_STK_SIZE ) APP_CFG_TASK_EQ_STK_SIZE,
//                 (OS_MSG_QTY   ) 0u,
//                 (OS_TICK      ) 0u,
//                 (void        *) 0,
//                 (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
//                 (OS_ERR      *)&os_err);
//}


/*
*********************************************************************************************************
*                                          AppObjCreate()
*
* Description : Create application kernel objects tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

//static  void  AppObjCreate (void)
//{

//}


/*
*********************************************************************************************************
*                                             App_TaskEq0Fp()
*
* Description : This task finds the root of the following equation.
*               f(x) =  e^-x(3.2 sin(x) - 0.5 cos(x)) using the bisection mehtod
*
* Argument(s) : p_arg   is the argument passed to 'App_TaskEq0Fp' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

//void  App_TaskEq0Fp (void  *p_arg)
//{
//    CPU_FP32    a;
//    CPU_FP32    b;
//    CPU_FP32    c;
//    CPU_FP32    eps;
//    CPU_FP32    f_a;
//    CPU_FP32    f_c;
//    CPU_FP32    delta;
//    CPU_INT08U  iteration;
//    RAND_NBR    wait_cycles;
//        
//    
//    while (DEF_TRUE) {
//        eps       = 0.00001;
//        a         = 3.0; 
//        b         = 4.0;
//        delta     = a - b;
//        iteration = 0u;
//        if (delta < 0) {
//            delta = delta * -1.0;
//        }
//        
//        while (((2.00 * eps) < delta) || 
//               (iteration    > 20u  )) {
//            c   = (a + b) / 2.00;
//            f_a = (exp((-1.0) * a) * (3.2 * sin(a) - 0.5 * cos(a)));
//            f_c = (exp((-1.0) * c) * (3.2 * sin(c) - 0.5 * cos(c)));
//            
//            if (((f_a > 0.0) && (f_c < 0.0)) || 
//                ((f_a < 0.0) && (f_c > 0.0))) {
//                b = c;
//            } else if (((f_a > 0.0) && (f_c > 0.0)) || 
//                       ((f_a < 0.0) && (f_c < 0.0))) {
//                a = c;           
//            } else {
//                break;
//            }
//                
//            delta = a - b;
//            if (delta < 0) {
//               delta = delta * -1.0;
//            }
//            iteration++;

//            wait_cycles = Math_Rand();
//            wait_cycles = wait_cycles % 1000;

//            while (wait_cycles > 0u) {
//                wait_cycles--;
//            }

//            if (iteration > APP_TASK_EQ_0_ITERATION_NBR) {
//                APP_TRACE_INFO(("App_TaskEq0Fp() max # iteration reached\n"));
//                break;
//            }            
//        }

//        APP_TRACE_INFO(("Eq0 Task Running ....\n"));
//        
//        if (iteration == APP_TASK_EQ_0_ITERATION_NBR) {
//            APP_TRACE_INFO(("Root = %f; f(c) = %f; #iterations : %d\n", c, f_c, iteration));
//        }
//    }
//}
