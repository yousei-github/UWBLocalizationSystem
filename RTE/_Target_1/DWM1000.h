#ifndef DWM1000_H_
#define DWM1000_H_

#ifdef __cplusplus
extern "C" {
#endif

#include ".\decadriver\deca_device_api.h"
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
#define TAG_FUNCTION 1
#define ANCHOR_FUNCTION 2
#define USE_TXRX_STATES_LED 1
#define USE_TXRX_LED 1
#define TWR_USE_INTERRUPT_OR_LOOP 0 // 1 refer to use interrupt function (there still have some problems), 0 is using loop function
#define FEEDBACK_DISTANCE 1 // 1 refer to tranfer distance measurement from anchor to tag, 0 is disabling this functionality
#define RX_BUF_LEN 50
/* remember that you also need to define multiple DWM1000_module_t manually */
#define EN_MULTIPLE_MODULES 0
/* 0 represents using multiple modules, TAG_FUNCTION represents tag, ANCHOR_FUNCTION represents anchor */
#define EN_SINGLE_MODULE (TAG_FUNCTION) 
/* Two way ranging mission period, in milliseconds. */
#define TWR_MISSION_MS (5u)
#define TWR_WAITMISSION_MS (TWR_MISSION_MS)
#define TWR_FINISHMISSION_DELAY_MS ((TWR_WAITMISSION_MS) * (POSITIONING_ANCHOR_NUM - 1) + TWR_RUN_MS * (POSITIONING_ANCHOR_NUM - 2))
/* Note: remember to update the definition DWT_NUM_DW_DEV if the number of DWM1000 has changed, 
 * below definition means that how many nodes you want your module to communicate */
#define MAX_NODE_NUM 10    
#define POSITIONING_ANCHOR_NUM (4)
#if (MAX_NODE_NUM > UINT16_MAX)
#error "Too many nodes!"
#endif // (MAX_NODE_NUM > UINT16_MAX)

/* error output*/
#if (EN_MULTIPLE_MODULES>0)&&(EN_SINGLE_MODULE>0)
#undef EN_MULTIPLE_MODULES
#undef EN_SINGLE_MODULE
#define EN_MULTIPLE_MODULES 0
#define EN_SINGLE_MODULE TAG_FUNCTION 
#error "module definition error!"
#elif (EN_MULTIPLE_MODULES==0)&&(EN_SINGLE_MODULE==0)
#define EN_MULTIPLE_MODULES 0
#define EN_SINGLE_MODULE TAG_FUNCTION 
#error "must define one module!"
#endif // (EN_MULTIPLE_MODULES>0)&&(EN_SINGLE_MODULE>0)

#if EN_MULTIPLE_MODULES
#define EN_TAG 1
#define EN_ANCHOR 1
#endif // EN_MULTIPLE_MODULES

#if (EN_SINGLE_MODULE==TAG_FUNCTION)
#define EN_TAG 1
#define EN_ANCHOR 0
#elif (EN_SINGLE_MODULE==ANCHOR_FUNCTION)
#define EN_TAG 0
#define EN_ANCHOR 1
#elif ((EN_SINGLE_MODULE!=0)&&(EN_SINGLE_MODULE!=TAG_FUNCTION)&&(EN_SINGLE_MODULE!=ANCHOR_FUNCTION))
#define EN_TAG 1
#define EN_ANCHOR 0
#error "wrong parameter for EN_SINGLE_MODULE!"
#endif // EN_SINGLE_MODULE

/****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/
/* interrupt service routine definitions, need user to alter or increase them manually, 
 * and user may also need to change the DWM1000_EXTI_Callback(uint16_t GPIO_Pin) function */
#define DWM1000_ISR1 DWM1000_tag_isr
#define DWM1000_ISR2 DWM1000_anchor_isr
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
	Anchor_module = 0,
	Tag_module = 1,
	Communication_module
}DWM1000_type;


/* see NOTES 1 below */
typedef enum
{
	Initialization = 0,
	Update_comnodes,
	Schedule,
	Poll_msg,
	Response_msg,
	Final_msg,
	Wait_isr_1th,
	Wait_isr_2nd,
	Error
}DWM1000_status;

typedef struct
{
	float x;
	float y;
	float z;
}Node_Cartesian_coordinate;

typedef struct 
{
	uint16 module_nb;    // use for Decawave deriver
	uint16 module_srcaddress;    // Source address
	uint16 communication_nb;    // refer to Destination address: the number of communication nodes
	uint16 module_desaddress[MAX_NODE_NUM];    // Destination address (indexed by current_node)
	/* Destination address enable (TRUE refer to enable relevant anchor, FALSE represents disable correspond anchor,
	 * indexed by BYTE0(module_desaddress[module.current_node])- '0' ) */
	uint8  module_des_en[MAX_NODE_NUM];    
	/* communication error count (refer to UWB_MAXLOST_T) */
	uint16 lost_count[MAX_NODE_NUM];
	uint8* tx_poll_msg;
	uint8* rx_poll_msg;
	uint8* tx_resp_msg;
	uint8* rx_resp_msg;
	uint8* tx_final_msg;
	uint8* rx_final_msg;
	uint8* feedback_msg;
	uint8* tx_broadcast_msg;
	uint8* rx_broadcast_msg;
	uint8 current_node;    // current communicating node
	uint8 frame_seq_nb;
	uint8 rx_buffer[RX_BUF_LEN];
	uint8 module_type;    // DM1000_type
	uint8 module_status;    // refer to DWM1000_status
	uint8 pre_timeout;
	uint16 poll_tx_to_resp_rx_dly_uus;
	uint16 poll_rx_to_resp_tx_dly_uus;
	uint16 resp_tx_to_final_rx_dly_uus;
	uint16 resp_rx_to_final_tx_dly_uus;
	uint16 resp_rx_timeout_uus;
	uint16 final_rx_timeout_uus;
	uint32 tx_ant_dly;
	uint32 rx_ant_dly;
	/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
	uint32 status_reg;
	/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
	double tof;
	double distance;
	Node_Cartesian_coordinate coord;
	dwt_config_t config;
	dwt_txconfig_t txconfig;
}DWM1000_module_t;

typedef struct
{
	DWM1000_module_t* module;
}Callback_module_t;
/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
extern DWM1000_module_t initiator, responder;



/****************************************************************************//**
 *
 *         Function prototypes
 *
 *******************************************************************************/
uint8 DWM1000_initialization(void);

uint8 DWM1000_run_tag(DWM1000_module_t * _module);
uint8 DWM1000_run_anchor(DWM1000_module_t* _module);

uint8 DWM1000_update_communicationnodes(DWM1000_module_t* _module);




#ifdef __cplusplus
}
#endif


/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. Tage normal runing status is (isr is Wait_isr status)
 *
 *                                                                                +-------+
 *                                                                        +-------| Error |<-----------------+
 *                                                                        |       +-------+   |              |
 *                                                                        v                   |              |
 *    +----------------+     +-----------------+      +----------+      +----------+ isr +--------------+    |
 *    | Initialization |---->| Update_comnodes |<---->| Schedule |<---->| Poll_msg |---->| Response_msg |    |
 *    +----------------+     +-----------------+      +----------+      +----------+     +--------------+    |
 *                                                                        ^                   |              |
 *                                                                    isr |   +-----------+   |              |
 *                                                                        +---| Final_msg |<--+              |
 *                                                                        |   +-----------+                  |
 *                                                                        |                                  |
 *                                                                        +----------------------------------+
 * 2. Anchor normal runing status is
 *
 *                                                                                +-------+
 *                                                                        +-------| Error |<-----------------+
 *                                                                        |       +-------+   |              |
 *                                                                        v                   |              |
 *    +----------------+     +-----------------+      +----------+      +--------------+ isr +----------+    |
 *    | Initialization |---->| Update_comnodes |<---->| Schedule |<---->| Response_msg |---->| Poll_msg |    |
 *    +----------------+     +-----------------+      +----------+      +--------------+     +----------+    |
 *                                                                        ^                   |              |
 *                                                                        |   +-----------+   | isr          |
 *                                                                        +---| Final_msg |<--+              |
 *                                                                        |   +-----------+                  |
 *                                                                        |                                  |
 *                                                                        +----------------------------------+
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ****************************************************************************************************************************************************/


#endif /* DWM1000_PERIPHERAL_H_ */


