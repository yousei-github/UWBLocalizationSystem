#include "DWM1000.h"
#include ".\decadriver\deca_device_api.h"
#include ".\decadriver\deca_regs.h"
#include ".\platform\deca_spi.h"
#include ".\platform\DWM1000_peripheral.h"
#include "Position_algorithm.h"

/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
#if EN_TAG
DWM1000_module_t initiator;
#endif
#if EN_ANCHOR
DWM1000_module_t responder;
#endif
/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/
/* DWM1000 default anchor source address */
#define UWB_DEFAULTANCHOR_SRCADDR "Y0"
/* DWM1000 default tag source address */
#define UWB_DEFAULTTAG_SRCADDR "S0"
/* DWM1000 default tag broadcast anchor address */
#define UWB_DEFAULTANCHOR_BROADCAST_LADDR '0'
#define UWB_DEFAULTANCHOR_BROADCAST_HADDR '0'
/* UWM channel (see RELATED PARAMETERS below) */
#define UWB_CHANNEL_IS (7)
/* UWM Pulse repetition frequency (see RELATED PARAMETERS below) */
#define UWB_PRF (DWT_PRF_64M)
/* UWB SFD (0 to use standard SFD, 1 to use non-standard SFD) */
#define UWB_NSFD (1)
/* UWB PHR mode */
#define UWB_PHRMODE DWT_PHRMODE_STD
/* WARNING! must check these tables in RELATED PARAMETERS in order to set correct values below */
#define UWB_PREAMBLECODES (17)
#define UWB_PGDLYVALUE   (0x93)
#define UWB_TXPOWERVALUE (0x5171B1D1)
/* UWB max lost time */
#define UWB_MAXLOST_T (10)
/* DWM1000 status register error */
#define STATUS_REG_ERROR (0xFFFFFFFF)

/* Clear Channel Assessment (CCA) mechanism */
#define CCA_PREAMBLE_TO (3)
/* Inter-frame delay period, in milliseconds.*/
#define TX_DELAY_MS (15)
/* initial backoff period when failed to transmit a frame due to preamble detection */
#define INITIAL_BACKOFF_PERIOD (10) /* This constant would normally smaller (e.g. 1ms), however here it is set to 400 ms so that
									* user can see (on LCD) the report that the CCA detects a preamble on the air occasionally.
									* and is doing a TX back-off.
									*/
#define CCA_TAG_RX_TIMEOUT_UUS (6000)
/* Length of the broadcast message */
#define BROADCAST_MSG_LEN (14)
/* Acknowledge function */
#define ACK_YES (0x01)
#define ACK_NO  (0x02)
/* Length of the feedback message */
#define FEEDBACK_MSG_LEN 32
#define FEEDBACK_MSG_DISTANCE_IDX 10
#define FEEDBACK_MSG_X_IDX 18
#define FEEDBACK_MSG_Y_IDX 22
#define FEEDBACK_MSG_Z_IDX 26
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000
/* Two way ranging period, in milliseconds. */
#define TWR_RUN_MS 2
/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. 63.8976GHz sampling clock (15.65 picsecond) */
/* For DWM 1000, the value is about 16418.744 (Default: 16436) */
#define TX_ANT_DLY 16420    // unit is 15.65ps, i.e. 499.2MHz * 128 (16 bit, max value is 65535 * 15.65ps = 1025.622 us)
#define RX_ANT_DLY 16420    // unit is 15.65ps, i.e. 499.2MHz * 128 (16 bit, max value is 65535 * 15.65ps = 1025.622 us)
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Length of the poll message */
#define POLL_MSG_LEN 12
/* Length of the response message */
#define RESP_MSG_LEN 15
/* Length of the finall message */
#define FINAL_MSG_LEN 24
/* Indexes to access some of the fields in the frames defined below. */
#define FRAME_CONTROL_L_8BIT 0
#define FRAME_CONTROL_H_8BIT 1
#define ALL_MSG_SN_IDX 2
#define PAN_ID_L_8BIT 3
#define PAN_ID_H_8BIT 4
#define DES_ADDRESS_L_8BIT 5
#define DES_ADDRESS_H_8BIT 6
#define SRC_ADDRESS_L_8BIT 7
#define SRC_ADDRESS_H_8BIT 8
#define FUNCTION_CODE 9
#define ACTIVITY_CODE 10
#define ACKNOWLEDGE_CODE 11
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536    //512*128

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
#define RECEIVE_TRAIL_UUS 10
#define MAX_TRANSMITTING_TIME_UUS 130 //110K: 3500 6M8: 130
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
/* The maximum wait-for-response turn-around time is 1075ms (1048576)) */
#define POLL_TX_TO_RESP_RX_DLY_UUS (600)  //unit: 1.0256 us (1 UWB microsecond is 128 system clock cycles (125 MHz), 20 bit so max value is 1048575 * 1.0256 us = 1075418.52 us = 1075.41852 ms) (110K: 300 6M8: 200)
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS (300 * 3)  //unit: 1.0256 us, 32 bit (40 bit, the lower-order 9-bits are be ignored) (110K: 3100+ 6M8: 300*2)
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS (POLL_RX_TO_RESP_TX_DLY_UUS + MAX_TRANSMITTING_TIME_UUS)   //unit: 1.0256 us (512 * 499.2 MHz UWB clock, the maximum RX timeout is 65ms (65536))
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. Max value is 65535 ¡Á (PAC size), a period of over 500 ms for the smallest PAC size */
#define PRE_TIMEOUT 32    //need calculate, which is the preamble length (like 1024) divided by the PAC size (like 32)
#define FINAL_TX_TO_FEED_RX_DLY_UUS (100)
#define FEED_RX_TIMEOUT_UUS (FINAL_TX_TO_FEED_RX_DLY_UUS * 4 + MAX_TRANSMITTING_TIME_UUS) //110K: 3000 + 6M8: FINAL_TX_TO_FEED_RX_DLY_UUS*2

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS (300 * 3)    //unit: 1.0256 us, 32 bit (40 bit, the lower-order 9-bits are be ignored) (110K: *10 6M8: 300*3)
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 600    //unit: 1.0256 us (1 UWB microsecond is 128 system clock cycles (125 MHz)) (110K: 500 6M8: 200)
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS (RESP_RX_TO_FINAL_TX_DLY_UUS + MAX_TRANSMITTING_TIME_UUS)    //unit: 1.0256 us (512 * 499.2 MHz UWB clock, the maximum RX timeout is 65ms (65536))
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */

//#define POLL_RX_TO_RESP_TX_DLY_UUS 2750
//  /* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
///* Receive final timeout. See NOTE 5 below. */
//#define FINAL_RX_TIMEOUT_UUS 3300

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define ISR_EVENTS_USER_RX_TO    (DWT_INT_RFTO | DWT_INT_RXPTO)
#define ISR_EVENTS_USER_RX_ERR   (DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL)
/****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/
#define BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )
#define BYTE4(dwTemp)       ( *( (u8 *)(&dwTemp) + 4) )
#define BYTE5(dwTemp)       ( *( (u8 *)(&dwTemp) + 5) )
#define BYTE6(dwTemp)       ( *( (u8 *)(&dwTemp) + 6) )
#define BYTE7(dwTemp)       ( *( (u8 *)(&dwTemp) + 7) )
/****************************************************************************//**
 *
 *         Private variables and function prototypes
 *
 *******************************************************************************/
/* Frames used in the broadcast process. */
/* Broadcast blink */ 
/*
 *     -byte 0/1: frame control(0x8841 to indicate a data frame using 16-bit addressing).
 *     -byte 2 : sequence number, incremented for each new frame.
 *     -byte 3/4: PAN ID (0xDECA).
 *     -byte 5/6: destination address.
 *     -byte 7/8: source address.
 *     -byte 9: encoding header (0x43 to indicate no extended ID, temperature, or battery status is carried in the message).
 *     -byte 10: EXT header (0x02 to indicate tag is listening for a response immediately after this message, 0x03 to indicate anchor has responded).
 *     -byte 11: acknowledge (0x01 to indicate yes, 0x02 refers to no acknowledge).
 *     -byte 12/13: frame check - sum, automatically set by DW1000.  */
															 /* Bit:  0     1    2   3     4     5    6    7    8    9     10    11  12  13 */
static uint8 tx_broadcast_msg[DWT_NUM_DW_DEV][BROADCAST_MSG_LEN] = { 0x41, 0x88, 0, 0xCA, 0xDE, UWB_DEFAULTANCHOR_BROADCAST_LADDR, UWB_DEFAULTANCHOR_BROADCAST_HADDR, '0', '0', 0x43, 0x02, 0x01, 0, 0 };
static uint8 rx_broadcast_msg[DWT_NUM_DW_DEV][BROADCAST_MSG_LEN] = { 0x41, 0x88, 0, 0xCA, 0xDE, '0', '0', '0', '0', 0x43, 0x03, 0x01, 0, 0 };
/* Frames used in the feedback process. */
/* Feedback blink */
/*
 *     -byte 0/1: frame control(0x8841 to indicate a data frame using 16-bit addressing).
 *     -byte 2 : sequence number, incremented for each new frame.
 *     -byte 3/4: PAN ID (0xDECA).
 *     -byte 5/6: destination address.
 *     -byte 7/8: source address.
 *     -byte 9: function code.
 *     -byte 10 -> 17: distance data
 *     -byte 18 -> 21: x axis data.
 *     -byte 22 -> 25: y axis data.
 *     -byte 26 -> 29: z axis data.
 *     -byte 30/31: frame check - sum, automatically set by DW1000.  */
static uint8 feedback_msg[DWT_NUM_DW_DEV][FEEDBACK_MSG_LEN] = { 0x41, 0x88, 0, 0xCA, 0xDE, '0', '0', '0', '0', 0x24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Frames used in the ranging process. See NOTE 2 below. */
/* Tag blink */                                      /* Bit:  0     1    2   3     4     5    6    7    8    9   10 11 */
static uint8 tx_poll_msg[DWT_NUM_DW_DEV][POLL_MSG_LEN]   = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0 };    //Sender: initiator -> responder (T(SB))
static uint8 rx_poll_msg[DWT_NUM_DW_DEV][POLL_MSG_LEN]   = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0 };    //Checker: responder -> initiator (T(RB))
/* simple ACK response */
static uint8 rx_resp_msg[DWT_NUM_DW_DEV][RESP_MSG_LEN]   = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0 };    //Checker: initiator -> responder (T(RR))
static uint8 tx_resp_msg[DWT_NUM_DW_DEV][RESP_MSG_LEN]   = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0 };    //Sender: responder -> initiator (T(SR))
/* Final Message */
static uint8 tx_final_msg[DWT_NUM_DW_DEV][FINAL_MSG_LEN] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };    //Sender: initiator -> responder (T(SF))
static uint8 rx_final_msg[DWT_NUM_DW_DEV][FINAL_MSG_LEN] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };    //Checker: responder -> initiator (T(RF))

#if TWR_USE_INTERRUPT_OR_LOOP
static Callback_module_t uwb_callback;
#endif // TWR_USE_INTERRUPT_OR_LOOP
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
//static uint32 status_reg = 0;

static uint8 dwm1000_set_module(DWM1000_module_t* _module, uint8 _rate, DWM1000_type _type);
#if EN_TAG
static uint8 dwm1000_tag_initialization(DWM1000_module_t* _module);
#endif
#if EN_ANCHOR
static uint8 dwm1000_anchor_initialization(DWM1000_module_t * _module);
#endif
static uint8 dwm1000_error_commonprocess(DWM1000_module_t* _module);
static uint8 dwm1000_load_address(uint8 * _msg, uint16 _address);
static void dwt1000_txrx_statexled(uint8 _en);

#if TWR_USE_INTERRUPT_OR_LOOP
/* Callback functions for dwt_isr() */
static void dwm1000_callback_txdone(const dwt_cb_data_t * _data);
static void dwm1000_callback_rxok(const dwt_cb_data_t * _data);
static void dwm1000_callback_rxto(const dwt_cb_data_t * _data);
static void dwm1000_callback_rxerr(const dwt_cb_data_t * _data);
#endif // TWR_USE_INTERRUPT_OR_LOOP

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
#if EN_TAG
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
#endif
#if EN_ANCHOR
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
#endif
#if FEEDBACK_DISTANCE
#if EN_TAG
static double get_frame_f64(uint8 * _pointer);
static float get_frame_f32(uint8 * _pointer);
#endif
#if EN_ANCHOR
static void send_frame_f64(uint8 * _pointer, double _data);
static void send_frame_f32(uint8 * _pointer, float _data);
#endif
#endif
#if EN_TAG
static uint8 arrange_address(uint16 * _pointer, uint16 _avail_num);
#endif
/****************************************************************************//**
 *
 *         DWM1000 function section
 *
 *******************************************************************************/



uint8 DWM1000_initialization(void)
{
	uint8 state = TRUE;
	Sleep(10);
#if EN_TAG
	//dwm1000_set_module(&initiator, DWT_BR_110K, Tag_module);
	dwm1000_set_module(&initiator, DWT_BR_6M8, Tag_module);
	setup_DW1000RSTnIRQ(0, initiator.module_nb);
	state = dwm1000_tag_initialization(&initiator);
	
	if(state == FALSE )
	{ 
		state = dwm1000_tag_initialization(&initiator);
	}
	
	DWM1000_update_communicationnodes(&initiator);
#endif // EN_TAG

#if EN_ANCHOR
	//dwm1000_set_module(&responder, DWT_BR_110K, Anchor_module);
	dwm1000_set_module(&responder, DWT_BR_6M8, Anchor_module);
	setup_DW1000RSTnIRQ(0, responder.module_nb);
	state = dwm1000_anchor_initialization(&responder);
	
	if(state == FALSE )
	{ 
		state = dwm1000_anchor_initialization(&responder);
	}
	
	DWM1000_update_communicationnodes(&responder);
#endif // EN_ANCHOR

#if USE_TXRX_STATES_LED
	dwt1000_txrx_statexled(TRUE);
#endif // USE_TXRX_STATES_LED

#if USE_TXRX_LED
	dwt_setleds(DWT_LEDS_INIT_BLINK);
	Sleep(1000);
	dwt_setleds(DWT_LEDS_ENABLE);
#endif // USE_TXRX_LED

#if TWR_USE_INTERRUPT_OR_LOOP
	dwt_setcallbacks(dwm1000_callback_txdone, dwm1000_callback_rxok, dwm1000_callback_rxto, dwm1000_callback_rxerr);
#endif // TWR_USE_INTERRUPT_OR_LOOP

	return state;
}

#if EN_TAG
uint8 DWM1000_run_tag(DWM1000_module_t* _module)
{
#define module (*_module)
	/* Handle multiple DW1000 devices by using an array of those structures, as set by the #define of the DWT_NUM_DW_DEV pre-processor symbol. */
	dwt_setlocaldataptr(module.module_nb);

	//while (TRUE)
	//{
		if (module.module_status == Schedule)
		{
			if (module.lost_count[module.current_node] >= UWB_MAXLOST_T)
			{
				uint8 temp_num;
				/* get this anchor's number */
				temp_num = BYTE0(module.module_desaddress[module.current_node]) - '0';
				/* delete current anchor */
				module.module_des_en[temp_num] = FALSE;
				module.module_desaddress[module.current_node] = 0;
				module.lost_count[module.current_node] = 0;
				module.communication_nb--;
				if (module.communication_nb == 0)
				{
					module.current_node = 0;
					DWM1000_update_communicationnodes(&module);
					module.current_node = module.communication_nb - 1;
				}
				else
				{
					arrange_address((uint16*)&module.module_desaddress, module.communication_nb);
					module.current_node = module.communication_nb - 1 + 1;
					DWM1000_update_communicationnodes(&module);
				}
			}

			/* Add destination address for tx poll massage */
			dwm1000_load_address(&(module.tx_poll_msg[DES_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
			/* Add source address for rx response massage */
			dwm1000_load_address(&(module.rx_resp_msg[SRC_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
			/* Add destination address for tx final massage */
			dwm1000_load_address(&(module.tx_final_msg[DES_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
#if FEEDBACK_DISTANCE
			/* Add source address for feedback massage */
			dwm1000_load_address(&(module.feedback_msg[SRC_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
#endif
			/* success */
			module.module_status = Poll_msg;
		}
		if (module.module_status == Poll_msg)
		{
			/* Execute a delay between ranging exchanges. */
			//Sleep(RNG_DELAY_MS);

			/* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
			 * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
			dwt_setrxaftertxdelay(module.poll_tx_to_resp_rx_dly_uus);
			dwt_setrxtimeout(module.resp_rx_timeout_uus);
			//	dwt_setpreambledetecttimeout(module.pre_timeout);

			/* Write frame data to DW1000 and prepare transmission. See NOTE 10 below. */
			module.tx_poll_msg[ALL_MSG_SN_IDX] = module.frame_seq_nb;
			dwt_writetxdata(POLL_MSG_LEN, module.tx_poll_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(POLL_MSG_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

			/* wait for the dwm1000 triggers the function dwt_isr() */
			module.module_status = Wait_isr_1th;

#if TWR_USE_INTERRUPT_OR_LOOP
			dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO, 1);
#endif // TWR_USE_INTERRUPT_OR_LOOP

			/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
			 * set by dwt_setrxaftertxdelay() has elapsed. */
			dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);	//timestamp: T(SB)

#if !TWR_USE_INTERRUPT_OR_LOOP
		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
			while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));
			if (module.status_reg & SYS_STATUS_RXFCG)
			{
				module.module_status = Response_msg;
				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
			}
			else
			{
				module.module_status = Error;
			}
#endif 
			/* Clear */
			dwt_setrxaftertxdelay(0);
			dwt_setrxtimeout(0);
		}
		if (module.module_status == Response_msg)
		{
			uint32 frame_len;
			/* Increment frame sequence number after transmission of the poll message (modulo 256). */
			module.frame_seq_nb++;

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
			if (frame_len <= RX_BUF_LEN)
			{
				dwt_readrxdata(module.rx_buffer, frame_len, 0);
			}

			/* Check that the frame is the expected response from the companion "DS TWR responder" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			module.rx_buffer[ALL_MSG_SN_IDX] = 0;

			if (memcmp(module.rx_buffer, module.rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				/* success */
				module.module_status = Final_msg;
			}
			else
				module.module_status = Error;
		}
		if (module.module_status == Final_msg)
		{
			uint32 final_tx_time;
			int ret;
			uint64 poll_tx_ts;
			uint64 resp_rx_ts;
			uint64 final_tx_ts;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts = get_tx_timestamp_u64();    //timestamp: T(SB)
			resp_rx_ts = get_rx_timestamp_u64();	//timestamp: T(RR)

			/* Compute final message transmission time. See NOTE 14 below. */
			final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;    //timestamp: T(SF)

			/* Write all timestamps in the final message. See NOTE 12 below. */
			final_msg_set_ts(&(module.tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX]), poll_tx_ts);    //timestamp: T(SB)
			final_msg_set_ts(&(module.tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX]), resp_rx_ts);    //timestamp: T(RR)
			final_msg_set_ts(&(module.tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX]), final_tx_ts);    //timestamp: T(SF)

			/* Write and send final message. See NOTE 10 below. */
			module.tx_final_msg[ALL_MSG_SN_IDX] = module.frame_seq_nb;
			dwt_writetxdata(FINAL_MSG_LEN, module.tx_final_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(FINAL_MSG_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

			module.module_status = Wait_isr_2nd;

#if TWR_USE_INTERRUPT_OR_LOOP
			dwt_setinterrupt(DWT_INT_TFRS, 1);
#endif 

#if FEEDBACK_DISTANCE
			dwt_setrxaftertxdelay(FINAL_TX_TO_FEED_RX_DLY_UUS);
			dwt_setrxtimeout(FEED_RX_TIMEOUT_UUS);
			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
#else
			ret = dwt_starttx(DWT_START_TX_DELAYED);	//May fail (See <<DW1000 Device Driver API Guide>> 5.19 dwt_starttx)
#endif 

		/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 15 below. */
			if (ret == DWT_SUCCESS)
			{
#if !TWR_USE_INTERRUPT_OR_LOOP
				/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS));
				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
				module.module_status = Schedule;

				/* Increment frame sequence number after transmission of the final message (modulo 256). */
				module.frame_seq_nb++;
				/* succeed to connect node*/
				if (module.lost_count[module.current_node] != 0)
					module.lost_count[module.current_node]--;

#if FEEDBACK_DISTANCE
				while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));
				if (module.status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;
					/* Clear good RX frame event in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

					/* A frame has been received, read it into the local buffer. */
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(module.rx_buffer, frame_len, 0);
					}

					/* Check that the frame is the expected response from the companion "DS TWR responder" example.
					 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
					module.rx_buffer[ALL_MSG_SN_IDX] = 0;

					if (memcmp(module.rx_buffer, module.feedback_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						uint8 temp_num;
						/* get this anchor's number */
						temp_num = BYTE0(module.module_desaddress[module.current_node]) - '0';

						/* success: get distance */
						module.distance = get_frame_f64(&(module.rx_buffer[FEEDBACK_MSG_DISTANCE_IDX]));
						module.coord.x = get_frame_f32(&(module.rx_buffer[FEEDBACK_MSG_X_IDX]));
						module.coord.y = get_frame_f32(&(module.rx_buffer[FEEDBACK_MSG_Y_IDX]));
						module.coord.z = get_frame_f32(&(module.rx_buffer[FEEDBACK_MSG_Z_IDX]));

						Update_threeD_trilaterationpositioning(module.coord.x, module.coord.y, module.coord.z, module.distance, temp_num + 1);
					}
					else
						module.module_status = Error;
				}
				else
				{
					module.module_status = Error;
				}
#endif 		
#endif 		
			}
			else
			{
				module.module_status = Error;

#if TWR_USE_INTERRUPT_OR_LOOP
				dwt_setinterrupt(DWT_INT_TFRS, 0);
#endif 
			}
			/* Clear */
			dwt_setdelayedtrxtime(0);
#if FEEDBACK_DISTANCE
			dwt_setrxtimeout(0);
#endif 

		}
		if ((module.module_status == Wait_isr_1th) || (module.module_status == Wait_isr_2nd))
		{
			if ((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))
			{
				module.module_status = Error;
			}
		}
		if (module.module_status == Schedule)
		{
			module.current_node++;
			if (module.current_node >= module.communication_nb)
			{
				module.current_node = 0;
				// Get all anchors' data
				//return TRUE;
			}
		}
		if (module.module_status == Error)
		{
#if TWR_USE_INTERRUPT_OR_LOOP
			/* Disable all interrupt */
			dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | ISR_EVENTS_USER_RX_TO | ISR_EVENTS_USER_RX_ERR, 0);
#endif 
			dwm1000_error_commonprocess(&module);
			module.module_status = Schedule;
			module.lost_count[module.current_node]++;
		}
	//}
	return TRUE;
#undef module
}
#endif
#if EN_ANCHOR
uint8 DWM1000_run_anchor(DWM1000_module_t* _module)
{
#define module (*_module)
	uint64 poll_rx_ts;
	uint64 resp_tx_ts;
	uint64 final_rx_ts;
	/* Handle multiple DW1000 devices by using an array of those structures, as set by the #define of the DWT_NUM_DW_DEV pre-processor symbol. */
	dwt_setlocaldataptr(module.module_nb);

	if (module.module_status == Schedule)
	{
		if (module.lost_count[module.current_node] >= (UWB_MAXLOST_T * POSITIONING_ANCHOR_NUM))
		{
			uint8 temp_num;
			/* get this tag's number */
			temp_num = BYTE0(module.module_desaddress[module.current_node]) - '0';
			/* delete current anchor */
			module.module_des_en[temp_num] = FALSE;
			module.module_desaddress[module.current_node] = 0;
			module.lost_count[module.current_node] = 0;
			module.communication_nb--;
			if (module.communication_nb == 0)
			{
				module.current_node = 0;
				DWM1000_update_communicationnodes(&module);
				module.current_node = module.communication_nb - 1;
			}
		}

		/* Add source address for rx poll massage */
		dwm1000_load_address(&(module.rx_poll_msg[SRC_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
		/* Add destination address for tx response massage */
		dwm1000_load_address(&(module.tx_resp_msg[DES_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
		/* Add source address for tx final massage */
		dwm1000_load_address(&(module.rx_final_msg[SRC_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
#if FEEDBACK_DISTANCE
		/* Add destination address for feedback massage */
		dwm1000_load_address(&(module.feedback_msg[DES_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
#endif
		/* success */
		module.module_status = Response_msg;
	}
	if (module.module_status == Response_msg)
	{
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);

		/* wait for the dwm1000 triggers the function dwt_isr() */
		module.module_status = Wait_isr_1th;

#if TWR_USE_INTERRUPT_OR_LOOP
		dwt_setinterrupt(DWT_INT_RFCG, 1);
#endif 

		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

#if !TWR_USE_INTERRUPT_OR_LOOP
		/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
		while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));
		/* success */
		if (module.status_reg & SYS_STATUS_RXFCG)
		{
			module.module_status = Poll_msg;
			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
		}
		else
		{
			module.module_status = Error;
		}
#endif
	}
	if (module.module_status == Poll_msg)
	{
		uint32 frame_len;
		
		/* A frame has been received, read it into the local buffer. */
		//frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;	//a non-standard mode of operation with data frame lengths up to 1023 octets, RX_FINFO_RXFLEN_MASK
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUFFER_LEN)    
		{
			dwt_readrxdata(module.rx_buffer, frame_len, 0);
		}

		/* Check that the frame is a poll sent by "DS TWR initiator" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		module.rx_buffer[ALL_MSG_SN_IDX] = 0;

		if (memcmp(module.rx_buffer, module.rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 resp_tx_time;
			int ret;
				
			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64();	//timestamp: T(RB)

			/* Set send time for response. See NOTE 9 below. */
			resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;	//timestamp: T(SR)
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
			dwt_setrxaftertxdelay(module.resp_tx_to_final_rx_dly_uus);
			dwt_setrxtimeout(module.final_rx_timeout_uus);

			/* Write and send the response message. See NOTE 10 below.*/
			module.tx_resp_msg[ALL_MSG_SN_IDX] = module.frame_seq_nb;
			dwt_writetxdata(RESP_MSG_LEN, module.tx_resp_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(RESP_MSG_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */
			
			/* wait for the dwm1000 triggers the function dwt_isr() */
			module.module_status = Wait_isr_2nd;

#if TWR_USE_INTERRUPT_OR_LOOP
			dwt_setinterrupt(DWT_INT_RFTO | DWT_INT_RFCG, 1);
#endif 

			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

			/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
			if (ret == DWT_ERROR)
			{
				module.module_status = Error;
#if TWR_USE_INTERRUPT_OR_LOOP
				dwt_setinterrupt(DWT_INT_RFTO | DWT_INT_RFCG, 0);
#endif 			
			}
			else
			{
#if !TWR_USE_INTERRUPT_OR_LOOP
				/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
				while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));
				/* success */
				if (module.status_reg & SYS_STATUS_RXFCG)
				{
					module.module_status = Final_msg;
					/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
				}
				else
				{
					module.module_status = Error;
				}
#endif 
			}
			/* Clear */
			dwt_setdelayedtrxtime(0);
			dwt_setrxaftertxdelay(0);
			dwt_setrxtimeout(0);
		}
		else
		{
			module.module_status = Error;
			/* wait for other anchors to finish their communication */
			Sleep(TWR_WAITMISSION_MS);
		}
	}
	if (module.module_status == Final_msg)
	{
		uint32 frame_len;

		/* Increment frame sequence number after transmission of the response message (modulo 256). */
		module.frame_seq_nb++;

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(module.rx_buffer, frame_len, 0);
		}

		/* Check that the frame is a final message sent by "DS TWR initiator" example.
		 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
		module.rx_buffer[ALL_MSG_SN_IDX] = 0;

		if (memcmp(module.rx_buffer, module.rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
			uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
			double Ra, Rb, Da, Db;
			int64 tof_dtu;

			/* Retrieve response transmission and final reception timestamps. */
			resp_tx_ts = get_tx_timestamp_u64();	//timestamp: T(SR)
			final_rx_ts = get_rx_timestamp_u64();	//timestamp: T(RF)

			/* Get timestamps embedded in the final message. */
			final_msg_get_ts(&(module.rx_buffer[FINAL_MSG_POLL_TX_TS_IDX]), &poll_tx_ts);    //timestamp: T(SB)
			final_msg_get_ts(&(module.rx_buffer[FINAL_MSG_RESP_RX_TS_IDX]), &resp_rx_ts);    //timestamp: T(RR)
			final_msg_get_ts(&(module.rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX]), &final_tx_ts);    //timestamp: T(SF)

			/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
			poll_rx_ts_32 = (uint32)poll_rx_ts;    //timestamp: T(RB)
			resp_tx_ts_32 = (uint32)resp_tx_ts;	   //timestamp: T(SR)
			final_rx_ts_32 = (uint32)final_rx_ts;	//timestamp: T(RF)
			Ra = (double)(resp_rx_ts - poll_tx_ts);    	//T(RR) - T(SB)
			Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);    //T(RF) - T(SR)
			Da = (double)(final_tx_ts - resp_rx_ts);    //T(SF) - T(RR)
			Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);    //T(SR) - T(RB)
			// Link: https://forum.bitcraze.io/viewtopic.php?t=1944#p9959
			tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));    //weird !

			module.tof = tof_dtu * DWT_TIME_UNITS;
			module.distance = module.tof * SPEED_OF_LIGHT;

			/* success */
			module.module_status = Response_msg;
			/* succeed to connect node*/
			module.lost_count[module.current_node] = 0;

#if FEEDBACK_DISTANCE
			send_frame_f64(&module.feedback_msg[FEEDBACK_MSG_DISTANCE_IDX], module.distance);
			send_frame_f32(&module.feedback_msg[FEEDBACK_MSG_X_IDX], module.coord.x);
			send_frame_f32(&module.feedback_msg[FEEDBACK_MSG_Y_IDX], module.coord.y);
			send_frame_f32(&module.feedback_msg[FEEDBACK_MSG_Z_IDX], module.coord.z);
			/* Write frame data to DW1000 and prepare transmission. */
			dwt_writetxdata(FEEDBACK_MSG_LEN, module.feedback_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(FEEDBACK_MSG_LEN, 0, 0); /* Zero offset in TX buffer, no ranging. */

			/* Start transmission. */
			dwt_starttx(DWT_START_TX_IMMEDIATE);

			/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
			while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS));
			/* Clear TXFRS event. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
#endif
			/* wait for other anchors to finish their communication */
			Sleep(TWR_FINISHMISSION_DELAY_MS);
		}
		else
			module.module_status = Error;
	}
	if ((module.module_status == Wait_isr_1th) || (module.module_status == Wait_isr_2nd))
	{
		if ((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))
		{
			module.module_status = Error;
		}
	}
	if (module.module_status == Error)
	{
#if TWR_USE_INTERRUPT_OR_LOOP
		/* Disable all interrupt */
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | ISR_EVENTS_USER_RX_TO | ISR_EVENTS_USER_RX_ERR, 0);
#endif // TWR_USE_INTERRUPT_OR_LOOP
		dwm1000_error_commonprocess(&module);
		module.module_status = Schedule;
		module.lost_count[module.current_node]++;
	}
	return TRUE;
#undef module
}
#endif



uint8 DWM1000_update_communicationnodes(DWM1000_module_t* _module)
{
#define module (*_module)
	if (module.module_nb > DWT_NUM_DW_DEV)
		return FALSE;

	/* update communication nodes for the module */
	if((module.module_status == Initialization)||(module.module_status == Schedule))
		module.module_status = Update_comnodes;
	else
		return FALSE;

	while (module.module_status == Update_comnodes)
	{
		if (module.module_type == Anchor_module)
		{
			/* Clear reception timeout to start next ranging process. */
			dwt_setrxtimeout(0);

			/* Activate reception immediately. */
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
#if !TWR_USE_INTERRUPT_OR_LOOP
			/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
			while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

			if (module.status_reg & SYS_STATUS_RXFCG)
			{
				uint32 frame_len;
				uint8 address[2];
				/* Clear good RX frame event in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUFFER_LEN)
				{
					dwt_readrxdata(module.rx_buffer, frame_len, 0);
				}
				module.rx_buffer[ALL_MSG_SN_IDX] = 0;
				/* Get tag address */
				address[0] = module.rx_buffer[SRC_ADDRESS_L_8BIT];
				module.rx_buffer[SRC_ADDRESS_L_8BIT] = '0';
				address[1] = module.rx_buffer[SRC_ADDRESS_H_8BIT];
				module.rx_buffer[SRC_ADDRESS_H_8BIT] = '0';

				if (memcmp(module.rx_buffer, module.tx_broadcast_msg, ALL_MSG_COMMON_LEN) == 0)
				{
					/* Receive broadcast frame */
					module.module_desaddress[module.current_node] = address[0] + (address[1] << 8);

					/* Add destination address for rx broadcast massage */
					dwm1000_load_address(&(module.rx_broadcast_msg[DES_ADDRESS_L_8BIT]), module.module_desaddress[module.current_node]);
					/* Add source address for rx broadcast massage */
					dwm1000_load_address(&(module.rx_broadcast_msg[SRC_ADDRESS_L_8BIT]), module.module_srcaddress);

					/* Write response frame data to DW1000 and prepare transmission. See NOTE 6 below.*/
					dwt_writetxdata(BROADCAST_MSG_LEN, module.rx_broadcast_msg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(BROADCAST_MSG_LEN, 0, 0); /* Zero offset in TX buffer, no ranging. */

					/* Configure preamble timeout to 3 PACs; if no preamble detected in this time we assume channel is clear. */
					dwt_setpreambledetecttimeout(CCA_PREAMBLE_TO);

					/* Activate RX to perform CCA. */
					dwt_rxenable(DWT_START_RX_IMMEDIATE);
					/* Send the response. */
					dwt_starttx(DWT_START_TX_IMMEDIATE);

					/* Poll DW1000 until preamble timeout or detection. */
					while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXPRD | SYS_STATUS_RXPTO | SYS_STATUS_ALL_RX_ERR)));

					if (module.status_reg & SYS_STATUS_RXPTO)
					{
						/* Clear RX Preamble detection timeout event in the DW1000 status register. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPTO);

						dwt_setpreambledetecttimeout(0);

						/* Poll DW1000 until TX frame sent event set. */
						while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS));

						/* Clear TX frame sent event. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
				
						uint8 temp_num;

						/* get this tag's number */
						temp_num = address[0] - '0';
						if (module.module_des_en[temp_num] == FALSE)    // get a new tag
						{
							module.module_des_en[temp_num] = TRUE;
							module.communication_nb++;
							module.current_node = module.communication_nb - 1;
						}
						/* success */
						module.module_status = Schedule;

					}
					else if (module.status_reg & SYS_STATUS_RXPRD)
					{
						/* Clear RX Preamble Detected events in the DW1000 status register. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPRD);

						/* if DW IC detects the preamble, as we don't want to receive a frame we TRX OFF
						 * and wait for a backoff_period before trying to transmit again */
						dwt_forcetrxoff();

						module.module_status = Update_comnodes;
					}
					else
					{
						module.module_status = Error;
					}
				}
				else
				{
					module.module_status = Error;
				}
			}
			else
			{
				module.module_status = Error;
			}

			if (module.module_status == Error)
			{
#if TWR_USE_INTERRUPT_OR_LOOP
				/* Disable all interrupt */
				dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | ISR_EVENTS_USER_RX_TO | ISR_EVENTS_USER_RX_ERR, 0);
#endif // TWR_USE_INTERRUPT_OR_LOOP
				dwm1000_error_commonprocess(&module);
				module.module_status = Update_comnodes;
			}
#endif
		}
		else if (module.module_type == Tag_module)
		{
			/* local variables holdes the result of the most recent channel assessment y pseudo CCA algorithm,
			 * 1 (channel is clear) or 0 (preamble was detected) */
			//uint8 channel_clear = 1;
			uint8 tx_sleep_period; /* Sleep period until the next TX attempt */
			uint8 next_backoff_interval = INITIAL_BACKOFF_PERIOD; /* Next backoff in the event of busy channel detection by this pseudo CCA algorithm */

			/* Configure preamble timeout to 3 PACs; if no preamble detected in this time we assume channel is clear. */
			dwt_setpreambledetecttimeout(CCA_PREAMBLE_TO);
			dwt_setrxtimeout(0);

			/* Add source address for tx broadcast massage */
			dwm1000_load_address(&(module.tx_broadcast_msg[SRC_ADDRESS_L_8BIT]), module.module_srcaddress);
			/* Add destination address for rx broadcast massage */
			dwm1000_load_address(&(module.rx_broadcast_msg[DES_ADDRESS_L_8BIT]), module.module_srcaddress);

			/* Write frame data to DW1000 and prepare transmission. */
			dwt_writetxdata(BROADCAST_MSG_LEN, module.tx_broadcast_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(BROADCAST_MSG_LEN, 0, 0); /* Zero offset in TX buffer, no ranging. */

			/* Activate RX to perform CCA. */
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			/* Start transmission. Will be delayed (the above RX command has to finish first)
			 * until we get the preamble timeout or canceled by TRX OFF if a preamble is detected. */
			dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

			/* Poll DW1000 until preamble timeout or detection. */
			while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXPRD | SYS_STATUS_RXPTO | SYS_STATUS_ALL_RX_ERR)));

			if (module.status_reg & SYS_STATUS_RXPTO)
			{
				//channel_clear = 1;
					
				/* Clear RX Preamble detection timeout event in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPTO);

				dwt_setrxtimeout(CCA_TAG_RX_TIMEOUT_UUS);
				dwt_setpreambledetecttimeout(0);

				/* Poll DW1000 until frame sent */
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS));

				/* Clear TX frame sent event in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				while (!((module.status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

				if (module.status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;
					uint8 address[2];
					uint8 ackowledge;
					/* Clear good RX frame event in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

					/* A frame has been received, read it into the local buffer. */
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(module.rx_buffer, frame_len, 0);
					}

					module.rx_buffer[ALL_MSG_SN_IDX] = 0;
					/* Get anchor address */
					address[0] = module.rx_buffer[SRC_ADDRESS_L_8BIT];
					module.rx_buffer[SRC_ADDRESS_L_8BIT] = '0';
					address[1] = module.rx_buffer[SRC_ADDRESS_H_8BIT];
					module.rx_buffer[SRC_ADDRESS_H_8BIT] = '0';

					ackowledge = module.rx_buffer[ACKNOWLEDGE_CODE];

					if (memcmp(module.rx_buffer, module.rx_broadcast_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						/* acknowledge (0x01 to indicate yes, 0x02 refers to no acknowledge) */
						if (ackowledge == ACK_YES)
						{
							uint8 temp_num;
							
							/* get this anchor's number */
							temp_num = address[0] - '0';
							if (module.module_des_en[temp_num] == FALSE)    // get a new anchor
							{
								module.module_des_en[temp_num] = TRUE;
								module.module_desaddress[module.current_node] = address[0] + (address[1] << 8);
								module.communication_nb++;
								module.current_node = module.communication_nb - 1;
							}
							else
							{
								/* skip current anchor */
							}		
						}
						else if (ackowledge == ACK_NO)
						{
							/* no acknowledge */
						}

						if (module.communication_nb >= POSITIONING_ANCHOR_NUM)
						{
							/* success */
							module.module_status = Schedule;						
						}
						else
						{
							module.module_status = Update_comnodes;
							module.current_node = module.communication_nb - 1 + 1;
						}				
					}
					else
					{
						module.module_status = Error;
					}
				}
				else
				{
					module.module_status = Error;
				}

				dwt_setrxtimeout(0);
				tx_sleep_period = TX_DELAY_MS; /* sent a frame - set interframe period */
				next_backoff_interval = INITIAL_BACKOFF_PERIOD; /* set initial backoff period */
			}
			else if (module.status_reg & SYS_STATUS_RXPRD)
			{
				/* Clear RX Preamble Detected events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPRD);

				/* if DW IC detects the preamble, as we don't want to receive a frame we TRX OFF
					and wait for a backoff_period before trying to transmit again */
				dwt_forcetrxoff();

				tx_sleep_period = next_backoff_interval; /* set the TX sleep period */
				next_backoff_interval++; /* If failed to transmit, increase backoff and try again.
										  * In a real implementation the back-off is typically a randomised period
										  * whose range is an exponentially related to the number of successive failures.
										  * See https://en.wikipedia.org/wiki/Exponential_backoff */
				//channel_clear = 0;
				module.module_status = Update_comnodes;
			}
			else
			{
				module.module_status = Error;
			}

			if (module.module_status == Update_comnodes)
			{
				/* Execute a delay between transmissions. */
				Sleep(tx_sleep_period);
			}

			if (module.module_status == Error)
			{
#if TWR_USE_INTERRUPT_OR_LOOP
				/* Disable all interrupt */
				dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | ISR_EVENTS_USER_RX_TO | ISR_EVENTS_USER_RX_ERR, 0);
#endif // TWR_USE_INTERRUPT_OR_LOOP
				dwm1000_error_commonprocess(&module);
				module.module_status = Update_comnodes;
			}
		}
		else
			return FALSE;
	}
	/* success */
	//module.module_status = Schedule;

	return TRUE;
#undef module
}

#if TWR_USE_INTERRUPT_OR_LOOP
void DWM1000_tag_isr(void)
{
	uwb_callback.module = &initiator;
	/* Handle multiple DW1000 devices by using an array of those structures, as set by the #define of the DWT_NUM_DW_DEV pre-processor symbol. */
	dwt_setlocaldataptr(uwb_callback.module->module_nb);
	dwt_isr();
}

void DWM1000_anchor_isr(void)
{
	uwb_callback.module = &responder;
	/* Handle multiple DW1000 devices by using an array of those structures, as set by the #define of the DWT_NUM_DW_DEV pre-processor symbol. */
	dwt_setlocaldataptr(uwb_callback.module->module_nb);
	dwt_isr();
}

void DWM1000_EXTI_Callback(uint16_t GPIO_Pin)
{
#if EN_SINGLE_MODULE
	if (GPIO_Pin == DW_IRQn0_Pin)
	{
		//DWM1000_ISR1();
#if EN_TAG
		DWM1000_ISR1();
#endif // EN_TAG

#if EN_ANCHOR
		DWM1000_ISR2();
#endif // EN_ANCHOR
	}
#endif // EN_SINGLE_MODULE

#if EN_MULTIPLE_MODULES
	if (GPIO_Pin == DW_IRQn0_Pin)
	{
		DWM1000_ISR1();
	}
	else if (GPIO_Pin == DW_IRQn1_Pin)
	{
		DWM1000_ISR2();
	}
#endif // EN_MULTIPLE_MODULES
}
#endif // TWR_USE_INTERRUPT_OR_LOOP



static uint8 dwm1000_set_module(DWM1000_module_t* _module, uint8 _rate, DWM1000_type _type)
{
#define module (*_module)
	uint8 pac_tmp = 0;
	uint8 sfd_tmp = 0;
	uint8* pointer = NULL;
	uint16 preamblength_tmp = 0;
	/* module number */
	static uint16 number = 0;
	uint16 i;
	/* initialize module */
	module.module_status = Initialization;
	/* use for Decawave deriver */
	if (number >= DWT_NUM_DW_DEV)
		return FALSE;

	module.module_nb = number;
	/* config */
	if (_rate == DWT_BR_110K)
	{
		module.config.txPreambLength = DWT_PLEN_1024;
		module.config.rxPAC = DWT_PAC32;
		module.config.dataRate = DWT_BR_110K;
		sfd_tmp = DW_NS_SFD_LEN_110K;
	}
	else if (_rate == DWT_BR_850K)
	{
		module.config.txPreambLength = DWT_PLEN_256;
		module.config.rxPAC = DWT_PAC16;
		module.config.dataRate = DWT_BR_850K;
		sfd_tmp = DW_NS_SFD_LEN_850K;
	}
	else if (_rate == DWT_BR_6M8)
	{
		module.config.txPreambLength = DWT_PLEN_64;
		module.config.rxPAC = DWT_PAC8;
		module.config.dataRate = DWT_BR_6M8;
		sfd_tmp = DW_NS_SFD_LEN_6M8;
	}
	else
		return FALSE;
	/* Anchor or Tag */
	if (_type == Anchor_module)
	{
		module.module_type = Anchor_module;
		pointer = (uint8*)(&UWB_DEFAULTANCHOR_SRCADDR);
		module.module_srcaddress = pointer[1] + (pointer[0] << 8);
		module.communication_nb = 0;
		module.current_node = 0;
		/* fill all massages */
		module.rx_poll_msg = (uint8*) &(rx_poll_msg[module.module_nb][0]);
		module.tx_resp_msg = (uint8*) &(tx_resp_msg[module.module_nb][0]);
		module.rx_final_msg = (uint8*) &(rx_final_msg[module.module_nb][0]);
		module.feedback_msg = (uint8*) &(feedback_msg[module.module_nb][0]);
		module.tx_poll_msg = NULL;
		module.rx_resp_msg = NULL;
		module.tx_final_msg = NULL;

		/* Add destination address for rx poll massage */
		dwm1000_load_address(&(module.rx_poll_msg[DES_ADDRESS_L_8BIT]), module.module_srcaddress);
		/* Add source address for tx response massage */
		dwm1000_load_address(&(module.tx_resp_msg[SRC_ADDRESS_L_8BIT]), module.module_srcaddress);
		/* Add destination address for tx final massage */
		dwm1000_load_address(&(module.rx_final_msg[DES_ADDRESS_L_8BIT]), module.module_srcaddress);
#if FEEDBACK_DISTANCE
		/* Add source address for feedback massage */
		dwm1000_load_address(&(module.feedback_msg[SRC_ADDRESS_L_8BIT]), module.module_srcaddress);
#endif
	}
	else if (_type == Tag_module)
	{
		module.module_type = Tag_module;
		pointer = (uint8*)(&UWB_DEFAULTTAG_SRCADDR);
		module.module_srcaddress = pointer[1] + (pointer[0] << 8);
		module.communication_nb = 0;
		module.current_node = 0;
		/* fill all massages */
		module.tx_poll_msg = (uint8*) &(tx_poll_msg[module.module_nb][0]);
		module.rx_resp_msg = (uint8*) &(rx_resp_msg[module.module_nb][0]);
		module.tx_final_msg = (uint8*) &(tx_final_msg[module.module_nb][0]);
		module.feedback_msg = (uint8*) &(feedback_msg[module.module_nb][0]);
		module.rx_poll_msg = NULL;
		module.tx_resp_msg = NULL;
		module.rx_final_msg = NULL;

		/* Add source address for tx poll massage */
		dwm1000_load_address(&(module.tx_poll_msg[SRC_ADDRESS_L_8BIT]), module.module_srcaddress);
		/* Add destination address for rx response massage */
		dwm1000_load_address(&(module.rx_resp_msg[DES_ADDRESS_L_8BIT]), module.module_srcaddress);
		/* Add source address for tx final massage */
		dwm1000_load_address(&(module.tx_final_msg[SRC_ADDRESS_L_8BIT]), module.module_srcaddress);
#if FEEDBACK_DISTANCE
		/* Add destination address for feedback massage */
		dwm1000_load_address(&(module.feedback_msg[DES_ADDRESS_L_8BIT]), module.module_srcaddress);
#endif
		
	}
	else
		return FALSE;
	/* config */
	module.config.chan = UWB_CHANNEL_IS;
	module.config.prf = UWB_PRF;
	module.config.txCode = UWB_PREAMBLECODES;
	module.config.rxCode = UWB_PREAMBLECODES;
	module.config.nsSFD = UWB_NSFD;
	module.config.phrMode = UWB_PHRMODE;
	switch (module.config.txPreambLength)
	{
	case DWT_PLEN_4096: preamblength_tmp = 4096;
		break;
	case DWT_PLEN_2048: preamblength_tmp = 2048;
		break;
	case DWT_PLEN_1536: preamblength_tmp = 1536;
		break;
	case DWT_PLEN_1024: preamblength_tmp = 1024;
		break;
	case DWT_PLEN_512: preamblength_tmp = 512;
		break;
	case DWT_PLEN_256: preamblength_tmp = 256;
		break;
	case DWT_PLEN_128: preamblength_tmp = 128;
		break;
	case DWT_PLEN_64: preamblength_tmp = 64;
		break;
	default: return FALSE;
	}
	switch (module.config.rxPAC)
	{
	case DWT_PAC8: pac_tmp = 8;
		break;
	case DWT_PAC16: pac_tmp = 16;
		break;
	case DWT_PAC32: pac_tmp = 32;
		break;
	case DWT_PAC64: pac_tmp = 64;
		break;
	default: return FALSE;
	}
	module.config.sfdTO = preamblength_tmp + 1 + sfd_tmp /*- pac_tmp*/;
	/* txconfig */
	module.txconfig.PGdly = UWB_PGDLYVALUE;
	module.txconfig.power = UWB_TXPOWERVALUE;

	module.tx_ant_dly = TX_ANT_DLY;
	module.rx_ant_dly = RX_ANT_DLY;
	module.status_reg = 0;
	module.lost_count[module.current_node] = 0;
	module.pre_timeout = (preamblength_tmp / pac_tmp);
	module.poll_tx_to_resp_rx_dly_uus = POLL_TX_TO_RESP_RX_DLY_UUS;
	module.poll_rx_to_resp_tx_dly_uus = POLL_RX_TO_RESP_TX_DLY_UUS;
	module.resp_rx_to_final_tx_dly_uus = RESP_RX_TO_FINAL_TX_DLY_UUS;
	module.resp_tx_to_final_rx_dly_uus = RESP_TX_TO_FINAL_RX_DLY_UUS;
	module.resp_rx_timeout_uus = RESP_RX_TIMEOUT_UUS;
	module.final_rx_timeout_uus = FINAL_RX_TIMEOUT_UUS;
	module.tx_broadcast_msg = (uint8*) &(tx_broadcast_msg[module.module_nb][0]);
	module.rx_broadcast_msg = (uint8*) &(rx_broadcast_msg[module.module_nb][0]);
	/* clear variables */
	module.frame_seq_nb = 0;
	module.tof = 0;
	module.distance = 0;
	module.coord.x = 0;
	module.coord.y = 0;
	module.coord.z = 0;
	for (i = module.communication_nb; i < MAX_NODE_NUM; i++)
	{
		module.module_desaddress[i] = 0;
		module.module_des_en[i] = 0;
		module.lost_count[i] = 0;
	}
	for (i = 0; i < RX_BUF_LEN; i++)
	{
		module.rx_buffer[i] = 0;
	}

	number++;
	return TRUE;
#undef module
}

#if EN_TAG
static uint8 dwm1000_tag_initialization(DWM1000_module_t* _module)
{
#define module (*_module)
	/* Handle multiple DW1000 devices by using an array of those structures, as set by the #define of the DWT_NUM_DW_DEV pre-processor symbol. */
	dwt_setlocaldataptr(module.module_nb);

	/* Reset and initialise DW1000.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance. */
	reset_DW1000(module.module_nb); /* Target specific drive of RSTn line into DW1000 low for a period. */
	port_set_dw1000_slowrate();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		return FALSE;
	}
	port_set_dw1000_fastrate();

	/* Configure DW1000. See NOTE 7 below. */
	dwt_configure(&module.config);

	//Big change
	/* Enables smart TX power functionality of DW1000 */
	dwt_setsmarttxpower(1);
	/* Setting up the transmit RF configuration parameters. */
	dwt_configuretxrf(&module.txconfig);

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(module.rx_ant_dly);
	dwt_settxantennadelay(module.tx_ant_dly);

	return TRUE;
#undef module
}
#endif
#if EN_ANCHOR
static uint8 dwm1000_anchor_initialization(DWM1000_module_t* _module)
{
#define module (*_module)
	/* Handle multiple DW1000 devices by using an array of those structures, as set by the #define of the DWT_NUM_DW_DEV pre-processor symbol. */
	dwt_setlocaldataptr(module.module_nb);

	/* Reset and initialise DW1000.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance. */
	reset_DW1000(module.module_nb); /* Target specific drive of RSTn line into DW1000 low for a period. */
	port_set_dw1000_slowrate();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		return FALSE;
	}
	dwt_readdevid();
	port_set_dw1000_fastrate();

	/* Configure DW1000. See NOTE 7 below. */
	dwt_configure(&module.config);

	//Big change
	/* Enables smart TX power functionality of DW1000 */
	dwt_setsmarttxpower(1);
	/* Setting up the transmit RF configuration parameters. */
	dwt_configuretxrf(&module.txconfig);

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(module.rx_ant_dly);
	dwt_settxantennadelay(module.tx_ant_dly);

	/* Set preamble timeout for expected frames. See NOTE 6 below. */
//	dwt_setpreambledetecttimeout(module.pre_timeout);

	return TRUE;
#undef module
}
#endif

static uint8 dwm1000_error_commonprocess(DWM1000_module_t* _module)
{
#define module (*_module)

	/* Clear RX error/timeout events in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
	/* Clear all receive status bits */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);
	/* Clear TX event bits */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
	/* Reset RX to properly reinitialise LDE operation. */
	dwt_forcetrxoff();
	dwt_rxreset();

	if (module.status_reg == STATUS_REG_ERROR)
	{
		DWM1000_initialization();
	}

	return TRUE;
#undef module
}

static uint8 dwm1000_load_address(uint8* _msg, uint16 _address)
{
	/* from LSB to MSB */
	_msg[0] = BYTE0(_address);
	_msg[1] = BYTE1(_address);

	return TRUE;
}

#if USE_TXRX_STATES_LED
static void dwt1000_txrx_statexled(uint8 _en)
{
	uint32 gpio_mode = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
	gpio_mode &= ~(GPIO_MSGP5_MASK | GPIO_MSGP6_MASK);
	if (_en != FALSE)
	{
		gpio_mode |= (GPIO_PIN5_EXTTXE | GPIO_PIN6_EXTRXE);
		dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, gpio_mode);
	}

}
#endif // USE_TXRX_STATES_LED

#if TWR_USE_INTERRUPT_OR_LOOP
/* Before call callback functions, you should confirm your DWM1000_module_t by using uwb_callback variable */
static void dwm1000_callback_txdone(const dwt_cb_data_t * _data)
{
	if (uwb_callback.module->module_type == Tag_module)
	{
		if (uwb_callback.module->module_status == Wait_isr_2nd)
		{
			uwb_callback.module->module_status = Poll_msg;
			dwt_setinterrupt(DWT_INT_TFRS, 0);
		}
	}

}

static void dwm1000_callback_rxok(const dwt_cb_data_t * _data)
{
	if (uwb_callback.module->module_type == Tag_module)
	{
		if (uwb_callback.module->module_status == Wait_isr_1th)
		{
			uwb_callback.module->module_status = Response_msg;
			dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO, 0);
		}
	}
	else if (uwb_callback.module->module_type == Anchor_module)
	{
		switch (uwb_callback.module->module_status)
		{
		case Wait_isr_1th: uwb_callback.module->module_status = Poll_msg; dwt_setinterrupt(DWT_INT_RFCG, 0);
			break;
		case Wait_isr_2nd: uwb_callback.module->module_status = Final_msg; dwt_setinterrupt(DWT_INT_RFTO | DWT_INT_RFCG, 0);
			break;
		default:
			break;
		}
	}
}

static void dwm1000_callback_rxto(const dwt_cb_data_t * _data)
{
	if (uwb_callback.module->module_type == Tag_module)
	{
		uwb_callback.module->module_status = Error;
		dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO, 0);
	}
	else if (uwb_callback.module->module_type == Anchor_module)
	{
		uwb_callback.module->module_status = Error;
		dwt_setinterrupt(DWT_INT_RFTO | DWT_INT_RFCG, 0);
	}
}

static void dwm1000_callback_rxerr(const dwt_cb_data_t * _data)
{
	if (uwb_callback.module->module_type == Tag_module)
	{
		uwb_callback.module->module_status = Error;
	}
	else if (uwb_callback.module->module_type == Anchor_module)
	{
		uwb_callback.module->module_status = Error;
	}
}
#endif // TWR_USE_INTERRUPT_OR_LOOP

#if EN_TAG
static uint8 arrange_address(uint16* _pointer, uint16 _avail_num)
{
	uint8 num_array[MAX_NODE_NUM] = { 0 };
	uint16 i, j;
	if (_avail_num == 0)
		return FALSE;

	for (i = 0, j = 0; i < MAX_NODE_NUM; i++)
	{
		if (_pointer[i] != 0)
		{
			num_array[j] = i;
			j++;
			if (j >= _avail_num)
			{
				break;
			}
		}
	}
	for (i = 0; i < _avail_num; i++)
	{
		_pointer[i] = _pointer[num_array[i]];
	}
	for (i = _avail_num; i < MAX_NODE_NUM; i++)
	{
		_pointer[i] = 0;    // clean useless address
	}

	return TRUE;
}
#endif
#if FEEDBACK_DISTANCE
#if EN_TAG
static double get_frame_f64(uint8* _pointer)
{
	uint64 ts = 0;
	int i;
	for (i = 7; i >= 0; i--)
	{
		ts <<= 8;
		ts |= _pointer[i];
	}
	return (*((double *)(&ts)));
}

static float get_frame_f32(uint8* _pointer)
{
	uint32 ts = 0;
	int i;
	for (i = 3; i >= 0; i--)
	{
		ts <<= 8;
		ts |= _pointer[i];
	}
	return (*((float *)(&ts)));
}
#endif
#if EN_ANCHOR
static void send_frame_f64(uint8* _pointer, double _data)
{
	uint8* ts = (uint8*)&_data;
	int i;
	for (i = 0; i <= 7; i++)
	{
		_pointer[i] = ts[i];
	}

}

static void send_frame_f32(uint8* _pointer, float _data)
{
	uint8* ts = (uint8*)&_data;
	int i;
	for (i = 0; i <= 3; i++)
	{
		_pointer[i] = ts[i];
	}

}
#endif
#endif

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

#if EN_TAG
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
	int i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		ts_field[i] = (uint8)ts;
		ts >>= 8;
	}
}
#endif
#if EN_ANCHOR
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
	int i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the EVB1000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. In this operation, the high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps 
 *     are not separated by more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed 
 *     in the time-of-flight computation) can be handled by a 32-bit subtraction.       
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 * 14. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 * 15. When running this example on the EVB1000 platform with the RESP_RX_TO_FINAL_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange to try another one after 1 second. If this error handling code was not here, a late dwt_starttx() would result in the code
 *     flow getting stuck waiting for a TX frame sent event that will never come. The companion "responder" example (ex_05b) should timeout from
 *     awaiting the "final" and proceed to have its receiver on ready to poll of the following exchange.
 *
 * RELATED PARAMETERS:
 *
 * For the function: void dwt_configure(dwt_config_t *config)
 * 1. The supported channels are 1, 2, 3, 4, 5, and 7 (4 and 7 have big band)
 * 2. DW1000 supported UWB channels and recommended preamble codes
 *    | Channel number | Preamble Codes (16 MHz PRF) | Preamble Codes (64 MHz PRF) |
 *    |       1        |           1, 2              |    9, 10, 11, 12            |
 *    |       2        |           3, 4              |    9, 10, 11, 12            |
 *    |       3        |           5, 6              |    9, 10, 11, 12            |
 *    |       4        |           7, 8              |   17, 18, 19, 20            |
 *    |       5        |           3, 4              |    9, 10, 11, 12            |
 *    |       7        |           7, 8              |   17, 18, 19, 20            |
 * 3. Recommended preamble lengths
 *    | Date Rate | Recommended preamble sequence length |
 *    | 6.8Mbps   |           64 or 128 or 256           |
 *    | 850kbps   |         256 or 512 or 1024           |
 *    | 110kbps   |      1024 or 1536, or 2048           |
 * 4. Recommended PAC size
 *    | Expected preamble length of frames being received | Recommended PAC size |
 *    |                    64                             |           8          |
 *    |                   128                             |           8          |
 *    |                   256                             |          16          |
 *    |                   512                             |          16          |
 *    |                  1024                             |          32          |
 *    |                  1536                             |          64          |
 *    |                  2048                             |          64          |
 *    |                  4096                             |          64          |
 *
 * For the function: void dwt_configuretxrf(dwt_txconfig_t *config)
 * 1. PGdly recommended values
 * | TX Channel | recommended PGdly value |
 * |     1      |           0xC9          |
 * |     2      |           0xC2          |
 * |     3      |           0xC5          |
 * |     4      |           0x95          |
 * |     5      |           0xC0          |
 * |     7      |           0x93          |
 * 2. TX power recommended values (when smart power is disabled)
 * | TX Channel | recommended TX power value (16 MHz) | recommended TX power value (64 MHz) |
 * |     1      |             0x75757575              |             0x67676767              |
 * |     2      |             0x75757575              |             0x67676767              |
 * |     3      |             0x6F6F6F6F              |             0x8B8B8B8B              |
 * |     4      |             0x5F5F5F5F              |             0x9A9A9A9A              |
 * |     5      |             0x48484848              |             0x85858585              |
 * |     7      |             0x92929292              |             0xD1D1D1D1              |
 * 3. TX power recommended values (when smart power is enabled)
 * | TX Channel | recommended TX power value (16 MHz) | recommended TX power value (64 MHz) |
 * |     1      |             0x15355575              |             0x07274767              |
 * |     2      |             0x15355575              |             0x07274767              |
 * |     3      |             0x0F2F4F6F              |             0x2B4B6B8B              |
 * |     4      |             0x1F1F3F5F              |             0x3A5A7A9A              |
 * |     5      |             0x0E082848              |             0x25456585              |
 * |     7      |             0x32527292              |             0x5171B1D1              |
 * 3. Non standard SFD sequences
 *    see uint8 dwnsSFDlen[3]
 *    110kbps - 64
 *    850kbps - 16
 *    6.8Mbps - 8
 *
 * GRAPH FOR TWO-WAY RANGING EXCHANGE
 * Tag sees Round Trip, T(RT), of (T(RR) - T(SB))
 * Reader sees Round Trip, R(RT), of (T(RF) - T(SR))
 *                                                    Tag
 *              Reader                               Times
 *               Times          Tag blink
 *                                 (ID)              -T(SB)
 *                                               ----
 *                                          ----
 *                                     ----
 *                                ----
 *                           ----
 *               T(RB) <----
 *
 *
 *               T(SR)-      simple ACK response     
 *                     ----
 *                          ----
 *                               ----
 *                                    ----
 *                                         ----
 *                                              ----> T(RR)
 *
 *                            Final Message
 *                      (ID, T(SB), T(RR), T(SF))    -T(SF)
 *                                               ----
 *                                          ----
 *                                     ----
 *                                ----
 *                           ----
 *               T(RF) <----
 *
 * For the function: DWM1000_update_communicationnodes(DWM1000_module_t* _module)
 * Node status:
 *              Reader                                Tag              
 *                                                 listen (CCA)<-------+
 *                                                   -TX               |
 *                          tx_broadcast_msg     ----                  |
 *                                          ----                       |
 *                                     ----                            |
 *                                ----                                 |
 *                          ----                                       |
 *                RX  <----                                            |
 *            listen (CCA)                                             |
 *                TX-                                                  |
*                     ----   rx_broadcast_msg                          |
 *                          ----                                       | 
 *                               ----                                  |
 *                                    ----                             |
 *                                         ----                        |
 *                                              ----> RX               |
 *           Go schedule                           ACK_YES/NO          |
 *                                           POSITIONING_ANCHOR_NUM? --+
 * For the functionality: FEEDBACK_DISTANCE
 * Node status:
 *              Reader                                Tag
 *               T(RF)                               T(SF)
 *         calculate distance                      
 *                                         FINAL_TX_TO_FEED_RX_DLY_UUS        
 *               TX-
 *                     ----
 *                          ----
 *                               ----
 *                                    ----
 *                                         ----
 *                                              ----> RX
 *                                               get distance
 *
 *
 * TEST CONNECTION
 * 1. program example:
 * < main() >
 * hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
 * HAL_SPI_Init(&hspi2);
 * while(1)
 * {
 * 		reset_DW1000(responder.module_nb); // Target specific drive of RSTn line into DW1000 low for a period. 
 *      hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;   
 *      HAL_SPI_Init(&hspi2);
 *      dwt_read32bitoffsetreg(0x00,0);
 * }
 *
 *
 *
 *
 *
 *
 ****************************************************************************************************************************************************/


