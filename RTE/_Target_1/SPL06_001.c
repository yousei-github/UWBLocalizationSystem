#ifndef SPL06_01_C
#define SPL06_01_C
#include "spl06_001.h"

#define USE_DEVICE "stm32f4xx.h"
#define USE_DRIVER "stm32f4xx_hal.h"
#include "main.h"
#define USE_OS_UCOS_III 1
#include USE_DEVICE
#include USE_DRIVER
#if USE_OS_UCOS_III
#include  <UCOS_includes.h>
#endif



#if USE_IIC_FOR_SPL06

#undef SUCCESS 
#define SUCCESS 0
#undef FAILED 
#define FAILED 1
#undef ENABLE 
#define ENABLE 1
#undef DISABLE 
#define DISABLE 0

// register definition
#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG_REG 0x06 // Barometric pressure measurement rate configuration
#define TMP_CFG_REG 0x07 // Temperature measurement speed configuration
#define MEAS_CFG_REG 0x08 // Measurement configuration and sensor configuration
#define CFG_REG 0x09 // Interrupt/FIFO/SPI line number and other configuration
#define INT_STS_REG 0X0A // interrupt status flag
#define FIFO_STS_REG 0X0B // FIFO status
#define RESET_REG 0X0C
#define ID_REG 0x0D
#define COEF_REG 0x10


#define s32 int32
#define int16 short
#define int32 int
#define uint8 unsigned char
//#define HW_ADR 0x77 //SDO HIGH OR NC
#define HW_ADR 0x76 //SDO LOW
static struct {
	int16 c0;
	int16 c1;
	int32 c00;
	int32 c10;
	int16 c01;
	int16 c11;
	int16 c20;
	int16 c21;
	int16 c30;
}spl06_calib_param;


struct {
	uint8 chip_id; /**<chip id*/
	int32 i32rawPressure;
	int32 i32rawTemperature;
	int32 i32kP;
	int32 i32kT;
}spl06;



// I2C configuration
#define spl0601_write(ADDR,REG,DATA) IIC_Write_1Byte(ADDR,REG,DATA)

uint8 spl0601_read(unsigned char hwadr, unsigned char regadr)
{
	uint8 reg_data;

	IIC_Read_1Byte(hwadr, regadr, &reg_data);
	return reg_data;
}

/* What is oversampling, that is, sampling multiple groups but only take one data, for example, 
  if the temperature sampling frequency is 10HZ, and the oversampling is set to 8, that is, 
  each temperature reading is an average of 8 groups, and 80 groups are actually collected internally, 
  and there is no oversampling in background mode. 
	*/
/***********************************************************************
 * Barometric Sampling Rate Configuration (background mode)
 * @param[in] background_rate: Sampling frequency N times/second (Only used in background mode, 
 * the sampling frequency of command mode is determined by the user, the minimum interval depends on the time required 
 * for oversampling, and different oversampling times can have different power consumption)
 * @param[in] oversamply: Oversampling times
 * @param[out]
 * @return
 **********************************************************************/
#define PRESSURE_RATE_1_TIMES 0 // Sampling Rate
#define PRESSURE_RATE_2_TIMES 1
#define PRESSURE_RATE_4_TIMES 2
#define PRESSURE_RATE_8_TIMES 3
#define PRESSURE_RATE_16_TIMES 4
#define PRESSURE_RATE_32_TIMES 5
#define PRESSURE_RATE_64_TIMES 6
#define PRESSURE_RATE_128_TIMES 7

void spl06_pressure_rate_config(u8 background_rate, u8 oversamply)
{
	u8 data;

	data = (background_rate << 4) | oversamply;
	if (oversamply > PRESSURE_RATE_8_TIMES)// Oversampling times greater than EMPERATURE_RATE_8_TIMES, it should allow data to be overwritten with new data
	{
		u8 data;
		data = spl0601_read(HW_ADR, CFG_REG);
		data |= 0X04;
		spl0601_write(HW_ADR, CFG_REG, data);
	}
	switch (oversamply)
	{
	case PRESSURE_RATE_2_TIMES:
		spl06.i32kP = 1572864;
		break;
	case PRESSURE_RATE_4_TIMES:
		spl06.i32kP = 3670016;
		break;
	case PRESSURE_RATE_8_TIMES:
		spl06.i32kP = 7864320;
		break;
	case PRESSURE_RATE_16_TIMES:
		spl06.i32kP = 253952;
		break;
	case PRESSURE_RATE_32_TIMES:
		spl06.i32kP = 516096;
		break;
	case PRESSURE_RATE_64_TIMES:
		spl06.i32kP = 1040384;
		break;
	case PRESSURE_RATE_128_TIMES:
		spl06.i32kP = 2088960;
		break;
	case PRESSURE_RATE_1_TIMES:
	default:
		spl06.i32kP = 524288;
		break;
	}
	spl0601_write(HW_ADR, PRS_CFG_REG, data);// write configuration
}

/***********************************************************************
 * Temperature Sampling Rate Configuration
 * @param[in] background_rate: Sampling frequency N times/second (Only used in background mode, 
 * the sampling frequency of command mode is determined by the user, the minimum interval depends on the time required 
 * for oversampling, and different oversampling times can have different power consumption)
 * @param[in] oversamply: Oversampling times
 * @param[in] ext: Thermometer selection
 * @param[out]
 * @return
 **********************************************************************/
#define TEMPERATURE_RATE_1_TIMES 0 // Sampling Rate
#define TEMPERATURE_RATE_2_TIMES 1
#define TEMPERATURE_RATE_4_TIMES 2
#define TEMPERATURE_RATE_8_TIMES 3
#define TEMPERATURE_RATE_16_TIMES 4
#define TEMPERATURE_RATE_32_TIMES 5
#define TEMPERATURE_RATE_64_TIMES 6
#define TEMPERATURE_RATE_128_TIMES 7
#define TEMPERATURE_RATE_TMP_EXT_INTERNAL 0  // thermometer on integrated circuit
#define TEMPERATURE_RATE_TMP_EXT_EXTERNAL 1  // Sensor MEMS Barometric Pressure On-Chip Thermometer
void spl06_temperature_rate_config(u8 background_rate, u8 oversamply, u8 ext)
{
	u8 data;

	data = (ext << 7) | (background_rate << 4) | oversamply;
	if (oversamply > TEMPERATURE_RATE_8_TIMES)// Oversampling times greater than EMPERATURE_RATE_8_TIMES, it should allow data to be overwritten with new data
	{
		u8 data;
		data = spl0601_read(HW_ADR, CFG_REG);
		data |= 0X08;
		spl0601_write(HW_ADR, CFG_REG, data);
	}
	switch (oversamply)
	{
	case TEMPERATURE_RATE_2_TIMES:
		spl06.i32kT = 1572864;
		break;
	case TEMPERATURE_RATE_4_TIMES:
		spl06.i32kT = 3670016;
		break;
	case TEMPERATURE_RATE_8_TIMES:
		spl06.i32kT = 7864320;
		break;
	case TEMPERATURE_RATE_16_TIMES:
		spl06.i32kT = 253952;
		break;
	case TEMPERATURE_RATE_32_TIMES:
		spl06.i32kT = 516096;
		break;
	case TEMPERATURE_RATE_64_TIMES:
		spl06.i32kT = 1040384;
		break;
	case TEMPERATURE_RATE_128_TIMES:
		spl06.i32kT = 2088960;
		break;
	case TEMPERATURE_RATE_1_TIMES:
	default:
		spl06.i32kT = 524288;
		break;
	}
	spl0601_write(HW_ADR, TMP_CFG_REG, data);// write configuration
}
/***********************************************************************
 * Sensor measurement mode configuration and status reading
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
#define MEAS_CFG_COEF_RDY 0X80 // The internal calibration value of the sensor is readable, after start-up
#define MEAS_CFG_SENSOR_RDY 0X40 // The sensor has been initialized, after startup
#define MEAS_CFG_TMP_RDY 0x20 // The temperature value is ready to be read, and the flag is automatically cleared to 0 after being read
#define MEAS_CFG_PRS_RDY 0x10 // The air pressure value is ready to be read, the flag bit
#define MEAS_CFG_MEAS_CTR_STANDBY 0 // Mode Configuration: Suspend Mode
#define MEAS_CFG_MEAS_CTR_COMMAND_PRS 0x01 // Mode configuration: Start barometric pressure collection in command mode
#define MEAS_CFG_MEAS_CTR_COMMAND_TMP 0x02 // Mode configuration: Start temperature acquisition in command mode
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PRS 0x05 // Mode configuration: Background mode only reads barometric pressure
#define MEAS_CFG_MEAS_CTR_BACKGROUND_TMP 0X06 // Mode configuration: Background mode only reads temperature values
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP 0X07 //Mode configuration: Background mode reads temperature and barometric pressure values at the same time
 // Get sensor startup status and data in-place status
u8 spl06_get_measure_status(void)
{
	return spl0601_read(HW_ADR, MEAS_CFG_REG);
}
// mode + read mode
void spl06_set_measure_mode(u8 mode)
{
	spl0601_write(HW_ADR, MEAS_CFG_REG, mode);
}
// Command mode to read temperature value
void spl06_start_temperature(void)
{
	spl0601_write(HW_ADR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_COMMAND_TMP);
}
// Command mode to read barometric pressure
void spl06_start_pressure(void)
{
	spl0601_write(HW_ADR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_COMMAND_PRS);
}
// enter suspend mode
void spl06_enter_standby(void)
{
	spl0601_write(HW_ADR, MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_STANDBY);
}

/***********************************************************************
 * Interrupt and FIFO configuration, SPI mode configuration
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
#define CFG_INT_LEVEL_ACTIVE_LOW 0// Interrupt active low
#define CFG_INT_LEVEL_ACTIVE_HIGH 1// Interrupt active high
#define CFG_INT_FIFO 0X40    // Enable interrupt when FIFO is full
#define CFG_INT_PRS 0X20    // Enable interrupt when barometer reading is complete
#define CFG_INT_TMP 0X10    // Enable interrupt when temperature reading is complete 
#define CFG_T_SHIFT 0X08    // Allow the data to be overwritten, and the next collection can be performed
#define CFG_P_SHIFT 0X04    // Allow the data to be overwritten, and the next collection can be performed
#define CFG_FIF 0X02    // enable FIFO
#define CFG_SPI_3_WIRE 1    // 3-wire SPI
#define CFG_SPI_4_WIRE 0    // 4-wire SPI

void spl06_set_interrupt(u8 interrupt, u8 type)// set interrupt enable
{
	u8 data;
	data = spl0601_read(HW_ADR, CFG_REG);
	if (type != ENABLE)
		data &= ~interrupt;
	else
		data |= interrupt;
	spl0601_write(HW_ADR, CFG_REG, data);
}

void spl06_set_spi_wire(u8 wire)// Set the number of SPI lines
{
	u8 data;
	data = spl0601_read(HW_ADR, CFG_REG);
	data &= 0xf7;// The bit where the SPI line is configured is cleared to 0
	data |= wire;
	spl0601_write(HW_ADR, CFG_REG, data);
}

void spl06_set_intrupt_level(u8 level)// Set interrupt active level
{
	u8 data;
	data = spl0601_read(HW_ADR, CFG_REG);
	data &= 0x7f;// Interrupt level active bit is cleared to 0
	data |= level << 7;
	spl0601_write(HW_ADR, CFG_REG, data);
}

/***********************************************************************
 * Interrupt status acquisition. Set to 1 by hardware, and a related pin interrupt occurs, read this register and clear it to 0 automatically
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
#define INT_STS_FIFO_FULL  0X04 // FIFO full interrupt status
#define INT_STS_FIFO_TMP   0X02  // Temperature measurement complete flag
#define INT_STS_FIFO_PRS  0X01  // Barometric pressure measurement completion flag

u8 spl06_get_int_status(void)
{
	return spl0601_read(HW_ADR, INT_STS_REG);
}

/***********************************************************************
 * FIFO status acquisition
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
#define FIFO_STS_FULL  0X02 //FIFO full
#define FIFO_STS_EMPTY   0X01 //FIFO empty
u8 spl06_get_fifo_status(void)
{
	return spl0601_read(HW_ADR, FIFO_STS_REG);
}
/***********************************************************************
 * reset
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
#define RESET_FIFO_FLUSH 0X80 // FIFO is cleared to 0
#define RESET_SOFT 0X09// software reset

void spl06_soft_reset(void)
{
	spl0601_write(HW_ADR, RESET_REG, RESET_SOFT);
}
void spl06_reset_fifo(void)
{
	spl0601_write(HW_ADR, RESET_REG, RESET_FIFO_FLUSH);
}
/***********************************************************************
 * ID
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
#define PRODUCT_ID 0X10// Product ID
u8 spl06_get_chip_id(void)
{
	return spl0601_read(HW_ADR, ID_REG);
}

/***********************************************************************
 * Get the calibration parameters inside the barometer
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void spl0601_get_calib_param(void)
{
	unsigned long h;
	unsigned long m;
	unsigned long l;
	h = spl0601_read(HW_ADR, 0x10);
	l = spl0601_read(HW_ADR, 0x11);
	spl06_calib_param.c0 = (int16)h << 4 | l >> 4;
	spl06_calib_param.c0 = (spl06_calib_param.c0 & 0x0800) ? (0xF000 | spl06_calib_param.c0) : spl06_calib_param.c0;
	h = spl0601_read(HW_ADR, 0x11);
	l = spl0601_read(HW_ADR, 0x12);
	spl06_calib_param.c1 = (int16)(h & 0x0F) << 8 | l;
	spl06_calib_param.c1 = (spl06_calib_param.c1 & 0x0800) ? (0xF000 | spl06_calib_param.c1) : spl06_calib_param.c1;
	h = spl0601_read(HW_ADR, 0x13);
	m = spl0601_read(HW_ADR, 0x14);
	l = spl0601_read(HW_ADR, 0x15);
	spl06_calib_param.c00 = (int32)h << 12 | (int32)m << 4 | (int32)l >> 4;
	spl06_calib_param.c00 = (spl06_calib_param.c00 & 0x080000) ? (0xFFF00000 | spl06_calib_param.c00) : spl06_calib_param.c00;
	h = spl0601_read(HW_ADR, 0x15);
	m = spl0601_read(HW_ADR, 0x16);
	l = spl0601_read(HW_ADR, 0x17);
	spl06_calib_param.c10 = (int32)h << 16 | (int32)m << 8 | l;
	spl06_calib_param.c10 = (spl06_calib_param.c10 & 0x080000) ? (0xFFF00000 | spl06_calib_param.c10) : spl06_calib_param.c10;
	h = spl0601_read(HW_ADR, 0x18);
	l = spl0601_read(HW_ADR, 0x19);
	spl06_calib_param.c01 = (int16)h << 8 | l;
	h = spl0601_read(HW_ADR, 0x1A);
	l = spl0601_read(HW_ADR, 0x1B);
	spl06_calib_param.c11 = (int16)h << 8 | l;
	h = spl0601_read(HW_ADR, 0x1C);
	l = spl0601_read(HW_ADR, 0x1D);
	spl06_calib_param.c20 = (int16)h << 8 | l;
	h = spl0601_read(HW_ADR, 0x1E);
	l = spl0601_read(HW_ADR, 0x1F);
	spl06_calib_param.c21 = (int16)h << 8 | l;
	h = spl0601_read(HW_ADR, 0x20);
	l = spl0601_read(HW_ADR, 0x21);
	spl06_calib_param.c30 = (int16)h << 8 | l;
}
/***********************************************************************
 * initialization
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
u8 spl0601_init(void)
{
	u8 spl06_start_status;
	
	/* Software IIC initialization */
	I2c_Soft_Init();
	spl06_delay(10);
	
	do
		spl06_start_status = spl06_get_measure_status();// Read barometer activation status
	while ((spl06_start_status&MEAS_CFG_COEF_RDY) != MEAS_CFG_COEF_RDY);// Wait for internal calibration data to be available
	spl0601_get_calib_param();	// Read internal calibration value
	do
		spl06_start_status = spl06_get_measure_status();// Read barometer activation status
	while ((spl06_start_status&MEAS_CFG_SENSOR_RDY) != MEAS_CFG_SENSOR_RDY);// Wait for the internal initialization of the sensor to complete

	spl06.chip_id = spl06_get_chip_id();// Read CHIP ID
	if ((spl06.chip_id & 0xf0) != PRODUCT_ID)
		return FAILED;

	spl06_pressure_rate_config(PRESSURE_RATE_128_TIMES, PRESSURE_RATE_32_TIMES);// Background data sampling rate 128HZ, oversampling rate 32 times
	spl06_temperature_rate_config(TEMPERATURE_RATE_32_TIMES, TEMPERATURE_RATE_8_TIMES, TEMPERATURE_RATE_TMP_EXT_EXTERNAL);// Background data sampling rate 32HZ, oversampling rate 8 times
	spl06_set_measure_mode(MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP);// Start background reading data
	return SUCCESS;
}


/***********************************************************************
 * Get raw temperature value
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void spl0601_get_raw_temp(void)
{
	uint8 h[3] = { 0 };

	h[0] = spl0601_read(HW_ADR, 0x03);
	h[1] = spl0601_read(HW_ADR, 0x04);
	h[2] = spl0601_read(HW_ADR, 0x05);

	spl06.i32rawTemperature = (int32)h[0] << 16 | (int32)h[1] << 8 | (int32)h[2];
	spl06.i32rawTemperature = (spl06.i32rawTemperature & 0x800000) ? (0xFF000000 | spl06.i32rawTemperature) : spl06.i32rawTemperature;
}

/***********************************************************************
 * Get the original air pressure value
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void spl0601_get_raw_pressure(void)
{
	uint8 h[3];

	h[0] = spl0601_read(HW_ADR, 0x00);
	h[1] = spl0601_read(HW_ADR, 0x01);
	h[2] = spl0601_read(HW_ADR, 0x02);

	spl06.i32rawPressure = (int32)h[0] << 16 | (int32)h[1] << 8 | (int32)h[2];
	spl06.i32rawPressure = (spl06.i32rawPressure & 0x800000) ? (0xFF000000 | spl06.i32rawPressure) : spl06.i32rawPressure;
}


/***********************************************************************
 * temperature value
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float spl0601_get_temperature(void)
{
	float fTCompensate;
	float fTsc;

	fTsc = spl06.i32rawTemperature / (float)spl06.i32kT;
	fTCompensate = spl06_calib_param.c0 * 0.5 + spl06_calib_param.c1 * fTsc;
	return fTCompensate;
}

/***********************************************************************
 * Air pressure calculation and temperature compensation
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float spl0601_get_pressure(void)
{
	float fTsc, fPsc;
	float qua2, qua3;
	float fPCompensate;

	fTsc = spl06.i32rawTemperature / (float)spl06.i32kT;
	fPsc = spl06.i32rawPressure / (float)spl06.i32kP;
	qua2 = spl06_calib_param.c10 + fPsc * (spl06_calib_param.c20 + fPsc * spl06_calib_param.c30);
	qua3 = fTsc * fPsc * (spl06_calib_param.c11 + fPsc * spl06_calib_param.c21);
	//qua3 = 0.9f *fTsc * fPsc * (spl06_calib_param.c11 + fPsc * spl06_calib_param.c21);

	fPCompensate = spl06_calib_param.c00 + fPsc * qua2 + fTsc * spl06_calib_param.c01 + qua3;
	//fPCompensate = spl06_calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06_calib_param.c01 + qua3;
	return fPCompensate;
}



/***********************************************************************
 * Get temperature value
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float user_spl0601_get_temperature()
{
	spl0601_get_raw_temp();// Read temperature raw value
	return spl0601_get_temperature();// Temperature calculated value
}
/***********************************************************************
 * Get the barometer temperature compensation value
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float user_spl0601_get_presure()
{
	spl0601_get_raw_pressure();// Read the original air pressure value
	return spl0601_get_pressure();	// Air pressure calculated and temperature compensated
}
#elif  USE_SPI_FOR_SPL06

/****************************************************************************//**
 *
 *         APP global variables
 *
 *******************************************************************************/
extern SPI_HandleTypeDef SPL06_SPI;

/****************************************************************************//**
 *
 *         MACRO
 *
 *******************************************************************************/

#undef SUCCESS 
#define SUCCESS 0
#undef FAILED 
#define FAILED 1

//*****Definition for Inner registers*****
//1. Pressure Data
#define PRS_B2 0X00
#define PRS_B1 0X01
#define PRS_B0 0X02
//2. Temperature Data
#define TMP_B2 0X03
#define TMP_B1 0X04
#define TMP_B0 0X05
//3. Pressure Configuration
#define PRS_CFG 0x06  
//Field:
#define PM_RATE 0x70
#define PM_PRC  0x0F
//4. Temperature Configuration
#define TMP_CFG 0x07  
//Field:
#define TMP_EXT  0x80
#define TMP_RATE 0x70
#define TMP_PRC  0x07
//5. Sensor Operating Mode and Status
#define MEAS_CFG 0x08  
//Field:
#define COEF_RDY   0x80
#define SENSOR_RDY 0x40
#define TMP_RDY    0x20
#define PRS_RDY    0x10
#define MEAS_CTRL  0x07
//6. Interrupt and FIFO configuration
#define CFG_REG 0x09  
//Field:
#define INT_HL   0x80
#define INT_FIFO 0x40
#define INT_PRS  0x20
#define INT_TMP  0x10
#define T_SHIFT  0x08
#define P_SHIFT  0x04
#define FIFO_EN  0x02
#define SPI_MODE 0x01
//7. Interrupt Status
#define INT_STS 0X0A 
//Field:
#define INT_FIFO_FULL  0x04
#define INT_TMP_ACTIVE 0x02
#define INT_PRS_ACTIVE 0x01
//8. FIFO Status
#define FIFO_STS 0X0B  
//Field:
#define FIFO_FULL  0x02
#define FIFO_EMPTY 0x01
//9. Soft Reset and FIFO flush
#define RESET 0X0C
//Field:
#define FIFO_FLUSH 0x80
#define SOFT_RST   0x0F   //Write '1001' to generate a soft reset
//10. Product and Revision ID
#define ID 0x0D
//Field:
#define PROD_ID 0xF0
#define REV_ID  0x0F
//11. Calibration Coefficients
#define COEFG	0x10
#define C0		0x10
#define C0_C1	0x11
#define C1		0x12
#define C00_H   0x13
#define C00_M   0x14
#define C00_C10 0x15
#define C10_H	0x16
#define C10_L	0x17
#define C01_H	0x18
#define C01_L	0x19
#define C11_H	0x1A
#define C11_L	0x1B
#define C20_H	0x1C
#define C20_L	0x1D
#define C21_H	0x1E
#define C21_L	0x1F
#define C30_H	0x20
#define C30_L	0x21
//Field:
#define C0_C1_H4   0xF0
#define C0_C1_L4   0x0F
#define C00_C10_H4 0xF0
#define C00_C10_L4 0x0F
//*****End for Definition*****
//*****Pressure measurement rate*****
#define PRESSURE_MEASUREMENT_RATE_1_TIMES	(0<<4) 
#define PRESSURE_MEASUREMENT_RATE_2_TIMES	(1<<4)
#define PRESSURE_MEASUREMENT_RATE_4_TIMES	(2<<4)
#define PRESSURE_MEASUREMENT_RATE_8_TIMES	(3<<4)
#define PRESSURE_MEASUREMENT_RATE_16_TIMES  (4<<4)
#define PRESSURE_MEASUREMENT_RATE_32_TIMES  (5<<4)
#define PRESSURE_MEASUREMENT_RATE_64_TIMES  (6<<4)
#define PRESSURE_MEASUREMENT_RATE_128_TIMES (7<<4)
//*****End for Pressure measurement rate*****
//*****Pressure oversampling rate*****
#define PRESSURE_OVERSAMPLING_RATE_1_TIMES 	 0
#define PRESSURE_OVERSAMPLING_RATE_2_TIMES	 1
#define PRESSURE_OVERSAMPLING_RATE_4_TIMES	 2
#define PRESSURE_OVERSAMPLING_RATE_8_TIMES	 3
#define PRESSURE_OVERSAMPLING_RATE_16_TIMES  4
#define PRESSURE_OVERSAMPLING_RATE_32_TIMES  5
#define PRESSURE_OVERSAMPLING_RATE_64_TIMES  6
#define PRESSURE_OVERSAMPLING_RATE_128_TIMES 7
//*****End for Pressure oversampling rate*****
//*****Temperature measurement rate*****
#define TEMPERATURE_MEASUREMENT_RATE_1_TIMES   (0<<4) 
#define TEMPERATURE_MEASUREMENT_RATE_2_TIMES   (1<<4)
#define TEMPERATURE_MEASUREMENT_RATE_4_TIMES   (2<<4)
#define TEMPERATURE_MEASUREMENT_RATE_8_TIMES   (3<<4)
#define TEMPERATURE_MEASUREMENT_RATE_16_TIMES  (4<<4)
#define TEMPERATURE_MEASUREMENT_RATE_32_TIMES  (5<<4)
#define TEMPERATURE_MEASUREMENT_RATE_64_TIMES  (6<<4)
#define TEMPERATURE_MEASUREMENT_RATE_128_TIMES (7<<4)
//*****End for Temperature measurement rate*****
//*****Temperature oversampling rate*****
#define TEMPERATURE_OVERSAMPLING_RATE_1_TIMES 	0
#define TEMPERATURE_OVERSAMPLING_RATE_2_TIMES	1
#define TEMPERATURE_OVERSAMPLING_RATE_4_TIMES	2
#define TEMPERATURE_OVERSAMPLING_RATE_8_TIMES	3
#define TEMPERATURE_OVERSAMPLING_RATE_16_TIMES  4
#define TEMPERATURE_OVERSAMPLING_RATE_32_TIMES  5
#define TEMPERATURE_OVERSAMPLING_RATE_64_TIMES  6
#define TEMPERATURE_OVERSAMPLING_RATE_128_TIMES 7
//*****End for Temperature oversampling rate*****

/****************************************************************************//**
 *
 *         MACRO function
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *         Private variables and function prototypes
 *
 *******************************************************************************/
static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

static void  Drv_SPL06CSPin_Init(void);
static void  spl06_enable(u8 ena);
static void  spl0601_write(unsigned char regadr, unsigned char val);
static u8    spl0601_read(unsigned char regadr);
static void  spl0601_rateset(u8 iSensor, u8 u8SmplRate, u8 u8OverSmpl);
static void  spl0601_get_calib_param(void);
static void  spl0601_start_continuous(u8 mode);
static void  spl0601_get_raw_temp(void);
static void  spl0601_get_raw_pressure(void);
static float spl0601_get_temperature(void);
static float spl0601_get_pressure(void);
static u8 spl06_get_measure_status(void);
static void spl06_initialize_cfg(void);
static void spl06_delay(uint32 _t);
static uint8 spl06_spi_readwritebyte(u8 TxData);



s32 baro_height = 0;
s32 baro_pa = 0;
float baro_Offset = 0;
float baro_temperature = 0;
unsigned char baro_start;


/****************************************************************************//**
 *
 *         SPL06-001 function section
 *
 *******************************************************************************/
float Drv_Spl0601_Read(void)
{
	float baro_pressure, alt_3, height, alt_high;

	spl0601_get_raw_temp();
	baro_temperature = spl0601_get_temperature();  //Celsius

	spl0601_get_raw_pressure();
	baro_pressure = spl0601_get_pressure();  //Pa

	/////////////////////////////////////////////////////////////
	baro_pa = 101000 - baro_pressure;

	alt_3 = (101000 - baro_pressure) / 1000.0f;
	height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * (101000 - baro_pressure) * 100.0f;
	alt_high = (height - baro_Offset); //cm +

 
	//alt_3 = (102000.0f - baro_pressure) * 78.740f;
	//alt_high = (alt_3 - baro_Offset); //cm +

	/////////////////////////////////////////////////////////////
	return alt_high;
}

unsigned char Drv_Spl0601_Offset(unsigned char _times)
{
	unsigned char i = _times, j;
	float _baro_temp, _baro_sum;
	
	if(i > 255)
		return FAILED;
	
	baro_Offset = 0;
	_baro_temp = 0;
	for(j = 0; j < i; j++)
	{
		_baro_temp += Drv_Spl0601_Read();
		spl06_delay(100);
	}
	
	_baro_sum = 0;
	_baro_sum = _baro_temp/i;
	
	baro_Offset = _baro_sum;
	return SUCCESS;
}


u8 Drv_Spl0601_Init(void)
{
#define T_COEF_RDY_MS   80  //40*2
#define T_SENSOR_RDY_MS 12  

	u8 spl06_start_status;
	u32 time;

//	Drv_SPL06CSPin_Init();

	p_spl0601 = &spl0601; /* read Chip Id */
	p_spl0601->i32rawPressure = 0;
	p_spl0601->i32rawTemperature = 0;
	
	//p_spl0601->chip_id = spl0601_read(ID);// 0x34  0x10 
	HAL_GPIO_WritePin(SENSOR_NSS3_GPIO_Port, SENSOR_NSS3_Pin, GPIO_PIN_RESET);
	spl06_spi_readwritebyte(ID | 0x80);	  //0x80:R/W bit(0->write 1->read)
	p_spl0601->chip_id = spl06_spi_readwritebyte(0xff);
	HAL_GPIO_WritePin(SENSOR_NSS3_GPIO_Port, SENSOR_NSS3_Pin, GPIO_PIN_SET);
	
	//p_spl0601->chip_id = spl0601_read(0x00);//AK nss2 Device ID = 0x48
	HAL_GPIO_WritePin(SENSOR_NSS2_GPIO_Port, SENSOR_NSS2_Pin, GPIO_PIN_RESET);
	spl06_spi_readwritebyte(0x00 | 0x80);	  //0x80:R/W bit(0->write 1->read)
	p_spl0601->chip_id = spl06_spi_readwritebyte(0xff);
	HAL_GPIO_WritePin(SENSOR_NSS2_GPIO_Port, SENSOR_NSS2_Pin, GPIO_PIN_SET);
	
	//p_spl0601->chip_id = spl0601_read(0x75);//icm nss 0x12
	HAL_GPIO_WritePin(SENSOR_NSS_GPIO_Port, SENSOR_NSS_Pin, GPIO_PIN_RESET);
	spl06_spi_readwritebyte(0x75 | 0x80);	  //0x80:R/W bit(0->write 1->read)
	p_spl0601->chip_id = spl06_spi_readwritebyte(0xff);
	HAL_GPIO_WritePin(SENSOR_NSS_GPIO_Port, SENSOR_NSS_Pin, GPIO_PIN_SET);
	
	spl06_start_status = 0;
	time = 0;
	do
	{
		if (time > T_COEF_RDY_MS)
			break;
		spl06_start_status = spl06_get_measure_status();
		time++;
		spl06_delay(1);
	}
	while ((spl06_start_status&COEF_RDY) != COEF_RDY);
	if(time > T_COEF_RDY_MS)
		return FAILED;

	spl0601_get_calib_param();

	spl06_start_status = 0;
	time = 0;
	do
	{
		if (time > T_SENSOR_RDY_MS)
			break;
		spl06_start_status = spl06_get_measure_status();
		time++;
		spl06_delay(1);
	}
	while ((spl06_start_status&SENSOR_RDY) != SENSOR_RDY);
	if (time > T_SENSOR_RDY_MS)
		return FAILED;

	spl06_initialize_cfg();

	//spl0601_rateset(PRESSURE_SENSOR, 128, 16);

	//spl0601_rateset(TEMPERATURE_SENSOR, 8, 8);

	spl0601_rateset(PRESSURE_SENSOR, PRESSURE_MEASUREMENT_RATE_128_TIMES, PRESSURE_OVERSAMPLING_RATE_32_TIMES);

	spl0601_rateset(TEMPERATURE_SENSOR, TEMPERATURE_MEASUREMENT_RATE_32_TIMES, TEMPERATURE_OVERSAMPLING_RATE_8_TIMES);

	spl0601_start_continuous(CONTINUOUS_P_AND_T);

	if (p_spl0601->chip_id == 0x10)
	{
		return SUCCESS;
	}
	else
	{
		return FAILED;
	}
}

void Drv_spl06_soft_reset(void)
{
	spl0601_write(RESET, 0X09);
}


static void Drv_SPL06CSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	SPL06_CS_RCC;
	GPIO_Initure.Pin = SPL06_CS_PIN;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;     // Push-pull output
	GPIO_Initure.Pull = GPIO_PULLUP;             // pull up
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;         // high speed
	HAL_GPIO_Init(SPL06_CS_GPIO, &GPIO_Initure);

	HAL_GPIO_WritePin(SPL06_CS_GPIO, SPL06_CS_PIN, GPIO_PIN_SET);
}


static void spl06_enable(u8 ena)
{
	if (ena)
		HAL_GPIO_WritePin(SPL06_CS_GPIO, SPL06_CS_PIN, GPIO_PIN_RESET);//GPIO_ResetBits(SPL06_CS_GPIO, SPL06_CS_PIN);
	else
		HAL_GPIO_WritePin(SPL06_CS_GPIO, SPL06_CS_PIN, GPIO_PIN_SET);//GPIO_SetBits(SPL06_CS_GPIO, SPL06_CS_PIN);
}


static void spl0601_write(unsigned char regadr, unsigned char val)
{
	spl06_enable(1);
	spl06_spi_readwritebyte(regadr);
	spl06_spi_readwritebyte(val);
	spl06_enable(0);
}

static u8 spl0601_read(unsigned char regadr)
{
	u8 reg_data;
	spl06_enable(1);
	spl06_spi_readwritebyte(regadr | 0x80);	  //0x80:R/W bit(0->write 1->read)
	reg_data = spl06_spi_readwritebyte(0xff);
	spl06_enable(0);
	return reg_data;
}

static void spl0601_rateset(u8 iSensor, u8 u8SmplRate, u8 u8OverSmpl)
{
	u8 reg = 0;
	int32_t i32kPkT = 0;

	//switch (u8SmplRate)
	//{
	//case 2:
	//	reg |= (1 << 4);
	//	break;
	//case 4:
	//	reg |= (2 << 4);
	//	break;
	//case 8:
	//	reg |= (3 << 4);
	//	break;
	//case 16:
	//	reg |= (4 << 4);
	//	break;
	//case 32:
	//	reg |= (5 << 4);
	//	break;
	//case 64:
	//	reg |= (6 << 4);
	//	break;
	//case 128:
	//	reg |= (7 << 4);
	//	break;
	//case 1:
	//default:
	//	break;
	//}
	reg |= u8SmplRate + u8OverSmpl;


	//switch (u8OverSmpl)
	//{
	//case 2:
	//	reg |= 1;
	//	i32kPkT = 1572864;
	//	break;
	//case 4:
	//	reg |= 2;
	//	i32kPkT = 3670016;
	//	break;
	//case 8:
	//	reg |= 3;
	//	i32kPkT = 7864320;
	//	break;
	//case 16:
	//	i32kPkT = 253952;
	//	reg |= 4;
	//	break;
	//case 32:
	//	i32kPkT = 516096;
	//	reg |= 5;
	//	break;
	//case 64:
	//	i32kPkT = 1040384;
	//	reg |= 6;
	//	break;
	//case 128:
	//	i32kPkT = 2088960;
	//	reg |= 7;
	//	break;
	//case 1:
	//default:
	//	i32kPkT = 524288;
	//	break;
	//}

	if (iSensor == PRESSURE_SENSOR)
	{
		switch (u8OverSmpl)
		{
		case PRESSURE_OVERSAMPLING_RATE_2_TIMES:
			i32kPkT = 1572864;
			break;
		case PRESSURE_OVERSAMPLING_RATE_4_TIMES:
			i32kPkT = 3670016;
			break;
		case PRESSURE_OVERSAMPLING_RATE_8_TIMES:
			i32kPkT = 7864320;
			break;
		case PRESSURE_OVERSAMPLING_RATE_16_TIMES:
			i32kPkT = 253952;
			break;
		case PRESSURE_OVERSAMPLING_RATE_32_TIMES:
			i32kPkT = 516096;
			break;
		case PRESSURE_OVERSAMPLING_RATE_64_TIMES:
			i32kPkT = 1040384;
			break;
		case PRESSURE_OVERSAMPLING_RATE_128_TIMES:
			i32kPkT = 2088960;
			break;
		case PRESSURE_OVERSAMPLING_RATE_1_TIMES:
		default:
			i32kPkT = 524288;
			break;
		}

		p_spl0601->i32kP = i32kPkT;
		spl0601_write(PRS_CFG, reg);
		if (u8OverSmpl > PRESSURE_OVERSAMPLING_RATE_8_TIMES)
		{
			reg = spl0601_read(CFG_REG);
			spl0601_write(CFG_REG, reg | P_SHIFT);
		}
	}
	if (iSensor == TEMPERATURE_SENSOR)
	{
		switch (u8OverSmpl)
		{
		case TEMPERATURE_OVERSAMPLING_RATE_2_TIMES:
			i32kPkT = 1572864;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_4_TIMES:
			i32kPkT = 3670016;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_8_TIMES:
			i32kPkT = 7864320;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_16_TIMES:
			i32kPkT = 253952;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_32_TIMES:
			i32kPkT = 516096;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_64_TIMES:
			i32kPkT = 1040384;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_128_TIMES:
			i32kPkT = 2088960;
			break;
		case TEMPERATURE_OVERSAMPLING_RATE_1_TIMES:
		default:
			i32kPkT = 524288;
			break;
		}

		p_spl0601->i32kT = i32kPkT;
		spl0601_write(TMP_CFG, reg | TMP_EXT); //Using mems temperature
		if (u8OverSmpl > TEMPERATURE_OVERSAMPLING_RATE_8_TIMES)
		{
			reg = spl0601_read(CFG_REG);
			spl0601_write(CFG_REG, reg | T_SHIFT);
		}
	}

}

static void spl0601_get_calib_param(void)
{
	uint32 h;
	uint32 m;
	uint32 l;
	h = spl0601_read(C0);
	l = spl0601_read(C0_C1);
	p_spl0601->calib_param.c0 = (int16_t)h << 4 | l >> 4;
	p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0 & 0x0800) ? (0xF000 | p_spl0601->calib_param.c0) : p_spl0601->calib_param.c0;
	h = spl0601_read(C0_C1);
	l = spl0601_read(C1);
	p_spl0601->calib_param.c1 = (int16_t)(h & C0_C1_L4) << 8 | l;
	p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1 & 0x0800) ? (0xF000 | p_spl0601->calib_param.c1) : p_spl0601->calib_param.c1;
	h = spl0601_read(C00_H);
	m = spl0601_read(C00_M);
	l = spl0601_read(C00_C10);
	p_spl0601->calib_param.c00 = (int32_t)h << 12 | (int32_t)m << 4 | (int32_t)l >> 4;
	p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00 & 0x080000) ? (0xFFF00000 | p_spl0601->calib_param.c00) : p_spl0601->calib_param.c00;
	h = spl0601_read(C00_C10);
	m = spl0601_read(C10_H);
	l = spl0601_read(C10_L);
	p_spl0601->calib_param.c10 = (int32_t)h << 16 | (int32_t)m << 8 | l;
	p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10 & 0x080000) ? (0xFFF00000 | p_spl0601->calib_param.c10) : p_spl0601->calib_param.c10;
	h = spl0601_read(C01_H);
	l = spl0601_read(C01_L);
	p_spl0601->calib_param.c01 = (int16_t)h << 8 | l;
	h = spl0601_read(C11_H);
	l = spl0601_read(C11_L);
	p_spl0601->calib_param.c11 = (int16_t)h << 8 | l;
	h = spl0601_read(C20_H);
	l = spl0601_read(C20_L);
	p_spl0601->calib_param.c20 = (int16_t)h << 8 | l;
	h = spl0601_read(C21_H);
	l = spl0601_read(C21_L);
	p_spl0601->calib_param.c21 = (int16_t)h << 8 | l;
	h = spl0601_read(C30_H);
	l = spl0601_read(C30_L);
	p_spl0601->calib_param.c30 = (int16_t)h << 8 | l;
}

//static void spl0601_start_temperature(void)
//{
//	spl0601_write(0x08, 0x02);
//}

//static void spl0601_start_pressure(void)
//{
//	spl0601_write(0x08, 0x01);
//}

static void spl0601_start_continuous(u8 mode)
{
	spl0601_write(MEAS_CFG, mode + 4);
}

static void spl0601_get_raw_temp(void)
{
	u8 h[3] = { 0 };

	h[0] = spl0601_read(TMP_B2);
	h[1] = spl0601_read(TMP_B1);
	h[2] = spl0601_read(TMP_B0);

	p_spl0601->i32rawTemperature = (int32_t)h[0] << 16 | (int32_t)h[1] << 8 | (int32_t)h[2];
	p_spl0601->i32rawTemperature = (p_spl0601->i32rawTemperature & 0x800000) ? (0xFF000000 | p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}


static void spl0601_get_raw_pressure(void)
{
	u8 h[3];

	h[0] = spl0601_read(PRS_B2);
	h[1] = spl0601_read(PRS_B1);
	h[2] = spl0601_read(PRS_B0);

	p_spl0601->i32rawPressure = (int32_t)h[0] << 16 | (int32_t)h[1] << 8 | (int32_t)h[2];
	p_spl0601->i32rawPressure = (p_spl0601->i32rawPressure & 0x800000) ? (0xFF000000 | p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}


static float spl0601_get_temperature(void)
{
	float fTCompensate;
	float fTsc;

	fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
	fTCompensate = p_spl0601->calib_param.c0 * 0.5f + p_spl0601->calib_param.c1 * fTsc;  //Celsius
	return fTCompensate;
}

static float spl0601_get_pressure(void)
{
	float fTsc, fPsc;
	float qua2, qua3;
	float fPCompensate;

	fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
	fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
	qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30);
	qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

	fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;  //Pa
	return fPCompensate;
}

static u8 spl06_get_measure_status(void)
{
	return spl0601_read(MEAS_CFG);
}

static void spl06_initialize_cfg(void)
{
	u8 reg;
	reg = spl0601_read(CFG_REG);
	reg = reg & ((~INT_FIFO)&(~INT_PRS)&(~INT_TMP)&(~FIFO_EN)&(~SPI_MODE));
	spl0601_write(CFG_REG, reg);
}

static uint8 spl06_spi_readwritebyte(u8 TxData)
{
    u8 Rxdata;
    HAL_SPI_TransmitReceive(&SPL06_SPI,&TxData,&Rxdata,1, HAL_MAX_DELAY);       
 	return Rxdata;          		 	
}


#if USE_OS_UCOS_III
static void spl06_delay(uint32 _t)
{
	OS_ERR  err;
	OSTimeDlyHMSM(0u, 0u, 0u, _t,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
}
#endif


#endif

#endif
