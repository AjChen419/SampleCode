 #ifndef __BMP180_H__ 
 #define __BMP180_H__ 
  
/*
	BMP180 I2C slave Address
*/

#define BMP180_I2C_ADDR				0x77

/* 
 *	register adress definitions 	
 */

#define BMP180_CTRL_MEAS_REG		0xF4

#define BMP180_ADC_OUT_MSB_REG		0xF6
#define BMP180_ADC_OUT_LSB_REG		0xF7
#define BMP180_ADC_OUT_XLSB_REG		0xF8

#define BMP180_SOFT_RESET_REG		0xE0
#define BMP180_CHIP_ID_REG			0xD0

#define BMP180_T_MEASURE			0x2E	// temperature measurent default setting values
#define BMP180_P_MEASURE			0x34	// pressure measurement default setting value


/*
 * Calibration coefficients Reg Address 
 */
 
#define Calibra_AC1_MSB_reg   0xAA
#define Calibra_AC1_LSB_reg		0xAB
#define Calibra_AC2_MSB_reg		0xAC
#define Calibra_AC2_LSB_reg	  0xAD
#define Calibra_AC3_MSB_reg		0xAE
#define Calibra_AC3_LSB_reg		0xAF
#define Calibra_AC4_MSB_reg		0xB0
#define Calibra_AC4_LSB_reg		0xB1
#define Calibra_AC5_MSB_reg		0xB2
#define Calibra_AC5_LSB_reg		0xB3
#define Calibra_AC6_MSB_reg		0xB4
#define Calibra_AC6_LSB_reg		0xB5
#define Calibra_B1_MSB_reg		0xB6
#define Calibra_B1_LSB_reg		0xB7
#define Calibra_B2_MSB_reg		0xB8
#define Calibra_B2_LSB_reg		0xB9
#define Calibra_MB_MSB_reg		0xBA
#define Calibra_MB_LSB_reg		0xBB
#define Calibra_MC_MSB_reg		0xBC
#define Calibra_MC_LSB_reg		0xBD
#define Calibra_MD_MSB_reg		0xBE
#define Calibra_MD_LSB_reg		0xBF
  

/* driver data */ 

struct barometer_platform_data
{
	void (*power_on)(void);
};

struct boashbmp180_data 
{ 
	struct i2c_client *client;
    struct work_struct work_barometer; 
    struct barometer_platform_data *pdata;
    char *vdda_supply; 
}; 
  
struct workqueue_struct *barometer_wq; 
 
/*
 *this structure holds all device specific calibration parameters 
 */
  
typedef struct 
{
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;      		   
} bmp180_calibration_param_t; 


 /* prototype */ 

 int BMP_i2c_read(u8 reg); 
 int BMP_i2c_write( u8 reg, int val );
 void reset_registers(void); 
 long get_up (short);
 long get_ut (void);
 #endif 
