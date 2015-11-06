/*  
  *  Title : Borameter sensor driver
  *  Date  : 2011.09.08
  *  Name  : AJ Chen
  * 
  */ 

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
 
#include <linux/earlysuspend.h>
#include <linux/input.h> 
#include <linux/workqueue.h>

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
  
  
#include "boschbmp180.h" 


 
 /* global var */ 
static struct i2c_client *boschbmp180_client = NULL;
struct class *barometer_class; 
struct device *boschbmp180_dev;
bmp180_calibration_param_t bmp_calibration ;
 
long uT = 0;
long uP = 0;
 
#define BOSCHBMP180_DRV_NAME   "BOSCHBMP180"
 
void reset_registers()
{
	BMP_i2c_write((u8)(BMP180_SOFT_RESET_REG), 0xB6);
} 
 
/*
 * Fuction: read_calibration
 * Description: get calibration. The calibrations are the nessary valus in the algorthim.
 */
void read_calibration ( void )
{
	bmp_calibration.ac1 = ( ( BMP_i2c_read( (u8)Calibra_AC1_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_AC1_LSB_reg) ) ) ;
 	bmp_calibration.ac2 = ( ( BMP_i2c_read( (u8)Calibra_AC2_MSB_reg) << 8 ) |\
 	  ( BMP_i2c_read( (u8)Calibra_AC2_LSB_reg) ) ) ;
	bmp_calibration.ac3 = ( ( BMP_i2c_read( (u8)Calibra_AC3_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_AC3_LSB_reg) ) ) ;
	bmp_calibration.ac4 = ( ( BMP_i2c_read( (u8)Calibra_AC4_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_AC4_LSB_reg) ) ) ;
	bmp_calibration.ac5 = ( ( BMP_i2c_read( (u8)Calibra_AC5_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_AC5_LSB_reg) ) ) ;
	bmp_calibration.ac6 = ( ( BMP_i2c_read( (u8)Calibra_AC6_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_AC6_LSB_reg) ) ) ;
	bmp_calibration.b1 = ( ( BMP_i2c_read( (u8)Calibra_B1_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_B1_LSB_reg) ) ) ;
	bmp_calibration.b2 = ( ( BMP_i2c_read( (u8)Calibra_B2_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_B2_LSB_reg) ) ) ;
	bmp_calibration.mb = ( ( BMP_i2c_read( (u8)Calibra_MB_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_MB_LSB_reg) ) ) ;
	bmp_calibration.mc = ( ( BMP_i2c_read( (u8)Calibra_MC_MSB_reg) << 8 ) |\
	 ( BMP_i2c_read( (u8)Calibra_MC_LSB_reg) ) ) ;
	bmp_calibration.md = ( ( BMP_i2c_read( (u8)Calibra_MD_MSB_reg) << 8 ) |\
	  ( BMP_i2c_read( (u8)Calibra_MD_LSB_reg) ) ) ;
}

/*
 * Fuction: get_ut
 * Description: get value of UT. The UT is the nessary valu in the algorthim.
 */
 
long get_ut (void)
{	 
	BMP_i2c_write( (u8) BMP180_CTRL_MEAS_REG , BMP180_T_MEASURE ) ; 
		
	usleep ( 4500 );
	uT = ( ( BMP_i2c_read( (u8)BMP180_ADC_OUT_MSB_REG) << 8 ) | \
		( BMP_i2c_read( (u8)BMP180_ADC_OUT_LSB_REG) ) ) ;
	 
	return uT;
}
 
/*
 * Fuction: get_up
 * Description: get value of UP. The UP is the nessary valu in the algorthim.
 */ 
 
long get_up (short oss)
{
	long err;
	 
	err = BMP_i2c_write ( (u8) BMP180_CTRL_MEAS_REG , ( BMP180_P_MEASURE | ( oss << 6 ) ) );		 
	 
	if ( !(err == 0) )
		return 0;
		
 	if ( oss == 0 )
		usleep (4500);
	else if ( oss == 1)
		usleep (7500);
	else if ( oss == 2)
		usleep (13500); 
	else if ( oss == 3)
		usleep (25500);
	else;
				
	uP = (( int )( ( BMP_i2c_read( (u8)BMP180_ADC_OUT_MSB_REG) << 16 ) |  \
	 ( BMP_i2c_read( (u8)BMP180_ADC_OUT_LSB_REG)  << 8 ) | \
	 ( BMP_i2c_read( (u8)BMP180_ADC_OUT_XLSB_REG) ) ) >> ( 8 - oss ) ) ;
	 
	return uP;
}

 
static ssize_t temp0_input_show (struct device *dev,
        struct device_attribute *attr, char *buf )
{
	long x1 , x2 , b5 , T;
	int err;
	 
	uT = get_ut ();
	x1 = ( (( uT - bmp_calibration.ac6 ) * bmp_calibration.ac5 ) >> 15 );
	x2 = ( bmp_calibration.mc << 11 ) / ( x1 + bmp_calibration.md ) ;
	b5 = x1 + x2;
	T = ( ( b5 + 8 ) >> 4 );
	 
	err = sprintf( buf , "%ld\n", T);
	return err;
 }
	 
static DEVICE_ATTR(temp0_input, 0444 , temp0_input_show, NULL);	  


static ssize_t pressure0_input_show (struct device *dev,
        struct device_attribute *attr, char *buf)
{
	unsigned long b4;
	long x1 , x2 , x3 , b3 , b5 , b6 , b7 , P;
	int err;
	short oss;
	 
	oss = ( BMP_i2c_read ( (u8) BMP180_CTRL_MEAS_REG ) >> 6 );
	uT = get_ut ();
	uP = get_up (oss);
	 
	x1 = ( (( uT - bmp_calibration.ac6 ) * bmp_calibration.ac5 ) >> 15 );
	x2 = ( bmp_calibration.mc << 11 ) / ( x1 + bmp_calibration.md );
	b5 = x1 + x2;
	 
	b6 = b5 - 4000;
	 
	x1 = ( ( bmp_calibration.b2 * ( ( b6 *b6 ) >> 12 ) ) >> 11 );
	x2 = ( ( bmp_calibration.ac2 * b6 ) >> 11);
	x3 = ( x1 + x2 );
	b3 = ( ( ( ( bmp_calibration.ac1 * 4 + x3 ) << oss ) + 2 ) / 4 );
	 
	x1 = ( ( bmp_calibration.ac3 * b6 ) >> 13 );
	x2 = ( ( bmp_calibration.b1 * ( ( b6 * b6 ) >> 12 ) ) >> 16 );
	x3 = ( ( ( x1 + x2 ) + 2 ) >> 2 ); 
	b4 = ( ( bmp_calibration.ac4 * (unsigned long)( x3 + 32768 ) ) >> 15 );
	 
	b7 = ( (unsigned long)uP - b3 ) * ( 50000 >> oss );
	 
	if ( b7 < 0x80000000 ) 
		P = ( ( b7 * 2 ) / b4 );
	else 
		P = ( b7 / b4 ) * 2;
	
	x1 = ( P >> 8 ) * ( P >> 8 );
	x1 = ( x1 * 3038 ) >> 16;
	x2 = ( ( -7357 ) *P ) >> 16;
	P =  P + ( ( x1 +x2 +3791 )  >> 4 );
	 
	err = sprintf ( buf , "%ld\n" , P );
	 
	return err;
 }
	 
static DEVICE_ATTR(pressure0_input,0444, pressure0_input_show, NULL);


static ssize_t oversampling_show (struct device *dev,
        struct device_attribute *attr, char *buf )
{
	int oss = 0;

	oss = ( BMP_i2c_read( (u8) BMP180_CTRL_MEAS_REG ) >> 6 );

	return sprintf ( buf , "%d\n" , oss );
}
 
static ssize_t oversampling_store (struct device *dev,
        struct device_attribute *attr, const char *buf , size_t count)
{
	int oss = 0;
	sscanf(buf, "%d", &oss);
	BMP_i2c_write( (u8) BMP180_CTRL_MEAS_REG , ( ( oss << 6 ) | \
	 BMP180_T_MEASURE ) ) ;
	
	return count;
}
static DEVICE_ATTR(oversampling , 0666 , oversampling_show , oversampling_store );

/*
 * Fuction: BMP_i2c_read (register address)
 * Description: read function by byte.
 */
int BMP_i2c_read(u8 reg) 
{ 
	int val = 0; 
	 
    val = i2c_smbus_read_byte_data( boschbmp180_client, reg);     
     
    if (val < 0) 
		printk("%s %d i2c transfer error\n", __func__, __LINE__); 
     
    return val; 
} 

/*
 * Fuction: BMP_i2c_write ( register address , value )
 * Description: write function by byte.
 */  
int BMP_i2c_write( u8 reg, int val )
{ 
	int err = 0; 
  
    if( (boschbmp180_client == NULL) || (!boschbmp180_client->adapter) )
	{
		return -ENODEV; 
    }
      
    err = i2c_smbus_write_byte_data(boschbmp180_client, reg, val); 
     
    if (err >= 0) return 0; 
  
    printk("%s %d i2c transfer error\n", __func__, __LINE__); 
  
    return err; 
} 

 
static int __devinit bosch_barometer_probe( struct i2c_client *client,
				    const struct i2c_device_id *id ) 
{ 
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct boashbmp180_data *barometer;  
	barometer->pdata = client->dev.platform_data;
	barometer->pdata->power_on;
	
	printk("%d i2c transfer error\n", *barometer->pdata->number);  
	
  	if (!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
		 
		 /* allocate driver_data */ 
	barometer = kzalloc(sizeof(struct boashbmp180_data),GFP_KERNEL);
         		     
	if(!barometer) 
	{ 
		pr_err("kzalloc error\n"); 
        return -ENOMEM; 
	} 
		 
	barometer->client = client;
	i2c_set_clientdata(client, barometer);
		 
	boschbmp180_client = client;    
		
	/* get calibration value */
	read_calibration (); 
		 
    /* set sysfs for proximity sensor */ 
    barometer_class = class_create(THIS_MODULE, "barometer"); 
    if (IS_ERR(barometer_class)) 
		pr_err("Failed to create class(barometersensor)!\n"); 
  
	boschbmp180_dev = device_create(barometer_class, NULL, 0, NULL, "boschbmp180");

    if (IS_ERR(boschbmp180_dev))							
		pr_err("Failed to create device ( boschbmp180_dev )!\n"); 
  
    if (device_create_file(boschbmp180_dev, &dev_attr_pressure0_input) < 0)    
        pr_err("Failed to create device file(%s)!\n", dev_attr_pressure0_input.attr.name); 

	if (device_create_file(boschbmp180_dev, &dev_attr_temp0_input) < 0)    
		pr_err("Failed to create device file(%s)!\n", dev_attr_temp0_input.attr.name); 
    
	if (device_create_file(boschbmp180_dev, &dev_attr_oversampling) < 0)    
		pr_err("Failed to create device file(%s)!\n", dev_attr_oversampling.attr.name); 
               
    dev_set_drvdata(boschbmp180_dev,barometer); 
    printk("BOSCHBMP180 barometer Sensor initialized\n"); 
    
    return 0; 
 } 
 
#if 0
static int bosch_barometer_suspend( struct platform_device* pdev, pm_message_t state ) 
{ 
	struct boashbmp180_data *barometer = pdev;

	if (barometer && deinit_hw)
		deinit_hw();

	return 0;
 } 



static int bosch_barometer_resume( struct platform_device* pdev ) 
{ 
	struct boashbmp180_data *barometer = pdev;

	if (barometer && init_hw)
		return init_hw();

	return 0;
} 
#endif
 
static const struct i2c_device_id boschbmp180_id[] = 
{
	{ "boschbmp180", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, boschbmp180_id);

static struct i2c_driver bosch_barometer_driver = 
{	
	.driver  = 
	{
		.name  = BOSCHBMP180_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.suspend = NULL,
	.resume  = NULL,
	.probe 	 = bosch_barometer_probe,
	.remove  = NULL,
	.id_table = boschbmp180_id,	
 }; 
  
static int __init bosch_barometer_init(void) 
 {  
	return i2c_add_driver(&bosch_barometer_driver); 
 } 
  
 static void __exit bosch_barometer_exit(void) 
 { 
    struct boashbmp180_data *barometer = dev_get_drvdata(boschbmp180_dev);

	if (barometer_wq)
		destroy_workqueue(barometer_wq);

	kfree(barometer);
	i2c_del_driver(&bosch_barometer_driver);
 } 
  
 module_init( bosch_barometer_init ); 
 module_exit( bosch_barometer_exit ); 
  
 MODULE_AUTHOR("AJ Chen"); 
 MODULE_DESCRIPTION("barometer sensor driver"); 
 MODULE_LICENSE("GPL"); 
