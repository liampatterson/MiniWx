#include "fsl_debug_console.h"
#include "board.h"
extern "C" {
	#include "utils.h"
}

//code references: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-12.pdf

#include "Driver_I2C.h"
//#include "Driver_USART.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include <string.h>
#include <math.h>
#include "temperature.h"


extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C0;
ARM_DRIVER_I2C *drv_info;

static uint8_t DeviceAddr= SENSOR_ADDRESS;


//read one byte of i2c data
static int32_t i2c_read (uint8_t reg, uint8_t *val) {
  uint8_t data[1];
  data[0] = reg;
  I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
  while (I2Cdrv->GetStatus().busy);
  if (I2Cdrv->GetDataCount() != 1) { return -1; }
  I2Cdrv->MasterReceive (DeviceAddr, val, 1, false);
  while (I2Cdrv->GetStatus().busy);
  if (I2Cdrv->GetDataCount() != 1) { return -1; }

  return 0;
}


//getVersion

void i2c_setup (void)  {
  ARM_DRIVER_VERSION  version;
  drv_info = &Driver_I2C0;
  version = drv_info->GetVersion ();
  if (version.api < 0x10A)   {      //check minimum version spec
    return;
  }
}

/*
 //getCapabilities
void getCapabilities (void)  {
  ARM_I2C_CAPABILITIES drv_capabilities;
  drv_info = &Driver_I2C0;
  drv_capabilities = drv_info->GetCapabilities ();
  // interrogate capabilities

}*/

 //Initialize & Power Control
int i2c_init (void) {
	I2Cdrv->Initialize		(NULL); //Initialize to defaults
	I2Cdrv->PowerControl	(ARM_POWER_FULL); //give it full power
	I2Cdrv->Control     (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST); //make it fast
  I2Cdrv->Control     (ARM_I2C_BUS_CLEAR, 0); //clear the bus of old info

	//identify i2c device, make sure it's our sensor
	uint8_t who = 0;
  int32_t res = i2c_read(SENSOR_CHIPID, &who);
  if (res == 0) {
    if (who != SENSOR_READVAL) {
      return -1;                        /* Wrong ID */
    }
  }

	//reset itself
	unsigned char data[2];
	data[0] = SENSOR_SOFTRESET;
	data[1] = 0x5B;
	I2Cdrv->MasterTransmit(DeviceAddr, data, 2, false);
	while (I2Cdrv->GetStatus().busy);
	if (I2Cdrv->GetDataCount() != 2) { return -1; }
	long_delay();

	//setup according to weather monitor specs from datasheet
	data[0] = 0xF3;
	I2Cdrv->MasterTransmit(DeviceAddr, data, 1, false);
	while (I2Cdrv->GetStatus().busy);
	return 0;
}

 //Data Txfer
int bme280_init(void) {
		t_fine= 0;
		unsigned char data[8];

		//read coeffs
		data[0]= SENSOR_DIG_T1;
		//read dig T1
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 1) { return -1; }
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_T1 = (data[1] << 8) | data[0];

		//read dig T2
		data[0]= SENSOR_DIG_T2;
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 1) { return -1; }
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_T2 = (data[1] << 8) | data[0];

		//read dig T3
		data[0]= SENSOR_DIG_T3;
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 1) { return -1; }
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_T3 = (data[1] << 8) | data[0];


		//debug_printf("dig_T = 0x%x, 0x%x, 0x%x\r\n", dig_T1, dig_T2, dig_T3);

		data[0] = SENSOR_DIG_P1; // read dig_P1 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P1 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P2; // read dig_P2 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P2 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P3; // read dig_P3 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P3 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P4; // read dig_P4 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P4 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P5; // read dig_P5 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P5 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P6; // read dig_P6 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P6 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P7; // read dig_P7 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P7 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P8; // read dig_P8 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P8 = (data[ 1] << 8) | data[ 0];

		data[0] = SENSOR_DIG_P9; // read dig_P9 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_P9 = (data[ 1] << 8) | data[ 0];

		//debug_printf("dig_P = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);

		data[0] = SENSOR_DIG_H1; // read dig_H1 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 1, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 1) { return -1; }
		dig_H1 = data[0];

		data[0] = SENSOR_DIG_H2; // read dig_H2 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_H2 = (data[1] << 8) | data[0];

		data[0] = SENSOR_DIG_H3; // read dig_H3 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 1, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 1) { return -1; }
		dig_H3 = data[0];

		data[0] = SENSOR_DIG_H4; // read dig_H4 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_H4 = (data[0] << 4) | (data[1] & 0x0f);

		data[0] = SENSOR_DIG_H5; // read dig_H5 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }
		dig_H5 = (data[1] << 4) | ((data[0]>>4) & 0x0f);

		data[0] = SENSOR_DIG_H6; // read dig_H6 regs
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 1, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 1) { return -1; }
		dig_H6 = data[0];
		//debug_printf("dig_H = 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);

		//setup
    data[0] = SENSOR_CONTROLHUMID; // Humidity
    data[1] = 0x02; // SAMPLE x1
		I2Cdrv->MasterTransmit(DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }

		data[0] = SENSOR_CONFIG; // config
    data[1] = 0x00;
		I2Cdrv->MasterTransmit(DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }

		data[0] = SENSOR_CONTROL; // ctrl_temp
    data[1] = 0x55; //weather monitor mode (normal)
		I2Cdrv->MasterTransmit(DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		if (I2Cdrv->GetDataCount() != 2) { return -1; }

		return 0;
}

float getTemp(){
	uint32_t temp_raw;
	float tempf;
	unsigned char data[4];

	data[0]= SENSOR_TEMPDATA; //temp_msb

	//read
	I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
	while (I2Cdrv->GetStatus().busy);
	I2Cdrv->MasterReceive (DeviceAddr, &data[0], 3, false);
	while (I2Cdrv->GetStatus().busy);
	temp_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	int32_t temp;
	 temp =
        (((((temp_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11) +
        ((((((temp_raw >> 4) - dig_T1) * ((temp_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);

    t_fine = temp;
    temp = (temp * 5 + 128) >> 8;
    tempf = (float)temp;

    return (tempf/100.0f);
}


float getHumidity()
{
    uint32_t hum_raw;
    float humf;
    unsigned char data[4];

    data[0] = SENSOR_HUMIDDATA; // hum_msb

		//write
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, false);
		while (I2Cdrv->GetStatus().busy);

		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 2, false);
		while (I2Cdrv->GetStatus().busy);
		//math provided from BOSCH datasheet, compensates and calculates
    hum_raw = (data[0] << 8) | data[1];
    int32_t v_x1;
    v_x1 = t_fine - 76800;
    v_x1 =  (((((hum_raw << 14) -(((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) +
               ((int32_t)16384)) >> 15) * (((((((v_x1 * (int32_t)dig_H6) >> 10) *
                                            (((v_x1 * ((int32_t)dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                                            (int32_t)dig_H2 + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int32_t)dig_H1) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    humf = (float)(v_x1 >> 12);

    return (humf/1024.0f);
}


float getPressure()
{
    uint32_t press_raw;
    float pressf;
    unsigned char data[4];

    data[0] = SENSOR_PRESSUREDATA; // press

		//read
		I2Cdrv->MasterTransmit(DeviceAddr, data, 1, true);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->MasterReceive (DeviceAddr, data, 3, false);
		while (I2Cdrv->GetStatus().busy);
		I2Cdrv->GetDataCount();

		//ensure the bits are set
    press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t var1, var2;
    uint32_t press;

		//math provided from BOSCH datasheet, compensates and calculates
    var1 = (t_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2)*(var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * dig_P1) >> 15;
    if (var1 == 0) {
        return 0;
    }
    press = (((1048576 - press_raw) - (var2 >> 12))) * 3125;
    if(press < 0x80000000) {
        press = (press << 1) / var1;
    } else {
        press = (press / var1) * 2;
    }
    var1 = ((int32_t)dig_P9 * ((int32_t)(((press >> 3) * (press >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(press >> 2)) * (int32_t)dig_P8) >> 13;
    press = (press + ((var1 + var2 + dig_P7) >> 4));

    pressf = (float)press;
		return (pressf/3386.375258f); //returns in in.Hg
}


 float getDewpoint(float temp, float hum)
 {
	 float b= 17.67;  //unitless, from NOAA 1980
	 float c= 257.14; //celsius, from NOAA 1980
	 float d= 234.5;  //celsius, from NOAA 1980
	 float y= log((hum/100.0000f) * exp(((b-(temp/d))*(temp/(c+temp)))));
	 float dewpoint= (c*y)/(b-y);
	 return dewpoint;
 }


 //obtains pressure altitude, uncorrected for station elevation
 float getAltitude(float press)
 {
		return (29.92f - press)*1000.f;
 }

 int getSLP(float press, float temp)
 {
	 float GPSAlt= 245; //station elevation in meters
	 float slp= (press*33.86375258f) / (exp((-GPSAlt)/((temp+273.15f)*29.263f))); //returns in mbars
	 return slp/.3386375258f; //return in 100*inHg
 }

 int getClouds(float dew, float temp){
	 float fartemp = (temp*1.4)+32;
	 float fardew = (dew*1.4)+32;
	 float value= (((fartemp-fardew)/4.4f)*10);
	 return value;
 }


int main() {
	volatile float temp;
	volatile float hum;
	volatile float press;
	volatile float dewpoint;
	volatile float alt;
	volatile int clouds;
	volatile int slp;
	int delimitter= 0;

	hardware_init();
	LED_Initialize();
	debug_printf("Starting...\r\n");
	//i2c_setup();
	//getCapabilities();
	i2c_init();
	if (bme280_init()!= 0) return -1;
	while(1){
		LEDRed_On(); //TURN ON LED
		if (bme280_init()!= 0) return -1; //READ REGISTERS
		temp= getTemp(); //CALCULATE TEMP
		press= getPressure(); //CALCULATE PRESSURE
		hum= getHumidity(); //CALCULATE HUM
		dewpoint= getDewpoint(temp, hum); //DO SOME MATH FOR DEWPOINT
		alt= getAltitude(press); //DO SOME MATH FOR ALT
		clouds= getClouds(dewpoint, temp);
		slp= getSLP(press, temp); //get sea level pressure
		if (delimitter%10 == 0)
				debug_printf("\r\n METAR CORNELL 160000Z UKN%03d %02d/%02d A%d \r\n", clouds, (int)temp, (int)dewpoint, slp);
		if (delimitter%5 == 0)
			debug_printf("\r\nTemp/Dewpoint | Pressure/Pressure Alt | Rel Humidity\r\n"); //EVERY 5 TIMES PRINT THE HEADER
		debug_printf("%4.2fC/%4.2fC | %4.2f inHg/%4.2f ft | %4.2f%%\r\n", temp, dewpoint, press, alt, hum); //PRINT IT OUT
		short_delay();
		LED_Off(); //TURN OFF LED WHEN FINISHED
		long_delay(); //DELAY LOOP
		long_delay();
		delimitter++; //INCREMENT delimitter
	}
}
