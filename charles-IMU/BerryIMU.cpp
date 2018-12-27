#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "linux/i2c-dev.h"
#include "LSM9DS1.h"

enum LSM9DS0_DEVICES {
    MAGNETOMETER=0x1E,
    ACCELEROMETER=0x1E,
    GYROSCOPE=0x6A
}

enum LSM9DS1_DEVICES {
    MAGNETOMETER=0x1C,  //Would be 0x1E if SDO_M is HIGH
    ACCELEROMETER=0x6A,
    GYROSCOPE=0x6A      //Would be 0x6B if SDO_AG is HIGH
};

enum LSM9DS0_REGISTERS {
    
    // Gyroscope registers
    WHO_AM_I_G=0x0F,
    CTRL_REG1_G=0x20,
    CTRL_REG2_G=0x21,
    CTRL_REG3_G=0x22,
    CTRL_REG4_G=0x23,
    CTRL_REG5_G=0x24,
    REFERENCE_G=0x25,
    STATUS_REG_G=0x27,
    OUT_X_L_G=0x28, OUT_X_H_G=0x29,
    OUT_Y_L_G=0x2A, OUT_Y_H_G=0x2B,
    OUT_Z_L_G=0x2C, OUT_Z_H_G=0x2D,
    FIFO_CTRL_REG_G=0x2E,
    FIFO_SRC_REG_G=0x2F,
    INT1_CFG_G=0x30,
    INT1_SRC_G=0x31,
    INT1_THS_XH_G=0x32, INT1_THS_XL_G=0x33,
    INT1_THS_YH_G=0x34, INT1_THS_YL_G=0x35,
    INT1_THS_ZH_G=0x36, INT1_THS_ZL_G=0x37,
    INT1_DURATION_G=0x38,
    
    // Accelerometer and Magnetometer Registers
    OUT_TEMP_L_XM=0x05, OUT_TEMP_H_XM=0x06,
    STATUS_REG_M=0x07,
    OUT_X_L_M=0x08, OUT_X_H_M=0x09,
    OUT_Y_L_M=0x0A, OUT_Y_H_M=0x0B,
    OUT_Z_L_M=0x0C, OUT_Z_H_M=0x0D,
    WHO_AM_I_XM=0x0F,
    INT_CTRL_REG_M=0x12,
    INT_SRC_REG_M=0x13,
    INT_THS_L_M=0x14, INT_THS_H_M=0x15,
    OFFSET_X_L_M=0x16, OFFSET_X_H_M=0x17,
    OFFSET_Y_L_M=0x18, OFFSET_Y_H_M=0x19,
    OFFSET_Z_L_M=0x1A, OFFSET_Z_H_M=0x1B,
    REFERENCE_X=0x1C,
    REFERENCE_Y=0x1D,
    REFERENCE_Z=0x1E,
    CTRL_REG0_XM=0x1F,
    CTRL_REG1_XM=0x20,
    CTRL_REG2_XM=0x21,
    CTRL_REG3_XM=0x22,
    CTRL_REG4_XM=0x23,
    CTRL_REG5_XM=0x24,
    CTRL_REG6_XM=0x25,
    CTRL_REG7_XM=0x26,
    STATUS_REG_A=0x27,
    OUT_X_L_A=0x28, OUT_X_H_A=0x29,
    OUT_Y_L_A=0x2A, OUT_Y_H_A=0x2B,
    OUT_Z_L_A=0x2C, OUT_Z_H_A=0x2D,
    FIFO_CTRL_REG=0x2E,
    FIFO_SRC_REG=0x2F,
    INT_GEN_1_REG=0x30,
    INT_GEN_1_SRC=0x31,
    INT_GEN_1_THS=0x32,
    INT_GEN_1_DURATION=0x33,
    INT_GEN_2_REG=0x34,
    INT_GEN_2_SRC=0x35,
    INT_GEN_2_THS=0x36,
    INT_GEN_2_DURATION=0x37,
    CLICK_CFG=0x38,
    CLICK_SRC=0x39,
    CLICK_THS=0x3A,
    TIME_LIMIT=0x3B,
    TIME_LATENCY=0x3C,
    TIME_WINDOW=0x3D,

    // Defined responses
    WHO_AM_I_AG_RSP=0xd4,
    WHO_AM_I_M_RSP=0x49
};

enum LSM9DS1_devices {
    MAG_ADDRESS 0x1C	//Would be 0x1E if SDO_M is HIGH		
    ACC_ADDRESS 0x6A
    GYR_ADDRESS 0x6A	//Would be 0x6B if SDO_AG is HIGH
};


#/////////////////////////////////////////
#// LSM9DS1 Accel/Gyro (XL/G) Registers //
#/////////////////////////////////////////
#define LSM9DS1_ACT_THS			 0x04
#define LSM9DS1_ACT_DUR			 0x05
#define LSM9DS1_INT_GEN_CFG_XL	 0x06
#define LSM9DS1_INT_GEN_THS_X_XL 0x07
#define LSM9DS1_INT_GEN_THS_Y_XL 0x08
#define LSM9DS1_INT_GEN_THS_Z_XL 0x09
#define LSM9DS1_INT_GEN_DUR_XL	 0x0A
#define LSM9DS1_REFERENCE_G		 0x0B
#define LSM9DS1_INT1_CTRL		 0x0C
#define LSM9DS1_INT2_CTRL		 0x0D
#define LSM9DS1_WHO_AM_I_XG		 0x0F
#define LSM9DS1_CTRL_REG1_G		 0x10
#define LSM9DS1_CTRL_REG2_G		 0x11
#define LSM9DS1_CTRL_REG3_G		 0x12
#define LSM9DS1_ORIENT_CFG_G	 0x13
#define LSM9DS1_INT_GEN_SRC_G	 0x14
#define LSM9DS1_OUT_TEMP_L		 0x15
#define LSM9DS1_OUT_TEMP_H		 0x16
#define LSM9DS1_STATUS_REG_0	 0x17
#define LSM9DS1_OUT_X_L_G		 0x18
#define LSM9DS1_OUT_X_H_G		 0x19
#define LSM9DS1_OUT_Y_L_G		 0x1A
#define LSM9DS1_OUT_Y_H_G		 0x1B
#define LSM9DS1_OUT_Z_L_G		 0x1C
#define LSM9DS1_OUT_Z_H_G		 0x1D
#define LSM9DS1_CTRL_REG4		 0x1E
#define LSM9DS1_CTRL_REG5_XL	 0x1F
#define LSM9DS1_CTRL_REG6_XL	 0x20
#define LSM9DS1_CTRL_REG7_XL	 0x21
#define LSM9DS1_CTRL_REG8		 0x22
#define LSM9DS1_CTRL_REG9		 0x23
#define LSM9DS1_CTRL_REG10		 0x24
#define LSM9DS1_INT_GEN_SRC_XL	 0x26
#define LSM9DS1_STATUS_REG_1	 0x27
#define LSM9DS1_OUT_X_L_XL		 0x28
#define LSM9DS1_OUT_X_H_XL		 0x29
#define LSM9DS1_OUT_Y_L_XL		 0x2A
#define LSM9DS1_OUT_Y_H_XL		 0x2B
#define LSM9DS1_OUT_Z_L_XL		 0x2C
#define LSM9DS1_OUT_Z_H_XL		 0x2D
#define LSM9DS1_FIFO_CTRL		 0x2E
#define LSM9DS1_FIFO_SRC		 0x2F
#define LSM9DS1_INT_GEN_CFG_G	 0x30
#define LSM9DS1_INT_GEN_THS_XH_G 0x31
#define LSM9DS1_INT_GEN_THS_XL_G 0x32
#define LSM9DS1_INT_GEN_THS_YH_G 0x33
#define LSM9DS1_INT_GEN_THS_YL_G 0x34
#define LSM9DS1_INT_GEN_THS_ZH_G 0x35
#define LSM9DS1_INT_GEN_THS_ZL_G 0x36
#define LSM9DS1_INT_GEN_DUR_G	 0x37

#///////////////////////////////
#// LSM9DS1 Magneto Registers //
#///////////////////////////////
#define LSM9DS1_OFFSET_X_REG_L_M 0x05
#define LSM9DS1_OFFSET_X_REG_H_M 0x06
#define LSM9DS1_OFFSET_Y_REG_L_M 0x07
#define LSM9DS1_OFFSET_Y_REG_H_M 0x08
#define LSM9DS1_OFFSET_Z_REG_L_M 0x09
#define LSM9DS1_OFFSET_Z_REG_H_M 0x0A
#define LSM9DS1_WHO_AM_I_M		 0x0F
#define LSM9DS1_CTRL_REG1_M		 0x20
#define LSM9DS1_CTRL_REG2_M		 0x21
#define LSM9DS1_CTRL_REG3_M		 0x22
#define LSM9DS1_CTRL_REG4_M		 0x23
#define LSM9DS1_CTRL_REG5_M		 0x24
#define LSM9DS1_STATUS_REG_M	 0x27
#define LSM9DS1_OUT_X_L_M		 0x28
#define LSM9DS1_OUT_X_H_M		 0x29
#define LSM9DS1_OUT_Y_L_M		 0x2A
#define LSM9DS1_OUT_Y_H_M		 0x2B
#define LSM9DS1_OUT_Z_L_M		 0x2C
#define LSM9DS1_OUT_Z_H_M		 0x2D
#define LSM9DS1_INT_CFG_M		 0x30
#define LSM9DS1_INT_SRC_M		 0x30
#define LSM9DS1_INT_THS_L_M		 0x32
#define LSM9DS1_INT_THS_H_M		 0x33

#////////////////////////////////
#// LSM9DS1 WHO_AM_I Responses //
#////////////////////////////////
#define LSM9DS1_WHO_AM_I_AG_RSP	 0x68
#define LSM9DS1_WHO_AM_I_M_RSP	 0x3D






/* Utility functions */
void inline i2cSelectDevice(int file, int addr)
{
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		printf("Error: Failed to select I2C device");
	}
}

class BerryIMU {
    public:
        BerryIMU(void);
        ~BerryIMU(void);
        void status(void);
        enum DEVICES {ACC, GYR, MAG};
        enum CHIPSETS { LSM9DS0, LSM9DS1 };
    protected:
        void identify(void);
        void enable(void);
    private:
        int file;
        CHIPSETS chip;
        void enable_LSM9DS0(void);
        void enable_LSM9DS1(void);
};

BerryIMU::BerryIMU(void){
    // Constructor

    // Using the WHO_AM_I register detect the IMU and determine which
    // version is present

    const char filename[] = "/dev/i2c-1";

	this->file = open(filename, O_RDWR);
	if (this->file <= 0) {
		printf("Error: Unable to open I2C bus!\n");
		exit(1);
	}
    this->identify();
    this->enable();
}

BerryIMU::~BerryIMU(void){
    if (this->file > 0) {
        close(this->file);
    }
}

void BerryIMU::status(void){
    printf("BerryIMU Status:\n");
    printf(" - File handle: %d\n", this->file);
    printf(" - Chipset: ");
    if (this->chip == LSM9DS0) {
        printf("LSM9DS0\n");
    } else if (this->chip == LSM9DS1) {
        printf("LSM9DS1\n");
    } else {
        printf("Unknown\n");
    }
}

void BerryIMU::identify(void) {
    int who_magneto, who_gyro;

    // Attempt reading from the LSM9DS0 chip WHO_AM_I register
    i2cSelectDevice(this->file, LSM9DS0_ACC_ADDRESS);
	who_magneto = i2c_smbus_read_byte_data(this->file, LSM9DS0_WHO_AM_I_XM);
	i2cSelectDevice(this->file,LSM9DS0_GYR_ADDRESS);
	who_gyro = i2c_smbus_read_byte_data(this->file, LSM9DS0_WHO_AM_I_G);
	if (who_gyro == LSM9DS0_WHO_AM_I_AG_RSP && who_magneto == LSM9DS0_WHO_AM_I_M_RSP){
		printf ("Info: BerryIMUv1/LSM9DS0  DETECTED\n");
		this->chip = LSM9DS0;
	} else {

        // Attempt reading from LSM9DS1 chip WHO_AM_I register
        i2cSelectDevice(this->file, LSM9DS1_MAG_ADDRESS);
        who_magneto = i2c_smbus_read_byte_data(this->file, LSM9DS1_WHO_AM_I_M);
        i2cSelectDevice(this->file, LSM9DS1_GYR_ADDRESS);
        who_gyro = i2c_smbus_read_byte_data(this->file, LSM9DS1_WHO_AM_I_XG);

        if (who_gyro == LSM9DS1_WHO_AM_I_AG_RSP && who_magneto == LSM9DS1_WHO_AM_I_M_RSP){
            printf ("Info: BerryIMUv2/LSM9DS1 DETECTED\n");
            this->chip = LSM9DS1;
        } else {

            printf ("Error: No IMU detected\n");
            exit(1);
        }
    }
}

void BerryIMU::enable(void) {
    switch(this->chip) {
        case LSM9DS0:
            // Enable accelerometer.
            writeAccReg(LSM9DS0_CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuous update,  100Hz data rate
            writeAccReg(LSM9DS0_CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

            //Enable the magnetometer
            writeMagReg(LSM9DS0_CTRL_REG5_XM, 0b11110000); // Temp enable, M data rate = 50Hz
            writeMagReg(LSM9DS0_CTRL_REG6_XM, 0b01100000); // +/-12gauss
            writeMagReg(LSM9DS0_CTRL_REG7_XM, 0b00000000); // Continuous-conversion mode

            // Enable Gyro
            writeGyrReg(LSM9DS0_CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
            writeGyrReg(LSM9DS0_CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
            break;

        case LSM9DS1:
            // Enable the accelerometer
            writeAccReg(LSM9DS1_CTRL_REG5_XL,0b00111000);   // z, y, x axis enabled for accelerometer
            writeAccReg(LSM9DS1_CTRL_REG6_XL,0b00101000);   // +/- 16g

            //Enable the magnetometer
            writeMagReg(LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
            writeMagReg(LSM9DS1_CTRL_REG2_M, 0b01000000);   // +/-12gauss
            writeMagReg(LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuos update
            writeMagReg(LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis

            // Enable the gyroscope
            writeGyrReg(LSM9DS1_CTRL_REG4,0b00111000);      // z, y, x axis enabled for gyro
            writeGyrReg(LSM9DS1_CTRL_REG1_G,0b10111000);    // Gyro ODR = 476Hz, 2000 dps
            writeGyrReg(LSM9DS1_ORIENT_CFG_G,0b10111000);   // Swap orientation 
            break;

        default:
            printf("Error: Unable to enable unknown chip\n");
            exit(1)
    }
}

void BerryIMU::writeReg(DEVICES device, uint8_t reg, uint8_t value){
    i2cSelectDevice(this->file, ADDRESSES[this->chip][device])

	
	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf ("Failed to write byte to I2C Acc.");
		exit(1);
	}
}

int main(void) {
    BerryIMU imu;

    imu.status();
    return 1;
}