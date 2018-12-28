#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "linux/i2c-dev.h"

enum BERRY_IMU_DEVICES {
    ACCELEROMETER,
    GYROSCOPE,
    MAGNETOMETER,
    PRESSURE
};

enum CHIPSETS {
    LSM9DS0,
    LSM9DS1
};

int LSM9DS0_DEV_ADDRESSES[] = {
    0x1E,   // ACCELEROMETER
    0x6A,   // GYROSCOPE
    0x1E,   // MAGNETOMETER
    0x00,   // PRESSURE TODO Need actual value
};

int LSM9DS1_DEV_ADDRESSES[] = {
    0x6A,   // ACCELEROMETER
    0x6A,   // GYROSCOPE
    0x1C,   // MAGNETOMETER
    0x77,   // PRESSURE
};

enum LSM9DS0_REGISTERS {
    
    // Gyroscope registers
    LSM9DS0_WHO_AM_I_G=0x0F,
    LSM9DS0_CTRL_REG1_G=0x20,
    LSM9DS0_CTRL_REG2_G=0x21,
    LSM9DS0_CTRL_REG3_G=0x22,
    LSM9DS0_CTRL_REG4_G=0x23,
    LSM9DS0_CTRL_REG5_G=0x24,
    LSM9DS0_REFERENCE_G=0x25,
    LSM9DS0_STATUS_REG_G=0x27,
    LSM9DS0_OUT_X_L_G=0x28, LSM9DS0_OUT_X_H_G=0x29,
    LSM9DS0_OUT_Y_L_G=0x2A, LSM9DS0_OUT_Y_H_G=0x2B,
    LSM9DS0_OUT_Z_L_G=0x2C, LSM9DS0_OUT_Z_H_G=0x2D,
    LSM9DS0_FIFO_CTRL_REG_G=0x2E,
    LSM9DS0_FIFO_SRC_REG_G=0x2F,
    LSM9DS0_INT1_CFG_G=0x30,
    LSM9DS0_INT1_SRC_G=0x31,
    LSM9DS0_INT1_THS_XH_G=0x32, LSM9DS0_INT1_THS_XL_G=0x33,
    LSM9DS0_INT1_THS_YH_G=0x34, LSM9DS0_INT1_THS_YL_G=0x35,
    LSM9DS0_INT1_THS_ZH_G=0x36, LSM9DS0_INT1_THS_ZL_G=0x37,
    LSM9DS0_INT1_DURATION_G=0x38,
    
    // Accelerometer and Magnetometer Registers
    LSM9DS0_OUT_TEMP_L_XM=0x05, LSM9DS0_OUT_TEMP_H_XM=0x06,
    LSM9DS0_STATUS_REG_M=0x07,
    LSM9DS0_OUT_X_L_M=0x08, LSM9DS0_OUT_X_H_M=0x09,
    LSM9DS0_OUT_Y_L_M=0x0A, LSM9DS0_OUT_Y_H_M=0x0B,
    LSM9DS0_OUT_Z_L_M=0x0C, LSM9DS0_OUT_Z_H_M=0x0D,
    LSM9DS0_WHO_AM_I_XM=0x0F,
    LSM9DS0_INT_CTRL_REG_M=0x12,
    LSM9DS0_INT_SRC_REG_M=0x13,
    LSM9DS0_INT_THS_L_M=0x14, LSM9DS0_INT_THS_H_M=0x15,
    LSM9DS0_OFFSET_X_L_M=0x16, LSM9DS0_OFFSET_X_H_M=0x17,
    LSM9DS0_OFFSET_Y_L_M=0x18, LSM9DS0_OFFSET_Y_H_M=0x19,
    LSM9DS0_OFFSET_Z_L_M=0x1A, LSM9DS0_OFFSET_Z_H_M=0x1B,
    LSM9DS0_REFERENCE_X=0x1C,
    LSM9DS0_REFERENCE_Y=0x1D,
    LSM9DS0_REFERENCE_Z=0x1E,
    LSM9DS0_CTRL_REG0_XM=0x1F,
    LSM9DS0_CTRL_REG1_XM=0x20,
    LSM9DS0_CTRL_REG2_XM=0x21,
    LSM9DS0_CTRL_REG3_XM=0x22,
    LSM9DS0_CTRL_REG4_XM=0x23,
    LSM9DS0_CTRL_REG5_XM=0x24,
    LSM9DS0_CTRL_REG6_XM=0x25,
    LSM9DS0_CTRL_REG7_XM=0x26,
    LSM9DS0_STATUS_REG_A=0x27,
    LSM9DS0_OUT_X_L_A=0x28, LSM9DS0_OUT_X_H_A=0x29,
    LSM9DS0_OUT_Y_L_A=0x2A, LSM9DS0_OUT_Y_H_A=0x2B,
    LSM9DS0_OUT_Z_L_A=0x2C, LSM9DS0_OUT_Z_H_A=0x2D,
    LSM9DS0_FIFO_CTRL_REG=0x2E,
    LSM9DS0_FIFO_SRC_REG=0x2F,
    LSM9DS0_INT_GEN_1_REG=0x30,
    LSM9DS0_INT_GEN_1_SRC=0x31,
    LSM9DS0_INT_GEN_1_THS=0x32,
    LSM9DS0_INT_GEN_1_DURATION=0x33,
    LSM9DS0_INT_GEN_2_REG=0x34,
    LSM9DS0_INT_GEN_2_SRC=0x35,
    LSM9DS0_INT_GEN_2_THS=0x36,
    LSM9DS0_INT_GEN_2_DURATION=0x37,
    LSM9DS0_CLICK_CFG=0x38,
    LSM9DS0_CLICK_SRC=0x39,
    LSM9DS0_CLICK_THS=0x3A,
    LSM9DS0_TIME_LIMIT=0x3B,
    LSM9DS0_TIME_LATENCY=0x3C,
    LSM9DS0_TIME_WINDOW=0x3D
};

enum LSM9DS0_RESPONSES {
    LSM9DS0_WHO_AM_I_AG_RSP=0xd4,
    LSM9DS0_WHO_AM_I_M_RSP=0x49
};

enum LSM9DS1_REGISTERS {
    // Accelerometer and Gyroscope Registers
	LSM9DS1_ACT_THS=0x04,
	LSM9DS1_ACT_DUR=0x05,
	LSM9DS1_INT_GEN_CFG_XL=0x06,
	LSM9DS1_INT_GEN_THS_X_XL=0x07,
	LSM9DS1_INT_GEN_THS_Y_XL=0x08,
	LSM9DS1_INT_GEN_THS_Z_XL=0x09,
	LSM9DS1_INT_GEN_DUR_XL=0x0A,
	LSM9DS1_REFERENCE_G=0x0B,
	LSM9DS1_INT1_CTRL=0x0C,
	LSM9DS1_INT2_CTRL=0x0D,
	LSM9DS1_WHO_AM_I_XG=0x0F,
	LSM9DS1_CTRL_REG1_G=0x10,
	LSM9DS1_CTRL_REG2_G=0x11,
	LSM9DS1_CTRL_REG3_G=0x12,
	LSM9DS1_ORIENT_CFG_G=0x13,
	LSM9DS1_INT_GEN_SRC_G=0x14,
	LSM9DS1_OUT_TEMP_L=0x15,
	LSM9DS1_OUT_TEMP_H=0x16,
	LSM9DS1_STATUS_REG_0=0x17,
	LSM9DS1_OUT_X_L_G=0x18,	LSM9DS1_OUT_X_H_G=0x19,
	LSM9DS1_OUT_Y_L_G=0x1A,	LSM9DS1_OUT_Y_H_G=0x1B,
	LSM9DS1_OUT_Z_L_G=0x1C,	LSM9DS1_OUT_Z_H_G=0x1D,
	LSM9DS1_CTRL_REG4=0x1E,
	LSM9DS1_CTRL_REG5_XL=0x1F,
	LSM9DS1_CTRL_REG6_XL=0x20,
	LSM9DS1_CTRL_REG7_XL=0x21,
	LSM9DS1_CTRL_REG8=0x22,
	LSM9DS1_CTRL_REG9=0x23,
	LSM9DS1_CTRL_REG10=0x24,
	LSM9DS1_INT_GEN_SRC_XL=0x26,
	LSM9DS1_STATUS_REG_1=0x27,
	LSM9DS1_OUT_X_L_XL=0x28, LSM9DS1_OUT_X_H_XL=0x29,
	LSM9DS1_OUT_Y_L_XL=0x2A, LSM9DS1_OUT_Y_H_XL=0x2B,
	LSM9DS1_OUT_Z_L_XL=0x2C, LSM9DS1_OUT_Z_H_XL=0x2D,
	LSM9DS1_FIFO_CTRL=0x2E,
	LSM9DS1_FIFO_SRC=0x2F,
	LSM9DS1_INT_GEN_CFG_G=0x30,
	LSM9DS1_INT_GEN_THS_XH_G=0x31,
	LSM9DS1_INT_GEN_THS_XL_G=0x32,
	LSM9DS1_INT_GEN_THS_YH_G=0x33,
	LSM9DS1_INT_GEN_THS_YL_G=0x34,
	LSM9DS1_INT_GEN_THS_ZH_G=0x35,
	LSM9DS1_INT_GEN_THS_ZL_G=0x36,
	LSM9DS1_INT_GEN_DUR_G=0x37,

    // Magnemometer registers
	LSM9DS1_OFFSET_X_REG_L_M=0x05, LSM9DS1_OFFSET_X_REG_H_M=0x06,
	LSM9DS1_OFFSET_Y_REG_L_M=0x07, LSM9DS1_OFFSET_Y_REG_H_M=0x08,
	LSM9DS1_OFFSET_Z_REG_L_M=0x09, LSM9DS1_OFFSET_Z_REG_H_M=0x0A,
	LSM9DS1_WHO_AM_I_M=0x0F,
	LSM9DS1_CTRL_REG1_M=0x20,
	LSM9DS1_CTRL_REG2_M=0x21,
	LSM9DS1_CTRL_REG3_M=0x22,
	LSM9DS1_CTRL_REG4_M=0x23,
	LSM9DS1_CTRL_REG5_M=0x24,
	LSM9DS1_STATUS_REG_M=0x27,
	LSM9DS1_OUT_X_L_M=0x28, LSM9DS1_OUT_X_H_M=0x29,
	LSM9DS1_OUT_Y_L_M=0x2A, LSM9DS1_OUT_Y_H_M=0x2B,
	LSM9DS1_OUT_Z_L_M=0x2C,	LSM9DS1_OUT_Z_H_M=0x2D,
	LSM9DS1_INT_CFG_M=0x30,
	LSM9DS1_INT_SRC_M=0x30,
	LSM9DS1_INT_THS_L_M=0x32, LSM9DS1_INT_THS_H_M=0x33
};

enum LSM9DS1_RESPONSES {
	LSM9DS1_WHO_AM_I_AG_RSP=0x68,
	LSM9DS1_WHO_AM_I_M_RSP=0x3D
};

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
    protected:
        void identify(void);
        void enable(void);
    private:
        int file;
        CHIPSETS chip;
        inline bool is_LSM9DS0(void);
        inline bool is_LSM9DS1(void);
        inline void enable_LSM9DS0(void);
        inline void enable_LSM9DS1(void);
        inline void selectDevice(BERRY_IMU_DEVICES device);
        inline void writeReg(BERRY_IMU_DEVICES device, uint8_t reg, uint8_t value);
};

BerryIMU::BerryIMU(void){
    // Constructor
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
    if (this->is_LSM9DS0()) {
		printf ("Info: BerryIMUv1/LSM9DS0  DETECTED\n");
		this->chip = LSM9DS0;
	} else if (this->is_LSM9DS1()) {
        printf ("Info: BerryIMUv2/LSM9DS1 DETECTED\n");
        this->chip = LSM9DS1;
    } else {
        printf ("Error: No IMU detected\n");
        exit(1);
    }
}

bool BerryIMU::is_LSM9DS0(void) {
    int magneto_resp, gyro_resp;

    i2cSelectDevice(this->file, LSM9DS0_DEV_ADDRESSES[ACCELEROMETER]);
	magneto_resp = i2c_smbus_read_byte_data(this->file, LSM9DS0_WHO_AM_I_XM);
	i2cSelectDevice(this->file, LSM9DS0_DEV_ADDRESSES[GYROSCOPE]);
	gyro_resp = i2c_smbus_read_byte_data(this->file, LSM9DS0_WHO_AM_I_G);
	if (gyro_resp == LSM9DS0_WHO_AM_I_AG_RSP && \
        magneto_resp == LSM9DS0_WHO_AM_I_M_RSP) {
        return true;
    }
    return false;

}

bool BerryIMU::is_LSM9DS1(void) {
    int magneto_resp, gyro_resp;

    i2cSelectDevice(this->file, LSM9DS1_DEV_ADDRESSES[MAGNETOMETER]);
    magneto_resp = i2c_smbus_read_byte_data(this->file, LSM9DS1_WHO_AM_I_M);
    i2cSelectDevice(this->file, LSM9DS1_DEV_ADDRESSES[GYROSCOPE]);
    gyro_resp = i2c_smbus_read_byte_data(this->file, LSM9DS1_WHO_AM_I_XG);

    if (gyro_resp == LSM9DS1_WHO_AM_I_AG_RSP && \
        magneto_resp == LSM9DS1_WHO_AM_I_M_RSP){
        return true;
    }
    return false;
}

void BerryIMU::enable(void) {
    switch(this->chip) {
        case LSM9DS0:
            this->enable_LSM9DS0();
            break;

        case LSM9DS1:
            this->enable_LSM9DS1();
            break;

        default:
            printf("Error: Unable to enable unknown chip\n");
            exit(1);
    }
}

void BerryIMU::enable_LSM9DS0(void) {
    // Enable accelerometer
    writeReg(ACCELEROMETER, LSM9DS0_CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuous update,  100Hz data rate
    writeReg(ACCELEROMETER, LSM9DS0_CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

    //Enable the magnetometer
    writeReg(MAGNETOMETER, LSM9DS0_CTRL_REG5_XM, 0b11110000); // Temp enable, M data rate = 50Hz
    writeReg(MAGNETOMETER, LSM9DS0_CTRL_REG6_XM, 0b01100000); // +/-12gauss
    writeReg(MAGNETOMETER, LSM9DS0_CTRL_REG7_XM, 0b00000000); // Continuous-conversion mode

    // Enable gyroscope
    writeReg(GYROSCOPE, LSM9DS0_CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
    writeReg(GYROSCOPE, LSM9DS0_CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
}

void BerryIMU::enable_LSM9DS1(void) {
    // Enable the accelerometer
    writeReg(ACCELEROMETER, LSM9DS1_CTRL_REG5_XL,0b00111000);   // z, y, x axis enabled for accelerometer
    writeReg(ACCELEROMETER, LSM9DS1_CTRL_REG6_XL,0b00101000);   // +/- 16g

    //Enable the magnetometer
    writeReg(MAGNETOMETER, LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
    writeReg(MAGNETOMETER, LSM9DS1_CTRL_REG2_M, 0b01000000);   // +/-12gauss
    writeReg(MAGNETOMETER, LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuos update
    writeReg(MAGNETOMETER, LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis

    // Enable the gyroscope
    writeReg(GYROSCOPE, LSM9DS1_CTRL_REG4,0b00111000);      // z, y, x axis enabled for gyro
    writeReg(GYROSCOPE, LSM9DS1_CTRL_REG1_G,0b10111000);    // Gyro ODR = 476Hz, 2000 dps
    writeReg(GYROSCOPE, LSM9DS1_ORIENT_CFG_G,0b10111000);   // Swap orientation 
}

void BerryIMU::selectDevice(BERRY_IMU_DEVICES device) {
    switch(this->chip){
        case LSM9DS0:
            i2cSelectDevice(this->file, LSM9DS0_DEV_ADDRESSES[device]);
            break;
        case LSM9DS1:
            i2cSelectDevice(this->file, LSM9DS1_DEV_ADDRESSES[device]);
            break;
    }
}

void BerryIMU::writeReg(BERRY_IMU_DEVICES device, uint8_t reg, uint8_t value) {
    this->selectDevice(BERRY_IMU_DEVICES::ACCELEROMETER);
	int result = i2c_smbus_write_byte_data(this->file, reg, value);
	if (result == -1){
		printf ("Failed to write byte to I2C accelerometer");
		exit(1);
	}
}

int main(void) {
    BerryIMU imu;

    imu.status();
    return 1;
}