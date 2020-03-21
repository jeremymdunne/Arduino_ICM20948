/**
 * Library to interface with the ICM20948 IMU 
 * No support for FIFO buffer or DMP stuff 
 * Currently I2C only, should be relatively easy to implement SPI 
 * 
 *  
 */ 


#ifndef _ARDUINO_ICM20948_H_
#define _ARDUINO_ICM20948_H_

#include <Arduino.h> 
#include <Wire.h> 

// most important definitions 

// comment out to not use mag 
#define USE_MAG
// select the communication mode 
#define ICM20948_I2C 


// error definitions 
// diable to prevent all messages (error messages are always present)
//#define ICM20498_DEBUG_SERIAL               
// disable to prevent printing verbose messages 
//#define ICM20948_DEBUG_VERBOSE              

// printing tags 
#define ICM20498_DEBUG_VERBOSE_STATEMENT        "ICM20948 DEBUG VERBOSE: "
#define ICM20948_DEBUG_ERROR_STATEMENT          "ICM20948 DEBUG ERROR: "


//error tags 
#define ICM20498_WHO_AM_I_WRONG_RESPONCE        -13
#define ICM20498_MAG_DEVICE_ID_WRONG_RESPONSE   -14
#define ICM20948_NO_NEW_DATA_AVAILABLE          13
#define ICM20948_RESET_TIMEOUT                  -15 


// I2C Address specifications 
#define ICM20948_I2C_ADDRESS_ADO_LOW            0x68
#define ICM20948_I2C_ADDRESS_ADO_HIGH           0x69        
#define ICM20948_I2C_SPEED                      400000L
#define ICM20948_MAGNETOMETER_ADDR              0x0C

// register definitions 
// some are not currently used and therefore not defined 
// all High and Low register pairs are only mentioned by the High address 

// Bank 0 Reg adresses   
#define ICM20948_WHO_AM_I_REG                   0x00
#define ICM20948_USER_CTRL_REG                  0x03
#define ICM20948_LP_CONFIG_REG                  0x05
#define ICM20948_PWR_MGMT_1_REG                 0x06
#define ICM20948_PWR_MGMT_2_REG                 0x07
#define ICM20948_INT_PIN_CFG_REG                0x0F
#define ICM20948_INT_ENABLE_REG                 0x10
#define ICM20948_INT_ENABLE_1_REG               0x11
#define ICM20948_INT_ENABLE_2_REG               0x12
#define ICM20948_INT_ENABLE_3_REG               0x13
#define ICM20948_I2C_MST_STATUS_REG             0x17
#define ICM20948_INT_STATUS_REG                 0x19
#define ICM20948_INT_STATUS_1_REG               0x1A
#define ICM20948_INT_STATUS_2_REG               0x1B
#define ICM20948_INT_STATUS_3_REG               0x1C
#define ICM20948_DELAY_TIMEH_REG                0x28
#define ICM20948_DELAY_TIMEL_REG                0x29
#define ICM20948_ACCEL_XOUT_H_REG               0x2D
#define ICM20948_ACCEL_YOUT_H_REG               0x2F
#define ICM20948_ACCEL_ZOUT_H_REG               0x31
#define ICM20948_GYRO_XOUT_H_REG                0x33
#define ICM20948_GYRO_YOUT_H_REG                0x35
#define ICM20948_GYRO_ZOUT_H_REG                0x37
#define ICM20948_TEMP_OUT_H_REG                 0x39
#define ICM20948_EXT_SLV_SENS_DATA_00_REG       0x3B 

#define ICM20948_FIFO_EN_1_REG                  0x66
#define ICM20948_FIFO_EN_2_REG                  0x67
#define ICM20948_FIFO_RST_REG                   0x68
#define ICM20948_FIFO_MODE_REG                  0x69
#define ICM20948_FIFO_COUNTH_REG                0x70
#define ICM20948_FIFO_R_W_REG                   0x72
#define ICM20948_DATA_RDY_STATUS_REG            0x74
#define ICM20948_FIFO_CFG_REG                   0x76
#define ICM20948_REG_BANK_SEL_REG               0x7F

// Bank 1 Addresses 
#define ICM20948_SELF_TEST_X_GYRO_REG           0x02 
#define ICM20948_SELF_TEST_Y_GYRO_REG           0x03
#define ICM20948_SELF_TEST_Z_GYRO_REG           0x04
#define ICM20948_SELF_TEST_X_ACCEL_REG          0x0E
#define ICM20948_SELF_TEST_Y_ACCEL_REG          0x0F
#define ICM20948_SELF_TEST_Z_ACCEL_REG          0x10
#define ICM20948_XA_OFFS_H_REG                  0x14
#define ICM20948_YA_OFFS_H_REG                  0x17
#define ICM20948_ZA_OFFS_H_REG                  0x1A
#define ICM20948_TIMEBASE_CORRECTION_PLL_REG    0x28 

// Bank 2 Addresses 
#define ICM20948_GYRO_SMPLRT_DIV_REG            0x00
#define ICM20948_GYRO_CONFIG_1_REG              0x01
#define ICM20948_GYRO_CONFIG_2_REG              0x02
#define ICM20948_XG_OFFS_USRH_REG               0x03 
#define ICM20948_YG_OFFS_USRH_REG               0x05
#define ICM20948_ZG_OFFS_USRH_REG               0x07 
#define ICM20948_ODR_ALIGN_EN_REG               0x09 
#define ICM20948_ACCEL_SMPLRT_DIV_1_REG         0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2_REG         0x11
#define ICM20948_ACCEL_INTEL_CTRL_REG           0x12 
#define ICM20948_ACCEL_WOM_THR_CTRL_REG         0x13 
#define ICM20948_ACCEL_CONFIG_REG               0x14 
#define ICM20948_ACCEL_CONFIG_2_REG             0x15
#define ICM20948_FSYNC_CONFIG_REG               0x52
#define ICM20948_TEMP_CONFIG_REG                0x53 
#define ICM20948_MOD_CTRL_USR_REG               0x54 


// Bank 3 Addresses 
#define ICM20948_I2C_MST_ODR_CONFIG_REG         0x00
#define ICM20948_I2C_MST_CTRL_REG               0x01
#define ICM20948_I2C_MST_DELAY_CTRL_REG         0x02
#define ICM20948_I2C_SLV0_ADDR_REG              0x03
#define ICM20948_I2C_SLV0_REG_REG               0x04
#define ICM20948_I2C_SLV0_CTRL_REG              0x05
#define ICM20948_I2C_SLV0_DO_REG                0x06


// Magnetometer 
#define ICM20948_MAG_DEVICE_ID_REG              0x01
#define ICM20948_MAG_STATUS_1_REG               0x10
#define ICM20948_MAG_X_DATA_LOW_REG             0x11
#define ICM20948_MAG_Y_DATA_LOW_REG             0x13
#define ICM20948_MAG_Z_DATA_LOW_REG             0x15 
#define ICM20948_MAG_STATUS_2_REG               0x18
#define ICM20948_MAG_CONTROL_2_REG              0x31 
#define ICM20948_MAG_CONTROL_3_REG              0x32 

// generals 
#define ICM20948_BANK_0                         0x00
#define ICM20948_BANK_1                         0x01
#define ICM20948_BANK_2                         0x02
#define ICM20948_BANK_3                         0x03

class Arduino_ICM20948{
public: 
    // define enums local to this class 

    // standard storage method 
    struct ICM20948_Raw_Data{
        float gyro[3];  // in degress/second 
        float accel[3]; // in gravities 
        float temperature; // in degrees c
        #ifdef USE_MAG
            float mag[3]; 
        #endif
        unsigned long long time_micros;  
    };

    // gyro range select 
    enum ICM20948_GYRO_FS_SEL{
        ICM20948_GYRO_250_DPS = 0x00,
        ICM20948_GYRO_500_DPS = 0x01, 
        ICM20948_GYRO_1000_DPS = 0x02,
        ICM20948_GYRO_2000_DPS = 0x03  
    };

    // accel range select 
    enum ICM20948_ACCEL_FS_SEL{ 
        ICM20948_ACCEL_2_G = 0x00,
        ICM20948_ACCEL_4_G = 0x01,
        ICM20948_ACCEL_8_G = 0x02,
        ICM20948_ACCEL_16_G = 0x03
    }; 

    /**
     * copy over available data to an outside struct 
     * @param dest destination to copy data into 
     * @return success or error code
     */
    int get_data(ICM20948_Raw_Data *dest); 

    /**
     * standard update method 
     * @return status code on whether data is available 
     */ 
    int update(); 

    /**
     * reset the imu 
     * @return error or success code 
     */
    int reset(); 

    #ifdef ICM20948_I2C
    /**
     * initialize the imu and begin processing data using I2C 
     */ 
    int begin(int addr = ICM20948_I2C_ADDRESS_ADO_LOW, bool initialize_I2C = true, ICM20948_GYRO_FS_SEL gyro_range = ICM20948_GYRO_250_DPS, ICM20948_ACCEL_FS_SEL accel_range = ICM20948_ACCEL_2_G); 
    
    #endif 

    

private: 
    // variables liable to change 
    float _accel_scale; // scalar to normalize to g's 
    float _gyro_scale;  // scalar to normalize to dps 
    float _mag_scale = 0.15; // scalar to normalize to microteslas  
    float _temp_scale = 1.0/333.87;;
    float _temp_offset = 21.0;
    ICM20948_Raw_Data _raw_data; 

    int _addr; 

    ICM20948_GYRO_FS_SEL _gyro_range; 
    ICM20948_ACCEL_FS_SEL _accel_range; 
    uint8_t _cur_reg_bank;

    /**
     * check connection to the imu 
     * check the who am i register for a response 
     * @return error or success code 
     */ 
    int check_who_am_i(); 

    /**
     * set the accel range
     * @param range the accel range to use 
     * @return error or success code 
     */ 
    int set_accel_range(ICM20948_ACCEL_FS_SEL accel_range); 

    /**
     * set the gyro range
     * @param range the gyro range to use 
     * @return error or success code 
     */ 
    int set_gyro_range(ICM20948_GYRO_FS_SEL gyro_range); 

    /**
     * set the sleep of the imu
     * @param sleep true for sleep (power down) false for on
     * @return error or success code
     */ 
    int set_sleep(bool sleep); 

    /**
     * set the clock source
     * currently only supports the recommended mode
     * @return error or success code
     */ 
    int set_clock_source();

    /**
     * read all data and store to local data
     * @return error or status code 
     */ 
    int readAll(); 

    /**
     * check for available data
     * @return whether data is available or not 
     */
    bool check_for_data(){
        return true;
    }

    /**
     * begin with default initialization 
     * typically called after the normal begin 
     * @return error or success code
     */ 
    int init(); 

    /**
     * init the magnetometer 
     * The magnetometer uses external slave sensor data buffers to work
     * @return error or success code
     */ 
    int init_mag();  

    #ifdef ICM20948_I2C
    /**
     * read from the imu
     * @param reg register to begin reading from
     * @param bank bank to read from (see above register addresses)
     * @param buffer buffer to store the read data into 
     * @param count number of bytes to read 
     * @return error or number of bits read
     */ 
    int icmRead(uint8_t reg, uint8_t bank, uint8_t *buffer, uint8_t count); 

    /**
     * read 8 bits from the imu 
     * @param reg the register to read 
     * @param bank the bank to read from 
     * @return value read from the imu 
     */ 
    uint8_t icmRead8(uint8_t reg, uint8_t bank); 

    /**
     * write to the imu 
     * @param reg the register to start writing to 
     * @param bank the bank to write to 
     * @param buffer the data to write to the imu 
     * @param count the number of bytes to write 
     * @return error or status code 
     */ 
    int icmWrite(uint8_t reg, uint8_t bank, uint8_t *buffer, uint8_t count); 

    /**
     * lower level write 8 to the imu
     * typically used to override the bank selection 
     * @param reg the register to write to 
     * @return error or success code
     */ 
    int icmWrite8(uint8_t reg, uint8_t value); 

    /**
     * write 8 bits to the imu 
     * @param reg the register to write to 
     * @param bank the bank to write to 
     * @param value the value to write
     * @return error or success code 
     */
    int icmWrite8(uint8_t reg, uint8_t bank, uint8_t value); 

    /**
     * set the bank to read from 
     * @param bank the bank to write to 
     * @param force force writing the bank if true 
     * @return error or success code 
     */ 
    int setBank(uint8_t bank, bool force = false); 

    #ifdef USE_MAG
    /**
     * set the slave I2C speed 
     * default to 400 khz for now 
     * @return error or success code
     */
    int set_slave_I2C_speed(); 

    /**
     * enable I2C master for the imu 
     * @return error or success code 
     */
    int enable_I2C_master(); 

    /**
     * set the slave device to read mode 
     * @return error or success code
     */
    int set_slave_read(); 

    /**
     * set the slave device to write mode
     * @return error or success code
     */
    int set_slave_write(); 

    /**
     * read to the mag 
     * @param reg register to read 
     * @return value at the registor
     */
    uint8_t read_mag(uint8_t reg); 

    /**
     * write to the mag 
     * @param reg register to write 
     * @param value value to write 
     * @return error or success code 
     */
    int write_mag(uint8_t reg, uint8_t value);

    /**
     * reset the mag 
     * @return error or success code 
     */
    int reset_mag(); 

    #endif 

    #endif 

    #ifdef ICM20498_DEBUG_SERIAL

    /**
     * Print an error message to the standard serial interface 
     * Add a error tag when printing for ease of use
     * @param message message to display to user 
     */ 
    void printError(String message); 

    /**
     * Print a verbose message to the standard serial interface
     * Add a verbose tag, and don't print if verbose is disabled
     * @param message message to display to the user 
     */ 
    void printVerbose(String message); 
    #endif 

}; 



#endif 