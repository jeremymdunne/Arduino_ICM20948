#include <Arduino_ICM20948.h>

int Arduino_ICM20948::get_data(ICM20948_Raw_Data *dest){
    // perform a memcopy on structs 
    memcpy(dest,&_raw_data,sizeof(_raw_data)); 
    return 0; 
}

int Arduino_ICM20948::update(){
    // read new data 
    int status = readAll(); 
    if(status == ICM20948_NO_NEW_DATA_AVAILABLE) return 0; 
    else if(status >= 0) return 1; 
    else return status; 
}

int Arduino_ICM20948::init(){
    // first check who am i 
    int status = check_who_am_i(); 
    if(status < 0) return status;
    // reset 
    reset(); 
    //delay 
    delay(20);
    // set ranges 
    // wake up the device 
    status = set_sleep(false); 
    // set the clock source 
    status = set_clock_source(); 
    if(status < 0) return status;
    status = set_accel_range(_accel_range);
    if(status < 0) return status;
    status = set_gyro_range(_gyro_range); 
    if(status < 0) return status;
    
    #ifdef USE_MAG
     init_mag(); 
    #endif 
    // should be good to go! 
    return status; 
}

int Arduino_ICM20948::reset(){
    //set the reset bit, wait until it clears
    uint8_t base = icmRead8(ICM20948_PWR_MGMT_1_REG,ICM20948_BANK_0);
    base &= 0b01111111;
    base |= (1<<7);
    icmWrite8(ICM20948_PWR_MGMT_1_REG,base);
    //give a small delay for the reset
    delay(2);
    //Serial.println(icmRead8(ICM20948_PWR_MGMT_1_REG),BIN);
    //wait until its free
    bool done = false;
    long start = millis();
    while(!done && (millis() - start < 200)){
    //read it
    base = icmRead8(ICM20948_PWR_MGMT_1_REG,ICM20948_BANK_0);
    if((base & 0b10000000) == 0){
        done = true;
        #ifdef ICM20498_DEBUG_SERIAL
            printVerbose("RESET SUCCESS"); 
        #endif
    }
    }
    if(!done){
        //timeout occurred
        #ifdef ICM20498_DEBUG_SERIAL
            printError("RESET TIMEOUT");
        #endif 
        return ICM20948_RESET_TIMEOUT;
    }
return 0;
}

int Arduino_ICM20948::readAll(){
  if(!check_for_data()) return ICM20948_NO_NEW_DATA_AVAILABLE;
  #ifdef USE_MAG
    uint8_t tempData[20];
    icmRead(ICM20948_ACCEL_XOUT_H_REG, ICM20948_BANK_0, &tempData[0], 20);
  #else 
    uint8_t tempData[14]; 
    icmRead(ICM20948_ACCEL_XOUT_H_REG, ICM20948_BANK_0, &tempData[0], 14);
  #endif  
  _raw_data.accel[0] = (int16_t)(tempData[0]<<8 | tempData[1]) * _accel_scale;
  _raw_data.accel[1] = (int16_t)(tempData[2]<<8 | tempData[3]) * _accel_scale;
  _raw_data.accel[2] = (int16_t)(tempData[4]<<8 | tempData[5]) * _accel_scale;
  _raw_data.gyro[0] = (int16_t)(tempData[6]<<8 | tempData[7]) * _gyro_scale;
  _raw_data.gyro[1] = (int16_t)(tempData[8]<<8 | tempData[9]) * _gyro_scale;
  _raw_data.gyro[2] = (int16_t)(tempData[10]<<8 | tempData[11]) * _gyro_scale;
  _raw_data.temperature = (int16_t)(tempData[12]<<8 | tempData[13]) * _temp_scale + _temp_offset;
  #ifdef USE_MAG
    _raw_data.mag[0] = (int16_t)(tempData[15]<<8 | tempData[14]) * _mag_scale;
    _raw_data.mag[1] = (int16_t)(tempData[17]<<8 | tempData[16]) * _mag_scale;
    _raw_data.mag[2] = (int16_t)(tempData[19]<<8 | tempData[18]) * _mag_scale;
  #endif 
  _raw_data.time_micros = micros();
  return 0;
}

int Arduino_ICM20948::check_who_am_i(){
    // read the WHO_AM_I buffer
    uint8_t who_am_i = icmRead8(ICM20948_WHO_AM_I_REG, ICM20948_BANK_0); 
    // check if the value received is as expected 
    if(who_am_i != 0xEA){
        // something is wrong... 
        #ifdef ICM20498_DEBUG_SERIAL
            printError("WHO AM I WRONG RESPONSE, RECEIVED " + String(who_am_i)); 
        #endif 
        return ICM20498_WHO_AM_I_WRONG_RESPONCE; 
    }    
    #ifdef ICM20498_DEBUG_SERIAL
        printVerbose("WHO AM I CORRECT RESPONSE, RECEIVED: " + String(who_am_i)); 
    #endif 
    return 0; 
}

int Arduino_ICM20948::set_accel_range(ICM20948_ACCEL_FS_SEL accel_range){
    // get the base value, don't change what's not necessary 
    uint8_t base = icmRead8(ICM20948_ACCEL_CONFIG_REG, ICM20948_BANK_2); 
    base &= 0b11111001;
    base |= (accel_range << 1);
    icmWrite8(ICM20948_ACCEL_CONFIG_REG, base);
    switch(accel_range){
    case(ICM20948_ACCEL_2_G):
        _accel_scale = 1/16384.0;
        break;
    case(ICM20948_ACCEL_4_G):
        _accel_scale = 1/8192.0;
        break;
    case(ICM20948_ACCEL_8_G):
        _accel_scale = 1/4096.0;
        break;
    case(ICM20948_ACCEL_16_G):
        _accel_scale = 1/2048.0;
        break;
    }
    #ifdef ICM20498_DEBUG_SERIAL
        printVerbose("ACCEL RANGE SET WITH SCALE: " + String(_accel_scale)); 
    #endif
    return 0;
} 

int Arduino_ICM20948::set_gyro_range(ICM20948_GYRO_FS_SEL gyro_range){
      //set the range then update the scale factor
  uint8_t base = icmRead8(ICM20948_GYRO_CONFIG_1_REG, ICM20948_BANK_2); //in bank 2
  base &= 0b11111001;
  base |= gyro_range << 1;
  icmWrite8(ICM20948_GYRO_CONFIG_1_REG, base); //in bank 2
  switch(gyro_range){
    case(ICM20948_GYRO_250_DPS):
      _gyro_scale = 1/131.0;
      break;
    case(ICM20948_GYRO_500_DPS):
      _gyro_scale = 1/65.5;
      break;
    case(ICM20948_GYRO_1000_DPS):
      _gyro_scale = 1/32.8;
      break;
    case(ICM20948_GYRO_2000_DPS):
      _gyro_scale = 1/16.4;
      break;
  }
    #ifdef ICM20498_DEBUG_SERIAL
        printVerbose("GYRO RANGE SET WITH SCALE: " + String(_gyro_scale)); 
    #endif
  return 0;
}

int Arduino_ICM20948::set_sleep(bool sleep){
    // grab the register's current content 
    uint8_t base = icmRead8(ICM20948_PWR_MGMT_1_REG, ICM20948_BANK_0);
    base &= 0b10111111;
    // modify the necessary bit 
    if(sleep) base |= (1 << 6);
    // write the correct value 
    icmWrite8(ICM20948_PWR_MGMT_1_REG, base);
    return 0;
}

 int Arduino_ICM20948::set_clock_source(){
    // grab the register's current content
    uint8_t base = icmRead8(ICM20948_PWR_MGMT_1_REG, ICM20948_BANK_0);
    base &= 0b11111100;
    // modify the necessary bit
    base |= 0x01;
    // write the correct value 
    icmWrite8(ICM20948_PWR_MGMT_1_REG, base);
    return 0;
 }



#ifdef ICM20948_I2C

int Arduino_ICM20948::begin(int addr, bool initialize_I2C, ICM20948_GYRO_FS_SEL gyro_range, ICM20948_ACCEL_FS_SEL accel_range){
    _addr = addr; 
    _accel_range = accel_range;
    _gyro_range = gyro_range; 

    if(initialize_I2C){
        Wire.begin();
        Wire.setClock(ICM20948_I2C_SPEED); 
    }
    // call the normal begin 
    return init();
    
}

int Arduino_ICM20948::icmWrite(uint8_t reg, uint8_t bank, uint8_t *buffer, uint8_t count){
    // set the bank 
    int status = setBank(bank); 
    if(status < 0){
        //something happened, return the error 
        return status; 
    }
    // send the data 
    Wire.beginTransmission(_addr); 
    Wire.write(reg); 
    Wire.write(buffer,count); 
    Wire.endTransmission(); 
    return 0; 
}

int Arduino_ICM20948::icmWrite8(uint8_t reg, uint8_t value){
    // write the data 
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value); 
    Wire.endTransmission();
    // TODO work out better return code 
    return 0; 
}

int Arduino_ICM20948::icmWrite8(uint8_t reg, uint8_t bank, uint8_t value){
    // set the bank 
    int status = setBank(bank); 
    if(status < 0){
        //something happened, return the error 
        return status; 
    }
    // write the data 
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value); 
    Wire.endTransmission();
    // TODO work out better return code 
    return 0; 
}

int Arduino_ICM20948::icmRead(uint8_t reg, uint8_t bank, uint8_t *buffer, uint8_t count){
    // set the bank 
    int status = setBank(bank); 
    if(status < 0){
        //something happened, return the error 
        return status; 
    }
    // request the available data 
    Wire.beginTransmission(_addr);
    Wire.write(reg); 
    Wire.endTransmission(); 
    Wire.requestFrom(_addr,count); 
    int counter = 0; 
    while(Wire.available()){
        buffer[counter] = Wire.read(); 
        counter ++; 
    }
    return counter; 
}

uint8_t Arduino_ICM20948::icmRead8(uint8_t reg, uint8_t bank){
    // temp store the buffer
    uint8_t value; 
    // read into the buffer 
    icmRead(reg, bank, &value, 1); 
    // return the value
    return value; 
}

int Arduino_ICM20948::setBank(uint8_t bank, bool force){
    // check if forced 
    int status = 0; 
    if(force){
        //write the bank no matter what 
        //shift the value 
        uint8_t value = bank << 4; 
        status = icmWrite8(ICM20948_REG_BANK_SEL_REG, value); 
        if(status >= 0){
            _cur_reg_bank = bank; 
        }
        return status; 
    }
    else{
        // check if already in the right bank 
        if(bank == _cur_reg_bank){
            //do nothing, return success 
            return status; 
        }
        // otherwise, switch bank 
        uint8_t value = bank << 4; 
        status = icmWrite8(ICM20948_REG_BANK_SEL_REG, value); 
        if(status >= 0){
            _cur_reg_bank = bank; 
        }
        return status;
    }
    return -1;
}

int Arduino_ICM20948::set_slave_I2C_speed(){
    //default to 400 khz
    return icmWrite8(ICM20948_I2C_MST_CTRL_REG, ICM20948_BANK_3, 0x07); 
}

int Arduino_ICM20948::enable_I2C_master(){
  uint8_t base = icmRead8(ICM20948_USER_CTRL_REG, ICM20948_BANK_0);
  base &= 0b11011111;
  base |= 1 << 5;
  return icmWrite8(ICM20948_USER_CTRL_REG, 1 << 5);
}


int Arduino_ICM20948::set_slave_read(){
    uint8_t base = icmRead8(ICM20948_I2C_SLV0_ADDR_REG,ICM20948_BANK_3);
    base &= 0b01111111;
    base |= 1 << 7;
    return icmWrite8(ICM20948_I2C_SLV0_ADDR_REG, base);

}

int Arduino_ICM20948::set_slave_write(){
    uint8_t base = icmRead8(ICM20948_I2C_SLV0_ADDR_REG, ICM20948_BANK_3);
    base &= 0b01111111;
    return icmWrite8(ICM20948_I2C_SLV0_ADDR_REG, base);
}


int Arduino_ICM20948::write_mag(uint8_t reg, uint8_t value){
    set_slave_write();
    icmWrite8(ICM20948_I2C_SLV0_REG_REG, ICM20948_BANK_3, reg);
    icmWrite8(ICM20948_I2C_SLV0_DO_REG,value);
    icmWrite8(ICM20948_I2C_SLV0_CTRL_REG,1 << 7|1);
    return 0;
}

uint8_t Arduino_ICM20948::read_mag(uint8_t reg){
    //read through the slave mode
    //tell icm to read data
    set_slave_read();
    icmWrite8(ICM20948_I2C_SLV0_REG_REG, ICM20948_BANK_3, reg);
    icmWrite8(ICM20948_I2C_SLV0_CTRL_REG,1 << 7 | 1);
    delay(10);
    return icmRead8(ICM20948_EXT_SLV_SENS_DATA_00_REG, ICM20948_BANK_0);
}

int Arduino_ICM20948::reset_mag(){
    //reset the master i2c bus
    uint8_t base = icmRead8(ICM20948_USER_CTRL_REG, ICM20948_BANK_0);
    base &= 0b11111101;
    base |= 1 << 1;
    icmWrite8(ICM20948_USER_CTRL_REG, base);
    delay(1);
    write_mag(ICM20948_USER_CTRL_REG, 1); //reset
    delay(10);
    return 0;
}


#ifdef USE_MAG
int Arduino_ICM20948::init_mag(){
    //set the adress, do the appropriate, etc. etc.
    set_slave_I2C_speed();
    enable_I2C_master();

    // set the slave 0 I2C address 
    icmWrite8(ICM20948_I2C_SLV0_ADDR_REG, ICM20948_BANK_3, ICM20948_MAGNETOMETER_ADDR);

    // check the device id 
    uint8_t whoAmIResponse = read_mag(ICM20948_MAG_DEVICE_ID_REG);
    //Serial.println(whoAmIResponse);
    if(whoAmIResponse != 9){
        #ifdef ICM20498_DEBUG_SERIAL
            printError("MAG DEVICE ID WRONG, RECEIVED: " + String(whoAmIResponse)); 
        #endif 
        return ICM20498_MAG_DEVICE_ID_WRONG_RESPONSE;
    } 
    #ifdef ICM20498_DEBUG_SERIAL
        printVerbose("MAG DEVIE ID SUCCESS"); 
    #endif 
    //go write to be continuous mode
    reset_mag();
    //give it a while to reset!
    delay(100);
    // set to continuous mode 4 
    write_mag(ICM20948_MAG_CONTROL_2_REG,0b01000); 
    
    //set up to place data in the ext sense data slots
    set_slave_read();
    // set the register to begin reading from 
    icmWrite8(ICM20948_I2C_SLV0_REG_REG,ICM20948_BANK_3, ICM20948_MAG_X_DATA_LOW_REG);
    // set the expected read size 
    icmWrite8(ICM20948_I2C_SLV0_CTRL_REG,ICM20948_BANK_3,1 << 7 | 8);
    // extra delay for caution 
    delay(10);
    return 0;
}

#endif 

#endif 

#ifdef ICM20498_DEBUG_SERIAL

void Arduino_ICM20948::printError(String message){
    // print a debug tag 
    Serial.print(ICM20948_DEBUG_ERROR_STATEMENT); 
    Serial.print(" ");
    Serial.print(message); 
    Serial.print('\n');
}


void Arduino_ICM20948::printVerbose(String message){
    // print a debug tag 
    #ifdef ICM20948_DEBUG_VERBOSE
        Serial.print(ICM20498_DEBUG_VERBOSE_STATEMENT); 
        Serial.print(" ");
        Serial.print(message); 
        Serial.print('\n');
    #endif 
}


#endif 

