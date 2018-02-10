#include "ch.h"
#include "hal.h"
#include "ch_test.h"
#include "string.h"
#include "Accelerometer.h"



#define device_address 0x6a
#define acceleration_control_1_address 0x10


int AccelerometerInit(){

    static const I2CConfig i2ccfg = {
        OPMODE_I2C,
        100000,
        STD_DUTY_CYCLE,
    };



    palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7) );
     palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7) );

    i2cStart(&I2CD1, &i2ccfg);
    sdStart(&SD3, NULL);

    return 1;

}

/*
    Linear acceleration sensor control register 1(sensor setup)
    write to it ODR_XL[0:3] - data rate/power mode, FS_XL[1:0] Measurent scale, BW_XL[1:0] anti-aliasing bandwidth[1:0]
    XL_HM_MODE default = 0
    write byte: 0b01000011 normal 104Hz +-2g 50Hz filter (0x43)
    default powered down
 */

int AccelerometerConfig(uint8_t writeByte){



    uint8_t control_1_write_byte = writeByte;

    volatile uint8_t regcontent;

    uint8_t data[2] = {acceleration_control_1_address, control_1_write_byte};
    uint8_t *transmit;
    transmit = data;
    volatile msg_t err = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 2, (uint8_t *) &regcontent, 0, MS2ST(1000));

    (void) err;

    return 1;

}

/*
 *
 * Accesses Linear Acceleration Registers
 * Gets two bytes per axis
 * two bytes two's complement of acceleration value
 * merges two unsigned bytes into signed 16 bit value
 * (value/32766)*scale if value is positive, or (value/32767)*scale if value is negative
 * that number is in g's where scale is set in accelerometer config
 *
 */
int AccelerometerGetValues(int16_t* buff){

    uint8_t x0_data;
    uint8_t x1_data;

    uint8_t y0_data;
    uint8_t y1_data;

    uint8_t z0_data;
    uint8_t z1_data;

    //x axis output LSbyte
    uint8_t x0 = 0x28;
    //x axis output MSbyte
    uint8_t x1 = 0x29;

    //y axis output LSbyte
    uint8_t y0 = 0x2A;
    //y axis output MSbyte
    uint8_t y1 = 0x2B;

    //z axis output LSbyte
    uint8_t z0 = 0x2C;
    //z axis output MSbyte
    uint8_t z1 = 0x2D;

    volatile uint8_t regcontent;

    uint8_t *transmit;
    transmit = &x0;

    volatile msg_t err2 = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
    x0_data = regcontent;

    transmit = &x1;
    volatile msg_t err3 = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
    x1_data = regcontent;

    transmit = &y0;
    volatile msg_t err4 = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
    y0_data = regcontent;

    transmit = &y1;
    volatile msg_t err5 = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
    y1_data = regcontent;

    transmit = &z0;
    volatile msg_t err6 = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
    z0_data = regcontent;

    transmit = &z1;
    volatile msg_t err7 = i2cMasterTransmitTimeout(&I2CD1, device_address, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
    z1_data = regcontent;

    uint16_t u_x_data = (x1_data << 8)|x0_data;
    int16_t x_data;
    memcpy(&x_data, &u_x_data, sizeof(u_x_data));

    uint16_t u_y_data = (y1_data << 8)|y0_data;
    int16_t y_data;
    memcpy(&y_data, &u_y_data, sizeof(u_y_data));

    uint16_t u_z_data = (z1_data << 8)|z0_data;
    int16_t z_data;
    memcpy(&z_data, &u_z_data, sizeof(u_z_data));

    buff[0] = x_data;
    buff[1] = y_data;
    buff[2] = z_data;

    (void) err2;
    (void) err3;
    (void) err4;
    (void) err5;
    (void) err6;
    (void) err7;

    return 1;


}

