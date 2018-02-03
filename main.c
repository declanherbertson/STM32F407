/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "ch_test.h"
#include "string.h" //added ask alan
#include "Accelerometer.h"

/*
//added code
static const I2CConfig i2ccfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};

*/



/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
/*

  //I2C (accelerometer)
   palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
   palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

   palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7) );
    palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7) );

   i2cStart(&I2CD1, &i2ccfg);
   sdStart(&SD3, NULL);



   //find effective presets for device to config

   //write bytes is first num of transmit

   //read bytes is second num of transmit

   //to read variables from debugger
   //break after read - i2cMasterTransmitTimeout
   //gdb = debugger;
   //change o3 to o0 in makefile to lower optimization for debugging
   //b filename:linenumber - sets a breakpoint in code
   //mon reset halt -halts cpu at top of code; p var -prints variable; c -continue to break point
   //set breakpoint; halt cpu; continue to breakpoint; print variable

   //to transmit multiple bytes, make regaddr a byte array

    //to push to git - git add .;git commit -a -m "";git push -u origin master -f

   //to pull from git - git pull origin master


   //Returns Values 69h
   uint8_t test_address = 0x0f;

   //Linear acceleration sensor control register 1(sensor setup)
   //write to it ODR_XL[0:3] - data rate/power mode, FS_XL[1:0] Measurent scale, BW_XL[1:0] anti-aliasing bandwidth[1:0]
   //XL_HM_MODE default = 0
   //write byte: 0b01000011 normal 104Hz +-2g 50Hz filter
   //default powered down
   uint8_t acceleration_control_1_address = 0x10;
   uint8_t control_1_write_byte = 0x43;

   //Linear acceleration sensor control 9(enables x,y,z axis)
   //default enabled
   uint8_t acceleration_control_9_address = 0x19;
   uint8_t control_9_write_byte = 0x38;



   //Wake up sources register(wakeup x,y,z event detection)
   //read register
   uint8_t event_detection = 0x1B;

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

   uint8_t x0_data;
   uint8_t x1_data;

   uint8_t y0_data;
   uint8_t y1_data;

   uint8_t z0_data;
   uint8_t z1_data;



   while(1) {

        //sdWrite(&SD3, "H", 1);




       volatile uint8_t regcontent;

       uint8_t data[2] = {test_address};
       uint8_t *transmit;
       transmit = data;
       volatile msg_t err = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));


       data[0] = acceleration_control_1_address;
       data[1] = control_1_write_byte;
       transmit = data;
       volatile msg_t err1 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 2, (uint8_t *) &regcontent, 0, MS2ST(1000));


       data[0] =x0;
       transmit = data;
       volatile msg_t err2 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
       x0_data = regcontent;



       data[0] =x1;
       transmit = data;
       volatile msg_t err3 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
       x1_data = regcontent;

       uint16_t u_x_data = (x1_data << 8)|x0_data;
       int16_t x_data;

       memcpy(&x_data, &u_x_data, sizeof(u_x_data));

       data[0] =y0;
       transmit = data;
       volatile msg_t err4 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
       y0_data = regcontent;

       data[0] =y1;
       transmit = data;
       volatile msg_t err5 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
       y1_data = regcontent;

       uint16_t u_y_data = (y1_data << 8)|y0_data;
       int16_t y_data;

       memcpy(&y_data, &u_y_data, sizeof(u_y_data));


       data[0] =z0;
       transmit = data;
       volatile msg_t err6 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
       z0_data = regcontent;


       data[0] =z1;
       transmit = data;
       volatile msg_t err7 = i2cMasterTransmitTimeout(&I2CD1, 0x6a, transmit, 1, (uint8_t *) &regcontent, 1, MS2ST(1000));
       z1_data = regcontent;


       uint16_t u_z_data = (z1_data << 8)|z0_data;
       int16_t z_data;
       memcpy(&z_data, &u_z_data, sizeof(u_z_data));


       //printf //plug wire into gold square hole on board to solo male pin
       //displays over minicom - first sudo dmesg | tail , (sudo minicom -s) in serial port setup - /dev/ttyACM0 , /var/lock , 38400 8N1 , No , No
       //printing integer values - (2/32766) * value or (2/32767) *value(negative) = value in g's when set to 2 g max
      chprintf((BaseSequentialStream *)&SD3, "%d, %d, %d \r\n",x_data, y_data, z_data);


//test


       //stop compiler complaining
       (void) err;
       (void) err1;
       (void) err2;
       (void) err3;
       (void) err4;
       (void) err5;
       (void) err6;
       (void) err7;




*/
  int16_t buff[3];
  while(1){

       AccelerometerInit();
       AccelerometerConfig(1,1);
       AccelerometerGetValues(buff);



       chprintf((BaseSequentialStream *)&SD3, "%d, %d, %d \r\n",buff[0],buff[1], buff[2]);
       chThdSleepMilliseconds(100);

     }






  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (palReadPad(GPIOA, GPIOA_BUTTON))
      test_execute((BaseSequentialStream *)&SD2);
    chThdSleepMilliseconds(500);
  }

}
