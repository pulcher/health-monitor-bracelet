/*
Driver for MAX30102
Coded specifically for the nRF5340DK
*/
#include <device.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <drivers/i2c.h>
#include "max30102.h"

// to be private variables
struct i2c_msg msgs[1];
uint8_t dst = 1;
uint8_t error = -1;


bool is_max30102_available(struct device *i2c_dev) {

    msgs[0].buf = &dst;
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    error = i2c_transfer(i2c_dev, &msgs[0], 1, MAX30102_IIC_ADDRESS);
    
    return error == 0;
}