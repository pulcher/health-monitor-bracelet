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
uint8_t _activeLEDs;

// access methods
bool is_max30102_available(struct device *i2c_dev);
uint8_t get_max30102_part_id(struct device *i2c_dev);

// confiuration methods
void max30102_softReset(struct device *i2c_dev);
void max30102_sensorConfiguration(struct device *i2c_dev, uint8_t ledBrightness, uint8_t sampleAverage, uint8_t ledMode, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange);

// utility methods
uint8_t max30102_readReg(struct device *i2c_dev, uint8_t reg, const void* pBuf, uint8_t size);
int max30102_writeReg(struct device *i2c_dev, uint8_t reg, const void* pBuf, uint8_t size);
uint8_t max30102_write_read_reg(struct device *i2c_dev, uint8_t reg, void* pBuf, uint8_t size);

bool is_max30102_available(struct device *i2c_dev) {

    msgs[0].buf = &dst;
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    error = i2c_transfer(i2c_dev, &msgs[0], 1, MAX30102_IIC_ADDRESS);
    
    return error == 0;
}

uint8_t get_max30102_part_id(struct device *i2c_dev)
{
  uint8_t byteTemp, error;
  error = max30102_write_read_reg(i2c_dev, MAX30102_PARTID, &byteTemp, 1);
  return byteTemp;
}

void max30102_softReset(struct device *i2c_dev)
{
  sMode_t modeReg;
  max30102_readReg(i2c_dev, MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.reset = 1;
  max30102_writeReg(i2c_dev, MAX30102_MODECONFIG, &modeReg, 1);
  uint32_t startTime = k_uptime_get();
  while (k_uptime_get() - startTime < 100) {
    max30102_readReg(i2c_dev, MAX30102_MODECONFIG, &modeReg, 1);
    if (modeReg.reset == 0) break; 
    k_sleep(K_MSEC(1));
  }
}

void max30102_sensorConfiguration(struct device *i2c_dev, uint8_t ledBrightness, uint8_t sampleAverage,
                                  uint8_t ledMode, uint8_t sampleRate, uint8_t pulseWidth,
                                  uint8_t adcRange)
{
  setFIFOAverage(sampleAverage);

  setADCRange(adcRange);

  setSampleRate(sampleRate);

  setPulseWidth(pulseWidth);

  setPulseAmplitudeRed(ledBrightness);
  setPulseAmplitudeIR(ledBrightness);

  enableSlot(1, SLOT_RED_LED);
  if (ledMode > MODE_REDONLY) enableSlot(2, SLOT_IR_LED);

  setLEDMode(ledMode);

  if (ledMode == MODE_REDONLY) {
    _activeLEDs = 1;
  } else {
    _activeLEDs = 2;
  }

  enableFIFORollover(); 
  resetFIFO(); 
}


// Utilities
uint8_t max30102_readReg(struct device *i2c_dev, uint8_t reg, const void* pBuf, uint8_t size)
{
    return max30102_write_read_reg(i2c_dev, reg, pBuf, size);
}

int max30102_writeReg(struct device *i2c_dev, uint8_t reg, const void* pBuf, uint8_t size)
{
    return i2c_write(i2c_dev, pBuf, size, reg);
}


uint8_t max30102_write_read_reg(struct device *i2c_dev, uint8_t reg, void* pBuf, uint8_t size)
{
    int error;
    error = i2c_write_read(i2c_dev, MAX30102_IIC_ADDRESS, &reg, 1, pBuf, size);

    if (error) {
        printk("ERROR: couldn't communicate with max30102: %d", error);
    }

    return error;
}

