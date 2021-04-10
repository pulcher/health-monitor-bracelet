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
void max30102_setLEDMode(struct device *i2c_dev, uint8_t ledMode);
void setFIFOAverage(struct device *i2c_dev, uint8_t samples);
void max30102_setADCRange(struct device *i2c_dev, uint8_t adcRange);
void max30102_setSampleRate(struct device *i2c_dev, uint8_t sampleRate);
void max30102_setPulseWidth(struct device *i2c_dev, uint8_t pulseWidth);
void max30102_setPulseAmplitudeRed(struct device *i2c_dev, uint8_t amplitude);
void max30102_setPulseAmplitudeIR(struct device *i2c_dev, uint8_t amplitude);
void max30102_enableSlot(struct device *i2c_dev, uint8_t slotNumber, uint8_t device);
void max30102_enableFIFORollover(struct device *i2c_dev);
void max30102_resetFIFO(struct device *i2c_dev);

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
  max30102_setFIFOAverage(i2c_dev, sampleAverage);

  max30102_setADCRange(i2c_dev, adcRange);

  max30102_setSampleRate(i2c_dev, sampleRate);

  max30102_setPulseWidth(i2c_dev, pulseWidth);

  max30102_setPulseAmplitudeRed(i2c_dev, ledBrightness);
  max30102_setPulseAmplitudeIR(i2c_dev, ledBrightness);

  max30102_enableSlot(i2c_dev, 1, SLOT_RED_LED);
  if (ledMode > MODE_REDONLY) max30102_enableSlot(i2c_dev, 2, SLOT_IR_LED);

  max30102_setLEDMode(i2c_dev, ledMode);

  if (ledMode == MODE_REDONLY) {
    _activeLEDs = 1;
  } else {
    _activeLEDs = 2;
  }

  max30102_enableFIFORollover(i2c_dev); 
  max30102_resetFIFO(i2c_dev); 
}

void max30102_setLEDMode(struct device *i2c_dev, uint8_t ledMode)
{
  sMode_t modeReg;
  max30102_readReg(i2c_dev, MAX30102_MODECONFIG, &modeReg, 1);
  modeReg.ledMode = ledMode;
  max30102_writeReg(i2c_dev, MAX30102_MODECONFIG, &modeReg, 1);
}

void max30102_setFIFOAverage(struct device *i2c_dev, uint8_t numberOfSamples)
{
  sFIFO_t FIFOReg;
  max30102_readReg(i2c_dev, MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.sampleAverag = numberOfSamples;
  max30102_writeReg(i2c_dev, MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void max30102_setADCRange(struct device *i2c_dev, uint8_t adcRange)
{
  sParticle_t particleReg;
  max30102_readReg(i2c_dev, MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.adcRange = adcRange;
  max30102_writeReg(i2c_dev, MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void max30102_setSampleRate(struct device *i2c_dev, uint8_t sampleRate)
{
  sParticle_t particleReg;
  max30102_readReg(i2c_dev, MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.sampleRate = sampleRate;
  max30102_writeReg(i2c_dev, MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void max30102_setPulseWidth(struct device *i2c_dev, uint8_t pulseWidth)
{
  sParticle_t particleReg;
  max30102_readReg(i2c_dev, MAX30102_PARTICLECONFIG, &particleReg, 1);
  particleReg.pulseWidth = pulseWidth;
  max30102_writeReg(i2c_dev, MAX30102_PARTICLECONFIG, &particleReg, 1);
}

void max30102_setPulseAmplitudeRed(struct device *i2c_dev, uint8_t amplitude)
{
  uint8_t byteTemp = amplitude;
  max30102_writeReg(i2c_dev, MAX30102_LED1_PULSEAMP, &byteTemp, 1);
}

void max30102_setPulseAmplitudeIR(struct device *i2c_dev, uint8_t amplitude)
{
  uint8_t byteTemp = amplitude;
  max30102_writeReg(i2c_dev, MAX30102_LED2_PULSEAMP, &byteTemp, 1);
}

void max30102_enableSlot(struct device *i2c_dev, uint8_t slotNumber, uint8_t device)
{
  sMultiLED_t multiLEDReg;
  switch (slotNumber) {
  case (1):
    max30102_readReg(i2c_dev, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    multiLEDReg.SLOT1 = device;
    max30102_writeReg(i2c_dev, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    break;
  case (2):
    max30102_readReg(i2c_dev, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    multiLEDReg.SLOT2 = device;
    max30102_writeReg(i2c_dev, MAX30102_MULTILEDCONFIG1, &multiLEDReg, 1);
    break;
  default:
    break;
  }
}

void max30102_enableFIFORollover(struct device *i2c_dev)
{
  sFIFO_t FIFOReg;
  max30102_readReg(i2c_dev, MAX30102_FIFOCONFIG, &FIFOReg, 1);
  FIFOReg.RollOver = 1;
  max30102_writeReg(i2c_dev, MAX30102_FIFOCONFIG, &FIFOReg, 1);
}

void max30102_resetFIFO(struct device *i2c_dev)
{
  uint8_t byteTemp = 0;
  max30102_writeReg(i2c_dev, MAX30102_FIFOWRITEPTR, &byteTemp, 1);
  max30102_writeReg(i2c_dev, MAX30102_FIFOOVERFLOW, &byteTemp, 1);
  max30102_writeReg(i2c_dev, MAX30102_FIFOREADPTR, &byteTemp, 1);
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

