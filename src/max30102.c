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
sSenseBuf_t senseBuf;

bool is_max30102_available(struct device *i2c_dev) {

    msgs[0].buf = &dst;
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    error = i2c_transfer(i2c_dev, &msgs[0], 1, MAX30102_IIC_ADDRESS);
    
    return error == 0;
}

uint8_t get_max30102_part_id(struct device *i2c_dev)
{
  uint8_t byteTemp, revisionId, error;
  error = max30102_write_read_reg(i2c_dev, MAX30102_PARTID, &byteTemp, 1);
  printk("Found part id: %x\n", byteTemp);

  error = max30102_write_read_reg(i2c_dev, MAX30102_REVISION_ID, &revisionId, 1);
  printk("Found revision id: %x\n", revisionId);
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
  max30102_write_read_reg(i2c_dev, MAX30102_MODECONFIG, &modeReg, 1);
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

uint8_t max30102_getWritePointer(struct device *i2c_dev)
{
  uint8_t byteTemp;
  max30102_readReg(i2c_dev, MAX30102_FIFOWRITEPTR, &byteTemp, 1);
  printk("max30102_getWritePointer: %d\n", byteTemp);
  return byteTemp;
}

uint8_t max30102_getReadPointer(struct device *i2c_dev)
{
  uint8_t byteTemp;
  max30102_readReg(i2c_dev, MAX30102_FIFOREADPTR, &byteTemp, 1);
  printk("max30102_getReadPointer: %d\n", byteTemp);
  return byteTemp;
}

// get info
uint32_t max30102_getIR(struct device *i2c_dev)
{
  max30102_getNewData(i2c_dev);
  return (senseBuf.IR[senseBuf.head]);
}

void max30102_getNewData(struct device *i2c_dev)
{
  int32_t numberOfSamples = 0;
  uint8_t readPointer = 0;
  uint8_t writePointer = 0;
  while (1) {
    readPointer = max30102_getReadPointer(i2c_dev);
    writePointer = max30102_getWritePointer(i2c_dev);

    if (readPointer == writePointer) {
      printk("...no data...");
    } else {
      numberOfSamples = writePointer - readPointer;
      if (numberOfSamples < 0) numberOfSamples += 32;
      int32_t bytesNeedToRead = numberOfSamples * _activeLEDs * 3;
   
        while (bytesNeedToRead > 0) {
          senseBuf.head++;
          senseBuf.head %= MAX30102_SENSE_BUF_SIZE;
          uint32_t tempBuf = 0;
          if (_activeLEDs > 1) { 
            uint8_t temp[6];
            uint8_t tempex;

            max30102_readReg(i2c_dev, MAX30102_FIFODATA, temp, 6);

            for(uint8_t i = 0; i < 3; i++){
              tempex = temp[i];
              temp[i] = temp[5-i];
              temp[5-i] = tempex;
            }

            memcpy(&tempBuf, temp, 3*sizeof(temp[0]));
            tempBuf &= 0x3FFFF;
            senseBuf.IR[senseBuf.head] = tempBuf;
            memcpy(&tempBuf, temp+3, 3*sizeof(temp[0]));
            tempBuf &= 0x3FFFF;
            senseBuf.red[senseBuf.head] = tempBuf;
          } else { 
            uint8_t temp[3];
            uint8_t tempex;

       
            max30102_readReg(i2c_dev, MAX30102_FIFODATA, temp, 3);
            tempex = temp[0];
            temp[0] = temp[2];
            temp[2] = tempex;

            memcpy(&tempBuf, temp, 3*sizeof(temp[0]));
            tempBuf &= 0x3FFFF;
            senseBuf.red[senseBuf.head] = tempBuf;
          }
          bytesNeedToRead -= _activeLEDs * 3;
        }
      return;
    }
    k_sleep(K_MSEC(1));
  }
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

