/*
Driver for MAX30102
Coded specifically for the nRF5340DK

Using stuff from the the Arduino C++ Library for the MAX30102
*/

#ifndef _MAX30102_H
#define _MAX30102_H

// Base address for the breakout board
#define MAX30102_IIC_ADDRESS 0x57 //I2C Address

//Configuraion Registers
#define MAX30102_FIFOCONFIG 0x08 //FIFO Configuration
#define MAX30102_MODECONFIG 0x09 //Mode Configuration

//Part ID Registers
#define MAX30102_REVISIONID 0xFE //Revision ID
#define MAX30102_PARTID 0xFF //Part ID:0x15
#define MAX30102_EXPECTED_PARTID 0x15

/*
Mode configuration(0x09) (pg 18)
* ------------------------------------------------------------------------------------------
* |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
* ------------------------------------------------------------------------------------------
* |   SHDN   |   RESET  |             NONE               |              MODE               |
* ------------------------------------------------------------------------------------------
*/
typedef struct {
uint8_t   ledMode:6; /*!< 010:Heart Rate mode, Red only. 011:SpO2 mode, Red and IR. 111:Multi-LED mode, Red and IR*/
uint8_t   reset:1; /*!< 1:reset */
uint8_t   shutDown:1; /*!< 0: wake up 1: put IC into low power mode*/
} __attribute__ ((packed)) sMode_t;

  /*
    FIFO Configuration(0x08) (pg 17)
    * ------------------------------------------------------------------------------------------
    * |    b7    |    b6    |    b5    |    b4          | b3 |    b2    |    b1     |    b0    |
    * ------------------------------------------------------------------------------------------
    * |            SMP_AVE             |FIFO_ROLLOVER_EN|               FIFO_A_FULL            |
    * ------------------------------------------------------------------------------------------
  */
  typedef struct {
    uint8_t   almostFull:4; // FIFO Almost Full Value
    uint8_t   RollOver:1;   // FIFO Rolls on Full
    uint8_t   sampleAverag:3;  // Sample Averaging
  } __attribute__ ((packed)) sFIFO_t;

// Class Level defines
//Configuration Options 
//FIFO Configuration(Register address 0x08)
//sampleAverage(Table 3. Sample Averaging)
#define SAMPLEAVG_1     0
#define SAMPLEAVG_2     1
#define SAMPLEAVG_4     2
#define SAMPLEAVG_8     3
#define SAMPLEAVG_16    4
#define SAMPLEAVG_32    5

//Mode configuration(Register address 0x09)
//ledMode(Table 4. Mode Control)
#define MODE_REDONLY    2
#define MODE_RED_IR     3
#define MODE_MULTILED   7

//Particle sensing configuration(Register address 0x0A)
//adcRange(Table 5. SpO2 ADC Range Control)
#define ADCRANGE_2048   0
#define ADCRANGE_4096   1
#define ADCRANGE_8192   2
#define ADCRANGE_16384  3
//sampleRate(Table 6. SpO2 Sample Rate Control)
#define SAMPLERATE_50   0 
#define SAMPLERATE_100  1
#define SAMPLERATE_200  2
#define SAMPLERATE_400  3
#define SAMPLERATE_800  4
#define SAMPLERATE_1000 5
#define SAMPLERATE_1600 6
#define SAMPLERATE_3200 7
//pulseWidth(Table 7. LED Pulse Width Control)
#define PULSEWIDTH_69   0 
#define PULSEWIDTH_118  1
#define PULSEWIDTH_215  2
#define PULSEWIDTH_411  3

//Multi-LED Mode Control Registers(Register address 0x011)
#define SLOT_NONE       0
#define SLOT_RED_LED    1
#define SLOT_IR_LED     2
#endif