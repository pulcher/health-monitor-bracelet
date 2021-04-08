/*
Driver for MAX30102
Coded specifically for the nRF5340DK
*/

#ifndef _MAX30102_H
#define _MAX30102_H

// Base address for the breakout board
#define MAX30102_IIC_ADDRESS 0x57 //I2C Address

//Part ID Registers
#define MAX30102_REVISIONID 0xFE //Revision ID
#define MAX30102_PARTID 0xFF //Part ID:0x15
#define MAX30102_EXPECTED_PARTID 0x15



#endif