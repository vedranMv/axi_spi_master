/*
 * axispi.c
 *
 *  Created on: Apr 19, 2017
 *      Author: v125
 */
#include <stdlib.h>
#include "axispi.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

//	Physical address of AXI peripheral
#define BASE_ADDR	0x42000000
//	R/W register containing data to be sent to slave
#define REG_MOSIDATA	(BASE_ADDR+0)
//	Read-only register containing data received from slave
#define REG_MISODATA	(BASE_ADDR+4)
//	R/W register containing clock divider
#define REG_SPICLKDIV	(BASE_ADDR+8)
//	R/W register used to set a slave to which master communicates
#define REG_SLAVESEL	(BASE_ADDR+12)


#define MAP_SIZE 32
#define MAP_MASK (MAP_SIZE-1)
off_t target = BASE_ADDR;

//	Parameters for R/W map
int fd1;
//	Array mapped to physical memory location of AXI peripheral
volatile uint32_t *regMap;

/******************************************************************************
 *******    Register manipulation    ******************************************
 *****************************************************************************/

/**
 *    Read AXI peripheral register
 *    @param regAddr address of register to access
 *    @return value stored in requested register
 */
uint32_t Reg_Read(uint32_t regAddr)
{
    return regMap[(regAddr - BASE_ADDR)/sizeof(uint32_t)];
}

/**
 *    Write to AXI peripheral register
 *    @param regAddr address of register to access
 *    @param regVal new value to write to register
 *    @return value stored in requested register
 */
uint32_t Reg_Write(uint32_t regAddr, uint32_t regVal)
{
    regMap[(regAddr - BASE_ADDR)/sizeof(uint32_t)] = regVal;

    return regMap[(regAddr - BASE_ADDR)/sizeof(uint32_t)];
}

/******************************************************************************
 *******    SPI master API    *************************************************
 *****************************************************************************/

/**
 *    Initialize SPI peripheral
 *    Map physical memory belonging to AXI module to virtual address space of 
 *    this program. Registers accessible through regMap[0/1/2/3] variable
 */
int SPIInit()
{
    if((fd1=open("/dev/mem",O_RDWR))==0) 
    {
        perror("Error openning file /dev/mem");
        return -1;
    }

    regMap = (volatile uint32_t*)mmap(0,MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd1,target&~MAP_MASK);

    if(regMap == (void *)-1) 
    {
        perror("Error mapping");
        return -2;
    }
}

/**
 *    Deinitialize
 */
void SPIDeinit()
{
    close(fd1);
}

/**
 *    Set clock divider for for SPI clock
 *    @param divider clock divider by which AXI bus clock is divided when
 *    generating SPI clock (500 produces CLK of 100kHz)
 */
void SPISetClockDivider(uint32_t divider)
{
    Reg_Write(REG_SPICLKDIV, divider);
}

/**
 *    Initiate communication with a slave by bringing its SS line low
 *    @param id number between 1 and 3 (inclusive) of slave line to bring low
 */
void SPIActivateSlave(uint32_t id)
{
    Reg_Write(REG_SLAVESEL, id & 0x00000003);
}

/**
 *    Terminate communication with a slave by bringing its SS line high
 *    @param id number between 1 and 3 (inclusive) of slave line to bring high
 */
void SPIDeactivateSlave(uint32_t id)
{
    Reg_Write(REG_SLAVESEL, (id & 0x00000003) | 0x00000004);
}

/**
 *    Send one byte through MOSI line
 *    @note Sending is initialized as soon as data is written into te register,
 *    make sure SPIActivateSlave() has been called previously
 *    @param data 8bit value to be pushed through MOSI line
 *    @return 8-bit value received on MISO line while sending
 */
uint8_t  SPIPutByte(uint32_t data)
{
    uint16_t i;

    //  All data being sent has to be ORed with 0xABCD0000 to make start 
    //  transmission
    Reg_Write(REG_MOSIDATA, 0xABCD0000 | data);

    //  Short delay, wait for end of transmission
    for (i = 0; i < 8000; i++);	

    return SPIGetByte();
}

/**
 *    Get last received byte on SPI bus
 *    Reads registers and returns last received byte stored in there
 */
uint8_t SPIGetByte()
{
	uint8_t retVal;

	retVal = (uint8_t)(Reg_Read(REG_MISODATA) & 0xFF);
	return retVal;
}

/**
 *    Reads and returns value of whole MISO data register, not only byte of data
 */
uint32_t SPIGetMISO()
{
	uint32_t retVal;

	retVal = Reg_Read(REG_MISODATA);
	return retVal;
}

