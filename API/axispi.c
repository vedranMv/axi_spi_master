/*
 * axispi.c
 *
 *  Created on: Apr 19, 2017
 *      Author: v125
 */
#include "axispi.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#define MAP_SIZE 32
#define MAP_MASK (MAP_SIZE-1)
off_t target = BASE_ADDR;

//	Parameters for R/W map
int fd1;
volatile uint32_t *map_base;


//	R/W register containing data to be sent to slave
#define REG_MOSIDATA	(device_handle.BaseAddress+0)
//	Read-only register containing data received from slave
#define REG_MISODATA	(device_handle.BaseAddress+4)
//	R/W register containing clock divider
#define REG_SPICLKDIV	(device_handle.BaseAddress+8)
//	R/W register used configure SPI peripheral
#define REG_CONFIG	(device_handle.BaseAddress+12)

AXISPI device_handle;

/******************************************************************************
 *******    Register manipulation    ******************************************
 *****************************************************************************/

/**
 *    Read AXI peripheral register
 *    @param regAddr Address of register to access
 *    @return value Stored in requested register
 */
uint32_t Xil_In32(uint32_t regAddr)
{
    return map_base[(regAddr - BASE_ADDR)/4];
}

/**
 *    Write to AXI peripheral register
 *    @param regAddr Address of register to access
 *    @param regVal New value to write to register
 *    @return Value stored in requested register
 */
uint32_t Xil_Out32(uint32_t regAddr, uint32_t regVal)
{
    map_base[(regAddr - BASE_ADDR)/4] = regVal;
    return map_base[(regAddr - BASE_ADDR)/4];
}

/******************************************************************************
 *******    SPI master API    *************************************************
 *****************************************************************************/

/**
 *    Initialize SPI peripheral
 *    Map physical memory belonging to AXI module to virtual address space of 
 *    this program. Registers accessible through regMap[0/1/2/3] variable
 */
void SPIInit()
{
    device_handle.BaseAddress = BASE_ADDR;

    if((fd1=open("/dev/mem",O_RDWR))==0) 
    {
        perror("Error openning file /dev/mem");
        return;
    }

    map_base = (volatile uint32_t*)mmap(0,MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd1,target&~MAP_MASK);

    if(map_base == (void *)-1) 
    {
        perror("Error mapping");
        return;
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
 *    Configure SPI peripheral
 *    Function sets clock divider (base clock: 50MHz) and data width for
 *    SPI communication (word size in bits)
 */
void SPIConfig(uint32_t div, uint8_t dataWidth)
{
    //  Sanity check on data width
    if (dataWidth < 5)
        dataWidth = 5;
    else if (dataWidth > 32)
        dataWidth = 32;

    //  Apply range on clock divider
    if (div < 120)
        div = 120;
    else if (div > 11000)
        div = 11000;

    device_handle.BaseAddress = BASE_ADDR;
    Xil_Out32(REG_SPICLKDIV, div);

    //  Write data width to config register
    //  Read register and clear bus width-related bits
    uint32_t reg = Xil_In32(REG_CONFIG) & (~(0x0000003F<<4));
    //  OR new bus width into the register and write it to memory
    Xil_Out32(REG_CONFIG, reg | ((dataWidth & 0x3F)<<4));

}

/**
 *    Initiate communication with a slave by bringing its SS line low
 *    @param id number between 1 and 3 (inclusive) of slave line to bring low
 */
void SPIActivateSlave(uint32_t id)
{
    //  Read register and clear slave-related bits
    uint32_t reg = Xil_In32(REG_CONFIG) & (~0x0000000F);
    //  OR new slave data into the register and write it in memory
    Xil_Out32(REG_CONFIG, reg | (id & 0x00000003));
}

/**
 *    Terminate communication with a slave by bringing its SS line high
 *    @param id number between 1 and 3 (inclusive) of slave line to bring high
 */
void SPIDeactivateSlave(uint32_t id)
{
    //  Read register and clear slave-related bits
    uint32_t reg = Xil_In32(REG_CONFIG) & (~0x0000000F);
    //  OR new slave data into the register and write it in memory
    Xil_Out32(REG_CONFIG, reg | (id & 0x00000003) | 0x00000004);
}

/**
 *    Send data of size up to 32-bit over SPI
 *    When configured to use data width different than 8bit this function
 *    must be used to send data through SPI
 *    @note Sending is initialized as soon as data is written into te register,
 *    make sure SPIActivateSlave() has been called previously
 *    @param data Data to be sent through SPI, size depends on data width
 *    set through SPIConfig() function
 *    @return Data received on MISO while sending
 */
uint32_t  SPIPutData(uint32_t data)
{
    Xil_Out32(REG_MOSIDATA, data);
    while (SPIBusy());    //  Wait for end of transmission
    
    return SPIGetData();
}

/**
 *    Read data received during last SPI transmission
 *    When configured to use data width different than 8bit this function
 *    must be used to read received data
 *    @note This function just reads internal register, DOESN'T initiate new
 *    sending task
 *    @return Data received on MISO during last transaction
 */
uint32_t SPIGetData()
{
    uint32_t retVal;

    retVal = Xil_In32(REG_MISODATA);
    return retVal;
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
    uint32_t retVal = SPIPutData(data);

    return (uint8_t)(retVal & 0xFF);
}

/**
 *    Get last received byte on SPI bus
 *    Reads registers and returns last received byte stored in there
 *    @note This function just reads internal register, DOESN'T initiate new
 *    sending task
 *    @return Last received byte
 */
uint8_t  SPIGetByte()
{
    uint32_t retVal = SPIGetData();

    return (uint8_t)(retVal & 0xFF);
}

/**
 *    Returns status of SPI communication
 *    @return true if SPI transaction is in process, false if SPI is idle
 */
bool SPIBusy()
{
    //  Last bit (31) is held high by PL during transmission
    uint32_t reg = Xil_In32(REG_CONFIG);

    return reg & 0x80000000;
}
