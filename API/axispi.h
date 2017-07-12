/*
 * axispi.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Vedran Mikov
 */

#ifndef SRC_AXISPI_H_
#define SRC_AXISPI_H_

#include <stdbool.h>
#include <stdint.h>

#define BASE_ADDR	0x42000000

typedef struct
{
	uint32_t BaseAddress;
	uint32_t IsReady;
	int InterruptPresent;
} AXISPI;

extern AXISPI device_handle;

extern void 	SPIInit();
extern void     SPIDeinit();
extern void     SPIConfig(uint32_t div, uint8_t dataWidth);

extern void     SPIActivateSlave(uint32_t id);
extern void     SPIDeactivateSlave(uint32_t id);

extern uint32_t SPIPutData(uint32_t data);
extern uint32_t SPIGetData();

extern uint8_t  SPIPutByte(uint32_t data);
extern uint8_t  SPIGetByte();

extern bool	SPIBusy();


#endif /* SRC_AXISPI_H_ */
