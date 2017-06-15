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
#include <stdio.h>

extern int 	SPIInit();
extern void     SPIDeinit();

extern void     SPISetClockDivider(uint32_t divider);

extern void 	SPIActivateSlave(uint32_t id);
extern void 	SPIDeactivateSlave(uint32_t id);

extern uint8_t 	SPIPutByte(uint32_t data);
extern uint8_t 	SPIGetByte();

extern uint32_t SPIGetMISO();


#endif /* SRC_AXISPI_H_ */
