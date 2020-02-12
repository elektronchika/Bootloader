// ----------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016 by MED-EL Elektromedizinische Geraete GmbH. All rights reserved!
// ----------------------------------------------------------------------------------------------------------------------

#ifndef SRECORD_H_
#define SRECORD_H_

#include <stdint.h>
#include "stm32f411xe.h"

//extern uint8_t const test_srec[48];

/* Application that flashed the blue LED */
extern uint8_t app_srec_blue[2032];
/* Application that flashed the red LED */
extern uint8_t app_srec_red[2032];

//void Process_Srec(uint8_t const *srec, uint32_t Address, uint8_t *Data);
void Process_Srec(uint8_t const *srec, uint8_t flash_data[16]);

uint8_t srec_process(uint8_t const *srec);

#endif /* SRECORD */
