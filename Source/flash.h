// ----------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016 by MED-EL Elektromedizinische Geraete GmbH. All rights reserved!
// ----------------------------------------------------------------------------------------------------------------------

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>
#include "stm32f411xe.h"

#define RESET              0
#define FLASH_KEY1         ((uint32_t)0x45670123) /* Flash key 1 */
#define FLASH_KEY2         ((uint32_t)0xCDEF89AB) /* Flash key 2 */

#define FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
#define FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */
#define FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbyte */
#define FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbyte */

void Flash_Init(void);
void Flash_Deinit(void);
void Flash_Write_x32(uint32_t const Address, uint32_t const Data);
void Flash_Read(uint32_t const Address, uint16_t const * Data, uint8_t const Size);
void Flash_Erase_Sector(uint32_t const Sector);
void Flash_RegisterWrite(uint32_t const Address, uint32_t const Value);
uint32_t Flash_RegisterRead(uint32_t const Address);
//void Flash_CallbackRegister(FlashCallback_t const Function, TYPE (*CallbackFunction)(type));

#endif /*FLASH_H_*/
