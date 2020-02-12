// ----------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016 by MED-EL Elektromedizinische Geraete GmbH. All rights reserved!
// ----------------------------------------------------------------------------------------------------------------------

#include "flash.h"

void Flash_Init(void)
{
  
  /* Unlock flash if locked */
  
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }  
  
  /* Clear pending flags, if any */
  
  FLASH->SR &= ~(FLASH_SR_EOP + FLASH_SR_SOP + FLASH_SR_WRPERR + FLASH_SR_PGAERR + FLASH_SR_PGPERR + FLASH_SR_PGSERR);
  
}  

void Flash_Deinit(void)
{
  
  /* The FLASH_CR register can be locked again by software by setting the LOCK bit in the FLASH_CR register. */
  FLASH->CR |= FLASH_CR_LOCK;
  
}

void Flash_Write_x32(uint32_t const Address, uint32_t const Data)
{
  
  /* Check that no main Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register. */
  while((FLASH->SR & FLASH_SR_BSY) != RESET);
  
  /* Set program parallelism to x32 */
  FLASH->CR |= FLASH_CR_PSIZE_1;
  
  /* Set the PG bit in the FLASH_CR register. */
  FLASH->CR |= FLASH_CR_PG;
  
  /* Perform the data write operation(s) to the desired memory address. */
  *(__IO uint32_t*)Address = Data;
  
  /* Wait for the BSY bit to be cleared. */
  while((FLASH->SR & FLASH_SR_BSY) != RESET);
  
}

void Flash_Read(uint32_t const Address, uint16_t const * Data, uint8_t const Size);

/* Flash Erase */
void Flash_Erase_Sector(uint32_t const Sector)
{
  
  /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register */
  while((FLASH->SR & FLASH_SR_BSY) != RESET);
  
  /* Set the SER bit and select the sector out of the 7 sectors (STM32F411xC/E)
     in the main memory block you wish to erase (SNB) in the FLASH_CR register */
  if((Sector <= 7) & (Sector >= 1))
  {
    FLASH->CR |= ((Sector << 3) + FLASH_CR_SER);
  }
  else
  {
    /* ERROR */
  }
  
  /* Set the STRT bit in the FLASH_CR register */
  FLASH->CR |= FLASH_CR_STRT;
  
  /* Wait for the BSY bit to be cleared */
  while((FLASH->SR & FLASH_SR_BSY) != RESET);
  
}

void Flash_RegisterWrite(uint32_t const Address, uint32_t const Value);
uint32_t Flash_RegisterRead(uint32_t const Address);
//void Flash_CallbackRegister(FlashCallback_t const Function, TYPE (*CallbackFunction)(type));
