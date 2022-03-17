#include "flash_helper.h"

uint16_t FLASH_EraseSector(uint32_t Sector, uint8_t VoltageRange)
{
  FLASH_Erase_Sector(Sector, VoltageRange);
  return HAL_FLASH_ERROR_NONE;
}

uint16_t FLASH_ProgramHalfWord(uint16_t Address, uint16_t Data)
{
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t) Address, (uint16_t) Data) == HAL_OK)
  {
    return HAL_FLASH_ERROR_NONE;
  }
  else
  {
    return HAL_FLASH_ERROR_OPERATION;
  }
}
