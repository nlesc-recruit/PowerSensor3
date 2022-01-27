/*ADD HEADER*/

/* Define to prevent recursive inclusion */
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes */
#include <stm32f4xx.h>
#include <stm32f4xx_hal_flash.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions */
uint16_t FLASH_EraseSector(uint32_t Sector, uint8_t VoltageRange);
uint16_t FLASH_ProgramHalfWord(uint16_t Address, uint16_t Data);


#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */
