/*
 * eeprom.c
 *
 *  Created on: 13 déc. 2020
 *      Author: Patrick
 */

#include "eeprom.h"
#include <string.h>

static uint32_t const page_nb = 63U; //(uint32_t)FLASH_PAGE_NB-1;
static uint32_t const start_address = 0x08000000UL+63*0x800U; //(FLASH_BASE)+page_nb*(FLASH_PAGE_SIZE);

uint8_t const * eeprom_base_address()
{
	return (uint8_t*)start_address;
}

bool eeprom_empty()
{
	return *(uint32_t*)(start_address)==0xFFFFFFFF;
}

HAL_StatusTypeDef eeprom_restore(uint8_t * regs, uint32_t size)
{
	memcpy(regs,(uint8_t const*)start_address,size);
	return HAL_OK;
}

HAL_StatusTypeDef eeprom_store(uint8_t const * regs, uint32_t size)
{
	HAL_FLASH_Unlock();
	// erase the last page of bank1 (STM32G43x : 1 bank, 64 pages, 2kB per page, 64-bit data)
	{
		FLASH_EraseInitTypeDef erase =
			{
				FLASH_TYPEERASE_PAGES,
				FLASH_BANK_1,
				page_nb,
				1
			};
		uint32_t page_error = 0;
		HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&erase, &page_error);
		if(result!=HAL_OK)
		{
			HAL_FLASH_Lock();
			return result;
		}
	}
	// write the last page
	{
		uint64_t data = 0xDEADBEEF;
		for(uint32_t index=0; index<size;index+=sizeof(uint64_t)) // 64 bits
		{
			memcpy(&data,regs+index,sizeof(uint64_t));
			HAL_StatusTypeDef result = HAL_FLASH_Program(
					FLASH_TYPEPROGRAM_DOUBLEWORD,
					start_address+index,
					data
				);
			if(result!=HAL_OK)
			{
				uint32_t error = HAL_FLASH_GetError();
				if(error)
				{
					HAL_FLASH_Lock();
					return error;
				}
				HAL_FLASH_Lock();
				return result;
			}
		}
	}
	HAL_FLASH_Lock();
	return HAL_OK;
}
