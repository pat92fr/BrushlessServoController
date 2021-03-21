/*
 * protocol.h
 *
 *  Created on: 4 nov. 2020
 *      Author: Patrick
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void packet_encapsulate(uint32_t payload_status_length);
void instruction_handler();
void packet_handler(char c);



#ifdef __cplusplus
}
#endif

#endif /* INC_PROTOCOL_H_ */
