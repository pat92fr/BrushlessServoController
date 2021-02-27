/*
 * protocol.c
 *
 *  Created on: 4 nov. 2020
 *      Author: Patrick
 */

#include "protocol.h"
#include "control_table.h"
#include "binary_tool.h"
#include "serial.h"

extern HAL_Serial_Handler serial;

uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  static const uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

uint8_t rx_packet_buffer[1100];
uint32_t rx_packet_position = 0;
uint32_t rx_packet_payload_bytes = 0; // length without crc
uint8_t tx_packet_buffer[1100];
uint32_t tx_packet_length = 0;

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8
#define PKT_PARAMETER1          9
#define PKT_PARAMETER2          10
#define PKT_PARAMETER3          11
#define PKT_PARAMETER4          12

void packet_encapsulate(uint32_t payload_status_length) // length from instruction byte to last parameter byte
{
	// header
	tx_packet_buffer[PKT_HEADER0]=0xFF;
	tx_packet_buffer[PKT_HEADER1]=0xFF;
	tx_packet_buffer[PKT_HEADER2]=0xFD;
	tx_packet_buffer[PKT_RESERVED]=0x00;
	tx_packet_buffer[PKT_ID]=regs[REG_ID];
	// length
	uint16_t length = payload_status_length+2; // +crc
	tx_packet_buffer[PKT_LENGTH_L]= LOW_BYTE(length);
	tx_packet_buffer[PKT_LENGTH_H]= HIGH_BYTE(length);
	// crc
	uint16_t packet_crc = updateCRC(0, tx_packet_buffer, length+5);
	tx_packet_buffer[PKT_INSTRUCTION+payload_status_length]= LOW_BYTE(packet_crc); // CRC 1
	tx_packet_buffer[PKT_INSTRUCTION+payload_status_length+1]= HIGH_BYTE(packet_crc); // CRC 2
	tx_packet_length = PKT_INSTRUCTION+payload_status_length+2;
}

#define INSTR_PING        	0x01
#define INSTR_READ			0x02
#define INSTR_WRITE       	0x03
#define INSTR_FACTORY_RESET	0x06
#define INSTR_REBOOT       	0x08
#define INSTR_STATUS       	0x55
#define INSTR_SYNC_READ     0x82
#define INSTR_SYNC_WRITE    0x83

#define ERROR_NONE					0x00
#define ERROR_RESULT_FAIL			0x01
#define ERROR_INSTRUCTION_ERROR		0x02
#define ERROR_CRC_ERROR			  	0x03
#define ERROR_DATA_RANGE_ERROR  	0x04
#define ERROR_DATA_LENGTH_ERROR  	0x05
#define ERROR_DATA_LIMIT_ERROR  	0x06
#define ERROR_ACCESS_ERROR  		0x07

void instruction_handler()
{
	switch(rx_packet_buffer[PKT_INSTRUCTION])
	{
	case INSTR_PING:
		{
			// reply with a status packet with Model Number LSB MSB and Firmware version
			tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
			tx_packet_buffer[PKT_ERROR]= ERROR_NONE;
			tx_packet_buffer[PKT_PARAMETER1]= regs[REG_MODEL_NUMBER_L];
			tx_packet_buffer[PKT_PARAMETER2]= regs[REG_MODEL_NUMBER_H];
			tx_packet_buffer[PKT_PARAMETER3]= regs[REG_VERSION];
			packet_encapsulate(5);
			// send packet
			HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
		}
		break;
	case INSTR_READ:
		{
			// resassemble 16-bit address and length
			uint16_t address = MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER0],rx_packet_buffer[PKT_PARAMETER1]);
			uint16_t length =  MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER2],rx_packet_buffer[PKT_PARAMETER3]);
			// capture value
			if(address<REG_MAX)
			{
				// reply with a status packet with ERR only
				tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
				tx_packet_buffer[PKT_ERROR]= 0x00; // TODO : Alarm flag to handle here
				for(uint32_t index=0;index<length;++index)
					tx_packet_buffer[PKT_ERROR+1+index]= regs[address+index];
				packet_encapsulate(2+length);
			}
			else
			{
				// reply with a status packet with ERR only
				tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
				tx_packet_buffer[PKT_ERROR]= ERROR_ACCESS_ERROR; // TODO : Alarm flag to handle here
				packet_encapsulate(2);
			}
			// send packet
			HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
		}
		break;
	case INSTR_WRITE:
		{
			// resassemble 16-bit address and value
			uint16_t address = MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER0],rx_packet_buffer[PKT_PARAMETER1]);
			// TODO ; depend on length of packet
			uint16_t value_length =  MAKE_SHORT(rx_packet_buffer[PKT_LENGTH_L],rx_packet_buffer[PKT_LENGTH_H])-2-1-2; // remove INSTR, CRC-16 and PARAM0/1
			// write into register
			if(address<REG_MAX)
			{
				// TODO : check data range for global position, and other registers in RAM
				for(uint32_t index=0;index<value_length;++index)
					regs[address+index]=rx_packet_buffer[PKT_PARAMETER2+index];
				// store when accessing EEPROM regs
				if(address<REG_TORQUE_ENABLE)
				{
					store_eeprom_regs();
				}
				// reply with a status packet with ERR only
				tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
				tx_packet_buffer[PKT_ERROR]= 0x00; // TODO : Alarm flag to handle here
				packet_encapsulate(2);
			}
			else
			{
				// reply with a status packet with ERR only
				tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
				tx_packet_buffer[PKT_ERROR]= ERROR_ACCESS_ERROR; // TODO : Alarm flag to handle here
				packet_encapsulate(2);
			}
			// send packet
			HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
		}
		break;
	case INSTR_SYNC_READ:
		{

		}
		break;
	case INSTR_SYNC_WRITE:
		{
			// decode length of [ID,PARAMS]
			uint16_t const values_length =  MAKE_SHORT(rx_packet_buffer[PKT_LENGTH_L],rx_packet_buffer[PKT_LENGTH_H])-1-4-2; // remove INSTR, CRC-16 and PARAM0/1/2/3
			// decode the address
			uint16_t address = MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER0],rx_packet_buffer[PKT_PARAMETER1]);
			// decode the size of data per ID
			uint32_t const data_length = MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER2],rx_packet_buffer[PKT_PARAMETER3]);
			// count the [ID,PARAMS]
			uint16_t const number_of_id_and_data = values_length / (1+data_length); // 1 for ID + data_length
			// search for my ID
			uint32_t const my_id = regs[REG_ID];
			for(uint32_t index=0;index<number_of_id_and_data;++index)
			{
				uint32_t position = PKT_PARAMETER4+index*(data_length+1);
				uint32_t id = rx_packet_buffer[position];
				// found my own id in one ID,PARAMS]
				if(id==my_id)
				{
					// update RAM
					for(uint32_t index2=0;index2<data_length;++index2)
						regs[address+index2]=rx_packet_buffer[position+1+index2];
				}
			}
		}
		break;
	case INSTR_FACTORY_RESET:
		{
			factory_reset_eeprom_regs();
			// reply with a status packet with Model Number LSB MSB and Firmware version
			tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
			tx_packet_buffer[PKT_ERROR]= ERROR_NONE;
			packet_encapsulate(2);
			// send packet
			HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
			// reboot
			HAL_Delay(100);
			HAL_NVIC_SystemReset();
		}
		break;
	case INSTR_REBOOT:
		{
			// reply with a status packet with Model Number LSB MSB and Firmware version
			tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
			tx_packet_buffer[PKT_ERROR]= ERROR_NONE;
			packet_encapsulate(2);
			// send packet
			HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
			// reboot
			HAL_Delay(100);
			HAL_NVIC_SystemReset();
		}
		break;
	default:
		// reply with a status packet with ERR only
		tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
		tx_packet_buffer[PKT_ERROR]= ERROR_INSTRUCTION_ERROR; // TODO : Alarm flag to handle here
		packet_encapsulate(2);
		// send packet
		HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
		break;
		// nope
	}
}

typedef enum
{
	HEADER1,HEADER2,HEADER3,RESERVED,ID,LENGTH1,LENGTH2,PAYLOAD,CRC1,CRC2
} e_packet_state;
e_packet_state packet_state = HEADER1;

void packet_handler(char c)
{
	switch(packet_state)
	{
	case HEADER1:
		if(c==0xFF)
		{
			rx_packet_buffer[PKT_HEADER0] = (uint8_t)c;
			packet_state = HEADER2;
		}
		break;
	case HEADER2:
		if(c==0xFF)
		{
			rx_packet_buffer[PKT_HEADER1] = (uint8_t)c;
			packet_state = HEADER3;
		}
		else if(c==0xFD)
		{
			rx_packet_buffer[PKT_HEADER2] = (uint8_t)c;
			packet_state = RESERVED;
		}
		else
			packet_state = HEADER1;
		break;
	case HEADER3:
		if(c==0xFD)
		{
			rx_packet_buffer[PKT_HEADER2] = (uint8_t)c;
			packet_state = RESERVED;
		}
		else
			packet_state = HEADER1;
		break;
	case RESERVED:
		if(c==0x00)
		{
			rx_packet_buffer[PKT_RESERVED] = (uint8_t)c;
			packet_state = ID;
		}
		else
			packet_state = HEADER1;
		break;
	case ID:
		if( (c<=252) || (c==254) ) // validate ID
		{
			rx_packet_buffer[PKT_ID] = (uint8_t)c;
			packet_state = LENGTH1;
		}
		else
			packet_state = HEADER1;
		break;
	case LENGTH1:
		rx_packet_buffer[PKT_LENGTH_L] = (uint8_t)c;
		packet_state = LENGTH2;
		break;
	case LENGTH2:
		rx_packet_buffer[PKT_LENGTH_H] = (uint8_t)c;
		// reassemble length
		rx_packet_payload_bytes = (uint32_t)MAKE_SHORT(rx_packet_buffer[PKT_LENGTH_L],rx_packet_buffer[PKT_LENGTH_H])-2;
		if(rx_packet_payload_bytes<=1024) // validate length
		{
			packet_state = PAYLOAD;
			rx_packet_position = PKT_INSTRUCTION;
		}
		else
			packet_state = HEADER1;
		break;
	case PAYLOAD:
		rx_packet_buffer[rx_packet_position] = (uint8_t)c;
		++rx_packet_position;
		--rx_packet_payload_bytes;
		if(rx_packet_payload_bytes==0)
			packet_state = CRC1;
		break;
	case CRC1:
		rx_packet_buffer[rx_packet_position] = (uint8_t)c;
		++rx_packet_position;
		packet_state = CRC2;
		break;
	case CRC2:
		rx_packet_buffer[rx_packet_position] = (uint8_t)c;
		// check CRC
		uint16_t received_crc = (uint16_t)MAKE_SHORT(rx_packet_buffer[rx_packet_position-1],rx_packet_buffer[rx_packet_position]);
		uint16_t calculated_crc = updateCRC(0,rx_packet_buffer,rx_packet_position-1);
		if(calculated_crc==received_crc)
		{
			if(rx_packet_buffer[PKT_ID]==regs[REG_ID] || rx_packet_buffer[PKT_ID]==0xFE)
				instruction_handler();
		}
		else
		{
			regs[REG_PROTOCOL_CRC_FAIL] = regs[REG_PROTOCOL_CRC_FAIL] + 1;
			// reply with a status packet with ERR only
			tx_packet_buffer[PKT_INSTRUCTION]= INSTR_STATUS;
			tx_packet_buffer[PKT_ERROR]= ERROR_CRC_ERROR; // TODO : Alarm flag to handle here
			packet_encapsulate(2);
			// send packet
			HAL_Serial_Write(&serial, (uint8_t const *)tx_packet_buffer,tx_packet_length);
		}
		packet_state = HEADER1;
		break;
	}
}
