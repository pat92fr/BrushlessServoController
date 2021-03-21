/*
 * binary_tool.h
 *
 *  Created on: 27 sept. 2020
 *      Author: patrick
 */

#ifndef INC_BINARY_TOOL_H_
#define INC_BINARY_TOOL_H_

#define LOW_BYTE(x)     ((unsigned char)((x)&0xFF))
#define HIGH_BYTE(x)    ((unsigned char)(((x)>>8)&0xFF))
#define MAKE_SHORT(l,h)    (((uint16_t)((h)&0xFF)<<8) | (uint16_t)((l)&0xFF))

#endif /* INC_BINARY_TOOL_H_ */
