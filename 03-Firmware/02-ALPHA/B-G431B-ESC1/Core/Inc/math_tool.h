/*
 * math_tool.h
 *
 *  Created on: 4 nov. 2020
 *      Author: Patrick, Kai
 */

#ifndef INC_MATH_TOOL_H_
#define INC_MATH_TOOL_H_

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#include<math.h>

int32_t constrain(int32_t x, int32_t min, int32_t max);
float fconstrain(float x, float min, float max);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

#define M_2PI (2.0f*M_PI)
#define M_3PI_2 (3.0f*M_PI_2)
#define RADIANS_TO_DEGREES(rad) ((rad)*180.0f/M_PI)
#define DEGREES_TO_RADIANS(deg) ((deg)*M_PI/180.0f)

// normalizing radian angle to [0,2PI]
float normalize_angle(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* INC_MATH_TOOL_H_ */
