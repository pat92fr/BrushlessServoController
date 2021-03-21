/*
 * math_tool.c
 *
 *  Created on: Nov 6, 2020
 *      Author: Patrick, Kai
 */

#include "math_tool.h"
#include<math.h>

int32_t constrain(int32_t x, int32_t min, int32_t max)
{
    if(x<min)
        return min;
    else if(x>max)
        return max;
    else
        return x;
}

float fconstrain(float x, float min, float max)
{
    if(x<min)
        return min;
    else if(x>max)
        return max;
    else
        return x;
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    if(x<=in_min)
        return out_min;
    else if(x>=in_max)
        return out_max;
    else
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    if(x<=in_min)
        return out_min;
    else if(x>=in_max)
        return out_max;
    else
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float normalize_angle(float angle_rad)
{
	float const a = fmodf(angle_rad, M_2PI);
	return a >= 0.0f ? a : (a + M_2PI);
}

