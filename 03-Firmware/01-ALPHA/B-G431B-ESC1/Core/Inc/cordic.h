/*
 * mycordic.h
 *
 *  Created on: Jan 14, 2021
 *      Author: Patrick
 */

#ifndef INC_CORDIC_H_
#define INC_CORDIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <math.h>

#define TWO_TO_POW_31 2147483648.0f
#define Q31_TO_FLOAT(x) ( (float)(x)/(TWO_TO_POW_31))
#define FLOAT_TO_Q31(x) ( (int)( (float)(x)*(float)0x7FFFFFFF ) )

float WRAP_TO_PI(float angle_radians)
{
	if (angle_radians>=0)
        return fmodf(angle_radians+M_PI, 2.0f*M_PI)-M_PI;
    else
    	return fmodf(angle_radians-M_PI, 2.0f*M_PI)+M_PI;
}

int32_t FLOAT_RADIANS_TO_Q31(float angle_radians)  // Q31 have a scaled input with the range [-1 1] mapping to [-pi pi).
{
	return FLOAT_TO_Q31(WRAP_TO_PI(angle_radians)/M_PI);
}

extern CORDIC_HandleTypeDef hcordic;

HAL_StatusTypeDef API_CORDIC_Processor_Init()
{
	CORDIC_ConfigTypeDef config = {
			CORDIC_FUNCTION_COSINE, // ouput : cosine, then sine
			CORDIC_SCALE_0, // not used
			CORDIC_INSIZE_32BITS, // q31
			CORDIC_OUTSIZE_32BITS, // q31
			CORDIC_NBWRITE_1, // ARG2 is 1 default
			CORDIC_NBREAD_2, // read cosine and sine
			CORDIC_PRECISION_6CYCLES // better than 10-3
	};
	return HAL_CORDIC_Configure(&hcordic, &config);
}

HAL_StatusTypeDef API_CORDIC_Processor_Update(float theta, float * c, float * s)
{
	static int32_t InBuff[1] = {0};
	static int32_t OutBuff[2] = {0,0};
	InBuff[0] = FLOAT_RADIANS_TO_Q31(theta);
	HAL_StatusTypeDef result = HAL_CORDIC_Calculate(&hcordic,InBuff,OutBuff,1,10);
	if(HAL_OK==result)
	{
		if(c!=0)
			*c = Q31_TO_FLOAT(OutBuff[0]);
		if(s!=0)
			*s = Q31_TO_FLOAT(OutBuff[1]);
	}
	return result;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_CORDIC_H_ */
