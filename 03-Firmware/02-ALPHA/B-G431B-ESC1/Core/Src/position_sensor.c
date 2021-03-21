/*
 * position_sensor.c
 *
 *  Created on: Jan 15, 2021
 *      Author: Patrick
 */

#include "position_sensor.h"
#include <math.h>
#include"math_tool.h"
#include "control_table.h"

// µs TIMER
extern TIM_HandleTypeDef htim6;
// PWM IC TIMER
static TIM_HandleTypeDef * position_sensor_htim = 0;
static uint32_t calls = 0;
// position state
static uint32_t position_sensor_error = 0;
static uint32_t position_sensor_error_counter = 0;
static uint16_t present_time_us = 0;
static float present_position_rad = 0.0f;
static float delta_position_rad = 0.0f;
static float const bit_to_radians_ratio = M_2PI/4096.0f;
static float const max_radians = M_2PI/4096.0f*4095.0f;
// velocity state
static uint16_t position_delta_time_us = 0;
static uint16_t last_position_time_us = 0;
static float last_position_rad = 0.0f;
static float present_velocity_rad = 0.0f;
// multi-turn position state
static int32_t present_revolution = 0;
static float present_position_multi_rad = 0.0f;

#define ALPHA_VELOCITY 0.01f // 0.1f default

void API_AS5048A_Position_Sensor_Init(TIM_HandleTypeDef * htim)
{
	position_sensor_htim = htim;
	HAL_TIM_IC_Start_IT(position_sensor_htim,TIM_CHANNEL_1); // Trigger It only when period is over.
	HAL_TIM_IC_Start(position_sensor_htim,TIM_CHANNEL_2); // CHANNEL 1 is PWM width and CHANNEL 2 is period
	HAL_Delay(3);
	present_revolution = 0;
	present_velocity_rad = 0.0f;
}

void API_AS5048A_Position_Sensor_It(TIM_HandleTypeDef *htim)
{
	if(htim==position_sensor_htim)
	{
		++calls;
		// timestamp as soon as possible
		present_time_us = __HAL_TIM_GET_COUNTER(&htim6);
		// capture the number of bits 1b
		// period is 4119 bits long with 8-bit trailer (value = 00000000b)
		// header is 16-bit long (12-bit init field (value=111111111111b), 4 for error field (value=1111b when OK))
		// when position is 0°, length is 16 bits
		// when position is MAX = 2*PI*(1-1/4096)°, length is 16+4095 bits
		// compute PWM width / PWM period * 4119bits that gives the number of 1 bits
		// @150MHz, CHANNEL1 = period = 45500 with PSC=3
		float const init_error_data_bits = 4119.0f*(float)__HAL_TIM_GET_COMPARE(position_sensor_htim,TIM_CHANNEL_2)/(float)__HAL_TIM_GET_COMPARE(position_sensor_htim,TIM_CHANNEL_1);
		// if data < 0 bits ==> must be an error
		if(init_error_data_bits<(16.0f-0.8f)) // add a 0.8 margin due to IC TIMER PRECISION and PWM precision
		{
			// set error
			position_sensor_error = 1;
			++position_sensor_error_counter;
			// Note : use the state when error (present time / position / velocity)
			// set encoder error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR;
		}
		else
		{
			// clear encoder error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR);
			// reset error
			position_sensor_error = 0;
			// compute new position in radians and constrain it to [0..2pi[
			present_position_rad = roundf(init_error_data_bits-16.0f)*bit_to_radians_ratio;
			if(present_position_rad<0.0f)
				present_position_rad=0.0f;
			if(present_position_rad>max_radians)
				present_position_rad=max_radians;
			// compute multi-turn position and velocity in radians
			delta_position_rad = present_position_rad-last_position_rad;
			if(delta_position_rad>M_PI)
			{
				--present_revolution;
				delta_position_rad-=M_2PI;
			}
			if(delta_position_rad<-M_PI)
			{
				++present_revolution;
				delta_position_rad+=M_2PI;
			}
			present_position_multi_rad = present_position_rad+(float)present_revolution*M_2PI;
			// compute velocity
			position_delta_time_us = present_time_us-last_position_time_us;
			float const alpha_vel = (float)(regs[REG_EWMA_ENCODER]+1)/2560.0f; // 255 => B=0.1, 1 => beta = 0.0004
			present_velocity_rad =
					alpha_vel * (delta_position_rad / (float)position_delta_time_us * 1000000.0f)
					+ (1.0f-alpha_vel) * present_velocity_rad;
			// save last position
			last_position_time_us = present_time_us;
			last_position_rad = present_position_rad;
		}
	}
}

float API_AS5048A_Position_Sensor_Get_Radians_Estimation(uint16_t time_us)
{
	uint16_t delta_t_us = time_us-present_time_us;
	// check old sample error
	if(delta_t_us>2000) //2ms
	{
		// set encoder error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_NOS_RESPONDING;
	}
	else
	{
		// clear encoder error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_POSITION_SENSOR_NOS_RESPONDING);
	}
	return present_position_rad + present_velocity_rad*(float)(delta_t_us)/1000000.0f;
}

float API_AS5048A_Position_Sensor_Get_Radians()
{
	return present_position_rad;
}

float API_AS5048A_Position_Sensor_Get_Multiturn_Radians()
{
	return present_position_multi_rad;
}

float API_AS5048A_Position_Sensor_Get_RPS()
{
	return present_velocity_rad;
}

float API_AS5048A_Position_Sensor_Get_DPS()
{
	return RADIANS_TO_DEGREES(present_velocity_rad);
}

uint16_t API_AS5048A_Position_Sensor_Get_Timestamp()
{
	return present_time_us;
}

uint16_t API_AS5048A_Position_Sensor_Get_DeltaTimestamp()
{
	return position_delta_time_us;
}

uint32_t API_AS5048A_Position_Sensor_Error()
{
	return position_sensor_error;
}

uint32_t API_AS5048A_Position_Sensor_Error_Counter()
{
	return position_sensor_error_counter;
}

float API_AS5048A_Position_Sensor_Get_DeltaRad()
{
	return delta_position_rad;
}

uint32_t API_AS5048A_Position_Sensor_It_Counter()
{
	return calls;
}
