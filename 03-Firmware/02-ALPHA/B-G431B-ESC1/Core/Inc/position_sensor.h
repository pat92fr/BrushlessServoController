/*
 * position_sensor.h
 *
 *  Created on: Jan 15, 2021
 *      Author: Patrick
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

// Support AS5048A PWM interface only

void API_AS5048A_Position_Sensor_Init(TIM_HandleTypeDef * htim);
void API_AS5048A_Position_Sensor_It(TIM_HandleTypeDef *htim);
float API_AS5048A_Position_Sensor_Get_Radians();
float API_AS5048A_Position_Sensor_Get_Radians_Estimation(uint16_t time_us);
float API_AS5048A_Position_Sensor_Get_Multiturn_Radians();
float API_AS5048A_Position_Sensor_Get_RPS();
float API_AS5048A_Position_Sensor_Get_DPS();
uint16_t API_AS5048A_Position_Sensor_Get_Timestamp();
uint16_t API_AS5048A_Position_Sensor_Get_DeltaTimestamp();
uint32_t API_AS5048A_Position_Sensor_Error();
uint32_t API_AS5048A_Position_Sensor_Error_Counter();
float API_AS5048A_Position_Sensor_Get_DeltaRad();
uint32_t API_AS5048A_Position_Sensor_It_Counter();

#ifdef __cplusplus
}
#endif

#endif /* INC_POSITION_SENSOR_H_ */
