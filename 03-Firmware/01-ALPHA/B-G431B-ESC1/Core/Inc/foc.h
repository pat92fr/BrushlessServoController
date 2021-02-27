/*
 * foc.h
 *
 *  Created on: 15 janv. 2021
 *      Author: Patrick
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void API_FOC_Init();

void API_FOC_Update_Torque_Closed_Loop(
		uint16_t present_time_us,
		float setpoint_torque_current_mA,
		float setpoint_flux_current_mA,
		float phase_synchro_offset_rad,
		uint32_t closed_loop
);

void API_FOC_Update_Torque_Open_Loop(
		uint16_t present_time_us,
		float setpoint_velocity_dps,
		float setpoint_flux_voltage_100
);

float API_FOC_Get_Present_Position();
float API_FOC_Get_Present_Position_Multi();
float API_FOC_Get_Present_Velocity();
float API_FOC_Get_Present_Torque_Current();
float API_FOC_Get_Present_Flux_Current();
float API_FOC_Get_Present_Current_SQ();

void API_FOC_It(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif


#endif /* INC_FOC_H_ */
