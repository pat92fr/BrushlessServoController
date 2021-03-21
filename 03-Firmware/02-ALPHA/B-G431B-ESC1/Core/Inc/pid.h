/*
 * pid.h
 *
 *  Created on: 16 nov. 2020
 *      Author: Patrick
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PID_INTEGRAL_DEPTH 100

typedef struct{
	float err_last_one;
	float err_integral;
	float derivative_filtered;

} pid_context_t;

void pid_reset( pid_context_t * ctx );

float pid_process_antiwindup_clamp_with_ff(
		pid_context_t * ctx,
		float error,
		float kp,
		float ki,
		float kd,
		float output_limit,
		float alpha_derivative,
		float feed_forward
);


float pid_process_antiwindup_clamp(
		pid_context_t * ctx,
		float error,
		float kp,
		float ki,
		float kd,
		float output_limit,
		float alpha_derivative
);

float pid_process_antiwindup_back_calculation(
		pid_context_t * ctx,
		float error,
		float kp,
		float ki,
		float kd,
		float output_limit,
		float kt,
		float alpha_derivative
);


#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
