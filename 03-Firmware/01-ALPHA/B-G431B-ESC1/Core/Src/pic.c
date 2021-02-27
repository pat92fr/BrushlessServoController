/*
 * pic.c
 *
 *  Created on: 16 nov. 2020
 *      Author: Patrick
 */

#include "pid.h"
#include <string.h>
#include <stdbool.h>
#include "math_tool.h"

void pid_reset( pid_context_t * ctx )
{
	ctx->err_last_one = 0.0f;
	ctx->err_integral = 0.0f;
	ctx->derivative_filtered = 0.0f;
}

float pid_process_antiwindup_clamp_with_ff(
		pid_context_t * ctx,
		float error,
		float kp,
		float ki,
		float kd,
		float output_limit,
		float alpha_derivative,
		float feed_forward
)
{
	// filter derivative
	ctx->derivative_filtered = alpha_derivative*(error-ctx->err_last_one)+(1.0f-alpha_derivative)*ctx->derivative_filtered;
	// update derivative state
	ctx->err_last_one = error;
	// PID
	float const p_term = kp*error;
	float const i_term = ctx->err_integral;
	float const d_term = kd*ctx->derivative_filtered;
	// compute output before saturation
	float const v = p_term + i_term + d_term + feed_forward;
	// saturation
	float const u = fconstrain(v,-output_limit,output_limit);
	// output saturating
	bool saturating = (u!=v);
	// error and output same sign
	bool sign = (error*v >= 0);
	// zero
	bool clamp = saturating && sign;
	if(!clamp)
		ctx->err_integral = ctx->err_integral + ki*error;
	// output
	return u;
}

float pid_process_antiwindup_clamp(
		pid_context_t * ctx,
		float error,
		float kp,
		float ki,
		float kd,
		float output_limit,
		float alpha_derivative
)
{
	// filter derivative
	ctx->derivative_filtered = alpha_derivative*(error-ctx->err_last_one)+(1.0f-alpha_derivative)*ctx->derivative_filtered;
	// update derivative state
	ctx->err_last_one = error;
	// PID
	float const p_term = kp*error;
	float const i_term = ctx->err_integral;
	float const d_term = kd*ctx->derivative_filtered;
	// compute output before saturation
	float const v = p_term + i_term + d_term;
	// saturation
	float const u = fconstrain(v,-output_limit,output_limit);
	// output saturating
	bool saturating = (u!=v);
	// error and output same sign
	bool sign = (error*v >= 0);
	// zero
	bool clamp = saturating && sign;
	if(!clamp)
		ctx->err_integral = ctx->err_integral + ki*error;
	// output
	return u;
}

float pid_process_antiwindup_back_calculation(
	pid_context_t * ctx,
	float error,
	float kp,
	float ki,
	float kd,
	float output_limit,
	float kt,
	float alpha_derivative
)
{
	// filter derivative
	ctx->derivative_filtered = alpha_derivative*(error-ctx->err_last_one)+(1.0f-alpha_derivative)*ctx->derivative_filtered;
	// update derivative state
	ctx->err_last_one = error;
	// PID
	float const p_term = kp*error;
	float const i_term = ctx->err_integral;
	float const d_term = kd*ctx->derivative_filtered;
	// compute output before saturation
	float const v = p_term + i_term + d_term;
	// saturation
	float const u = fconstrain(v,-output_limit,output_limit);
	// compute delta saturation
	float const e = u-v;
	// compute feedback
	float i_feedback = e*kt;
	// update integral state
	ctx->err_integral = ctx->err_integral + ki*error + i_feedback;
	// output
	return u;
}

