/*
 * foc.c
 *
 *  Created on: 16 janv. 2021
 *      Author: Patrick
 */

/// DOC FOC(open loop) : https://docs.simplefoc.com/foc_theory
/// DOC SVM https://www.embedded.com/painless-mcu-implementation-of-space-vector-modulation-for-electric-motor-systems/

#include "foc.h"
#include "cordic.h"
#include "serial.h"
#include "position_sensor.h"
#include "math_tool.h"
#include "pid.h"
#include "control_table.h"
#include "binary_tool.h"

#include <string.h>
#include <math.h>

// peripherals
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

// serial communication (UART2) for TRACEs
extern HAL_Serial_Handler serial;

// FOC period
static uint32_t current_sample_drop_rate = 3;
// 4:250us cycle
// 3:200us cycle
// 2:150us cycle
// 1:100us cycle
// 0:50us cycle (not possible, one iteration takes about ~50us)

// FOC private vars
static int32_t current_samples = 0;
volatile uint16_t ADC1_DMA[5] = { 0,0,0,0,0 }; 	// Dummy conversion (ST workaround for -x),
volatile uint16_t ADC2_DMA[3] = { 0,0,0 }; 		// Dummy conversion (ST workaround for -x)
#define ALPHA_CURRENT_SENSE_OFFSET	0.001f
static uint16_t motor_current_input_adc[3] = {0.0f,0.0f,0.0f};
static uint16_t motor_current_sample_adc[3] = {0.0f,0.0f,0.0f};
static float motor_current_input_adc_offset[3] = {2464.0f,2482.0f,2485.0f};
static float motor_current_input_adc_mA[3] = {0.28f,0.28f,0.28f}; // 0.28f
static float motor_current_mA[3] = {0.0f,0.0f,0.0f};
#define ALPHA_CURRENT_DQ	0.05f
static float present_Id_filtered = 0.0f;
static float present_Iq_filtered = 0.0f;
static pid_context_t pid_flux;
static pid_context_t pid_torque;
// foc feedback
static float present_current_sq = 0.0f;
static float absolute_position_rad = 0.0f;
static float absolute_position_multi_rad = 0.0f;
static float velocity_dps = 0.0f;
// foc performance monitoring (public)
float average_processing_time = 0.0f;
// foc analog measure (public)
static float potentiometer_input_adc = 0.0f;
static float vbus_input_adc = 0.0f;
static float temperature_input_adc = 0.0f;
float present_voltage_V = 0.0f;
float present_temperature_C = 0.0f;

void API_FOC_Init()
{
	// Motor PWM init and BRAKE
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1) ;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2) ;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3) ;
	// OPAMP and ADC init
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_DMA,5);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_DMA,3);
	// CORDIC init
	API_CORDIC_Processor_Init();
	// encoder init
	API_AS5048A_Position_Sensor_Init(&htim4);
	// PID init
	pid_reset(&pid_flux);
	pid_reset(&pid_torque);
	// state
	present_current_sq = 0.0f;
	absolute_position_rad = 0.0f;
	absolute_position_multi_rad = 0.0f;
	velocity_dps = 0.0f;
}


void API_FOC_Update_Torque_Open_Loop(
		uint16_t present_time_us,
		float setpoint_velocity_dps,
		float setpoint_flux_voltage_100
)
{
	// compute theta
	static float theta_rad = 0.0f;
	static float last_time_us = 0.0f;
	uint16_t delta_t_us = last_time_us-present_time_us;
	last_time_us = present_time_us;
	theta_rad += DEGREES_TO_RADIANS(setpoint_velocity_dps) * (float)delta_t_us/1000000.0f;
	// compute cosine(theta)
	static float cosine_theta = 0.0f;
	static float sine_theta = 0.0f;
	API_CORDIC_Processor_Update(theta_rad,&cosine_theta,&sine_theta);
	// Vd
	float const Vd = fconstrain(setpoint_flux_voltage_100/100.0f,-1.0f,1.0f);
	// (Vd) [0.1] to (Valpha,Vbeta) [-1.0,1.0] [Inverse Park Transformation]
	float const Valpha = Vd * cosine_theta;
	float const Vbeta = Vd * sine_theta;
	// (Valpha,Vbeta) [-1.0,1.0] to (Va,Vb,Vc) [-1.0,1.0] [Inverse Clarke Transformation]
	static float const sqrt3 = sqrtf(3.0f);
	float const Va = Valpha;
	float const Vb = (-Valpha+sqrt3*Vbeta)/2.0f;
	float const Vc = (-Valpha-sqrt3*Vbeta)/2.0f;
	// (Va,Vb,Vc) [-1.0,1.0] to PWM duty cycle % [0 1]
	float const duty_cycle_PWMa = fconstrain((Va+1.0f)*0.5f,0.0f,1.0f); // [0 1]
	float const duty_cycle_PWMb = fconstrain((Vb+1.0f)*0.5f,0.0f,1.0f); // [0 1]
	float const duty_cycle_PWMc = fconstrain((Vc+1.0f)*0.5f,0.0f,1.0f); // [0 1]
	// fPWM = 20KHz
	// fTIM = 150MHz
	// in PWM centered mode, for the finest possible resolution :
	// ARR = fTIM/(2 * fPWM) -1 => ARR = 3749
	uint16_t const CCRa = (uint16_t)(duty_cycle_PWMa*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
	uint16_t const CCRb = (uint16_t)(duty_cycle_PWMb*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
	uint16_t const CCRc = (uint16_t)(duty_cycle_PWMc*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
	// update motor PWM
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCRa);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCRb);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CCRc);
}

void API_FOC_Update_Torque_Closed_Loop(
		uint16_t present_time_us,
		float setpoint_torque_current_mA,
		float setpoint_flux_current_mA,
		float phase_synchro_offset_rad,
		uint32_t closed_loop
)
{
	// note : FOC period is less than motor PWM period
	// drop phase current samples a few times
	if(current_samples>current_sample_drop_rate)
	{
		current_samples-=(current_sample_drop_rate+1);

		// backup 3-phase currents
		memcpy(motor_current_sample_adc,motor_current_input_adc,sizeof(uint16_t)*3);

		// performance monitoring
		uint16_t const t_begin = __HAL_TIM_GET_COUNTER(&htim6);

		// process absolute position, and compute theta ahead using average processing time and velocity
		//absolute_position_rad = API_AS5048A_Position_Sensor_Get_Radians_Estimation(t_begin+average_processing_time);
		absolute_position_rad = API_AS5048A_Position_Sensor_Get_Radians_Estimation(t_begin); // we suppose that ADC sample have just been acquired (a few us before...)
		absolute_position_multi_rad = API_AS5048A_Position_Sensor_Get_Multiturn_Radians();
		velocity_dps = API_AS5048A_Position_Sensor_Get_DPS();

		// process temperature (STM32G431-ESC1 specific)
		{
			static float const R60 = 4700.0f; // ohm
			static float const eps = 0.1f; // epsilon (avoid divide by zero)
			float const R_NTC = R60*(4096.0f/(temperature_input_adc+eps)-1.0f); // 10kohm NTC at 25°C
			static float const Beta = 3455.0f; // for a 10k NTC
			static float const Kelvin = 273.15f; //°C
			static float const T0 = Kelvin + 25.0f;
			static float const R0 = 10000.0f; // 10kohm at 25° for 10k NTC
			float const present_temperature_K = Beta * T0 / ( Beta - T0*logf(R0/R_NTC) );
			present_temperature_C = present_temperature_K-Kelvin;
		}

		// apply thermal protection and update hardware error register
		float const max_temperature_C = regs[REG_TEMPERATURE_LIMIT];
		if(present_temperature_C>max_temperature_C)
		{
			setpoint_torque_current_mA = 0.0f;
			setpoint_flux_current_mA = 0.0f;
			// set overheating error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_OVERHEATING;
		}
		else
		{
			// clear overheating error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_OVERHEATING);
		}

		// process input voltage (STM32G431-ESC1 specific)
		{
			static float const R68 = 169.0f; // kohm
			static float const R76 = 18.0f; // kohm
			present_voltage_V = vbus_input_adc/4096.0f*3.3f*(R68+R76)/R76;
		}

		// apply voltage protection and update
		float const min_voltage_V = regs[REG_LOW_VOLTAGE_LIMIT];
		float const max_voltage_V = regs[REG_HIGH_VOLTAGE_LIMIT];
		if((present_voltage_V>max_voltage_V)||(present_voltage_V<min_voltage_V))
		{
			setpoint_torque_current_mA = 0.0f;
			setpoint_flux_current_mA = 0.0f;
			// set voltage error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_VOLTAGE;
		}
		else
		{
			// clear voltage error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_VOLTAGE);
		}


		// process phase current
		// Note : when current flows inward phase, shunt voltage is negative
		// Note : when current flows outward phase, shunt voltage is positive
		// Note : The current sign will be reversed when computing PID error
		for(size_t index=0;index<3;++index)
		{
			 motor_current_mA[index]= ((float)motor_current_sample_adc[index]-motor_current_input_adc_offset[index])/motor_current_input_adc_mA[index];
		}
		present_current_sq = 2.0f/3.0f*(powf(motor_current_mA[0],2.0f)+powf(motor_current_mA[1],2.0f)+powf(motor_current_mA[2],2.0f));

		// process theta for Park and Clarke Transformation and compute cosine(theta) and sine(theta)
		float const phase_offset_rad = DEGREES_TO_RADIANS((int16_t)(MAKE_SHORT(regs[REG_MOTOR_SYNCHRO_L],regs[REG_MOTOR_SYNCHRO_H])));
		float const reg_reverse_phase = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
		float const reg_pole_pairs = regs[REG_MOTOR_POLE_PAIRS];
		float const theta_rad = fmodf(absolute_position_rad*reg_reverse_phase*reg_pole_pairs,M_2PI) + phase_offset_rad + phase_synchro_offset_rad; // theta
		// TODO : corriger avec phase velocity pour compenser le temps de calcul des consignes (anticipation de theta)
		static float cosine_theta = 0.0f;
		static float sine_theta = 0.0f;
		API_CORDIC_Processor_Update(theta_rad,&cosine_theta,&sine_theta);

		// phase current (Ia,Ib,Ic) [0..xxxmA] to (Ialpha,Ibeta) [0..xxxmA] [Clarke Transformation]
		static float const sqrt3 = sqrtf(3.0f);
		float const present_Ialpha = 2.0f/3.0f*motor_current_mA[0]-1.0f/3.0f*(motor_current_mA[1]+motor_current_mA[2]);
		float const present_Ibeta = 1.0f/sqrt3*(motor_current_mA[1]-motor_current_mA[2]);
		// Note Ialpha synchone de Ia et de même phase/signe
		// Note Ibeta suit Iaplha de 90°

		// (Ialpha,Ibeta) [0..xxxmA] to (Id,Iq) [0..xxxmA] [Park Transformation]
		float present_Id =  present_Ialpha*cosine_theta+present_Ibeta*sine_theta;
		float present_Iq = -present_Ialpha*sine_theta+present_Ibeta*cosine_theta;

		// (Id,Iq) filtering
		present_Id_filtered = ALPHA_CURRENT_DQ*present_Id+(1.0f-ALPHA_CURRENT_DQ)*present_Id_filtered;
		present_Iq_filtered = ALPHA_CURRENT_DQ*present_Iq+(1.0f-ALPHA_CURRENT_DQ)*present_Iq_filtered;

		// flux controller (PI+FF) ==> Vd [-1.0,1.0]
		float const setpoint_Id = setpoint_flux_current_mA;
		float const Flux_Kp = 0.0012f; // Kp 0.012 HT4310
		float const Flux_Ki = 0.0000000f; // Ki 0.0001 HT4310
		float const Flux_Kff = 0.00f; //
		// NOTE : Current phase ABC = - shunt current/voltage ABC (REVERSED)
		float const error_Id = setpoint_Id+( closed_loop == 1 ? present_Id_filtered : 0.0f);
		float const Vd = pid_process_antiwindup_clamp_with_ff(
				&pid_flux,
				error_Id,
				Flux_Kp,
				Flux_Ki,
				0.0f,
				1.0f,
				0.0f,
				Flux_Kff*setpoint_Id
		);

		// torque controller (PI+FF) ==> Vq [-1.0,1.0]
		float const setpoint_Iq = setpoint_torque_current_mA;
		float const Torque_Kp = 0.0012f; // Kp 0.012 HT4310
		float const Torque_Ki = 0.0000000f; // Ki 0.0001 HT4310
		float const Torque_Kff = 0.000f; // 0.004f
		// NOTE : Current phase ABC = - shunt current/voltage ABC (REVERSED)
		float const error_Iq = setpoint_Iq+( closed_loop == 1 ? present_Iq_filtered : 0.0f);
		float const Vq = pid_process_antiwindup_clamp_with_ff(
				&pid_torque,
				error_Iq,
				Torque_Kp,
				Torque_Ki,
				0.0f,
				1.0f,
				0.0f,
				Torque_Kff*setpoint_Iq
		);

		// (Vd,Vq) [-1.0,1.0] to (Valpha,Vbeta) [-1.0,1.0] [Inverse Park Transformation]
		float const Valpha = Vd * cosine_theta - Vq * sine_theta;
		float const Vbeta = Vq * cosine_theta + Vd * sine_theta;

		// (Valpha,Vbeta) [-1.0,1.0] to (Va,Vb,Vc) [-1.0,1.0] [Inverse Clarke Transformation]
		float const Va = Valpha;
		float const Vb = (-Valpha+sqrt3*Vbeta)/2.0f;
		float const Vc = (-Valpha-sqrt3*Vbeta)/2.0f;

		// SVM post-processing
#define SVM
#ifdef SVM
		float const Vneutral = 0.5f*(fmaxf(fmaxf(Va,Vb),Vc)+fminf(fminf(Va,Vb),Vc));
		float const Va_svm = Va-Vneutral;
		float const Vb_svm = Vb-Vneutral;
		float const Vc_svm = Vc-Vneutral;
#endif

		// (Va,Vb,Vc) [-1.0,1.0] to PWM duty cycle % [0 1]
		// TODO take in account Vbus
		// TODO take in account Vbus
		// TODO take in account Vbus
		// TODO take in account Vbus

#ifdef SVM
		float const duty_cycle_PWMa = fconstrain((Va_svm+1.0f)*0.5f,0.0f,1.0f); // [0 1]
		float const duty_cycle_PWMb = fconstrain((Vb_svm+1.0f)*0.5f,0.0f,1.0f); // [0 1]
		float const duty_cycle_PWMc = fconstrain((Vc_svm+1.0f)*0.5f,0.0f,1.0f); // [0 1]
#else
		float const duty_cycle_PWMa = fconstrain((Va+1.0f)*0.5f,0.0f,1.0f); // [0 1]
		float const duty_cycle_PWMb = fconstrain((Vb+1.0f)*0.5f,0.0f,1.0f); // [0 1]
		float const duty_cycle_PWMc = fconstrain((Vc+1.0f)*0.5f,0.0f,1.0f); // [0 1]
#endif
		// fPWM = 20KHz
		// fTIM = 150MHz
		// in PWM centered mode, for the finest possible resolution :
		// ARR = fTIM/(2 * fPWM) -1 => ARR = 3749
		uint16_t const CCRa = (uint16_t)(duty_cycle_PWMa*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
		uint16_t const CCRb = (uint16_t)(duty_cycle_PWMb*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
		uint16_t const CCRc = (uint16_t)(duty_cycle_PWMc*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
		// update motor PWM
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCRa);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCRb);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CCRc);

		// performance monitoring
		uint16_t const t_end = __HAL_TIM_GET_COUNTER(&htim6);
		uint16_t const t_tp = t_end-t_begin;
		static const float alpha_performance_monitoring = 0.01;
		average_processing_time = (1.0f-alpha_performance_monitoring)*average_processing_time+alpha_performance_monitoring*(float)t_tp;

		// TRACE/DEBUG
		static uint32_t count = 0;
		if(++count%4==0)
		{
			// motor current adc offset
			//HAL_Serial_Print(&serial,"%d\n",(int)average_processing_time);
//			HAL_Serial_Print(&serial,"%d %d\n",
//					(int)RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_Radians()),
//					(int)RADIANS_TO_DEGREES(absolute_position_rad)
//				);

			//HAL_Serial_Print(&serial,"%d %d %d\n",(int)motor_current_input_adc_offset[0], (int)motor_current_input_adc_offset[1],(int)motor_current_input_adc_offset[2]);
//			HAL_Serial_Print(&serial,"%d %d %d %d %d %d\n",
//					(int)(Va*100.0f),
//					(int)(Vb*100.0f),
//					(int)(Vc*100.0f),
//					(int)(motor_current_mA[0]),
//					(int)(motor_current_mA[1]),
//					(int)(motor_current_mA[2])
//				);
			// AS5048A position
			//float const absolute_position_rad = API_AS5048A_Position_Sensor_Get_Radians();
			//HAL_Serial_Print(&serial,"%d: %d\n",(int)present_time_us, (int)RADIANS_TO_DEGREES(absolute_position_rad));
			// Position & Velocity
			//HAL_Serial_Print(&serial,"%d(%d): %d %d %d\n",(int)present_cycle_us, (int)t_tp, (int)RADIANS_TO_DEGREES(absolute_position_rad),(int)RADIANS_TO_DEGREES(absolute_position_multi_rad),(int)(velocity_dps));
//			HAL_Serial_Print(&serial,"%d %d %d\n",
//					(int)RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_Radians()),
//					(int)RADIANS_TO_DEGREES(absolute_position_rad),
//					(int)(velocity_dps)
//			);
			// ADC curents
			//HAL_Serial_Print(&serial,"%d(%d): %d %d %d\n",(int)present_cycle_us, (int)t_tp, (int)motor_current_sample_adc[0],(int)motor_current_sample_adc[1],(int)motor_current_sample_adc[2]);
			//HAL_Serial_Print(&serial,"%d(%d): %dmA %dmA %dmA\n",(int)present_cycle_us, (int)t_tp, (int)motor_current_mA[0],(int)motor_current_mA[1],(int)motor_current_mA[2]);
			// THETA+CCR
			//HAL_Serial_Print(&serial,"%d(%d): %d %d %d %d\n",(int)RADIANS_TO_DEGREES(theta_rad), (int)present_cycle_us, (int)t_tp, (int)CCRa,(int)CCRb,(int)CCRc);
			//Duty cycle
			//HAL_Serial_Print(&serial,"%d(%d): %d %d %d\n",(int)present_cycle_us, (int)t_tp, (int)(duty_cycle_PWMa*100.0f),(int)(duty_cycle_PWMb*100.0f),(int)(duty_cycle_PWMc*100.0f));
			//theta
			//HAL_Serial_Print(&serial,"%d(%d): %d %d \n",(int)present_cycle_us, (int)t_tp, (int)RADIANS_TO_DEGREES(absolute_position_rad),(int)RADIANS_TO_DEGREES(theta_rad));
			// currents DQ
			//HAL_Serial_Print(&serial,"%d(%d): d:%d q:%d \n",(int)present_cycle_us, (int)t_tp, (int)present_Id,(int)present_Iq);
			//HAL_Serial_Print(&serial,"%d(%d): d:%d q:%d \n",(int)present_cycle_us, (int)t_tp, (int)present_Id_filtered,(int)present_Iq_filtered);
			//HAL_Serial_Print(&serial,"%d(%d): alpha:%d beta:%d \n",(int)present_cycle_us, (int)t_tp, (int)present_Ialpha,(int)present_Ibeta);


			// LOG
			// motor current adc
			//HAL_Serial_Print(&serial,"%d %d %d\n",(int)motor_current_input_adc[0], (int)motor_current_input_adc[1],(int)motor_current_input_adc[2]);
			//HAL_Serial_Print(&serial,"%d %d %d\n",(int)motor_current_mA[0],(int)motor_current_mA[1],(int)motor_current_mA[2]);
			// IdIq
			//HAL_Serial_Print(&serial,"%d %d\n",(int)present_Id_filtered,(int)present_Iq_filtered);
			// alphabeta
			/*
			HAL_Serial_Print(&serial,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
					(int)RADIANS_TO_DEGREES(theta_rad),
					(int)(motor_current_mA[0]),
					(int)(motor_current_mA[1]),
					(int)(motor_current_mA[2]),
					(int)present_Ialpha,
					(int)present_Ibeta,
					(int)present_Id_filtered,
					(int)present_Iq_filtered,
					(int)(Vd*100.0f),
					(int)(Vq*100.0f),
					(int)(Valpha*100.0f),
					(int)(Vbeta*100.0f),
					(int)(duty_cycle_PWMa*100.0f),
					(int)(duty_cycle_PWMb*100.0f),
					(int)(duty_cycle_PWMc*100.0f)
				);
				*/
//			HAL_Serial_Print(&serial,"%d %d %d %d %d %d\n",
//					(int)present_Id,
//					(int)present_Iq,
//					(int)(fabs(present_Iq)+fabs(present_Id)),
//					(int)present_Id_filtered,
//					(int)present_Iq_filtered,
//					(int)(fabs(present_Iq_filtered)+fabs(present_Id_filtered))
//				);
			// CCR + THETA
			//HAL_Serial_Print(&serial,"%d %d %d %d\n",(int)RADIANS_TO_DEGREES(theta_rad), (int)CCRa,(int)CCRb,(int)CCRc);
			// DutyCycle + THETA
			//HAL_Serial_Print(&serial,"%d %d %d %d\n",(int)RADIANS_TO_DEGREES(theta_rad), (int)(duty_cycle_PWMa*100.0f),(int)(duty_cycle_PWMb*100.0f),(int)(duty_cycle_PWMc*100.0f));
			// Valphabeta + THETA
			//HAL_Serial_Print(&serial,"%d %d %d\n",(int)RADIANS_TO_DEGREES(theta_rad), (int)(Valpha*100.0f),(int)(Vbeta*100.0f));
			// THETA ValphaVbeta Ialpha Ibeta
			//HAL_Serial_Print(&serial,"%d %d %d %d %d\n",(int)RADIANS_TO_DEGREES(theta_rad), (int)(Valpha*100.0f),(int)(Vbeta*100.0f),(int)present_Ialpha,(int)present_Ibeta);
			// THETA Vdq Id Iq
			//HAL_Serial_Print(&serial,"%d %d %d %d %d\n",(int)RADIANS_TO_DEGREES(theta_rad), (int)(Vd*100.0f),(int)(Vq*100.0f),(int)present_Id_filtered,(int)present_Iq_filtered);
			// synchro
			//HAL_Serial_Print(&serial,"%d %d\n",(int)RADIANS_TO_DEGREES(phase_synchro_offset_rad),(int)velocity_filtered_dps);

			// speed and theta
			//HAL_Serial_Print(&serial,"%d %d\n",(int)RADIANS_TO_DEGREES(theta_rad),(int)velocity_filtered_dps);
		}
	}
}

float API_FOC_Get_Present_Current_SQ()
{
	return present_current_sq;
}

float API_FOC_Get_Present_Position()
{
	return absolute_position_rad;
}

float API_FOC_Get_Present_Position_Multi()
{
	return absolute_position_multi_rad;
}

float API_FOC_Get_Present_Velocity()
{
	return velocity_dps;
}

float API_FOC_Get_Present_Torque_Current()
{
	return present_Iq_filtered;
}

float API_FOC_Get_Present_Flux_Current()
{
	return present_Id_filtered;
}

void API_FOC_It(ADC_HandleTypeDef *hadc)
{
	if(hadc==&hadc1)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			// Filter (EWMA) position and voltage ADC samples
			motor_current_input_adc[0] = ADC1_DMA[1];
			potentiometer_input_adc = ADC1_DMA[2];
			vbus_input_adc = ADC1_DMA[3];
			temperature_input_adc = ADC1_DMA[4];
		}
		else
		{
			motor_current_input_adc_offset[0] = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC1_DMA[1]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset[0];
		}
		// restart ADC
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_DMA,5);
	}
	if(hadc==&hadc2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			// Filter (EWMA) position and voltage ADC samples
			motor_current_input_adc[1] = ADC2_DMA[1];
			motor_current_input_adc[2] = ADC2_DMA[2];
			++current_samples;
		}
		else
		{
			motor_current_input_adc_offset[1] = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC2_DMA[1]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset[1];
			motor_current_input_adc_offset[2] = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC2_DMA[2]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset[2];
		}
		// restart ADC
		HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_DMA,3);
	}
}

