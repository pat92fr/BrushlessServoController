/*
 * foc.c
 *
 *  Created on: 16 janv. 2021
 *      Author: Patrick, Kai
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

// hard-coded settings
#define ALPHA_CURRENT_DQ			0.05f 	// low pass filter for present Id and presetn Iq estimation
#define ALPHA_CURRENT_SENSE_OFFSET	0.001f 	// low pass filter for calibrating the phase current ADC offset (automatically)
#define MAX_PWM_DUTY_CYCLE 			0.95f 	// %
#define MIN_PWM_DUTY_CYCLE 			0.05f 	// %
#define CSVPWM 					 	// uncomment to use CSVPWM (conventional space vector pulse width modulation),
								 	// if commented default SPWM is used

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
// TODO : use STM32 CUBE MONITOR
extern HAL_Serial_Handler serial;

// FOC period at PWM output = 16kHz (check TIMER1 ARR value = 4999 and timer frequency =160MHz)
static uint32_t const current_sample_drop_rate = 1;
// 3:250us cycle
// 2:187us cycle
// 1:125us cycle <- default (conservative, allows debbuging)
// 0: 62us cycle <- best possible (one FOC iteration takes about ~54us of processing time)

// FOC private variables
static int32_t current_samples = 0;
volatile uint16_t ADC1_DMA[5] = { 0,0,0,0,0 }; 	// Dummy conversion (ST workaround for -x),
volatile uint16_t ADC2_DMA[3] = { 0,0,0 }; 		// Dummy conversion (ST workaround for -x)
static uint16_t motor_current_input_adc[3] = {0.0f,0.0f,0.0f};
static uint16_t motor_current_sample_adc[3] = {0.0f,0.0f,0.0f};
static float motor_current_input_adc_offset[3] = {2464.0f,2482.0f,2485.0f};
static float motor_current_input_adc_mA[3] = {0.28f,0.28f,0.28f}; // 0.28f
static float motor_current_mA[3] = {0.0f,0.0f,0.0f};
static float present_Id_filtered = 0.0f;
static float present_Iq_filtered = 0.0f;
static pid_context_t pid_flux;
static pid_context_t pid_torque;
// foc feedback
static float present_current_sq = 0.0f;
static float absolute_position_rad = 0.0f;
static float absolute_position_multi_rad = 0.0f;
static float velocity_dps = 0.0f;
// foc analog measure
static float potentiometer_input_adc = 0.0f;
static float vbus_input_adc = 0.0f;
static float temperature_input_adc = 0.0f;
static float present_voltage_V = 0.0f;
static float present_temperature_C = 0.0f;
// foc performance monitoring (public)
static float average_processing_time_us = 0.0f;

// user API function
// this function reset state of FOC
// This function starts peripherals
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
}

// low level function
// this function update present_temperature_C
// this function update REG_HARDWARE_ERROR_STATUS register (set/reset HW_ERROR_BIT_OVERHEATING bit)
// this function use REG_TEMPERATURE_LIMIT register
// the temperature ADC samples are collected with phase current samples
void LL_FOC_Update_Temperature()
{
	// convert ADC sample into temperature (STM32G431-ESC1 specific)
	static float const R60 = 4700.0f; // ohm
	static float const eps = 0.1f; // epsilon (avoid divide by zero)
	float const R_NTC = R60*(4096.0f/(temperature_input_adc+eps)-1.0f); // 10kohm NTC at 25°C
	static float const Beta = 3455.0f; // for a 10k NTC
	static float const Kelvin = 273.15f; //°C
	static float const T0 = 273.15f + 25.0f;
	static float const R0 = 10000.0f; // 10kohm at 25° for 10k NTC
	float const present_temperature_K = Beta * T0 / ( Beta - T0*logf(R0/R_NTC) );
	present_temperature_C = present_temperature_K-Kelvin;

	// apply thermal protection and update hardware error register
	float const max_temperature_C = regs[REG_TEMPERATURE_LIMIT];
	if(present_temperature_C>max_temperature_C)
	{
		// set overheating error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_OVERHEATING;
	}
	else
	{
		// clear overheating error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_OVERHEATING);
	}
}

// low level function
// this function update present_voltage_V
// this function update REG_HARDWARE_ERROR_STATUS register (set/reset HW_ERROR_BIT_VOLTAGE bit)
// this function use REG_LOW_VOLTAGE_LIMIT and REG_HIGH_VOLTAGE_LIMIT registers
// the voltage ADC samples are collected with phase current samples
void LL_FOC_Update_Voltage()
{
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
		// set voltage error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_VOLTAGE;
	}
	else
	{
		// clear voltage error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_VOLTAGE);
	}
}

// low level function
// this function checks REG_HARDWARE_ERROR_STATUS register and enforce BRAKE is register not null
// this function use the present_voltage_V state variable to adjust PWM duty cycle according power supply voltage
void LL_FOC_Inverse_Clarke_Park_PWM_Generation( float Vd, float Vq, float cosine_theta, float sine_theta )
{
	// convert (Vd,Vq) [-max_voltage_V,max_voltage_V] to (Valpha,Vbeta) [-max_voltage_V,max_voltage_V] [Inverse Park Transformation]
	float const Valpha = Vd * cosine_theta - Vq * sine_theta;
	float const Vbeta = Vq * cosine_theta + Vd * sine_theta;

	// convert (Valpha,Vbeta) [-max_voltage_V,max_voltage_V] to (Va,Vb,Vc) [-max_voltage_V,max_voltage_V] [Inverse Clarke Transformation]
	static float const sqrt3 = sqrtf(3.0f);
	float Va = Valpha;
	float Vb = (-Valpha+sqrt3*Vbeta)/2.0f;
	float Vc = (-Valpha-sqrt3*Vbeta)/2.0f;
	// SPWM done

#ifdef CSVPWM

	// apply CSVPWM to (Va,Vb,Vc)
	float const Vneutral = 0.5f*(fmaxf(fmaxf(Va,Vb),Vc)+fminf(fminf(Va,Vb),Vc));
	Va -= Vneutral;
	Vb -= Vneutral;
	Vc -= Vneutral;

#endif

	// convert (Va,Vb,Vc) [-max_voltage_V,max_voltage_V] to PWM duty cycles % [0.0 1.0]
	float const duty_cycle_PWMa = fconstrain((Va/present_voltage_V+1.0f)*0.5f,MIN_PWM_DUTY_CYCLE,MAX_PWM_DUTY_CYCLE); // [0 1]
	float const duty_cycle_PWMb = fconstrain((Vb/present_voltage_V+1.0f)*0.5f,MIN_PWM_DUTY_CYCLE,MAX_PWM_DUTY_CYCLE); // [0 1]
	float const duty_cycle_PWMc = fconstrain((Vc/present_voltage_V+1.0f)*0.5f,MIN_PWM_DUTY_CYCLE,MAX_PWM_DUTY_CYCLE); // [0 1]

	// convert PWM duty cycles % to TIMER1 CCR register values
	// fPWM = 16KHz
	// fTIM = 160MHz
	// in PWM centered mode, for the finest possible resolution :
	// ARR = fTIM/(2 * fPWM) -1 => ARR = 4999
	uint16_t const CCRa = (uint16_t)(duty_cycle_PWMa*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
	uint16_t const CCRb = (uint16_t)(duty_cycle_PWMb*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;
	uint16_t const CCRc = (uint16_t)(duty_cycle_PWMc*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1;

	// update TIMER CCR registers
	//   and apply BRAKE if error
	if(regs[REG_HARDWARE_ERROR_STATUS] != 0 )
	{
		// compute a valid BRAKE value
		uint16_t const CCRx = (uint16_t)(0.5f*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))+1; // note : 0 is OK too
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCRx);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCRx);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CCRx);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCRa);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCRb); // switch b and c phases
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CCRc); // switch b and c phases
	}
}

// user API function
// this function process an open-loop FOC from electrical angle and voltage setpoints
void API_FOC_Set_Flux_Angle(
		float setpoint_electrical_angle_rad,
		float setpoint_flux_voltage_V
)
{
	// check temperature and voltage
	LL_FOC_Update_Temperature();
	LL_FOC_Update_Voltage();

	// compute theta
	float const theta_rad = normalize_angle(setpoint_electrical_angle_rad);

	// compute cosine and sine
	static float cosine_theta = 0.0f;
	static float sine_theta = 0.0f;
	API_CORDIC_Processor_Update(theta_rad,&cosine_theta,&sine_theta);

	// compute (Vd,Vq) [-max_voltage_V,max_voltage_V]
	float const Vd = fconstrain(setpoint_flux_voltage_V,-regs[REG_HIGH_VOLTAGE_LIMIT_VALUE],regs[REG_HIGH_VOLTAGE_LIMIT_VALUE]); // torque setpoint open loop
	float const Vq = 0.0f; // no torque

	// do inverse clarke and park transformation and update TIMER1 register (3-phase PWM generation)
	LL_FOC_Inverse_Clarke_Park_PWM_Generation(Vd,Vq,cosine_theta,sine_theta);
	// this function checks REG_HARDWARE_ERROR_STATUS register and enforce BRAKE is register not null
	// this function use the present_voltage_V state variable to adjust PWM duty cycle according power supply voltage
}

// user API function
// this function process an open-loop FOC from electrical velocity and voltage setpoints
void API_FOC_Set_Flux_Velocity(
		uint16_t present_time_us,
		float setpoint_electrical_velocity_dps,
		float setpoint_flux_voltage_V
)
{
	// check temperature and voltage
	LL_FOC_Update_Temperature();
	LL_FOC_Update_Voltage();

	// compute theta
	static float theta_rad = 0.0f;
	static float last_time_us = 0.0f;
	uint16_t delta_t_us = last_time_us-present_time_us;
	last_time_us = present_time_us;
	theta_rad += DEGREES_TO_RADIANS(setpoint_electrical_velocity_dps) * (float)delta_t_us/1000000.0f;

	// compute cosine and sine
	static float cosine_theta = 0.0f;
	static float sine_theta = 0.0f;
	API_CORDIC_Processor_Update(theta_rad,&cosine_theta,&sine_theta);

	// compute (Vd,Vq) [-max_voltage_V,max_voltage_V]
	float const Vd = fconstrain(setpoint_flux_voltage_V,-regs[REG_HIGH_VOLTAGE_LIMIT_VALUE],regs[REG_HIGH_VOLTAGE_LIMIT_VALUE]); // torque setpoint open loop
	float const Vq = 0.0f; // no torque

	// do inverse clarke and park transformation and update TIMER1 register (3-phase PWM generation)
	LL_FOC_Inverse_Clarke_Park_PWM_Generation(Vd,Vq,cosine_theta,sine_theta);
	// this function checks REG_HARDWARE_ERROR_STATUS register and enforce BRAKE is register not null
	// this function use the present_voltage_V state variable to adjust PWM duty cycle according power supply voltage
}

// user API function
// this function synchronize physical and electrical angles, set motor normal/reverse rotation, and check pole pairs
// this function uses REG_MOTOR_POLE_PAIRS register
int API_FOC_Calibrate()
{
	// reset settings
	regs[REG_INV_PHASE_MOTOR] = 0;
	regs[REG_MOTOR_SYNCHRO_L] = 0;
	regs[REG_MOTOR_SYNCHRO_H] = 0;

	// find natural direction

	// set electrical angle
	float setpoint_electrical_angle_rad = M_3PI_2;
	float setpoint_flux_voltage_V = 1.0f; // hard-coded V setpoint
	API_FOC_Set_Flux_Angle(setpoint_electrical_angle_rad,setpoint_flux_voltage_V);

    // move one electrical revolution forward
    for (int i = 0; i <=500; ++i )
    {
    	setpoint_electrical_angle_rad = M_3PI_2 + M_2PI * i / 500.0f;
    	API_FOC_Set_Flux_Angle(setpoint_electrical_angle_rad,setpoint_flux_voltage_V);
    	HAL_Delay(2);
    }
    HAL_Delay(200);
    // take and angle in the middle
    float const mid_angle = API_AS5048A_Position_Sensor_Get_Radians();

    // move one electrical revolution backward
    for (int i = 500; i >=0; --i )
    {
    	setpoint_electrical_angle_rad = M_3PI_2 + M_2PI * i / 500.0f;
    	API_FOC_Set_Flux_Angle(setpoint_electrical_angle_rad,setpoint_flux_voltage_V);
    	HAL_Delay(2);
    }
    HAL_Delay(200);
    // take and angle in the end
    float const end_angle = API_AS5048A_Position_Sensor_Get_Radians();

    // release motor
    API_FOC_Set_Flux_Angle(0.0f,0.0f);

    // determine the direction the sensor moved
    float const delta_angle = mid_angle-end_angle;
    if(fabsf(delta_angle)<0.1f) // arbitrary delta angle
    {
    	return 1; // failed calibration
    }
    if(delta_angle>0.0f)
    {
    	regs[REG_INV_PHASE_MOTOR] = 0;
    	HAL_Serial_Print(&serial,"Normal (%d)\n",0 ); // CCW
    }
    else
    {
    	regs[REG_INV_PHASE_MOTOR] = 1;
    	HAL_Serial_Print(&serial,"Reverse (%d)\n",1 ); // CW
    }

    // check pole pairs
    float const reg_pole_pairs = regs[REG_MOTOR_POLE_PAIRS];
    if( fabsf(fabsf(delta_angle)*reg_pole_pairs-M_2PI) > 0.5f )
    {
    	HAL_Serial_Print(&serial,"PP error (%d)\n",(int)( M_2PI/fabsf(delta_angle) ) );
    	return 2; // failed calibration
    }

    // set electrical angle
    setpoint_electrical_angle_rad = 0.0f;
    setpoint_flux_voltage_V = 1.0f; // hard-coded V setpoint
    API_FOC_Set_Flux_Angle(setpoint_electrical_angle_rad,setpoint_flux_voltage_V);
    HAL_Delay(1000);
    float const reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
    float const phase_synchro_offset_rad = normalize_angle(-API_AS5048A_Position_Sensor_Get_Radians()*reg_pole_pairs*reverse);
	HAL_Serial_Print(&serial,"Synchro (%d)\n",(int)(RADIANS_TO_DEGREES(phase_synchro_offset_rad)) );
	regs[REG_MOTOR_SYNCHRO_L] = LOW_BYTE((int)RADIANS_TO_DEGREES(phase_synchro_offset_rad));
	regs[REG_MOTOR_SYNCHRO_H] = HIGH_BYTE((int)RADIANS_TO_DEGREES(phase_synchro_offset_rad));

	// release motor
	API_FOC_Set_Flux_Angle(0.0f,0.0f);

	// store calibration into EEPROM
	store_eeprom_regs();

	return 0; // calibration success
}

// user API function
// this function process an closed-loop FOC from flux and torque current setpoints
// this function allow on-the-go synchronization angle adjustment
// the open loop mode means that the present Id and Iq are forced to 0
//    this may require adjustment of the Kp and Ki of both flux and torque PI regulator
// note : with a 5008 motor, there is no need for Ki and Kff in both flux and torque PI
void API_FOC_Torque_Update(
		uint16_t present_time_us,
		float setpoint_torque_current_mA,
		float setpoint_flux_current_mA,
		float phase_synchro_offset_rad,
		uint32_t closed_loop
)
{
	// note : absolute position increases when turning CCW (encoder)
	// note : when Iq is positive, motor turns CW
	// note : FOC period is less than motor PWM period
	// drop phase current samples a few times between each FOC iteration
	if(current_samples>current_sample_drop_rate)
	{
		current_samples-=(current_sample_drop_rate+1);

		// backup 3-phase currents as soon as possible
		memcpy(motor_current_sample_adc,motor_current_input_adc,sizeof(uint16_t)*3);

		// performance monitoring
		uint16_t const t_begin = __HAL_TIM_GET_COUNTER(&htim6);

		// process absolute position, and compute theta ahead using average processing time and velocity
		//absolute_position_rad = API_AS5048A_Position_Sensor_Get_Radians_Estimation(t_begin+average_processing_time);
		absolute_position_rad = API_AS5048A_Position_Sensor_Get_Radians_Estimation(t_begin); // we suppose that ADC sample have just been acquired (a few us before...)
		absolute_position_multi_rad = API_AS5048A_Position_Sensor_Get_Multiturn_Radians();
		velocity_dps = API_AS5048A_Position_Sensor_Get_DPS();

		// check temperature and voltage
		LL_FOC_Update_Temperature();
		LL_FOC_Update_Voltage();

		// if ALARM then zeroize currents setpoints
		if(regs[REG_HARDWARE_ERROR_STATUS] != 0 )
		{
			setpoint_torque_current_mA = 0.0f;
			setpoint_flux_current_mA = 0.0f;
		}

		// process phase current
		// Note : when current flows inward phase, shunt voltage is negative
		// Note : when current flows outward phase, shunt voltage is positive
		// Note : The current sign is positive when flowing in to a phase
		// Note : The current sign is negative when flowing out from a phase
		for(size_t index=0;index<3;++index)
		{
			 motor_current_mA[index]= -((float)motor_current_sample_adc[index]-motor_current_input_adc_offset[index])/motor_current_input_adc_mA[index]; // note : the (-) sign here
		}
		present_current_sq = 2.0f/3.0f*(powf(motor_current_mA[0],2.0f)+powf(motor_current_mA[1],2.0f)+powf(motor_current_mA[2],2.0f));

		// process theta for Park and Clarke Transformation and compute cosine(theta) and sine(theta)
		float const phase_offset_rad = DEGREES_TO_RADIANS((int16_t)(MAKE_SHORT(regs[REG_MOTOR_SYNCHRO_L],regs[REG_MOTOR_SYNCHRO_H])));
		float const reg_pole_pairs = regs[REG_MOTOR_POLE_PAIRS];
		float const reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
		float const theta_rad = fmodf(absolute_position_rad*reg_pole_pairs*reverse,M_2PI) + phase_offset_rad + phase_synchro_offset_rad; // theta
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

		// flux controller (PI+FF) ==> Vd [-max_voltage_V,max_voltage_V]
		float const setpoint_Id = setpoint_flux_current_mA;
		float const Flux_Kp = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KP_L],regs[REG_PID_FLUX_CURRENT_KP_H])))/100000.0f;
		float const Flux_Ki = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KI_L],regs[REG_PID_FLUX_CURRENT_KI_H])))/10000000.0f;
		float const Flux_Kff = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KFF_L],regs[REG_PID_FLUX_CURRENT_KFF_H])))/100000.0f;
		float const error_Id = setpoint_Id-( closed_loop == 1 ? present_Id_filtered : 0.0f);
		float const Vd = pid_process_antiwindup_clamp_with_ff(
				&pid_flux,
				error_Id,
				Flux_Kp,
				Flux_Ki,
				0.0f, // no Kd
				regs[REG_HIGH_VOLTAGE_LIMIT_VALUE],
				0.0f, // no derivative low pass filter
				Flux_Kff*setpoint_Id
		);

		// torque controller (PI+FF) ==> Vq [-max_voltage_V,max_voltage_V]
		float const setpoint_Iq = setpoint_torque_current_mA;
		float const Torque_Kp = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KP_L],regs[REG_PID_TORQUE_CURRENT_KP_H])))/100000.0f;
		float const Torque_Ki = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KI_L],regs[REG_PID_TORQUE_CURRENT_KI_H])))/10000000.0f;
		float const Torque_Kff = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KFF_L],regs[REG_PID_TORQUE_CURRENT_KFF_H])))/100000.0f;
		float const error_Iq = setpoint_Iq-( closed_loop == 1 ? present_Iq_filtered : 0.0f);
		float const Vq = pid_process_antiwindup_clamp_with_ff(
				&pid_torque,
				error_Iq,
				Torque_Kp,
				Torque_Ki,
				0.0f, // no Kd
				regs[REG_HIGH_VOLTAGE_LIMIT_VALUE],
				0.0f, // no derivative low pass filter
				Torque_Kff*setpoint_Iq
		);

		// do inverse clarke and park transformation and update TIMER1 register (3-phase PWM generation)
		LL_FOC_Inverse_Clarke_Park_PWM_Generation(Vd,Vq,cosine_theta,sine_theta);

		// performance monitoring
		uint16_t const t_end = __HAL_TIM_GET_COUNTER(&htim6);
		uint16_t const t_tp = t_end-t_begin;
		static const float alpha_performance_monitoring = 0.001f;
		average_processing_time_us = (1.0f-alpha_performance_monitoring)*average_processing_time_us+alpha_performance_monitoring*(float)t_tp;

		// TRACE/DEBUG
		// TODO : use STM32 CUBE MONITOR
		/*static uint32_t count = 0;
		if(++count%4==0)
		{
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
		}*/
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

float API_FOC_Get_Present_Voltage()
{
	return present_voltage_V;
}

float API_FOC_Get_Present_Temp()
{
	return present_temperature_C;
}

float API_FOC_Get_Processing_Time()
{
	return average_processing_time_us;

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

