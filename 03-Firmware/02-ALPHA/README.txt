New GUI Tool :
--------------

- Change line 20 of SBSUtility.py
- Launch SBSUtility.py

Warning :
- Do not use "round position test" and "square current test" at the moment



ESC Firmware update :
---------------------

Clocks :
- CAN interface clocked at 1MBbps
- System frequency increased to 160MHz (for CAN)
- Change of frequencyz PWM outputs to 16KHz (for less heat dissipation)
- Defaut FOC iteration frequency is 8KHz, and may be increased to 16KHz (FOC processing time is about 54us)
- Position and velocity PIDs frequency is 1KHz
- Position sensor update rate is about 1KHz (AS5048A in PWM interface), and may be faster with a SPI or I2C position sensors.

Functionalities :
- New operating mode 0 : Position control with torque limiter and velocity/acceleration profil
	- User configurable position setpoint (CAN and Serial interaces, RAM registers)
	- User configurable velocity and torque limits (CAN and Serial interfaces, RAM resgister)
	- User configurable acceleration (Serial interface, EEPROM register)
- Operating mode 1 is deprecated (PIV regulator)
- SimpleFOC calibration procedure added
- EWMA low pass filter on velocity estimation (user configurable)
- Thermal protection is implemented and temperature threshold is user configurable (default 60°C)
- Voltage protection is implemented and voltage min and max thresholds are user configurable (min 6V, max 30V)
- System error of the position sensor is decoded and triggers an alarm that brakes the motor (HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR set in REG_HARDWARE_ERROR_STATUS register).
- No response from the position sensor is detected and triggers an alarm that brakes the motor (HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR set in REG_HARDWARE_ERROR_STATUS register).
- Power supply voltage is used in the PWM ouput duty cyle computation (both SPWM and CSVPWM algorithms).


Source code :
- clean-up foc.h/.c and new API
	- A slighty better encapsulation but still C with static varaibles at the moment
 	- API_FOC_Update_Torque_Closed_Loop() becomes API_FOC_Torque_Update()
	- New API_FOC_Calibrate()
	- New API_FOC_Set_Flux_Angle() used by calibration procedure
	- New API_FOC_Set_Flux_Velocity()
	- New API_FOC_Get_Present_Voltage()
	- New API_FOC_Get_Present_Temp()
	- New API_FOC_Get_Processing_Time()

Serial interface :
- Resgister deleted :
	- REG_INV_CURRENT_MOTOR
	- REG_INV_ROTATION_SENSOR
- Resgister added :
	- REG_EWMA_ENCODER (default 255)

CAN interface :
- Datarate 1Mbps
- Decoded frames ID = 000h only at the moment
- Frame ID=000h Payload :
	- 8-bit servo ID (same value as register REG_ID)
	- 8-bit operating mode (=0 for position)
	- 16-bit goal position (°)
	- 16-bit goal velocity (°/s)
	- 16-bit goal torque (mA)

Recommandations :
- Uncomment API_FOC_Calibration() the first time, then comment the line again (main.c). The calibration procedure write into EEPROM.
- Kp Ki Kff of flux and torque PI must be reconfigfured (a x15 factor may be applied from 01-Alpha release) 
- Defaut EWMA velocity filter is 255 (not filtering). Maximum filtering value = 0 (not recommanded).
- Acceleration can be set to 32000°/s² for 5008
- Velocity can be set to 10000°/s for 5008
- Current can be set to 5000mA for 5008
- Kp for flux and torque PI is 3000 for 5008
- Ki for flux and torque PI is 0 for 5008
- Kff for flux and torque PI is 0 for 5008
- Motor turns CCW when current is positive (due to AS5048)
- Motor turns CW when current is negative (due to AS5048)


Team :
------

- Kai joined the project, with a slighty different ESC wiring and peripherals. He is using a position sensor with faster interface (I2C). Kai has helped me in improving FOC source code and functionalities. Thank you Kai.





