#haptic_example.py
import time
import serial

from protocol2 import *

servo_A_id = 1	
servo_B_id = 11

servo_A_port = "COM6"	
servo_B_port = "COM15"

def main():

	servo_A = servo_protocol2()
	servo_B = servo_protocol2()

	servo_A.open(servo_A_port,1000000) ##eeprom baud = 3	
	servo_B.open(servo_B_port,1000000) ##eeprom baud = 3

	servo_A.write_byte_command(
				servo_A_id,
				0x80,
				[1], #torque enable
				extra_timeout=50,
				verbose=1
			)
	servo_A.write_byte_command(
				servo_A_id,
				0x8B,
				[200], #kp
				extra_timeout=50,
				verbose=1
			)	
	servo_A.write_byte_command(
				servo_A_id,
				0x8C,
				[100], #kd
				extra_timeout=50,
				verbose=1
			)


	servo_B.write_byte_command(
				servo_B_id,
				0x80,
				[1], #torque enable
				extra_timeout=50,
				verbose=1
			)
	servo_B.write_byte_command(
				servo_B_id,
				0x8B,
				[100], #kp
				extra_timeout=50,
				verbose=1
			)	
	servo_B.write_byte_command(
				servo_B_id,
				0x8C,
				[20], #kd
				extra_timeout=50,
				verbose=1
			)

	running = True
	while(running):

		error_A, result_A = servo_A.read_byte_command(
					servo_A_id,		# ID
					0x90, # present position from RAM
					2,	# byte number to read
					verbose=0
				)
		error_B, result_B = servo_B.read_byte_command(
				servo_B_id,		# ID
				0x90, # present position from RAM
				2,	# byte number to read
				verbose=0
			)
		
		if error_A != 0 :
			print("error_A:"+str(error_A))	
			running = False
		if error_B != 0 :
			print("error_B:"+str(error_B))
			running = False

		present_position_A 	= 0
		present_position_B = 0

		if error_A == 0 :
			present_position_A = float(sign(result_A[0] + (result_A[1]<<8))/10.0)	
		if error_B == 0 :
			present_position_B 	= float(sign(result_B[0] + (result_B[1]<<8))/10.0)

		#print(str(int(present_position_A))+" "+str(int(present_position_B)))

		fusion_position = (present_position_A+present_position_B)/2.0

		servo_B.write_word_command(
					servo_B_id,
					0x83,
					[int(fusion_position*10.0)], #pos
					extra_timeout=5,
					verbose=0
				)

		servo_A.write_word_command(
					servo_A_id,
					0x83,
					[int(fusion_position*10.0)], #pos
					extra_timeout=5,
					verbose=0
				)

		time.sleep(0.003) 

if __name__ == "__main__":

	##cProfile.run("main()")
	main()
