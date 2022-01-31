import serial
from datetime import datetime
from datetime import timedelta

# HELPERS #########################################################################################

start_time = datetime.now()

# returns the elapsed milliseconds since the start of the program
def millis():
   dt = datetime.now() - start_time
   ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
   return ms

def byte(number, i):
    return (number & (0xff << (i * 8))) >> (i * 8)

def sign(number):
	if(number & 0x8000):
		number = -0x10000 + number
	return number

def updateCRC(crc_accum, data_blk_ptr, data_blk_size):
    crc_table = [0x0000,
                 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                 0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                 0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                 0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                 0x820D, 0x8207, 0x0202]

    for j in range(0, data_blk_size):
        i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF
        crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xFFFF

    return crc_accum

# CMD #############################################################################################

class servo_protocol2:

	def __init__(self):
		self.serial = serial.Serial()
		self.timeout_input = 20

		# recv packet
		self.rx_packet = bytearray(1024)
		self.rx_position = 0
		self.rx_packet_id = 0
		self.rx_packet_payload_length = 0

	def __del__(self):
		if self.serial:
			self.serial.close()
			print("Serial "+ self.serial.port + " closed!")

	# helper

	def open(self,port,baud):
		if self.serial:
			self.serial.close()
		self.serial.baudrate = baud
		self.serial.port = port
		self.serial.open()
		if self.serial.is_open:
			print("Serial "+ self.serial.port + " openend...")

	def opened(self):
		return self.serial.is_open

	def close(self):
		self.serial.close()
		print("Serial "+ self.serial.port + " closed!")

	# base instructions 

	def ping_command(self,id):

		# flush serial input
		self.serial.reset_input_buffer()

		# build payload
		instruction_payload = bytearray()
		instruction_payload.append(0x01) # Instruction (0x01:Ping)

		# build packet and send
		self.encapsulate_and_send(id,instruction_payload)

		# receive
		rx_packet_error = self.receive_status_packet_with_timeout(id,self.timeout_input,verbose=1)

		# exit when internal error :
		if rx_packet_error<0:
			return rx_packet_error,-1,-1

		# check length
		if self.rx_packet_payload_length == 4+3: # INSTR + ERROR + PARAMS (model(2)+version(1)) + CRC1 + CRC2
			#print("rx_packet_error:"+str(rx_packet_error))
			model_number = self.rx_packet[9] + (self.rx_packet[10]<<8)
			#print("model_number:"+str(model_number))
			firmware_version = self.rx_packet[11]
			#print("firmware_version:"+str(firmware_version))
			return rx_packet_error, model_number, firmware_version
		else:
			return rx_packet_error, -1, -1


	def reboot_command(self,id):

		# flush serial input
		self.serial.reset_input_buffer()

		# build payload
		instruction_payload = bytearray()
		instruction_payload.append(0x08) # Instruction (0x01:Reboot)

		# build packet and send
		self.encapsulate_and_send(id,instruction_payload)

		# receive
		rx_packet_error = self.receive_status_packet_with_timeout(id,self.timeout_input,verbose=1)

		# exit when internal error :
		return rx_packet_error


	def read_byte_command(self,id,reg,len,verbose=0):

		# flush serial input
		self.serial.reset_input_buffer()

		# payload
		instruction_payload = bytearray()
		instruction_payload.append(0x02) # Instruction (0x02:Read)
		instruction_payload.append(byte(reg,0)) # Parameter #1 (register address low byte)			
		instruction_payload.append(byte(reg,1)) # Parameter #2 (register address high byte)
		instruction_payload.append(byte(len,0)) # Parameter #3 (length low byte)
		instruction_payload.append(byte(len,1)) # Parameter #4 (length high byte)

		# build packet and send
		self.encapsulate_and_send(id,instruction_payload)

		# receive
		rx_packet_error = self.receive_status_packet_with_timeout(id,self.timeout_input,verbose)

		# exit when internal error :
		if rx_packet_error<0:
			return rx_packet_error,[]

		# check length
		if self.rx_packet_payload_length == 4+len: # INSTR + ERROR + PARAMS (len) + CRC1 + CRC2
			return rx_packet_error, [self.rx_packet[9+i] for i in range(0,len)]
		else:
			return rx_packet_error,[]


	def read_word_command(self,id,reg,len,verbose=0):

		# flush serial input
		self.serial.reset_input_buffer()

		# payload
		instruction_payload = bytearray()
		instruction_payload.append(0x02) # Instruction (0x02:Read)
		instruction_payload.append(byte(reg,0)) # Parameter #1 (register address low byte)			
		instruction_payload.append(byte(reg,1)) # Parameter #2 (register address high byte)
		instruction_payload.append(byte(len*2,0)) # Parameter #3 (length low byte)
		instruction_payload.append(byte(len*2,1)) # Parameter #4 (length high byte)

		# build packet and send
		self.encapsulate_and_send(id,instruction_payload)

		# receive
		rx_packet_error = self.receive_status_packet_with_timeout(id,self.timeout_input,verbose)

		# exit when internal error :
		if rx_packet_error<0:
			return rx_packet_error,[]

		# check length
		if self.rx_packet_payload_length == 4+len*2: # INSTR + ERROR + PARAMS (len*2) + CRC1 + CRC2
			return rx_packet_error, [ (self.rx_packet[9+i*2]+(self.rx_packet[9+i*2+1]<<8)) for i in range(0,len)]
		else:
			return rx_packet_error,[]

	def write_byte_command(self,id,reg,values,extra_timeout=0,verbose=0):

		# flush serial input
		self.serial.reset_input_buffer()

		# payload
		instruction_payload = bytearray()
		instruction_payload.append(0x03) # Instruction (0x03:Write)
		instruction_payload.append(byte(reg,0)) # Parameter #1 (register address low byte)			
		instruction_payload.append(byte(reg,1)) # Parameter #2 (register address high byte)
		for value in values:
			instruction_payload.append(byte(value,0)) # Parameter #3 (value low byte)
		
		# build packet and send
		self.encapsulate_and_send(id,instruction_payload)

		# receive
		rx_packet_error = self.receive_status_packet_with_timeout(id,self.timeout_input+extra_timeout,verbose)

		# exit when internal error :
		return rx_packet_error


	def write_word_command(self,id,reg,values,extra_timeout=0,verbose=0):

		# flush serial input
		self.serial.reset_input_buffer()

		# payload
		instruction_payload = bytearray()
		instruction_payload.append(0x03) # Instruction (0x03:Write)
		instruction_payload.append(byte(reg,0)) # Parameter #1 (register address low byte)			
		instruction_payload.append(byte(reg,1)) # Parameter #2 (register address high byte)
		for value in values:
			instruction_payload.append(byte(value,0)) # Parameter #3 (value low byte)
			instruction_payload.append(byte(value,1)) # Parameter #4 (value high byte)
		
		# build packet and send
		self.encapsulate_and_send(id,instruction_payload)

		# receive
		rx_packet_error = self.receive_status_packet_with_timeout(id,self.timeout_input+extra_timeout,verbose)

		# exit when internal error :
		return rx_packet_error

	def sync_write_byte_command(self,reg,values):

		# payload
		instruction_payload = bytearray()
		instruction_payload.append(0x83) # Instruction (0x03:Write)
		instruction_payload.append(byte(reg,0)) # Parameter #1 (register address low byte)			
		instruction_payload.append(byte(reg,1)) # Parameter #2 (register address high byte)
		instruction_payload.append(byte(len(values[0])-1,0)) # Parameter #3 (register address low byte)			
		instruction_payload.append(byte(len(values[0])-1,1)) # Parameter #4 (register address high byte)
		for line in values:
			instruction_payload.append(byte(line[0],0)) # Parameter #5 (ID low byte)
			for index in range(1,len(line)):
				instruction_payload.append(byte(line[index],0)) # Parameter #7... (value low byte)
		
		# build packet and send
		self.encapsulate_and_send(0xFE,instruction_payload)


	def sync_write_word_command(self,reg,values):

		# payload
		instruction_payload = bytearray()
		instruction_payload.append(0x83) # Instruction (0x03:Write)
		instruction_payload.append(byte(reg,0)) # Parameter #1 (register address low byte)			
		instruction_payload.append(byte(reg,1)) # Parameter #2 (register address high byte)
		instruction_payload.append(byte(2*(len(values[0])-1),0)) # Parameter #3 (register address low byte)			
		instruction_payload.append(byte(2*(len(values[0])-1),1)) # Parameter #4 (register address high byte)
		for line in values:
			instruction_payload.append(byte(line[0],0)) # Parameter #5 (ID low byte)
			for index in range(1,len(line)):
				instruction_payload.append(byte(line[index],0)) # Parameter #7 (value low byte)
				instruction_payload.append(byte(line[index],1)) # Parameter #8... (value high byte)
		
		# build packet and send
		self.encapsulate_and_send(0xFE,instruction_payload)


	# extended instructions #######################################################################

	
	def ping(self,id,verbose=0):
		if verbose == 1:
			print("Ping ID"+str(id)+"...")
		error, model_number, firmware_version = self.ping_command(id)
		if verbose == 1:
			if error == 0:
				print("model_number:"+str(model_number))
				print("firmware_version:"+str(firmware_version))
				print("   OK!")
			else:
				print("   Error:"+str(error))
			print('')

	def torque_enable(self,id,verbose=0):
		if verbose == 1:
			print("Enable torque ID"+str(id)+"...")
		error = self.write_byte_command(id,0x40,[1],verbose)
		if verbose == 1:
			if error == 0:
				print("   OK!")
			else:
				print("   Error:"+str(error))
			print('')

	def torque_disable(self,id,verbose=0):
		if verbose == 1:
			print("Disable torque ID"+str(id)+"...")
		error = self.write_byte_command(id,0x40,[0],verbose)
		if verbose == 1:
			if error == 0:
				print("   OK!")
			else:
				print("   Error:"+str(error))
			print('')

	def get_position(self,id,verbose=0):
		error,result = self.read_word_command(id,0x4D,1,verbose)
		if verbose == 1:
			if error == 0:
				print("Position ID"+str(id)+":"+str(result[0]/10.0))
			else:
				print("Position ID"+str(id)+" Error:"+str(error)+"!")
			print('')
		if error == 0:
			return result[0]/10.0		
		else:
			return error

	def get_current(self,id,verbose=0):
		error,result = self.read_word_command(id,0x51,1,verbose)
		if verbose == 1:
			if error == 0:
				print("Current ID"+str(id)+":"+str(sign(result[0])))
			else:
				print("Current ID"+str(id)+" Error:"+str(error)+"!")
			print('')
		if error == 0:
			return sign(result[0])
		else:
			return error

	def set_position(self,id,position,verbose=0):
		error = self.write_word_command(id,0x43,[int(position*10.0)],verbose)
		if verbose == 1:
			if error == 0:
				print("Position target ID"+str(id)+":"+str(position))
			else:
				print("Position target ID"+str(id)+" Error:"+str(error)+"!")
			print('')
		return error

	def set_position_velocity_current_pwm(self,id,position,velocity,current,pwm,verbose=0):
		error = self.write_word_command(id,0x43,[int(position*10.0),velocity,current,pwm],verbose)
		if verbose == 1:
			if error == 0:
				print("Position target ID"+str(id)+":"+str(position))
				print("Velocity limit ID"+str(id)+":"+str(velocity))
				print("Current limit ID"+str(id)+":"+str(current))
				print("PWM limit ID"+str(id)+":"+str(pwm))
			else:
				print("Position target ID"+str(id)+" Error:"+str(error)+"!")
				print("Velocity limit ID"+str(id)+" Error:"+str(error)+"!")
				print("Current limit ID"+str(id)+" Error:"+str(error)+"!")
				print("PWM limit ID"+str(id)+":"+" Error:"+str(error)+"!")
			print('')
		return error

	# low level functions

	def encapsulate_and_send(self,id,instruction_payload):

		# packet
		packet = bytearray()
		# header
		packet.append(0xFF) # Header 1
		packet.append(0xFF) # Header 2
		packet.append(0xFD) # Header 3
		packet.append(0x00) # Reserved
		# destination
		packet.append(id) # ID (destination : 0-252 for unicast, 254 for broadcast)
		# length
		packet_length = len(instruction_payload) + 2 # + 16-bit CRC
		packet.append( byte(packet_length,0) )
		packet.append( byte(packet_length,1) )
		# insert instruction packet
		for i in instruction_payload:
			packet.append(i) 
		# add CRC16
		packet_crc = updateCRC(0, packet, len(packet)) 
		packet.append( byte(packet_crc,0) ) # CRC 1
		packet.append( byte(packet_crc,1) ) # CRC 2
		# send
		self.serial.write(packet)	
		self.serial.flush()
		##print("sending>" + ":".join("{:02x}".format(c) for c in packet))


	def receive_status_packet_with_timeout(self,id,timeout,verbose=0):
		# wait for serial input
		start_time = millis()
		while(millis()<start_time+timeout):
			##print('.')
			# receive data
			read_size = self.serial.in_waiting
			if read_size>0 :
				read_data = self.serial.read(read_size)
				#print("receiving>" + ":".join("{:02x}".format(c) for c in read_data))
				for byte in read_data:
					if self.decode(byte):
						# one packet received
						# check ID
						#print("rx_packet_id:"+str(self.rx_packet_id))
						if self.rx_packet_id == id or id == 0xFE: 
							# good ID
							if verbose==1:
								print("reply delay:"+str(millis()-start_time)+"ms")
							# check length and instruction (return)
							#print("rx_packet_payload_length:"+str(self.rx_packet_payload_length))
							rx_packet_instruction = self.rx_packet[7]
							#print("rx_packet_instruction:"+str(rx_packet_instruction))
							if rx_packet_instruction == 0x55:
								# good instruction (return)
								# check error and extract parameters
								rx_packet_error = self.rx_packet[8]
								return rx_packet_error

							else:
								print("[PROTOCOL] ERROR : Received an unexpected packet!")
								return -1

						else:
							print("[PROTOCOL] ERROR : Received packet from another ID!")
							return -1
		
		print("[PROTOCOL] ERROR : Time-out!")
		return -2


	def decode(self,byte):
		# HEADER 1
		if self.rx_position == 0: 
			if byte == 0xFF:
				self.rx_packet[self.rx_position]=byte
				self.rx_position += 1
				return False
			else:
				return False
		# HEADER 2
		if self.rx_position == 1: 
			#print("[PROTOCOL] NOTICE : HDR2 ("+ str(byte)+")")
			if byte == 0xFF:
				self.rx_packet[self.rx_position]=byte
				self.rx_position += 1
				return False
			else:
				self.rx_position = 0
				return False
		# HEADER 3
		if self.rx_position == 2: 
			if byte == 0xFD:
				self.rx_packet[self.rx_position]=byte
				self.rx_position += 1
				return False
			else:
				self.rx_position = 0
				return False
		# RESERVED
		if self.rx_position == 3: 
			if byte == 0x00:
				self.rx_packet[self.rx_position]=byte
				self.rx_position += 1
				return False
			else:
				self.rx_position = 0
				return False
		# ID
		if self.rx_position == 4: 
			if byte <= 252 or byte == 254:
				self.rx_packet[self.rx_position]=byte
				self.rx_packet_id = byte
				self.rx_position += 1
				return False
			else:
				self.rx_position = 0
				return False
		# LENGTH 1
		if self.rx_position == 5: 
			self.rx_packet[self.rx_position]=byte
			self.rx_position += 1
			return False
		# LENGTH 2
		if self.rx_position == 6: 
			self.rx_packet[self.rx_position]=byte
			self.rx_position += 1	
			self.rx_packet_payload_length = ((self.rx_packet[self.rx_position-1]<<8)&0xFF00) + (self.rx_packet[self.rx_position-2]&0xFF)
			#print("[PROTOCOL] NOTICE : rx_packet_payload_length:("+ str(rx_packet_payload_length)+")")
			if self.rx_packet_payload_length>1024:
				print("[PROTOCOL] ERROR : Received packet too long!")
				self.rx_position = 0
				return False
			else:
				return False
		# PAYLOAD
		if self.rx_position <= 6+self.rx_packet_payload_length-2:
			self.rx_packet[self.rx_position]=byte
			self.rx_position += 1	
			return False
		# CRC 1
		if self.rx_position == 6+self.rx_packet_payload_length-1:
			self.rx_packet[self.rx_position]=byte
			self.rx_position += 1	
			return False
		# CRC 2
		if self.rx_position == 6+self.rx_packet_payload_length:
			self.rx_packet[self.rx_position]=byte
			self.rx_position += 1	
			rx_packet_crc = (self.rx_packet[self.rx_position-1]<<8) + self.rx_packet[self.rx_position-2]
			calculated_crc = updateCRC(0, self.rx_packet, 6+self.rx_packet_payload_length-1)
			#print("[PROTOCOL] NOTICE : rx_packet_crc:("+ str(hex(rx_packet_crc))+")")
			#print("[PROTOCOL] NOTICE : calculated_crc:("+ str(hex(calculated_crc))+")")
			if rx_packet_crc == calculated_crc:
				#print("[PROTOCOL] NOTICE : Received packet (" + str(rx_packet_payload_length) + ")!")
				self.rx_position = 0
				return True
			else:
				print("[PROTOCOL] ERROR : CRC check FAILED!")
				self.rx_position = 0
				return False


