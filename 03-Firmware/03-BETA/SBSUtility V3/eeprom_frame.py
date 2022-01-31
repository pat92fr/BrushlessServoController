
## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
import time
from protocol2 import byte
from protocol2 import sign

class eeprom_frame(LabelFrame):

	def __init__(self,window,protocol,id):
		super().__init__(text="EEPROM")
		self.protocol = protocol
		self.id = id
		#self["width"]=200
		#self["height"]=1000
		self.labels = {}
		self.entries = {}
		self.variables = {}
		self.row = 0

		
		self.gui_spacer("")
		self.gui_entry("Model Number", "model_number", 0, False, True, False, 0x01, 1, 2 )
		self.gui_entry("Version", "version", 0, False, True, False, 0x02, 1, 1 )
		self.gui_entry("ID", "id", 0, True, True, True, 0x03, 1, 1 )
		self.gui_entry("Baud Rate", "baud_rate", 0, True, True, True, 0x04, 1, 1 )
		#self.gui_entry("Return Delay", "return_delay", 0, True, True, True, 0x05, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Min Position", "min_position", 0, True, True, True, 0x10, 1, 2 )
		self.gui_entry("Max Position", "max_position", 0, True, True, True, 0x12, 1, 2 )
		self.gui_entry("Max Velocity", "max_velocity", 0, True, True, True, 0x14, 1, 2 )
		#self.gui_entry("Max Acceleration", "max_acceleration", 0, True, True, True, 0x16, 1, 2 )
		self.gui_entry("Max Current", "max_current", 0, True, True, True, 0x18, 1, 2 )
		self.gui_entry("Max Temperature", "max_temperature", 0, True, True, True, 0x1C, 1, 1 )
		self.gui_entry("Min Voltage", "min_voltage", 0, True, True, True, 0x1D, 1, 1 )
		self.gui_entry("Max Voltage", "max_voltage", 0, True, True, True, 0x1E, 1, 1 )
		self.gui_spacer("---")
		#self.gui_entry("Moving Threshold", "moving_threshold", 0, True, True, True, 0x1F, 1, 1 )
		#self.gui_entry("Status Return lvl", "status_return_level", 0, True, True, True, 0x20, 1, 1 )
		#self.gui_entry("Alarm Led", "alarm_led", 0, True, True, True, 0x21, 1, 1 )
		#self.gui_entry("Alarm Shutdown", "alarm_shutdown", 0, True, True, True, 0x22, 1, 1 )
		#self.gui_spacer("---")
		#(self.gui_entry("Encoder Resolution Bits", "encoder_bits", 0, True, True, True, 0x23, 1, 2 )
		self.gui_entry("Motor Pole Pairs", "motor_pole_pairs", 0, True, True, True, 0x24, 1, 1 )
		self.gui_entry("Motor Synchro Angle", "motor_synchro", 0, True, True, True, 0x25, 1, 2 )
		self.gui_entry("Motor Reverse Phase", "inv_phase_motor", 0, True, True, True, 0x28, 1, 1 )
		#self.gui_entry("Field Weakening K", "field_weaknening_k", 0, True, True, True, 0x29, 1, 1 )
		self.gui_spacer("---")
		#self.gui_entry("PID Position KP", "pid_position_kp", 0, True, True, True, 0x2A, 1, 2 )
		#self.gui_entry("PID Position KI", "pid_position_ki", 0, True, True, True, 0x2C, 1, 2 )
		#self.gui_entry("PID Position KD", "pid_position_kd", 0, True, True, True, 0x2E, 1, 2 )
		#self.gui_entry("PID Velocity KP", "pid_velocity_kp", 0, True, True, True, 0x30, 1, 2 )
		#self.gui_entry("PID Velocity KI", "pid_velocity_ki", 0, True, True, True, 0x32, 1, 2 )
		#self.gui_entry("PID Velocity KD", "pid_velocity_kd", 0, True, True, True, 0x34, 1, 2 )
		#self.gui_entry("PID Velocity KFF", "pid_velocity_kff", 0, True, True, True, 0x36, 1, 2 )
		#self.gui_entry("PID Acceleration KFF", "pid_acceleration_kff", 0, True, True, True, 0x38, 1, 2 )
		#self.gui_spacer("---")
		self.gui_entry("PI Flux Current KP", "pid_flux_current_kp", 0, True, True, True, 0x3A, 1, 2 )
		self.gui_entry("PI Flux Current KI", "pid_flux_current_ki", 0, True, True, True, 0x3C, 1, 2 )
		#self.gui_entry("PID Flux Current KFF", "pid_flux_current_kff", 0, True, True, True, 0x3E, 1, 2 )
		#self.gui_spacer("---")
		self.gui_entry("PI Torque Current KP", "pid_torque_current_kp", 0, True, True, True, 0x40, 1, 2 )
		self.gui_entry("PI Torque Current KI", "pid_torque_current_ki", 0, True, True, True, 0x42, 1, 2 )
		#self.gui_entry("PID Torque Current KFF", "pid_torque_current_kff", 0, True, True, True, 0x44, 1, 2 )
		#self.gui_spacer("---")
		#self.gui_entry("PH1 Current Sense MA", "cal_phase1_current_sense_ma", 0, True, True, True, 0x46, 1, 2 )
		#self.gui_entry("Ph1 Current Sense Offset", "cal_phase1_current_sense_offset", 0, True, True, True, 0x48, 1, 2 )
		#self.gui_entry("PH2 Current Sense MA", "cal_phase2_current_sense_ma", 0, True, True, True, 0x4A, 1, 2 )
		#self.gui_entry("Ph2 Current Sense Offset", "cal_phase2_current_sense_offset", 0, True, True, True, 0x4C, 1, 2 )
		#self.gui_entry("PH3 Current Sense MA", "cal_phase3_current_sense_ma", 0, True, True, True, 0x4E, 1, 2 )
		#self.gui_entry("Ph3 Current Sense Offset", "cal_phase3_current_sense_offset", 0, True, True, True, 0x50, 1, 2 )
		#self.gui_entry("Calibration Voltage sensor", "cal_voltage_sensor", 0, True, True, True, 0x52, 1, 2 )
		#self.gui_entry("EWMA encoder", "ewma_encoder", 0, True, True, True, 0x54, 1, 1 )

		# update button
		button_update = Button(self,text="Update",command = self.read_all)
		button_update.grid(column = 2, row = 0, sticky='we')

		self.read_all()

	def gui_entry(self,text_label,variable_name,variable_value,has_local,has_servo,has_callback,callback_reg_address,callback_reg_scale,callback_reg_size):
		self.labels[variable_name] = Label(self, text = text_label, anchor="w", justify=LEFT)
		self.labels[variable_name].grid(column = 0, row = self.row, sticky='w')
		if has_local:
			self.variables[variable_name+"_local"] = StringVar()
			self.variables[variable_name+"_local"].set(str(variable_value))
		if has_servo:
			self.variables[variable_name+"_servo"] = StringVar()
			self.variables[variable_name+"_servo"].set("empty")
		if has_local:
			self.entries[variable_name+"_local"] = Entry(self, width = 15, textvariable = self.variables[variable_name+"_local"]) 
			self.entries[variable_name+"_local"].grid(column = 1, row = self.row)
		if has_servo:
			self.entries[variable_name+"_servo"] = Entry(self, width = 15, state="readonly", textvariable = self.variables[variable_name+"_servo"]) 
			self.entries[variable_name+"_servo"].grid(column = 2, row = self.row)
		if has_local and has_callback:
			self.entries[variable_name+"_local"].bind('<Return>', (lambda _: self.callback_entry(variable_name,callback_reg_address,callback_reg_scale,callback_reg_size)))
		self.row += 1

	def callback_entry(self,variable_name,callback_reg_address,callback_reg_scale,callback_reg_size):
		print("set " + variable_name + ":"+self.variables[variable_name+"_local"].get())
		print("write EEPROM...")
		if callback_reg_size==1:
			self.protocol.write_byte_command(
				self.id.current_id,
				callback_reg_address,
				[
					callback_reg_scale*int(self.variables[variable_name+"_local"].get())
				],
				extra_timeout=50,
				verbose=1
			)
		if callback_reg_size==2:
			self.protocol.write_word_command(
				self.id.current_id,
				callback_reg_address,
				[
					callback_reg_scale*int(self.variables[variable_name+"_local"].get())
				],
				extra_timeout=50,
				verbose=1
			)


	def gui_spacer(self,text_label):
		label = Label(self, text = text_label, anchor="w", justify=LEFT)
		label.grid(column = 0, row = self.row, sticky='w')
		self.row += 1		

	def read_all(self):
		if self.protocol:
			print("real all EEPROM...")
			# send read command
			error, result = self.protocol.read_byte_command(
				self.id.current_id,		# ID
				0x00, # from EEPROM
				85,	# byte number to read
				verbose=1
			) # TODO change ID through GUI
			if error != 0 :
				print("error:"+str(error))
			elif len(result)==85:
				self.variables['model_number_servo'].set(str(result[0] + (result[1]<<8)))
				self.variables['version_servo'].set(str(result[2]))
				self.variables['id_local'].set(str(result[3]))
				self.variables['id_servo'].set(str(result[3]))
				self.variables['baud_rate_local'].set(str(result[4]))
				self.variables['baud_rate_servo'].set(str(result[4]))
				#self.variables['return_delay_local'].set(str(result[5]))
				#self.variables['return_delay_servo'].set(str(result[5]))
				self.variables['min_position_local'].set(str(sign(result[16] + (result[17]<<8))))
				self.variables['min_position_servo'].set(str(sign(result[16] + (result[17]<<8))))
				self.variables['max_position_local'].set(str(sign(result[18] + (result[19]<<8))))
				self.variables['max_position_servo'].set(str(sign(result[18] + (result[19]<<8))))
				self.variables['max_velocity_local'].set(str(result[20] + (result[21]<<8)))
				self.variables['max_velocity_servo'].set(str(result[20] + (result[21]<<8)))
				#self.variables['max_acceleration_local'].set(str(result[22] + (result[23]<<8)))
				#self.variables['max_acceleration_servo'].set(str(result[22] + (result[23]<<8)))
				self.variables['max_current_local'].set(str(result[24] + (result[25]<<8)))
				self.variables['max_current_servo'].set(str(result[24] + (result[25]<<8)))
				self.variables['max_temperature_local'].set(str(result[28]))
				self.variables['max_temperature_servo'].set(str(result[28]))
				self.variables['min_voltage_local'].set(str(result[29]))
				self.variables['min_voltage_servo'].set(str(result[29]))
				self.variables['max_voltage_local'].set(str(result[30]))
				self.variables['max_voltage_servo'].set(str(result[30]))
				#self.variables['moving_threshold_local'].set(str(result[31]))
				#self.variables['moving_threshold_servo'].set(str(result[31]))
				#self.variables['status_return_level_local'].set(str(result[32]))
				#self.variables['status_return_level_servo'].set(str(result[32]))
				#self.variables['alarm_led_local'].set(str(result[33]))
				#self.variables['alarm_led_servo'].set(str(result[33]))
				#self.variables['alarm_shutdown_local'].set(str(result[34]))
				#self.variables['alarm_shutdown_servo'].set(str(result[34]))

				#self.variables['encoder_bits_local'].set(str(result[35]))
				#self.variables['encoder_bits_servo'].set(str(result[35]))
				self.variables['motor_pole_pairs_local'].set(str(result[36]))
				self.variables['motor_pole_pairs_servo'].set(str(result[36]))
				self.variables['motor_synchro_local'].set(str(result[37] + (result[38]<<8)))
				self.variables['motor_synchro_servo'].set(str(result[37] + (result[38]<<8)))
				self.variables['inv_phase_motor_local'].set(str(result[40]))
				self.variables['inv_phase_motor_servo'].set(str(result[40]))
				#self.variables['field_weaknening_k_local'].set(str(result[41]))
				#self.variables['field_weaknening_k_servo'].set(str(result[41]))

				#self.variables['pid_position_kp_local'].set(str(result[42] + (result[43]<<8)))
				#self.variables['pid_position_kp_servo'].set(str(result[42] + (result[43]<<8)))
				#self.variables['pid_position_ki_local'].set(str(result[44] + (result[45]<<8)))
				#self.variables['pid_position_ki_servo'].set(str(result[44] + (result[45]<<8)))
				#self.variables['pid_position_kd_local'].set(str(result[46] + (result[47]<<8)))
				#self.variables['pid_position_kd_servo'].set(str(result[46] + (result[47]<<8)))

				#self.variables['pid_velocity_kp_local'].set(str(result[48] + (result[49]<<8)))
				#self.variables['pid_velocity_kp_servo'].set(str(result[48] + (result[49]<<8)))
				#self.variables['pid_velocity_ki_local'].set(str(result[50] + (result[51]<<8)))
				#self.variables['pid_velocity_ki_servo'].set(str(result[50] + (result[51]<<8)))
				#self.variables['pid_velocity_kd_local'].set(str(result[52] + (result[53]<<8)))
				#self.variables['pid_velocity_kd_servo'].set(str(result[52] + (result[53]<<8)))
				#self.variables['pid_velocity_kff_local'].set(str(result[54] + (result[55]<<8)))
				#self.variables['pid_velocity_kff_servo'].set(str(result[54] + (result[55]<<8)))
				#self.variables['pid_acceleration_kff_local'].set(str(result[56] + (result[57]<<8)))
				#self.variables['pid_acceleration_kff_servo'].set(str(result[56] + (result[57]<<8)))

				self.variables['pid_flux_current_kp_local'].set(str(result[58] + (result[59]<<8)))
				self.variables['pid_flux_current_kp_servo'].set(str(result[58] + (result[59]<<8)))
				self.variables['pid_flux_current_ki_local'].set(str(result[60] + (result[61]<<8)))
				self.variables['pid_flux_current_ki_servo'].set(str(result[60] + (result[61]<<8)))
				#self.variables['pid_flux_current_kff_local'].set(str(result[62] + (result[63]<<8)))
				#self.variables['pid_flux_current_kff_servo'].set(str(result[62] + (result[63]<<8)))

				self.variables['pid_torque_current_kp_local'].set(str(result[64] + (result[65]<<8)))
				self.variables['pid_torque_current_kp_servo'].set(str(result[64] + (result[65]<<8)))
				self.variables['pid_torque_current_ki_local'].set(str(result[66] + (result[67]<<8)))
				self.variables['pid_torque_current_ki_servo'].set(str(result[66] + (result[67]<<8)))
				#self.variables['pid_torque_current_kff_local'].set(str(result[68] + (result[69]<<8)))
				#self.variables['pid_torque_current_kff_servo'].set(str(result[68] + (result[69]<<8)))

				#self.variables['cal_phase1_current_sense_ma_local'].set(str(result[70] + (result[71]<<8)))
				#self.variables['cal_phase1_current_sense_ma_servo'].set(str(result[70] + (result[71]<<8)))
				#self.variables['cal_phase1_current_sense_offset_local'].set(str(result[72] + (result[73]<<8)))
				#self.variables['cal_phase1_current_sense_offset_servo'].set(str(result[72] + (result[73]<<8)))

				#self.variables['cal_phase2_current_sense_ma_local'].set(str(result[74] + (result[75]<<8)))
				#self.variables['cal_phase2_current_sense_ma_servo'].set(str(result[74] + (result[75]<<8)))
				#self.variables['cal_phase2_current_sense_offset_local'].set(str(result[76] + (result[77]<<8)))
				#self.variables['cal_phase2_current_sense_offset_servo'].set(str(result[76] + (result[77]<<8)))

				#self.variables['cal_phase3_current_sense_ma_local'].set(str(result[78] + (result[79]<<8)))
				#self.variables['cal_phase3_current_sense_ma_servo'].set(str(result[78] + (result[79]<<8)))
				#self.variables['cal_phase3_current_sense_offset_local'].set(str(result[80] + (result[81]<<8)))
				#self.variables['cal_phase3_current_sense_offset_servo'].set(str(result[80] + (result[81]<<8)))

				#self.variables['cal_voltage_sensor_local'].set(str(result[82] + (result[83]<<8)))
				#self.variables['cal_voltage_sensor_servo'].set(str(result[82] + (result[83]<<8)))


				#self.variables['ewma_encoder_local'].set(str(result[84]))
				#self.variables['ewma_encoder_servo'].set(str(result[84]))
