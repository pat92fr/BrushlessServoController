
## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
from protocol2 import sign
import time

class ram_frame(LabelFrame):

	def __init__(self,window,protocol,trace,id):
		super().__init__(text="RAM")
		self.protocol = protocol
		self.trace = trace
		self.id = id
		#self["width"]=200
		#self["height"]=1000
		self.labels = {}
		self.entries = {}
		self.variables = {}
		self.row = 0
		self.data_ready = 0
		self.counter = 0
		self.start_time = time.time()*1000.0

		self.gui_spacer("")
		self.gui_entry("Torque Enable", "torque_enable", 0, False, True, False, 0x80, 1, 1 )
		self.gui_entry("LED", "led", 0, True, True, True, 0x81, 1, 1 )
		self.gui_entry("Control Mode", "control_mode", 0, True, True, True, 0x82, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Goal Position", "goal_position", 0, True, True, True, 0x83, 10, 2 )
		self.gui_entry("Goal Velocity", "goal_velocity", 0, True, True, True, 0x85, 1, 2 )
		self.gui_entry("Goal Torque Current", "goal_torque_current", 0, True, True, True, 0x87, 1, 2 )
		self.gui_entry("Goal Flux Current", "goal_flux_current", 0, True, True, True, 0x89, 1, 2 )
		self.gui_spacer("---")
		self.gui_entry("Manual Synchro Offset", "goal_synchro_offset", 0, True, True, True, 0x8D, 1, 2 )
		self.gui_entry("Force Open Loop", "goal_open_loop", 0, True, True, True, 0x8F, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Present Position", "present_position", 0, False, True, False, 0x90, 10, 2 )
		self.gui_entry("Present Velocity", "present_velocity", 0, False, True, False, 0x92, 1, 2 )
		self.gui_entry("Present Torque Current", "present_torque_current", 0, False, True, False, 0x94, 1, 2 )
		self.gui_entry("Present Flux Current", "present_flux_current", 0, False, True, False, 0x96, 1, 2 )
		self.gui_entry("Present Voltage", "present_voltage", 0, False, True, False, 0x98, 1, 1 )
		self.gui_entry("Present Temperature", "present_temperature", 0, False, True, False, 0x99, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Moving", "moving", 0, False, True, False, 0x9A, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Setpoint Position", "setpoint_position", 0, False, True, False, 0x100, 10, 2 )
		self.gui_entry("Setpoint Velocity", "setpoint_velocity", 0, False, True, False, 0x102, 1, 2 )
		self.gui_entry("Setpoint Torque Current", "setpoint_torque_current", 0, False, True, False, 0x104, 1, 2 )
		self.gui_entry("Setpoint Flux Current", "setpoint_flux_current", 0, False, True, False, 0x106, 1, 2 )
		self.gui_spacer("---")
		self.gui_entry("FOC ProcessingTime", "processing_time", 0, False, True, False, 0x10A, 1, 1 )
		self.gui_spacer("---")
		self.gui_entry("Protocol CRC Fail", "protocol_crc_fail", 0, False, True, False, 0x110, 1, 1 )
		self.gui_entry("Hardware Error Status", "hardware_error_status", 0, False, True, False, 0x111, 1, 1 )

		# update button
		#button_update = Button(self,text="Update",command = self.read_all)
		#button_update.grid(column = 2, row = 0, sticky='we')

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
		#self.protocol.write_command(1,callback_reg_address,callback_reg_scale*int(self.variables[variable_name+"_local"].get()),callback_reg_size)
		print("write RAM...")
		if callback_reg_size==1:
			self.protocol.write_byte_command(
				self.id.current_id,
				callback_reg_address,
				[
					callback_reg_scale*int(self.variables[variable_name+"_local"].get())
				],
				extra_timeout=50
			)
		if callback_reg_size==2:
			self.protocol.write_word_command(
				self.id.current_id,
				callback_reg_address,
				[
					callback_reg_scale*int(self.variables[variable_name+"_local"].get())
				],
				extra_timeout=50
			)

	def gui_spacer(self,text_label):
		label = Label(self, text = text_label, anchor="w", justify=LEFT)
		label.grid(column = 0, row = self.row, sticky='w')
		self.row += 1		

	def read_all(self):
		# write test
		if self.trace.variables["square_current"].get() == 1:
			value = self.trace.test_square_current()
			if  value != 0:
				print("write RAM...")
				self.protocol.write_word_command(self.id.current_id,0x47,[value],verbose=1)
		# write test
		elif self.trace.variables["square_position"].get() == 1:
			value = self.trace.test_square_position()
			if  value != 0:
				print("write RAM...")
				self.protocol.write_word_command(self.id.current_id,0x83,[value],verbose=1)
		elif self.trace.variables["triangle_position"].get() == 1:
			value = self.trace.test_triangle_position()
			if  value != 0:
				print("write RAM...")
				self.protocol.write_word_command(self.id.current_id,0x83,[value],verbose=1)
		elif self.trace.variables["round_position"].get() == 1:
			update, servo_angles = self.trace.test_round_position()
			if  update:
				#print("write RAM...")
				self.protocol.write_word_command(2,0x43,[int((servo_angles[0,0]+45.0)*10.0)],verbose=0)
				self.protocol.write_word_command(1,0x43,[int((135.0-servo_angles[1,0])*10.0)],verbose=0)


		# send read command
		if (self.counter%100)==0:
			verb = 1
			end_time = time.time()*1000.0
			print("delay for 100 iterations:" + str(end_time-self.start_time) + "ms")
			self.start_time = end_time
		else:
			verb = 0
		error, result = self.protocol.read_byte_command(
			self.id.current_id,		# ID
			0x80, # from EEPROM
			50,	# byte number to read
			verbose=verb
		) # TODO change ID through GUI


		if error != 0 :
			print("error:"+str(error))
		elif len(result)==50:
			goal_position 		= float(sign(result[3] + (result[4]<<8))/10.0)
			setpoint_position 	= float(sign(result[32] + (result[33]<<8))/10.0)
			present_position 	= float(sign(result[16] + (result[17]<<8))/10.0)
			goal_velocity 		= float(sign(result[5] + (result[6]<<8)))
			setpoint_velocity 	= float(sign(result[34] + (result[35]<<8)))
			present_velocity 	= float(sign(result[18] + (result[19]<<8)))
			goal_torque_current 	= float(sign(result[7] + (result[8]<<8)))
			setpoint_torque_current = float(sign(result[36] + (result[37]<<8)))
			present_torque_current 	= float(sign(result[20] + (result[21]<<8)))
			goal_flux_current 		= float(sign( result[9] + (result[10]<<8)))
			setpoint_flux_current 	= float(sign( result[38] + (result[39]<<8)))
			present_flux_current 	= float(sign( result[22] + (result[23]<<8)))
			goal_synchro_offset		= float(sign( result[13] + (result[14]<<8)))

			self.trace.update(
				goal_position,
				setpoint_position,
				present_position,
				goal_velocity,
				setpoint_velocity,
				present_velocity,
				goal_torque_current,
				setpoint_torque_current,
				present_torque_current,
				goal_flux_current,
				setpoint_flux_current,
				present_flux_current
			)

			if self.counter == 0:
				self.variables['led_local'].set(str(result[1]))
				self.variables['control_mode_local'].set(str(result[2]))
				self.variables['goal_position_local'].set(str( goal_position ))
				self.variables['goal_velocity_local'].set(str( goal_velocity ))
				self.variables['goal_torque_current_local'].set(str( goal_torque_current ))
				self.variables['goal_flux_current_local'].set(str( goal_flux_current ))
				self.variables['goal_synchro_offset_local'].set(str( goal_synchro_offset ))
				self.variables['goal_open_loop_local'].set(str( result[15] ))

			self.variables['torque_enable_servo'].set(str(result[0]))
			self.variables['led_servo'].set(str(result[1]))
			self.variables['control_mode_servo'].set(str(result[2]))

			self.variables['goal_position_servo'].set(str( goal_position ))
			self.variables['goal_velocity_servo'].set(str( goal_velocity ))
			self.variables['goal_torque_current_servo'].set(str( goal_torque_current ))
			self.variables['goal_flux_current_servo'].set(str( goal_flux_current ))
			self.variables['goal_synchro_offset_servo'].set(str( goal_synchro_offset ))
			self.variables['goal_open_loop_servo'].set(str( result[15] ))

			self.variables['present_position_servo'].set(str( present_position ))
			self.variables['present_velocity_servo'].set(str( present_velocity ))
			self.variables['present_torque_current_servo'].set(str( present_torque_current ))
			self.variables['present_flux_current_servo'].set(str( present_flux_current ))
			self.variables['present_voltage_servo'].set(str(result[24]))
			self.variables['present_temperature_servo'].set(str(result[25]))
			self.variables['moving_servo'].set(str(result[26]))
			self.variables['setpoint_position_servo'].set(str( setpoint_position ))
			self.variables['setpoint_velocity_servo'].set(str( setpoint_velocity ))
			self.variables['setpoint_torque_current_servo'].set(str( setpoint_torque_current ))
			self.variables['setpoint_flux_current_servo'].set(str( setpoint_flux_current ))
			self.variables['processing_time_servo'].set(str(result[42]))

			self.variables['protocol_crc_fail_servo'].set(str(result[48]))
			self.variables['hardware_error_status_servo'].set(str(result[49]))
			self.data_ready = 1


		self.counter += 1
		self.after(1,self.read_all)