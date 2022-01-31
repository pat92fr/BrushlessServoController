
## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
import time
from trace_frame import *
import math

class trace_frame(LabelFrame):

	def __init__(self,window,protocol,eeprom,id):
		super().__init__(text="TRACE")
		self.protocol = protocol
		self.eeprom = eeprom
		self.id = id
		self["width"]=200
		self["height"]=1000
		self.labels = {}
		self.entries = {}
		self.variables = {}
		self.checks = {}
		self.viewports = {}

		self.test_timer = 0
		self.test_square_value = 0

		self.columnconfigure(0, weight=1)

		fa = LabelFrame(self,text="Actions")
		fa.grid(column = 0, row = 0, sticky='nesw')
		fa.columnconfigure(2, weight=1)

		self.variables["square_position"] = IntVar()
		self.variables["square_position"].set(0)		
		self.checks['square_position'] = Checkbutton(fa,text="Square position Test", variable=self.variables["square_position"])
		self.checks['square_position'].grid(column = 0, row = 0, sticky='nw')

		self.variables["triangle_position"] = IntVar()
		self.variables["triangle_position"].set(0)		
		self.checks['triangle_position'] = Checkbutton(fa,text="Triangle position Test", variable=self.variables["triangle_position"])
		self.checks['triangle_position'].grid(column = 1, row = 0, sticky='nw')

		self.variables["sinus_position"] = IntVar()
		self.variables["sinus_position"].set(0)		
		self.checks['sinus_position'] = Checkbutton(fa,text="Sinus position Test", variable=self.variables["sinus_position"])
		self.checks['sinus_position'].grid(column = 2, row = 0, sticky='nw')
		
		fp = LabelFrame(self,text="Position Control")
		fp.grid(column = 0, row = 1, sticky='nesw')
		fp.columnconfigure(0, weight=1)
		self.viewports['position'] = Canvas(fp, bg="#FFFFFF") #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['position'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_position'] = IntVar()
		self.variables['goal_position'].set(1)
		self.checks['goal_position']= Checkbutton(fp,text="Goal Position",variable=self.variables['goal_position'])
		self.checks['goal_position'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_position'] = IntVar()
		self.variables['setpoint_position'].set(1)
		self.checks['setpoint_position']= Checkbutton(fp,text="Setpoint Position",variable=self.variables['setpoint_position'])
		self.checks['setpoint_position'].grid(column = 4, row = 2, sticky='w')
		self.variables['present_position'] = IntVar()
		self.variables['present_position'].set(1)		
		self.checks['present_position']= Checkbutton(fp,text="Present Position",variable=self.variables['present_position'])
		self.checks['present_position'].grid(column = 4, row = 3, sticky='w')


		fv = LabelFrame(self,text="Velocity Control")
		fv.grid(column = 0, row = 2, sticky='nesw')
		fv.columnconfigure(0, weight=1)
		self.viewports['velocity'] = Canvas(fv, bg="#FFFFFF") #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['velocity'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_velocity'] = IntVar()
		self.variables['goal_velocity'].set(0)
		self.checks['goal_velocity']= Checkbutton(fv,text="Goal Velocity",variable=self.variables['goal_velocity'])
		self.checks['goal_velocity'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_velocity'] = IntVar()
		self.variables['setpoint_velocity'].set(1)
		self.checks['setpoint_velocity']= Checkbutton(fv,text="Setpoint Velocity",variable=self.variables['setpoint_velocity'])
		self.checks['setpoint_velocity'].grid(column = 4, row = 2, sticky='w')
		self.variables['present_velocity'] = IntVar()
		self.variables['present_velocity'].set(1)		
		self.checks['present_velocity']= Checkbutton(fv,text="Present Velocity",variable=self.variables['present_velocity'])
		self.checks['present_velocity'].grid(column = 4, row = 3, sticky='w')


		fc = LabelFrame(self,text="Current Control")
		fc.grid(column = 0, row = 3, sticky='nesw')
		fc.columnconfigure(0, weight=1)
		self.viewports['current'] = Canvas(fc, bg="#FFFFFF") #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['current'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_current'] = IntVar()
		self.variables['goal_current'].set(0)
		self.checks['goal_current']= Checkbutton(fc,text="Goal Current",variable=self.variables['goal_current'])
		self.checks['goal_current'].grid(column = 4, row = 1, sticky='w')
		self.variables['setpoint_current'] = IntVar()
		self.variables['setpoint_current'].set(1)
		self.checks['setpoint_current']= Checkbutton(fc,text="Setpoint Current",variable=self.variables['setpoint_current'])
		self.checks['setpoint_current'].grid(column = 4, row = 2, sticky='w')
		self.variables['present_current'] = IntVar()
		self.variables['present_current'].set(1)		
		self.checks['present_current']= Checkbutton(fc,text="Present Current",variable=self.variables['present_current'])
		self.checks['present_current'].grid(column = 4, row = 3, sticky='w')

		fpwm = LabelFrame(self,text="FOC Control")
		fpwm.grid(column = 0, row = 4, sticky='nesw')
		fpwm.columnconfigure(0, weight=1)
		self.viewports['pwm'] = Canvas(fpwm, bg="#FFFFFF", height=160) #, width=viewport_max_x, height=viewport_size_y_pos, bg="#FFFFFF")
		self.viewports['pwm'].grid(column = 0, row = 1, rowspan=20, sticky="we") #, sticky='wens')
		self.variables['goal_pwm'] = IntVar()
		self.variables['goal_pwm'].set(1)
		self.checks['goal_pwm']= Checkbutton(fpwm,text="Field Orientation",variable=self.variables['goal_pwm'])
		self.checks['goal_pwm'].grid(column = 4, row = 1, sticky='w')

		self.viewport_current_x = 0
		self.viewport_start_time = time.time()
		self.viewport_size_y_pos = self.viewports['position'].winfo_height()
		self.viewport_size_x_pos = self.viewports['position'].winfo_width()
		self.viewport_size_y_vel = self.viewports['velocity'].winfo_height()
		self.viewport_size_y_cur = self.viewports['current'].winfo_height()
		self.viewport_size_y_pwm = self.viewports['pwm'].winfo_height()
		self.grid_x()

	def grid_x(self):
			self.viewports['position'].create_line(0,self.viewport_size_y_pos/2.0,self.viewport_size_x_pos,self.viewport_size_y_pos/2.0,width=1,fill="#CCCCCC")
			self.viewports['velocity'].create_line(0,self.viewport_size_y_vel/2.0,self.viewport_size_x_pos,self.viewport_size_y_vel/2.0,width=1,fill="#CCCCCC")
			self.viewports['current'].create_line(0,self.viewport_size_y_cur/2.0,self.viewport_size_x_pos,self.viewport_size_y_cur/2.0,width=1,fill="#CCCCCC")
			self.viewports['pwm'].create_line(0,self.viewport_size_y_pwm/2.0,self.viewport_size_x_pos,self.viewport_size_y_pwm/2.0,width=1,fill="#CCCCCC")
			self.viewports['pwm'].create_line(0,self.viewport_size_y_pwm/4.0*3.0,self.viewport_size_x_pos,self.viewport_size_y_pwm/4.0*3.0,width=1,fill="#00FF00")
			self.viewports['pwm'].create_line(0,self.viewport_size_y_pwm/4.0,self.viewport_size_x_pos,self.viewport_size_y_pwm/4.0,width=1,fill="#00FF00")

	def clear(self):
		for n,c in self.viewports.items():
			c.delete("all")
			self.grid_x()
		self.viewport_current_x = 0

	def torque_enable(self):

		if self.variables["torque_enable_local"].get()==0:
			print("torque disable")

		elif self.variables["torque_enable_local"].get()==1:
			print("torque enable")

		print("write RAM...")
		error = self.protocol.write_byte_command(
			self.id.current_id, # ID
			0x40, #RAM
			[int(self.variables['torque_enable_local'].get())], # data,
			verbose=1
		)
		print("error:"+str(error))


	def test_square_current(self):
		current = self.variables["current"].get()
		if time.time()*1000.0 >= self.test_timer+2000.0:
			self.test_timer = time.time()*1000.0
			if self.test_square_value == 0:
				self.test_square_value = 1
				return -current
			elif self.test_square_value == 1:
				self.test_square_value = -1
				return 1
			elif self.test_square_value == -1:
				self.test_square_value = 1
				return -current
		else:
			return 0

	def test_square_position(self):
		amplitude = 90.0*6.0/2.0 #60° test
		if time.time()*1000.0 >= self.test_timer+1000.0:
			self.test_timer = time.time()*1000.0
			if self.test_square_value == 0:
				self.test_square_value = 1
				return int((amplitude)*10)
			elif self.test_square_value == 1:
				self.test_square_value = -1
				return int((-amplitude)*10)
			elif self.test_square_value == -1:
				self.test_square_value = 1
				return int((amplitude)*10)
		else:
			return 0

	def test_triangle_position(self):
		amplitude = 90.0
		if time.time()*1000.0 >= self.test_timer+250.0:
			self.test_timer = time.time()*1000.0
			if self.test_square_value == 0:
				self.test_square_value = 1
				return int((amplitude)*10), 
			elif self.test_square_value == 1:
				self.test_square_value = -1
				return int((-amplitude)*10)
			elif self.test_square_value == -1:
				self.test_square_value = 1
				return int((amplitude)*10)
		else:
			return 0

	def test_sinus_position(self):
		amplitude = 90.0*6.0*10.0
		return int(math.sin(time.time()*6.0*math.pi)*amplitude/2.0)

	def test_round_position(self):
		if time.time()*1000.0 >= self.test_timer+20.0:
			self.test_timer = time.time()*1000.0
			r = 0.025;
			x0 = 0.005
			z0 = -0.095;
			foot_position = np.zeros((3,1)) # x,y,z
			foot_position[0,0] = x0+r*math.cos(self.test_timer/500*2.0*math.pi)
			foot_position[2,0] = z0+r*math.sin(self.test_timer/500*2.0*math.pi)
			servo_angles_rad = ik(foot_position)
			return True, np.degrees(servo_angles_rad)
		else: 
			foot_position = np.zeros((3,1)) # x,y,z
			return False, None


	def update(self,
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
		):

			self.viewport_size_y_pos = self.viewports['position'].winfo_height()
			self.viewport_size_x_pos = self.viewports['position'].winfo_width()
			self.viewport_size_y_vel = self.viewports['velocity'].winfo_height()
			self.viewport_size_y_cur = self.viewports['current'].winfo_height()
			self.viewport_size_y_pwm = self.viewports['pwm'].winfo_height()

			#print(str(viewport_size_y_pos)+"x"+str(viewport_size_x_pos))

			# time line trace
			if self.viewport_current_x == 0:
				self.viewport_start_time=time.time()

			if time.time() >= self.viewport_start_time+1.0:
				self.viewport_start_time += 1.0
				self.viewports['position'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_pos,
					width=2,
					fill="#DDDDDD"
				)
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_vel,
					width=2,
					fill="#DDDDDD"
				)
				self.viewports['current'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_cur,
					width=2,
					fill="#DDDDDD"
				)
				self.viewports['pwm'].create_line(
					self.viewport_current_x,
					0,
					self.viewport_current_x+1,
					self.viewport_size_y_pwm,
					width=2,
					fill="#DDDDDD"
				)

				# change 180 by the max rotation angle of the servo form ram
				# change 180 by the max rotation angle of the servo form ram
				# change 180 by the max rotation angle of the servo form ram

			if self.variables['goal_position'].get() == 1:
				self.viewports['position'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pos-(goal_position-int(self.eeprom.variables['min_position_servo'].get()))*self.viewport_size_y_pos/(int(self.eeprom.variables['max_position_servo'].get())-int(self.eeprom.variables['min_position_servo'].get())+1),
					self.viewport_current_x+1,
					self.viewport_size_y_pos-(goal_position-int(self.eeprom.variables['min_position_servo'].get()))*self.viewport_size_y_pos/(int(self.eeprom.variables['max_position_servo'].get())-int(self.eeprom.variables['min_position_servo'].get())+1),
					width=2,
					fill="#0000FF"
				)
			if self.variables['setpoint_position'].get() == 1:						
				self.viewports['position'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pos-(setpoint_position-int(self.eeprom.variables['min_position_servo'].get()))*self.viewport_size_y_pos/(int(self.eeprom.variables['max_position_servo'].get())-int(self.eeprom.variables['min_position_servo'].get())+1),
					self.viewport_current_x+1,
					self.viewport_size_y_pos-(setpoint_position-int(self.eeprom.variables['min_position_servo'].get()))*self.viewport_size_y_pos/(int(self.eeprom.variables['max_position_servo'].get())-int(self.eeprom.variables['min_position_servo'].get())+1),
					width=3,
					fill="#FF0000"
				)		
			if self.variables['present_position'].get() == 1:
				self.viewports['position'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_pos-(present_position-int(self.eeprom.variables['min_position_servo'].get()))*self.viewport_size_y_pos/(int(self.eeprom.variables['max_position_servo'].get())-int(self.eeprom.variables['min_position_servo'].get())+1),
					self.viewport_current_x+1,
					self.viewport_size_y_pos-(present_position-int(self.eeprom.variables['min_position_servo'].get()))*self.viewport_size_y_pos/(int(self.eeprom.variables['max_position_servo'].get())-int(self.eeprom.variables['min_position_servo'].get())+1),
					width=3,
					fill="#000000"
				)
		
			if self.variables['goal_velocity'].get() == 1:
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_vel/2-goal_velocity/int(self.eeprom.variables['max_velocity_servo'].get())*self.viewport_size_y_vel/2,
					self.viewport_current_x+1,
					self.viewport_size_y_vel/2-goal_velocity/int(self.eeprom.variables['max_velocity_servo'].get())*self.viewport_size_y_vel/2,
					width=2,
					fill="#0000FF"
				)
			if self.variables['setpoint_velocity'].get() == 1:
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_vel/2-setpoint_velocity/int(self.eeprom.variables['max_velocity_servo'].get())*self.viewport_size_y_vel/2,
					self.viewport_current_x+1,
					self.viewport_size_y_vel/2-setpoint_velocity/int(self.eeprom.variables['max_velocity_servo'].get())*self.viewport_size_y_vel/2,
					width=2,
					fill="#FF0000"
				)					
			if self.variables['present_velocity'].get() == 1:
				self.viewports['velocity'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_vel/2-present_velocity/int(self.eeprom.variables['max_velocity_servo'].get())*self.viewport_size_y_vel/2,
					self.viewport_current_x+1,
					self.viewport_size_y_vel/2-present_velocity/int(self.eeprom.variables['max_velocity_servo'].get())*self.viewport_size_y_vel/2,
					width=2,
					fill="#000000"
				)

			if self.variables['goal_current'].get() == 1:
				self.viewports['current'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_cur/2-goal_torque_current/int(self.eeprom.variables['max_current_servo'].get())*self.viewport_size_y_cur/2,
					self.viewport_current_x+1,
					self.viewport_size_y_cur/2-goal_torque_current/int(self.eeprom.variables['max_current_servo'].get())*self.viewport_size_y_cur/2,
					width=2,
					fill="#0000FF"
				)					
			if self.variables['setpoint_current'].get() == 1:
				self.viewports['current'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_cur/2-setpoint_torque_current/int(self.eeprom.variables['max_current_servo'].get())*self.viewport_size_y_cur/2,
					self.viewport_current_x+1,
					self.viewport_size_y_cur/2-setpoint_torque_current/int(self.eeprom.variables['max_current_servo'].get())*self.viewport_size_y_cur/2,
					width=2,
					fill="#FF0000"
				)					
			if self.variables['present_current'].get() == 1:
				self.viewports['current'].create_line(
					self.viewport_current_x,
					self.viewport_size_y_cur/2-present_torque_current/int(self.eeprom.variables['max_current_servo'].get())*self.viewport_size_y_cur/2,
					self.viewport_current_x+1,
					self.viewport_size_y_cur/2-present_torque_current/int(self.eeprom.variables['max_current_servo'].get())*self.viewport_size_y_cur/2,
					width=2,
					fill="#000000"
				)					


			#if self.variables['goal_pwm'].get() == 1:
			foc_angle = math.atan2(present_torque_current,present_flux_current+0.0000000001)
			self.viewports['pwm'].create_line(
				self.viewport_current_x,
				self.viewport_size_y_pwm/2-foc_angle/3.1415*self.viewport_size_y_pwm/2.0,
				self.viewport_current_x+1,
				self.viewport_size_y_pwm/2-foc_angle/3.1415*self.viewport_size_y_pwm/2.0,
				width=2,
				fill="#0000FF"
			)										

			# inc x
			self.viewport_current_x = self.viewport_current_x + 1 

			# repeat
			if self.viewport_current_x > self.viewport_size_x_pos:
				self.clear()
