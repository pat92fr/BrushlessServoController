## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
from protocol2 import *

import time
import serial

class id_frame(LabelFrame):

	def __init__(self,window,protocol):
		super().__init__(text="COM / ID")
		self.protocol = protocol
		self.labels = {}
		self.entries = {}
		self.lists = {}
		self.variables = {}
		self.row = 0

		self.current_id = 1
		self.current_port = 1

		# update button
		button_update = Button(self,text="Update COM",command = self.update_com)
		button_update.grid(column = 0, row = self.row, sticky='we')
		self.row += 1

		# list COM
		self.lists["com"] = Listbox(self)
		self.lists["com"].grid(column = 0, row = self.row, sticky='w')
		self.lists["com"].bind("<<ListboxSelect>>", self.select_com)
		self.row += 1

		# update button
		button_update = Button(self,text="Update ID",command = self.update_ids)
		button_update.grid(column = 0, row = self.row, sticky='we')
		self.row += 1


		# list ID
		self.lists["ids"] = Listbox(self)
		self.lists["ids"].grid(column = 0, row = self.row, sticky='w')
		self.lists["ids"].bind("<<ListboxSelect>>", self.select_id)
		self.row += 1

		# startup auto ping and update list
		self.update_com()
		self.update_ids()


	def gui_spacer(self,text_label):
		label = Label(self, text = text_label, anchor="w", justify=LEFT)
		label.grid(column = 0, row = self.row, sticky='w')
		self.row += 1		


	def update_com(self):
		ports = ['COM%s' % (i + 1) for i in range(1, 256)] # avoid port 1
		print("COM:"+str(ports))
		counter = 0
		for port in ports:
			try:
				s = serial.Serial(port)
				s.close()
				self.lists["com"].insert(counter,str(port))
				counter += 1
			except (OSError, serial.SerialException):
				pass
		self.lists["com"].selection_set(0)
		if self.lists["com"].curselection():
			self.current_port = self.lists["com"].get(self.lists["com"].curselection())
			print("self.current_port:"+str(self.current_port))
			self.protocol.open(self.current_port,1000000) ##eeprom baud = 3


	def update_ids(self):
		self.lists["ids"].delete(0,END)
		counter = 0
		for i in range(1,20):
			print("ping servo ID:" + str(i) + '...')
			error, model_number, firmware_version = self.protocol.ping_command(i)
			#print("error:"+str(error))
			if(error==0):
				self.lists["ids"].insert(counter,str(i))
				counter += 1
		self.lists["ids"].selection_set(0)
		if self.lists["ids"].curselection():
			self.current_id = int(self.lists["ids"].get(self.lists["ids"].curselection()))
			print("self.current_id:"+str(self.current_id))

	def select_id(self,event):
		if self.lists["ids"].curselection():
			self.current_id = int(self.lists["ids"].get(self.lists["ids"].curselection()))
			print("self.current_id:"+str(self.current_id))

	def select_com(self,event):
		if self.lists["com"].curselection():
			self.current_port = self.lists["com"].get(self.lists["com"].curselection())
			print("self.current_port:"+str(self.current_port))
			self.protocol.open(self.current_port,1000000) ##eeprom baud = 3
			self.update_ids()
#		else:
#			self.current_port = 0
#			print("self.current_port:"+str(self.current_port))
#			self.protocol = None

