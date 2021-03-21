## http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter


from tkinter import *
#from tkinter.ttk import *
from protocol2 import sign
import time

class id_frame(LabelFrame):

	def __init__(self,window,protocol):
		super().__init__(text="ID")
		self.protocol = protocol
		self.labels = {}
		self.entries = {}
		self.lists = {}
		self.variables = {}
		self.row = 0

		self.current_id = 1

		# update button
		button_update = Button(self,text="Update",command = self.update)
		button_update.grid(column = 0, row = self.row, sticky='we')
		self.row += 1

		# list
		self.lists["ids"] = Listbox(self)
		self.lists["ids"].grid(column = 0, row = self.row, sticky='w')
		self.lists["ids"].bind("<<ListboxSelect>>", self.select_id)
		self.row += 1

		# startup auto ping and update list
		self.update()


	def gui_spacer(self,text_label):
		label = Label(self, text = text_label, anchor="w", justify=LEFT)
		label.grid(column = 0, row = self.row, sticky='w')
		self.row += 1		

	def update(self):
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
