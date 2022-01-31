import time

import serial
# help : https://pythonhosted.org/pyserial/pyserial_api.html

from tkinter import *
#from tkinter.ttk import *

from id_frame import *
from eeprom_frame import *
from ram_frame import *
from trace_frame import *
from protocol2 import *

import cProfile


def main():
	servo = servo_protocol2()
	window = Tk()
	window.title(" 8yServoGUI")
	window.geometry("1600x1020")
	window.minsize(1600,1020)

	fi = id_frame(window,servo)
	fe = eeprom_frame(window,servo,fi)
	ft =trace_frame(window,servo,fe,fi)
	fr = ram_frame(window,servo,ft,fi)

	fi.grid(column = 0, row = 0, sticky='nsew')		
	fe.grid(column = 1, row = 0, sticky='nsew')		
	ft.grid(column = 2, row = 0, sticky='nsew')		
	fr.grid(column = 3, row = 0, sticky='nsew')		
		
	window.columnconfigure(2, weight=1)
	window.rowconfigure(0, weight=1)

	mainloop()


if __name__ == "__main__":

	##cProfile.run("main()")
	main()

