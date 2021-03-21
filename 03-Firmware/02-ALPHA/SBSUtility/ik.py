import numpy as np 
import math

from felin import *
from intersection import *

##DEBUG #######################################################################

# TRACE macro
DEBUG = False

def log(s):
    if DEBUG:
        print(s)

##DEBUG #######################################################################

# return 3x1 vector (a (femur), b (tibia)) in RADIANS
def ik(foot_position): # input 3x1 vector  (x,y,z,1) in METERS, where x axis pointing forward, y axis pointing exterior, and z axis pointing upward
	log("foot_position:\n"+str(foot_position))
	# Angle Y
	Y = math.atan2(-foot_position[2,0],foot_position[0,0])
	log("Y: "+str(round(math.degrees(Y),0))+"deg")
	# Distance HK
	HK = math.sqrt(foot_position[2,0]**2+foot_position[0,0]**2)
	log("HK: "+str(round(HK,3))+"m")
	# Angle E
	E = math.acos((HK**2 + Lf**2 - Lt**2)/(2*HK*Lf))
	log("E: "+str(round(math.degrees(E),0))+"deg")
	# Angle S1
	A = math.pi - E - Y
	log("A: "+str(round(math.degrees(A),0))+"deg")
	# Knee position
	Kx = -Lf * math.cos(A)
	Kz = - Lf * math.sin(A)
	log("Kx:"+str(Kx))
	log("Kz:"+str(Kz))
	# Position B
	Bx = (Lt+Ll)/Lt*(Kx-foot_position[0,0])+foot_position[0,0]
	Bz = (Lt+Ll)/Lt*(Kz-foot_position[2,0])+foot_position[2,0]
	log("Bx:"+str(Bx))
	log("Bz:"+str(Bz))
	# A position at circle intersection
	Ax, Az = intetsection(Bx,Bz,Lb,S2x,S2z,Lh2)
	log("Ax:"+str(Ax))
	log("Az:"+str(Az))
	# Angle S2
	B = math.atan2(Az-S2z,-(Ax-S2x))
	log("B: "+str(round(math.degrees(B),0))+"deg")
	return np.array((A,B)).reshape((2,1))


if __name__ == "__main__":
	print("Test Unitaire ik.py")
	foot_position = np.zeros((3,1)) # x,y,z
	foot_position[0,0] =  0.000
	foot_position[1,0] =  0.000
	foot_position[2,0] = -0.060
	print(foot_position)
	leg_joint_angles = ik(foot_position) # A,B
	print(np.round(np.degrees(leg_joint_angles),1))
	# return 0.0 90.0

	foot_position[0,0] =  0.000
	foot_position[1,0] =  0.000
	foot_position[2,0] = -0.060
	print(foot_position)
	leg_joint_angles = ik(foot_position) # A,B
	print(np.round(np.degrees(leg_joint_angles),1))
	# return 41.4 82.8

	foot_position[0,0] =  0.000
	foot_position[1,0] =  0.000
	foot_position[2,0] = -0.060
	print(foot_position)
	leg_joint_angles = ik(foot_position) # A,B
	print(np.round(np.degrees(leg_joint_angles),1))
	# return 41.4 82.8












