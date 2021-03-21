import math

def intetsection(x0,y0,R0,x1,y1,R1):
	dx = x0-x1
	dy = y0-y1
	ratio = dx/dy
	N = ( R1**2 - R0**2 - x1**2 + x0**2 - y1**2 + y0**2 ) / ( 2 * (y0-y1) )
	A = (dx/dy)**2+1
	B = 2*y0*(dx/dy)-2*N*(dx/dy)-2*x0
	C = x0**2+y0**2+N**2-R0**2-2*y0*N
	delta = math.sqrt(B**2-4*A*C)
	xi1 = (-B+delta)/(2*A)
	xi2 = (-B-delta)/(2*A)
	yi1 = N-xi1*dx/dy
	yi2 = N-xi2*dx/dy
	if xi1<xi2:
		return xi1,yi1
	else:
		return xi2,yi2
