from leg import *
from neck import *

# Import custom message data.
import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from ROSPoMoCo.msg import pose

class hexapod():

	def __init__(self,con):
		self.con = con
		
		self.RF	= leg(con,'rightFront',24,25,26)
		self.RM	= leg(con,'rightMid',20,21,22)
		self.RB	= leg(con,'rightBack',16,17,18)

		self.LF	= leg(con,'leftFront',7,6,5)
		self.LM	= leg(con,'leftMid',11,10,9)
		self.LB	= leg(con,'leftBack',15,14,13)

		self.legs	= [
					self.RF,
					self.RM,
					self.RB,
					self.LF,
					self.LM,
					self.LB]

		self.neck	= neck(con,31)

		self.tripod1 = [self.RF,self.RB,self.LM]
		self.tripod2 = [self.LF,self.LB,self.RM]
