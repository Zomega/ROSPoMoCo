from Leg import *
from Neck import *

# Import custom message data.
import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from ROSPoMoCo.msg import pose

class Hexapod:
	def __init__( self, servos ):
		self.servos = servos
		self.RF	= Leg('rightFront',	servos[0], servos[1], servos[2] )
		self.RM	= Leg('rightMid',	servos[3], servos[4], servos[5] )
		self.RB	= Leg('rightBack',	servos[6], servos[7], servos[8] )

		self.LF	= Leg('leftFront',	servos[9], servos[10], servos[11] )
		self.LM	= Leg('leftMid',	servos[12], servos[13], servos[14] )
		self.LB	= Leg('leftBack',	servos[15], servos[16], servos[17] )

		self.legs	= [
					self.RF,
					self.RM,
					self.RB,
					self.LF,
					self.LM,
					self.LB]

		self.neck	= Neck( servos[18] )

		self.tripod1 = [self.RF,self.RB,self.LM]
		self.tripod2 = [self.LF,self.LB,self.RM]
		
	def getPose( self ):
		# TODO test!
		currentPose = pose()
		
		currentPose.left_front = self.LF.getPose()
		currentPose.left_middle = self.LM.getPose()
		currentPose.left_back = self.LB.getPose()
		
		currentPose.right_front = self.RF.getPose()
		currentPose.right_middle = self.RM.getPose()
		currentPose.right_back = self.RB.getPose()
		
		return currentPose
		
	def setPose( self, newPose ):
		# TODO test!
		self.LF.setPose( newPose.left_front )
		self.LM.setPose( newPose.left_middle )
		self.LB.setPose( newPose.left_back )
		
		self.RF.setPose( newPose.right_front )
		self.RM.setPose( newPose.right_middle )
		self.RB.setPose( newPose.right_back )
