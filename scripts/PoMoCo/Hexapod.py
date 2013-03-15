from Leg import *
from Neck import *

# Import custom message data.
import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from ROSPoMoCo.msg import pose

class Hexapod:
	def __init__( self, con ):
		self.con = con		
		self.RF	= Leg('rightFront',	con.getServo(24), con.getServo(25), con.getServo(26) )
		self.RM	= Leg('rightMid',	con.getServo(20), con.getServo(21), con.getServo(22) )
		self.RB	= Leg('rightBack',	con.getServo(16), con.getServo(17), con.getServo(18) )

		self.LF	= Leg('leftFront',	con.getServo(7),   con.getServo(6),   con.getServo(5) )
		self.LM	= Leg('leftMid',	con.getServo(11), con.getServo(10), con.getServo(9) )
		self.LB	= Leg('leftBack',	con.getServo(15), con.getServo(14), con.getServo(13) )

		self.legs	= [
					self.RF,
					self.RM,
					self.RB,
					self.LF,
					self.LM,
					self.LB]

		self.neck	= Neck( con.getServo(31) )

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
		self.LF.set_pose( newPose.left_front )
		self.LM.set_pose( newPose.left_middle )
		self.LB.set_pose( newPose.left_back )
		
		self.RF.set_pose( newPose.right_front )
		self.RM.set_pose( newPose.right_middle )
		self.RB.set_pose( newPose.right_back )
