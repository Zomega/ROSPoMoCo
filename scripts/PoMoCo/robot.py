from leg import *
from neck import *

# Import custom message data.
import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from ROSPoMoCo.msg import pose

class hexapod():

	def __init__( self, con ):
		self.con = con		
		self.RF	= leg('rightFront',	con.getServo(24), con.getServo(25), con.getServo(26) )
		self.RM	= leg('rightMid',	con.getServo(20), con.getServo(21), con.getServo(22) )
		self.RB	= leg('rightBack',	con.getServo(16), con.getServo(17), con.getServo(18) )

		self.LF	= leg('leftFront',	con.getServo(7),   con.getServo(6),   con.getServo(5) )
		self.LM	= leg('leftMid',	con.getServo(11), con.getServo(10), con.getServo(9) )
		self.LB	= leg('leftBack',	con.getServo(15), con.getServo(14), con.getServo(13) )

		self.legs	= [
					self.RF,
					self.RM,
					self.RB,
					self.LF,
					self.LM,
					self.LB]

		self.neck	= neck( con.getServo(31) )

		self.tripod1 = [self.RF,self.RB,self.LM]
		self.tripod2 = [self.LF,self.LB,self.RM]
		
	def get_pose( self ):
		Pose = pose()
		
		Pose.left_front = self.LF.get_pose()
		Pose.left_middle = self.LM.get_pose()
		Pose.left_back = self.LB.get_pose()
		
		Pose.right_front = self.RF.get_pose()
		Pose.right_middle = self.RM.get_pose()
		Pose.right_back = self.RB.get_pose()
		
		return Pose
		
	def set_pose( self, Pose ):
		# TODO test!
		self.LF.set_pose( Pose.left_front )
		self.LM.set_pose( Pose.left_middle )
		self.LB.set_pose( Pose.left_back )
		
		self.RF.set_pose( Pose.right_front )
		self.RM.set_pose( Pose.right_middle )
		self.RB.set_pose( Pose.right_back )
		
	def set_pose( self, pose ):
		# TODO test!
		def uint8_to_degrees( unit8 ):
			return ( uint8 * 180 ) / 2 ** 8
		self.hip( uint8_to_degrees(  pose.hip_angle ) )
		self.knee( uint8_to_degrees(  pose.hip_angle ) )
		self.ankle( uint8_to_degrees(  pose.ankle_angle ) )	
