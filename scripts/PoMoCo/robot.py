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
