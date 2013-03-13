import time
import math

from servotorComm import runMovement

# Import custom message data.
import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from ROSPoMoCo.msg import leg_pose

# Modifies how smoothly the servos move.
# Smoother means more processing power, and fills the serial line.
# Lower if movements start to slow down, or get weird.
# Anything higher than 50 is pointless ( faster than the maximum refresh of standard servos ).
stepPerS = 5

class leg():

	def __init__(self,con,name,hipServoNum,kneeServoNum,ankleServoNum,simOrigin=(0,3,0)):
		self.con = con
		self.name = name
		self.hipServoNum = hipServoNum
		self.kneeServoNum = kneeServoNum
		self.ankleServoNum = ankleServoNum
		l = leg_pose()
		l.hip_angle = 2
		rospy.logwarn(l)
		
	def get_pose( self ):
		# TODO test!
		def degrees_to_uint8( degrees ):
			return ( degrees * 2 ** 8 ) / 180
		pose = leg_pose()
		pose.hip_angle = degrees_to_uint8( self.con.servos[self.hipServoNum].getOffsetDeg() )
		pose.knee_angle = degrees_to_uint8( self.con.servos[self.kneeServoNum].getOffsetDeg() )
		pose.ankle_angle = degrees_to_uint8( self.con.servos[self.ankleServoNum].getOffsetDeg() )
		return pose
	def set_pose( self, pose ):
		# TODO test!
		def uint8_to_degrees( unit8 ):
			return ( uint8 * 180 ) / 2 ** 8
		self.hip( uint8_to_degrees(  pose.hip_angle ) )
		self.knee( uint8_to_degrees(  pose.hip_angle ) )
		self.ankle( uint8_to_degrees(  pose.ankle_angle ) )	

	def hip(self, deg):
		if deg == "sleep":
			self.con.servos[self.hipServoNum].kill()
		else:
			self.con.servos[self.hipServoNum].setPos(deg=deg)

	def knee(self, deg):
		if deg == "sleep":
			self.con.servos[self.hipServoNum].kill()
		else:
			self.con.servos[self.kneeServoNum].setPos(deg=deg)

	def ankle(self, deg):
		if deg == "sleep":
			self.con.servos[self.hipServoNum].kill()
		else:
			self.con.servos[self.ankleServoNum].setPos(deg=deg)

	def setHipDeg(self,endHipAngle,stepTime=1):
		runMovement(self.setHipDeg_function,endHipAngle,stepTime)

	def setFootY(self,footY,stepTime=1):
		runMovement(self.setFootY_function,footY,stepTime)

	def replantFoot(self,endHipAngle,stepTime=1):
		runMovement(self.replantFoot_function,endHipAngle,stepTime)

	def setHipDeg_function(self,endHipAngle,stepTime):
		currentHipAngle = self.con.servos[self.hipServoNum].getPosDeg()
		hipMaxDiff = endHipAngle-currentHipAngle

		steps = range(int(stepPerS))
		for i,t in enumerate(steps):
			# TODO: Implement time-movements the servo commands sent for far fewer
			#	   total servo commands
			hipAngle = (hipMaxDiff/len(steps))*(i+1)
			try:
				anglNorm=hipAngle*(180/(hipMaxDiff))
			except:
				anglNorm=hipAngle*(180/(1))
			hipAngle = currentHipAngle+hipAngle
			self.con.servos[self.hipServoNum].setPos(deg=hipAngle)

			#wait for next cycle
			time.sleep(stepTime/float(stepPerS))

	def setFootY_function(self,footY,stepTime):
		# TODO: Max step-time dependent
		# TODO: Implement time-movements the servo commands sent for far fewer
		#	   total servo commands

		if (footY < 75) and (footY > -75):
			kneeAngle = math.degrees(math.asin(float(footY)/75.0))
			ankleAngle = 90-kneeAngle

			self.con.servos[self.kneeServoNum].setPos(deg=kneeAngle)
			self.con.servos[self.ankleServoNum].setPos(deg=-ankleAngle)

	def replantFoot_function(self,endHipAngle,stepTime):
	# Smoothly moves a foot from one position on the ground to another in time seconds
	# TODO: implement time-movements the servo commands sent for far fewer total servo
	#	   commands

		currentHipAngle = self.con.servos[self.hipServoNum].getPosDeg()

		hipMaxDiff = endHipAngle-currentHipAngle

		steps = range(int(stepPerS))
		for i,t in enumerate(steps):

			hipAngle = (hipMaxDiff/len(steps))*(i+1)
			#print "hip angle calculated:",hipAngle

			# Calculate the absolute distance between the foot's highest and lowest point
			footMax = 0
			footMin = floor
			footRange = abs(footMax-footMin)

			# Normalize the range of the hip movement to 180 deg
			try:
				anglNorm=hipAngle*(180/(hipMaxDiff))
			except:
				anglNorm=hipAngle*(180/(1))
			#print "normalized angle:",anglNorm

			# Base footfall on a sin pattern from footfall to footfall with 0 as the midpoint
			footY = footMin-math.sin(math.radians(anglNorm))*footRange
			#print "calculated footY",footY

			# Set foot height
			self.setFootY(footY,stepTime=0)
			hipAngle = currentHipAngle+hipAngle
			self.con.servos[self.hipServoNum].setPos(deg=hipAngle)

			# Wait for next cycle
			time.sleep(stepTime/float(stepPerS))
