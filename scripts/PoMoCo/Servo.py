# Import custom message data.
import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from ROSPoMoCo.msg import servo_pose

# Defines a general servo interface and the basics of it's operation.
# If python supported it, this class would be abstract.
class Servo:
	# Define a general default constructor.
	def __init__( self, position = 0, offset = 0, attached = False ):
		self.position = position
		self.offset = offset
		self.attached = attached
	# Set the servo position in degrees.
	def setPosition( self, position ):
		self.position = position
	# Get the listed servo position in degrees.
	def getPosition( self ):
		return self.position
	# Set the servo offset ( zero position ) in degrees.
	def setOffset( self, offset ):
		self.offset = offset
	# Get the servo offset ( zero position ) in degrees.
	def getOffset( self ):
		return self.offset
	# Set the servo to the zero position
	def reset( self ):
		self.setPosition( 0 )
	# Return the current status of the servo as a Boolean.
	def isAttached( self ):
		return self.attached
	# Attach and tense the servo.
	def attach( self ):
		self.attached = True
	# Detach and relax the servo.
	def detach( self ):
		self.attached = False
		
	def getPose( self ):
		currentPose = servo_pose()
		
		currentPose.attached = self.isAttached()
		currentPose.position = self.getPosition()
		
		return currentPose
		
	def setPose( self, newPose ):
		self.setPosition( newPose.position )
		if newPose.attached:
			self.attach()
		else:
			self.detach()
