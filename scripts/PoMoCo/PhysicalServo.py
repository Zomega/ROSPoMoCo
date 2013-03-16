from Servo import *

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
	
class physicalServo( Servo ):
	# Physical servos need a reference to the servo handler, so that they can notify it when they need to be physically updated.
	def __init__( self, controller, position = 0, offset = 0, attached = False ):
		Servo.__init__( self, position, offset, attached )
		self.controller = controller
		self.__calculateTiming__()
	
	# Caches the timing value so that it does not have to be calculated on the fly.
	def __calculateTiming__( self ):
		# Figure out the angle we need to actually send to the servo.
		angle = self.getPosition() + self.getOffset()
		# Convert into the corresponding uS.
		self.__timing__ =  100 * ( angle - 1500 ) / 9.0
		# Clip to the valid range.
		if self.__timing__ < 500:
			self.__timing__ = 500
		if self.__timing__ > 2500:
			self.__timing__ = 2500
		
	# Set the servo position in degrees, then notify the handler if we should move.
	def setPosition( self, position ):
		Servo.setPosition( self, position )
		self.__calculateTiming__()
		
		if self.isAttached():
			self.controller.notify( self )
		
	# Set the servo offset ( zero position ) in degrees, then notify the handler if we should move.
	def setOffset( self, offset ):
		Servo.setOffset( self, position )
		self.__calculateTiming__()
		
		if self.isAttached():
			self.controller.notify( self )

	# Attach and tense the physical servo.
	def attach( self ):
		Servo.attach( self )
		self.controller.notify( self )

	# Detach and relax the physical servo.
	def detach( self ):
		Servo.detach( self )
		self.controller.notify( self )
