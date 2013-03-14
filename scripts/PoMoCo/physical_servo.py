from servo import *

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy

def degree_to_uS( angle ):
	# Convert the value to uS...
	timing = ( angle - 1500 ) / 11.1111111
	
	# Clip the output to bound within 500uS to 2500uS
	# these are the limits of the servos
	if timing < 500:
		timing = 500
	if timing > 2500:
		timing = 2500
		
	return timing
	
class physical_servo( servo ):
	# Physical servos need a reference to the servo handler, so that they can notify it when they need to be physically updated.
	def __init__( self, controller, position = 0, offset = 0, attached = False ):
		servo.__init__( self, position, offset, attached )
		self.controller = controller
		self.__calculateTiming__()
	
	# Caches the timing value so that it does not have to be calculated on the fly.
	def __calculateTiming__( self ):
		angle = self.getPosition() + self.getOffset()
		self.__timing__ = degree_to_uS( angle )
		
	# Set the servo position in degrees, then notify the handler if we should move.
	def setPosition( self, position ):
		servo.setPosition( self, position )
		self.__calculateTiming__()
		
		if self.isAttached():
			self.controller.notify( self )
		
	# Set the servo offset ( zero position ) in degrees, then notify the handler if we should move.
	def setOffset( self, offset ):
		servo.setOffset( self, position )
		self.__calculateTiming__()
		
		if self.isAttached():
			self.controller.notify( self )

	# Attach and tense the physical servo.
	def attach( self ):
		servo.attach( self )
		self.controller.notify( self )

	# Detach and relax the physical servo.
	def detach( self ):
		servo.detach( self )
		self.controller.notify( self )
