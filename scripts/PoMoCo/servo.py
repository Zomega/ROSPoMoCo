# Defines a general servo interface and the basics of it's operation.
class Servo:
	# Define a general default constructor.
	def __init__( self, position = 0, offset = 0, attached = False ):
		self.position = position
		self.offset = offset
		self.attached = False
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
	def getOffset( self, offset ):
		return self.offset
	# Return the current status of the servo as a Boolean.
	def isAttached( self ):
		return self.attached
	# Attach and tense the physical servo.
	def attach( self ):
		self.attached = True
	# Detach and relax the physical servo.
	def detach( self ):
		self.attached = False
