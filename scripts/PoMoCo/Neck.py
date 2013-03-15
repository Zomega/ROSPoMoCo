class Neck():
	def __init__(self, neckServo):
		self.neckServo = neckServo

	def set( self, deg ):
		self.neckServo.setPosition( deg )
