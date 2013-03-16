class Neck():
	def __init__(self, neckServo):
		self.neckServo = neckServo

	def set( self, deg ):
		self.neckServo.setPosition( deg )
		
	def getPose( self ):
		return self.neckServo.getPose()
		
	def setPose( self, newPose ):
		self.neckServo.setPose( newPose )
