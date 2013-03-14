from servo import *

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy

class physical_servo( servo ):
	def __init__(self,servoNum,serHandler,servoPos=1500,offset=0,active=False):
	
		self.serHandler = serHandler
		
		if active:
			self.attach()
		else:
			self.detach()
		self.servoNum = servoNum

		# Servo position and offset is stored in microseconds (uS)
		self.servoPos = servoPos
		self.offset = offset


	def setPos(self,timing=None,deg=None,move=True):
		if timing != None:
			self.servoPos = timing
		if deg != None:
			self.servoPos = int(1500.0+float(deg)*11.1111111)
		if move:
			self.attach()
			self.move()
			rospy.logdebug( "Moved " + str( self.servoNum ) )
		rospy.logdebug( "Servo " + str( self.servoNum ) + " set to " + str( self.servoPos ) )

	def getPosDeg(self):
		return (self.servoPos-1500)/11.1111111

	def getPosuS(self):
		return self.servoPos

	def getActive(self):
		return self.isAttached()

	def getOffsetDeg(self):
		return (self.offset-1500)/11.1111111

	def getOffsetuS(self):
		return self.offset

	def setOffset(self,timing=None,deg=None):
		if timing != None:
			self.offset = timing
		if deg != None:
			self.offset = int(float(deg)*11.1111111)

	def reset(self):
		self.setPos(timing=1500)
		self.move()

	def kill(self):
		self.detach()
		self.serHandler.send( "#%dL\r"%(self.servoNum) )
		
	def move(self):
		if self.isAttached():
			servoPos = self.servoPos+self.offset
			# Auto-correct the output to bound within 500uS to 2500uS signals, the limits of the servos
			if servoPos < 500:
				servoPos = 500
			if servoPos > 2500:
				servoPos = 2500
				
			# Send the message the serial handler
			self.serHandler.send( "#%dP%.4dT0\r"%(self.servoNum,int(servoPos)) )
		else:
			try:
				self.serHandler.send( "#%.4dL\r"%(self.servoNum,int(servoPos)) )
			except:
				pass
