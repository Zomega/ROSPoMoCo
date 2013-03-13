from servo import *

debug = False

class PhysicalServo( Servo ):
	def __init__(self,servoNum,serHandler,servoPos=1500,offset=0,active=False):
		self.serHandler = serHandler
		self.active = active
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
			self.active = True
			self.move()
			if debug: print "Moved ",self.servoNum
		if debug: print "Servo",self.servoNum,"set to",self.servoPos

	def getPosDeg(self):
		return (self.servoPos-1500)/11.1111111

	def getPosuS(self):
		return self.servoPos

	def getActive(self):
		return self.active

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
		self.active = False
		toSend = "#%dL\r"%(self.servoNum)
		self.serHandler.sendLock.acquire()
		self.serHandler.sendQueue.append(toSend)
		self.serHandler.sendLock.release()
		if debug: print "Sending command #%dL to queue"%self.servoNum

	def move(self):
		if self.active:
			servoPos = self.servoPos+self.offset
			# Auto-correct the output to bound within 500uS to 2500uS signals, the limits of the servos
			if servoPos < 500:
				servoPos = 500
			if servoPos > 2500:
				servoPos = 2500
				
			# Debug message if needed
			if debug: print "Sending command #%dP%dT0 to queue"%(self.servoNum,int(servoPos))

			# Send the message the serial handler in a thread-safe manner
			toSend = "#%dP%.4dT0\r"%(self.servoNum,int(servoPos))
			self.serHandler.sendLock.acquire()
			self.serHandler.sendQueue.append(toSend)
			self.serHandler.sendLock.release()
		else:
			try:
				toSend = "#%.4dL\r"%(self.servoNum,int(servoPos))
				self.serHandler.sendLock.acquire()
				self.serHandler.sendQueue.append(toSend)
				self.serHandler.sendLock.release()
				if debug: print "Sending command #%dL to queue"%self.servoNum
			except:
				pass
