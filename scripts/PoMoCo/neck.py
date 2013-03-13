class neck():
	def __init__(self,con,servoNum):
		self.con = con
		self.servoNum = servoNum

	def set(self,deg):
		self.con.servos[self.servoNum].setPos(deg=deg)
