import time
import math
import serial
import serial.tools.list_ports
import threading

# Load servo architecture...
from servo import *
from virtual_servo import *
from physical_servo import *

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy

startTime = time.clock()
serialSends = []

BAUD_RATE = 9600

# Used by the GUI to ensure responsiveness while running moves.
# Depreciated, ROS can handle the specifics using topics...
# TODO: Remove once GUI has been spun off.
# TODO: Move name translator node should keep a Queue?
class runMovement(threading.Thread):

	def __init__(self,function,*args):
		threading.Thread.__init__(self)
		self.function=function
		self.args = args
		self.start()

	def run(self):
		self.function(*self.args)

class serHandler(threading.Thread):

	def __init__( self ):
		threading.Thread.__init__( self )

		self.ser = None

		self.sendQueue=[]
		self.sendLock = threading.Lock()

		self.recieveQueue=[]
		self.recieveLock = threading.Lock()

		self.serOpen = False
		self.serNum = 0

		self.start()

	def __del__( self ):
		self.ser.close()

	def run( self ):
		self.connect()
		while(True):
			# Send waiting messages
			send = False
			if(len(self.sendQueue)>0):
				self.sendLock.acquire()
				toSend = self.sendQueue.pop(0)
				self.sendLock.release()
				send = True
			else:
				time.sleep(0.01) # Keeps infinite while loop from wasting processor cycles.
			if send:
				sendTime = time.clock()-startTime
				serialSends.append([float(sendTime),str(toSend)])
				time.sleep(0.003)
				# TODO: Determine if the double check of self.serOpen is needed.
				if self.serOpen:
					if self.ser.writable:
						if self.serOpen:
							self.ser.write(str(toSend))
							rospy.logdebug( "Sent '%s' to COM%d"%(str(toSend).strip('\r'),self.serNum+1) )
			rospy.logdebug( "Sent '%s' to COM%d"%(str(toSend).strip('\r'),self.serNum+1) )

			# Retrieve waiting responses
			# TODO: Don't need reading yet, holding off on fully implementing it till needed.
			"""
			if self.ser.readable():
				read = self.ser.read()
				if len(read) == 0:
					pass
					#print "derp"
				else:
					if debug: print "recieved %s from COM %d"%(str(read),self.serNum+1)
					self.recieveLock.acquire()
					self.recieveQueue.append(read)
					self.recieveLock.release()
			"""

	def connect(self):
			comList = []
			comports = serial.tools.list_ports.comports()
			for comport in comports:
					for thing in comport:
							#print thing
							comList.append(thing)
			
			comList = list(set(comList))
			rospy.loginfo( "Attempting to connect to Servotor" )
			for port in comList:
					try:
							ser = serial.Serial(port, baudrate= BAUD_RATE, timeout=2)
							ser.write('V\n')
							result = ser.readline()
							if "SERVOTOR" in result:
									rospy.loginfo( "Connect Successful! Connected on port:",port )
									self.ser = ser
									self.ser.flush()
									self.serOpen = True
									self.serNum = 1
									break
					except:
							pass
			if self.serOpen == False:
				rospy.logwarn( "Connection not yet open. Trying Windows Method.")
				for i in range(1,100):
					try:
						try:
							ser = serial.Serial(i, baudrate=BAUD_RATE, timeout=1)
							#print "ser",i
						except:
							#print "ser",i,"failed"
							raise Exception
						ser.flush()
						time.sleep(0.1)
						ser.write("V\n")
						time.sleep(1)
						readReply = ser.readline()
						#print "read:",readReply
						if "SERVOTOR" in readReply:
							rospy.loginfo( "Connect Successful! Connected on port COM"+str(i+1) )
							ser.flush()
							self.ser = ser
							self.serNum = i
							self.serOpen = True
							break
						else:
							ser.close()
							pass
					except:
						pass
						
	def send( self, message ):
		self.sendLock.acquire()
		self.sendQueue.append( str( message ) )
		rospy.logdebug( "Sending serial message: " + str( message ) )
		self.sendLock.release()

# Software model of the Servator32 board ( might work for other boards? ).
class Controller:
	def __init__(self,servos=32):
		self.serialHandler = serHandler()
		timeout = time.time()
		while not (self.serialHandler.serOpen or (time.time()-timeout > 10.0)):
			time.sleep(0.01)
		if self.serialHandler.serOpen == False:
			rospy.logerr( "Connection to Servotor failed. No robot movement will occur.")
		rospy.loginfo("Initializing servos.")
		self.servos = {}
		for i in range(32):
			self.servos[i]=physical_servo(i,serialHandler=self.serialHandler)
			self.servos[i].kill()
		rospy.loginfo("Servos initialized.")

	def __del__(self):
		del self.serialHandler

	def killAll(self):
		if self.serialHandler.serOpen:
			for servo in self.servos:
				self.servos[servo].kill()
		rospy.loginfo("Killing all servos.")

if __name__ == '__main__':
	conn = Controller()

