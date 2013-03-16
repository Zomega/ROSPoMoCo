import time
import math
import serial
import serial.tools.list_ports
import threading

# Load servo architecture...
from Servo import *
from VirtualServo import *
from PhysicalServo import *

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy

startTime = time.clock()
serialSends = []

BAUD_RATE = 9600

# Used by the GUI to ensure responsiveness while running moves.
# Depreciated, ROS can handle the specifics using topics...
# TODO: Remove once GUI, etc has been spun off.
# TODO: Move name translator node should keep a Queue?
class runMovement(threading.Thread):

	def __init__(self,function,*args):
		threading.Thread.__init__(self)
		self.function=function
		self.args = args
		self.start()

	def run(self):
		self.function(*self.args)

#TODO: Rewrite this class. Some of it is a mess.
class SerialHandler(threading.Thread):

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
			# If there are messages waiting, send the first one...
			if(len(self.sendQueue)>0):
			
				self.sendLock.acquire()
				toSend = self.sendQueue.pop(0)
				self.sendLock.release()
				
				sendTime = time.clock()-startTime
				serialSends.append([float(sendTime),str(toSend)])
				time.sleep(0.003)
				# TODO: Determine if the double check of self.serOpen is needed.
				if self.serOpen:
					if self.ser.writable:
						if self.serOpen:
							toSend = toSend
							self.ser.write( str(toSend) )
							rospy.logdebug( "Sent '%s' to COM%d"%(str(toSend).strip('\r'),self.serNum+1) )
			
			else:
				time.sleep(0.01) # Keeps infinite while loop from wasting processor cycles.

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
		
# Software model of the Servator32 board.
class Servotor32:
	# Initialize the software. First a serial connection must be established.
	# This may take up to 10 seconds. The Servotor will create it's own servo
	# objects, which can be extracted with getServo.
	# The rest should handle itself.
	def __init__( self ):
		# Open a serial connection to the board, if possible.
		self.serialHandler = SerialHandler()
		starttime = time.time()
		while not ( self.serialHandler.serOpen or (time.time() - starttime > 10.0) ):
			# Make sure we use a minimal amount of processor time.
			# In the worst case, we will wait a full tenth of a second after the connection opens.
			time.sleep(0.1)
		if self.serialHandler.serOpen == False:
			rospy.logerr( "Connection to Servotor32 board failed. No robot movement will occur.")
			
		#TODO: This is not a very good name.
		self.servos = { physicalServo( self ) : i for i in range(32) }
		
		# Ensure the physical servos relax.
		self.detachAll()
			
	def getServo( self, i ):
		if i >= 32 or i < 0:
			rospy.logerr( "Queried the Servotor32 for servo " + str(i) + ". This is an invalid index. " )
			return None
		# The result should be in our dictionary. We just need to look it up.
		for serv in self.servos:
			if self.servos[ serv ] == i:
				return serv
	
	# Detach (relax) all connected servos. It's generally bad
	# to leave the servos running for long periods of time.		
	def detachAll( self ):
		for serv in self.servos:
			serv.detach()
			
	# Allows a servo to notify the Servotor32 object that the physical servo should be update.
	def notify( self, serv ):
		try:
			# If the servo should be tensed...
			if serv.isAttached():	
				# Send the message the serial handler
				self.serialHandler.send( "#%dP%.4dT0\r"%( self.servos[ serv ], int( serv.__timing__ ) ) )
			# If the servo should be relaxed...
			else:
				#TODO what exactly goes here? Seems to throw an awful lot of exceptions.
				self.serialHandler.send( "#%.4dL\r"%( self.servos[ serv ], int( serv.__timing__ ) ) )
				
		# If the an exception is thrown, then keep going...
		# TODO: what is the use case here precisely?
		except Exception as e:
			pass#rospy.logwarn( "Attempts to use serial threw an exception: \"" + str( e ) + "\". Continuing..." )

