import time
import math
import serial
import serial.tools.list_ports
import threading

# Load servo architecture...
from Servo import *
from VirtualServo import *

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

from serialHandler import *
from PhysicalServo import *
		
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

