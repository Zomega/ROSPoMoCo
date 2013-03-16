import time
import math
import serial
import serial.tools.list_ports
import threading

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy

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
