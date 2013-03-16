#!/usr/bin/env python

##############################################################################
#	ROS imports
##############################################################################

import roslib
roslib.load_manifest('ROSPoMoCo')
import rospy
from std_msgs.msg import String
from ROSPoMoCo.msg import pose

##############################################################################
#	PoMoCo imports
##############################################################################

import os
import time
import re
import sys 

# Keep the folders clean for beginners
sys.dont_write_bytecode = True

# This is a hack to ensure we can roslaunch this script.
# TODO: Do something pythonic.
scriptDirectory = os.path.dirname(os.path.realpath(__file__))

# Include the moves folder
sys.path.append( scriptDirectory + '/Moves')
sys.path.append( scriptDirectory + '/PoMoCo')
sys.path.append( scriptDirectory )

from Servo import *
from Hexapod import *

##############################################################################
#	ROS functions
##############################################################################

def callback(data):
	moveName = data.data
	rospy.loginfo( rospy.get_name() + " received a command to do: " + moveName )
	moveName = moveName.replace(" ","")
	if moveName in moves:
		move( moveName )
	else:
		rospy.logwarn( "Unknown move \"" + moveName + "\" sent. Ignoring." )
		
##############################################################################
#	Special Servo
##############################################################################

class SpecialServo( Servo ):
	def setPosition( self, position ):
		Servo.setPosition( self, position )
		pub.publish( hexy.getPose() )
	def detach( self ):
		Servo.detach( self )
		pub.publish( hexy.getPose() )
	def attach( self ):
		Servo.attach( self )
		pub.publish( hexy.getPose() )
	
##############################################################################
#	PoMoCo functions
##############################################################################

def initMoves():
	
	# Go through the Moves folder to find move files
	moves = []
	for fileName in os.listdir( scriptDirectory + '/Moves' ):
		if os.path.splitext(fileName)[1] == '.py':
			moveName = os.path.splitext(fileName)[0]
			moves.append(moveName)
	
	# Function for running move files
	def move(moveName):
		rospy.loginfo("Performing move: "+moveName)
		moveName = moveName.replace(" ","") # TODO: Figure out if this is needed...
		if moveName in sys.modules:
			reload(sys.modules[moveName])
		else:
			__import__(moveName)
	
	# Make everything global.
	__builtins__.moves = moves
	__builtins__.move = move

##############################################################################
#	main
##############################################################################

if __name__ == '__main__':

	# Start the ROSNode
	rospy.init_node('MotionController')
	
	# Set up the servo controller to run Hexy
	rospy.loginfo("Initializing Hexapod Datastructure.")
	
	hexy = Hexapod( [ SpecialServo() for i in range(19)	] )
	__builtins__.hexy = hexy # sets 'hexy' to be a global variable common to all modules
	__builtins__.floor = 60  # this is the minimum level the legs will reach
	
	# Load all the moves and define the global function move( moveName )
	rospy.loginfo("Loading move files.")
	initMoves()
	
	# Subscribe to /moves, where we will receive commands.
	rospy.Subscriber("moves", String, callback)
	# Set up this node to broadcast to /pose
	__builtins__.pub = rospy.Publisher('pose', pose)
	
	# Prevent the node from exiting until so ordered.
	rospy.loginfo("MotionController is entering spin!")
	rospy.spin()
	
	# The program only reaches this point if the ROSNode has been closed.
	# In this case, we want to clean up and exit.
	del hexy
	os._exit(0)
