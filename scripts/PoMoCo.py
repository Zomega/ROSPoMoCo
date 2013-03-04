#!/usr/bin/env python

##############################################################################
#	ROS imports
##############################################################################

import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import String

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

import servotorComm
from robot import hexapod

##############################################################################
#	ROS functions
##############################################################################

def callback(data):
	moveName = data.data
	rospy.loginfo( rospy.get_name() + " received a command to do: " + moveName )
	if moveName in moves:
		move( moveName )
	else:
		rospy.logwarn( "Unknown move \"" + moveName + "\" sent. Ignoring." )
	
##############################################################################
#	PoMoCo functions
##############################################################################

def initMoves():
	
	# Go through the Moves folder to find move files
	moves = []
	for fileName in os.listdir( scriptDirectory + '/Moves' ):
		if os.path.splitext(fileName)[1] == '.py':
			fileName = os.path.splitext(fileName)[0]
			s1 = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', fileName)
			moves.append(s1)
			__builtins__.moves = moves
	
	# Function for running move files
	def move(moveName):
		rospy.loginfo("Performing move: "+moveName)
		moveName = moveName.replace(' ','')
		if moveName in sys.modules:
			reload(sys.modules[moveName])
		else:
			__import__(moveName)
	
	# Make move global.
	__builtins__.move = move

##############################################################################
#	main
##############################################################################

if __name__ == '__main__':

	# Start the ROSNode
	rospy.init_node('PoMoCo')
	
	# Initialize the servo controller
	rospy.loginfo("Starting Servo Driver.")
	controller = servotorComm.Controller()
	
	# Set up the servo controller to run Hexy
	rospy.loginfo("Initializing Hexapod Datastructures.")
	hexy = hexapod(controller)
	__builtins__.hexy = hexy # sets 'hexy' to be a global variable common to all modules
	__builtins__.floor = 60  # this is the minimum level the legs will reach
	
	# Load all the moves and define the global function move( moveName )
	rospy.loginfo("Loading move files.")
	initMoves()
	
	rospy.Subscriber("moves", String, callback)
	
	# Prevent the node from exiting until so ordered.
	rospy.spin()
	
	# The program only reaches this point if the ROSNode has been closed.
	# In this case, we want to clean up and exit.
	del hexy
	del controller
	os._exit(0)
