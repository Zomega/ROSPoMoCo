#!/usr/bin/env python
import roslib; roslib.load_manifest('ROSPoMoCo')
import rospy
from std_msgs.msg import String

def interpet_message( data ):
	message = data.data
	moveName = "Get Up"
	rospy.logwarn( message + " -> " + moveName )
	pub.publish( String(moveName) )
	
if __name__ == '__main__':
	try:
		pub = rospy.Publisher('moves', String)
		rospy.init_node('translator')
		rospy.Subscriber('recognizer/output', String, interpet_message)
		rospy.loginfo("Starting translator.")
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
