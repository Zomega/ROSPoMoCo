#!/usr/bin/env python
import roslib; roslib.load_manifest('ROSPoMoCo')
import rospy
from std_msgs.msg import String


def talker():
	pub = rospy.Publisher('moves', String)
	rospy.init_node('talker')
	rospy.loginfo("Starting talker")
	while not rospy.is_shutdown():
		string = 'Reset'
		rospy.loginfo(rospy.get_name() + " is sending a command to do: " + string)
		pub.publish(String(string))
		rospy.sleep(4.0)


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
