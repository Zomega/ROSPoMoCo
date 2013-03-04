#!/usr/bin/env python
import roslib; roslib.load_manifest('ROSPoMoCo')
import rospy
from std_msgs.msg import String

names = {
	'Belly Flop':		['belly flop'],
	'Dance':		['dance'],
	'Get Up':		['get up'],
	'Lean Back':		['lean back'],
	'Kill All Servos':	['limp','go limp'],
	'Move Backward':	['move backward','go backward','walk backward','move back','go back','walk back'],
	'Move Forward':		['move forward','go forward','walk forward'],
	'Point':		['point'],
	'Reset':		['reset'],
	'Rotate Left':		['turn left', 'rotate left'],
	'Rotate Right':		['turn right', 'rotate right'],
	'Set Zero':		['set zero'],
	'Tilt Backward':	['tilt backward','lean backward','tilt back','lean back'],
	'Tilt Forward':		['tilt forward','lean forward'],
	'Tilt Left':		['tilt left','lean left'],
	'Tilt None':		['tilt none','lean none'],
	'Tilt Right':		['tilt right','lean right'],
	'Typing':		['typing','type'],
	'Wave':			['wave'],
	}
	
def get_move_name( message ):
	for name in names:
		for command in names[ name ]:
			if command in message:
				return name
	rospy.logwarn( "No valid move for the command \"" + message +"\"." )
	return None

def interpret_message( data ):
	message = data.data
	moveName = get_move_name( message )
	if message == None:
		return
	pub.publish( String(moveName) )
	
if __name__ == '__main__':
	try:
		pub = rospy.Publisher('moves', String)
		rospy.init_node('translator')
		rospy.Subscriber('recognizer/output', String, interpret_message)
		rospy.loginfo("Starting translator.")
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
