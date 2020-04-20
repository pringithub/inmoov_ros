#!/usr/bin/env python

# USAGE


import rospy
from inmoov_perception.msg import MouthOpenness
from sensor_msgs.msg import JointState
MOUTH_OPENNESS_TOPIC = 'mouth_openness'
JOINT_STATES_TOPIC = 'joint_states' 

MIN_MOUTH_OPENNESS=0.3
MAX_MOUTH_OPENNESS=0.8
MIN_JAW_JOINT=0.0
MAX_JAW_JOINT=0.09


def interpolate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


class TELE_MOVE_JAW:
    def __init__(self):
	rospy.loginfo('Starting tele_move_jaw node')

	self.mouth_openness_sub = rospy.Subscriber(MOUTH_OPENNESS_TOPIC, MouthOpenness, self.cb, queue_size=1)
	self.joint_states_pub = rospy.Publisher(JOINT_STATES_TOPIC, JointState, queue_size=1) 


	self.js = JointState()
	self.js.header.stamp = rospy.Time.now()
	self.js.name = ["waist_pan_joint", "waist_roll_joint", 
			"head_roll_joint", "head_tilt_joint", "head_pan_joint",
  			"jaw_joint", 
			"eyes_tilt_joint", "eyes_pan_joint", "l_eye_pan_joint", 
			"r_shoulder_lift_joint", "r_upper_arm_roll_joint", 
			"r_elbow_flex_joint", "r_shoulder_out_joint", 
			"r_thumb1_joint", "r_thumb_joint", "r_thumb3_joint", 
			"r_index1_joint", "r_index_joint", "r_index3_joint", 
			"r_middle1_joint", "r_middle_joint", "r_middle3_joint", 
			"r_ring1_joint", "r_ring_joint", "r_ring3_joint", "r_ring4_joint",
  			"r_pinky1_joint", "r_pinky_joint", "r_pinky3_joint", "r_pinky4_joint", 
			"r_wrist_roll_joint",
  			"l_shoulder_lift_joint", "l_upper_arm_roll_joint", 
			"l_elbow_flex_joint", "l_shoulder_out_joint",
  			"l_thumb1_joint", "l_thumb_joint", "l_thumb3_joint", 
			"l_index1_joint", "l_index_joint", "l_index3_joint",
  			"l_middle1_joint", "l_middle_joint", "l_middle3_joint", 
			"l_ring1_joint", "l_ring_joint", "l_ring3_joint", "l_ring4_joint", 
			"l_pinky1_joint", "l_pinky_joint", "l_pinky3_joint", "l_pinky4_joint", 
			"l_wrist_roll_joint"]
	self.js.position = [0.0, 0.0, 0.0, 0.0, -5.2359878085939116e-05, 0.0, 0.0, 0.0, 0.0, -0.7853981633964999, 0.0, -0.8290313946994999, -0.56713601379099, -2.6179938571065792e-05, -3.4906584761421056e-05, -3.4906584761421056e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, 8.726646231664581e-06, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, 8.726646231664581e-06, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -1.570796326795, -0.7853981633964999, 0.0, -0.8726646259995, 0.56713601379099, 2.6179938571065792e-05, -3.4906584761421056e-05, -3.4906584761421056e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-06, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-06, -8.726646231664581e-05, -8.726646231664581e-05, -8.726646231664581e-05, 1.570796326795]
	self.js.velocity = []
	self.js.effort = []

	self.jaw_idx = 5

	#print(self.js)
	self.joint_states_pub.publish(self.js)	
	rospy.loginfo('Published center joint_states config')

    def cb(self, data):
	# mouth openness normally between 0.2 and 0.8	

	mouth_openness = data.mouth_openness	
	if   mouth_openness < MIN_MOUTH_OPENNESS: mouth_openness = MIN_MOUTH_OPENNESS 
	elif mouth_openness > MAX_MOUTH_OPENNESS: mouth_openness = MAX_MOUTH_OPENNESS 

	joint_pos = interpolate(mouth_openness,
				MIN_MOUTH_OPENNESS, MAX_MOUTH_OPENNESS,
				MIN_JAW_JOINT, MAX_JAW_JOINT) 
	print(joint_pos)
	self.js.header.seq += 1
	self.js.header.stamp = rospy.Time.now() 
	self.js.position[self.jaw_idx] = joint_pos 

	self.joint_states_pub.publish(self.js)	

if __name__=='__main__':
    rospy.init_node('tele_move_jaw')
    d = TELE_MOVE_JAW()
    rospy.spin()

 
