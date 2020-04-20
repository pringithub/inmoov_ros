#!/usr/bin/env python

# TODO load filename into rosparams and do rospy.loadparam within code

# USAGE
# ---- arparse doesn't work well with ros .. hardcoded file for now ... python ****.py --shape-predictor shape_predictor_68_face_landmarks.dat

# import the necessary packages
from scipy.spatial import distance as dist
from imutils.video import FileVideoStream
from imutils.video import VideoStream
from imutils import face_utils
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from inmoov_perception.msg import MouthOpenness
COLOR_IMAGE_TOPIC = '/camera/color/image_raw'
DEBUG_IMAGE_TOPIC = '/debug_jaw_openness'
MOUTH_OPENNESS_TOPIC = 'mouth_openness'
 
# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-p", "--shape-predictor", required=True,
#	help="path to facial landmark predictor")
#ap.add_argument("-v", "--video", type=str, default="",
#	help="path to input video file")
#args = vars(ap.parse_args())
 

# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
rospy.loginfo("loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('/home/lxu9/inmoov_ros/src/inmoov_ros/inmoov_perception/scripts/models/shape_predictor_68_face_landmarks.dat')
#predictor = dlib.shape_predictor(args["shape_predictor"])
rospy.loginfo("loading facial landmark predictor... done")

# grab the indexes of the facial landmarks for the mouth 
(mStart, mEnd) = face_utils.FACIAL_LANDMARKS_IDXS["mouth"] # [48,68] whole mouth [61,68] mouth hole
(mhStart, mhEnd) = (61,68) 


def mouth_openness(mouth, outer_lips=True):
# use idx=3,9 for outer lips, idx=14,18 for inner lips
# use idx=0,6 for outer lip edges, idx=12,16 for inner lip edges

    if outer_lips:
        a = dist.euclidean(mouth[3], mouth[9])
        b = dist.euclidean(mouth[0], mouth[6])
	r = a/b
    else:
        a = dist.euclidean(mouth[14], mouth[18])
        b = dist.euclidean(mouth[12], mouth[16])
	r = a/b
    return r 


class MOUTH_OPENNESS_DETECTOR:
    def __init__(self):
        self.cv_bridge = CvBridge()
	self.img_sub = rospy.Subscriber(COLOR_IMAGE_TOPIC, Image, self.image_cb, queue_size=1)
	self.debug_img_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
	rospy.loginfo("Starting detector")

	self.mouth_openness = -1
	self.mouth_openness_pub = rospy.Publisher(MOUTH_OPENNESS_TOPIC, MouthOpenness, queue_size=1)

    def image_cb(self, img):

	try:
	    frame = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
	except Exception as e:
	    rospy.logwarn(e)
	    rospy.logwarn("Unable to convert image to cv2")
	    pass

	frame = imutils.resize(frame, width=450)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the grayscale frame
	rects = detector(gray, 0)

	# loop over the face detections
	for rect in rects:
	    # determine the facial landmarks for the face region, then
	    # convert the facial landmark (x, y)-coordinates to a NumPy
	    # array
	    shape = predictor(gray, rect)
	    shape = face_utils.shape_to_np(shape)

	    # extract the left and right eye coordinates, then use the
	    # coordinates to compute the eye aspect ratio for both eyes
	    mouth = shape[mStart:mEnd]
	    self.mouth_openness = mouth_openness(mouth)
	    #mouth_inner_open_dist = mouth_openness(mouth, outer_lips=False)

	    # compute the convex hull for the left and right eye, then
	    # visualize each of the eyes
	    mouthHull = cv2.convexHull(mouth)
	    cv2.drawContours(frame, [mouthHull], -1, (0, 255, 0), 1)


	    # draw the computed mouth openness for the frame
	    cv2.putText(frame, "Outer Lips Openness: {:.2f}".format(self.mouth_openness), (10, 30),
		cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

	self.mouth_openness_pub.publish(self.mouth_openness)

	try:
	    debug_img = self.debug_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
	except CvBridgeError as e:
	    rospy.logwarn(e)
	    pass

if __name__=='__main__':
    rospy.init_node('mouth_openness_detector')
    d = MOUTH_OPENNESS_DETECTOR()
    rospy.spin()

 
