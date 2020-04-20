#!/usr/bin/env python

# USAGE
# python ***.py --shape-predictor shape_predictor_68_face_landmarks.dat 

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

 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
	help="path to facial landmark predictor")
ap.add_argument("-v", "--video", type=str, default="",
	help="path to input video file")
args = vars(ap.parse_args())
 

# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])

# grab the indexes of the facial landmarks for the mouth 
(mStart, mEnd) = face_utils.FACIAL_LANDMARKS_IDXS["mouth"] # [48,68] whole mouth [61,68] mouth hole
(mhStart, mhEnd) = (61,68) 

# start the video stream thread
print("[INFO] starting video stream thread...")
#vs = FileVideoStream(args["video"]).start()
#fileStream = True
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=True).start()
fileStream = False
time.sleep(1.0)

# loop over frames from the video stream
while True:
	# if this is a file video stream, then we need to check if
	# there any more frames left in the buffer to process
	if fileStream and not vs.more():
		break

	# grab the frame from the threaded video file stream, resize
	# it, and convert it to grayscale
	# channels)
	frame = vs.read()
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
		mouth_open_dist = mouth_openness(mouth)
		#mouth_inner_open_dist = mouth_openness(mouth, outer_lips=False)

		# compute the convex hull for the left and right eye, then
		# visualize each of the eyes
		mouthHull = cv2.convexHull(mouth)
		cv2.drawContours(frame, [mouthHull], -1, (0, 255, 0), 1)


		# draw the computed mouth openness for the frame
		cv2.putText(frame, "Outer Lips Dist: {:.2f}".format(mouth_open_dist), (10, 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
 
	# show the frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
