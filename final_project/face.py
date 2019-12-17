
# USAGE
# python detect_face_parts.py --shape-predictor shape_predictor_68_face_landmarks.dat --image images/example_01.jpg 

# import the necessary packages
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2
import math
import rospy
import json
from std_msgs.msg import Float32MultiArray, Empty, String, Int16

def dist(x1,y1,x2,y2):
	return (math.sqrt((x2-x1)**2+(y2-y1)**2))
# construct the argument parser and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-p", "--shape-predictor", required=True,
	#help="path to facial landmark predictor")
#ap.add_argument("-i", "--image", required=True,
	#help="path to input image")
#args = vars(ap.parse_args())

# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# load the input image, resize it, and convert it to grayscale
#image = cv2.imread(args["image"])
#image = imutils.resize(image, width=500)
#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# detect faces in the grayscale image
#rospy.init_node('project', anonymous=True)

#publisher_camera = rospy.Publisher('/project/camera', Float32MultiArray)

#trying to get fame with 10% so we need to set boarders 5%  to the center. can change if needed

cap = cv2.VideoCapture(-1)
# loop over the face detections
while 1:
	ret, image = cap.read()
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
	
	rects = detector(gray, 1)
	goodface=0
	center=(0,0)
	leyew=0
	reyew=0
	leyelh=0
	leyerh=0
	reyelh=0
	reyerh=0
	r=(0,0)
	l=(0,0)
	h=image.shape[0]
	w=image.shape[0]
	top=h*.55
	bottom=h*.45
	left=w*.45
	right=w*.55
	
	
	for (i, rect) in enumerate(rects):
		# determine the facial landmarks for the face region, then
		# convert the landmark (x, y)-coordinates to a NumPy array
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)
		
		
		#print(shape)
		# loop over the face parts individually
		for (name, (i, j)) in face_utils.FACIAL_LANDMARKS_IDXS.items():
			# clone the original image so we can draw on it, then
			# display the name of the face part on the image
			#clone = image.copy()
			#cv2.putText(image, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

			# loop over the subset of facial landmarks, drawing the
			# specific face part
			#print (name)
			if ((name == "left_eye") | (name == "right_eye")):

				for (x, y) in shape[i:j]:
					cv2.circle(image, (x, y), 1, (0, 0, 255), -1)
					#print("eyes")
					#print(x,y)
					#print("done")
					
			if (name == "right_eye"):
				#print("right")
				#print(shape[i:j])
				r=(shape[i:j][5][0],shape[i:j][5][1])
				reyew=dist(shape[i:j][0][0],shape[i:j][0][1],shape[i:j][3][0],shape[i:j][3][1])
				reyelh=dist(shape[i:j][1][0],shape[i:j][1][1],shape[i:j][5][0],shape[i:j][5][1])
				reyerh=dist(shape[i:j][2][0],shape[i:j][2][1],shape[i:j][4][0],shape[i:j][4][1])
			if (name == "left_eye"):
				#print("left")
				#print(shape[i:j])
				#(x,y)=shape[i:j][1]
				#cv2.circle(image, (x, y), 1, (255, 0, 0), -1)
				l=(shape[i:j][0][0],shape[i:j][0][1])
				leyew=dist(shape[i:j][0][0],shape[i:j][0][1],shape[i:j][3][0],shape[i:j][3][1])
				leyelh=dist(shape[i:j][1][0],shape[i:j][1][1],shape[i:j][5][0],shape[i:j][5][1])
				leyerh=dist(shape[i:j][2][0],shape[i:j][2][1],shape[i:j][4][0],shape[i:j][4][1])

			# extract the ROI of the face region as a separate image
			#(x, y, w, h) = cv2.boundingRect(np.array([shape[i:j]]))
			#roi = image[y:y + h, x:x + w]
			#roi = imutils.resize(roi, width=250, inter=cv2.INTER_CUBIC)

			# show the particular face part
			#cv2.imshow("ROI", roi)
			#cv2.imshow("Image", image)
			#cv2.waitKey(0)

		# visualize all facial landmarks with a transparent overlay
		#output = face_utils.visualize_facial_landmarks(image, shape)
	#print(l,r)
	#print(r[0],r[1])
	if ((r[0]!=0) & (r[1]!=0) & (l[0]!=0) & (l[1]!=0) ):
		#print(l,r)
		
		center=((r[0]+l[0])/2,(r[1]+l[1])/2)
		
		#print(center)
	if ((leyew*.9 <=reyew<=leyew*1.1) & (leyew!=0) & (leyew!=0)):
		if (leyerh*.9 <=reyerh<=leyerh*1.1):
			if (leyelh*.9 <=reyelh<=leyelh*1.1):
				goodface=1
	if (goodface):
		print("good", center)
		msg = Float32MultiArray()
		msg.data=[0.0,0.0]
		if (bottom > center[1]):
			if (left> center[0]):
				print("move up +right")
				msg.data=[1.0,1.0]
				
			elif (right>center[0]):
				print("move up +left ")
				msg.data=[1.0,-1.0]
			else:
				msg.data=[1.0,0.0]
                		print("move up")
		elif (top < center[1]):
			if (left>center[0]):
				print("move down +right")
				msg.data=[-1.0,1.0]
			if (right>center[0]):
				print("move down +left ")
				msg.data=[-1.0,-1.0]
			else:
				msg.data=[-1.0,0.0]
                		print("move down")
		else:
			msg.data=[0.0,0.0]
		
		#publisher_camera.publish(msg)
	cv2.imshow("Image", image)
	#cv2.waitKey(0)
	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break
cap.release()
cv2.destroyAllWindows()
