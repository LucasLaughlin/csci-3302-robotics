# CSCI 3302: Homework 4 -- Object Detection via background subtraction and color filtering
# Using Color Filtering with Blob Detection to find objects using a webcam
# Please do not collaborate on this assignment -- your code must be your own!

LAST_NAME = "Laughlin"

import pdb
import pickle
import random
import copy
import cv2 
import imutils
import numpy as np 


def main():
  cap = cv2.VideoCapture(-1)
  x=0
  while 1:      

    """ GET IMAGE MASK """
    # load the image, convert it to grayscale, and blur it slightly
    ret, image = cap.read()
    gray = image
    if ret == True:
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
 
    # threshold the image, then perform a series of erosions +
    # dilations to remove any small regions of noise
    thresh = cv2.threshold(gray, 160, 200, cv2.THRESH_BINARY)[1]
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=2)

    """ FIND CONTOURS """
    output = image.copy()

    im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    corners=[]

    if len(contours) != 0:
        # the contours are drawn here
        cv2.drawContours(output, contours, -1, 255, 3)

        #find the biggest area of the contour
        c = max(contours, key = cv2.contourArea)

        x,y,w,h = cv2.boundingRect(c)
        # draw the 'human' contour (in green)
        cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

        """ FIND CORNERS"""
        rc = cv2.minAreaRect(c)
        corners = cv2.boxPoints(rc)
        for p in corners:
          pt = (p[0],p[1])
          print pt
          cv2.circle(output,pt,5,(200,0,0),2)    

    #object_positions_list = get_blob_centroids(blobs)
    
    cv2.imshow("Image", image)    # original
    cv2.imshow("gray",gray)       # grayscale + gaussian blur
    cv2.imshow("thresh",thresh)   # mask
    cv2.imshow("Result", output)  # Bounding box + corners on biggest blob
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  
  cv2.waitKey(-1)  # Wait until a key is pressed to exit the program
  cv2.destroyAllWindows() # Close all the windows

if __name__ == '__main__':
  main()
