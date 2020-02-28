from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import numpy as np
import cv2
import imutils
import copy
import random
import pickle
import pdb
LAST_NAME = "Laughlin"


def main():
    cap = cv2.VideoCapture(2)
    x = 0
    while 1:

        """ GET IMAGE MASK """
        # load the image, convert it to grayscale, and blur it slightly
        ret, image = cap.read()
        width = image.shape[1]
        height = image.shape[0]
        height_adjust = 0.4
        width_adjust = 0.4
        y = (int)((height - height*height_adjust)*0.5 )-25
        h =  (int)(height*height_adjust )
        x = (int)((width - width*width_adjust)*0.5 ) - 23
        w = (int)(width * width_adjust )
        
        image = image[y:y+h, x:x+w]    
        image = cv2.resize(image, (1920, 1080), interpolation=cv2.INTER_AREA)
        gray = image
        if ret == True:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # threshold the image, then perform a series of erosions +
        # dilations to remove any small regions of noise
        thresh = cv2.threshold(gray, 140, 170, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)

        """ FIND CONTOURS """
        output = image.copy()
        contours, hierarchy = [0, 0]
        if cv2.getVersionMajor() in [2, 4]:
            contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            # OpenCV 3 case
            im2, contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corners = []
        dstTri = []

        if len(contours) != 0:
            # the contours are drawn here
            cv2.drawContours(output, contours, -1, 255, 3)

            # find the biggest area of the contour
            c = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)
            # draw the 'human' contour (in green)
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 2)

            """ FIND CORNERS"""
            rc = cv2.minAreaRect(c)
            corners = cv2.boxPoints(rc)

            #blue is bottom
            #green is left
            #red is top
            # grey is right

            cv2.circle(
                output, (corners[0][0], corners[0][1]), 5, (200, 0, 0), 2)
            # bottom right
            cv2.circle(
                output, (corners[1][0], corners[1][1]), 5, (0, 200, 0), 2)
            # top left
            cv2.circle(
                output, (corners[2][0], corners[2][1]), 5, (0, 0, 200), 2)
            # top right
            cv2.circle(
                output, (corners[3][0], corners[3][1]), 5, (100, 100, 100), 2)

            print(corners)
            print(corners[2])
            print(corners[2, 0])
            print(corners[2, 1])
            if corners[1][1] > corners[3][1]:
                print(corners[2][0])
                dstTri = np.array(
                    [[corners[2][0],corners[2][1]], [corners[3][0],corners[3][1]], [corners[1][0],corners[1][1]]]).astype(np.float32)
            else:
                dstTri = np.array([
                    [corners[1][0],corners[1][1]], [corners[2][0],corners[2][1]], [corners[0][0],corners[0][1]]]).astype(np.float32)

        cv2.imshow("Image", image)    # original
        cv2.imshow("gray", gray)       # grayscale + gaussian blur
        cv2.imshow("thresh", thresh)   # mask
        cv2.imshow("Result", output)  # Bounding box + corners on biggest blob

        """ PROJECTOR STUFF """
        src = cv2.imread('pl-boraham.jpg')

        if src is None:
            print('image not avaible')
            exit(0)
            
        src = cv2.resize(src, (1920, 1080), interpolation=cv2.INTER_AREA)
        # [Load the image]
        srcTri = np.array([[0, 0], [src.shape[1] - 1, 0], [0, src.shape[0] - 1]]).astype(np.float32)

        p = 0
        name = 'warp'
        warp_mat = cv2.getAffineTransform(srcTri, dstTri)
        warp_dst = cv2.warpAffine(src, warp_mat, (src.shape[1], src.shape[0]))
        cv2.imshow(name, warp_dst)
        cv2.waitKey(20)
        
        
        cv2.namedWindow(name)
        #cv2.imshow(name, src)
        cv2.moveWindow(name, 0, 0)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()  # Close all the windows
            break


if __name__ == '__main__':
    main()
