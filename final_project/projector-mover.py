def main_projector(x, y):
    import cv2
    import numpy as np
    #creating a black square 
    image=np.zeros((500,1000,4),np.uint8)
    cv2.rectangle(image,(x,y),(x + 20, y + 20),(255,127,0),-1)
    #cv2.rectangle(image,(250,150),(500,300),(255,127,0),-1)
    cv2.imshow("rectangle",image)
    cv2.waitKey(1000)
    cv2.destroyWindow("rectangle")

for i in range(10):
    main_projector(i, 0)