import cv2
import numpy as np


cap = cv2.VideoCapture(0)

##cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
##cv2.resizeWindow('frame',640,480)

try:
    while True:
        ret, frame = cap.read() 
        

##        cv2.imshow('frame',frame)

        if frame.mean() != 0:
            print('received frame, mean intensity: ', frame.mean())

##        if cv2.waitKey(1) & 0xFF == ord('q'):
##            break
except KeyboardInterrupt:
    pass
cap.release()
##cv2.destroyAllWindows()
