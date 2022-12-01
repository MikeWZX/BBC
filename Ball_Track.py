import cv2 as cv
import  numpy as np
import serial
import time

videoCapture = cv.VideoCapture(1) #Turns on cam
prevCircle = None
dist = lambda x1, y1, x2, y2: (x1-x2)**2 +(y1-y2)**2 #Function to calculate norm 2 dist
#Connect to serial port
ser = serial.Serial('/dev/cu.usbmodem144401', 115200, timeout=1)
#time.sleep(2)
maxtime = 0

while True:
    timestart = time.time()
    ret, Frame = videoCapture.read()
    if not ret: break
    
    blurFrame = cv.GaussianBlur(Frame, (17,17), 0)
    grayFrame = cv.cvtColor(blurFrame, cv.COLOR_BGR2GRAY)
    #grayFrame = blurFrame[:,:,1]
    
    circles = cv.HoughCircles(grayFrame, cv.HOUGH_GRADIENT_ALT, 1.5, 30, param1 = 200, param2 = 0.85, minRadius = 30, maxRadius = 200)
    #dp 1.0-1.4, minDist, param 1 = sensitivity, param 2 = accuracy
    #Returns (X, y, radius)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0,:]:
            if chosen is None: chosen = i
            #If prev circle exists, choose 
            if prevCircle is not None:
                if dist(chosen[0],chosen[1],prevCircle[0],prevCircle[1]) <= dist(i[0],i[1],prevCircle[0],prevCircle[1]):
                    chosen = i

        cv.circle(Frame, (chosen[0],chosen[1]),1 ,(0,100,100), 3)
        cv.circle(Frame, (chosen[0],chosen[1]), chosen[2], (255,8,255), 3)
        prevCircle = chosen
        
        #print(chosen)
        #Write to arduino msg type "<x_coordinate, y_coordinate>"
        ser.write("<{:.2f},{:.2f}>".format(chosen[0],chosen[1]).encode()) 
        print(ser.readline().strip())

    cv.imshow("circles",Frame)
    
    difftime = time.time()-timestart
    timestart = time.time()
    #maxtime = max(difftime, maxtime)
    print(difftime)


    if cv.waitKey(1) & 0xFF == ord('q'): break

ser.close()
videoCapture.release()
cv.destroyAllWindows()