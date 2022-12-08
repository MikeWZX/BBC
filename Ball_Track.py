import cv2 as cv
import numpy as np
import serial
import time


videoCapture = cv.VideoCapture(1) #Turns on cam
videoCapture.set(cv.CAP_PROP_BUFFERSIZE, 2)
prevCircle = None
dist = lambda x1, y1, x2, y2: (x1-x2)**2 +(y1-y2)**2 #Function to calculate norm 2 dist

#Connect to serial port
ser = serial.Serial('COM3', 9600, timeout=1)
maxtime = 0

# frame dimensions
cam_len = np.array([640,480])        # x length (pixels), y length (pixels)
plat_len = np.array([0.160,0.120])   # x length (m), y length (m) <---Unknown

# function to transform points from camera frame to servo platform frame
def transform(obj):
    # scale
    obj_scaled = obj * plat_len/cam_len
    # translate
    obj_tra = obj_scaled - plat_len/2
    # flip just y
    obj_flip = np.array([obj_tra[0],-obj_tra[1]])
    # rotate
    angle = np.pi/6
    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    obj_final = R @ obj_flip
    # return final transformed object
    return obj_final

# determine position of ball continuously
while True:
    timestart = time.time()
    ret, Frame = videoCapture.read()
    if not ret: break
    
    blurFrame = cv.GaussianBlur(Frame, (17,17), 0)
    #grayFrame = cv.cvtColor(blurFrame, cv.COLOR_BGR2GRAY)
    grayFrame = blurFrame[:,:,1]
    
    circles = cv.HoughCircles(grayFrame, cv.HOUGH_GRADIENT_ALT, 1.5, 30, param1 = 200, param2 = 0.85, minRadius = 50, maxRadius = 80)
    #dp 1.0-1.4, minDist, param 1 = sensitivity, param 2 = accuracy
    # chosen = None
    # print(circles)
    if circles is not None:
        circles = np.uint16(np.around(circles))[0]
        # print(circles)
        circles = circles[:,:2] # strip circles to only x and y
        if prevCircle is not None:
            diff = (circles-prevCircle)**2
            chosen = circles[np.argmin(diff[:,1]-diff[:,0])]
        else:
            chosen = circles[0]
        # chosen = None
        # for i in circles[0,:]:
        #     if chosen is None: chosen = i
        #     #If prev circle exists, choose 
        #     if prevCircle is not None:
        #         if dist(chosen[0],chosen[1],prevCircle[0],prevCircle[1]) <= dist(i[0],i[1],prevCircle[0],prevCircle[1]):
        #             chosen = i
        # print(chosen[2])
        cv.circle(Frame, (chosen[0],chosen[1]),1 ,(255,8,255), 3)

        
        prevCircle = chosen  # prevCircle should just be 2 by 2
  
        ball_coords = transform(chosen[:2])
        #Write to arduino msg type "x_coordinate, y_coordinate" followed by new line
        ser.write("{:.2f},{:.2f}\n".format(ball_coords[0],ball_coords[1]).encode()) 
        # print(ser.readline().strip())

    cv.imshow("circles",Frame)
    
    difftime = time.time()-timestart
    timestart = time.time()
    #maxtime = max(difftime, maxtime)
    #print(difftime)

    if cv.waitKey(1) & 0xFF == ord('q'): break

ser.close()
videoCapture.release()
cv.destroyAllWindows()