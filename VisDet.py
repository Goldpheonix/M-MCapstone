import cv2
from picamera2 import Picamera2
import time
import numpy as np
picam2 = Picamera2()
dispW=1920
dispH=1080
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
#picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)

def onTrack1(val): #Functions to create the trackbars
   global hueLow
   hueLow=val
   print('Hue Low',hueLow)
def onTrack2(val):
   global hueHigh
   hueHigh=val
   print('Hue High',hueHigh)
def onTrack3(val):
   global satLow
   satLow=val
   print('Sat Low',satLow)
def onTrack4(val):
   global satHigh
   satHigh=val
   print('Sat High',satHigh)
def onTrack5(val):
   global valLow
   valLow=val
   print('Val Low',valLow)
def onTrack6(val):
   global valHigh
   valHigh=val
   print('Hue Low',valHigh)


cv2.namedWindow('myTracker')

cv2.createTrackbar('Hue Low','myTracker',10,179,onTrack1) #Puts the trackbars into the Window
cv2.createTrackbar('Hue High','myTracker',30,179,onTrack2)
cv2.createTrackbar('Sat Low','myTracker',100,255,onTrack3)
cv2.createTrackbar('Sat High','myTracker',255,255,onTrack4)
cv2.createTrackbar('Val Low','myTracker',100,255,onTrack5)
cv2.createTrackbar('Val High','myTracker',255,255,onTrack6)

while True:
   tStart=time.time()
   frame= picam2.capture_array() # Finds the first frame from the camera
   roi = frame[:, 185:975] # Cuts the Camera Frame into the proper calibration for the baseboard. May need to be adjusted based on how the frame gets moved.
   cv2.putText(roi,str(int(fps))+' FPS',pos,font,height,myColor,weight) # Puts the FPS calculation into the top left
   lowerBound=np.array([hueLow,satLow,valLow]) # Puts the values from the trackbars into the HSV mask
   upperBound=np.array([hueHigh,satHigh,valHigh]) # Puts the High values into the mask
   frameHSV=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV) # Uses the HSV Mask to overlay the camera input
   myMask=cv2.inRange(frameHSV,lowerBound,upperBound) # Masks the Camera Frame, and allows the selected Range of HSV to Allow the specific colors through
   myMaskSmall=cv2.resize(myMask,(int(dispW/2),int(dispH/2))) # Makes the Window Smaller
   kernel = np.ones((5,5), np.uint8) # Creates an Identity Matrix that is 5x5
   mask = cv2.morphologyEx(myMaskSmall, cv2.MORPH_OPEN, kernel) # Blurs the image
   mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # Blurs it again
   myObject=cv2.bitwise_and(roi, roi, mask=myMask) # Grabs the image with masks
   myObjectSmall=cv2.resize(myObject,(int(dispW/2),int(dispH/2))) # Makes the Window Smaller
   cnts = cv2.findContours(myMaskSmall.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2] #generates number of contiguous "1" pixels
   center = None # create a variable for x, y location of targe
   centers = []
   if len(cnts) > 0:   # begin processing if there are "1" pixels discovered
           #c = max(cnts, key=cv2.contourArea)          # return the largest target area
       centerarray = []
       for i in range(len(cnts)): #Added to test multiple contours
           moments = cv2.moments(cnts[i])
           #centers.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
           #c = cnts
           #c1 = max(c, key=cv2.contourArea)
           x,y,w,h = cv2.boundingRect(cnts[i])               # Get bounding rectangle (x,y,w,h) of the largest contour
           center = (int(x+0.5*w), int(y+0.5*h))       # defines center of rectangle around the largest target area
           if 0.5*w > 6:
               cv2.rectangle(myMaskSmall, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 255), 2)  # draw bounding box
               cv2.circle(myMaskSmall, center, 3, (0, 0, 0), -1) # draw a dot on the target center
               cv2.putText(myMaskSmall,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.2,(0,0,0),2,cv2.LINE_AA)
               cv2.putText(myMaskSmall,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.2,(255,255,255),1,cv2.LINE_AA)
               centerarray.append(center) # Adds the centers
               print(centerarray) # Troubleshooting Print
   
   cv2.imshow("Camera", roi) # Makes a window for the raw camera view
   cv2.imshow('my Mask',myMaskSmall) # Makes a window for the Mask
   cv2.imshow('My Objest',myObjectSmall) # Makes a window for the Mask with Color
   if cv2.waitKey(1)==ord('q'): # Waits for an input of 'q' to break out of loop
       break
   tEnd=time.time()
   loopTime=tEnd-tStart
   fps=.9*fps + .1*(1/loopTime)
cv2.destroyAllWindows() # Closes all opencv windows

