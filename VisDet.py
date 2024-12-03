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

def onTrack1(val):
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

cv2.createTrackbar('Hue Low','myTracker',10,179,onTrack1)
cv2.createTrackbar('Hue High','myTracker',30,179,onTrack2)
cv2.createTrackbar('Sat Low','myTracker',100,255,onTrack3)
cv2.createTrackbar('Sat High','myTracker',255,255,onTrack4)
cv2.createTrackbar('Val Low','myTracker',100,255,onTrack5)
cv2.createTrackbar('Val High','myTracker',255,255,onTrack6)

while True:
   tStart=time.time()
   frame= picam2.capture_array()
   roi = frame[:, 185:975]
   cv2.putText(roi,str(int(fps))+' FPS',pos,font,height,myColor,weight)
   lowerBound=np.array([80,50,69])
   upperBound=np.array([160,90,255])
   frameHSV=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
   myMask=cv2.inRange(frameHSV,lowerBound,upperBound)
   myMaskSmall=cv2.resize(myMask,(int(dispW/2),int(dispH/2)))
   kernel = np.ones((5,5), np.uint8)
   mask = cv2.morphologyEx(myMaskSmall, cv2.MORPH_OPEN, kernel)
   mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
   myObject=cv2.bitwise_and(roi, roi, mask=myMask)
   myObjectSmall=cv2.resize(myObject,(int(dispW/2),int(dispH/2)))
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
               centerarray.append(center)
               print(centerarray)
   
   cv2.imshow("Camera", roi)
   cv2.imshow('my Mask',myMaskSmall)
   cv2.imshow('My Objest',myObjectSmall)
   if cv2.waitKey(1)==ord('q'):
       break
   tEnd=time.time()
   loopTime=tEnd-tStart
   fps=.9*fps + .1*(1/loopTime)
cv2.destroyAllWindows()

