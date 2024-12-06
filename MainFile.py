import cv2
from picamera2 import Picamera2
import time
import numpy as np
import mecademicpy.robot as mdr
robot = mdr.Robot()
robot.Connect(address='192.168.0.100', enable_synchronous_mode=False)
robot.ResetError()
robot.ResumeMotion()
robot.ActivateAndHome()
robot.WaitHomed()
robot.MoveJoints(0,-60,60,0,0,0)
robot.WaitIdle()
picam2 = Picamera2()
dispW=1280
dispH=1024
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

cv2.createTrackbar('Hue Low','myTracker',10,179,onTrack1) #Creates the sliding number bars in the window
cv2.createTrackbar('Hue High','myTracker',30,179,onTrack2)
cv2.createTrackbar('Sat Low','myTracker',100,255,onTrack3)
cv2.createTrackbar('Sat High','myTracker',255,255,onTrack4)
cv2.createTrackbar('Val Low','myTracker',100,255,onTrack5)
cv2.createTrackbar('Val High','myTracker',255,255,onTrack6)
# Look at VisDet.py for all documentation until Line 103
while True:
   tStart=time.time()
   frame= picam2.capture_array()
   roi = frame[11:720, 185:975]
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
   if cv2.waitKey(1) == ord('w'): # Waits for an input of 'w'
      actioncent = centerarray # Makes a list of the current centers detected
      robot.ResumeMotion()
      for i in range(len(actioncent)): # Loops for an amount of time equal to the length of the List
        if actioncent[i][0] >= 300 and actioncent[i][0] <= 350:
         print("skipped")
         continue
        actualx = ((actioncent[i][0]/740)*600)-220 #Converts Camera location of center into Robot Coordinates DOESN'T WORK RIGHT, PROBABLY ((actioncent[i][0]/790)*600'(2*300)')-300
        actualy = ((actioncent[i][1]/719)*400)-150 #As above. The y and x axis flips between the robot coords and the camera pixels ((actioncent[i][1]/719)*540'(2*270)')-270
        print(actualx) #Troubleshootingw
        print(actualy) #Troubleshooting
        roboty = -actualx # Since the robot and camera's axes are flipped, this flips them with numbers
        robotx = -actualy
        robot.MovePose(robotx, roboty, 200, -180, 0, 180) # This is the sticking point, it should move the robot to a position above the detected objects, but it doesn't
        robot.WaitIdle()
        robot.MoveLin(robotx, roboty, 50, -180, 0, 180) # Lowers the end effector to the rock, theoretically
        robot.WaitIdle() # Waits until the robot has stopped
        robot.MovePose(150, 100, 300, 60, 90, 0) #Moves Robot to a neutral point
        robot.WaitIdle()
# Theoretically, in the above for loop, you will need to add a close gripper function, move the robot to the drop off point, and open gripper, then just loop it

   tEnd=time.time()
   loopTime=tEnd-tStart
   fps=.9*fps + .1*(1/loopTime)
cv2.destroyAllWindows() # Closes all opencv windows
robot.DeactivateRobot() # Starts shutting the robot down
robot.WaitDeactivated() # Waits until the robot is deactivated
robot.Disconnect() # Fully disconnects from the robot

