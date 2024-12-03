import cv2
from picamera2 import Picamera2
import time
import numpy as np
import mecademicpy.robot as mdr

robot = mdr.Robot()
robot.Connect(address='192.168.0.100', enable_synchronous_mode=False)
robot.ActivateAndHome()
robot.WaitHomed()
robot.MoveJoints(0, -60, 60, 0, 0, 0)

picam2 = Picamera2()
dispW = 1280
dispH = 1024
picam2.preview_configuration.main.size = (dispW, dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

fps = 0
pos = (30, 60)
font = cv2.FONT_HERSHEY_SIMPLEX
height = 1.5
weight = 3
myColor = (0, 0, 255)

def onTrack1(val): global hueLow; hueLow = val
def onTrack2(val): global hueHigh; hueHigh = val
def onTrack3(val): global satLow; satLow = val
def onTrack4(val): global satHigh; satHigh = val
def onTrack5(val): global valLow; valLow = val
def onTrack6(val): global valHigh; valHigh = val

cv2.namedWindow('myTracker')
cv2.createTrackbar('Hue Low', 'myTracker', 10, 179, onTrack1)
cv2.createTrackbar('Hue High', 'myTracker', 30, 179, onTrack2)
cv2.createTrackbar('Sat Low', 'myTracker', 100, 255, onTrack3)
cv2.createTrackbar('Sat High', 'myTracker', 255, 255, onTrack4)
cv2.createTrackbar('Val Low', 'myTracker', 100, 255, onTrack5)
cv2.createTrackbar('Val High', 'myTracker', 255, 255, onTrack6)

def move_robot_safely(x, y, z, rx, ry, rz):
    try:
        # Check if the pose is reachable (optional, depending on your robot's API)
        if robot.IsPoseReachable(x, y, z, rx, ry, rz):
            robot.MovePose(x, y, z, rx, ry, rz)
            robot.WaitIdle()  # Wait for robot to finish the move
        else:
            print(f"Pose ({x}, {y}, {z}, {rx}, {ry}, {rz}) is out of reach!")
    except Exception as e:
        print(f"Error during robot movement: {e}")
        robot.Disconnect()  # Disconnect robot on error (if required)
        return False
    return True

while True:
    tStart = time.time()
    frame = picam2.capture_array()
    roi = frame[11:720, 185:975]
    cv2.putText(roi, str(int(fps)) + ' FPS', pos, font, height, myColor, weight)
    
    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])
    frameHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)
    myMaskSmall = cv2.resize(myMask, (int(dispW / 2), int(dispH / 2)))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(myMaskSmall, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    myObject = cv2.bitwise_and(roi, roi, mask=myMask)
    myObjectSmall = cv2.resize(myObject, (int(dispW / 2), int(dispH / 2)))
    
    cnts = cv2.findContours(myMaskSmall.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    centerarray = []
    
    if len(cnts) > 0:
        for i in range(len(cnts)):
            moments = cv2.moments(cnts[i])
            x, y, w, h = cv2.boundingRect(cnts[i])
            center = (int(x + 0.5 * w), int(y + 0.5 * h))
            if 0.5 * w > 6:
                cv2.rectangle(myMaskSmall, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 255), 2)
                cv2.circle(myMaskSmall, center, 3, (0, 0, 0), -1)
                centerarray.append(center)

    cv2.imshow("Camera", roi)
    cv2.imshow('my Mask', myMaskSmall)
    cv2.imshow('My Object', myObjectSmall)
    
    if cv2.waitKey(1) == ord('q'):
        break
    
    if cv2.waitKey(1) == ord('w'):
        actioncent = centerarray
        if actioncent:
            for i in range(len(actioncent)):
                actualx = ((actioncent[i][0] / 790) * 600) - 300
                actualy = ((actioncent[i][1] / 720) * 540) - 270
                pose_rx = 0
                pose_ry = 0
                pose_rz = 0
                move_robot_safely(actualx, actualy, 200, -180, 0, 180)
                robot.MoveLin(actualx, actualy, 100, -180, 0, 180)
                robot.WaitIdle()
                robot.MovePose(150, 100, 300, 60, 90, 0)
    
    tEnd = time.time()
    loopTime = tEnd - tStart
    fps = 0.9 * fps + 0.1 * (1 / loopTime)

cv2.destroyAllWindows()
robot.DeactivateRobot()
robot.WaitDeactivated()
robot.Disconnect()
