import mecademicpy.robot as mdr
import mecademicpy.robot_initializer as initializer
import mecademicpy.tools as tools
import time
robot = mdr.Robot()
robot.Connect(address='192.168.0.100', enable_synchronous_mode=False)
robot.ActivateAndHome()
robot.WaitHomed()
robot.SetJointVelLimit(25)
robot.MoveJoints(0, 0, 0, 0, 0, 0)
robot.WaitIdle()
#robot.MoveJoints(0, -60, 60, 0, 0, 0)

#for i in range(100):
#    robot.MoveJoints(-50+i, -60, 60, 0, 0, 0)
#    print(robot.GetJoints())
#    time.sleep(0.05)

#robot.MoveLin(100, -60, 90, 0, 0, 0)
#time.sleep(5)
robot.MovePose(200, 100, 60, 0, 90, 0)
robot.WaitIdle()
robot.DeactivateRobot()
robot.WaitDeactivated()
robot.Disconnect()