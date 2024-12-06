import pygame
import mecademicpy.robot as mdr
import time     

# # Keymapping
# # Button 0:Trigger
# # Button 1: Thumb Button
# # Button 2: Top Bottom Left Button
# # Button 3: Top Bottom Right Button
# # Button 4: Top Up Left Button
# # Button 5: Top Up Right Button
# # Button 6: Bottom First Left
# # Button 7: Bottom First Right
# # Button 8: Bottom Second Left
# # Button 9: Bottom Second Right
# # Button 10: Bottom Third Left
# # Button 11: Bottom Third Right

# # D Pad Up (0,1)
# # D Pad Left (-1,0)
# # D Pad Right (1,0)
# # D Pad Down (0,-1)

# # Axis 0: Left (-1) and Right (1) 
# # Axis 1: Foward (-1) and Backward (1)
# # Axis 2: Twist Left (-1) and Twist Right (1)
# # Axis 3: Throttle


# def ManualControl():
#     # Initialize the robot
#     robot = mdr.Robot()
#     robot.Connect(address='192.168.0.100')
    
#     try:
#         robot.ActivateAndHome()  # Activate and home the robot
#         robot.MoveJoints(0, 0, 0, 0, 0, 0)  # Move all joints to zero position

#         # Initialize Pygame and the controller
#         pygame.init()

#         if pygame.joystick.get_count() == 0:
#             print("No controller connected!") #Detects if controller is connected
#             return  # Exit the function

#         joystick = pygame.joystick.Joystick(0)
#         joystick.init()

#         # Joint angles initialized and List of speeds
#         joint_angles = [0, 0, 0, 0, 0, 0]  # Start with all joints at 0 degrees
#         speeds = [0.1, 1, 10, 20, 30, 40, 100]  # List of speeds - More can be addes up to max (100)
#         speed_index = 1  # Start with the default speed (1)
#         speed = speeds[speed_index]  # Current speed based on the speed index

#         # Set default joint velocity
#         robot.SetJointVel(speed)  # Default velocity

#         # Define angle limits for each joint
#         angle_limits = [
#             (-175, 175),      # Joint 1 limits
#             (-70, 90),        # Joint 2 limits
#             (-135, 70),       # Joint 3 limits
#             (-170, 170),      # Joint 4 limits
#             (-115, 115),      # Joint 5 limits
#             (-90, 90)         # Joint 6 limits
#         ]

#         # Movement threshold for joystick axis (Makes it more accurate in joystick;doesnt move if joystick is let go)
#         movement_threshold = 0.1  # Only move if joystick value exceeds this threshold

#         # Set an update frequency for the control loop
#         update_frequency = 0.01  # 100 Hz (10ms delay) for smoother control

#         # Flags for speed button press
#         increase_speed_pressed = False #Depress and Press (Not Hold Down)
#         decrease_speed_pressed = False

#         try:
#             while True:
#                 pygame.event.pump()  # Update Pygame events

#                 # Get joystick values
#                 left_stick_x = joystick.get_axis(0)  # Left stick X-axis
#                 left_stick_y = joystick.get_axis(1)  # Left stick Y-axis
#                 right_stick_x = joystick.get_axis(2)  # Right stick X-axis
#                 right_stick_y = joystick.get_axis(3)  # Right stick Y-axis
#                 left_bumper = joystick.get_axis(4)   # Left bumper
#                 right_bumper = joystick.get_axis(5)  # Right bumper

#                 # Speed increase button press (button 10)
#                 if joystick.get_button(10) and not increase_speed_pressed:
#                     increase_speed_pressed = True
#                     speed_index = min(speed_index + 1, len(speeds) - 1)  # Increase speed index by 1
#                     speed = speeds[speed_index]
#                     robot.SetJointVel(speed)  # Adjust joint velocity based on speed
#                     print(f"Speed increased to: {speed}")
#                 elif not joystick.get_button(10):
#                     increase_speed_pressed = False  # Reset the flag when button is released

#                 # Speed decrease button press (button 9)
#                 if joystick.get_button(9) and not decrease_speed_pressed:
#                     decrease_speed_pressed = True
#                     speed_index = max(speed_index - 1, 0)  # Decrease speed index by 1
#                     speed = speeds[speed_index]
#                     robot.SetJointVel(speed)  # Adjust joint velocity based on speed
#                     print(f"Speed decreased to: {speed}")
#                 elif not joystick.get_button(9):
#                     decrease_speed_pressed = False  # Reset the flag when button is released

#                 # Adjust joint angles based on joystick input only if above threshold
#                 if abs(left_stick_x) > movement_threshold:
#                     joint_angles[0] += left_stick_x * speed * update_frequency   # Joint 1 controlled by left stick X
                
#                 if abs(left_stick_y) > movement_threshold:
#                     joint_angles[1] += left_stick_y * speed * update_frequency  # Joint 2 controlled by left stick Y
                
#                 if abs(right_stick_x) > movement_threshold:
#                     joint_angles[5] -= right_stick_x * speed * update_frequency  # Joint 6 controlled by right stick X
                
#                 if abs(right_stick_y) > movement_threshold:
#                     joint_angles[2] += right_stick_y * speed * update_frequency  # Joint 3 controlled by right stick Y

#                 # Adjust joint 5 based on bumper input
#                 joint_angles[4] += (left_bumper - right_bumper) * speed * update_frequency  # Joint 5 controlled by bumpers

#                 # Ensure the joint angles are within valid limits (Keeps joints within its limits)
#                 for i in range(len(joint_angles)):
#                     joint_angles[i] = max(min(joint_angles[i], angle_limits[i][1]), angle_limits[i][0])

#                 # Move joints smoothly by sending updated joint positions
#                 robot.MoveJoints(*joint_angles)  # Update joint positions
                
                
#                 # Print current joint angles  and current speed                        
#                 print(f"Joint Angles: {joint_angles}, Speed: {speed}")    

#                 # Check for gripper controls
#                 if joystick.get_button(0):  # X button
#                     robot.GripperClose()
                    
#                 if joystick.get_button(2):  # Square button
#                     robot.GripperOpen()

#                 # Shipping Position and Deactivate Robot
#                 if joystick.get_button(3):  # Triangle button
#                     print("Resetting to home position...")
#                     robot.MoveJoints(0, -60, 60, 0, 0, 0)  # Move to home position
#                     time.sleep(2)
#                     robot.DeactivateRobot()  # Deactivate robot
#                     break  # Exit the control loop

#                 # Sleep for a short duration to smooth out control
#                 time.sleep(update_frequency)  # Control loop updated according to update_frequency

#         except KeyboardInterrupt:
#             print("Exiting manual control...")

#     finally:
#         # Ensure the robot is deactivated and disconnected safely
#         try:
#             robot.DeactivateRobot()
#         except Exception as e:
#             print(f"Error deactivating robot: {e}")

#         robot.Disconnect()  # Always attempt to disconnect the robot
#         pygame.joystick.quit()
#         pygame.quit()

# # Run the manual control function
# ManualControl()

def ManualControl():
    # Initialize the robot
    robot = mdr.Robot()
    robot.Connect(address='192.168.0.100')
    
    try:
        robot.ActivateAndHome()  # Activate and home the robot
        robot.MoveJoints(0, 0, 0, 0, 0, 0)  # Move all joints to zero position

        # Initialize Pygame and the controller
        pygame.init()

        if pygame.joystick.get_count() == 0:
            print("No controller connected!")  # Detects if controller is connected
            return  # Exit the function

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        # Joint angles initialized
        joint_angles = [0, 0, 0, 0, 0, 0]  # Start with all joints at 0 degrees

        # Define angle limits for each joint
        angle_limits = [
            (-175, 175),      # Joint 1 limits
            (-70, 90),        # Joint 2 limits
            (-135, 70),       # Joint 3 limits
            (-170, 170),      # Joint 4 limits
            (-115, 115),      # Joint 5 limits
            (-90, 90)         # Joint 6 limits
        ]

        # Movement threshold for joystick axis
        movement_threshold = 0.1  # Only move if joystick value exceeds this threshold

        # Set an update frequency for the control loop
        update_frequency = 0.01  # 100 Hz (10ms delay) for smoother control

        # Variable to keep track of the selected joint pair
        selected_joint_pair = None  # None indicates no button pressed yet

        # Store the previous throttle value to detect changes
        previous_throttle = 0

        try:
            while True:
                pygame.event.pump()  # Update Pygame events

                # Throttle speed control using Axis 3
                throttle = 100-((joystick.get_axis(3) + 1) / 2 * 100)  # Converts range from 0 to 100

                # Only update the robot speed if there's a significant change in throttle
                if abs(throttle - previous_throttle) > 1:  # Check if throttle has changed by more than 1
                    robot.SetJointVel(throttle)  # Set dynamic speed based on throttle value
                    previous_throttle = throttle
                    print(f"Current speed set by throttle: {throttle}")

                # Get joystick values for Axis 0 and Axis 1
                axis_0 = joystick.get_axis(0)  # Left/right on Axis 0
                axis_1 = joystick.get_axis(1)  # Forward/backward on Axis 1
                axis_2 = joystick.get_axis(2)

                # Check if Button 7 or 8 is pressed to select a joint set
                if joystick.get_button(6):  # Button 7 pressed
                    selected_joint_pair = (0, 1, 2)  # Control Joint 1, Joint 2, and Joint 3
                    print("Selected Joints: Joint 1, Joint 2, Joint 3")
                elif joystick.get_button(7):  # Button 8 pressed
                    selected_joint_pair = (3, 4,5)  # Control Joint 4, Joint 5, and Joint 6
                    print("Selected Joints: Joint 4, Joint 5, Joint 6")

                # DPAD Controls for Joint 6
                dpad = joystick.get_hat(0)  # Get DPAD (hat) input
                if dpad[1] == -1:  # DPAD right
                    joint_angles[4] += throttle * update_frequency  # Increase Joint 6 angle
                elif dpad[1] == 1:  # DPAD left
                    joint_angles[4] -= throttle * update_frequency  # Decrease Joint 6 angle


                    
                elif joystick.get_button(5):  # Top Bottom Right button
                    print("Resetting to home position...")
                    joint_angles = [0,0,0,0,0,0]
                
                
                               

                # Adjust angles for the selected joints based on joystick axis movement
                if selected_joint_pair is not None:
                    joint_a, joint_b, joint_c = selected_joint_pair

                    # Adjust the angles for the selected joints
                    if abs(axis_0) > movement_threshold:
                        joint_angles[joint_a] -= axis_0 * throttle * update_frequency
                    if abs(axis_1) > movement_threshold:
                        joint_angles[joint_b] -= axis_1 * throttle * update_frequency
                    if abs(axis_2) > movement_threshold:
                        joint_angles[joint_c] += axis_2 * throttle * update_frequency

                # Ensure the joint angles are within valid limits
                for i in range(len(joint_angles)):
                    joint_angles[i] = max(min(joint_angles[i], angle_limits[i][1]), angle_limits[i][0])

                # Move joints smoothly by sending updated joint positions
                robot.MoveJoints(*joint_angles)  # Update joint positions
                
                # Print current joint angles and throttle speed
                print(f"Joint Angles: {joint_angles}, Throttle Speed: {throttle}")

                # Check for gripper controls
                if joystick.get_button(0):  # Trigger button
                    robot.GripperClose()
                    
                if joystick.get_button(2):  # Top Bottom Left button
                    robot.GripperOpen()





                # Shipping Position and Deactivate Robot
                if joystick.get_button(3):  # Top Bottom Right button
                    print("Resetting to home position...")
                    robot.MoveJoints(0, -60, 60, 0, 0, 0)  # Move to home position
                    time.sleep(2)
                    robot.DeactivateRobot()  # Deactivate robot
                    break  # Exit the control loop

                # Sleep for a short duration to smooth out control
                time.sleep(update_frequency)  # Control loop updated according to update_frequency

        except KeyboardInterrupt:
            print("Exiting manual control...")

    finally:
        # Ensure the robot is deactivated and disconnected safely
        try:
            robot.DeactivateRobot()
        except Exception as e:
            print(f"Error deactivating robot: {e}")

        robot.Disconnect()  # Always attempt to disconnect the robot
        pygame.joystick.quit()
        pygame.quit()

# Run the manual control function
ManualControl()
