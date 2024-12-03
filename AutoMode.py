# import mecademicpy.robot as mdr
# import cv2
# import numpy as np

# # Define the pixel width of 24 inches in your ROI (you can get this from the ROI width)
# roi_width_inches = 24.0  # The known width in inches
# roi_width_pixels = 960    # The width of your ROI in pixels (as per your example, 1185 - 225)

# # Calculate pixel-to-inch ratio
# pixel_to_inch_ratio = roi_width_inches / roi_width_pixels

# # Define the origin point in pixels (where x = 131.2 and y = 12 starts)
# origin_x_pixel = 131.2
# origin_y_pixel = 12

# def detect_objects(frame, roi_coords):
#     global pixel_to_inch_ratio
    
#     # Extract the Region of Interest (ROI) from the frame
#     x1, y1, x2, y2 = roi_coords
#     roi = frame[y1:y2, x1:x2]

#     # Convert ROI from BGR to HSV color space
#     hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
#     # Define lower and upper bounds for black and dark gray in HSV
#     lower_black = np.array([0, 0, 0])       # Lower bound for black
#     upper_black = np.array([180, 255, 50])   # Upper bound for black (dark gray)

#     # Create a mask to ignore black and gray areas
#     mask = cv2.inRange(hsv, lower_black, upper_black)
    
#     # Invert the mask to keep non-black/gray areas
#     mask_inv = cv2.bitwise_not(mask)

#     # Apply the mask to the ROI to keep only non-black/gray areas
#     masked_roi = cv2.bitwise_and(roi, roi, mask=mask_inv)
    
#     # Convert the masked ROI from BGR to grayscale
#     gray = cv2.cvtColor(masked_roi, cv2.COLOR_BGR2GRAY)
    
#     # Apply Gaussian blur to reduce noise
#     blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    
#     # Apply Canny edge detection to find edges
#     edges = cv2.Canny(blurred, 30, 150)
    
#     # Perform morphological operations to close gaps in between object edges
#     kernel = np.ones((5, 5), np.uint8)
#     closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
#     # Find contours of detected objects
#     contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     # Analyze contours to determine positions, shapes, sizes, etc.
#     for contour in contours:
#         # Calculate contour area
#         area = cv2.contourArea(contour)
        
#         # Filter out small contours (noise)
#         if area > 250:
#             # Get bounding box coordinates and draw it on the frame
#             x, y, w, h = cv2.boundingRect(contour)
#             cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
#             # Calculate centroid of the contour
#             M = cv2.moments(contour)
#             if M["m00"] != 0:  # Avoid division by zero
#                 centroid_x = int(M["m10"] / M["m00"])
#                 centroid_y = int(M["m01"] / M["m00"])
                
#                 # Draw centroid on the ROI
#                 cv2.circle(roi, (centroid_x, centroid_y), 5, (255, 0, 0), -1)
                
#                 # Adjust coordinates based on your coordinate system
#                 # Positive x is now down, negative x is up
#                 # Positive y is right, negative y is left
                
#                 # Calculate centroid coordinates in inches (relative to the origin)
#                 x_inch = ((centroid_y) * pixel_to_inch_ratio * 25.4)+(131.2-26.2) # Positive for down, negative for up
#                 y_inch = (centroid_x) * pixel_to_inch_ratio * 25.4 -(294)
                
#                 # Print centroid coordinates in inches according to the specified axis
#                 print(f"Centroid Coordinates in Inches: X (vertical)={x_inch:.2f}, Y (horizontal)={y_inch:.2f}")

#     # Return the updated frame with the ROI drawn
#     frame[y1:y2, x1:x2] = roi


# # Capture video from the camera
# cap = cv2.VideoCapture(0)

# # Define the ROI coordinates (x1, y1, x2, y2)
# roi_coords = (150, 280, 1075, 470)  # Example ROI coordinates, adjust as needed

# # Capture a single frame
# ret, frame = cap.read()

# if ret:
#     # Detect objects within the ROI
#     detect_objects(frame, roi_coords)
    
#     # Draw a rectangle to indicate the ROI on the full frame (optional)
#     cv2.rectangle(frame, (roi_coords[0], roi_coords[1]), (roi_coords[2], roi_coords[3]), (255, 0, 0), 2)
    
#     # Display the frame with the ROI
#     cv2.imshow('Object Detection with ROI', frame)
    
#     # Save the screenshot
#     cv2.imwrite('screenshot.png', frame)  # Save the image as 'screenshot.png'
    
#     # Wait for a brief moment before closing the window
#     cv2.waitKey(2000)  # Wait for 2 seconds to view the image before closing

# # Release video capture and close windows
# cap.release()
# cv2.destroyAllWindows()


# import mecademicpy.robot as mdr
# import cv2
# import numpy as np
# import time

# # Define the pixel width of 24 inches in your ROI (you can get this from the ROI width)
# roi_width_inches = 24.0  # The known width in inches
# roi_width_pixels = 960    # The width of your ROI in pixels (as per your example, 1185 - 225)

# # Calculate pixel-to-inch ratio
# pixel_to_inch_ratio = roi_width_inches / roi_width_pixels

# # Define the origin point in pixels (where x = 131.2 and y = 12 starts)
# origin_x_pixel = 131.2
# origin_y_pixel = 12

# def detect_objects(frame, roi_coords):
#     global pixel_to_inch_ratio
    
#     # Extract the Region of Interest (ROI) from the frame
#     x1, y1, x2, y2 = roi_coords
#     roi = frame[y1:y2, x1:x2]

#     # Convert ROI from BGR to HSV color space
#     hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
#     # Define lower and upper bounds for black and dark gray in HSV
#     lower_black = np.array([0, 0, 0])       # Lower bound for black
#     upper_black = np.array([180, 255, 50])   # Upper bound for black (dark gray)

#     # Create a mask to ignore black and gray areas
#     mask = cv2.inRange(hsv, lower_black, upper_black)
    
#     # Invert the mask to keep non-black/gray areas
#     mask_inv = cv2.bitwise_not(mask)

#     # Apply the mask to the ROI to keep only non-black/gray areas
#     masked_roi = cv2.bitwise_and(roi, roi, mask=mask_inv)
    
#     # Convert the masked ROI from BGR to grayscale
#     gray = cv2.cvtColor(masked_roi, cv2.COLOR_BGR2GRAY)
    
#     # Apply Gaussian blur to reduce noise
#     blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    
#     # Apply Canny edge detection to find edges
#     edges = cv2.Canny(blurred, 30, 150)
    
#     # Perform morphological operations to close gaps in between object edges
#     kernel = np.ones((5, 5), np.uint8)
#     closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
#     # Find contours of detected objects
#     contours, _ = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     # Analyze contours to determine positions, shapes, sizes, etc.
#     for contour in contours:
#         # Calculate contour area
#         area = cv2.contourArea(contour)
        
#         # Filter out small contours (noise)
#         if area > 250:
#             # Get bounding box coordinates and draw it on the frame
#             x, y, w, h = cv2.boundingRect(contour)
#             cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
#             # Calculate centroid of the contour
#             M = cv2.moments(contour)
#             if M["m00"] != 0:  # Avoid division by zero
#                 centroid_x = int(M["m10"] / M["m00"])
#                 centroid_y = int(M["m01"] / M["m00"])
                
#                 # Draw centroid on the ROI
#                 cv2.circle(roi, (centroid_x, centroid_y), 5, (255, 0, 0), -1)
                
#                 # Adjust coordinates based on your coordinate system
#                 # Positive x is now down, negative x is up
#                 # Positive y is right, negative y is left
                
#                 # Calculate centroid coordinates in inches (relative to the origin)
#                 # Vertical (x-axis): positive is down, negative is up
#                 # Horizontal (y-axis): positive is right, negative is left
#                 x_inch = ((centroid_y) * pixel_to_inch_ratio * 25.4)+(131.2-26.2) # Positive for down, negative for up
#                 y_inch = (centroid_x) * pixel_to_inch_ratio * 25.4 -(294)
                
#                 # Print centroid coordinates in inches according to the specified axis
#                 print(f"Centroid Coordinates in Inches: X (vertical)={x_inch:.2f}, Y (horizontal)={y_inch:.2f}")

#     # Return the updated frame with the ROI drawn
#     frame[y1:y2, x1:x2] = roi


# # Capture video from the camera
# cap = cv2.VideoCapture(0)

# # Define the ROI coordinates (x1, y1, x2, y2)
# # Adjust these values to fit the region where the baseboard is located
# roi_coords = (150, 280, 1075, 470)  # Example ROI coordinates, adjust as needed

# while True:
#     ret, frame = cap.read()
    
#     if ret:
#         # Detect objects within the ROI
#         detect_objects(frame, roi_coords)
        
#         # Draw a rectangle to indicate the ROI on the full frame (optional)
#         cv2.rectangle(frame, (roi_coords[0], roi_coords[1]), (roi_coords[2], roi_coords[3]), (255, 0, 0), 2)
        
#         # Display the frame with the ROI
#         cv2.imshow('Object Detection with ROI', frame)
        
#         # Press 'q' to exit
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     else:
#         break

# # Release video capture and close windows
# cap.release()
# cv2.destroyAllWindows()






import mecademicpy.robot as mdr
import cv2
import numpy as np

# Initialize the robot
robot = mdr.Robot()
robot.Connect(address='192.168.0.100', enable_synchronous_mode=False)
robot.ActivateAndHome()
robot.WaitHomed()
robot.MoveJoints(0,-60,60,0,0,0)

# Define constant angles
# alpha = -180
# beta = 0
# gamma = 180
z=60
# Continuous input loop
try:
    while True:
        # Get user input for x and y coordinates
        x = float(input("Enter x coordinate: "))
        y = float(input("Enter y coordinate: "))


        # Move to the new position
        robot.MoveLin(100, 0, 90)
        robot.WaitIdle()  # Wait for the robot to complete the movement
        
        # Move z back down to the original position (if needed, to prevent accumulation)
        robot.MoveLin(200, 0, 90)
        robot.WaitIdle()  # Wait for the robot to complete the movement

except KeyboardInterrupt:
    # Clean up on exit
    robot.DeactivateRobot()
    robot.WaitDeactivated()
    robot.Disconnect()


























