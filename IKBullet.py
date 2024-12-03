import pybullet as p
import time
import math

# Connect to PyBullet and set up the simulation
p.connect(p.DIRECT)  # Use GUI for visualization
p.setGravity(0, 0, -9.8)

# Define the link lengths and other parameters
link_1_length = 1.0  # Length of the first link
link_2_length = 1.0  # Length of the second link
link_mass = 1.0  # Mass of each link
link_radius = 0.05  # Radius for the collision shape
link_half_height = 0.05  # Half height for collision shape

# Create the first link (base to first joint)
collision_shape_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[link_radius, link_1_length / 2, link_half_height])
visual_shape_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[link_radius, link_1_length / 2, link_half_height], rgbaColor=[1, 0, 0, 1])
link_1 = p.createMultiBody(baseMass=link_mass, baseCollisionShapeIndex=collision_shape_1, baseVisualShapeIndex=visual_shape_1, basePosition=[0, 0, link_1_length / 2])

# Create the second link (first joint to second joint)
collision_shape_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[link_radius, link_2_length / 2, link_half_height])
visual_shape_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[link_radius, link_2_length / 2, link_half_height], rgbaColor=[0, 1, 0, 1])
link_2 = p.createMultiBody(baseMass=link_mass, baseCollisionShapeIndex=collision_shape_2, baseVisualShapeIndex=visual_shape_2, basePosition=[link_1_length, 0, link_2_length / 2])

print(link_1)
print(link_2)
# Create revolute joints between links (for rotation)
#joint_1 = p.createConstraint(parentBodyUniqueId=link_1, parentLinkIndex=-1, childBodyUniqueId=link_2, childLinkIndex=-1, jointType=p.JOINT_REVOLUTE, jointAxis=[0, 0, 1], parentFramePosition=[link_1_length, 0, 0], childFramePosition=[0, 0, 0])
joint_1 = p.createConstraint(
    0, 
    -1, 
    1, 
    -1, 
    p.JOINT_REVOLUTE, 
    [0, 0, 1], 
    [link_1_length, 0, 0], 
    [0, 0, 0]
)
#p.createConstraint()
# Set joint limits (optional)
#p.changeConstraint(joint_1, lowerLimit=-math.pi, upperLimit=math.pi)

# Now the robot is ready. Let's move the joints using IK.

# Define the end-effector link index (last link in the chain, i.e., link_2)
#end_effector_link_index = 1  # The second link

# Set a target position for the end-effector
#target_position = [1.5, 0, 0]  # Target in 3D space (x, y, z)

# Calculate inverse kinematics
#joint_angles = p.calculateInverseKinematics(link_1, end_effector_link_index, target_position)

# Print the joint angles
#print("Calculated joint angles:", joint_angles)
