import pybullet as p
import time
import math

# Connect to the physics server
p.connect(p.DIRECT)  # Use GUI for visualization
p.setGravity(0, 0, -9.8)

# Create a simple body (a box) for the parent body
collision_shape_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.5, 0.1])
visual_shape_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.5, 0.1], rgbaColor=[1, 0, 0, 1])
link_1 = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collision_shape_1,
    baseVisualShapeIndex=visual_shape_1,
    basePosition=[0, 0, 0.5]
)
# Create a second simple body (a box) for the child body
collision_shape_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.5, 0.1])
visual_shape_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.5, 0.1], rgbaColor=[0, 1, 0, 1])
link_2 = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collision_shape_2,
    baseVisualShapeIndex=visual_shape_2,
    basePosition=[1.0, 0, 0.5]
)

# Now, let's create a revolute joint (a rotating joint between link_1 and link_2)
joint_1 = p.createConstraint(
    parentBodyUniqueId=link_1, parentLinkIndex=-1,
    childBodyUniqueId=link_2, childLinkIndex=-1,
    jointType=p.JOINT_REVOLUTE,
    jointAxis=[0, 0, 1],  # Revolute joint around the Z-axis
    parentFramePosition=[0.5, 0, 0],  # Position of the joint on the parent body
    childFramePosition=[0, 0, 0]  # Position of the joint on the child body
)

# Optional: Set joint limits for the revolute joint
p.changeConstraint(joint_1, lowerLimit=-math.pi, upperLimit=math.pi)

# Run a basic simulation for 240 steps to observe the behavior
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from the physics server
p.disconnect()
