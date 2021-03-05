# Example of forward and inverse kinematics with the EEZYbotARM_Mk1

# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1

# Initialise robot arm with initial joint angles
# Note it doesn't really matter too much what the initial joint angles are - they're only stored internally in the class. You don't have to use the intial values for anything if you don't want to!
myRobotArm = EEZYbotARM_Mk1(initial_q1=0, initial_q2=70, initial_q3=-100)


# --- FORWARD KINEMATICS EXAMPLE ----

# Assign new joint angles
a1 = 0  # joint angle 1
a2 = 90  # joint angle 2
a3 = -130  # joint angle 3

# Compute forward kinematics
x, y, z = myRobotArm.forwardKinematics(q1=a1, q2=a2, q3=a3)

# Print the result
print('With joint angles(degrees) q1={}, q2={}, q3={}, the cartesian position of the end effector(mm) is x={}, y={}, z={}'.format(a1, a2, a3, x, y, z))

# Visualise the new joint angles
myRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
print("Here is the plot after competing forward kinematics with these joint angles, and this cartesian position")
myRobotArm.plot()


# --- INVERSE KINEMATICS EXAMPLE ----

# Assign cartesian position where we want the robot arm end effector to move to
# (x,y,z in mm from centre of robot base)
x = 120  # mm
y = 0  # mm
z = 90  # mm

# Compute inverse kinematics
a1, a2, a3 = myRobotArm.inverseKinematics(x, y, z)

# Print the result
print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))

# Visualise the new joint angles
myRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
print("Here is the plot after competing inverse kinematics with these joint angles, and this cartesian position")
myRobotArm.plot()
