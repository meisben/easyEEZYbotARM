# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2

# initial joint angles
jointAngle1 = 0
jointAngle2 = 70
jointAngle3 = -100

# Create an EEZYbotARM object
myRobotArm = EEZYbotARM_Mk2(jointAngle1, jointAngle2, jointAngle3)
# Plot it
myRobotArm.plot()
