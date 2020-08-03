# Please note for this example you must have an Arduino connected to the computer
# with a PCA9685 (or equivalent) servo driver connected and powered. The Arduino sketch from
# the 'arduino_sketches/easyEEZYbotARM_Arduino_Communication' folder must be uploaded
# to the arduino.

# This example links the 'kinematic model' together with the Arduino controlling the
# EEZYbotARM

# You must have installed the servo's in their reference positions. If you are reading this and
# you're not sure what this means, please email ben.money@gmail.com. The procedure for this step
# will be published online at a later date.

# The reference positions for the three servos are as follows:
# - EzzyBot base (q1 -> 0 degrees) : 90 degree servo position is facing directly forwards
# - Main arm (q2 -> 90 degrees): 90 degree servo position is with main arm perpendicular (at 90 degrees to) base
# - Horarm (q3 -> -135): 90 degree servo poisition is with horarm servo link at 45 degrees to base

# The servos are plugged into the following pin positions on the PCA9685 board:
# Servo_q1 -> EzzyBot base (q1) -> PCA9685 pin 1
# Servo_q2 -> Main arm (q2) -> PCA9685 pin 2
# Servo_q3 -> Horarm (q3) -> PCA9685 pin 3
# Servo_EE -> End Effector (EE) -> PCA9685 pin 0

# Import EEZYbotARM library
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk2
from easyEEZYbotARM.serial_communication import arduinoController

# Insert your Arduino serial port here to initialise the arduino controller
myArduino = arduinoController(port="COM3")
myArduino.openSerialPort()

# Initialise kinematic model with initial joint angles (home position)
myVirtualRobotArm = EEZYbotARM_Mk2(
    initial_q1=0, initial_q2=90, initial_q3=-130)
# Plot it
myVirtualRobotArm.plot()

# Define end effector open and closed angle
servoAngle_EE_closed = 10
servoAngle_EE_open = 90

# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

# Send the movement command to the arduino. The physical EEZYbotARM will move to this position
myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_open))


# Assign new cartesian position where we want the robot arm end effector to move to
# (x,y,z in mm from centre of robot base)
x = 240  # mm
y = 85  # mm
z = 200  # mm

# Compute inverse kinematics
a1, a2, a3 = myVirtualRobotArm.inverseKinematics(x, y, z)

# Print the result
print('To move the end effector to the cartesian position (mm) x={}, y={}, z={}, the robot arm joint angles (degrees)  are q1 = {}, q2= {}, q3 = {}'.format(x, y, z, a1, a2, a3))

# Visualise the new joint angles
myVirtualRobotArm.updateJointAngles(q1=a1, q2=a2, q3=a3)
myVirtualRobotArm.plot()

# Calculate the current servo angles
servoAngle_q1, servoAngle_q2, servoAngle_q3 = myVirtualRobotArm.map_kinematicsToServoAngles()

# Send the movement command to the arduino. The physical EEZYbotARM will move to this position
myArduino.communicate(data=myArduino.composeMessage(servoAngle_q1=servoAngle_q1,
                                                    servoAngle_q2=servoAngle_q2,
                                                    servoAngle_q3=servoAngle_q3,
                                                    servoAngle_EE=servoAngle_EE_open))

# Close the serial port
myArduino.closeSerialPort()
