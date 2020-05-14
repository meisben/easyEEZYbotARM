# Please note for this example you must have an Arduino connected to the computer
# with a PCA9685 (or equivalent) servo driver connected and powered. The Arduino sketch from
# the 'arduino_sketches/easyEEZYbotARM_Arduino_Communication' folder must be uploaded
# to the arduino.

# This example shows the basis for moving the arduino servos. It does not link the 'kinematic model' together
# with the Arduino. For an example of this please see example_kinematics_2_Arduino.py

# The servos are plugged into the following pin positions on the PCA9685 board:
# Servo_q1 -> PCA9685 pin 1
# Servo_q2 -> PCA9685 pin 2
# Servo_q3 -> PCA9685 pin 3
# Servo_EE -> PCA9685 pin 0

from easyEEZYbotARM.serial_communication import EEZYBotARM2Arduino

EEZY = EEZYBotARM2Arduino(port="COM19")  # Insert your Arduino serial port here

# Create some test data.
# The angles used for this test data are 'raw' servo angles (i.e. not calibrated)
# against the kinematic model for the EEZYbotARM
testData = []
testData.append(EEZY.composeMessage(servoAngle_q1=90,
                                    servoAngle_q2=90,
                                    servoAngle_q3=90,
                                    servoAngle_EE=10))

testData.append(EEZY.composeMessage(servoAngle_q1=90,
                                    servoAngle_q2=90,
                                    servoAngle_q3=90,
                                    servoAngle_EE=90))


# The connection should be managed in this sequence (will be simplified in future)
EEZY.openSerialPort()  # Open a serial port and connect to the Arduino
# Send the test data which is managed by the 'run test' function'
EEZY.communicate(data=testData, delay_between_commands=5)
EEZY.closeSerialPort()  # Close the serial port
