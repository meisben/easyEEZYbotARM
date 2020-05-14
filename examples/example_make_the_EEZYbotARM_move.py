# Please note for this example you must have an Arduino connected to the computer
# with a PCA9685 (or equivalent) servo driver connected and powered. The Arduino sketch from
# the 'arduino_sketches/easyEEZYbotARM_Arduino_Communication' folder must be uploaded
# to the arduino.

from easyEEZYbotARM.serial_communication import EEZYBotARM2Arduino

EEZY = EEZYBotARM2Arduino(port="COM18")  # Insert your Arduino serial port here

testData = []
testData.append("<BUZZ,30,110,50,90,1000,1000,1000,1000>")
testData.append("<BUZZ,150,110,50,90,1000,1000,1000,1000>")
testData.append("<BUZZ,150,70,110,90,1000,1000,1000,1000>")
testData.append("<BUZZ,90,70,110,90,2000,1000,1000,1000>")
testData.append("<BUZZ,90,90,90,90,3000,3000,3000,1000>")

# The connection should be managed in this sequence (will be simplified in future)

EEZY.connectToSerialPort()  # Open a serial port and connect to the Arduino

EEZY.waitForArduino()  # Wait for the Arduino to respond

# Send the test data which is managed by the 'run test' function'
EEZY.runTest(testData)

EEZY.closeSerialPort()  # Close the serial port
