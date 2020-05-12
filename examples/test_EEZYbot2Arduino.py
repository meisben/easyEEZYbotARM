

from EEZYbotARM_controller.serial_communication import EEZYBotARM2Arduino

EEZY = EEZYBotARM2Arduino(port="COM3")  # Desktop

testData = []
testData.append("<BUZZ,30,110,50,90,1000,1000,1000,1000>")
testData.append("<BUZZ,150,110,50,90,1000,1000,1000,1000>")
testData.append("<BUZZ,150,70,110,90,1000,1000,1000,1000>")
testData.append("<BUZZ,90,70,110,90,2000,1000,1000,1000>")
testData.append("<BUZZ,90,90,90,90,3000,3000,3000,1000>")


EEZY.connectToSerialPort()

EEZY.waitForArduino()

EEZY.runTest(testData)

EEZY.closeSerialPort()
