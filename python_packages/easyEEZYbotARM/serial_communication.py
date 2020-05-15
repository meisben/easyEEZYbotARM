
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Program Title: serial_communication.py
# Program Purpose: Manages serial communication between a computer (running python) and Arduino for the purpose of controlling the EEXYbotARM

# **Version control**
# v3.5 -> moving to github

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# ------------------------------------------#
# Imports                                   #
# ------------------------------------------#

import serial
import time

# ------------------------------------------#
# Main class                              #
# ------------------------------------------#


class arduinoController:
    """
    --Description--
    This is a class for the EEZYBotARM2Arduino object

    This objects facilitates the creation of a serial connection to the Arduino (or another microcontroller) for the purpose of controling the
    three joint angles (and hence the position of the robot arm end effector)

    Credit: the methods in this function are primarily attributed to 'Robin2' in the Arduino forums

    --Methods--
    Description of available methods in this class:    
        - __init__ --> Initialise an object instance
        - connectToSerialPort--> Connects to Arduino serial port using pySerial.
        - closeSerialPort--> Close serial port using pySerial.
        - sendToArduino --> Sends data to Arduino
        - recvFromArduino --> Receive data from Arduino
        - waitForArduino --> Function waits until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
        - composeMessage --> Function composes a message (instruction) to send to the Arduino, this tells it where to position
        the connected servo motors
        - runTest --> Function runs a test to check communications with Arduino!
    """

    # Class attributes
    startMarker = 60  # unicode character '<'
    endMarker = 62  # unicode character '>'
    servoTime1 = 1000  # default times of servo movement
    servoTime2 = 1000
    servoTime3 = 1000
    servoTimeEE = 500
    msg = "<BUZZ,90,90,90,90,1000,1000,1000,1000>"  # default message

    # Initializer / Instance attributes

    def __init__(self, port="COM18"):
        self.port = port

    # Include function here connect and pass joint angle

    def __waitForArduino__(self):
        """
        --Description--
        Function waits until the Arduino sends 'Arduino Ready' - allows time for Arduino reset.
        It also ensures that any bytes left over from a previous message are discarded

        --Parameters--
        None

        --Returns--
        Function doesn't return a value

        """

        msg = ""
        while msg.find("Arduino is ready") == -1:

            # note changed to in_waiting, get the number of bytes in the input buffer
            while self.serialPort.inWaiting() == 0:
                pass

            msg = self.recvFromArduino()

            print(msg)
            print("")

    # Instance methods
    def openSerialPort(self, baudRate=9600, port=None):
        """
        --Description--
        Connects to a serial port using pySerial.
        This function exists to avoid confusion in naming of serial port!

        --Parameters--
        @myString -> the string to be sent

        --Returns--
        Function doesn't return a value

        """
        if port is None:
            port = self.port

        self.serialPort = serial.Serial(port=self.port, baudrate=baudRate)
        print("Serial port " + port + " opened  Baudrate " + str(baudRate))

        # wait for arduino to be ready
        self.__waitForArduino__()

    def closeSerialPort(self, port=None):
        """
        --Description--
        Close serial port using pySerial.
        This function exists to avoid confusion in naming of serial port!

        --Parameters--
        @myString -> the string to be sent

        --Returns--
        Function doesn't return a value

        """
        if port is None:
            port = self.port

        self.serialPort.close
        print("Serial port " + port + " closed")

    def sendToArduino(self, myString=None):
        """
        --Description--
        Sends a string to the Arduino. String must be in the format "<LED1,200,0.2>" [HOLD]

        --Parameters--
        @myString -> the string to be sent

        --Returns--
        Function doesn't return a value

        """
        if myString is None:
            myString = self.msg

        self.serialPort.write(myString.encode('utf-8'))  # encode as unicode

    def recvFromArduino(self):
        """
        --Description--
        Recieve a message from the Arduino. The message is interpreted as a string being
        between the start character '<' and end character '>'

        --Parameters--
        None

        --Returns--
        msg -> The received message as a string

        """
        msg = ""
        x = "z"  # any value that is not an end or startMarker
        byteCount = -1  # to allow for the fact that the last increment will be one too many

        # wait for the start character
        while ord(x) != self.startMarker:  # ord returns the unicode number for the character
            x = self.serialPort.read()

        # save data until the end marker is found
        while ord(x) != self.endMarker:
            if ord(x) != self.startMarker:
                msg = msg + x.decode("utf-8")  # decode from unicode
                byteCount += 1
            x = self.serialPort.read()

        return(msg)

    def composeMessage(self, servoAngle_q1, servoAngle_q2, servoAngle_q3, servoAngle_EE=90, instruction="BUZZ", **kwargs):
        """
        --Description--
        Function composes a message (instruction) to send to the Arduino, this tells it where to position
        the connected servo motors. As input we take the servo angles and optionally the movement durations
        for each angle

        This function can take direct output (for joint angles q1, q2, q3) from the function 'map_kinematicsToServoAngles()'

        --Parameters--
        @servoAngle_q1 -> the servo angle for servo #1, corresponding to q1
        @servoAngle_q2 -> the servo angle for servo #2, corresponding to q2
        @servoAngle_q3 -> the servo angle for servo #3, corresponding to q3
        @servoAngle_EE -> the servo angle for servo #0, corresponding to the end effector
        @instruction -> the text instruction to send to the Arduino (BUZZ-> sounds buzzer), (LED -> flashes LED)

        --Optional **kwargs parameters--
        @servoTime1 -> the time of the servo1 movement (if change in value is input for servo1Angle)
        @servoTime2 -> the time of the servo2 movement (if change in value is input for servo2Angle)
        @servoTime3 -> the time of the servo3 movement (if change in value is input for servo3Angle)
        @servoTimeEE -> the time of the servoEE movement (if change in value is input for servoEEAngle)

        --Returns--
        Function doesn't return a value

        """

        # Use **kwargs if provided, otherwise use current values
        servoTime1 = str(kwargs.get('servoTime1', self.servoTime1))
        servoTime2 = str(kwargs.get('servoTime2', self.servoTime2))
        servoTime3 = str(kwargs.get('servoTime3', self.servoTime3))
        servoTimeEE = str(kwargs.get('servoTimeEE', self.servoTimeEE))

        # Compose message
        message = "<" + ",".join([instruction, str(servoAngle_EE),
                                  str(servoAngle_q1), str(servoAngle_q2), str(servoAngle_q3)])
        message = message + "," + \
            ",".join([servoTimeEE, servoTime1, servoTime2, servoTime3]) + ">"

        self.msg = message

        return message

    def communicate(self, data, delay_between_commands=5):
        """
        --Description--
        Function runs a test to check communications with Arduino!

        --Parameters--
        data-> a command string (returned by the composeMessage method) or a list of strings, which will be communicated to the arduino
        delay_between_commands -> the time delay applied between a list of commands

        --Returns--
        Function doesn't return a value

        """
        # convert data to list if it isn't already a list
        if not isinstance(data, list):
            data = [data]

        # Declare variables
        numLoops = len(data)
        waitingForReply = False
        n = 0

        while n < numLoops:

            data_str = data[n]

            if waitingForReply == False:
                self.sendToArduino(data_str)
                print("Sent from PC -- LOOP NUM " +
                      str(n) + " TEST STR " + data_str)
                waitingForReply = True

            if waitingForReply == True:

                while self.serialPort.inWaiting() == 0:
                    pass

                dataRecvd = self.recvFromArduino()
                print("Reply Received  " + dataRecvd)
                n += 1
                waitingForReply = False

                print("===========")

            time.sleep(delay_between_commands)
