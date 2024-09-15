## Control Interface for MOTIMOVE 8
## (c) Dipl.-Ing. Dr. Martin Schmoll, BSc


# !/usr/bin/python3
import serial  # import the module
import time
import threading
from multiprocessing import Process, Value
from Stimulators.Motimove.MM_Message_Builder import MM_Message_Builder


# Debugflag
PYCHARM_DEBUG = True


# Motimove Manager
class MM_Manager(object):

    # Constructor
    def __init__(self):

        # State
        # -2 ... Error
        # -1 ... Module not active
        #  0 ... Connecting
        #  1 ... Connected
        self.State = Value('i', -1)

        self.__EmergencyStop = Value('i', 0)    # Initiates the Emergency Stop -> Turns off stimulation
        self.__TimeOut = Value('d', 0.0)        # TimeOutVariable - requires the main application to communicate at least every 50ms

        self.__KILL_PROCESS = Value('i', 0)  # Kills the Process

        self.message_builder = MM_Message_Builder()


    # Configures the MM_Manager and the MM_Messagebuilder
    def setConfiguration(self, config_JSON):

        # Read from JSON
        PhW = config_JSON["PhW"]
        self.message_builder.setPhasewidths(PhW)
        PhW_BOOST = config_JSON["PhW_Boost"]
        self.message_builder.setPhasewidths_BOOST(PhW_BOOST)

        Amlitudes_max = config_JSON["I_Max"]
        self.message_builder.setMaxAmplitudes(Amlitudes_max)

        F = config_JSON["F"]
        self.message_builder.setStimFrequency(F)

        F_BOOST = config_JSON["F_Boost"]
        self.message_builder.setStimFrequency_BOOST(F_BOOST)

        return True


    # Updates the stimulation amplitude in [%]
    def setIntensity(self, I):

        self.message_builder.setIntensity(I)
        return True


    # Sets the active Channels
    def setActiveChannels(self, activeChannels):
        self.message_builder.setActiveChannels(activeChannels)


    # Sets the Stimulation Frequency in [Hz]
    def setStimFrequency(self, F):
        self.message_builder.setStimFrequency(F)


    # Sets the stimulation phasewidth in [µs] for each channel
    def setStimPhasewidth(self,PhW):
        self.message_builder.setPhasewidths(PhW)

    # Sets the maximal stimulation amplitude in [mA] for each channel
    def setMaxAmplitude(self, A_max):
        self.message_builder.setMaxAmplitudes(A_max)

    # Updates the state of the BOOST Button
    # 0.. BOOST deactivated, 1 .. BOOST active
    def setBOOST(self, BOOST):

        self.message_builder.setBOOST_Mode(BOOST)
        return True

    # Initiates an emergency stop. The stimulator will immediatelly stopped.
    # Further the function resets the TimeOut value
    # 0... No Emergency Stop, 1 .. Emergency Stop
    def setEmergencyStop(self, state):
        self.__EmergencyStop.value = state
        self.__TimeOut.value = 0.0


    # Returns the current stimulation intensity in [%]
    def getStimulationIntensity(self):
        return self.message_builder.getIntensity()


    # Returns the current configuration
    def getStimConfig(self):
        data = {
            "F": self.message_builder.getFrequency(),
            "F_Boost": self.message_builder.getFrequency_BOOST(),
            "Monophasic": [True, True, True, True, True, True, True, True],
            "PhW": self.message_builder.getPhasewidths(),
            "PhW_Boost": self.message_builder.getPhasewidths_BOOST(),
            "IPG": [0, 0, 0, 0, 0, 0, 0, 0],
            "I_Max": self.message_builder.getAmplitudesMax(),
            "RampUP": [0, 0, 0, 0, 0, 0, 0, 0],
            "RampDOWN": [0, 0, 0, 0, 0, 0, 0, 0]
        }

        return data

        # Returns the current state of the stimulator

    def getState(self):
        return self.State.value

    # Function to start independent Thread generating pulses

    def start(self):

        # Start Thread
        self.__process_Worker = Process(name='Stimulation_Process', target=self.__run, args=())
        self.__process_Worker.daemon = True
        self.__process_Worker.start()

    # Worker method for Pulsegeneration
    def __run(self):

        # Connect stimulator
        self.connect()

        sendOK = True

        ## Main Loop
        while self.__KILL_PROCESS.value == 0:

            # Emergency Stop or communication time out > 50 ms
            if (self.__EmergencyStop.value == 1) :#| (self.__TimeOut.value > 0.05):
                if sendOK:
                    self.message_builder.setHighVoltage(0)
                    self.message_builder.setActiveChannels([False, False, False, False, False, False, False, False])
                    self.setIntensity(0)
                    message = self.message_builder.getMessage()
                    self.com.write(message)
                    sendOK = False

            else:
                # Prepare and send message Pulse by Pulse
                sendOK = True
                self.message_builder.setHighVoltage(1)
                message = self.message_builder.getMessage()

                #hex_string = "".join("%02x" % b for b in message)
                #print(hex_string)

                self.com.write(message)


                dataIn = self.com.read(16)  # Wait and read data
                # print(dataIn.hex())

                try:
                    #intensity = dataIn[14]
                    #self.message_builder.setIntensity(int(self.Intensity))
                    self.message_builder.setIntensity(100)

                    self.State.value = 1
                    # print('Intensity' + str(intensity) + "%")  # print the received data
                except:
                    self.State.value = -2
                    self.connect()

            T = self.message_builder.getStimPeriode()
            self.__TimeOut.value += T
            time.sleep(T)

    # Initializes Connection
    def connect(self):

        # Reconnect
        if self.State.value == -2:
            self.com.close()
            time.sleep(1)

        while self.State.value != 1:

            # Trying to connect
            self.State.value = 0

            # COM Settings
            self.com = serial.Serial("/dev/MOTIMOVE")  # open ttyMM0
            #self.com = serial.Serial("COM3")  # open ttyMM0
            self.com.baudrate = 115200  # set Baud rate to 115200
            self.com.bytesize = 8  # Number of data bits = 8
            self.com.parity = 'N'  # No parity
            self.com.stopbits = 1  # Number of Stop bits = 1
            self.com.timeout = 1   # Timeout 1 s

            # Test connection
            self.message_builder.setIntensity(0)
            self.message_builder.setHighVoltage(0)
            message = self.message_builder.getMessage()
            self.com.write(message)

            try:
                dataIn = self.com.read(16)
                # if there was a response from the Stimulator it is considered ready
                if dataIn:
                    self.State.value = 1
                else:
                    self.State.value = -2
            except:
                self.State.value = -2

    # Closes the connection
    def disconnect(self):
        try:
            self.__KILL_PROCESS.value = 1
            self.com.close()
        except:
            pass



# Testroutine
if __name__ == '__main__':

    myMM_manager = MM_Manager()

    #myMM_manager.connect()

    myMM_manager.start()

    #myMM_manager.setStimFrequency(100)
    #myMM_manager.setActiveChannels([False, False, False, False, False, False, False, False])
    #myMM_manager.setStimPhasewidth([100, 100, 100, 100, 100, 100, 100, 100])
    #myMM_manager.setMaxAmplitude([100, 100, 100, 100, 100, 100, 100, 100])

    # Just to make sure that High Voltage is turned up when you would like to start

    #myMM_manager.setIntensity(0)
    #myMM_manager.message_builder.setHighVoltage(1)
    #message = myMM_manager.message_builder.getMessage()
    #myMM_manager.com.write(message)
    #dataIn = myMM_manager.com.flushInput()

    #time.sleep(0.1)

    # Setup finished

    frequ = 25

    myMM_manager.setStimFrequency(frequ)
    myMM_manager.setActiveChannels([False, False, False, False, False, False, False, False])
    myMM_manager.setIntensity(100)
    myMM_manager.setStimPhasewidth([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])
    myMM_manager.setMaxAmplitude([100, 100, 100, 100, 100, 100, 100, 100])

    myMM_manager.message_builder.setRampingOnorOff(1)

    i = 0

    while i <= 170:

        # Test setup with myMM_Manager.start
        myMM_manager.setActiveChannels([True, False, False, False, True, False, False, False])
        time.sleep(0.6)

        myMM_manager.setActiveChannels([True, True, False, False, True, True, False, False])
        time.sleep(0.6)

        myMM_manager.setActiveChannels([False, True, True, False, False, True, True, False])
        time.sleep(0.6)

        myMM_manager.setActiveChannels([False, False, True, True, False, False, True, True])
        time.sleep(0.6)

        myMM_manager.setActiveChannels([False, False, False, True, False, False, False, True])
        time.sleep(0.6)

        myMM_manager.setActiveChannels([False, False, False, False, False, False, False, False])
        time.sleep(3)

        # Test setup with own messages
        #message = myMM_manager.message_builder.getMessage()
        #myMM_manager.com.write(message)

        #if i == 10:
        #   myMM_manager.setActiveChannels([True, False, False, False, False, False, False, False])

        #if i == 50:
        #    myMM_manager.setActiveChannels([False, False, False, False, False, False, False, False])

        #if i == 90:
        #    i = 0

        #i += 1
        #time.sleep(1/frequ)