## Control Interface for MOTIMOVE 8
## (c) Dipl.-Ing. Dr. Martin Schmoll, BSc

import numpy as np
import struct as struct
from multiprocessing import Process, Value


class MM_Message_Builder(object):

    # Constants
    MSG_START = b'\xFF'
    MSG_TYPE_PULSE_BY_PULSE = b'\x08'
    MSG_TYPE_PULSE_TRAIN_START = b'\x02'
    MSG_TYPE_PULSE_TRAIN_STOP = b'\x03'

    __MSG_START_TRAIN = b'\xFF,\x03,\x02,\x05'
    __MSG_STOP_TRAIN = b'\xFF,\x03,\x03,\x06'

    PULSE_DELAY_STD = b'\x00'
    PULSE_DELAY_OFF = b'\xAB'

    SENSOR_AI = b'\x00'
    SENSOR_S1 = b'\x01'
    SENSOR_S2 = b'\x02'

    HIGH_VOLTAGE_OFF = b'\x00'
    HIGH_VOLTAGE_ON = b'\x01'
    HIGH_VOLTAGE_DONT_CHANGE = b'\x02'

    RAMPING_UP = 1
    RAMPING_DOWN = -1
    NO_RAMPING = 0

    CH1 = 1
    CH2 = 2
    CH3 = 3
    CH4 = 4
    CH5 = 5
    CH6 = 6
    CH7 = 7
    CH8 = 8

    AVAL_COMPENSATION = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                         25, 26, 27, 28, 29, 30, 31, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
                         48, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
                         74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
                         98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
                         117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135,
                         136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
                         155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170]

    def __init__(self):

        # Active Channels
        self.__Ch1_active = Value('i', 0)
        self.__Ch2_active = Value('i', 0)
        self.__Ch3_active = Value('i', 0)
        self.__Ch4_active = Value('i', 0)
        self.__Ch5_active = Value('i', 0)
        self.__Ch6_active = Value('i', 0)
        self.__Ch7_active = Value('i', 0)
        self.__Ch8_active = Value('i', 0)

        # BOOST mode
        self.__BOOST_MODE = Value('i', 0)

        # Phasewidths in [µs]
        self.__PhW1 = Value('i', 100)
        self.__PhW2 = Value('i', 100)
        self.__PhW3 = Value('i', 100)
        self.__PhW4 = Value('i', 100)
        self.__PhW5 = Value('i', 100)
        self.__PhW6 = Value('i', 100)
        self.__PhW7 = Value('i', 100)
        self.__PhW8 = Value('i', 100)

        # Phasewidths during BOOST in [µs]
        self.__PhW1_BOOST = Value('i', 0)
        self.__PhW2_BOOST = Value('i', 0)
        self.__PhW3_BOOST = Value('i', 0)
        self.__PhW4_BOOST = Value('i', 0)
        self.__PhW5_BOOST = Value('i', 0)
        self.__PhW6_BOOST = Value('i', 0)
        self.__PhW7_BOOST = Value('i', 0)
        self.__PhW8_BOOST = Value('i', 0)

        # Maximal Stimulation Amplitudes in [mA]
        self.__A1_max = Value('i', 100)
        self.__A2_max = Value('i', 100)
        self.__A3_max = Value('i', 100)
        self.__A4_max = Value('i', 100)
        self.__A5_max = Value('i', 100)
        self.__A6_max = Value('i', 100)
        self.__A7_max = Value('i', 100)
        self.__A8_max = Value('i', 100)

        # Stimulation Intensity in [%]
        self.__Intensity = Value('i', 10)

        # Frequencies in [Hz]
        self.__F = Value('i', 0)
        self.__F_BOOST = Value('i', 0)

        # Stimulation Periode in [ms]
        self.__T = Value('i', 10)
        self.__T_BOOST = Value('i', 10)

        # Frequency PreScaler
        self.__Ch1_PreScaler = Value('i', 1)
        self.__Ch2_PreScaler = Value('i', 1)
        self.__Ch3_PreScaler = Value('i', 1)
        self.__Ch4_PreScaler = Value('i', 1)
        self.__Ch5_PreScaler = Value('i', 1)
        self.__Ch6_PreScaler = Value('i', 1)
        self.__Ch7_PreScaler = Value('i', 1)
        self.__Ch8_PreScaler = Value('i', 1)

        # Pulse Delay
        self.__Pulse_Delay = Value('i', 0)      # 0 .. PULSE_DELAY_STD
                                                # 1 .. PULSE_DELAY_OFF -> simultaneous pulses -> max. 100mA

        # Doublets
        self.__Doublet_Flag = Value('i', 0)     # 0 .. doublets off
                                                # 1 .. doublets active

        # Interstimulus Interval for Doublets in steps of 100 µs; range 2.7 - 10 ms (27 - 100)
        self.__Doublet_ISI = Value('i', 0)

        # Sensor Input
        self.__Sensor_Input = Value('i', 0)     # 0 .. MM_Message_Builder.SENSOR_AI
                                                # 1 .. MM_Message_Builder.SENSOR_S1
                                                # 2 .. MM_Message_Builder.SENSOR_S2

        # High Voltage
        self.__High_Voltage = Value('i', 0)     # 0 .. HIGH_VOLTAGE_OFF
                                                # 1 .. HIGH_VOLTAGE_ON
                                                # 2 .. HIGH_VOLTAGE_DONT_CHANGE

        # Ramp activation Flag
        # 0 means off
        # 1 means on
        self.__rampOnorOff = Value('i', 1)

        # Ramp values in %
        self.__CH1_ramp = Value('f', 0)
        self.__CH2_ramp = Value('f', 0)
        self.__CH3_ramp = Value('f', 0)
        self.__CH4_ramp = Value('f', 0)
        self.__CH5_ramp = Value('f', 0)
        self.__CH6_ramp = Value('f', 0)
        self.__CH7_ramp = Value('f', 0)
        self.__CH8_ramp = Value('f', 0)

        # time for ramping up in ms
        self.__CH1_rampup_time = Value('i', 1000)
        self.__CH2_rampup_time = Value('i', 750)
        self.__CH3_rampup_time = Value('i', 500)
        self.__CH4_rampup_time = Value('i', 250)
        self.__CH5_rampup_time = Value('i', 1000)
        self.__CH6_rampup_time = Value('i', 750)
        self.__CH7_rampup_time = Value('i', 500)
        self.__CH8_rampup_time = Value('i', 250)

        # time for ramping down in ms
        self.__CH1_rampdown_time = Value('i', 250)
        self.__CH2_rampdown_time = Value('i', 500)
        self.__CH3_rampdown_time = Value('i', 750)
        self.__CH4_rampdown_time = Value('i', 1000)
        self.__CH5_rampdown_time = Value('i', 250)
        self.__CH6_rampdown_time = Value('i', 500)
        self.__CH7_rampdown_time = Value('i', 750)
        self.__CH8_rampdown_time = Value('i', 1000)

        self.__rampup_startvalue = Value('i', 25)   # starting value for ramping up in %
        self.__rampdown_endvalue = Value('i', 50)   # end value for ramping down in %

        # Ramping Counters, Factors and Offsets used for calculating the individual peaks during the ramping process
        self.__CH1_rampCounter = Value('i', 0)
        self.__CH2_rampCounter = Value('i', 0)
        self.__CH3_rampCounter = Value('i', 0)
        self.__CH4_rampCounter = Value('i', 0)
        self.__CH5_rampCounter = Value('i', 0)
        self.__CH6_rampCounter = Value('i', 0)
        self.__CH7_rampCounter = Value('i', 0)
        self.__CH8_rampCounter = Value('i', 0)

        self.__CH1_rampFactor = Value('f', 0)
        self.__CH2_rampFactor = Value('f', 0)
        self.__CH3_rampFactor = Value('f', 0)
        self.__CH4_rampFactor = Value('f', 0)
        self.__CH5_rampFactor = Value('f', 0)
        self.__CH6_rampFactor = Value('f', 0)
        self.__CH7_rampFactor = Value('f', 0)
        self.__CH8_rampFactor = Value('f', 0)

        self.__CH1_rampOffset = Value('f', 0)
        self.__CH2_rampOffset = Value('f', 0)
        self.__CH3_rampOffset = Value('f', 0)
        self.__CH4_rampOffset = Value('f', 0)
        self.__CH5_rampOffset = Value('f', 0)
        self.__CH6_rampOffset = Value('f', 0)
        self.__CH7_rampOffset = Value('f', 0)
        self.__CH8_rampOffset = Value('f', 0)

        # Ramping Flags used to activate ramping
        # 0 means no ramping / regular stimulation
        # 1 means ramping upwards
        # -1 means ramping downwards
        self.__CH1_rampFlag = Value('i', 0)
        self.__CH2_rampFlag = Value('i', 0)
        self.__CH3_rampFlag = Value('i', 0)
        self.__CH4_rampFlag = Value('i', 0)
        self.__CH5_rampFlag = Value('i', 0)
        self.__CH6_rampFlag = Value('i', 0)
        self.__CH7_rampFlag = Value('i', 0)
        self.__CH8_rampFlag = Value('i', 0)

        # Channelstate Markers for identifying when to activate ramping
        self.__CH1_oldState = Value('i', 0)
        self.__CH2_oldState = Value('i', 0)
        self.__CH3_oldState = Value('i', 0)
        self.__CH4_oldState = Value('i', 0)
        self.__CH5_oldState = Value('i', 0)
        self.__CH6_oldState = Value('i', 0)
        self.__CH7_oldState = Value('i', 0)
        self.__CH8_oldState = Value('i', 0)

        self.__CH1_newState = Value('i', 0)
        self.__CH2_newState = Value('i', 0)
        self.__CH3_newState = Value('i', 0)
        self.__CH4_newState = Value('i', 0)
        self.__CH5_newState = Value('i', 0)
        self.__CH6_newState = Value('i', 0)
        self.__CH7_newState = Value('i', 0)
        self.__CH8_newState = Value('i', 0)

    # Activates / Deactivates the respective channels
    # Expects boolean array [False, False, False, False, False, False, False, False]
    def setActiveChannels(self, activeChannels):

        if (activeChannels[0]):
            self.__Ch1_active.value = 1
        else:
            self.__Ch1_active.value = 0

        if (activeChannels[1]):
            self.__Ch2_active.value = 1
        else:
            self.__Ch2_active.value = 0

        if (activeChannels[2]):
            self.__Ch3_active.value = 1
        else:
            self.__Ch3_active.value = 0

        if (activeChannels[3]):
            self.__Ch4_active.value = 1
        else:
            self.__Ch4_active.value = 0

        if (activeChannels[4]):
            self.__Ch5_active.value = 1
        else:
            self.__Ch5_active.value = 0

        if (activeChannels[5]):
            self.__Ch6_active.value = 1
        else:
            self.__Ch6_active.value = 0

        if (activeChannels[6]):
            self.__Ch7_active.value = 1
        else:
            self.__Ch7_active.value = 0

        if (activeChannels[7]):
            self.__Ch8_active.value = 1
        else:
            self.__Ch8_active.value = 0

    # Sets the Phasewidth for each channel for normal operation
    def setPhasewidths(self, PhW):

        # Check boundaries
        for i in range(0, 8):

            if (PhW[i] < 0):
                PhW[i] = 0

            if (PhW[i] > 1000):
                PhW[i] = 1000

        # Convert values
        self.__PhW1.value = int(PhW[0] / 10)
        self.__PhW2.value = int(PhW[1] / 10)
        self.__PhW3.value = int(PhW[2] / 10)
        self.__PhW4.value = int(PhW[3] / 10)
        self.__PhW5.value = int(PhW[4] / 10)
        self.__PhW6.value = int(PhW[5] / 10)
        self.__PhW7.value = int(PhW[6] / 10)
        self.__PhW8.value = int(PhW[7] / 10)

    # Sets the Phasewidth for each channel during BOOST in [µs]
    def setPhasewidths_BOOST(self, PhW_BOOST):

        # Check boundaries
        for i in range(0, 8):

            if (PhW_BOOST[i] < 0 ):
                PhW_BOOST[i] = 0

            if (PhW_BOOST[i] > 1000 ):
                PhW_BOOST[i] = 1000

        # Convert values
        self.__PhW1_BOOST.value = int(PhW_BOOST[0] / 10)
        self.__PhW2_BOOST.value = int(PhW_BOOST[1] / 10)
        self.__PhW3_BOOST.value = int(PhW_BOOST[2] / 10)
        self.__PhW4_BOOST.value = int(PhW_BOOST[3] / 10)
        self.__PhW5_BOOST.value = int(PhW_BOOST[4] / 10)
        self.__PhW6_BOOST.value = int(PhW_BOOST[5] / 10)
        self.__PhW7_BOOST.value = int(PhW_BOOST[6] / 10)
        self.__PhW8_BOOST.value = int(PhW_BOOST[7] / 10)

    # Sets the maximal allowed Stimulation amplitudes
    def setMaxAmplitudes(self, A):

        # Check boundaries
        for i in range(0, 8):

            if (A[i] < 0):
                A[i] = 0

            # Standard delayed pulses -> maximum 170 mA
            if (self.__Pulse_Delay.value == 0 &  A[i] > 170):
                A[i] = 170

            # Simultaneously delivered pulses -> maximum 100 mA
            if (self.__Pulse_Delay.value == 1 &  A[i] > 100):
                A[i] = 100

        self.__A1_max.value = A[0]
        self.__A2_max.value = A[1]
        self.__A3_max.value = A[2]
        self.__A4_max.value = A[3]
        self.__A5_max.value = A[4]
        self.__A6_max.value = A[5]
        self.__A7_max.value = A[6]
        self.__A8_max.value = A[7]

    # Sets the intensity in [%] for all channels
    def setIntensity(self, Intensity):

        # Check Value
        if (Intensity < 0):
            Intensity = 0

        if (Intensity > 100):
            Intensity = 100

        self.__Intensity.value = int(Intensity)

    # Returns the current stimulation intensity in [%]
    def getIntensity(self):
        return self.__Intensity.value

    # Activates or deactivates the high-voltage control of the stimulator.
    # 0.. High voltage OFF, 1.. High voltage ON
    def setHighVoltage(self, HighVoltage):

        # Check Value
        if (HighVoltage < 0):
            HighVoltage = 0

        if (HighVoltage > 1):
            HighVoltage = 1

        self.__High_Voltage.value = HighVoltage

    # Activates or deactivates BOOST Mode
    # 0.. BOOST OFF, 1.. BOOST ON
    def setBOOST_Mode(self, BOOST_MODE):

        # Check Value
        if (BOOST_MODE < 0):
            BOOST_MODE = 0

        if (BOOST_MODE > 1):
            BOOST_MODE = 1

        self.__BOOST_MODE.value = BOOST_MODE

    # Sets a new Stimulation Frequency
    # F given in [Hz]
    def setStimFrequency(self, F):

        # Check Value
        if (F < 1 ):
            F = 1

        if (F > 100):
            F = 100

        self.__F.value = int(F)


        # Standard Mode
        TT = np.round(1000 / F)
        if TT < 10:
            TT = 10
        elif TT > 254:
            TT = 254

        self.__T.value = int(TT)

    # Sets a new Stimulation Frequency during BOOST
    # F given in [Hz]
    def setStimFrequency_BOOST(self, F_BOOST):

        # Check Value
        if (F_BOOST < 1):
            F_BOOST = 1

        if (F_BOOST > 100):
            F_BOOST = 100

        self.__F_BOOST.value = int(F_BOOST)

        # BOOST Mode
        TT = np.round(1000 / F_BOOST)
        if TT < 10:
            TT = 10
        elif TT > 254:
            TT = 254

        self.__T_BOOST.value = int(TT)

    # Returns Stimulation periode in [s]
    def getStimPeriode(self):
        if self.__BOOST_MODE.value == 1:
            return 1.0/self.__F_BOOST.value
        else:
            return 1/self.__F.value

    # Returns the current stimulation frequency in [Hz]
    def getFrequency(self):
        return self.__F.value

    # Returns the stimulation frequency during BOOST in [Hz]
    def getFrequency_BOOST(self):
        return self.__F_BOOST.value

    # Returns an array of the Phasewidths in [µs]
    def getPhasewidths(self):
        return [self.__PhW1.value * 10, self.__PhW2.value * 10, self.__PhW3.value * 10, self.__PhW4.value * 10,
                self.__PhW5.value * 10, self.__PhW6.value * 10, self.__PhW7.value * 10, self.__PhW8.value * 10]

    # Returns an array of the Phasewidths during BOOST in [µs]
    def getPhasewidths_BOOST(self):
        return [self.__PhW1_BOOST.value * 10, self.__PhW2_BOOST.value * 10, self.__PhW3_BOOST.value * 10, self.__PhW4_BOOST.value * 10,
                self.__PhW5_BOOST.value * 10, self.__PhW6_BOOST.value * 10, self.__PhW7_BOOST.value * 10, self.__PhW8_BOOST.value * 10]

    # Returns an array of the maximal Amplitudes in [mA]
    def getAmplitudesMax(self):
        return [self.__A1_max.value, self.__A2_max.value, self.__A3_max.value, self.__A4_max.value,
                self.__A5_max.value, self.__A6_max.value, self.__A7_max.value, self.__A8_max.value]

    # Calculates a new Doublet Flag based on a bool input array
    # e.g. doublets on CH1, 7,8 -> [True, False, False, False, False, False, True, True]
    def setDoublets(self, doublet_flags):

        FLAG = b'\x00'

        MASK = b'\x01,\x02,\x04,\x08,\x10,\x20,\x40,\x80'

        for i in range(0, 8):
            if doublet_flags[i]:
                FLAG = bytes(FLAG[0] & MASK[i])

        self.__Doublet_Flag = FLAG

    # sets a new time for ramping up
    # T in [ms]
    def setRampUpTime(self, rampuptime):

        for i in range(0, 8):

            if (rampuptime[i] < 0):
                rampuptime[i] = 0

        self.__CH1_rampup_time.value = int(rampuptime[0])
        self.__CH2_rampup_time.value = int(rampuptime[1])
        self.__CH3_rampup_time.value = int(rampuptime[2])
        self.__CH4_rampup_time.value = int(rampuptime[3])
        self.__CH5_rampup_time.value = int(rampuptime[4])
        self.__CH6_rampup_time.value = int(rampuptime[5])
        self.__CH7_rampup_time.value = int(rampuptime[6])
        self.__CH8_rampup_time.value = int(rampuptime[7])

    # sets a new time for ramping down
    # T in [ms]
    def setRampDownTime(self, rampdowntime):

        for i in range(0, 8):

            if (rampdowntime[i] < 0):
                rampdowntime[i] = 0

        self.__CH1_rampdown_time.value = int(rampdowntime[0])
        self.__CH2_rampdown_time.value = int(rampdowntime[1])
        self.__CH3_rampdown_time.value = int(rampdowntime[2])
        self.__CH4_rampdown_time.value = int(rampdowntime[3])
        self.__CH5_rampdown_time.value = int(rampdowntime[4])
        self.__CH6_rampdown_time.value = int(rampdowntime[5])
        self.__CH7_rampdown_time.value = int(rampdowntime[6])
        self.__CH8_rampdown_time.value = int(rampdowntime[7])

    # sets a new starting value for ramping up in [%]
    def setRamUpStart(self, rampupstartvalue):

        if rampupstartvalue < 0:
            self.__rampup_startvalue.value = 0

        elif rampupstartvalue >= 100:
            self.__rampup_startvalue.value = 100

        else:
            self.__rampup_startvalue.value = rampupstartvalue

    # sets a new end value for ramping down in  [%]
    def setRampDownEnd(self, rampdownendvalue):

        if rampdownendvalue < 0:
            self.__rampdown_endvalue.value = 0

        elif rampdownendvalue >= 100:
            self.__rampdown_endvalue.value = 100

        else:
            self.__rampdown_endvalue.value = rampdownendvalue

    # option to manually set the Counter used for Ramping
    def setRampCounter(self, rampCounter):
        self.__CH1_rampCounter.value = rampCounter

    # activates or deactivates the Ramping, 1 is active, 0 is inactive
    def setRampingOnorOff(self, rampingactivate):
        self.__rampOnorOff.value = rampingactivate

    # returns the time for ramping up in [ms]
    def getRampUpTime(self):
        return [self.__CH1_rampup_time.value, self.__CH2_rampup_time.value, self.__CH3_rampup_time.value, self.__CH4_rampup_time.value,
                self.__CH5_rampup_time.value, self.__CH6_rampup_time.value, self.__CH7_rampup_time.value, self.__CH8_rampup_time.value]

    # returns the time for ramping down in [ms]
    def getRampDownTime(self):
        return [self.__CH1_rampdown_time.value, self.__CH2_rampdown_time.value, self.__CH3_rampdown_time.value, self.__CH4_rampdown_time.value,
                self.__CH5_rampdown_time.value, self.__CH6_rampdown_time.value, self.__CH7_rampdown_time.value, self.__CH8_rampdown_time.value]

    # returns the starting value for ramping up in [%]
    def getRampUpStart(self):
        return self.__rampup_startvalue.value

    # returns the end value for ramping down in [%]
    def getRampDownEnd(self):
        return self.__rampdown_endvalue.value

    # Calculation of the individual ramping peaks for upwards ramping of CH1
    def rampUpCH1(self):

        if self.__CH1_rampCounter.value == 0:     # if we enter the ramping for the first time, we want to calculate the required startvalue and stepheight for each following ramping pulse
            n = self.__F.value * self.__CH1_rampup_time.value / 1000            # calculate how much ramping steps are needed
            if self.__CH1_ramp.value < self.__rampup_startvalue.value:          # checking if starting from under the minimum starting value
                self.__CH1_rampOffset.value = self.__rampup_startvalue.value    # starting value
                self.__CH1_rampFactor.value = (100.0 - self.__CH1_rampOffset.value) / n     # calculation of step height
                self.__CH1_ramp.value = self.__rampup_startvalue.value          # setting the current ramp value
                self.__CH1_rampCounter.value += 1

            elif self.__CH1_ramp.value >= 100:                                  # if we start at 100% already we want to deativate ramping and keep the value at 100
                self.__CH1_ramp.value = 100
                self.__CH1_rampCounter.value = 0
                self.__CH1_rampFlag.value = 0

            else:
                self.__CH1_rampOffset.value = int(self.__CH1_ramp.value)        # starting from anywhere else, we want to start from the current point, calculating startvalue and stepheight
                self.__CH1_rampFactor.value = (100 - self.__CH1_rampOffset.value) / n
                self.__CH1_rampCounter.value += 1

        else:                                                                   # if we enter the ramping any other time than the first one, we want to check if we have exceeded any bounds
            if self.__CH1_ramp.value < self.__rampup_startvalue.value:
                self.__CH1_ramp.value = self.__rampup_startvalue.value
                self.__CH1_rampCounter.value = 1

            elif self.__CH1_ramp.value >= 100:
                self.__CH1_ramp.value = 100
                self.__CH1_rampCounter.value = 0
                self.__CH1_rampFlag.value = 0

            else:                                                               # if every check has been correct, we want to ramp upwards by calculating the next ramped impulse
                self.__CH1_ramp.value = (self.__CH1_rampFactor.value * self.__CH1_rampCounter.value) + self.__CH1_rampOffset.value
                self.__CH1_rampCounter.value += 1

        if self.__CH1_ramp.value >= 100:                                        # when the ramping has reached 100, it has finished and is deactivated
            self.__CH1_rampCounter.value = 0
            self.__CH1_ramp.value = 100
            self.__CH1_rampFlag.value = 0

        return self.__CH1_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH2, for exact explanation see CH1
    def rampUpCH2(self):

        if self.__CH2_rampCounter.value == 0:
            n = self.__F.value * self.__CH2_rampup_time.value / 1000.0
            if self.__CH2_ramp.value < self.__rampup_startvalue.value:
                self.__CH2_rampOffset.value = self.__rampup_startvalue.value
                self.__CH2_rampFactor.value = (100.0 - self.__CH2_rampOffset.value) / n
                self.__CH2_ramp.value = self.__rampup_startvalue.value
                self.__CH2_rampCounter.value += 1

            elif self.__CH2_ramp.value >= 100:
                self.__CH2_ramp.value = 100
                self.__CH2_rampCounter.value = 0
                self.__CH2_rampFlag.value = 0

            else:
                self.__CH2_rampOffset.value = int(self.__CH2_ramp.value)
                self.__CH2_rampFactor.value = (100.0 - self.__CH2_rampOffset.value) / n
                self.__CH2_rampCounter.value += 1

        else:
            if self.__CH2_ramp.value < self.__rampup_startvalue.value:
                self.__CH2_ramp.value = self.__rampup_startvalue.value
                self.__CH2_rampCounter.value = 1

            elif self.__CH2_ramp.value >= 100:
                self.__CH2_ramp.value = 100
                self.__CH2_rampCounter.value = 0
                self.__CH2_rampFlag.value = 0

            else:
                self.__CH2_ramp.value = (self.__CH2_rampFactor.value * self.__CH2_rampCounter.value) + self.__CH2_rampOffset.value
                self.__CH2_rampCounter.value += 1

        if self.__CH2_ramp.value >= 100:
            self.__CH2_rampCounter.value = 0
            self.__CH2_ramp.value = 100
            self.__CH2_rampFlag.value = 0

        return self.__CH2_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH3, for exact explanation see CH1
    def rampUpCH3(self):

        if self.__CH3_rampCounter.value == 0:
            n = self.__F.value * self.__CH3_rampup_time.value / 1000.0
            if self.__CH3_ramp.value < self.__rampup_startvalue.value:
                self.__CH3_rampOffset.value = self.__rampup_startvalue.value
                self.__CH3_rampFactor.value = (100.0 - self.__CH3_rampOffset.value) / n
                self.__CH3_ramp.value = self.__rampup_startvalue.value
                self.__CH3_rampCounter.value += 1

            elif self.__CH3_ramp.value >= 100:
                self.__CH3_ramp.value = 100
                self.__CH3_rampCounter.value = 0
                self.__CH3_rampFlag.value = 0

            else:
                self.__CH3_rampOffset.value = int(self.__CH3_ramp.value)
                self.__CH3_rampFactor.value = (100.0 - self.__CH3_rampOffset.value) / n
                self.__CH3_rampCounter.value += 1

        else:
            if self.__CH3_ramp.value < self.__rampup_startvalue.value:
                self.__CH3_ramp.value = self.__rampup_startvalue.value
                self.__CH3_rampCounter.value = 1

            elif self.__CH3_ramp.value >= 100:
                self.__CH3_ramp.value = 100
                self.__CH3_rampCounter.value = 0
                self.__CH3_rampFlag.value = 0

            else:
                self.__CH3_ramp.value = (self.__CH3_rampFactor.value * self.__CH3_rampCounter.value) + self.__CH3_rampOffset.value
                self.__CH3_rampCounter.value += 1

        if self.__CH3_ramp.value >= 100:
            self.__CH3_rampCounter.value = 0
            self.__CH3_ramp.value = 100
            self.__CH3_rampFlag.value = 0

        return self.__CH3_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH4, for exact explanation see CH1
    def rampUpCH4(self):

        if self.__CH4_rampCounter.value == 0:
            n = self.__F.value * self.__CH4_rampup_time.value / 1000.0
            if self.__CH4_ramp.value < self.__rampup_startvalue.value:
                self.__CH4_rampOffset.value = self.__rampup_startvalue.value
                self.__CH4_rampFactor.value = (100.0 - self.__CH4_rampOffset.value) / n
                self.__CH4_ramp.value = self.__rampup_startvalue.value
                self.__CH4_rampCounter.value += 1

            elif self.__CH4_ramp.value >= 100:
                self.__CH4_ramp.value = 100
                self.__CH4_rampCounter.value = 0
                self.__CH4_rampFlag.value = 0

            else:
                self.__CH4_rampOffset.value = int(self.__CH4_ramp.value)
                self.__CH4_rampFactor.value = (100.0 - self.__CH4_rampOffset.value) / n
                self.__CH4_rampCounter.value += 1

        else:
            if self.__CH4_ramp.value < self.__rampup_startvalue.value:
                self.__CH4_ramp.value = self.__rampup_startvalue.value
                self.__CH4_rampCounter.value = 1

            elif self.__CH4_ramp.value >= 100:
                self.__CH4_ramp.value = 100
                self.__CH4_rampCounter.value = 0
                self.__CH4_rampFlag.value = 0

            else:
                self.__CH4_ramp.value = (self.__CH4_rampFactor.value * self.__CH4_rampCounter.value) + self.__CH4_rampOffset.value
                self.__CH4_rampCounter.value += 1

        if self.__CH4_ramp.value >= 100:
            self.__CH4_rampCounter.value = 0
            self.__CH4_ramp.value = 100
            self.__CH4_rampFlag.value = 0

        return self.__CH4_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH5, for exact explanation see CH1
    def rampUpCH5(self):

        if self.__CH5_rampCounter.value == 0:
            n = self.__F.value * self.__CH5_rampup_time.value / 1000.0
            if self.__CH5_ramp.value < self.__rampup_startvalue.value:
                self.__CH5_rampOffset.value = self.__rampup_startvalue.value
                self.__CH5_rampFactor.value = (100.0 - self.__CH5_rampOffset.value) / n
                self.__CH5_ramp.value = self.__rampup_startvalue.value
                self.__CH5_rampCounter.value += 1

            elif self.__CH5_ramp.value >= 100:
                self.__CH5_ramp.value = 100
                self.__CH5_rampCounter.value = 0
                self.__CH5_rampFlag.value = 0

            else:
                self.__CH5_rampOffset.value = int(self.__CH5_ramp.value)
                self.__CH5_rampFactor.value = (100.0 - self.__CH5_rampOffset.value) / n
                self.__CH5_rampCounter.value += 1

        else:
            if self.__CH5_ramp.value < self.__rampup_startvalue.value:
                self.__CH5_ramp.value = self.__rampup_startvalue.value
                self.__CH5_rampCounter.value = 1

            elif self.__CH5_ramp.value >= 100:
                self.__CH5_ramp.value = 100
                self.__CH5_rampCounter.value = 0
                self.__CH5_rampFlag.value = 0

            else:
                self.__CH5_ramp.value = (self.__CH5_rampFactor.value * self.__CH5_rampCounter.value) + self.__CH5_rampOffset.value
                self.__CH5_rampCounter.value += 1

        if self.__CH5_ramp.value >= 100:
            self.__CH5_rampCounter.value = 0
            self.__CH5_ramp.value = 100
            self.__CH5_rampFlag.value = 0

        return self.__CH5_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH6, for exact explanation see CH1
    def rampUpCH6(self):

        if self.__CH6_rampCounter.value == 0:
            n = self.__F.value * self.__CH6_rampup_time.value / 1000.0
            if self.__CH6_ramp.value < self.__rampup_startvalue.value:
                self.__CH6_rampOffset.value = self.__rampup_startvalue.value
                self.__CH6_rampFactor.value = (100.0 - self.__CH6_rampOffset.value) / n
                self.__CH6_ramp.value = self.__rampup_startvalue.value
                self.__CH6_rampCounter.value += 1

            elif self.__CH6_ramp.value >= 100:
                self.__CH6_ramp.value = 100
                self.__CH6_rampCounter.value = 0
                self.__CH6_rampFlag.value = 0

            else:
                self.__CH6_rampOffset.value = int(self.__CH6_ramp.value)
                self.__CH6_rampFactor.value = (100.0 - self.__CH6_rampOffset.value) / n
                self.__CH6_rampCounter.value += 1

        else:
            if self.__CH6_ramp.value < self.__rampup_startvalue.value:
                self.__CH6_ramp.value = self.__rampup_startvalue.value
                self.__CH6_rampCounter.value = 1

            elif self.__CH6_ramp.value >= 100:
                self.__CH6_ramp.value = 100
                self.__CH6_rampCounter.value = 0
                self.__CH6_rampFlag.value = 0

            else:
                self.__CH6_ramp.value = (self.__CH6_rampFactor.value * self.__CH6_rampCounter.value) + self.__CH6_rampOffset.value
                self.__CH6_rampCounter.value += 1

        if self.__CH6_ramp.value >= 100:
            self.__CH6_rampCounter.value = 0
            self.__CH6_ramp.value = 100
            self.__CH6_rampFlag.value = 0

        return self.__CH6_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH7, for exact explanation see CH1
    def rampUpCH7(self):

        if self.__CH7_rampCounter.value == 0:
            n = self.__F.value * self.__CH7_rampup_time.value / 1000.0
            if self.__CH7_ramp.value < self.__rampup_startvalue.value:
                self.__CH7_rampOffset.value = self.__rampup_startvalue.value
                self.__CH7_rampFactor.value = (100.0 - self.__CH7_rampOffset.value) / n
                self.__CH7_ramp.value = self.__rampup_startvalue.value
                self.__CH7_rampCounter.value += 1

            elif self.__CH7_ramp.value >= 100:
                self.__CH7_ramp.value = 100
                self.__CH7_rampCounter.value = 0
                self.__CH7_rampFlag.value = 0

            else:
                self.__CH7_rampOffset.value = int(self.__CH7_ramp.value)
                self.__CH7_rampFactor.value = (100.0 - self.__CH7_rampOffset.value) / n
                self.__CH7_rampCounter.value += 1

        else:
            if self.__CH7_ramp.value < self.__rampup_startvalue.value:
                self.__CH7_ramp.value = self.__rampup_startvalue.value
                self.__CH7_rampCounter.value = 1

            elif self.__CH7_ramp.value >= 100:
                self.__CH7_ramp.value = 100
                self.__CH7_rampCounter.value = 0
                self.__CH7_rampFlag.value = 0

            else:
                self.__CH7_ramp.value = (self.__CH7_rampFactor.value * self.__CH7_rampCounter.value) + self.__CH7_rampOffset.value
                self.__CH7_rampCounter.value += 1

        if self.__CH7_ramp.value >= 100:
            self.__CH7_rampCounter.value = 0
            self.__CH7_ramp.value = 100
            self.__CH7_rampFlag.value = 0

        return self.__CH7_ramp.value

    # Calculation of the individual ramping peaks for upwards ramping of CH8, for exact explanation see CH1
    def rampUpCH8(self):

        if self.__CH8_rampCounter.value == 0:
            n = self.__F.value * self.__CH8_rampup_time.value / 1000.0
            if self.__CH8_ramp.value < self.__rampup_startvalue.value:
                self.__CH8_rampOffset.value = self.__rampup_startvalue.value
                self.__CH8_rampFactor.value = (100.0 - self.__CH8_rampOffset.value) / n
                self.__CH8_ramp.value = self.__rampup_startvalue.value
                self.__CH8_rampCounter.value += 1

            elif self.__CH8_ramp.value >= 100:
                self.__CH8_ramp.value = 100
                self.__CH8_rampCounter.value = 0
                self.__CH8_rampFlag.value = 0

            else:
                self.__CH8_rampOffset.value = int(self.__CH8_ramp.value)
                self.__CH8_rampFactor.value = (100.0 - self.__CH8_rampOffset.value) / n
                self.__CH8_rampCounter.value += 1

        else:
            if self.__CH8_ramp.value < self.__rampup_startvalue.value:
                self.__CH8_ramp.value = self.__rampup_startvalue.value
                self.__CH8_rampCounter.value = 1

            elif self.__CH8_ramp.value >= 100:
                self.__CH8_ramp.value = 100
                self.__CH8_rampCounter.value = 0
                self.__CH8_rampFlag.value = 0

            else:
                self.__CH8_ramp.value = (self.__CH8_rampFactor.value * self.__CH8_rampCounter.value) + self.__CH8_rampOffset.value
                self.__CH8_rampCounter.value += 1

        if self.__CH8_ramp.value >= 100:
            self.__CH8_rampCounter.value = 0
            self.__CH8_ramp.value = 100
            self.__CH8_rampFlag.value = 0

        return self.__CH8_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH1
    def rampDownCH1(self):

        if self.__CH1_rampCounter.value == 0:       # if we enter the ramping for the first time, we want to calculate the required startvalue and stepheight for each following ramping pulse
            n = self.__F.value * self.__CH1_rampdown_time.value / 1000                  # calculate how much ramping steps are needed
            if self.__CH1_ramp.value >= 100:                                            # checking if the ramping is started from full stimulaiton
                self.__CH1_rampOffset.value = 100                                       # setting the startvalue
                self.__CH1_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH1_rampOffset.value) / n    # calculation of the step height
                self.__CH1_ramp.value = self.__CH1_rampOffset.value                     # setting the current ramp value
                self.__CH1_rampCounter.value += 1
                self.__Ch1_active.value = 1                                             # the channel needs to be actively set to 1 to stay active
                                                                                        # important to deactivate it afterwards, that happens in the function getmessage

            elif self.__CH1_ramp.value < self.__rampdown_endvalue.value:               # if the current value is already below the endvalue, the ramping can be deactivated
                self.__CH1_ramp.value = 0
                self.__CH1_rampCounter.value = 0
                self.__CH1_rampFlag.value = 0
                self.__Ch1_active.value = 1

            else:       # if the current value is somewhere in between 100 and the endvalue, we want to start from that value, so we need to calculate the starting value and step height from here
                self.__CH1_rampOffset.value = int(self.__CH1_ramp.value)
                self.__CH1_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH1_rampOffset.value) / n
                self.__CH1_rampCounter.value += 1
                self.__Ch1_active.value = 1

        else:                                                                       # if we enter the ramping any other time than the first one, we want to check if we have exceeded any bounds
            if self.__CH1_ramp.value > 100:
                self.__CH1_ramp.value = 100
                self.__CH1_rampCounter.value = 1
                self.__Ch1_active.value = 1

            elif self.__CH1_ramp.value < self.__rampdown_endvalue.value:
                    self.__CH1_ramp.value = self.__rampdown_endvalue.value
                    self.__CH1_rampCounter.value = 0
                    self.__CH1_rampFlag.value = 0

            else:                                                                       # if every check has been correct, we want to ramp downwards by calculating the next ramped impulse
                self.__CH1_ramp.value = (self.__CH1_rampFactor.value * self.__CH1_rampCounter.value) + self.__CH1_rampOffset.value
                self.__Ch1_active.value = 1
                self.__CH1_rampCounter.value += 1

        if self.__CH1_ramp.value < self.__rampdown_endvalue.value:                      # when the ramping has reached its endvalue, it has finished and is deactivated
            self.__CH1_ramp.value = 0
            self.__CH1_rampCounter.value = 0
            self.__Ch1_active.value = 0
            self.__CH1_rampFlag.value = 0

        return self.__CH1_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH2, for exact explanation see CH1
    def rampDownCH2(self):

        if self.__CH2_rampCounter.value == 0:
            n = self.__F.value * self.__CH2_rampdown_time.value / 1000.0
            if self.__CH2_ramp.value > 100:
                self.__CH2_rampOffset.value = 100
                self.__CH2_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH2_rampOffset.value) / n
                self.__CH2_ramp.value = self.__CH2_rampOffset.value
                self.__CH2_rampCounter.value += 1
                self.__Ch2_active.value = 1

            elif self.__CH2_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH2_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH2_rampCounter.value = 0
                self.__CH2_rampFlag.value = 0
                self.__Ch2_active.value = 1

            else:
                self.__CH2_rampOffset.value = int(self.__CH2_ramp.value)
                self.__CH2_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH2_rampOffset.value) / n
                self.__CH2_rampCounter.value += 1
                self.__Ch2_active.value = 1

        else:
            if self.__CH2_ramp.value > 100:
                self.__CH2_ramp.value = 100
                self.__CH2_rampCounter.value = 1
                self.__Ch2_active.value = 1

            elif self.__CH2_ramp.value < self.__rampdown_endvalue.value:
                self.__CH2_ramp.value = self.__rampdown_endvalue.value
                self.__CH2_rampCounter.value = 0
                self.__CH2_rampFlag.value = 0

            else:
                self.__CH2_ramp.value = (self.__CH2_rampFactor.value * self.__CH2_rampCounter.value) + self.__CH2_rampOffset.value
                self.__Ch2_active.value = 1
                self.__CH2_rampCounter.value += 1

        if self.__CH2_ramp.value < self.__rampdown_endvalue.value:
            self.__CH2_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH2_rampCounter.value = 0
            self.__Ch2_active.value = 0
            self.__CH2_rampFlag.value = 0

        return self.__CH2_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH3, for exact explanation see CH1
    def rampDownCH3(self):

        if self.__CH3_rampCounter.value == 0:
            n = self.__F.value * self.__CH3_rampdown_time.value / 1000.0
            if self.__CH3_ramp.value > 100:
                self.__CH3_rampOffset.value = 100
                self.__CH3_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH3_rampOffset.value) / n
                self.__CH3_ramp.value = self.__CH3_rampOffset.value
                self.__CH3_rampCounter.value += 1
                self.__Ch3_active.value = 1

            elif self.__CH3_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH3_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH3_rampCounter.value = 0
                self.__CH3_rampFlag.value = 0
                self.__Ch3_active.value = 1

            else:
                self.__CH3_rampOffset.value = int(self.__CH3_ramp.value)
                self.__CH3_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH3_rampOffset.value) / n
                self.__CH3_rampCounter.value += 1
                self.__Ch3_active.value = 1

        else:
            if self.__CH3_ramp.value > 100:
                self.__CH3_ramp.value = 100
                self.__CH3_rampCounter.value = 1
                self.__Ch3_active.value = 1

            elif self.__CH3_ramp.value < self.__rampdown_endvalue.value:
                self.__CH3_ramp.value = self.__rampdown_endvalue.value
                self.__CH3_rampCounter.value = 0
                self.__CH3_rampFlag.value = 0

            else:
                self.__CH3_ramp.value = (self.__CH3_rampFactor.value * self.__CH3_rampCounter.value) + self.__CH3_rampOffset.value
                self.__Ch3_active.value = 1
                self.__CH3_rampCounter.value += 1

        if self.__CH3_ramp.value < self.__rampdown_endvalue.value:
            self.__CH3_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH3_rampCounter.value = 0
            self.__Ch3_active.value = 0
            self.__CH3_rampFlag.value = 0

        return self.__CH3_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH4, for exact explanation see CH1
    def rampDownCH4(self):

        if self.__CH4_rampCounter.value == 0:
            n = self.__F.value * self.__CH4_rampdown_time.value / 1000.0
            if self.__CH4_ramp.value > 100:
                self.__CH4_rampOffset.value = 100
                self.__CH4_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH4_rampOffset.value) / n
                self.__CH4_ramp.value = self.__CH4_rampOffset.value
                self.__CH4_rampCounter.value += 1
                self.__Ch4_active.value = 1

            elif self.__CH4_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH4_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH4_rampCounter.value = 0
                self.__CH4_rampFlag.value = 0
                self.__Ch4_active.value = 1

            else:
                self.__CH4_rampOffset.value = int(self.__CH4_ramp.value)
                self.__CH4_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH4_rampOffset.value) / n
                self.__CH4_rampCounter.value += 1
                self.__Ch4_active.value = 1

        else:
            if self.__CH4_ramp.value > 100:
                self.__CH4_ramp.value = 100
                self.__CH4_rampCounter.value = 1
                self.__Ch4_active.value = 1

            elif self.__CH4_ramp.value < self.__rampdown_endvalue.value:
                self.__CH4_ramp.value = self.__rampdown_endvalue.value
                self.__CH4_rampCounter.value = 0
                self.__CH4_rampFlag.value = 0

            else:
                self.__CH4_ramp.value = (self.__CH4_rampFactor.value * self.__CH4_rampCounter.value) + self.__CH4_rampOffset.value
                self.__Ch4_active.value = 1
                self.__CH4_rampCounter.value += 1

        if self.__CH4_ramp.value < self.__rampdown_endvalue.value:
            self.__CH4_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH4_rampCounter.value = 0
            self.__Ch4_active.value = 0
            self.__CH4_rampFlag.value = 0

        return self.__CH4_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH5, for exact explanation see CH1
    def rampDownCH5(self):

        if self.__CH5_rampCounter.value == 0:
            n = self.__F.value * self.__CH5_rampdown_time.value / 1000.0
            if self.__CH5_ramp.value > 100:
                self.__CH5_rampOffset.value = 100
                self.__CH5_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH5_rampOffset.value) / n
                self.__CH5_ramp.value = self.__CH5_rampOffset.value
                self.__CH5_rampCounter.value += 1
                self.__Ch5_active.value = 1

            elif self.__CH5_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH5_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH5_rampCounter.value = 0
                self.__CH5_rampFlag.value = 0
                self.__Ch5_active.value = 1

            else:
                self.__CH5_rampOffset.value = int(self.__CH5_ramp.value)
                self.__CH5_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH5_rampOffset.value) / n
                self.__CH5_rampCounter.value += 1
                self.__Ch5_active.value = 1

        else:
            if self.__CH5_ramp.value > 100:
                self.__CH5_ramp.value = 100
                self.__CH5_rampCounter.value = 1
                self.__Ch5_active.value = 1

            elif self.__CH5_ramp.value < self.__rampdown_endvalue.value:
                self.__CH5_ramp.value = self.__rampdown_endvalue.value
                self.__CH5_rampCounter.value = 0
                self.__CH5_rampFlag.value = 0

            else:
                self.__CH5_ramp.value = (self.__CH5_rampFactor.value * self.__CH5_rampCounter.value) + self.__CH5_rampOffset.value
                self.__Ch5_active.value = 1
                self.__CH5_rampCounter.value += 1

        if self.__CH5_ramp.value < self.__rampdown_endvalue.value:
            self.__CH5_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH5_rampCounter.value = 0
            self.__Ch5_active.value = 0
            self.__CH5_rampFlag.value = 0

        return self.__CH5_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH6, for exact explanation see CH1
    def rampDownCH6(self):

        if self.__CH6_rampCounter.value == 0:
            n = self.__F.value * self.__CH6_rampdown_time.value / 1000.0
            if self.__CH6_ramp.value > 100:
                self.__CH6_rampOffset.value = 100
                self.__CH6_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH6_rampOffset.value) / n
                self.__CH6_ramp.value = self.__CH6_rampOffset.value
                self.__CH6_rampCounter.value += 1
                self.__Ch6_active.value = 1

            elif self.__CH6_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH6_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH6_rampCounter.value = 0
                self.__CH6_rampFlag.value = 0
                self.__Ch6_active.value = 1

            else:
                self.__CH6_rampOffset.value = int(self.__CH6_ramp.value)
                self.__CH6_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH6_rampOffset.value) / n
                self.__CH6_rampCounter.value += 1
                self.__Ch6_active.value = 1

        else:
            if self.__CH6_ramp.value > 100:
                self.__CH6_ramp.value = 100
                self.__CH6_rampCounter.value = 1
                self.__Ch6_active.value = 1

            elif self.__CH6_ramp.value < self.__rampdown_endvalue.value:
                self.__CH6_ramp.value = self.__rampdown_endvalue.value
                self.__CH6_rampCounter.value = 0
                self.__CH6_rampFlag.value = 0

            else:
                self.__CH6_ramp.value = (self.__CH6_rampFactor.value * self.__CH6_rampCounter.value) + self.__CH6_rampOffset.value
                self.__Ch6_active.value = 1
                self.__CH6_rampCounter.value += 1

        if self.__CH6_ramp.value < self.__rampdown_endvalue.value:
            self.__CH6_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH6_rampCounter.value = 0
            self.__Ch6_active.value = 0
            self.__CH6_rampFlag.value = 0

        return self.__CH6_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH7, for exact explanation see CH1
    def rampDownCH7(self):

        if self.__CH7_rampCounter.value == 0:
            n = self.__F.value * self.__CH7_rampdown_time.value / 1000.0
            if self.__CH7_ramp.value > 100:
                self.__CH7_rampOffset.value = 100
                self.__CH7_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH7_rampOffset.value) / n
                self.__CH7_ramp.value = self.__CH7_rampOffset.value
                self.__CH7_rampCounter.value += 1
                self.__Ch7_active.value = 1

            elif self.__CH7_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH7_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH7_rampCounter.value = 0
                self.__CH7_rampFlag.value = 0
                self.__Ch7_active.value = 1

            else:
                self.__CH7_rampOffset.value = int(self.__CH7_ramp.value)
                self.__CH7_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH7_rampOffset.value) / n
                self.__CH7_rampCounter.value += 1
                self.__Ch7_active.value = 1

        else:
            if self.__CH7_ramp.value > 100:
                self.__CH7_ramp.value = 100
                self.__CH7_rampCounter.value = 1
                self.__Ch7_active.value = 1

            elif self.__CH7_ramp.value < self.__rampdown_endvalue.value:
                self.__CH7_ramp.value = self.__rampdown_endvalue.value
                self.__CH7_rampCounter.value = 0
                self.__CH7_rampFlag.value = 0

            else:
                self.__CH7_ramp.value = (
                                                    self.__CH7_rampFactor.value * self.__CH7_rampCounter.value) + self.__CH7_rampOffset.value
                self.__Ch7_active.value = 1
                self.__CH7_rampCounter.value += 1

        if self.__CH7_ramp.value < self.__rampdown_endvalue.value:
            self.__CH7_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH7_rampCounter.value = 0
            self.__Ch7_active.value = 0
            self.__CH7_rampFlag.value = 0

        return self.__CH7_ramp.value

    # Calculation of the individual ramping peaks for downwarding ramping of CH8, for exact explanation see CH1
    def rampDownCH8(self):

        if self.__CH8_rampCounter.value == 0:
            n = self.__F.value * self.__CH8_rampdown_time.value / 1000.0
            if self.__CH8_ramp.value > 100:
                self.__CH8_rampOffset.value = 100
                self.__CH8_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH8_rampOffset.value) / n
                self.__CH8_ramp.value = self.__CH8_rampOffset.value
                self.__CH8_rampCounter.value += 1
                self.__Ch8_active.value = 1

            elif self.__CH8_ramp.value <= self.__rampdown_endvalue.value:
                self.__CH8_ramp.value = int(self.__rampdown_endvalue.value)
                self.__CH8_rampCounter.value = 0
                self.__CH8_rampFlag.value = 0
                self.__Ch8_active.value = 1

            else:
                self.__CH8_rampOffset.value = int(self.__CH8_ramp.value)
                self.__CH8_rampFactor.value = (self.__rampdown_endvalue.value - self.__CH8_rampOffset.value) / n
                self.__CH8_rampCounter.value += 1
                self.__Ch8_active.value = 1

        else:
            if self.__CH8_ramp.value > 100:
                self.__CH8_ramp.value = 100
                self.__CH8_rampCounter.value = 1
                self.__Ch8_active.value = 1

            elif self.__CH8_ramp.value < self.__rampdown_endvalue.value:
                self.__CH8_ramp.value = self.__rampdown_endvalue.value
                self.__CH8_rampCounter.value = 0
                self.__CH8_rampFlag.value = 0

            else:
                self.__CH8_ramp.value = (
                                                self.__CH8_rampFactor.value * self.__CH8_rampCounter.value) + self.__CH8_rampOffset.value
                self.__Ch8_active.value = 1
                self.__CH8_rampCounter.value += 1

        if self.__CH8_ramp.value < self.__rampdown_endvalue.value:
            self.__CH8_ramp.value = int(self.__rampdown_endvalue.value)
            self.__CH8_rampCounter.value = 0
            self.__Ch8_active.value = 0
            self.__CH8_rampFlag.value = 0

        return self.__CH8_ramp.value

    # function for determining which channels need to be ramped at the moment
    def toRamp_or_not_to_Ramp(self, channel):

        if channel == self.CH1:                 # Decision tree for ramping CH1

            if (self.__CH1_newState.value == 0 and self.__CH1_oldState.value == 0 and not self.__CH1_rampFlag.value == -1) \
                    or (self.__CH1_newState.value == 1 and self.__CH1_oldState.value == 1 and not self.__CH1_rampFlag.value == 1) \
                    or self.__CH1_rampFlag.value == 0:                          # if no change in the state of the channel has occured and it is not already ramping, no ramping is needed
                self.__CH1_rampFlag.value = self.NO_RAMPING

            # if the channel has been switched on or it is already ramping upwards, we want to ramp upwards
            if (self.__CH1_newState.value == 1 and self.__CH1_oldState.value == 0) or self.__CH1_rampFlag.value == 1:
                self.__CH1_rampFlag.value = self.RAMPING_UP

            # if the channel has been switched off or it is already ramping downwards, we want to ramp downwards
            if (self.__CH1_newState.value == 0 and self.__CH1_oldState.value == 1) or self.__CH1_rampFlag.value == -1:
                self.__CH1_rampFlag.value = self.RAMPING_DOWN

            if self.__CH1_rampFlag.value == self.NO_RAMPING:                    # when we are not ramping, we want to set the stimulation to either no stimulation or full stimulation
                if self.__Ch1_active.value == 0:
                    self.__CH1_ramp.value = 0
                else:
                    self.__CH1_ramp.value = 100

            if self.__CH1_rampFlag.value == self.RAMPING_UP:                    # when ramping upwards is active, if it is the first time, the ramping counter is set to start from 0
                if self.__CH1_newState.value == 1 and self.__CH1_oldState.value == 0:
                    self.__CH1_rampCounter.value = 0
                self.rampUpCH1()                                                # the function for ramping up is called

            if self.__CH1_rampFlag.value == self.RAMPING_DOWN:                  # when ramping downwards is active, if it is the first time, the ramping counter is set to start from 0
                if self.__CH1_newState.value == 0 and self.__CH1_oldState.value == 1:
                    self.__CH1_rampCounter.value = 0
                self.rampDownCH1()                                              # the function for ramping up is called

        if channel == self.CH2:                 # Decision tree for ramping CH2, for exact explanation see CH1

            if (self.__CH2_newState.value == 0 and self.__CH2_oldState.value == 0 and not self.__CH2_rampFlag.value == -1) \
                    or (self.__CH2_newState.value == 1 and self.__CH2_oldState.value == 1 and not self.__CH2_rampFlag.value == 1) \
                    or self.__CH2_rampFlag.value == 0:
                self.__CH2_rampFlag.value = self.NO_RAMPING

            if (self.__CH2_newState.value == 1 and self.__CH2_oldState.value == 0) or self.__CH2_rampFlag.value == 1:
                self.__CH2_rampFlag.value = self.RAMPING_UP

            if (self.__CH2_newState.value == 0 and self.__CH2_oldState.value == 1) or self.__CH2_rampFlag.value == -1:
                self.__CH2_rampFlag.value = self.RAMPING_DOWN

            if self.__CH2_rampFlag.value == self.NO_RAMPING:
                if self.__Ch2_active.value == 0:
                    self.__CH2_ramp.value = 0
                else:
                    self.__CH2_ramp.value = 100

            if self.__CH2_rampFlag.value == self.RAMPING_UP:
                if self.__CH2_newState.value == 1 and self.__CH2_oldState.value == 0:
                    self.__CH2_rampCounter.value = 0
                self.rampUpCH2()

            if self.__CH2_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH2_newState.value == 0 and self.__CH2_oldState.value == 1:
                    self.__CH2_rampCounter.value = 0
                self.rampDownCH2()

        if channel == self.CH3:                 # Decision tree for ramping CH3, for exact explanation see CH1

            if (self.__CH3_newState.value == 0 and self.__CH3_oldState.value == 0 and not self.__CH3_rampFlag.value == -1) \
                    or (self.__CH3_newState.value == 1 and self.__CH3_oldState.value == 1 and not self.__CH3_rampFlag.value == 1) \
                    or self.__CH3_rampFlag.value == 0:
                self.__CH3_rampFlag.value = self.NO_RAMPING

            if (self.__CH3_newState.value == 1 and self.__CH3_oldState.value == 0) or self.__CH3_rampFlag.value == 1:
                self.__CH3_rampFlag.value = self.RAMPING_UP

            if (self.__CH3_newState.value == 0 and self.__CH3_oldState.value == 1) or self.__CH3_rampFlag.value == -1:
                self.__CH3_rampFlag.value = self.RAMPING_DOWN

            if self.__CH3_rampFlag.value == self.NO_RAMPING:
                if self.__Ch3_active.value == 0:
                    self.__CH3_ramp.value = 0
                else:
                    self.__CH3_ramp.value = 100

            if self.__CH3_rampFlag.value == self.RAMPING_UP:
                if self.__CH3_newState.value == 1 and self.__CH3_oldState.value == 0:
                    self.__CH3_rampCounter.value = 0
                self.rampUpCH3()

            if self.__CH3_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH3_newState.value == 0 and self.__CH3_oldState.value == 1:
                    self.__CH3_rampCounter.value = 0
                self.rampDownCH3()

        if channel == self.CH4:                 # Decision tree for ramping CH4, for exact explanation see CH1

            if (self.__CH4_newState.value == 0 and self.__CH4_oldState.value == 0 and not self.__CH4_rampFlag.value == -1) \
                    or (self.__CH4_newState.value == 1 and self.__CH4_oldState.value == 1 and not self.__CH4_rampFlag.value == 1) \
                    or self.__CH4_rampFlag.value == 0:
                self.__CH4_rampFlag.value = self.NO_RAMPING

            if (self.__CH4_newState.value == 1 and self.__CH4_oldState.value == 0) or self.__CH4_rampFlag.value == 1:
                self.__CH4_rampFlag.value = self.RAMPING_UP

            if (self.__CH4_newState.value == 0 and self.__CH4_oldState.value == 1) or self.__CH4_rampFlag.value == -1:
                self.__CH4_rampFlag.value = self.RAMPING_DOWN

            if self.__CH4_rampFlag.value == self.NO_RAMPING:
                if self.__Ch4_active.value == 0:
                    self.__CH4_ramp.value = 0
                else:
                    self.__CH4_ramp.value = 100

            if self.__CH4_rampFlag.value == self.RAMPING_UP:
                if self.__CH4_newState.value == 1 and self.__CH4_oldState.value == 0:
                    self.__CH4_rampCounter.value = 0
                self.rampUpCH4()

            if self.__CH4_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH4_newState.value == 0 and self.__CH4_oldState.value == 1:
                    self.__CH4_rampCounter.value = 0
                self.rampDownCH4()

        if channel == self.CH5:                 # Decision tree for ramping CH5, for exact explanation see CH1

            if (self.__CH5_newState.value == 0 and self.__CH5_oldState.value == 0 and not self.__CH5_rampFlag.value == -1) \
                    or (self.__CH5_newState.value == 1 and self.__CH5_oldState.value == 1 and not self.__CH5_rampFlag.value == 1) \
                    or self.__CH5_rampFlag.value == 0:
                self.__CH5_rampFlag.value = self.NO_RAMPING

            if (self.__CH5_newState.value == 1 and self.__CH5_oldState.value == 0) or self.__CH5_rampFlag.value == 1:
                self.__CH5_rampFlag.value = self.RAMPING_UP

            if (self.__CH5_newState.value == 0 and self.__CH5_oldState.value == 1) or self.__CH5_rampFlag.value == -1:
                self.__CH5_rampFlag.value = self.RAMPING_DOWN

            if self.__CH5_rampFlag.value == self.NO_RAMPING:
                if self.__Ch5_active.value == 0:
                    self.__CH5_ramp.value = 0
                else:
                    self.__CH5_ramp.value = 100

            if self.__CH5_rampFlag.value == self.RAMPING_UP:
                if self.__CH5_newState.value == 1 and self.__CH5_oldState.value == 0:
                    self.__CH5_rampCounter.value = 0
                self.rampUpCH5()

            if self.__CH5_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH5_newState.value == 0 and self.__CH5_oldState.value == 1:
                    self.__CH5_rampCounter.value = 0
                self.rampDownCH5()

        if channel == self.CH6:                 # Decision tree for ramping CH6, for exact explanation see CH1

            if (self.__CH6_newState.value == 0 and self.__CH6_oldState.value == 0 and not self.__CH6_rampFlag.value == -1) \
                    or (self.__CH6_newState.value == 1 and self.__CH6_oldState.value == 1 and not self.__CH6_rampFlag.value == 1) \
                    or self.__CH6_rampFlag.value == 0:
                self.__CH6_rampFlag.value = self.NO_RAMPING

            if (self.__CH6_newState.value == 1 and self.__CH6_oldState.value == 0) or self.__CH6_rampFlag.value == 1:
                self.__CH6_rampFlag.value = self.RAMPING_UP

            if (self.__CH6_newState.value == 0 and self.__CH6_oldState.value == 1) or self.__CH6_rampFlag.value == -1:
                self.__CH6_rampFlag.value = self.RAMPING_DOWN

            if self.__CH6_rampFlag.value == self.NO_RAMPING:
                if self.__Ch6_active.value == 0:
                    self.__CH6_ramp.value = 0
                else:
                    self.__CH6_ramp.value = 100

            if self.__CH6_rampFlag.value == self.RAMPING_UP:
                if self.__CH6_newState.value == 1 and self.__CH6_oldState.value == 0:
                    self.__CH6_rampCounter.value = 0
                self.rampUpCH6()

            if self.__CH6_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH6_newState.value == 0 and self.__CH6_oldState.value == 1:
                    self.__CH6_rampCounter.value = 0
                self.rampDownCH6()

        if channel == self.CH7:                 # Decision tree for ramping CH7, for exact explanation see CH1

            if (self.__CH7_newState.value == 0 and self.__CH7_oldState.value == 0 and not self.__CH7_rampFlag.value == -1) \
                    or (self.__CH7_newState.value == 1 and self.__CH7_oldState.value == 1 and not self.__CH7_rampFlag.value == 1) \
                    or self.__CH7_rampFlag.value == 0:
                self.__CH7_rampFlag.value = self.NO_RAMPING

            if (self.__CH7_newState.value == 1 and self.__CH7_oldState.value == 0) or self.__CH7_rampFlag.value == 1:
                self.__CH7_rampFlag.value = self.RAMPING_UP

            if (self.__CH7_newState.value == 0 and self.__CH7_oldState.value == 1) or self.__CH7_rampFlag.value == -1:
                self.__CH7_rampFlag.value = self.RAMPING_DOWN

            if self.__CH7_rampFlag.value == self.NO_RAMPING:
                if self.__Ch7_active.value == 0:
                    self.__CH7_ramp.value = 0
                else:
                    self.__CH7_ramp.value = 100

            if self.__CH7_rampFlag.value == self.RAMPING_UP:
                if self.__CH7_newState.value == 1 and self.__CH7_oldState.value == 0:
                    self.__CH7_rampCounter.value = 0
                self.rampUpCH7()

            if self.__CH7_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH7_newState.value == 0 and self.__CH7_oldState.value == 1:
                    self.__CH7_rampCounter.value = 0
                self.rampDownCH7()

        if channel == self.CH8:                 # Decision tree for ramping CH8, for exact explanation see CH1

            if (self.__CH8_newState.value == 0 and self.__CH8_oldState.value == 0 and not self.__CH8_rampFlag.value == -1) \
                    or (self.__CH8_newState.value == 1 and self.__CH8_oldState.value == 1 and not self.__CH8_rampFlag.value == 1) \
                    or self.__CH8_rampFlag.value == 0:
                self.__CH8_rampFlag.value = self.NO_RAMPING

            if (self.__CH8_newState.value == 1 and self.__CH8_oldState.value == 0) or self.__CH8_rampFlag.value == 1:
                self.__CH8_rampFlag.value = self.RAMPING_UP

            if (self.__CH8_newState.value == 0 and self.__CH8_oldState.value == 1) or self.__CH8_rampFlag.value == -1:
                self.__CH8_rampFlag.value = self.RAMPING_DOWN

            if self.__CH8_rampFlag.value == self.NO_RAMPING:
                if self.__Ch8_active.value == 0:
                    self.__CH8_ramp.value = 0
                else:
                    self.__CH8_ramp.value = 100

            if self.__CH8_rampFlag.value == self.RAMPING_UP:
                if self.__CH8_newState.value == 1 and self.__CH8_oldState.value == 0:
                    self.__CH8_rampCounter.value = 0
                self.rampUpCH8()

            if self.__CH8_rampFlag.value == self.RAMPING_DOWN:
                if self.__CH8_newState.value == 0 and self.__CH8_oldState.value == 1:
                    self.__CH8_rampCounter.value = 0
                self.rampDownCH8()

    # Generates a Pulse-by-Pulse / INIT message with the provided information
    def getMessage(self):

        # Build message
        message = bytearray()

        # Message Header
        message += MM_Message_Builder.MSG_START
        message += b'\x22'
        message += MM_Message_Builder.MSG_TYPE_PULSE_BY_PULSE

        # Pulse Delay
        if self.__Pulse_Delay.value == 0:
            message += MM_Message_Builder.PULSE_DELAY_STD
        elif self.__Pulse_Delay.value == 1:
             message += MM_Message_Builder.PULSE_DELAY_OFF

        # Stimulation Periode
        if self.__BOOST_MODE.value == 1:
            message += self.__T_BOOST.value.to_bytes(1, 'big')
        else:
            message += self.__T.value.to_bytes(1, 'big')

        # Stimulation Intensity
        message += self.__Intensity.value.to_bytes(1, 'big')

        # Algorithm to activate Ramping if wanted, 1 means ramping is on, anything else means ramping is off

        if self.__rampOnorOff.value == 1:

            # saving the current channel values for comparison to see if they were acitvated or deactivated
            self.__CH1_newState.value = self.__Ch1_active.value
            self.__CH2_newState.value = self.__Ch2_active.value
            self.__CH3_newState.value = self.__Ch3_active.value
            self.__CH4_newState.value = self.__Ch4_active.value
            self.__CH5_newState.value = self.__Ch5_active.value
            self.__CH6_newState.value = self.__Ch6_active.value
            self.__CH7_newState.value = self.__Ch7_active.value
            self.__CH8_newState.value = self.__Ch8_active.value

            # activate function to check if ramping is required for every singel channel
            self.toRamp_or_not_to_Ramp(self.CH1)
            self.toRamp_or_not_to_Ramp(self.CH2)
            self.toRamp_or_not_to_Ramp(self.CH3)
            self.toRamp_or_not_to_Ramp(self.CH4)
            self.toRamp_or_not_to_Ramp(self.CH5)
            self.toRamp_or_not_to_Ramp(self.CH6)
            self.toRamp_or_not_to_Ramp(self.CH7)
            self.toRamp_or_not_to_Ramp(self.CH8)

            # calculate the ramping value by multiplying the ramp factor with the maximum amplitude
            rampmessage_CH1 = int(round((self.__A1_max.value * (self.__CH1_ramp.value / 100.0))) * self.__Ch1_active.value)
            rampmessage_CH2 = int(round((self.__A2_max.value * (self.__CH2_ramp.value / 100.0))) * self.__Ch2_active.value)
            rampmessage_CH3 = int(round((self.__A3_max.value * (self.__CH3_ramp.value / 100.0))) * self.__Ch3_active.value)
            rampmessage_CH4 = int(round((self.__A4_max.value * (self.__CH4_ramp.value / 100.0))) * self.__Ch4_active.value)
            rampmessage_CH5 = int(round((self.__A5_max.value * (self.__CH5_ramp.value / 100.0))) * self.__Ch5_active.value)
            rampmessage_CH6 = int(round((self.__A6_max.value * (self.__CH6_ramp.value / 100.0))) * self.__Ch6_active.value)
            rampmessage_CH7 = int(round((self.__A7_max.value * (self.__CH7_ramp.value / 100.0))) * self.__Ch7_active.value)
            rampmessage_CH8 = int(round((self.__A8_max.value * (self.__CH8_ramp.value / 100.0))) * self.__Ch8_active.value)

            # compensation for MOTIMOVE error through comparison with array of compensation values
            rampmessage_CH1 = self.AVAL_COMPENSATION[rampmessage_CH1]
            rampmessage_CH2 = self.AVAL_COMPENSATION[rampmessage_CH2]
            rampmessage_CH3 = self.AVAL_COMPENSATION[rampmessage_CH3]
            rampmessage_CH4 = self.AVAL_COMPENSATION[rampmessage_CH4]
            rampmessage_CH5 = self.AVAL_COMPENSATION[rampmessage_CH5]
            rampmessage_CH6 = self.AVAL_COMPENSATION[rampmessage_CH6]
            rampmessage_CH7 = self.AVAL_COMPENSATION[rampmessage_CH7]
            rampmessage_CH8 = self.AVAL_COMPENSATION[rampmessage_CH8]

            # addition of compensated ramp value to the message
            message += (rampmessage_CH1).to_bytes(1, 'big')
            message += (rampmessage_CH2).to_bytes(1, 'big')
            message += (rampmessage_CH3).to_bytes(1, 'big')
            message += (rampmessage_CH4).to_bytes(1, 'big')
            message += (rampmessage_CH5).to_bytes(1, 'big')
            message += (rampmessage_CH6).to_bytes(1, 'big')
            message += (rampmessage_CH7).to_bytes(1, 'big')
            message += (rampmessage_CH8).to_bytes(1, 'big')

            # printing of channel amplitude just for checking while working on it, can later be removed
            if not rampmessage_CH1 == 0:
                print("Stim CH1: " + str(rampmessage_CH1) + "%")
            if not rampmessage_CH2 == 0:
                print("Stim CH2: " + str(rampmessage_CH2) + "%")
            if not rampmessage_CH3 == 0:
                print("Stim CH3: " + str(rampmessage_CH3) + "%")
            if not rampmessage_CH4 == 0:
                print("Stim CH4: " + str(rampmessage_CH4) + "%")
            if not rampmessage_CH5 == 0:
                print("Stim CH5: " + str(rampmessage_CH5) + "%")
            if not rampmessage_CH6 == 0:
                print("Stim CH6: " + str(rampmessage_CH6) + "%")
            if not rampmessage_CH7 == 0:
                print("Stim CH7: " + str(rampmessage_CH7) + "%")
            if not rampmessage_CH8 == 0:
                print("Stim CH8: " + str(rampmessage_CH8) + "%")

            # while ramping down the channel active value needs to be on longer than normally, but to get an accurate comparison it needs to be reset now
            if self.__CH1_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch1_active.value = 0
            if self.__CH2_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch2_active.value = 0
            if self.__CH3_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch3_active.value = 0
            if self.__CH4_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch4_active.value = 0
            if self.__CH5_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch5_active.value = 0
            if self.__CH6_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch6_active.value = 0
            if self.__CH7_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch7_active.value = 0
            if self.__CH8_rampFlag.value == self.RAMPING_DOWN:
                self.__Ch8_active.value = 0

            # saving of the current channel values for later comparison in next message
            self.__CH1_oldState.value = self.__CH1_newState.value
            self.__CH2_oldState.value = self.__CH2_newState.value
            self.__CH3_oldState.value = self.__CH3_newState.value
            self.__CH4_oldState.value = self.__CH4_newState.value
            self.__CH5_oldState.value = self.__CH5_newState.value
            self.__CH6_oldState.value = self.__CH6_newState.value
            self.__CH7_oldState.value = self.__CH7_newState.value
            self.__CH8_oldState.value = self.__CH8_newState.value

        # if no ramping is required a normal message is built
        else:
            message += (self.__A1_max.value * self.__Ch1_active.value).to_bytes(1, 'big')
            message += (self.__A2_max.value * self.__Ch2_active.value).to_bytes(1, 'big')
            message += (self.__A3_max.value * self.__Ch3_active.value).to_bytes(1, 'big')
            message += (self.__A4_max.value * self.__Ch4_active.value).to_bytes(1, 'big')
            message += (self.__A5_max.value * self.__Ch5_active.value).to_bytes(1, 'big')
            message += (self.__A6_max.value * self.__Ch6_active.value).to_bytes(1, 'big')
            message += (self.__A7_max.value * self.__Ch7_active.value).to_bytes(1, 'big')
            message += (self.__A8_max.value * self.__Ch8_active.value).to_bytes(1, 'big')

        # Phasewidths
        if self.__BOOST_MODE.value == 1:
            message += self.__PhW1_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW2_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW3_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW4_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW5_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW6_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW7_BOOST.value.to_bytes(1, 'big')
            message += self.__PhW8_BOOST.value.to_bytes(1, 'big')
        else:
            message += self.__PhW1.value.to_bytes(1, 'big')
            message += self.__PhW2.value.to_bytes(1, 'big')
            message += self.__PhW3.value.to_bytes(1, 'big')
            message += self.__PhW4.value.to_bytes(1, 'big')
            message += self.__PhW5.value.to_bytes(1, 'big')
            message += self.__PhW6.value.to_bytes(1, 'big')
            message += self.__PhW7.value.to_bytes(1, 'big')
            message += self.__PhW8.value.to_bytes(1, 'big')

        # PreScalers
        message += self.__Ch1_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch2_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch3_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch4_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch5_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch6_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch7_PreScaler.value.to_bytes(1, 'big')
        message += self.__Ch8_PreScaler.value.to_bytes(1, 'big')

        # Doublets
        message += self.__Doublet_Flag.value.to_bytes(1, 'big')

        # Doublet ISI
        message += self.__Doublet_ISI.value.to_bytes(1, 'big')

        # Sensor Input
        if self.__Sensor_Input.value == 0:
            message += MM_Message_Builder.SENSOR_AI
        elif self.__Sensor_Input.value == 1:
            message += MM_Message_Builder.SENSOR_S1
        else:
            message += MM_Message_Builder.SENSOR_S2

        # High Voltage
        message += self.__High_Voltage.value.to_bytes(1, 'big')

        # Checksum
        message = self.__addCheckSum(message)

        return message


    # Returns a Start Train Message
    # Please set the stimulation Parameter first through a Message generated by getMessage()
    # Also ensure that High Voltage is active
    def getStartTrainMessage(self):
        return MM_Message_Builder.__MSG_START_TRAIN

    # Returns a Stop Train Message
    def getStopTrainMessage(self):
        return MM_Message_Builder.__MSG_STOP_TRAIN


    # Calculates and appends the Checksum
    def __addCheckSum(self, message):

        # Checksum
        chksum = 0
        for i in range(1, len(message)):  # from 2. byte
            # print(str(i) + ': ' + str(message[i]))
            chksum += message[i]
            chksum = chksum & 0x7F

        message += chksum.to_bytes(1, 'big')

        return message

    #experimental start and stop train blocks
    # def getStartMessage(self):
    #
    #     # Build message
    #     message = bytearray()
    #
    #     # Message Header
    #     message += MM_Message_Builder.MSG_START
    #     message += b'\x22'
    #     message += MM_Message_Builder.MSG_TYPE_PULSE_TRAIN_START
    #
    #     message += self.Pulse_Delay
    #
    #     # Stimulation Periode
    #     message += self.__T.to_bytes(1, 'big')
    #
    #     # Stimulation Intensity
    #     message += self.__Intensity.to_bytes(1, 'big')
    #
    #     # Stimulation Amplitudes
    #     for i in range(0,8):
    #         if self.__activeChannels[i]:
    #             message += self.__A[i].to_bytes(1, 'big')
    #         else:
    #             message += b'\x00'
    #
    #     # Phasewidths
    #     for i in range(0, 8):
    #         message += self.__PhW[i].to_bytes(1, 'big')
    #
    #     # PreScalers
    #     for i in range(0, 8):
    #         message += self.PreScaler[i].to_bytes(1, 'big')
    #
    #     # Doublets
    #     message += self.__Doublet_Flag
    #
    #     # Doublet ISI
    #     message += self.Doublet_ISI.to_bytes(1, 'big')
    #
    #     # Sensor Input
    #     message += self.Sensor_Input
    #     message += self.__High_Voltage
    #
    #     message = self.__addCheckSum(message)
    #
    #     return message
    #
    # def getStopMessage(self):
    #
    #     # Build message
    #     message = bytearray()
    #
    #     # Message Header
    #     message += MM_Message_Builder.MSG_START
    #     message += b'\x22'
    #     message += MM_Message_Builder.MSG_TYPE_PULSE_TRAIN_STOP
    #
    #     message += self.Pulse_Delay
    #
    #     # Stimulation Periode
    #     message += self.__T.to_bytes(1, 'big')
    #
    #     # Stimulation Intensity
    #     message += self.__Intensity.to_bytes(1, 'big')
    #
    #     # Stimulation Amplitudes
    #     for i in range(0,8):
    #         if self.__activeChannels[i]:
    #             message += self.__A[i].to_bytes(1, 'big')
    #         else:
    #             message += b'\x00'
    #
    #     # Phasewidths
    #     for i in range(0, 8):
    #         message += self.__PhW[i].to_bytes(1, 'big')
    #
    #     # PreScalers
    #     for i in range(0, 8):
    #         message += self.PreScaler[i].to_bytes(1, 'big')
    #
    #     # Doublets
    #     message += self.__Doublet_Flag.value.to_bytes(1, 'big')
    #
    #     # Doublet ISI
    #     message += self.__Doublet_ISI.value.to_bytes(1, 'big')
    #
    #     # Sensor Input
    #     if self.__Sensor_Input.value == 0:
    #         message += MM_Message_Builder.SENSOR_AI
    #     elif self.__Sensor_Input.value == 1:
    #         message += MM_Message_Builder.SENSOR_S1
    #     else:
    #         message += MM_Message_Builder.SENSOR_S2
    #
    #     message += self.__High_Voltage.value.to_bytes(1, 'big')
    #
    #     message = self.__addCheckSum(message)
    #
    #     return message