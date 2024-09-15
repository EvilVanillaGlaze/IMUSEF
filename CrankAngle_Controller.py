from typing import List

from Controllers.CrankAngle_Controller_Data import CrankAngle_Controller_Data
import time
import copy

class CrankAngle_Controller(object):


    # Constants
    CRANKANGLE_SENSOR = "CRANKANGLE_SENSOR_OPENDAQ"
    CRANKANGLE_SENSOR_ROTOR = "CRANKANGLE_SENSOR_ROTOR"

    DELAY_ROTOR = 200   # in [ms]
    DELAY_OPENDAQ = 0   # in [ms]

    def __init__(self):

        # Input Parameters
        self.__CrankAngle = 0.0

        # Output Parameters
        self.__data = CrankAngle_Controller_Data()

        # Settings
        # CRANK ANGLE PATTERN USED BY J.P. DURING THE CYBATHLON 2016
        #self.__LQ_Start_Angle = 22
        #self.__LQ_Stop_Angle = 156
        #self.__RQ_Start_Angle = 202
        #self.__RQ_Stop_Angle = 337

        self.Min = 0
        self.Max = 360

        self.Start = [22, 202, 0, 0, 0, 0, 0, 0]
        self.Stop = [156, 337, 0, 0, 0, 0, 0, 0]

        self.compensate4Delays = True
        self.__Start_compensated = [0, 0, 0, 0, 0, 0, 0, 0]
        self.__Stop_compensated = [0, 0, 0, 0, 0, 0, 0, 0]

        self.MUSCLE_DELAY_ON = 200  # [ms]
        self.MUSCLE_DELAY_OFF = 200  # [ms]
        self.SENSOR_DELAY = 0  # [ms] - OpenDAQ (0); ROTOR: 200 ms
        self.SAMPLE_PERIODE = 10  # [ms] - expected time between 2 consecutive Datasamples


        # Limits
        self.CADENCE_LIMIT_MIN = 10     # [RPM]    # Minimal Cadence allowed in [RPM]
        self.CADENCE_LIMIT_MAX = 70     # [RPM]    # Maximal Cadence allowed in [RPM]

        # Cadence
        self.Cadence = 0
        self.__LastCycle_TimeStamp = 0
        self.RTCadence = 0
        self.RTCadenceSmooth = 0
        self.rt = 0
        self.sum_CrankAngle = 0
        self.previous = 0
        self.LastCadences = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.c = 0
        self.rtcheck = []

        self.activeChannels = [False, False, False, False, False, False, False, False]



    # Processes new Data and adjusts the controller
    def update(self, CrankAngle, Sensor):

        # Check Boundaries
        if CrankAngle < self.Min:
            CrankAngle = self.Min

        if CrankAngle > self.Max:
            CrankAngle = self.Max

        # Calculate Cadence
        now = time.time()
        delta_CrankAngle = CrankAngle - self.__CrankAngle
        if delta_CrankAngle <= -100:
            self.sum_CrankAngle = self.sum_CrankAngle + delta_CrankAngle + 360
        else:
            self.sum_CrankAngle = self.sum_CrankAngle + delta_CrankAngle
        if self.sum_CrankAngle > 45:
            self.RTCadence = (self.sum_CrankAngle / 360) / ((now - self.previous) / 60)
            self.sum_CrankAngle = 0
            self.previous = now

        # d = delta_CrankAngle
        # if d != 0:
        #     if d <= -100:
        #         d = d + 360
        #
        # self.RTCadence = (d/360)/((now - self.previous)/60)
        # self.previous = now
        #self.rtcheck.append(self.rt)

        # New Cycle
        if delta_CrankAngle <= -100:
            delta_t = now - self.__LastCycle_TimeStamp
            self.Cadence = 60.0 / delta_t
            self.__LastCycle_TimeStamp = now

        # Check for decreasing Cadence
        if self.Cadence > 0:
            diff = now - self.__LastCycle_TimeStamp
            expected_diff = 60.0 / self.Cadence

            if diff > expected_diff:
                self.Cadence = 60.0 / diff

        if self.Cadence < self.CADENCE_LIMIT_MIN:
            self.Cadence = 0

        if self.RTCadence > 0:
            rtdiff = now - self.previous
            rtexpected_diff = 7.5 / self.RTCadence

            if rtdiff > (rtexpected_diff+0.05):
                self.RTCadence = 7.5 / (rtdiff-0.05)



        if self.RTCadence < self.CADENCE_LIMIT_MIN:
            self.RTCadence = 0
            #self.rt = 0

        self.LastCadences[self.c] = self.RTCadence
        self.RTCadenceSmooth = sum(self.LastCadences) / len(self.LastCadences)
        self.c = self.c + 1
        if self.c >= len(self.LastCadences):
            self.c = 0


        # Update CrankAngle
        self.__CrankAngle = CrankAngle


        # Select correct Delays
        if Sensor == self.CRANKANGLE_SENSOR_ROTOR:
            self.SENSOR_DELAY = self.DELAY_ROTOR
        elif Sensor == self.CRANKANGLE_SENSOR:
            self.SENSOR_DELAY = self.DELAY_OPENDAQ
        else:
            self.SENSOR_DELAY = 0

        # Compensate delays
        if self.compensate4Delays:
            self.__compensateDelays()

        # Process the new Data
        self.__process()


    # Updates the stimulation channels
    def __process(self):


        if self.__CrankAngle < 0 or \
            self.Cadence < self.CADENCE_LIMIT_MIN or \
            self.Cadence > self.CADENCE_LIMIT_MAX:
            self.activeChannels = [False, False, False, False, False, False, False, False]
            return

        start = self.Start
        stop = self.Stop

        for ch in range(8):

            start = self.Start[ch]
            stop = self.Stop[ch]

            # Use delay compensation
            if self.compensate4Delays:
                start = self.__Start_compensated[ch]
                stop = self.__Stop_compensated[ch]

            result = False

            if start == stop:
                result = False

            elif start < stop:
                if start <= self.__CrankAngle and self.__CrankAngle <= stop:
                    result = True
                else:
                    result = False

            elif start > stop:
                if not (stop < self.__CrankAngle and self.__CrankAngle < start):
                    result = True
                else:
                    result = False

            self.activeChannels[ch] = result


    # Calculates compensated Start and Stop times for each channel in order to compensate for
    # Muscle Delay and an eventual Sensor Delay
    def __compensateDelays(self):

        if self.Cadence <= self.CADENCE_LIMIT_MIN:
            return

        # Make deep-copy to avoid overwritting of referenced values
        t_Start = copy.deepcopy(self.Start)
        t_Stop = copy.deepcopy(self.Stop)

        periode = (60 / self.Cadence) * 1000     # Duration of a full turn in [ms]

        # Overall delay
        delay_ON = self.MUSCLE_DELAY_ON + self.SENSOR_DELAY
        delay_OFF = self.MUSCLE_DELAY_OFF + self.SENSOR_DELAY

        for ch in range(8):

            # Only compensate if channel is active... i.e. Start and Stop are different
            # Only allow a shift which is half the periode time - a higher shift might be strange...
            if self.Start[ch] != self.Stop[ch]: # and delay_ON <= (periode / 2):

                # Compensate StartValues by smoothing-delay and muscle-delay
                self.__Start_compensated[ch] = self.__calcShiftedValue(t_Start[ch], periode, delay_ON)

                # Compensate StopValues only by smoothing-delay
                self.__Stop_compensated[ch] = self.__calcShiftedValue(t_Stop[ch], periode, delay_OFF)

            else:
                self.__Start_compensated[ch] = t_Start[ch]
                self.__Stop_compensated[ch] = t_Stop[ch]


    # Returns the shifted value based on a the used calculation method
    def __calcShiftedValue(self, value_unshifted, periode_ms, shift_ms):


        shift = round((shift_ms / periode_ms) * 360.0)
        value_shifted = value_unshifted - shift

        # Shift into previous cycle
        if value_shifted < 0:
            value_shifted = 360 + value_shifted

        return value_shifted


    # Returns a CrankAngle_Controller_Data object containing the current state of the Controller
    def getData(self):

        self.__data.activeChannels = self.activeChannels[:]
        self.__data.Cadence = self.Cadence
        self.__data.RTCadence = self.RTCadence
        self.__data.RTCadenceSmooth = self.RTCadenceSmooth

        return self.__data

    # Returns the active Channels of the Controller
    def getActiveChannels(self):
        return self.activeChannels

    # Returns the current configuration
    def getConfig(self):
        data = {
            "Name": "CONTROLLER_CRANKANGLE",
            "Min": self.Min,
            "Max": self.Max,
            "Start": self.Start,
            "Stop": self.Stop
        }

        return data

    # Sets a new Configuration
    def setConfig(self, _config):

        self.Name = _config["Name"];
        self.Min = _config["Min"];
        self.Max = _config["Max"];
        self.Start = _config["Start"];
        self.Stop = _config["Stop"];