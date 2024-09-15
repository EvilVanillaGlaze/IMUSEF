# -*- coding: utf-8 -*-
"""
IMUSEF - Stimulation Plattform

@author: Martin Schmoll, Ronan Le Guillou and Benoît SIJOBERT (CAMIN TEAM - INRIA)

"""
from openANT.myOpenANT_Manager import myOpenAnt_Manager
from openDAQ.myDAQManager import myDAQManager
from Stimulators.Motimove.MM_Manager import MM_Manager
from HomeTrainer.Hometrainer import *
from myGPIO.GPIO_Manager import GPIO_Manager
from UDP.UDP_Server import UDP_Server
from TCP.TCP_Server import TCP_Server
from TCP.TCP_Message import TCP_Message
from Controllers.Manual_Controller import Manual_Controller
from Controllers.Gonio_ThighAngle_Controller import Gonio_ThighAngle_Controller
from Controllers.Gonio_KneeAngle_Controller import Gonio_KneeAngle_Controller
from Controllers.CrankAngle_Controller import CrankAngle_Controller
from Controllers.TCP_Controller import TCP_Controller
from Controllers.Biodex_Controller import Biodex_Controller
from Controllers.PID_Controller import PID_Controller
from IMU.imu_wired.IMU_Manager import IMU_Manager
from DataLogger.DataLogger import DataLogger
from IMUSEF_Data import IMUSEF_Data
from Controllers.AutoTune_Controller import AutoTune_Controller


import json
import numpy as np
from CyclingComputer.CyclingComputer import CyclingComputer
import traceback
import sys

# Constants
CONTROLLER_NONE = "CONTROLLER_NONE"                 # No Controller is selected to controll the stimulator
CONTROLLER_TCP = "CONTROLLER_TCP"                   # Stimulator is controlled by the TCP-Client (testing and configuration)
CONTROLLER_CRANKANGLE = "CONTROLLER_CRANKANGLE"     # Stimulator is controlled my the CrankAngle Controller
CONTROLLER_THIGHANGLE = "CONTROLLER_THIGHANGLE"     # Stimulator is controlled my the ThighAngle Controller
CONTROLLER_KNEEANGLE = "CONTROLLER_KNEEANGLE"       # Stimulator is controlled my the KneeAngle Controller
CONTROLLER_OBSERVER = "CONTROLLER_OBSERVER"         # Stimulator is controlled my the Observer Controller
CONTROLLER_BIODEX = "CONTROLLER_BIODEX"             # Stimulator is controlled my the Biodex Controller
CONTROLLER_MANUAL = "CONTROLLER_MANUAL"             # Stimulator is controlled my the AutoTune Controller
CONTROLLER_AUTOTUNE = "CONTROLLER_AUTOTUNE"         # Stimulator is controlled my the AutoTune Controller



class IMUSEF(object):

    # Initialisation
    def __init__(self, Start_Mode):

        ## Settings

        # Activity flags - Basic Modules
        self.FLAG_LOG_DATA = False
        self.FLAG_UDP_SERVER = True
        self.FLAG_TCP_SERVER = True

        # Activity flags - Other Modules
        self.FLAG_STIMULATOR = False
        self.FLAG_CRANKANGLE_SENSOR = False
        self.FLAG_CRANKANGLE_SENSOR_IMU_FOX = False
        self.FLAG_IMUs = False
        self.FLAG_HEARTRATE_MONITOR = False
        self.FLAG_POWERMETER_ROTOR = False
        self.FLAG_HOMETRAINER = False
        self.FLAG_BIODEX = False

        # PID FLAG
        self.FLAG_PID = False
        #self.FLAG_PID = True
        self.PID_Mode = 0
        self.FLAG_ST_TEST = False
        self.FLAG_PID_TEST = False
        self.CV_Amp_Min = 20
        self.CV_Amp_Max = 60
        self.CV_Ph_Min = 150
        self.CV_Ph_Max = 800

        # Activate Controls
        self.FLAG_Emergency_Button = False
        self.FLAG_Left_Button = False
        self.FLAG_Right_Button = False
        self.FLAG_Boost_Button = False
        self.FLAG_Man_Auto_Switch = False

        # Indication of how IMUSEF got started
        # 0 ... started manually
        # 1 ... started via watchdog
        # 2 ... started via service
        self.Start_Mode = int(Start_Mode)

        # Watchdog Parameters
        self.Keepalive_timer = time.time()
        self.Keepalive_delay = 0.25

        if(self.FLAG_BIODEX):
            self.FLAG_HOMETRAINER = False

        # Channels activated turing stimulation
        self.__UsedChannels = [True, True, True, True, True, True, True, True]


        # Define Simulation
        self.FLAG_SIMULATE_CRANKANGLE = False
        self.FLAG_SIMULATE_KNEEANGLES = False
        self.FLAG_SIMULATE_THIGHANGLES = False
        self.FLAG_SIMULATE_FROM_FILE = False

        # Define CrankAngle Sensor
        self.CrankAngle_Sensor = CrankAngle_Controller.CRANKANGLE_SENSOR

        # Simulation Settings
        self.__simulatedCadence = 50.0
        self.__sim_CrankAngleSpeed = (self.__simulatedCadence / 60.0) * 360.0  # Simulated CrankAngle Speed
        self.__sim_w = 2 * np.pi * (self.__simulatedCadence / 60.0)            # Simulated angular Velocity w
        self.__Min_Sim_KneeAngle = -120.0
        self.__Max_Sim_KneeAngle = -10.0
        self.__Min_Sim_ThighAngle = -60.0
        self.__Max_Sim_ThighAngle = -10.0

        # Define Port Numbers
        self.__UDP_PortNumber = 12345
        self.__UDP_Counter = 0
        self.__UDP_Divider = 1              # Put 0 to send all data
        self.__TCP_PortNumber = 12346


        # Event to exit the IMUSEF
        self.exit = threading.Event()
        self.stop_Datalogging = threading.Event()

        # Data Container
        self.data = IMUSEF_Data()

        # Define Modules
        self.myDataLogger = None
        self.myGPIO_Manager = GPIO_Manager(self.exit)
        self.myUDP_Server = UDP_Server(self.__UDP_PortNumber)
        self.myTCP_Server = TCP_Server(self.__TCP_PortNumber)
        self.myMM_Manager = MM_Manager()
        self.myIMU_Manager = IMU_Manager()
        self.myHometrainer = HOMETRAINER()
        self.myCyclingComputer = CyclingComputer()
        self.myDAQManager = myDAQManager()
        self.myANT_Manager = myOpenAnt_Manager()
        self.myAutoTuneController = AutoTune_Controller()

        # Device Addresses
        self.__MAC_Stimulator = ""
        self.__ID_Rotor_Powermeter = 0
        self.__ID_HeartRateMonitor = 0

        # Stimulator Variables
        self.Stimulator_TimeStamp_LastStatusRequest = 0.0   # TimeStamp of last Status Request
        self.Stimulator_StatusRequestInterval = 1.0         #[s] Seconds between Status requests

        # Limitations
        self.__Cadence_Manual_Control_Max = 20  # [RPM] -> Maximal Cadence for manual Control
        self.__Cadence_Auto_Control_Min = 10    # [RPM] -> Minimal Cadence allowed in automatic Mode
        self.__Cadence_Auto_Control_Max = 70    # [RPM] -> Maximal Cadence allowed in automatic Mode
        self.__MuscleDelay_ON = 200                # [ms] -> Delay between onset of stimulation and full force of muscle
        self.__FLAG_Compensate4Delays = True    # Flag wheter to compensate for temporal delays (Sensor, Smoothing, Muscle)

        # Define Control-Mode
        self.CONTROL_MODE = CONTROLLER_CRANKANGLE
        self.LAST_CONTROL_MODE = CONTROLLER_CRANKANGLE

        # Generate Controllers
        self.myManualController = Manual_Controller()
        self.myBiodexController = Biodex_Controller()
        self.myRemoteController = TCP_Controller()
        self.myCrankAngleController = CrankAngle_Controller()
        self.myThighAngleController = Gonio_ThighAngle_Controller()
        self.myKneeAngleController = Gonio_KneeAngle_Controller()
        #self.myObserverController = Observer_Controller()
        self.myPIDController = PID_Controller()

        # Define Config-File
        self.__ConfigFile = "Settings/ConfigFile.imusef"

        # Load Configuration from File
        self.__loadConfig()


        ## Activate Modules
        # Activate UDP-Server
        if self.FLAG_UDP_SERVER:
            self.myUDP_Server.start(self.exit)

        # Activate TCP-Server
        if self.FLAG_TCP_SERVER:
            self.myTCP_Server.start(self.exit)


        # Start IMU_Manager
        if self.FLAG_IMUs:
            self.myIMU_Manager.start()

        # Start stimulator
        if self.FLAG_STIMULATOR:
            self.myMM_Manager.start()
            self.myDAQManager.start()

        if self.FLAG_HOMETRAINER:
            self.myHometrainer.start()




        if self.FLAG_BIODEX:
            self.myDAQManager.start()

        # if self.FLAG_PID:
        #     self.myPIDController.start(self.exit)

        if self.FLAG_POWERMETER_ROTOR or self.FLAG_HEARTRATE_MONITOR:
            self.myANT_Manager.start(self.exit, self.FLAG_POWERMETER_ROTOR, self.__ID_Rotor_Powermeter, self.FLAG_HEARTRATE_MONITOR, self.__ID_HeartRateMonitor)

        ## Init complete
        print("IMUSEF: Initialization Completed!")
        self.myGPIO_Manager.DoubleBeep()
        self.myGPIO_Manager.setSystemLED(True)

    # Main part of the program - Blocking!
    # This loop should run the entire time and even be restarted in case of error
    def run(self):

        try:


            #################
            # - Main Loop - #
            #################

            while True:

                # New Cycle
                now = time.time()
                self.data.LoopTime = now - self.data.timestamp
                self.data.timestamp = now

                ## Anything new from the TCP_Client?
                if self.myTCP_Server.hasData():
                    # msg = TCP_Message()
                    # msg.parseFromString(self.myTCP_Server.getMessage())

                    msg = self.myTCP_Server.getMessage()
                    self.__handleMessage(msg)


                ## Read PID
                if self.FLAG_PID:
                    self.data.Status_PID = 1
                else:
                    self.data.Status_PID = -1


                ## Read Buttons
                if self.FLAG_Left_Button:
                    self.data.Button_LEFT = self.myGPIO_Manager.getButton_LEFT()
                else:
                    self.data.Button_LEFT = 0

                if self.FLAG_Right_Button:
                    self.data.Button_RIGHT = self.myGPIO_Manager.getButton_RIGHT()
                else:
                    self.data.Button_RIGHT = 0

                if self.FLAG_Boost_Button:
                    self.data.Button_BOOST = self.myGPIO_Manager.getButton_BOOST()
                else:
                    self.data.Button_BOOST = 0

                if self.FLAG_Man_Auto_Switch:
                    self.data.Switch_MAN_AUTO = self.myGPIO_Manager.getSwitch_MAN_AUTO()
                else:
                    self.data.Switch_MAN_AUTO = 1

                if self.FLAG_Emergency_Button:
                    self.data.Button_Emergency = self.myGPIO_Manager.getButton_Emergency()
                else:
                    self.data.Button_Emergency = 0



                # Update Flags
                self.data.FLAG_Emergency_Button = self.FLAG_Emergency_Button
                self.data.FLAG_Left_Button = self.FLAG_Left_Button
                self.data.FLAG_Right_Button = self.FLAG_Right_Button
                self.data.FLAG_Boost_Button = self.FLAG_Boost_Button
                self.data.FLAG_Man_Auto_Switch = self.FLAG_Man_Auto_Switch


                ## Read Data from Modules

                #self.data.StimIntensity = self.myMM_Manager.getStimulationIntensity()
                #self.data.DebugValue = self.myMM_Manager.getStimulationIntensity()
                self.data.DesiredCadence = self.myDAQManager.getDesiredCadence()
                #self.myPIDController.Input.update(self.data.CyclingComputer_DATA.CrankAngle)


                ## Check PID Flags to determine which mode is active and transfer parameters

                if self.FLAG_PID:

                    # Step test to cycle through predetermined stimulation parameters
                    if self.FLAG_ST_TEST:
                        self.myPIDController.ST_TEST(self.PID_Mode)
                        self.data.StimIntensity = self.myPIDController.getST()[0]
                        Fr = round(self.myPIDController.getST()[1])
                        self.myMM_Manager.setStimFrequency(Fr)
                        self.data.StimFrequency = Fr
                        Ph = round(self.myPIDController.getST()[2])
                        self.myMM_Manager.setStimPhasewidth([Ph, Ph, Ph, Ph, Ph, Ph, Ph, Ph])
                        self.data.StimPhasewidth = Ph
                        if self.myPIDController.getST()[3]==0:
                            self.FLAG_ST_TEST = False


                    # PID test to cycle through predetermined desired cadences
                    else:
                        if self.FLAG_PID_TEST == True:
                            self.myPIDController.PID_TEST_Cadence()
                            self.myPIDController.updateDCadence(self.myPIDController.getTest())
                            self.data.DesiredCadence = self.myPIDController.getTest()
                            if self.myPIDController.getTest() == 5:
                                self.FLAG_PID_TEST = False
                        else:
                            self.myPIDController.updateDCadence(self.data.DesiredCadence)

                        #self.myPIDController.updateCCadence(self.myCrankAngleController.getData().Cadence)
                        self.myPIDController.updateCCadence(self.myPIDController.Input.getData().RTCadence)
                        self.myPIDController.PID(self.PID_Mode)

                        if self.data.Switch_MAN_AUTO == 0:
                            self.data.StimIntensity = 50
                            self.myMM_Manager.setStimFrequency(40)
                            self.myMM_Manager.setStimPhasewidth([400, 400, 400, 400, 400, 400, 400, 400])
                        else:
                            self.myPIDController.equalize(self.myCrankAngleController.activeChannels, self.data.PowerMeter_DATA.Power_Left, self.data.PowerMeter_DATA.Power_Right)
                            if self.PID_Mode == 0: #Amplitude Mode
                                self.data.StimIntensity = self.myPIDController.getResult()
                                self.myMM_Manager.setStimFrequency(40)
                                Ph = round(self.myPIDController.getOtherVariable())
                                self.myMM_Manager.setStimPhasewidth([Ph, Ph, Ph, Ph, Ph, Ph, Ph, Ph])
                                #elf.myMM_Manager.setStimPhasewidth([400, 400, 400, 400, 400, 400, 400, 400])

                            elif self.PID_Mode == 1: #Frequency Mode
                                Fr= round(self.myPIDController.getResult())
                                #Fr = 10
                                self.myMM_Manager.setStimFrequency(Fr)
                                self.data.StimFrequency = Fr
                                if Fr > 0:
                                    self.data.StimIntensity = 50
                                else:
                                    self.data.StimIntensity = 0
                                self.myMM_Manager.setStimPhasewidth([400, 400, 400, 400, 400, 400, 400, 400])

                            else:                   #Phasewidth Mode
                                Ph = round(self.myPIDController.getResult())
                                #Ph = 200
                                self.myMM_Manager.setStimPhasewidth([Ph, Ph, Ph, Ph, Ph, Ph, Ph, Ph])
                                self.data.StimPhasewidth = Ph
                                if Ph > 0:
                                    #self.data.StimIntensity = 80
                                    self.data.StimIntensity = round(self.myPIDController.getOtherVariable())
                                else:
                                    self.data.StimIntensity = 0
                                self.myMM_Manager.setStimFrequency(40)



                else:
                    self.data.StimIntensity = self.myDAQManager.getIntensity()
                    self.myPIDController.resetPID()



                self.data.DebugValue = self.myPIDController.equalizer
                #self.data.DebugValue = self.data.StimIntensity
                #self.data.DebugValue = self.data.PowerMeter_DATA.Cadence_Insta

                self.data.IMU_DATA_wired = self.myIMU_Manager.getIMU_Data_wired()

                self.data.Hometrainer_DATA = self.myHometrainer.getData()

                self.data.HeartRate = self.myANT_Manager.getHeartRate()

                self.data.PowerMeter_DATA = self.myANT_Manager.getData()

                # Determine CrankAngle from selected Sensor
                if self.CrankAngle_Sensor == CrankAngle_Controller.CRANKANGLE_SENSOR:
                    self.data.CyclingComputer_DATA.CrankAngle = self.myGPIO_Manager.getEncoderValue()
                elif self.CrankAngle_Sensor == CrankAngle_Controller.CRANKANGLE_SENSOR_ROTOR:
                    self.data.CyclingComputer_DATA.CrankAngle = self.data.PowerMeter_DATA.CrankAngle
                else:
                    self.data.CyclingComputer_DATA.CrankAngle = -1

                ## Simulate desired values
                if self.FLAG_SIMULATE_CRANKANGLE:
                    self.data.CyclingComputer_DATA.CrankAngle = (self.data.timestamp * self.__sim_CrankAngleSpeed) % 360;

                if self.FLAG_SIMULATE_KNEEANGLES:
                    A = (self.__Max_Sim_KneeAngle - self.__Min_Sim_KneeAngle) / 2
                    Base = self.__Min_Sim_KneeAngle + A
                    self.data.IMU_DATA_wired.leftKneeAngle = Base + A * np.sin(self.__sim_w * self.data.timestamp)
                    self.data.IMU_DATA_wired.rightKneeAngle = Base + A * np.sin(self.__sim_w * self.data.timestamp + np.pi)

                if self.FLAG_SIMULATE_THIGHANGLES:
                    A = (self.__Max_Sim_ThighAngle - self.__Min_Sim_ThighAngle) /2
                    Base = self.__Min_Sim_ThighAngle + A
                    self.data.IMU_DATA_wired.leftThighAngle = Base + A * np.sin(self.__sim_w * self.data.timestamp)
                    self.data.IMU_DATA_wired.rightThighAngle = Base + A * np.sin(self.__sim_w * self.data.timestamp + np.pi)

                ## Update CyclingComputer
                self.myCyclingComputer.update( self.data.timestamp,
                                               self.data.CyclingComputer_DATA.CrankAngle,
                                               self.data.Hometrainer_DATA.Power,
                                               self.myGPIO_Manager.getSpeedTimeStamp())


                self.data.CyclingComputer_DATA = self.myCyclingComputer.getData()

                ## Update Controllers
                self.myRemoteController.update()

                # TODO: Activate if used: Deactivated 17.03.2020 - Morten
                # self.myBiodexController.update(self.myDAQManager.getPosition(), self.myDAQManager.getSpeed())

                self.myManualController.update(self.data.Button_LEFT, self.data.Button_RIGHT)

                self.myThighAngleController.update(self.data.timestamp, \
                                                   self.data.IMU_DATA_wired.leftThighAngle, \
                                                   self.data.IMU_DATA_wired.rightThighAngle)

                # TODO: Activate if used: Deactivated 17.03.2020 - Morten
                # self.myKneeAngleController.update(self.data.IMU_DATA_wired.leftKneeAngle, \
                #                                    self.data.IMU_DATA_wired.rightKneeAngle)

                self.myCrankAngleController.update(self.data.CyclingComputer_DATA.CrankAngle, self.CrankAngle_Sensor)

                ##  AutoTune Controller Update
                self.updateAutoTuneController()
                if self.CONTROL_MODE == CONTROLLER_AUTOTUNE:
                    self.myThighAngleController.Activate_Adaptive_Smoothing = False
                    self.myThighAngleController.CADENCE_LIMIT_MIN = 1
                else:
                    self.myThighAngleController.Activate_Adaptive_Smoothing = True
                    self.myThighAngleController.CADENCE_LIMIT_MIN = self.__Cadence_Auto_Control_Min


                ## Read Data from controllers
                self.data.Remote_Controller_DATA = self.myRemoteController.getData()
                self.data.Manual_Controller_DATA = self.myManualController.getData()
                self.data.ThighAngle_Controller_DATA = self.myThighAngleController.getData()
                self.data.KneeAngle_Controller_DATA = self.myKneeAngleController.getData()
                self.data.CrankAngle_Controller_DATA = self.myCrankAngleController.getData()
                self.data.Biodex_Controller_DATA = self.myBiodexController.getData()
                self.data.PID_Controller_DATA = self.myPIDController.Input.getData()


                # Determine Current Controller
                if self.data.Switch_MAN_AUTO == 0:

                    # BackUp automatic control Mode
                    if self.CONTROL_MODE != CONTROLLER_MANUAL:
                        self.LAST_CONTROL_MODE = self.CONTROL_MODE

                    self.CONTROL_MODE = CONTROLLER_MANUAL
                elif self.data.Switch_MAN_AUTO == 1:

                    # Restore Backup
                    if self.CONTROL_MODE == CONTROLLER_MANUAL:
                        self.CONTROL_MODE = self.LAST_CONTROL_MODE


                # Cadence Information
                if self.CONTROL_MODE == CONTROLLER_THIGHANGLE:
                    self.data.CyclingComputer_DATA.Cadence = self.data.ThighAngle_Controller_DATA.Cadence_L

                ## Update Stimulator
                # Update BOOST
                if self.FLAG_PID != True:
                    self.myMM_Manager.setBOOST(self.data.Button_BOOST)

                self.myMM_Manager.setIntensity(self.data.StimIntensity)
                self.myMM_Manager.setEmergencyStop(self.data.Button_Emergency)

                ## Update Channels
                if self.CONTROL_MODE == CONTROLLER_TCP:
                    self.data.activeController = 'R'
                    self.myMM_Manager.setActiveChannels(self.myRemoteController.getActiveChannels())

                elif self.CONTROL_MODE == CONTROLLER_BIODEX:
                    self.data.activeController = 'B'
                    self.myMM_Manager.setActiveChannels(self.myBiodexController.getActiveChannels())

                elif self.CONTROL_MODE == CONTROLLER_MANUAL:
                    self.data.activeController = 'M'
                    self.myMM_Manager.setActiveChannels(self.myManualController.getActiveChannels())

                elif self.CONTROL_MODE == CONTROLLER_CRANKANGLE:
                    self.data.activeController = 'C'
                    self.myMM_Manager.setActiveChannels(self.myCrankAngleController.getActiveChannels())

                elif self.CONTROL_MODE == CONTROLLER_KNEEANGLE:
                    self.data.activeController = 'K'
                    self.myMM_Manager.setActiveChannels(self.myKneeAngleController.getActiveChannels())

                elif self.CONTROL_MODE == CONTROLLER_THIGHANGLE:
                    self.data.activeController = 'T'
                    self.myMM_Manager.setActiveChannels(self.myThighAngleController.getActiveChannels())

                elif self.CONTROL_MODE == CONTROLLER_AUTOTUNE:
                    self.data.activeController = 'A'
                    self.myMM_Manager.setIntensity(self.myAutoTuneController.get_Intensity())
                    self.myMM_Manager.setActiveChannels(self.myAutoTuneController.get_ActiveChannels())

                ## Write data to file
                if self.FLAG_LOG_DATA:
                    if not self.myDataLogger == None:
                        self.myDataLogger.logData(self.data)
                        self.data.comment = ""     # Reset comment to only be written once

                ## Update system State
                self.updateSystemStatus()

                ## Send data to UDP-Client:
                if self.FLAG_UDP_SERVER:
                    if self.__UDP_Counter >= self.__UDP_Divider:
                        self.myUDP_Server.sendData(self.data.getJSON())
                        self.__UDP_Counter = 0
                    else:
                        self.__UDP_Counter += 1


                ## Signal watchdog that we are still running (if there is a watchdog)
                if self.Start_Mode > 0:
                    # If Keepalive signal needs to be refreshed, refresh.
                    if time.time() > self.Keepalive_timer + self.Keepalive_delay:
                        # Send the KEEPALIVE_SIGNAL to stdout. Flushing is imperative for some environments.
                        # print("sending keepalive")
                        sys.stdout.write("KEEPALIVE_SIGNAL\n")
                        sys.stdout.flush()
                        self.Keepalive_timer = time.time()

                if self.exit.isSet():
                    raise KeyboardInterrupt


                # A bit of sleep - Bonne nuit and stuff
                processing_time = time.time() - self.data.timestamp
                if (processing_time < 0.010):
                    self.__my_sleep(0.010 - processing_time)




        except KeyboardInterrupt:

            print('\nIMUSEF: User requested Application STOP\n')
            self.__clean_exit()

        except Exception as error:
            print('\nIMUSEF: Exception error: \n')
            traceback.print_exc()


            self.__clean_exit()


    # Loads the configuration from the defined File and adjusts IMUSEF
    def __loadConfig(self):

        file = open(self.__ConfigFile, "r")
        JSON_str = file.read()
        file.close()

        try:
            config = json.loads(JSON_str)
        except Exception as error:
            traceback.print_exc()
            return

        # Configure Module FLAGS
        module_config = config["ModuleConfig"]
        self.FLAG_STIMULATOR = module_config["FLAG_STIMULATOR"]
        self.FLAG_CRANKANGLE_SENSOR = module_config["FLAG_CRANKANGLE_SENSOR_OPENDAQ"]
        self.FLAG_CRANKANGLE_SENSOR_IMU_FOX = module_config["FLAG_CRANKANGLE_SENSOR_IMU_FOX"]
        self.FLAG_IMUs = module_config["FLAG_IMUs"]
        self.FLAG_HEARTRATE_MONITOR = module_config["FLAG_HEARTRATE_MONITOR"]
        self.FLAG_POWERMETER_ROTOR = module_config["FLAG_POWERMETER_ROTOR"]
        self.FLAG_HOMETRAINER = module_config["FLAG_HOMETRAINER"]

        # Configure Button FLAGS
        button_config = config["ButtonConfig"]
        self.FLAG_Emergency_Button = button_config["FLAG_BUTTON_EMERGENCY"]
        self.FLAG_Left_Button = button_config["FLAG_BUTTON_LEFT"]
        self.FLAG_Right_Button = button_config["FLAG_BUTTON_RIGHT"]
        self.FLAG_Boost_Button = button_config["FLAG_BUTTON_BOOST"]
        self.FLAG_Man_Auto_Switch = button_config["FLAG_SWITCH_MAN_AUTO"]

        # Device Config
        device_config = config["DeviceConfig"]
        self.__MAC_Stimulator = device_config["MAC_Stimulator"]
        self.__ID_Rotor_Powermeter = device_config["ID_ROTOR"]
        self.__ID_HeartRateMonitor = device_config["ID_HeartRateMonitor"]

        # Configure General Settings
        general_config = config["GeneralConfig"]
        self.CONTROL_MODE = general_config["ControlMode"]
        self.CrankAngle_Sensor = general_config["CrankAngle_Sensor"]
        self.__Cadence_Manual_Control_Max = general_config["Cadence_Manual_Mode_Max"]
        self.__Cadence_Auto_Control_Min = general_config["Automatic_Cadence_Min"]
        self.__Cadence_Auto_Control_Max = general_config["Automatic_Cadence_Max"]
        self.__MuscleDelay_ON = general_config["MuscleDelay_ON"]
        self.__MuscleDelay_OFF = general_config["MuscleDelay_OFF"]
        self.__FLAG_Compensate4Delays = general_config["Compensate4Delays"]
        self.myCyclingComputer.WheelCircumference = general_config["Circumference_RearWheel"]
        self.FLAG_PID = general_config["EnablePID"]
        self.PID_Mode = general_config["PID_Mode"]

        # PID Config
        PID_config = config["PIDConfig"]
        self.CV_Amp_Min = PID_config["CV_Amp_Min"]
        self.CV_Amp_Max = PID_config["CV_Amp_Max"]
        self.CV_Ph_Min = PID_config["CV_Ph_Min"]
        self.CV_Ph_Max = PID_config["CV_Ph_Max"]

        # Transfer new settings to the Controllers
        self.myCrankAngleController.CADENCE_LIMIT_MIN = self.__Cadence_Auto_Control_Min
        self.myCrankAngleController.CADENCE_LIMIT_MAX = self.__Cadence_Auto_Control_Max
        self.myCrankAngleController.compensate4Delays = self.__FLAG_Compensate4Delays
        self.myCrankAngleController.MUSCLE_DELAY_ON = self.__MuscleDelay_ON
        self.myCrankAngleController.MUSCLE_DELAY_OFF = self.__MuscleDelay_OFF

        self.myThighAngleController.CADENCE_LIMIT_MIN = self.__Cadence_Auto_Control_Min
        self.myThighAngleController.CADENCE_LIMIT_MAX = self.__Cadence_Auto_Control_Max
        self.myThighAngleController.compensate4Delays = self.__FLAG_Compensate4Delays
        self.myThighAngleController.MUSCLE_DELAY_ON = self.__MuscleDelay_ON
        self.myThighAngleController.MUSCLE_DELAY_OFF = self.__MuscleDelay_OFF

        # Stimulator Config
        self.myMM_Manager.setConfiguration(config["StimConfig"])

        # Controller Config
        self.myBiodexController.setConfig(config["Biodex_ControllerConfig"])
        self.myCrankAngleController.setConfig(config["CrankAngle_ControllerConfig"])
        self.myKneeAngleController.setConfig(config["KneeAngle_ControllerConfig"])
        self.myThighAngleController.setAdvancedConfig(config["ThighAngle_ControllerConfig"])
        #self.myObserverController.setConfig(config["Observer_ControllerConfig"])


    # Saves the current IMUSEF configuration to file
    def __saveConfig(self):

        module_config = {
            "@@@@@@@FLAG_STIMULATOR": self.FLAG_STIMULATOR,
            "@@@@@@FLAG_CRANKANGLE_SENSOR_OPENDAQ": self.FLAG_CRANKANGLE_SENSOR,
            "@@@@@FLAG_CRANKANGLE_SENSOR_IMU_FOX": self.FLAG_CRANKANGLE_SENSOR_IMU_FOX,
            "@@@@FLAG_IMUs": self.FLAG_IMUs,
            "@@@FLAG_HEARTRATE_MONITOR": self.FLAG_HEARTRATE_MONITOR,
            "@@FLAG_POWERMETER_ROTOR": self.FLAG_POWERMETER_ROTOR,
            "@FLAG_HOMETRAINER": self.FLAG_HOMETRAINER
        }

        button_config = {
            "@@@@@FLAG_BUTTON_EMERGENCY": self.FLAG_Emergency_Button,
            "@@@@FLAG_BUTTON_LEFT": self.FLAG_Left_Button,
            "@@@FLAG_BUTTON_RIGHT": self.FLAG_Right_Button,
            "@@FLAG_BUTTON_BOOST": self.FLAG_Boost_Button,
            "@FLAG_SWITCH_MAN_AUTO": self.FLAG_Man_Auto_Switch
        }

        device_config = {
            "@@@MAC_Stimulator": self.__MAC_Stimulator,
            "@@ID_ROTOR": self.__ID_Rotor_Powermeter,
            "@ID_HeartRateMonitor": self.__ID_HeartRateMonitor
        }


        # Manual Control cannot be selected - thus keep the last automatic mode used
        controller = self.CONTROL_MODE
        if self.CONTROL_MODE == CONTROLLER_MANUAL:
            controller = self.LAST_CONTROL_MODE

        general_config = {

            "@@@@@@@@@@@PID_Mode" : self.PID_Mode,
            "@@@@@@@@@@EnablePID" : self.FLAG_PID,
            "@@@@@@@@@ControlMode" : controller,
            "@@@@@@@@CrankAngle_Sensor": self.CrankAngle_Sensor,
            "@@@@@@@Cadence_Manual_Mode_Max" : self.__Cadence_Manual_Control_Max,
            "@@@@@@Automatic_Cadence_Min": self.__Cadence_Auto_Control_Min,
            "@@@@@Automatic_Cadence_Max": self.__Cadence_Auto_Control_Max,
            "@@@@Compensate4Delays": self.__FLAG_Compensate4Delays,
            "@@@MuscleDelay_ON": self.__MuscleDelay_ON,
            "@@MuscleDelay_OFF": self.__MuscleDelay_OFF,
            "@Circumference_RearWheel": self.myCyclingComputer.WheelCircumference
        }

        PID_config = {
            "@@@@CV_Amp_Min": self.CV_Amp_Min,
            "@@@CV_Amp_Max": self.CV_Amp_Max,
            "@@CV_Ph_Min": self.CV_Ph_Min,
            "@CV_Ph_Max": self.CV_Ph_Max,
        }

        config = {
            "@@@@@@@@@@PIDConfig": PID_config,
            "@@@@@@@@@ModuleConfig": module_config,
            "@@@@@@@@ButtonConfig": button_config,
            "@@@@@@@@DeviceConfig": device_config,
            "@@@@@@@GeneralConfig": general_config,
            "@@@@@@StimConfig": self.__getSortedStimConfig(self.myMM_Manager.getStimConfig()),
            "@@@@@Biodex_ControllerConfig": self.__getSortedControllerConfig(self.myBiodexController.getConfig()),
            "@@@@CrankAngle_ControllerConfig": self.__getSortedControllerConfig(self.myCrankAngleController.getConfig()),
            "@@@KneeAngle_ControllerConfig": self.__getSortedControllerConfig(self.myKneeAngleController.getConfig()),
            "@@ThighAngle_ControllerConfig": self.myThighAngleController.getAdvancedConfig(),
            #"@Observer_ControllerConfig": self.__getSortedControllerConfig(self.myObserverController.getConfig())
        }

        config_JSON = json.dumps(config, sort_keys=True, indent=4, separators=(',', ': '))
        config_JSON = config_JSON.replace("@","")
        #print(config_JSON)

        file = open(self.__ConfigFile, "w")
        file.write(config_JSON)
        file.close()


    # Sorts the ParameterNames by the use of @ - make sure to remove after creation of JSON-String
    def __getSortedStimConfig(self, _config):

        sorted_config = {
                "@@@@@@@@@F": _config["F"],
                "@@@@@@@@F_Boost": _config["F_Boost"],
                "@@@@@@@Monophasic": _config["Monophasic"],
                "@@@@@@PhW": _config["PhW"],
                "@@@@@PhW_Boost": _config["PhW_Boost"],
                "@@@@IPG": _config["IPG"],
                "@@@I_Max": _config["I_Max"],
                "@@RampUP": _config["RampUP"],
                "@RampDOWN": _config["RampDOWN"]
        }

        return sorted_config


    # Sorts the ParameterNames by the use of @ - make sure to remove after creation of JSON-String
    def __getSortedControllerConfig(self, _config):

        sorted_config = {
            "@@@@@Name": _config["Name"],
            "@@@@Min": _config["Min"],
            "@@@Max": _config["Max"],
            "@@Start": _config["Start"],
            "@Stop": _config["Stop"]
        }

        return sorted_config



    # Interprets and processes a new message received from the TCP-Client
    def __handleMessage(self, msg):

        ## Return Current stimulation configuration to the TCP-Client
        if msg.CMD == TCP_Message.CMD_GET_STIM_PARAMS:

            self.myGPIO_Manager.DoubleBeep()
            stim_params = self.myMM_Manager.getStimConfig()
            msg.buildMessage(TCP_Message.CMD_RE_GET_STIM_PARAMS, json.dumps(stim_params))
            self.myTCP_Server.sendData(msg.toString())

        ## Re-Configure Stimulator
        elif msg.CMD == TCP_Message.CMD_SET_STIM_PARAMS:

            try:
                params = json.loads(msg.DATA)
            except Exception as error:
                traceback.print_exc()
                return

            # Inform that the message was received
            self.myGPIO_Manager.Beep()
            self.myRemoteController.update(I=0, CH_active=[False, False, False, False, False, False, False, False])
            update_OK = self.myMM_Manager.setConfiguration(params)


            # Inform that the update failed
            if not update_OK:
                # Annoying beep :(
                self.myGPIO_Manager.Beep(0.5)

                # Inform the TCP-Client
                msg.buildMessage(TCP_Message.CMD_RE_SET_STIM_PARAMS, "{"+str(update_OK)+"}")
                self.myTCP_Server.sendData(msg.toString())

            else:
                # Happy beep :)
                self.myGPIO_Manager.DoubleBeep()

                msg.buildMessage(TCP_Message.CMD_RE_SET_STIM_PARAMS, "{"+str(update_OK)+"}")
                self.myTCP_Server.sendData(msg.toString())

            # Saves new config to keep the values for the next start
            self.__saveConfig()

        ##Return PID Parameters to TCP Client
        elif msg.CMD == TCP_Message.CMD_GET_PID_PARAMS:
            self.myGPIO_Manager.DoubleBeep()
            PID_params = {
                "CV_Amp_Min": self.CV_Amp_Min,
                "CV_Amp_Max": self.CV_Amp_Max,
                "CV_Ph_Min": self.CV_Ph_Min,
                "CV_Ph_Max": self.CV_Ph_Max,}
            msg.buildMessage(TCP_Message.CMD_RE_GET_PID_PARAMS, json.dumps(PID_params))
            self.myTCP_Server.sendData(msg.toString())

        ##Configure PID Parameters
        elif msg.CMD == TCP_Message.CMD_SET_PID_PARAMS:

            try:
                params = json.loads(msg.DATA)
            except Exception as error:
                traceback.print_exc()
                return

            # Inform that the message was received
            self.myGPIO_Manager.Beep()
            if self.PID_Mode == 2:
                self.CV_Ph_Min = params["CV_Ph_Min"]
                self.CV_Ph_Max = params["CV_Ph_Max"]
                update_OK = self.myPIDController.updateCV(self.CV_Ph_Min,self.CV_Ph_Max)
            else:
                self.CV_Amp_Min = params["CV_Amp_Min"]
                self.CV_Amp_Max = params["CV_Amp_Max"]
                update_OK = self.myPIDController.updateCV(self.CV_Amp_Min, self.CV_Amp_Max)


            # Inform that the update failed
            if not update_OK:
                # Annoying beep
                self.myGPIO_Manager.Beep(0.5)

                # Inform the TCP-Client
                msg.buildMessage(TCP_Message.CMD_RE_SET_PID_PARAMS, str(update_OK))
                self.myTCP_Server.sendData(msg.toString())

            else:
                msg.buildMessage(TCP_Message.CMD_RE_SET_PID_PARAMS, str(update_OK))
                self.myTCP_Server.sendData(msg.toString())

            # Saves new config to keep the values for the next start
            self.__saveConfig()

        ## Execute Test-Stimulation
        elif msg.CMD == TCP_Message.CMD_SET_TEST_STIMULATION:

            # Inform that the message was received
            self.myGPIO_Manager.Beep()

            # Update TCP-Controller
            self.myRemoteController.update(JSON=msg.DATA)

        ## Change Stimulation Intensity
        elif msg.CMD == TCP_Message.CMD_SET_STIMULATION_INTENSITY:

            # Inform that the message was received
            self.myGPIO_Manager.Beep()

            # Update TCP-Controller
            self.myMM_Manager.setIntensity(I = int(msg.DATA))

        # Return current Settings
        elif msg.CMD == TCP_Message.CMD_GET_SETTINGS:

            recording_active = False
            file_path = ""

            if not self.myDataLogger == None:
                recording_active = (self.myDataLogger.Status == 1)
                file_path = self.myDataLogger.file_path

            _settings = {
                "Simulate_CrankAngle": self.FLAG_SIMULATE_CRANKANGLE,
                "Simulate_ThighAngles": self.FLAG_SIMULATE_THIGHANGLES,
                "Simulate_KneeAngles": self.FLAG_SIMULATE_KNEEANGLES,
                "Simulated_Cadence": int(self.__simulatedCadence),

                "DataLogging_Active": recording_active,
                "DataLogging_FilePath": file_path,

                "Biodex_Side": self.myBiodexController.getSide(),

                "FLAG_Button_Emergency":self.FLAG_Emergency_Button,
                "FLAG_Button_Left": self.FLAG_Left_Button,
                "FLAG_Button_Right":self.FLAG_Right_Button,
                "FLAG_Button_Boost": self.FLAG_Boost_Button,
                "FLAG_Switch_Man_Auto": self.FLAG_Man_Auto_Switch,
                "FLAG_Module_Stimulator": self.FLAG_STIMULATOR,
                "FLAG_Module_CrankAngle_Sensor_OpenDAQ": self.FLAG_CRANKANGLE_SENSOR,
                "FLAG_Module_CrankAngle_Sensor_IMU_FOX": self.FLAG_CRANKANGLE_SENSOR_IMU_FOX,
                "FLAG_Module_IMUs": self.FLAG_IMUs,
                "FLAG_Module_Heartrate_Monitor": self.FLAG_HEARTRATE_MONITOR,
                "FLAG_Module_PowerMeter_Rotor": self.FLAG_POWERMETER_ROTOR,
                "FLAG_Module_HomeTrainer": self.FLAG_HOMETRAINER,

                "MAC_Stimulator": self.__MAC_Stimulator,
                "ID_ROTOR": self.__ID_Rotor_Powermeter,
                "ID_HeartRateMonitor": self.__ID_HeartRateMonitor,

                "CrankAngle_Sensor": self.CrankAngle_Sensor,
                "Manual_Cadence_Max": self.__Cadence_Manual_Control_Max,
                "Automatic_Cadence_Min": self.__Cadence_Auto_Control_Min,
                "Automatic_Cadence_Max": self.__Cadence_Auto_Control_Max,
                "MuscleDelay_ON": self.__MuscleDelay_ON,
                "MuscleDelay_OFF": self.__MuscleDelay_OFF,
                "Compensate4Delays": self.__FLAG_Compensate4Delays,
                "EnablePID": self.FLAG_PID,
                "PID_Mode": self.PID_Mode,
                "WheelCircumference": (int)(self.myCyclingComputer.WheelCircumference*1000),
                "StartMode": self.Start_Mode,

                "ThighAngleController_Smoothing_Cadence_LOW": self.myThighAngleController.Smoothing_Cadence_LOW,
                "ThighAngleController_Smoothing_Cadence_HIGH": self.myThighAngleController.Smoothing_Cadence_HIGH,
                "ThighAngleController_Smoothing_MIN": self.myThighAngleController.Smoothing_Factor_MIN,
                "ThighAngleController_Smoothing_MAX": self.myThighAngleController.Smoothing_Factor_MAX,
                "ThighAngleController_LinearizeNormalisation": self.myThighAngleController.LineariseNormalisation,
                "ThighAngleController_Min_DirectionChange": self.myThighAngleController.MIN_DIR_CHANGE,
                "ThighAngleController_Min_RangeOfMotion": self.myThighAngleController.ROM_LIMIT
            }

            msg.buildMessage(TCP_Message.CMD_RE_SETTINGS, json.dumps(_settings))
            self.myTCP_Server.sendData(msg.toString())

        # Return current Controller Mode
        elif msg.CMD == TCP_Message.CMD_SET_SETTINGS:

            try:
                _settings = json.loads(msg.DATA)
            except Exception as error:
                traceback.print_exc()
                return

            # Switch off TCP-Controller
            self.myRemoteController.update(I=0, CH_active=[False, False, False, False, False, False, False, False])

            # Controller
            controller = _settings["Controller"]

            if (    controller == CONTROLLER_NONE or
                    controller == CONTROLLER_TCP or
                    controller == CONTROLLER_CRANKANGLE or
                    controller == CONTROLLER_THIGHANGLE or
                    controller == CONTROLLER_OBSERVER or
                    controller == CONTROLLER_BIODEX or
                    controller == CONTROLLER_AUTOTUNE):

                self.CONTROL_MODE = controller

            # Simulation Settings
            self.__simulatedCadence = _settings["Simulated_Cadence"]
            self.FLAG_SIMULATE_CRANKANGLE = _settings["Simulate_CrankAngle"]
            self.FLAG_SIMULATE_KNEEANGLES = _settings["Simulate_KneeAngles"]
            self.FLAG_SIMULATE_THIGHANGLES = _settings["Simulate_ThighAngles"]

            self.__sim_CrankAngleSpeed = (self.__simulatedCadence / 60.0) * 360.0
            self.__sim_w = 2 * np.pi * (self.__simulatedCadence / 60.0)

            # Button Flags
            self.FLAG_Emergency_Button = _settings["FLAG_Button_Emergency"]
            self.FLAG_Left_Button = _settings["FLAG_Button_Left"]
            self.FLAG_Right_Button = _settings["FLAG_Button_Right"]
            self.FLAG_Boost_Button = _settings["FLAG_Button_Boost"]
            self.FLAG_Man_Auto_Switch = _settings["FLAG_Switch_Man_Auto"]

            # Module Flags
            self.FLAG_STIMULATOR = _settings["FLAG_Module_Stimulator"]
            self.FLAG_CRANKANGLE_SENSOR = _settings["FLAG_Module_CrankAngle_Sensor_OpenDAQ"]
            self.FLAG_CRANKANGLE_SENSOR_IMU_FOX = _settings["FLAG_Module_CrankAngle_Sensor_IMU_FOX"]
            self.FLAG_IMUs = _settings["FLAG_Module_IMUs"]
            self.FLAG_HEARTRATE_MONITOR = _settings["FLAG_Module_Heartrate_Monitor"]
            self.FLAG_POWERMETER_ROTOR = _settings["FLAG_Module_PowerMeter_Rotor"]
            self.FLAG_HOMETRAINER = _settings["FLAG_Module_HomeTrainer"]

            # Device Identification
            self.__MAC_Stimulator = _settings["MAC_Stimulator"]
            self.__ID_Rotor_Powermeter = _settings["ID_ROTOR"]
            self.__ID_HeartRateMonitor = _settings["ID_HeartRateMonitor"]

            # General Settings
            self.CrankAngle_Sensor = _settings["CrankAngle_Sensor"]
            self.myCyclingComputer.WheelCircumference = _settings["WheelCircumference"]/1000.0
            self.__Cadence_Manual_Control_Max = _settings["Manual_Cadence_Max"]

            self.__Cadence_Auto_Control_Min = _settings["Automatic_Cadence_Min"]
            self.__Cadence_Auto_Control_Max = _settings["Automatic_Cadence_Max"]
            self.__FLAG_Compensate4Delays = _settings["Compensate4Delays"]
            self.FLAG_PID = _settings["EnablePID"]
            self.PID_Mode = _settings["PID_Mode"]
            self.__MuscleDelay_ON = _settings["MuscleDelay_ON"]
            self.__MuscleDelay_OFF = _settings["MuscleDelay_OFF"]

            # Transfer new settings to the Controllers
            self.myCrankAngleController.CADENCE_LIMIT_MIN = self.__Cadence_Auto_Control_Min
            self.myCrankAngleController.CADENCE_LIMIT_MAX = self.__Cadence_Auto_Control_Max
            self.myCrankAngleController.compensate4Delays = self.__FLAG_Compensate4Delays
            self.myCrankAngleController.MUSCLE_DELAY_ON = self.__MuscleDelay_ON
            self.myCrankAngleController.MUSCLE_DELAY_OFF = self.__MuscleDelay_OFF

            self.myThighAngleController.CADENCE_LIMIT_MIN = self.__Cadence_Auto_Control_Min
            self.myThighAngleController.CADENCE_LIMIT_MAX = self.__Cadence_Auto_Control_Max
            self.myThighAngleController.compensate4Delays = self.__FLAG_Compensate4Delays
            self.myThighAngleController.MUSCLE_DELAY_ON = self.__MuscleDelay_ON
            self.myThighAngleController.MUSCLE_DELAY_OFF = self.__MuscleDelay_OFF

            # Settings ThighAngleController
            self.myThighAngleController.Smoothing_Cadence_LOW = _settings["ThighAngleController_Smoothing_Cadence_LOW"]
            self.myThighAngleController.Smoothing_Cadence_HIGH = _settings["ThighAngleController_Smoothing_Cadence_HIGH"]
            self.myThighAngleController.Smoothing_Factor_MIN = _settings["ThighAngleController_Smoothing_MIN"]
            self.myThighAngleController.Smoothing_Factor_MAX = _settings["ThighAngleController_Smoothing_MAX"]
            self.myThighAngleController.LineariseNormalisation = _settings["ThighAngleController_LinearizeNormalisation"]
            self.myThighAngleController.ROM_LIMIT = _settings["ThighAngleController_Min_RangeOfMotion"]
            self.myThighAngleController.MIN_DIR_CHANGE = _settings["ThighAngleController_Min_DirectionChange"]


            # Saves new config to keep the values for the next start
            self.__saveConfig()

            self.myGPIO_Manager.DoubleBeep()

        # Return the requested Controller configuration
        elif msg.CMD == TCP_Message.CMD_GET_CONTROLLER_PARAMS:

            self.myGPIO_Manager.DoubleBeep()
            config = {
                "CrankAngle_ControllerConfig": self.myCrankAngleController.getConfig(),
                "KneeAngle_ControllerConfig": self.myKneeAngleController.getConfig(),
                "ThighAngle_ControllerConfig": self.myThighAngleController.getConfig(),
                #"Observer_ControllerConfig": self.myObserverController.getConfig(),
                "Biodex_ControllerConfig": self.myBiodexController.getConfig()
            }

            msg.buildMessage(TCP_Message.CMD_RE_CONTROLLER_PARAMS, json.dumps(config))
            self.myTCP_Server.sendData(msg.toString())

        # Sets new a Controller configuration
        elif msg.CMD == TCP_Message.CMD_SET_CONTROLLER_PARAMS:

            try:
                config = json.loads(msg.DATA)
            except Exception as error:
                traceback.print_exc()
                return

            self.myCrankAngleController.setConfig(config["CrankAngle_ControllerConfig"])
            self.myKneeAngleController.setConfig(config["KneeAngle_ControllerConfig"])
            self.myThighAngleController.setConfig(config["ThighAngle_ControllerConfig"])
            #.myObserverController.setConfig(config["Observer_ControllerConfig"])
            self.myBiodexController.setConfig(config["Biodex_ControllerConfig"])

            # Saves new config to keep the values for the next start
            self.__saveConfig()

            self.myGPIO_Manager.DoubleBeep()

        # Calibrates the system
        elif msg.CMD == TCP_Message.CMD_CALIBRATE_SYSTEM:
            self.myHometrainer.reCalibrate()
            self.myGPIO_Manager.DoubleBeep()

        # Stops IMUSEF
        elif msg.CMD == TCP_Message.CMD_STOP_SYSTEM:

            self.exit.set()
            self.myGPIO_Manager.DoubleBeep()

        # Resets Cycling Computer
        elif msg.CMD == TCP_Message.CMD_RESET_CYCLING_COMPUTER:

            self.myCyclingComputer.reset()
            self.myGPIO_Manager.DoubleBeep()

        # Start Recording
        elif msg.CMD == TCP_Message.CMD_START_RECORD:

            try:
                _filesettings = json.loads(msg.DATA)
            except Exception as error:
                traceback.print_exc()
                return

            self.myDataLogger = DataLogger(directory=_filesettings["Directory"],\
                                           filename=_filesettings["Filename"],\
                                           extension=_filesettings["Extension"],\
                                           timestamp=_filesettings["TimeStamp"])

            self.stop_Datalogging.clear()
            self.myDataLogger.start(self.stop_Datalogging)
            self.FLAG_LOG_DATA = True

            msg.buildMessage(TCP_Message.CMD_RE_START_RECORD, self.myDataLogger.file_path)
            self.myTCP_Server.sendData(msg.toString())

            self.myGPIO_Manager.DoubleBeep()

        # Stop Recording
        elif msg.CMD == TCP_Message.CMD_STOP_RECORD:

            if not self.myDataLogger == None:
                self.stop_Datalogging.set()
                self.FLAG_LOG_DATA = False

                msg.buildMessage(TCP_Message.CMD_RE_STOP_RECORD, self.myDataLogger.file_path)
                self.myTCP_Server.sendData(msg.toString())

                self.myDataLogger = None

                self.myGPIO_Manager.DoubleBeep()

        # Start Recording
        elif msg.CMD == TCP_Message.CMD_ADD_COMMENT:
            self.data.comment = msg.DATA

        # Start the Ziegler-Nichols Procedure (for PID)
        elif msg.CMD == TCP_Message.CMD_START_ST:
            self.FLAG_ST_TEST = True

        elif msg.CMD == TCP_Message.CMD_START_PID_TEST:
            self.FLAG_PID_TEST = True

        # Changes the Side of the BiodexController and returns the current State
        elif msg.CMD == TCP_Message.CMD_BIODEX_CHANGE_SIDE:
            if (msg.DATA == "LEFT"):
                self.myBiodexController.setSide(0)
            elif (msg.DATA == "RIGHT"):
                self.myBiodexController.setSide(1)

            self.myGPIO_Manager.DoubleBeep()

            recording_active = False
            file_path = ""

            if not self.myDataLogger == None:
                recording_active = (self.myDataLogger.Status == 1)
                file_path = self.myDataLogger.file_path

            _settings = {
                "Controller": self.CONTROL_MODE,
                "Simulate_CrankAngle": self.FLAG_SIMULATE_CRANKANGLE,
                "Simulate_ThighAngles": self.FLAG_SIMULATE_THIGHANGLES,
                "Simulate_KneeAngles": self.FLAG_SIMULATE_KNEEANGLES,
                "Simulated_Cadence": int(self.__simulatedCadence),
                "DataLogging_Active": recording_active,
                "DataLogging_FilePath": file_path,
                "Biodex_Side": self.myBiodexController.getSide()
            }

            msg.buildMessage(TCP_Message.CMD_RE_SETTINGS, json.dumps(_settings))
            self.myTCP_Server.sendData(msg.toString())

        # Calibrates the Biodex Controller to calculate the correct knee-angle
        elif msg.CMD == TCP_Message.CMD_BIODEX_CALIBRATE_KNEEANGLE:

            angle = 40
            try:
                angle = int(msg.DATA)
            except:
                angle = 40

            self.myBiodexController.calibrateKneeAngle(angle)
            self.myGPIO_Manager.DoubleBeep()

        # Configures and starts a new Turn for the AutoTune Controller
        elif msg.CMD == TCP_Message.CMD_START_AUTO_TUNE:

            try:
                _AT_config = json.loads(msg.DATA)
            except Exception as error:
                traceback.print_exc()
                return

            self.myAutoTuneController.StimChannels = _AT_config["StimChannels"]
            self.myAutoTuneController.Intensity = _AT_config["Intensity"]
            self.myAutoTuneController.StimSide = _AT_config["Side"]
            self.myAutoTuneController.set_SensorType(_AT_config["Sensor"])
            self.myAutoTuneController.NumberOfCycles = _AT_config["NumberOfCycles"]
            self.myAutoTuneController.Target_Cadence = _AT_config["TargetCadence"]
            self.myAutoTuneController.Tolerance_Cadence = _AT_config["CadenceTolerance"]
            self.myAutoTuneController.Threshold_Start = _AT_config["Threshold_Start"]
            self.myAutoTuneController.Threshold_Stop = _AT_config["Threshold_Stop"]


            self.CONTROL_MODE = CONTROLLER_AUTOTUNE
            self.myAutoTuneController.start()
            self.myGPIO_Manager.DoubleBeep()

        # Stops a new Turn for the AutoTune Controller
        elif msg.CMD == TCP_Message.CMD_STOP_AUTO_TUNE:

            self.myAutoTuneController.stop()
            self.myGPIO_Manager.DoubleBeep()




    # Updates the data Object with current information about the System
    def updateSystemStatus(self):

        # IMU´s
        if self.FLAG_IMUs == False:
            self.data.Status_IMUs = -1
        elif self.FLAG_IMUs == True and self.myIMU_Manager.getIMUsReady() == False:
            self.data.Status_IMUs = -0
        elif self.FLAG_IMUs == True and self.myIMU_Manager.getIMUsReady() == True:
            self.data.Status_IMUs = 1

        # CrankAngle Sensor (OpenDAQ)
        if self.FLAG_CRANKANGLE_SENSOR:
            self.data.Status_CrankAngle = self.myGPIO_Manager.getEncoderStatus()
        else:
            self.data.Status_CrankAngle = -1

        # Stimulator
        if self.FLAG_STIMULATOR:
            self.data.Status_Stimulator = self.myMM_Manager.getState()
            #self.data.StimIntensity = self.myMM_Manager.getStimulationIntensity()
            #self.data.StimIntensity = self.myDAQManager.getIntensity()
        else:
            self.data.Status_Stimulator = -1

        # DataLogging
        if self.FLAG_LOG_DATA == False or self.myGPIO_Manager is None:
            self.data.Status_Datalogging = -1
        elif self.FLAG_LOG_DATA == True and not self.myDataLogger is None:
            self.data.Status_Datalogging = self.myDataLogger.Status

        # HeartRate Monitor
        if self.FLAG_HEARTRATE_MONITOR:
            self.data.Status_HeartRateMonitor = self.myANT_Manager.getState_HeartRateMonitor()
        else:
            self.data.Status_HeartRateMonitor = -1

        # PowerMeter
        if self.FLAG_POWERMETER_ROTOR:
            self.data.Status_PowerMeter = self.myANT_Manager.getState_PowerMeter()
        else:
            self.data.Status_PowerMeter = -1

        # HomeTrainer
        if self.FLAG_HOMETRAINER:
            self.data.Status_HomeTrainer = self.myHometrainer.getState()
        else:
            self.data.Status_HomeTrainer = -1


    # Updates the AutoTuneController
    def updateAutoTuneController(self):

        ## Select the right Data
        # Torque Data
        TorqueData = 0

        if self.myAutoTuneController.StimSide == AutoTune_Controller.LEFT:
            TorqueData = self.data.PowerMeter_DATA.Torque_Left
        elif self.myAutoTuneController.StimSide == AutoTune_Controller.RIGHT:
            TorqueData = self.data.PowerMeter_DATA.Torque_Right

        # Sensor Data
        SensorData = 0

        if self.myAutoTuneController.get_SensorType() == AutoTune_Controller.CRANKANGLE:
            SensorData = self.data.PowerMeter_DATA.CrankAngle

        elif self.myAutoTuneController.get_SensorType() == AutoTune_Controller.THIGHANGLE:
            if self.myAutoTuneController.StimSide == AutoTune_Controller.LEFT:
                SensorData = self.data.ThighAngle_Controller_DATA.NormalizedAngle_Left
            elif self.myAutoTuneController.StimSide == AutoTune_Controller.RIGHT:
                SensorData = self.data.ThighAngle_Controller_DATA.NormalizedAngle_Right

        # Update AutoTune
        self.myAutoTuneController.update(SensorData, TorqueData)

        # Read results
        self.data.AutoTune_Cadence = self.myAutoTuneController.Cadence
        self.data.AutoTune_n_PassiveCycles = self.myAutoTuneController.n_passive
        self.data.AutoTune_n_ActiveCycles = self.myAutoTuneController.n_active
        self.data.Status_AutoTune = self.myAutoTuneController.get_Status()

        # Make nice BEEPs
        if self.myAutoTuneController.BEEP:
            self.myAutoTuneController.BEEP = False
            self.myGPIO_Manager.Beep()

        # Got all measurements :) - Send it
        if self.myAutoTuneController.get_Status() == 2:

            # Send Results
            msg = TCP_Message()
            msg.buildMessage(TCP_Message.CMD_RE_AUTO_TUNE_RESULT, self.myAutoTuneController.get_JSON())
            self.myTCP_Server.sendData(msg.toString())

            self.myAutoTuneController.finishMeasurement()
            self.myGPIO_Manager.DoubleBeep()


    def __my_sleep(self, duration):
        time2sleep = time.time() + duration - 0.0001
        while time.time() < time2sleep:
            time.sleep(0.0001)

    # Tries to perform a clean exit - shuting down all modules individually
    def __clean_exit(self):

        #TODO: implement nicely

        # OpenAnt
        if self.FLAG_POWERMETER_ROTOR:
            self.myANT_Manager.clean_exit()

        # Switch off System LED
        self.myGPIO_Manager.setSystemLED(False)

        # Stop Modules
        self.exit.set()

        if not self.myDataLogger == None:
            self.stop_Datalogging.set()

        self.myTCP_Server.stop()

        if self.FLAG_IMUs:
            self.myIMU_Manager.stop()

        if self.FLAG_STIMULATOR:
            self.myMM_Manager.disconnect()

        time.sleep(3)

        print('\n\n IMUSEF STOPPED - Au revoir!')
        sys.exit()



if __name__ == '__main__':
    
    # If started by service, the Start_Mode will be 2
    # If watchdog on his own is starting the script, Start_mode will be 1
    # Otherwise the Start_mode defaults to False when IMUSEF is started directly

    Start_mode = 0

    if len(sys.argv) >= 2:
        # print("Number of start arguments :", len(sys.argv))
        Start_mode = sys.argv[1]
        print("Start_mode flag from watchdog monitoring changed : ", Start_mode)

    print("Start_mode flag is : ", Start_mode)
        
    imusef = IMUSEF(Start_mode)
    imusef.run()