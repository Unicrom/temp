from enum import (IntEnum, auto)

from wpilib import CAN

#region RoboRio Constants
# included to help with communication and readability
class Rio_DIO(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    # TODO figure out how CAN works..
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()

class Rio_Pnue(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()


class Rio_PWM(IntEnum):
    ONE = 0
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()

class Rio_Relay(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()

class Rio_Analog(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
#endregion

#region CAN Constants
class CAN_Address(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()

#endregion
class DriveConstant:
    #purple Haze setup
    #rev neo 550 pulled from https://github.com/Team2556/Crescendo/tree/master/src/main/deploy/swerve/modules
    kDriveTye = "Tank"#"Swerve"#"Mecanum"#"Xdrive" #"Tank" #
    kLeftMotorFront_DrivePort = CAN_Address.TWO
    kRightMotorFront_DrivePort = CAN_Address.EIGHT
    kLeftMotorRear_DrivePort = CAN_Address.FOUR
    kRightMotorRear_DrivePort = CAN_Address.SIX
    #talonfx
    kLeftMotorFront_SteerPort = CAN_Address.ONE
    kRightMotorFront_SteerPort = CAN_Address.SEVEN
    kLeftMotorRear_SteerPort = CAN_Address.THREE
    kRightMotorRear_SteerPort = CAN_Address.FIVE
    

    #SANTANA setup
    # kDriveTye = "Mecanum"#"Xdrive" #"Tank" #
    # kLeftMotor1Port = CAN_Address.ONE
    # kRightMotor1Port = CAN_Address.TWO
    # kLeftMotor2Port = CAN_Address.THREE
    # kRightMotor2Port = CAN_Address.FOUR
    #nvx in rio for imu kImuPort = CAN_Address.SIX
    # reggie setup
    # kLeftMotor1Port = CAN_Address.THREE
    # kRightMotor1Port = CAN_Address.TWO
    # kLeftMotor2Port = CAN_Address.ONE
    # kRightMotor2Port = CAN_Address.FOUR
    # kImuPort = CAN_Address.SIX

    kFrontLeftEncoderPorts = (Rio_DIO.TEN, Rio_DIO.ELEVEN)
    kFrontRightEncoderPorts = (Rio_DIO.TWELVE, Rio_DIO.THIRTEEN)
    kBackLeftEncoderPorts = (Rio_DIO.FOURTEEN, Rio_DIO.FIFTEEN)
    kBackRightEncoderPorts = (Rio_DIO.SIXTEEN, Rio_DIO.SEVENTEEN)
    kEncoderDistancePerPulse = 1
    kMaxOutput = .9123
    kDeadband = .1
    kWheelBase = 1.111 # meters
    kTrackWidth = 1.112 # meters

class OIConstant:
    kDriver1ControllerPort = 0

    kDriver2ControllerPort = 1


class CannonConstant:
    kCompressorAddress = CAN_Address.FIVE
    kRelayAddress = Rio_Relay.ZERO
    