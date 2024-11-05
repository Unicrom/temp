from random import shuffle
import commands2
# from commands2 import Subsystem, CommandScheduler

import phoenix5.sensors
import wpilib
from wpilib import DriverStation, SmartDashboard
from wpilib.shuffleboard import (Shuffleboard, BuiltInWidgets)
import wpilib.drive
# import wpimath
import wpimath
from wpimath.geometry import Rotation2d
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelSpeeds, MecanumDriveOdometry,SwerveDrive4Kinematics
wpimath.kinematics

import phoenix5
import wpiutil

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants


from constants import DriveConstant

import typing
import math

class DriveTrain(commands2.Subsystem):
    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()
        # Register the subsystem with the CommandScheduler
        # CommandScheduler.getInstance().registerSubsystem(self) when calling it in robot it registers it

        self.right_invert_YN = True

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how the robot's
        # gearbox is constructed, you might have to invert the left side instead.    
        self.frontLeftMotor = phoenix5.WPI_TalonSRX(DriveConstant.kLeftMotor1Port)
        SmartDashboard.putData("frontLeftMotor -from drivetrain", self.frontLeftMotor)
        # self.frontLeftMotor.config_kF(0, 0.0)

        self.frontRightMotor = phoenix5.WPI_TalonSRX(DriveConstant.kRightMotor1Port)
        # self.frontRightMotor.getSimCollection().getMotorOutputLeadVoltage()
        self.frontRightMotor.setInverted(self.right_invert_YN)
        SmartDashboard.putData("frontRightMotor -from drivetrain", self.frontRightMotor)
        self.backLeftMotor = phoenix5.WPI_TalonSRX(DriveConstant.kLeftMotor2Port)
        self.backLeftMotor.config_kF(0, 0.10)
        SmartDashboard.putData("backLeftMotor -from drivetrain", self.backLeftMotor)
        self.backRightMotor = phoenix5.WPI_TalonSRX(DriveConstant.kRightMotor2Port)
        self.backRightMotor.setInverted(self.right_invert_YN)
        SmartDashboard.putData("backRightMotor -from drivetrain", self.backRightMotor)

        talon = phoenix5.WPI_TalonSRX(DriveConstant.kImuPort)
        self.gyro = phoenix5.sensors.WPI_PigeonIMU(talon)
        self.gyro.reset()
        Shuffleboard.getTab("IMU").add("IMU", self.gyro)
        
        ''' not using encoder yet
        # Set up encoders for each motor
        self.frontLeftEncoder = self.frontLeftMotor.getSensorCollection().getQuadraturePosition
        SmartDashboard.putNumber("frontLeftEncoder -from drivetrain", self.frontLeftEncoder())
        self.frontRightEncoder = self.frontRightMotor.getSensorCollection().getQuadraturePosition
        self.backLeftEncoder = self.backLeftMotor.getSensorCollection().getQuadraturePosition
        self.backRightEncoder = self.backRightMotor.getSensorCollection().getQuadraturePosition

        # Reset encoders to zero
        self.frontLeftMotor.getSensorCollection().setQuadraturePosition(0, 10)
        self.frontRightMotor.getSensorCollection().setQuadraturePosition(0, 10)
        self.backLeftMotor.getSensorCollection().setQuadraturePosition(0, 10)
        self.backRightMotor.getSensorCollection().setQuadraturePosition(0, 10)
        '''
        match DriveConstant.kDriveTye:
            case "Xdrive":
                self.robotDrive = TheWB_Xdrive(self.frontLeftMotor, self.frontRightMotor, self.backLeftMotor, self.backRightMotor)
            case "Tank":
                self.backLeftMotor.follow(self.frontLeftMotor)
                self.backRightMotor.follow(self.frontRightMotor)
                self.robotDrive = wpilib.drive.DifferentialDrive(self.frontLeftMotor, self.frontRightMotor)
            case "Mecanum": 
                self.robotDrive = wpilib.drive.MecanumDrive(frontLeftMotor= self.frontLeftMotor,    
                                                            frontRightMotor=self.frontRightMotor,
                                                            rearLeftMotor=self.backLeftMotor,
                                                            rearRightMotor=self.backRightMotor)
            case "Swerve":
                self.robotDrive = wpilib.drive.SwerveDrive(self.frontLeftMotor, self.backLeftMotor, self.frontRightMotor, self.backRightMotor)

        
        # if DriveConstant.kIsMecanum:
        #     self.robotDrive = wpilib.drive.MecanumDrive(frontLeftMotor= self.frontLeftMotor, 
        #                                                 frontRightMotor=self.frontRightMotor, 
        #                                                 rearLeftMotor=self.backLeftMotor, 
        #                                                 rearRightMotor=self.backRightMotor)
        # else:
        #     self.backLeftMotor.follow(self.frontLeftMotor)
        #     self.backRightMotor.follow(self.frontRightMotor)
        #     self.robotDrive = wpilib.drive.DifferentialDrive(self.frontLeftMotor, self.frontRightMotor)
        
        
       
        # self.choosable_maxoutput = (Shuffleboard.getTab("MaxSpeed")
        #                   .add("MaxSpeed", 0.40)
        #                   .withWidget(BuiltInWidgets.kNumberSlider)
        #                   .getEntry()
        #                    )
        # self.robotDrive.setMaxOutput( self.choosable_maxoutput.getFloat(0.40) )

        
        self.robotDrive.setMaxOutput( 0.80 )
        self.robotDrive.setDeadband(0.15)
        self.robotDrive.driveCartesian(0, 0, 0)
        '''rest are defaults so far:
        self.robotDrive.setExpiration(.05)'''

        #define the expected methods for pathplannerlib
        AutoBuilder.configureHolonomic(
            pose_supplier=self.getPose,
            reset_pose= self.resetPose,
            robot_relative_speeds_supplier=self.getRobotRelativeSpeeds,
            robot_relative_output=self.driveRobotRelative,
            config=HolonomicPathFollowerConfig(
                translationConstants=PIDConstants(5, 0.0, 0.0),
                rotationConstants=PIDConstants(5, 0.0, 0.0),
                maxModuleSpeed=1.0,
                driveBaseRadius=0.5,
                replanningConfig=ReplanningConfig(
                    enableInitialReplanning=True, 
                    enableDynamicReplanning=True, 
                    dynamicReplanningTotalErrorThreshold=1.0, 
                    dynamicReplanningErrorSpikeThreshold=0.25),
                    period=0.02),
            should_flip_path=self.shouldFlipPath,
            drive_subsystem=self
        )
    
    def shouldFlipPath():
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def getPose(self): #need encoders to feed odometry to produce a dynamic pose
        return 
    def resetPose(self):
        self.gyro.reset()
    

    def periodic(self) -> None:
        """This method will be called once per scheduler run"""
        self.gyro.setYawToCompass()
        # Add code here that needs to run periodically
        # SmartDashboard.putNumber("frontLeftMotor -from drivetrain getmotoroutputpercent", self.frontLeftMotor.getMotorOutputPercent())
        # SmartDashboard.putNumber("frontRightMotor -from drivetrain getmotoroutputpercent", self.frontRightMotor.getMotorOutputPercent())
        # SmartDashboard.putNumber("backLeftMotor -from drivetrain getmotoroutputpercent", self.backLeftMotor.getMotorOutputPercent())
        # SmartDashboard.putNumber("backRightMotor -from drivetrain getmotoroutputpercent", self.backRightMotor.getMotorOutputPercent())

        # SmartDashboard.putData("robotDrive -from drivetrain periodic", self.robotDrive)  


    def driveWithJoystick(self, joystick: wpilib.Joystick):
        """Drives the robot using the joystick"""
        if isinstance(self.robotDrive, wpilib.drive.MecanumDrive):
            self.robotDrive.driveCartesian(joystick.getLeftX(), joystick.getRightX(),-joystick.getLeftY(),  Rotation2d(0))
        elif isinstance(self.robotDrive, wpilib.drive.DifferentialDrive):
            self.robotDrive.arcadeDrive(joystick.getLeftY(), -joystick.getLeftX())
        else:
            self.robotDrive.driveCartesian(-joystick.getLeftY(), -joystick.getLeftX(), joystick.getRightX())
    def halt(self) -> None:
        self.robotDrive.driveCartesian(0, 0, 0)#, Rotation2d(0))
    def slowLeft(self,joystick: wpilib.Joystick) -> None:
        self.robotDrive.driveCartesian(0, 0, -.22, Rotation2d(0))
    def slowRight(self,joystick: wpilib.Joystick) -> None:
        self.robotDrive.driveCartesian(0, 0, .22, Rotation2d(0))
    # flight Checklist commands
    def OnlyFrontLeft(self) -> None:
        self.frontLeftMotor.set(0.51)
    def OnlyFrontRight(self) -> None:
        self.frontRightMotor.set(0.52)
    def OnlyBackLeft(self) -> None:
        self.backLeftMotor.set(0.53)
    def OnlyBackRight(self) -> None:
        self.backRightMotor.set(0.54)

class TheWB_Xdrive:
    def __init__(self, frontLeftmotor, frontRightmotor, backLeftmotor, backRightmotor):
        self.frontLeftmotor = frontLeftmotor
        self.backLeftmotor = backLeftmotor
        self.frontRightmotor = frontRightmotor
        self.backRightmotor = backRightmotor
        
        self.Deadband = 0.1
        self.MaxOutput = .995
    
    def setMaxOutput(self, maxOutput: float):
        self.MaxOutput = maxOutput
    def setDeadband(self, deadband: float):
        self.Deadband = deadband

    def driveCartesian(self, xSpeed, ySpeed, zRotation): #, gyroAngle:: wpimath.geometry._geometry.Rotation2d  = 0.0):
        '''
        xSpeed: The speed that the robot should drive in its X direction. [-1.0..1.0]
        ySpeed: The speed that the robot should drive in its Y direction. [-1.0..1.0]
        zRotation: The rate of rotation for the robot that is independent of the translation. [-1.0..1.0]
        '''
        SmartDashboard.putNumber("Dedband Settting", self.Deadband)
        xSpeed = xSpeed if abs(xSpeed) > self.Deadband else 0.0
        ySpeed = ySpeed if abs(ySpeed) > self.Deadband else 0.0
        zRotation = zRotation if abs(zRotation) > self.Deadband else 0.0
        
        SmartDashboard.putNumber("xSpeed in TheWB_drivecartesian", xSpeed)
        SmartDashboard.putNumber("ySpeed in TheWB_drivecartesian", ySpeed)
        SmartDashboard.putNumber("zRotation in TheWB_drivecartesian", zRotation)
        
        #create coding for the mecanum drive kinematics
        base_theta = math.atan2(xSpeed, ySpeed) - math.pi / 4.0
        r = math.hypot(xSpeed, ySpeed)
        cos = math.cos(base_theta)
        sin = math.sin(base_theta)
        max_trig = max(abs(cos), abs(sin))
        leftFront = r * sin/max_trig + zRotation
        rightFront = r * cos/max_trig - zRotation
        leftRear = r * cos/max_trig + zRotation
        rightRear = r * sin/max_trig - zRotation
        # leftFront = r * cos/max_trig + zRotation
        # rightFront = r * sin/max_trig - zRotation
        # leftRear = r * sin/max_trig + zRotation
        # rightRear = r * cos/max_trig - zRotation

        # Limit the toal power to the motors to self.MaxOutput

        maxMagnitude = max(max(abs(leftFront), abs(rightFront)), max(abs(leftRear), abs(rightRear)))
        if maxMagnitude > self.MaxOutput:
            leftFront /= maxMagnitude/self.MaxOutput
            rightFront /= maxMagnitude/self.MaxOutput
            leftRear /= maxMagnitude/self.MaxOutput
            rightRear /= maxMagnitude/self.MaxOutput
        SmartDashboard.putNumber("leftFront in TheWB_drivecartesian", leftFront)
        SmartDashboard.putNumber("rightFront in TheWB_drivecartesian", rightFront)
        SmartDashboard.putNumber("leftRear in TheWB_drivecartesian", leftRear)
        SmartDashboard.putNumber("rightRear in TheWB_drivecartesian", rightRear)

        self.frontLeftmotor.set( leftFront)
        self.frontRightmotor.set(rightFront)
        self.backLeftmotor.set(leftRear)  
        self.backRightmotor.set( rightRear)  


        # TODO: should this just set motor speeds instead of return something
        return (leftFront, rightFront, leftRear, rightRear)

