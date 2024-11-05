# from asyncio import constants
# import pyfrc.mains
# import pyfrc.physics.drivetrains
# import pyfrc.physics.motor_cfgs
# import pyfrc.util
from re import S
import wpilib
from wpilib import SmartDashboard, shuffleboard
from wpilib.simulation import (PWMSim, AnalogGyroSim,)
# from wpilib.drive import MecanumDrive
import wpilib.simulation

from wpimath.kinematics import (MecanumDriveKinematics,
                                # MecanumDriveKinematicsBase,
                                # MecanumDriveOdometry,
                                # MecanumDriveOdometryBase,
                                # MecanumDriveWheelPositions,
                                MecanumDriveWheelSpeeds,
                                DifferentialDriveKinematics,
                                DifferentialDriveOdometry,  
                                DifferentialDriveWheelSpeeds,
                                )
from wpimath.geometry import (Pose2d, Rotation2d, Translation2d)
# from wpimath.estimator import MecanumDrivePoseEstimator 
# from robotpy_ext.common_drivers import navx
from constants import (DriveConstant, OIConstant)
from phoenix6.unmanaged import feed_enable

from pyfrc.physics.drivetrains import FourMotorDrivetrain
from pyfrc.physics.units import units


class PhysicsEngine:
    def __init__(self, physics_controller, robot: "MyRobot"): # type: ignore
        self.physics_controller = physics_controller
        self.robot = robot
        self.robotDrive = robot.robotDrive

        # Initialize motor controllers
        self.right_invert_YN = robot.robotDrive.right_invert_YN

        # useing phoenix5 sim collection off of our actual motors
        self.frontLeftMotor = robot.robotDrive.frontLeftMotor.getSimCollection()
        self.frontRightMotor = robot.robotDrive.frontRightMotor.getSimCollection()
        # self.frontRightMotor.setInverted(self.right_invert_YN) sim colletion has no set inverted
        self.backLeftMotor = robot.robotDrive.backLeftMotor.getSimCollection()
        self.backRightMotor = robot.robotDrive.backRightMotor.getSimCollection()
        # self.backRightMotor.setInverted(self.right_invert_YN) sim colletion has no set inverted

        # Initialize the gyro
        self.simGyro = AnalogGyroSim(1)#navx.AHRS.create_spi()
        self.simGyro.setAngle(self.robot.gyro.getAngle())
        
        

        #initialize the Xbox conroller
        self.Drivercontroller = wpilib.XboxController(OIConstant.kDriver1ControllerPort)

    def update_sim(self, now, tm_diff):
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(0.020 * 2)
         


        # Update motor voltages
        frontLeftMotor_speed = self.frontLeftMotor.getMotorOutputLeadVoltage() /12 #assuming 12 volts for now (could sim the battery :)
        SmartDashboard.putNumber("frontLeftMotor_SIM getmotorOutput Volts by 12Volts", frontLeftMotor_speed)
        frontRightMotor_speed = self.frontRightMotor.getMotorOutputLeadVoltage() / (12*(1-2*self.right_invert_YN))
        SmartDashboard.putNumber("frontRightMotor_SIM getmotorOutput Volts by 12Volts", frontRightMotor_speed)
        backLeftMotor_speed = self.backLeftMotor.getMotorOutputLeadVoltage() /12
        SmartDashboard.putNumber("backLeftMotor_SIM getmotorOutput Volts by 12Volts", backLeftMotor_speed)
        backRightMotor_speed = self.backRightMotor.getMotorOutputLeadVoltage() / (12*(1-2*self.right_invert_YN))
        SmartDashboard.putNumber("backRightMotor_SIM getmotorOutput Volts by 12Volts", backRightMotor_speed)

        match DriveConstant.kDriveTye:
            case "Mecanum":
                wheel_speeds = MecanumDriveWheelSpeeds(
                    frontLeftMotor_speed,
                    frontRightMotor_speed,
                    backLeftMotor_speed,
                    backRightMotor_speed
                )
                # Create an odometry object
                drivetrain_kinematics = MecanumDriveKinematics(
                        Translation2d(DriveConstant.kWheelBase / 2, DriveConstant.kTrackWidth / 2),
                        Translation2d(DriveConstant.kWheelBase / 2, -DriveConstant.kTrackWidth / 2),
                        Translation2d(-DriveConstant.kWheelBase / 2, DriveConstant.kTrackWidth / 2),
                        Translation2d(-DriveConstant.kWheelBase / 2, -DriveConstant.kTrackWidth / 2)
                    )
                chassis_speeds = drivetrain_kinematics.toChassisSpeeds(wheel_speeds)
                # Update the physics controller with the new state
                self.physics_controller.drive(chassis_speeds, tm_diff)
            case "Tank":
                # TODO: FIX!!!!!!!!!!!!!!!

                wheel_speeds = DifferentialDriveWheelSpeeds(
                    frontLeftMotor_speed,
                    frontRightMotor_speed
                )
                # Create an odometry object
                drivetrain_kinematics = DifferentialDriveKinematics(DriveConstant.kTrackWidth)
                chassis_speeds = drivetrain_kinematics.toChassisSpeeds(wheel_speeds)
                # Update the physics controller with the new state
                self.physics_controller.drive(chassis_speeds, tm_diff)
            case "Xdrive":
                # TODO: FIX!!!!!!!!!!!!!!!
                
                wheel_speeds = MecanumDriveWheelSpeeds(
                    frontLeftMotor_speed,
                    frontRightMotor_speed,
                    backLeftMotor_speed,
                    backRightMotor_speed,
                )
                # Create an odometry object for X-drive (similar to Mecanum)
                drivetrain_kinematics = MecanumDriveKinematics(
                    Translation2d(DriveConstant.kWheelBase / 2, DriveConstant.kTrackWidth / 2),
                    Translation2d(DriveConstant.kWheelBase / 2, -DriveConstant.kTrackWidth / 2),
                    Translation2d(-DriveConstant.kWheelBase / 2, DriveConstant.kTrackWidth / 2),
                    Translation2d(-DriveConstant.kWheelBase / 2, -DriveConstant.kTrackWidth / 2)
                )
                chassis_speeds = drivetrain_kinematics.toChassisSpeeds(wheel_speeds)
                # Update the physics controller with the new state
                self.physics_controller.drive(chassis_speeds, tm_diff)

    
        # simulated_gyro =  # chassis_speeds.omega #self.simGyro.getAngle()
        # SmartDashboard.putNumber("Sim Gyro", simulated_gyro)