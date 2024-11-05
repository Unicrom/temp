#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import phoenix5
import phoenix5.sensors
import wpilib
from wpilib import (SmartDashboard, Field2d)
from wpilib.shuffleboard import (Shuffleboard, BuiltInWidgets)
import wpilib.drive
# import rev
# from pyfrc.physics.drivetrains import MecanumDrivetrain
import commands2
from commands2 import CommandScheduler
import wpimath
from wpimath.geometry import Translation2d, Rotation2d
import pathplannerlib

from constants import (DriveConstant,
                       OIConstant,
                       )
# import phoenix5
# import math
import constants
from subsystems.drivetrain import DriveTrain
from subsystems.cannon import Cannon



#region Helper functions

#endregion Helper functions
class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        CommandScheduler.getInstance().run()
        self.timer = wpilib.Timer()
        
        #region tie ins
        self.robotDrive = DriveTrain()
        self.cannon = Cannon()


        self.driverController = commands2.button.CommandXboxController(
            OIConstant.kDriver1ControllerPort)
        # Configure the button bindings
        self.ConfigureButtonBindings()

        #init a pigon2 gyro
        self.gyro = phoenix5.sensors.WPI_Pigeon2(DriveConstant.kImuPort)
        self.gyro.configFactoryDefault()
        

        #endregion tie ins

        self.robotDrive.setDefaultCommand(commands2.cmd.run(lambda: self.robotDrive.driveWithJoystick(self.driverController)
                                                            , self.robotDrive))
        self.cannon.setDefaultCommand(commands2.cmd.run(lambda: self.cannon.stop(), self.cannon))

        #region PathPlanner
        self.pathPlanner = pathplannerlib.PathPlanner(self.robotDrive)
        #endregion PathPlanner

        #region SmartDashboard init

        SmartDashboard.putData(CommandScheduler.getInstance())
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field) #end up viewing in Glass
        #endregion SmartDashBoard init


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""


    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        # if self.autonomousCommand is not None:
        #     self.autonomousCommand.cancel()


    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        commands2.cmd.runOnce(lambda: self.robotDrive.halt())
    def testInit(self):
        """This function is called once each time the robot enters test mode."""
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
    
    def end(self):
        """This function is called ?? ever """
        crash_if_run = 1/0
        self.robotDrive.driveCartesian(0,0,0,0)
        self.cannon.stop() #may want to change thsi to let out air??
        commands2.CommandScheduler.getInstance().cancelAll()

    def ConfigureButtonBindings(self):
        self.driverController.povLeft().onTrue(lambda: self.robotDrive.slowLeft(self.driverController))
        self.driverController.povRight().onTrue(lambda: self.robotDrive.slowRight(self.driverController))

        OnlyFrontLeft = commands2.SequentialCommandGroup(
            commands2.cmd.run(lambda: self.robotDrive.OnlyFrontLeft()).raceWith(
                commands2.WaitCommand(2.2)))
        self.driverController.x().onTrue(OnlyFrontLeft)



        OnlyFrontRight = (commands2.cmd.run(lambda: self.robotDrive.OnlyFrontRight())
                          .raceWith(commands2.WaitCommand(1.2))
                          .andThen(commands2.WaitCommand(0.5))
                          .andThen(commands2.cmd.run(lambda: self.robotDrive.OnlyFrontRight())
                          .raceWith(commands2.WaitCommand(1.2)))
                          )
        self.driverController.y().onTrue(OnlyFrontRight)

        OnlyBackLeft = (commands2.cmd.run(lambda: self.robotDrive.OnlyBackLeft())
                        .raceWith(commands2.WaitCommand(.7))
                        .andThen(commands2.WaitCommand(0.5))
                        .andThen(commands2.cmd.run(lambda: self.robotDrive.OnlyBackLeft())
                        .raceWith(commands2.WaitCommand(.7)))
                        .andThen(commands2.WaitCommand(0.5))
                        .andThen(commands2.cmd.run(lambda: self.robotDrive.OnlyBackLeft())
                        .raceWith(commands2.WaitCommand(.7)))
                        )
        self.driverController.a().onTrue(OnlyBackLeft)

        OnlyBackRight = (commands2.cmd.run(lambda: self.robotDrive.OnlyBackRight())
                         .raceWith(commands2.WaitCommand(.4))
                         .andThen(commands2.WaitCommand(0.35))
                         .andThen(commands2.cmd.run(lambda: self.robotDrive.OnlyBackRight())
                         .raceWith(commands2.WaitCommand(.4)))
                         .andThen(commands2.WaitCommand(0.35))
                         .andThen(commands2.cmd.run(lambda: self.robotDrive.OnlyBackRight())
                         .raceWith(commands2.WaitCommand(.4)))
                         .andThen(commands2.WaitCommand(0.35))
                         .andThen(commands2.cmd.run(lambda: self.robotDrive.OnlyBackRight())
                         .raceWith(commands2.WaitCommand(.4)))
                         )
        self.driverController.b().onTrue(OnlyBackRight)

        fire_cannon = (commands2.cmd.run(lambda: self.cannon.fire()).raceWith(commands2.WaitCommand(0.2)))
        #.andThen(commands2.cmd.run(lambda: self.cannon.stop())))  # TODO: do i need to stop the firing? put it in periodic of cannon
 
        self.driverController.rightBumper().onTrue(fire_cannon)
            

        


        
