# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from wpimath.geometry import Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)


from swervemodule import SwerveModule

import math


class Drivetrain:
    maxSpeed = 3.0  # 3 meters per second
    maxAngularSpeed = math.pi  # 1/2 rotation per second

    def __init__(self):
        self.frontLeftLocation = Translation2d(0.381, 0.381)
        self.frontRightLocation = Translation2d(0.381, -0.381)
        self.backLeftLocation = Translation2d(-0.381, 0.381)
        self.backRightLocation = Translation2d(-0.381, -0.381)

        self.frontLeft = SwerveModule(1, 2, 0, 1, 2, 3)
        self.frontRight = SwerveModule(3, 4, 4, 5, 6, 7)
        self.backLeft = SwerveModule(5, 6, 8, 9, 10, 11)
        self.backRight = SwerveModule(7, 8, 12, 13, 14, 15)

        self.gyro = wpilib.AnalogGyro(0)

        self.kinematics = SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )
        self.odometry = SwerveDrive4Odometry(
            kinematics=self.kinematics,
            gyroAngle=self.gyro.getRotation2d(),
            modulePositions=[
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ],
        )
        self.gyro.reset()

    def drive(
        self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool = False
    ):
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        """
        if fieldRelative:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                )
            )
        else:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                ChassisSpeeds(xSpeed, ySpeed, rot)
            )

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, self.maxSpeed)
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self):
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition(),
        )
