# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from wpimath.controller import (
    PIDController,
    ProfiledPIDController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.trajectory import TrapezoidProfile
from wpilib import Encoder, PWMMotorController, PWMSparkMax

from drivetrain import Drivetrain

import math


class SwerveModule:
    """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

    :param driveMotorChannel:      PWM output for the drive motor.
    :param turningMotorChannel:    PWM output for the turning motor.
    :param driveEncoderChannelA:   DIO input for the drive encoder channel A
    :param driveEncoderChannelB:   DIO input for the drive encoder channel B
    :param turningEncoderChannelA: DIO input for the turning encoder channel A
    :param turningEncoderChannelB: DIO input for the turning encoder channel B
    """

    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        driveEncoderChannelA: int,
        driveEncoderChannelB: int,
        turningEncoderChannelA: int,
        turningEncoderChannelB: int,
    ):
        self.wheelRadius = 0.0508
        self.EncoderResolution = 4096
        self.moduleMaxAngularVelocity = Drivetrain.maxAngularSpeed
        self.moduleMaxAngularAcceleration = 2 * math.pi  # radians per second squared

        self.driveMotor = PWMSparkMax(driveMotorChannel)
        self.turningMotor = PWMSparkMax(turningMotorChannel)

        self.driveEncoder = Encoder(driveEncoderChannelA, driveEncoderChannelB)
        self.turningEncoder = Encoder(turningEncoderChannelA, turningEncoderChannelB)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = PIDController(1, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = ProfiledPIDController(
            Kp=1,
            Ki=0,
            Kd=0,
            constraints=TrapezoidProfile.Constraints(
                self.moduleMaxAngularVelocity, self.moduleMaxAngularAcceleration
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!

        self.driveFeedForward = SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedForward = SimpleMotorFeedforwardMeters(1, 0.5)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.

        self.driveEncoder.setDistancePerPulse(
            2 * math.pi * self.wheelRadius / self.EncoderResolution
        )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.

        self.turningEncoder.setDistancePerPulse(2 * math.pi / self.EncoderResolution)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :return: The current state of the  module.
        """
        return SwerveModuleState(
            self.driveEncoder.getRate(),
            Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :return: The current position of the module.
        """
        return SwerveModulePosition(
            self.driveEncoder.getDistance(),
            Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param state: The desired state for the module.
        """
        encoderRotation = Rotation2d(self.turningEncoder.getDistance())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = SwerveModuleState.optimize(desiredState, encoderRotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.

        state.speed *= math.cos(encoderRotation.radians() - state.angle.radians())

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), state.speed
        )

        driveFeedForward = self.driveFeedForward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.

        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), state.angle
        )
        turnFeedForward = self.turnFeedForward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedForward)
        self.turningMotor.setVoltage(turnOutput + turnFeedForward)
