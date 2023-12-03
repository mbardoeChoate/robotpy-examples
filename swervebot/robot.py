# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband

from drivetrain import Drivetrain


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.XboxController(0)
        self.swerve = Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xSpeedLimiter = SlewRateLimiter(3)
        self.ySpeedLimiter = SlewRateLimiter(3)
        self.rotLimiter = SlewRateLimiter(3)

    def autonomousPeriodic(self):
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self):
        self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool):
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.

        xSpeed = (
            -self.xSpeedLimiter.calculate(
                applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * Drivetrain.maxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.

        ySpeed = (
            -self.ySpeedLimiter.calculate(
                applyDeadband(self.controller.getLeftY(), 0.02)
            )
            * Drivetrain.maxSpeed
        )
        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(applyDeadband(self.controller.getLeftX(), 0.02))
            * Drivetrain.maxAngularSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative)


if __name__ == "__main__":
    wpilib.run(MyRobot)
