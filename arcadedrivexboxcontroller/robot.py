#!/usr/bin/env python3


import wpilib
import wpilib.drive


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing the use of the DifferentialDrive class.
    Runs the motors with split arcade steering and an Xbox controller.
    """

    def robotInit(self):
        self.left = wpilib.Talon(0)
        self.right = wpilib.Talon(1)

        ##
        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead."""
        ##

        self.right.setInverted(True)

        self.robotDrive = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.robotDrive.setExpiration(0.1)

        self.controller = wpilib.XboxController(0)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.robotDrive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        ##
        # Drive with split arcade drive.
        # That means that the Y axis of the left stick moves forward
        # and backward, and the X of the right stick turns left and right.
        ##

        self.robotDrive.arcadeDrive(
            -self.controller.getLeftY(), -self.controller.getRightY()
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)
