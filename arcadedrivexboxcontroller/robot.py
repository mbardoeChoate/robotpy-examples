#!/usr/bin/env python3
"""
    This is a demo program showing the use of the DifferentialDrive class,
    specifically it contains the code necessary to operate a robot with
    split arcade drive with a single Xbox controller
"""

import wpilib
import wpilib.drive


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # create motor controller objects
        m_left = wpilib.Talon(0)
        m_right = wpilib.Talon(1)

        # invert the right side motors
        m_right.setInverted(True)

        # object that handles basic drive operations
        self.myRobot = wpilib.drive.DifferentialDrive(m_left, m_right)
        self.myRobot.setExpiration(0.1)

        # X box controller
        self.controller = wpilib.XboxController(0)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.myRobot.arcadeDrive(
            -self.controller.getLeftY(), -self.controller.getRightY()
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)
