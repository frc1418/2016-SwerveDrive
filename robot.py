#!/usr/bin/env python3

import magicbot
from robotpy_ext.common_drivers import navx
import wpilib

from components.drive import Drive
from components.swervemodule import SwerveModule
import pyfrc


class MyRobot(magicbot.MagicRobot):
    
    drive = Drive
    
    def createObjects(self):
        
        # #INITIALIZE JOYSTICKS##
        self.joystick1 = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)
        
        self.lf_module = SwerveModule(wpilib.Talon(0), wpilib.CANTalon(5))
        self.lr_module = SwerveModule(wpilib.Talon(1), wpilib.CANTalon(10))
        self.rf_module = SwerveModule(wpilib.Talon(2), wpilib.CANTalon(15))
        self.rr_module = SwerveModule(wpilib.Talon(3), wpilib.CANTalon(20))
        
        self.drive_modules = [self.lf_module, self.lr_module, self.rf_module, self.rr_module]
        
        self.navX = navx.AHRS.create_spi()
    
    def teleopInit(self):
        pass
    
    def teleopPeriodic(self):
        self.drive.move(self.joystick1.getY(), self.joystick1.getX(), self.self.joystick2.getX())
        
        
if __name__ == '__main__':
    wpilib.run(MyRobot)
