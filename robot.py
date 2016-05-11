#!/usr/bin/env python3

import wpilib
import magicbot
from components.swervemodule import SwerveModule
from components.drive import Drive

class MyRobot(magicbot.MagicRobot):
    
    drive = Drive
    
    
    def createObjects(self):
        
        # #INITIALIZE JOYSTICKS##
        self.joystick1 = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)
        
        self.lf_module = SwerveModule(wpilib.Talon(0), wpilib.Talon(4))
        self.lr_module = SwerveModule(wpilib.Talon(1), wpilib.Talon(5))
        self.rf_module = SwerveModule(wpilib.Talon(2), wpilib.Talon(6))
        self.rr_module = SwerveModule(wpilib.Talon(3), wpilib.Talon(7))
        
        self.drive_modules = [self.lf_module, self.lr_module, self.rf_module, self.rr_module]
        
        self.navX = navx.AHRS.create_spi()
        
if __name__ == '__main__':
    wpilib.run(MyRobot)
