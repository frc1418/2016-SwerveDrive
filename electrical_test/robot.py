import wpilib
from networktables import NetworkTable

from components.swervemodule import SwerveModule
from components.swervedrive import SwerveDrive

from robotpy_ext.common_drivers import navx

class MyRobot(wpilib.SampleRobot):
    
    def robotInit(self):
        self.sd = NetworkTable.getTable('SmartDashboard')
        
        self.navX = navx.AHRS.create_spi()
        
        #driveMotorPort, rotateMotorPort, encoderPort
        self.rr_module = SwerveModule(wpilib.VictorSP(2),wpilib.Talon(3),3, SDPrefix="RR Module", zero=3.3, inverted=True)
        self.rl_module = SwerveModule(wpilib.VictorSP(7),wpilib.VictorSP(6),1, SDPrefix="RL Module", zero=4.63)
        self.fl_module = SwerveModule(wpilib.VictorSP(4),wpilib.VictorSP(5),0, SDPrefix="FL Module", zero=0.79)
        self.fr_module = SwerveModule(wpilib.VictorSP(1),wpilib.VictorSP(0),2, SDPrefix="FR Module", zero=2.15, inverted=True)
        
        #rr_module, rl_module, fr_module, fl_module, navx, gyroCalc = False
        self.drive = SwerveDrive(self.rr_module, self.rl_module, self.fr_module, self.fl_module, self.navX)
        
        self.turret_motor = wpilib.Talon(8)
        
        self.components = {
            'swerve_drive': self.drive
        }
        
        self.joystick1 = wpilib.Joystick(1)
        self.joystick2 = wpilib.Joystick(0)
        
    
    def disabled(self):
        while not self.isEnabled():
            for component in self.components.values():
                component.update_smartdash()
            
            wpilib.Timer.delay(0.1)
        
    def operatorControl(self):
        while self.isOperatorControl() and self.isEnabled():
            
            #self.turret_motor.setSpeed(self.joystick2.getAxis(0))
            
            self.drive.move(self.joystick1.getAxis(1)*-1, self.joystick1.getAxis(0)*-1, self.joystick2.getAxis(0)*-1)
            
            self.update()
            wpilib.Timer.delay(0.005)
    
    def update (self):
        for component in self.components.values():
            component.doit()
        

if __name__ == '__main__':
    wpilib.run(MyRobot)