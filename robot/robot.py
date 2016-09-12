import wpilib
from networktables import NetworkTable

from components.swervemodule import SwerveModule
from components.swervedrive import SwerveDrive

from robotpy_ext.common_drivers import navx

class MyRobot(wpilib.SampleRobot):

    def robotInit(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

        self.navX = navx.AHRS.create_spi()

        self.turret_motor = wpilib.Talon(8)

        self.joystick1 = wpilib.Joystick(1)
        self.joystick2 = wpilib.Joystick(0)

        #Initalization of each indvidual wheel module FORMAT:(driveMotor, rotateMotor, encoderPort, Prefix for SmartDash vars, inverted)
        self.rr_module = SwerveModule(wpilib.VictorSP(2),wpilib.Talon(3),3, SDPrefix="RR Module", zero=3.25, inverted=True)
        self.rl_module = SwerveModule(wpilib.VictorSP(7),wpilib.VictorSP(6),1, SDPrefix="RL Module", zero=2.18, inverted=True)
        self.fl_module = SwerveModule(wpilib.VictorSP(4),wpilib.VictorSP(5),0, SDPrefix="FL Module", zero=3.19, inverted=True)
        self.fr_module = SwerveModule(wpilib.VictorSP(1),wpilib.VictorSP(0),2, SDPrefix="FR Module", zero=2.15, inverted=True)

        #Initalization of the SwerveDrive class that will handle movement calulations
        self.drive = SwerveDrive(self.rr_module, self.rl_module, self.fr_module, self.fl_module, self.navX)
        
        #Creation of components list to iterate over during update phase
        self.components = {
            'swerve_drive': self.drive
        }

    def disabled(self):
        while not self.isEnabled():
            for component in self.components.values():
                component.update_smartdash()

            wpilib.Timer.delay(0.1)

    def operatorControl(self):
        while self.isOperatorControl() and self.isEnabled():

            #Passing joystick axis values to drive function
            self.drive.move(self.joystick1.getAxis(1)*-1, self.joystick1.getAxis(0)*-1, self.joystick2.getAxis(0)*-1)

            if self.joystick1.getRawButton(4):
                self.turret_motor.set(-self.joystick1.getAxis(2))
            elif self.joystick1.getRawButton(5):
                self.turret_motor.set(self.joystick1.getAxis(2))
            else:
                self.turret_motor.set(0)

            self.update()
            wpilib.Timer.delay(0.005)

    def update (self):
        for component in self.components.values():
            component.doit()


if __name__ == '__main__':
    wpilib.run(MyRobot)
