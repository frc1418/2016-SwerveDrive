import wpilib
from networktables import NetworkTable

from components.swervemodule import SwerveModule

class MyRobot(wpilib.SampleRobot):
    
    def robotInit(self):
        
        self.sd = NetworkTable.getTable('SmartDashboard')
        wpilib.PIDController
        self.encoders = [];
        
        #driveMotorPort, rotateMotorPort, encoderPort
        self.rr_module = SwerveModule(wpilib.VictorSP(2),wpilib.Talon(3),3, SDPrefix="RR Module", zero=0.83)
        self.lr_module = SwerveModule(wpilib.VictorSP(7),wpilib.VictorSP(6),1, SDPrefix="LR Module", zero=2.24)
        self.lf_module = SwerveModule(wpilib.VictorSP(4),wpilib.VictorSP(5),0, SDPrefix="LF Module", zero=3.18)
        self.rf_module = SwerveModule(wpilib.VictorSP(1),wpilib.VictorSP(0),2, SDPrefix="RF Module", zero=2.16)
        
        self.turret_motor = wpilib.Talon(8)
        
        '''
        self.lr_encoder = wpilib.AnalogInput(1)
        self.lr_rotate_motor = wpilib.VictorSP(7)
        self.lr_move_motor = wpilib.VictorSP(6)
        
        self.rf_encoder = wpilib.AnalogInput(2)
        self.lf_rotate_motor = wpilib.VictorSP(5)
        self.lf_move_motor = wpilib.VictorSP(4)
        
        self.lf_encoder = wpilib.AnalogInput(0)
        self.lf_rotate_motor = wpilib.VictorSP(0)
        self.lf_move_motor = wpilib.VictorSP(1)
        '''
        
        self.components = {
            'lf_module': self.lf_module,
            'rf_module': self.rf_module,
            'lr_module': self.lr_module,
            'rr_module': self.rr_module,
        }
        
        self.joystick1 = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)
        self.wheel_rotation = 0
        
        '''
        for i in range(0,4):
            self.encoders.append(wpilib.AnalogInput(i))'''
    
    def disabled(self):
        while not self.isEnabled():
            for component in self.components.values():
                component.update_smartdash()
            
            wpilib.Timer.delay(0.1)
        
    def operatorControl(self):
        while self.isOperatorControl() and self.isEnabled():
            '''
            for i, encoder in enumerate(self.encoders):
                self.sd.putNumber('Encoder #%s' % i, encoder.getValue())
                self.sd.putNumber('Encoder Angle #%s' % i, self.tickToDeg(encoder.getValue()))
                self.sd.putNumber('Encoder Voltage #%s' % i, encoder.getVoltage())
            '''
            
            self.wheel_rotation = self.joystick1.getAxis(0)*180;
            
            self.turret_motor.setSpeed(self.joystick2.getAxis(0))
            
            self.rr_module.set_deg(self.wheel_rotation)
            self.lr_module.set_deg(self.wheel_rotation)
            self.rf_module.set_deg(self.wheel_rotation)
            self.lf_module.set_deg(self.wheel_rotation)
            
            
            self.update()
            wpilib.Timer.delay(0.005)
    
    def update (self):
        for component in self.components.values():
            component.doit()
        

if __name__ == '__main__':
    wpilib.run(MyRobot)