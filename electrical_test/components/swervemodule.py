import wpilib
from networktables import NetworkTable

import math

MAX_VOLTAGE = 5
MAX_TICK = 4050
MAX_DEG = 360

class SwerveModule:
    
    def __init__(self, driveMotor, rotateMotor, encoderPort, SDPrefix="SwerveModule", zero=0.0, inverted=False):
        '''
        :param driveMotorPort: Motor object
        :param rotateMotorPort: Motor object
        :param encoderPort: analog in port number of Absolute Encoder
        '''
        
        self.sd = NetworkTable.getTable('SmartDashboard')
        self.sd_prefix = SDPrefix
        
        self.driveMotor = driveMotor
        self.driveMotor.setInverted(inverted)
        self.rotateMotor = rotateMotor
        self.encoder = wpilib.AnalogInput(encoderPort)
        
        self.encoder_zero = 0
        
        self.pid_controller = wpilib.PIDController(1.2, 0.0, 0.0, self.encoder, self.rotateMotor)
        #self.pid_controller.setTolerance(5)
        self.pid_controller.setContinuous()
        self.pid_controller.setInputRange(0.0, 5.0)
        self.pid_controller.enable()
        
        self.requested_voltage = 0
        self.requested_speed = 0
        
        self.encoder_zero = zero
    
    def get_degrees(self):
        volt =  (self.encoder.getVoltage() - self.encoder_zero)
        deg = (volt/4050)*360
        deg = deg
        
        if deg < 0:
            deg = 360+deg;
        
        return deg
    
    @staticmethod
    def tick_to_deg(tick):
        deg = (tick/4050)*360
        return deg
    
    @staticmethod
    def deg_to_voltage(deg):
        volt = (deg/360)*5
        return volt
    
    @staticmethod
    def deg_to_tick(deg):
        volt = (deg/360)*4050
        return volt
    
    def zero_encoder(self):
        self.encoder_zero = self.encoder.getVoltage()
    
    def _set_deg(self, value):
        value = value % 360
        self.sd.putNumber('drive/%s/ Requested D' % self.sd_prefix, value)
        self.requested_voltage = ((self.deg_to_voltage(value)+self.encoder_zero) % 5)
    
    def move(self, speed, deg):
        if abs(deg - self.get_degrees()) > 90:
            deg +=180
            speed *= -1
        
        self.requested_speed = speed
        self._set_deg(deg)
    
    def doit(self):
        
        self.pid_controller.setSetpoint(self.requested_voltage)
        self.driveMotor.set(self.requested_speed)
        self.update_smartdash()
    
    def update_smartdash(self):
        self.sd.putNumber("drive/%s/ Requested Voltage" % self.sd_prefix, self.requested_voltage)
        self.sd.putNumber("drive/%s/ Voltage" % self.sd_prefix, self.encoder.getVoltage())
        self.sd.putNumber("drive/%s/ Tick " % self.sd_prefix, self.encoder.getValue())
        self.sd.putNumber("drive/%s/ Zero " % self.sd_prefix, self.encoder_zero)
        self.sd.putNumber("drive/%s/ Degrees" % self.sd_prefix, self.get_degrees())
        
        self.sd.putNumber("drive/%s/ PID GET" % self.sd_prefix, self.pid_controller.get())
        #self.sd.putBoolean("drive/%s/ On Target" % self.sd_prefix, self.pid_controller.onTarget())