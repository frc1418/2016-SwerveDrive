import wpilib
from networktables import NetworkTable

import math

MAX_VOLTAGE = 5
MAX_TICK = 4050
MAX_DEG = 360

class SwerveModule:

    def __init__(self, driveMotor, rotateMotor, encoderPort, SDPrefix="SwerveModule", zero=0.0, inverted=False, allow_reverse=False, debugging=False):
        '''
        Swerve drive module was written for a swerve drive train that uses absolute encoders for tracking wheel rotation.  
        
        :param driveMotor: Motor object
        :param rotateMotor: Motor object
        
        :param encoderPort: analog in port number of Absolute Encoder
        
        :param SDPrefix: a string used to differentiate modules when debugging
        :param zero: The default zero for the encoder
        :param inverted: boolean to invert the wheel rotation
        
        :param allow_reverse: If true allows wheels to spin backwards instead of rotating
        
        :param debugging: Prints more NT values if True
        '''

        #SmartDashboard
        self.sd = NetworkTable.getTable('SmartDashboard')
        self.sd_prefix = SDPrefix

        #Motors
        self.driveMotor = driveMotor
        self.driveMotor.setInverted(inverted)
        self.rotateMotor = rotateMotor
        
        self.requested_voltage = 0
        self.requested_speed = 0
        
        #Encoder
        self.encoder = wpilib.AnalogInput(encoderPort)
        self.encoder_zero = zero

        #PID
        self.pid_controller = wpilib.PIDController(1.2, 0.0, 0.0, self.encoder, self.rotateMotor)
        self.pid_controller.setContinuous()
        self.pid_controller.setInputRange(0.0, 5.0)
        self.pid_controller.enable()

        #State variables
        self.allow_reverse = allow_reverse
        self.debugging = debugging

    def get_voltage(self):
        return self.encoder.getVoltage() - self.encoder_zero
    
    @staticmethod
    def voltage_to_degrees(voltage):
        '''
        Converts a given voltage value to degrees

        :param voltage: a voltage value between 0 and 5
        '''

        deg = (voltage/5)*360
        deg = deg

        if deg < 0:
            deg = 360+deg;

        return deg

    @staticmethod
    def voltage_to_tick(voltage):
        '''
        Converts a given voltage value to tick

        :param voltage: a voltage value between 0 and 5
        '''

        return (voltage/5)*4050

    @staticmethod
    def degree_to_voltage(degree):
        '''
        Converts a given degree to voltage

        :param degree: a degree value between 0 and 360
        '''

        return (degree/360)*5
    
    @staticmethod
    def tick_to_voltage(tick):
        '''
        Converts a given tick to voltage

        :param tick: a tick value between 0 and 4050
        '''
        
        return (tick/4050)*5

    def zero_encoder(self):
        '''
        Sets the zero to the current voltage output
        '''

        self.encoder_zero = self.encoder.getVoltage()
        
    def set_allow_reverse(self, value):
        '''
        Sets the ability for the wheels to add 180deg to the requested rotation
        and invert speed rather than rotating over 90deg.
        '''
        self.allow_reverse = value
        
    def set_debugging(self, value):
        '''
        Sets if the update_smartdash function will output more vars.
        '''
        self.debugging = value

    def _set_deg(self, value):
        '''
        Rounds the value to within 360. Sets the requested rotate position (requested voltage).
        Intended to be used only by the move function.
        '''
        self.sd.putNumber('drive/%s/ Requested D' % self.sd_prefix, value)
        self.requested_voltage = ((self.degree_to_voltage(value)+self.encoder_zero) % 5)

    def move(self, speed, deg):
        '''
        Sets the requested speed and rotation of passed
        '''
        
        deg = deg % 360 #Prevents values past 360
        
        if self.allow_reverse:
            if abs( deg - self.voltage_to_degrees(self.get_voltage()) ) > 90:
                speed *= -1
                deg += 180
                
                deg = deg % 360 

        self.requested_speed = speed
        self._set_deg(deg)

    def doit(self):
        '''
        Uses the PID controller to get closer to the requested position.
        Sets the speed requested of the drive motor.

        Should be called every robot iteration/loop.
        '''
        self.pid_controller.setSetpoint(self.requested_voltage)
        self.driveMotor.set(self.requested_speed)
        self.update_smartdash()

    def update_smartdash(self):
        '''
        Outputs a bunch on internal variables for debugging purposes.
        '''
        
        self.sd.putNumber("drive/%s/Degrees" % self.sd_prefix, self.voltage_to_degrees(self.get_voltage()))
        
        if self.debugging:
            self.sd.putNumber("drive/%s/Requested Voltage" % self.sd_prefix, self.requested_voltage)
            self.sd.putNumber("drive/%s/Requested Speed" % self.sd_prefix, self.requested_speed)
            self.sd.putNumber("drive/%s/Voltage" % self.sd_prefix, self.encoder.getVoltage()) #DO NOT USE self.get_voltage() here
            self.sd.putNumber("drive/%s/Zero" % self.sd_prefix, self.encoder_zero)
    
            self.sd.putNumber("drive/%s/PID" % self.sd_prefix, self.pid_controller.get())
            
            self.sd.putBoolean("drive/%s/Allow Reverse" % self.sd_prefix, self.allow_reverse)
