import wpilib

class SwerveModule:
    
    def __init__(self, driveMotor, rotateMotor):
        '''
        :type driveMotor: wpilib.Talon
        :type rotateMotor: wpilib.CANTalon
        '''
        
        self.driveMotor = driveMotor
        self.rotateMotor = rotateMotor
        self.rotateMotor.changeControlMode(wpilib.CANTalon.ControlMode.Position)
        self.rotateMotor.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.AnalogEncoder)
        
        self.magnitude = 0
        self.angle = 0
        
    def drive(self, mag, ang):
        target_angle = ang
        
        current_angle = SwerveModule.ticks_to_deg(self.rotateMotor.get()) #Get current angle of robot
        
        
        if abs(target_angle - current_angle) > 90: #If error is > 90
            target_angle-=180 #target angle is directly opposite previous target angle
            mag*=-1 #Reverse wheel direction
        target_angle = SwerveModule.bind(target_angle) #Put in range -180, 180
        
    
    def zero(self):
        self.angle = 0
    
    def execute(self):
        self.rotateMotor.set(SwerveModule.deg_to_ticks(self.angle))
        self.driveMotor.set(self.magnitude)
    
    @staticmethod
    def deg_to_ticks(angle):
        return round((angle/360) * 1023)
    
    @staticmethod
    def ticks_to_deg(tick):
        deg =  round((tick/1023) * 360)
        deg = deg % 360
        if deg < -180:
            deg +=360 
        return deg
    
    @staticmethod
    def bind(val, low = -180, high = 180):
        if val == -180:
            return 180
        diff = high - low
        return (((val - low) % diff) + low)
    