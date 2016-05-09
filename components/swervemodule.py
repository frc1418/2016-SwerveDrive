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
    
    def drive(self, mag, ang):
        self.rotateMotor.set(SwerveModule.deg_to_ticks(ang))
        
        print(mag, ang, SwerveModule.ticks_to_deg(self.rotateMotor.get()), SwerveModule.ticks_to_deg(self.rotateMotor.getSetpoint()), SwerveModule.ticks_to_deg(abs(self.rotateMotor.getSetpoint() - self.rotateMotor.get())))
        if abs(self.rotateMotor.getSetpoint() - self.rotateMotor.get()) > SwerveModule.deg_to_ticks(90):
            
            self.rotateMotor.set(SwerveModule.deg_to_ticks(SwerveModule.put_in_range(ang-180))) #If the angle is more than 90 degrees away, it
                                        #is faster to rotate to directly opposite the angle, and reverse the wheel
            mag*=-1
            
        self.driveMotor.set(mag)
    
    
    @staticmethod
    def deg_to_ticks(angle):
        return round((angle/360) * 1023)
    
    @staticmethod
    def ticks_to_deg(tick):
        return round((tick/1023) * 360)
    
    @staticmethod
    def put_in_range(num):
        x = num
        if x < -180:
            while x < -180:
                x+=360
            print("x", x)
            return x
        elif x > 180:
            while x > 180:
                x-=360
            print("x", x)
            return x
        return x