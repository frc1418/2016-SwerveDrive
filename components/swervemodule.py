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
        target_angle = ang
        
        current_angle = SwerveModule.ticks_to_deg(self.rotateMotor.get())
        
        #target_angle = SwerveModule.put_in_range()
        print("Target", target_angle)
        
        if target_angle - current_angle > 90:
            print("Offset more than 90")
            target_angle-=180
            mag*=-1
        print("New Target", target_angle)
        target_angle = SwerveModule.bind(target_angle)
        print("New Target", target_angle)
        #self.rotateMotor.set(SwerveModule.deg_to_ticks(ang))
        
        #print(mag, ang, SwerveModule.ticks_to_deg(self.rotateMotor.get()), SwerveModule.ticks_to_deg(self.rotateMotor.getSetpoint()), SwerveModule.ticks_to_deg(abs(self.rotateMotor.getSetpoint() - self.rotateMotor.get())))
        #if abs(target_angle - current_angle) > 90:
        #    
        #    target_angle = (SwerveModule.put_in_range(target_angle-180)) #If the angle is more than 90 degrees away, it
        #                                #is faster to rotate to directly opposite the angle, and reverse the wheel
        #    mag*=-1
        print(target_angle, SwerveModule.deg_to_ticks(target_angle))
        self.driveMotor.set(mag)
        self.rotateMotor.set(SwerveModule.deg_to_ticks(target_angle))
    
    
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
        diff = high - low
        return (((val - low) % diff) + low)
    