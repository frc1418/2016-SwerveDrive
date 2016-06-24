
import math

from robotpy_ext.common_drivers import navx

class SwerveDrive:
    
    def __init__(self, rr_module, rl_module, fr_module, fl_module, navx, gyroCalc = False):
        
        self.modules = [fl_module,fr_module,rl_module,rr_module]
        self.module_speeds = [0,0,0,0]
        self.module_angles = [0,0,0,0]
        
        self.navx = navx
        
        #Square chasis
        self.set_chasis_deminsions(1, 1)
        
        self.gyro_calc = gyroCalc
    
    def set_gyro_calc(self, value):
        self.gyro_calc = value
    
    def set_chasis_deminsions(self, length, width):
        self.length = length
        self.width = width
        
        self.r = math.sqrt((self.length * self.length)+(self.width + self.width))
    
    def move(self, fwd, str, rcw):
        #Velocities per quadrant
        leftY = fwd + (rcw * (self.width / self.r))
        rightY = fwd - (rcw * (self.width / self.r))
        frontX = str + (rcw * (self.length / self.r))
        rearX = str - (rcw * (self.length / self.r))
        
        fr_speed = math.sqrt((rightY ** 2) + (frontX ** 2))
        fr_angle = math.degrees(math.atan2(rightY, frontX))
        
        fl_speed = math.sqrt((leftY ** 2) + (frontX ** 2))
        fl_angle = math.degrees(math.atan2(leftY, frontX))
        
        rl_speed = math.sqrt((leftY ** 2) + (rearX ** 2))
        rl_angle = math.degrees(math.atan2(leftY, rearX))
        
        rr_speed = math.sqrt((rightY ** 2) + (rearX ** 2))
        rr_angle = math.degrees(math.atan2(rightY, rearX))
        
        self.module_speeds = [fr_speed, fl_speed, rl_speed, rr_speed]
        self.module_angles = [fr_angle, fl_angle, rl_angle, rr_angle]
        
        max_speed = 0
        for speed in self.module_speeds:
            if speed > max_speed:
                max_speed = speed
        
        if max_speed > 1:
            for i, speed in enumerate(self.module_speeds):
                self.module_speeds[i] = speed / max_speed
    
    def doit(self):
        for i, module in enumerate(self.modules):
            module.move(self.module_speeds[i], self.module_angles[i])
        self.module_speeds = [0,0,0,0]
        self.module_angles = [0,0,0,0]
        
        for module in self.modules:
            module.doit()
    
    def update_smartdash(self):
        for module in self.modules:
            module.update_smartdash()
        
        