import math
from networktables import NetworkTable


from robotpy_ext.common_drivers import navx

class SwerveDrive:

    def __init__(self, rr_module, rl_module, fr_module, fl_module, navx, gyroCalc = False):
        self.sd = NetworkTable.getTable('SmartDashboard')

        self.modules = [fr_module,fl_module,rl_module,rr_module]
        self.module_speeds = [0,0,0,0]
        self.module_angles = [0,0,0,0]
        
        self.navx = navx

        #Square chasis
        self.set_chasis_deminsions(1, 1)

        self.gyro_calc = gyroCalc

    def set_gyro_calc(self, value):
        '''
        Sets weather or not the gyro centric code will be used

        TODO: Implment gyro centric code
        '''

        self.gyro_calc = value

    def set_chasis_deminsions(self, length, width):
        '''
        Sets the ratio to be used in movement calculations
        '''

        self.length = length
        self.width = width

        self.r = math.sqrt((self.length * self.length)+(self.width + self.width))

    def move(self, fwd, strafe, rcw):
        '''
        Calulates the speed and angle for each wheel given the requested movement

        :param fwd: the requested movement in the Y direction 2D plan
        :param strafe: the requested movement in the X direction of the 2D plan
        :param rcw: the requestest magnatude of the rotational vector of a 2D plan
        '''
        
        if(self.gyro_calc):
            #TODO: verify that gyro needs to be converted
            theta = math.radians(self.navx.yaw())
            
            fwdX = fwd * math.cos(theta)
            fwdY = (-fwd) * math.cos(theta) #TODO: verify and understand why fwd is neg
            strafeX = strafe * math.cos(theta)
            strafeY = strafe * math.sin(theta)
            
            fwd = fwdX + strafeY
            strafe = fwdY + strafeX

        #Velocities per quadrant
        leftY = fwd - (rcw * (self.width / self.r))
        rightY = fwd + (rcw * (self.width / self.r))
        frontX = strafe + (rcw * (self.length / self.r))
        rearX = strafe - (rcw * (self.length / self.r))

        #Calculate the speed and angle for each wheel given the combination of the corrisponding quadrant vectors
        fr_speed = math.sqrt((rightY ** 2) + (frontX ** 2))
        fr_angle = math.degrees(math.atan2(frontX, rightY))

        fl_speed = math.sqrt((leftY ** 2) + (frontX ** 2))
        fl_angle = math.degrees(math.atan2(frontX, leftY))

        rl_speed = math.sqrt((leftY ** 2) + (rearX ** 2))
        rl_angle = math.degrees(math.atan2(rearX, leftY))

        rr_speed = math.sqrt((rightY ** 2) + (rearX ** 2))
        rr_angle = math.degrees(math.atan2(rearX, rightY))

        #Assigns the speeds and angles in lists. MUST BE IN THIS ORDER
        self.module_speeds = [fr_speed, fl_speed, rl_speed, rr_speed]
        self.module_angles = [fr_angle, fl_angle, rl_angle, rr_angle]

        #If any speeds are over 1 it normalizes them.
        max_speed = 0
        for speed in self.module_speeds:
            if speed > max_speed:
                max_speed = speed
        if max_speed > 1:
            for i, speed in enumerate(self.module_speeds):
                self.module_speeds[i] = speed / max_speed

    def doit(self):
        '''
        Sends the speeds and angles to each corrisponding wheel module.
        Excutes the doit in each wheel module.
        '''
        self.update_smartdash()

        for i, module in enumerate(self.modules):
            module.move(self.module_speeds[i], self.module_angles[i])
        self.module_speeds = [0,0,0,0]
        #self.module_angles = [0,0,0,0]

        for module in self.modules:
            module.doit()

    def update_smartdash(self):
        '''
        Pushes some interal variables for debugging.
        '''
        for i, angle in enumerate(self.module_angles):
            self.sd.putNumber("drive/drive/ Angle %s" % i, angle)

        for module in self.modules:
            module.update_smartdash()


