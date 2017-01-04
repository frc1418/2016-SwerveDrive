import math

from robotpy_ext.common_drivers import navx

from components.swervemodule import  SwerveModule


L = 27
W = 27
R = math.sqrt(L * L + W * W)

class Drive:

    navX = navx.AHRS
    drive_modules = list

    def __init__(self):
        '''
        :type lf_module: SwerveModule
        :type lr_module: SwerveModule
        :type rf_module: SwerveModule
        :type rr_module: SwerveModule
        '''

        pass

    def on_enable(self):
        for module in self.modules:
            module.zero()

    def return_gyro_angle(self):
        return self.navX.getYaw()

    def reset_gyro_angle(self):
        self.navX.reset()

    def move(self, fwd, sid, rot):
        gyroAngle = self.return_gyro_angle()
        FWD = fwd * math.cos(math.radians(gyroAngle)) + sid(math.sin(math.radians(gyroAngle)))
        STR = -fwd * math.sin(math.radians(gyroAngle)) + sid(math.cos(math.radians(gyroAngle)))
        RCW = rot

        A = STR - RCW * (L / R)
        B = STR + RCW * (L / R)
        C = FWD - RCW * (W / R)
        D = FWD + RCW * (W / R)

        ws1 = math.sqrt(B ** 2 + C ** 2)
        wa1 = math.atan2(B, C) * 180 / math.pi;

        ws2 = math.sqrt(B ** 2 + D ** 2)
        wa2 = math.atan2(B, D) * 180 / math.pi

        ws3 = math.sqrt(A ** 2 + D ** 2);
        wa3 = math.atan2(A, D) * 180 / math.pi;

        ws4 = math.sqrt(A ** 2 + C ** 2)
        wa4 = math.atan2(A, C) * 180 / math.pi;

        self.wheel_speeds = [ws2, ws3, ws1, ws4]
        self.wheel_angles = [wa2, wa3, wa1, wa4]

    def execute(self):
        for i, module in enumerate(self.drive_modules):
            '''
            :type module: SwerveModule
            '''
            module.drive(self.wheel_speeds[i], self.wheel_angles[i])



