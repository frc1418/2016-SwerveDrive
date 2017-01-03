
import math

from networktables.util import ntproperty

import pyfrc

class PhysicsEngine:

    # Transmit data to robot via NetworkTables
    target_present = ntproperty('/components/autoaim/present', False)
    target_angle = ntproperty('/components/autoaim/target_angle', 0)
    target_height = ntproperty('/components/autoaim/target_height', 0)

    camera_enabled = ntproperty('/camera/enabled', False)

    camera_update_rate = 1/15.0
    target_location = (0, 16)

    def __init__(self, controller):
        self.controller = controller

        # keep track of the tote encoder position

        # keep track of the can encoder position

        self.controller.add_device_gyro_channel('navxmxp_spi_4_angle')

        self.last_cam_update = -10

    def update_sim(self, hal_data, now, tm_diff):
        # Simulate the arm


        # Simulate the drivetrain
        lf_motor = -hal_data['PWM'][0]['value']/1023
        lr_motor = -hal_data['CAN'][1]['value']/1023
        rf_motor = -hal_data['CAN'][2]['value']/1023
        rr_motor = -hal_data['CAN'][3]['value']/1023

        fwd, rcw = four_motor_drivetrain(lr_motor, rr_motor, lf_motor, rf_motor, speed=5)
        if abs(fwd) > 0.1:
            rcw += -(0.2*tm_diff)

        self.controller.drive(fwd, rcw, tm_diff)

        # Simulate the camera approaching the tower
        # -> this is a very simple approximation, should be good enough
        # -> calculation updated at 15hz
        if self.camera_enabled and now - self.last_cam_update > self.camera_update_rate:

            x, y, angle = self.controller.get_position()

            tx, ty = self.target_location

            dx = tx - x
            dy = ty - y

            distance = math.hypot(dx, dy)

            target_present = False

            if distance > 6 and distance < 17:
                # determine the absolute angle
                target_angle = math.atan2(dy, dx)
                angle = ((angle + math.pi) % (math.pi*2)) - math.pi

                # Calculate the offset, if its within 30 degrees then
                # the robot can 'see' it
                offset = math.degrees(target_angle - angle)
                if abs(offset) < 30:
                    target_present = True

                    # target 'height' is a number between -18 and 18, where
                    # the value is related to the distance away. -11 is ideal.

                    self.target_angle = offset
                    self.target_height = -(-(distance*3)+30)

            self.target_present = target_present
            self.last_cam_update = now


