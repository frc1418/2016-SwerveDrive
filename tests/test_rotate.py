import wpilib

from components.swervemodule import SwerveModule

def test_rotate_positions(hal_data):
    print("Starting Test")
    rotate_motor = wpilib.CANTalon(5)
    drive_motor = wpilib.Talon(1)
    
    module = SwerveModule(drive_motor, rotate_motor)
    
    can = hal_data['CAN'][5]
    
    test = [
        #start angle, goto angle, expected end angle, expected drive magnitude (1 or -1)
        (0, 180, 0, -1),
        (0, 30, 30, 1),
        (0, -75, -75, 1),
        (0, -100, 80, -1),
        (90, -50, 130, -1),
        (-150, 90, -90, -1),
        (-150, 215, -145, 1)
        ]
        
    for start_angle, goto_angle, expected_end_angle, expected_mag in test:
        can['analog_in_position'] = SwerveModule.deg_to_ticks(start_angle)
        print("###", goto_angle, can['analog_in_position'])
        assert rotate_motor.get() == SwerveModule.deg_to_ticks(start_angle)
        
        module.drive(1, goto_angle)
        
        assert can['value']== SwerveModule.deg_to_ticks(expected_end_angle)
        assert drive_motor.get()==expected_mag
    
    