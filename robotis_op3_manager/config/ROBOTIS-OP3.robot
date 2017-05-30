[ control info ]
control_cycle = 4   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 3000000  | r_hip_yaw

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL  | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | XH-430 | 2.0      | r_hip_yaw    | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 2   | XH-430 | 2.0      | l_hip_yaw    | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 3   | XH-430 | 2.0      | r_hip_roll   | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 4   | XH-430 | 2.0      | l_hip_roll   | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 5   | XH-430 | 2.0      | r_hip_pitch  | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 6   | XH-430 | 2.0      | l_hip_pitch  | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 7   | XH-430 | 2.0      | r_knee       | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 8   | XH-430 | 2.0      | l_knee       | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 9   | XH-430 | 2.0      | r_ank_pitch  | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 10  | XH-430 | 2.0      | l_ank_pitch  | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 11  | XH-430 | 2.0      | r_ank_roll   | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 12  | XH-430 | 2.0      | l_ank_roll   | present_position, present_velocity, present_current
