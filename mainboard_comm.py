import serial
from math import *

# Initialize serial
ser = serial.Serial("COM4", 115200)

# Constants
# wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution /
# (2 * PI * wheelRadius * pidControlFrequency)
wheel_speed_to_mainboard_units = 18.75 * 64 / (2 * pi * 0.035 * 60)


def robot_speed(robot_speed_x, robot_speed_y):
    return sqrt(robot_speed_x ** 2 + robot_speed_y ** 2)


def robot_direction_angle(robot_speed_x, robot_speed_y):
    return atan2(robot_speed_y, robot_speed_x)


def wheel_linear_velocity(robot_speed, robot_direction_angle, wheel_angle):
    return robot_speed * cos(robot_direction_angle - wheel_angle)


def wheel_angular_speed_maiboard_units(wheel_linear_velocity, wheel_speed_to_mainboard_units):
    return wheel_linear_velocity * wheel_speed_to_mainboard_units


def get_motor_speeds(robot_speed, direction_angle):
    motors = [0, 0, 0]
    motors[0] = wheel_angular_speed_maiboard_units(wheel_linear_velocity(robot_speed, direction_angle, 120),
                                                   wheel_speed_to_mainboard_units)
    motors[1] = wheel_angular_speed_maiboard_units(wheel_linear_velocity(robot_speed, direction_angle, 0),
                                                   wheel_speed_to_mainboard_units)
    motors[2] = wheel_angular_speed_maiboard_units(wheel_linear_velocity(robot_speed, direction_angle, 240),
                                                   wheel_speed_to_mainboard_units)
    return motors


def send_to_mainboard(motors):
    message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(round(motors[2])) + ":0\n").encode("'utf-8")
    print(message)
    ser.write(message)

    # Read the returned message.
    line = ""
    char = ser.read().decode()
    while char != "\n":
        line += char
        char = ser.read().decode()

    # Print out the returned message and also return it for other usage.
    print(line)
    return line


user_input = ""
while user_input == "":
    # Ask for information
    # ser.write("gs\n".encode('utf-8'))
    # ser.write(b"sd:0:10:0:0\n")
    #
    # line = ""
    # char = ser.read().decode()
    # while char != "\n":
    #     line += char
    #     char = ser.read().decode()
    # print(line)

    motors = get_motor_speeds(0.2, 90)
    send_to_mainboard(motors)

    user_input = input("Press enter..")
