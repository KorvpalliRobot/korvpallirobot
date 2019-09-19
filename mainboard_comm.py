import serial
from math import *
import keyboard
import time
from serial.tools import list_ports

# Initialize serial
def get_mainboard_serial_port() :
    ports = list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200)
            return ser
        except:
            continue
    raise Exception("Could not find suitable or any USB ports.")


ser = get_mainboard_serial_port()

# Constants
# wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution /
# (2 * PI * wheelRadius * pidControlFrequency)
wheel_speed_to_mainboard_units = 18.75 * 64 / (2 * pi * 0.035 * 60)


# print("Wheel speed to mainboard units:", wheel_speed_to_mainboard_units)


def robot_speed(robot_speed_x, robot_speed_y):
    return sqrt(robot_speed_x ** 2 + robot_speed_y ** 2)


def robot_direction_angle(robot_speed_x, robot_speed_y):
    return atan2(robot_speed_y, robot_speed_x)


def wheel_linear_velocity(robot_speed, robot_direction_angle, wheel_angle):
    # print("Robot_speed:", robot_speed, "; angle:", robot_direction_angle , "; cos:", cos(robot_direction_angle - wheel_angle))
    return robot_speed * cos(robot_direction_angle - radians(wheel_angle))


def wheel_angular_speed_mainboard_units(wheel_linear_velocity, wheel_speed_to_mainboard_units):
    # print("Wheel linear velocity:", wheel_linear_velocity)
    return wheel_linear_velocity * wheel_speed_to_mainboard_units


def get_motor_speeds(robot_speed_x, robot_speed_y, rotation):
    rbt_spd = robot_speed(robot_speed_x, robot_speed_y)
    dir_ang = robot_direction_angle(robot_speed_x, robot_speed_y)
    print("Speed:", rbt_spd, "; angle:", degrees(dir_ang))

    rot_constant = wheel_speed_to_mainboard_units
    rot = rotation * rot_constant

    motors = [0, 0, 0]
    motors[0] = wheel_angular_speed_mainboard_units(wheel_linear_velocity(rbt_spd, dir_ang, 120),
                                                    wheel_speed_to_mainboard_units) + rot
    motors[1] = wheel_angular_speed_mainboard_units(wheel_linear_velocity(rbt_spd, dir_ang, 0),
                                                    wheel_speed_to_mainboard_units) + rot
    motors[2] = wheel_angular_speed_mainboard_units(wheel_linear_velocity(rbt_spd, dir_ang, 240),
                                                    wheel_speed_to_mainboard_units) + rot
    return motors


def send_to_mainboard(motors):
    message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
        round(motors[2])) + ":0\n").encode("'utf-8")
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


def send_to_mainboard_debug(motors):
    message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
        round(motors[2])) + ":200q\n").encode("'utf-8")
    print(message)


def rotate_to_ball(q_ball, q_basket):
    ball_x = 0
    hysteresis = 8
    img_center = 320
    speed = 0.05

    while True:
        #if not q_ball.empty:
        ball_x = q_ball.get()

        print("BALL_X =", ball_x)
        if ball_x > img_center + hysteresis:
            motors = get_motor_speeds(0, 0, speed)
            send_to_mainboard(motors)
        elif ball_x < img_center - hysteresis:
            motors = get_motor_speeds(0, 0, 0-speed)
            send_to_mainboard(motors)
        else:
            motors = get_motor_speeds(0, 0, 0)
            send_to_mainboard(motors)


def main():
    user_input = ""
    while user_input == "":
        sleep = 0.5
        x_speed = 0
        y_speed = 0
        speed = 0.2

        presses = 0
        presses_max = 2

        while True:
            if presses >= presses_max:
                break
            if keyboard.is_pressed('a') and presses < presses_max:
                print("A")
                x_speed += speed
                presses += 1
            if keyboard.is_pressed('s') and presses < presses_max:
                print("S")
                y_speed += speed
                presses += 1
            if keyboard.is_pressed('w') and presses < presses_max:
                print("W")
                y_speed -= speed
                presses += 1
            if keyboard.is_pressed('d') and presses < presses_max:
                print("D")
                x_speed -= speed
                presses += 1

        # get_motor_speeds takes the X and Y speed as arguments, as well as rotation speed
        # (positive numbers as clockwise and negative as counterclockwise)
        motors = get_motor_speeds(x_speed, y_speed, 0)
        send_to_mainboard(motors)

        time.sleep(sleep)
        # user_input = input("Press enter..")

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
