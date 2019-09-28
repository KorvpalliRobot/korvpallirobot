import serial
from math import *
import keyboard
import time
from serial.tools import list_ports


# Initialize serial
def get_mainboard_serial_port():
    ports = list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200, timeout=0.01)
            return ser
        except:
            continue
    raise Exception("Could not find suitable or any USB ports.")


ser = get_mainboard_serial_port()
#ser = serial.Serial("COM3", 115200, timeout=0.01)

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
    #print("Speed:", rbt_spd, "; angle:", degrees(dir_ang))

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
    #print(message)
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


# Function for multithread use only. Sends motor speeds to mainboard.
def send(q_motors, stop_event):
    print("Starting mainboard_comm.")
    while True:
        # Check for stop signals
        if stop_event.is_set():
            if ser.is_open:
                ser.close()
            print("Closing mainboard_comm..")
            return

        #print(q_motors.empty())
        # Get speeds for all the motors
        #speeds = [0, 0, 0]
        speeds = q_motors.get()
        #print(speeds)
        motors = get_motor_speeds(speeds[0], speeds[1], speeds[2])
        # Send this information to our mainboard
        send_to_mainboard(motors)


def thrower_motor(speed):
    message = ("d:" + str(round(speed)) + "\n").encode("'utf-8")
    # print(message)
    ser.write(message)
    print(message)


# Game logic main
def game(stop_event, game_event):

    # Basically a state machine corresponding to a flow diagram
    state = "start"

    # Repeat until forced to stop
    while not stop_event.is_set():

        # Check whether we are on autonomous mode
        if game_event.is_set():
            # If we begin our game
            if state == "start":
                # Find the closest ball
                rotate_to_ball()
                # Move to the next state
                state = "drive"

            if state == "drive":
                # Drive to the ball we've found
                drive_to_ball()
                state = "align"

            if state == "align":
                # Align the robot with the ball and the basket
                align_to_basket()
                state = "throw"

            if state == "throw":
                # Throw the ball
                throw_ball()


def drive_to_ball():
    print(0)
    # Basically we have to drive until the ball is of some size


def align_to_basket():
    print(0)


def throw_ball():
    print(0)


# Rotation to ball using a proportional controller
def rotate_to_ball_p(q_ball, q_basket, q_motors, game_event, stop_event):
    ball_x = 0
    img_center = 320
    speed = 0.04
    gain = 0.2
    # When to stop moving (distance from the center)
    offset = 0.015

    # Variable for game state
    state = True

    while True:
        # Check for stop signals
        if stop_event.is_set():
            print("Closing rotate_to_ball..")
            return

        # if not q_ball.empty:
        ball_x = q_ball.get()
        #print("Ball_x:", ball_x)

        # If we calculate the error by NOT using absolute values, we can already determine which way the
        # robot is going to rotate (what sign the value "error" has).
        error = (ball_x - img_center) / img_center
        print("Error:", error)

        # Hysteresis control
        # If error is below a threshold then don't move at all
        if abs(error) <= offset:
            motors = [0, 0, 0]
        else:

            # P-controller algorithm:
            # output = output + gain*error
            #rot_speed = speed + gain * error
            rot_speed = copysign(speed, error) + gain * error
            if abs(rot_speed) < 0.079 :
                rot_speed = error/abs(error) * 0.079
            print(speed)

            # Send info to mainboard
            motors = [0, 0, rot_speed]
            #motors = get_motor_speeds(0, 0, rot_speed)
            #send_to_mainboard(motors)

        # Check to see whether we are on manual control or game logic

        if game_event.is_set():
            #print("To queue:", motors)
            q_motors.put(motors)
            #send_to_mainboard(motors)


# Currently calculates only movement angle.
# Movement values currently would be true if looking at a mirrored image.
def angular_movement():
    ball_x = 0
    # Hysteresis is the "deadzone" of our controller, that is, if the error is +/- hysteresis value,
    # the robot won't move.
    hysteresis = 5

    # The center of our camera image
    img_center = 320

    # Default speed for rotation
    rotation_speed = 0.07
    movement_speed = 1.0

    # Game state
    state = True

    # Main loop
    while True:
        # Check for stop signals
        if stop_event.is_set():
            if ser.is_open:
                ser.close()
            print("Closing rotate_to_ball..")
            return

        # if not q_ball.empty:
        ball_cords = q_ball.get()
        ball_x = img_center - ball_cords[0]
        ball_y = ball_cords[1]

        print("BALL_X =", ball_x)

        # Check to see whether we are on manual control or game logic
        if game_event.is_set():
            # Controller logic:
            # if ball is to our right, rotate right;
            # if it's to our left, rotate left;
            # if it's kind of in the middle, don't do anything (hysteresis)

            x_speed = img_center - ball_x
            y_speed = ball_y
            rda = degrees(robot_direction_angle(x_speed, y_speed))
            print(rda)
            wheel_120 = wheel_linear_velocity(movement_speed, rda, 120)
            wheel_0 = wheel_linear_velocity(movement_speed, rda, 0)
            wheel_240 = wheel_linear_velocity(movement_speed, rda, 240)
            motors = [wheel_120, wheel_0, wheel_240]

            q_motors.put(motors)


# Rotation and movement towards ball using a bang-bang controller with hysteresis
def rotate_to_ball(q_ball, q_basket, q_motors, game_event, stop_event):
    ball_x = 0
    # Hysteresis is the "deadzone" of our controller, that is, if the error is +/- hysteresis value,
    # the robot won't move.
    hysteresis = 5

    # The center of our camera image
    img_center = 320

    # Default speed for rotation
    rotation_speed = 0.06
    movement_speed = 0.3

    # Game state
    state = True

    # Main loop
    while True:
        # Check for stop signals
        if stop_event.is_set():
            if ser.is_open:
                ser.close()
            print("Closing rotate_to_ball..")
            return

        # if not q_ball.empty:
        ball_cords = q_ball.get()
        ball_x = ball_cords[0]
        ball_y = ball_cords[1]

        print("BALL_X =", ball_x)

        # Check to see whether we are on manual control or game logic
        if game_event.is_set():
            # Controller logic:
            # if ball is to our right, rotate right;
            # if it's to our left, rotate left;
            # if it's kind of in the middle, don't do anything (hysteresis)

            if ball_x > img_center :
                sign = 1;
            else:
                sign = -1;

            if ball_y > 384:
                if abs(img_center - ball_x) < hysteresis:
                    motors = [0, 0, 0]
                else:
                    motors = [0, 0, sign * rotation_speed]
            else:
                if abs(img_center - ball_x) > 200:
                    motors = [0, 0, sign*rotation_speed]
                elif abs(img_center - ball_x) < 200 and abs(img_center - ball_x) > hysteresis:
                    motors = [0, -movement_speed, sign*rotation_speed]
                else:
                    motors = [0, -movement_speed, 0]

            q_motors.put(motors)


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
