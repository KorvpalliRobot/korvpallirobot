import queue
import time
import pygame
import threading
import mainboard_comm


# Uncomment if you want to test the program by itself.
#q = queue.Queue()
#game_event = threading.Event()
#stop_event = threading.Event()

# Just a blank Exception class
class Closer(Exception):
    def __init__(self):
        print("Closing remote control..")


# Main program for parsing data from gamepad.
# Everything goes to a Queue which can be used in multithreading.
def gamepad(q, game_event, stop_event, lock):
    # Initialize PyGame
    pygame.init()

    # Use the first joystick/gamepad found
    j = pygame.joystick.Joystick(0)
    # Initialize the gamepad
    j.init()

    # x_speed, y_speed and rotation speed
    speeds = [0, 0, 0]

    # Set how many commands do we want to read before sending the data forward
    # because this code probably runs so fast that we don't need to send all this information to the controller.
    sequential_cmds = 4

    # How fast is the motor speed; we receive values between -1 and 1, so if we want faster speeds we have to multiply.
    # Maybe we could also change this with gamepad buttons?
    speed_multiplier = 0.3

    # Game logic state
    state = True

    # Try is used to make exiting the program easier, that is by using exceptions.
    try:
        # Main loop:
        while True:

            # Check for signals to stop
            # Check for stop signals
            if stop_event.is_set():
                raise Closer

            # Wait for some number of commands to be sent, it's inefficient to send data with every command.
            for i in range(sequential_cmds):
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.JOYAXISMOTION:
                        axis = event.axis
                        value = round(event.value, 1)


                        if axis == 0:
                            if not abs(value) == 0:
                                speeds[axis] = 0 - value * speed_multiplier
                            else:
                                speeds[axis] = 0
                        elif axis == 1:
                            if not abs(value) == 0:
                                speeds[axis] = value * speed_multiplier
                            else:
                                speeds[axis] = 0
                        elif axis <= 3:
                            if not abs(value) == 0:
                                speeds[2] = value * speed_multiplier
                            else:
                                speeds[2] = 0

                        # print("Value:", value)
                    #elif event.type == pygame.JOYBALLMOTION:
                        #print(event.dict, event.joy, event.ball, event.rel)
                    elif event.type == pygame.JOYBUTTONDOWN:
                        print(event.dict, event.joy, event.button, 'pressed')
                        # Close the gamepad program
                        if event.button == 9:
                            raise Closer

                        # Choose between manual control and game logic
                        if event.button == 8:
                            state = not state

                            if state is True:
                                game_event.set()
                            else:
                                game_event.clear()
                            print("Switched control..")

                        # Change the speed multiplier
                        if event.button == 7:
                            speed_multiplier += 0.1
                            print("Speed increased by 0.1..")
                        if event.button == 6:
                            speed_multiplier -= 0.1
                            print("Speed decreased by 0.1..")

                        if event.button == 2:
                            mainboard_comm.thrower_motor(250)
                            print("Thrower working.")
                    elif event.type == pygame.JOYBUTTONUP:
                        #print(event.dict, event.joy, event.button, 'released')

                        if event.button == 2:
                            mainboard_comm.thrower_motor(125)
                            print("Thrower stopped.")
                    #elif event.type == pygame.JOYHATMOTION:
                        #print(event.dict, event.joy, event.hat, event.value)

            # We wont need to check for commands as fast as we can..
            time.sleep(0.1)

            # Put the motor speeds into our queue, so that other threads can access it.
            # Only when in manual control
            if state is False:
                #lock.acquire()
                q.put(speeds)
                #lock.release()

            # Debug info
            #print("SPEEDS:", speeds)

    # In case this program being is run from command line
    except KeyboardInterrupt:
        print("EXITING NOW")
        j.quit()

    # Gamepad START button closes the program.
    except Closer:
        j.quit()

# Uncomment if you want to test the program by itself.
#gamepad(q, game_event, stop_event)
