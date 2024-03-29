import threading
import detector
import mainboard_comm
import remote_control
import cv2
import numpy as np
import time
import queue


def main():
    # Ball's horizontal value (in pixels)
    q_ball = queue.Queue()
    # Basket's horizontal value (in pixels)
    q_basket = queue.Queue()
    # Holds motor speeds from every thread.
    q_motors = queue.LifoQueue(5)
    # Queue for thrower
    q_thrower = queue.Queue()
    # Motors Queue Lock
    lock = threading.Lock()
    # Holds the information about game logic state (game or manual)
    game_event = threading.Event()
    # By default this should be set so that the robot starts autonomously
    game_event.set()
    # Stop signal for all threads
    stop_event = threading.Event()

    # Returns the horizontal position of the ball
    thread_image_processing = threading.Thread(name="img", target=detector.main, args=(q_ball, q_basket, stop_event),
                                               daemon=True)
    # Returns motor speeds needed to rotate to ball
    thread_game_logic = threading.Thread(name="ball", target=mainboard_comm.angular_movement,
                                             args=(q_ball, q_basket, q_motors, game_event, stop_event, lock, q_thrower), daemon=True)
    # Controls all the motors
    thread_mainboard_comm = threading.Thread(name="comm", target=mainboard_comm.send,
                                             args=(q_motors, stop_event, lock, q_thrower), daemon=True)
    # Manual control
    thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(q_motors, game_event, stop_event, lock), daemon=True)

    # Start the threads
    thread_image_processing.start()
    thread_game_logic.start()
    thread_mainboard_comm.start()
    thread_manual_control.start()

    # The main loop for our program, use to display values etc
    while True:
        # Check for stop signals
        if stop_event.is_set():

            thread_image_processing.join()
            thread_game_logic.join()
            thread_mainboard_comm.join()
            thread_manual_control.join()

            print("Closing main.py..")
            return

        #print(q_ball.get())


main()
