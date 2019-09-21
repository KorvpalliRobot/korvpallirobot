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
    # To send a stop signal to every thread.
    q_stop = queue.Queue()
    # Holds motor speeds from every thread.
    q_motors = queue.Queue()
    # Holds the information about game logic state (game or manual)
    q_game = queue.Queue()

    # Returns the horizontal position of the ball
    thread_image_processing = threading.Thread(name="img", target=detector.main, args=(q_ball, q_basket, q_stop),
                                               daemon=True)
    # Returns motor speeds needed to rotate to ball
    thread_rotate_to_ball = threading.Thread(name="ball", target=mainboard_comm.rotate_to_ball_p,
                                             args=(q_ball, q_basket, q_motors, q_game, q_stop),
                                             daemon=True)
    # Controls all the motors
    thread_mainboard_comm = threading.Thread(name="comm", target=mainboard_comm.send, args=(q_motors, q_stop),
                                             daemon=True)
    # Manual control
    thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(q_motors, q_game, q_stop), daemon=True)

    # Start the threads
    thread_image_processing.start()
    thread_rotate_to_ball.start()
    thread_mainboard_comm.start()
    thread_manual_control.start()

    # The main loop for our program, use to display values etc
    while True:
        # Check for stop signals
        if not q_stop.empty():
            print("Closing main.py..")

            thread_image_processing.join()
            thread_rotate_to_ball.join()
            thread_mainboard_comm.join()
            thread_manual_control.join()

            return

        #print(q_ball.get())


main()
