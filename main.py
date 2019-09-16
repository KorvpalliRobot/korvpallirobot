import threading
import detector
import mainboard_comm
import cv2
import numpy as np
import time
import queue


def main():
    q_ball = queue.Queue()
    q_basket = queue.Queue()

    thread_image_processing = threading.Thread(name="img", target=detector.main, args=(q_ball, q_basket),
                                               daemon=True)
    thread_mainboard_comm = threading.Thread(name="comm", target=mainboard_comm.rotate_to_ball, args=q_ball,
                                             daemon=True)
    thread_image_processing.start()
    thread_mainboard_comm.start()

    while True:
        print(q_ball.get())


main()
