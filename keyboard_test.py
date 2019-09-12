import keyboard
import time
sleep = 0.5
while True:
    if keyboard.is_pressed('a'):
        print("A")
        time.sleep(sleep)
        continue
    if keyboard.is_pressed('s'):
        print("S")
        time.sleep(sleep)
        continue
    if keyboard.is_pressed('w'):
        print("W")
        time.sleep(sleep)
        continue
    if keyboard.is_pressed('d'):
        print("D")
        time.sleep(sleep)
        continue
