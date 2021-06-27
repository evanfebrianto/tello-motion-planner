# simple example demonstrating how to control a Tello using your keyboard.
# For a more fully featured example see manual-control-pygame.py
# 
# Use W, A, S, D for moving, E, Q for rotating and R, F for going up and down.
# When starting the script the Tello will takeoff, pressing ESC makes it land
#  and the script exit.

from djitellopy import Tello
from threading import Thread
from lib.keyboardModule import checkKey
from pynput import keyboard
import cv2, math, time

LINEAR_SPEED = 50
YAW_SPEED = 25
DATA_STREAM = True

def videoStreamer():
    while DATA_STREAM:
        img = frame_read.frame
        draw = cv2.resize(img, (320,192))
        cv2.imshow("drone", draw)
        cv2.waitKey(1)
    cv2.destroyAllWindows()

def getVelocityFromKeyboard():
    lr, fb, ud, yaw, isEsc = 0, 0, 0, 0, False
    
    _key = checkKey()

    if _key == 'a': lr = -LINEAR_SPEED
    elif _key == 'd': lr = LINEAR_SPEED

    if _key == 'w': fb = LINEAR_SPEED
    elif _key == 's': fb = -LINEAR_SPEED

    if _key == 'r': ud = LINEAR_SPEED
    elif _key == 'f': ud = -LINEAR_SPEED

    if _key == 'q': yaw = -YAW_SPEED
    elif _key == 'e': yaw = YAW_SPEED

    if _key == keyboard.Key.esc: isEsc = True

    return lr, fb, ud, yaw, isEsc
    

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

streamer = Thread(target=videoStreamer)
streamer.start()

tello.takeoff()

while True:
    lr, fb, ud, yaw, isEsc = getVelocityFromKeyboard()
    if not isEsc:
        tello.send_rc_control(lr, fb, ud, yaw)
        print(getVelocityFromKeyboard())
    else:
        break
    time.sleep(1/5)

tello.land()
DATA_STREAM = False
streamer.join()
