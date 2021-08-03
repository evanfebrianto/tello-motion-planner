# Use W, A, S, D for moving, E, Q for rotating and R, F for going up and down.
# When starting the script the Tello will takeoff, pressing ESC makes it land
#  and the script exit.

from djitellopy import Tello
from threading import Thread
import pygame
import cv2, math, time

LINEAR_SPEED = 20
YAW_SPEED = 25
DATA_STREAM = True
keepRecording = True

def videoRecorder():
    # create a VideoWrite object, recoring to ./recording.mp4
    width, height = 1280,720
    video = cv2.VideoWriter('recording.mp4',cv2.VideoWriter_fourcc('M','J','P','G'), 25, (width,height))
    while keepRecording:
        img = frame_read.frame.copy()
        img = cv2.resize(img, (width, height))
        video.write(img)
        time.sleep(1 / 30)
    video.release()


def videoStreamer():
    while DATA_STREAM:
        img = frame_read.frame
        draw = img.copy()
        draw = cv2.resize(draw, (1280,720))
        cv2.imshow("drone", draw)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


def get_input():
    pygame.event.pump()
    pressed = pygame.key.get_pressed()
    q = pressed[pygame.K_q]
    w = pressed[pygame.K_w]
    e = pressed[pygame.K_e]
    r = pressed[pygame.K_r]
    a = pressed[pygame.K_a]
    s = pressed[pygame.K_s]
    d = pressed[pygame.K_d]
    f = pressed[pygame.K_f]
    x = pressed[pygame.K_x]
    esc = pressed[pygame.K_ESCAPE]
    return q, w, e, r, a, s, d, f, x, esc

def getVelocityFromKeyboard():
    lr, fb, ud, yaw = 0, 0, 0, 0
    
    q, w, e, r, a, s, d, f, x, esc = get_input()

    if a: lr = -LINEAR_SPEED
    elif d: lr = LINEAR_SPEED

    if w: fb = LINEAR_SPEED
    elif s: fb = -LINEAR_SPEED
    elif x: fb = -100

    if r: ud = LINEAR_SPEED
    elif f: ud = -LINEAR_SPEED

    if q: yaw = -YAW_SPEED
    elif e: yaw = YAW_SPEED

    return lr, fb, ud, yaw, esc

def init():
    pygame.init()
    size = width, height = 500,500
    pygame.display.set_mode(size)

tello = Tello()
tello.connect()
init()

tello.streamon()
frame_read = tello.get_frame_read()

recorder = Thread(target=videoRecorder)
streamer = Thread(target=videoStreamer)
recorder.start()
streamer.start()


tello.takeoff()

while True:
    lr, fb, ud, yaw, isEsc = getVelocityFromKeyboard()
    if not isEsc:
        tello.send_rc_control(lr, fb, ud, yaw)
    else:
        break

tello.land()
keepRecording = False
DATA_STREAM = False
recorder.join()
streamer.join()