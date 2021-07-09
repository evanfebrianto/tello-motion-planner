from djitellopy import Tello
import cv2
import numpy as np
import mediapipe as mp
from lib.helper import get_bb
 
def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone
 
def telloGetFrame(myDrone, w= 360,h=240):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv2.resize(myFrame,(w,h))
    return img
 
def findFace(img):
    mp_face_detection = mp.solutions.face_detection
    with mp_face_detection.FaceDetection(
            model_selection=1, 
            min_detection_confidence=0.8) as face_detection:
        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        img = cv2.cvtColor(cv2.flip(img, 1), cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        img.flags.writeable = False
        results = face_detection.process(img)

        # Draw the face mesh annotations on the image.
        img.flags.writeable = True
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        myFaceListC = []
        myFaceListArea = []
        if results.detections:
            for detection in results.detections:
                cx_min, cy_min, cx_max, cy_max = get_bb(img, detection)
                if cx_min is not None:
                    dx, dy = (cx_max-cx_min, cy_max-cy_min)
                    cx = cx_min + dx//2
                    cy = cy_min + dy//2
                    area = dx * dy
                    myFaceListArea.append(area)
                    myFaceListC.append([cx,cy])
                    cv2.rectangle(img, (cx_min, cy_min), (cx_max, cy_max), (0, 255, 0), 2)
        if len(myFaceListArea) !=0:
            i = myFaceListArea.index(max(myFaceListArea))
            return img, [myFaceListC[i],myFaceListArea[i]]
        else:
            return img,[[0,0],0]

def trackFace(myDrone,info,w,h,pid,pError):
 
    ## PID
    error = info[0][0] - w//2
    error_y = info[0][1] - h//2
    speed = pid[0]*error + pid[1]*(error-pError)
    speed = int(np.clip(speed,-100,100))
 
 
    print(speed)
    if info[0][0] !=0:
        myDrone.yaw_velocity = -speed
        myDrone.up_down_velocity = -(int(np.clip(0.7*error_y,-100,100)))
    else:
        myDrone.for_back_velocity = 0
        myDrone.left_right_velocity = 0
        myDrone.up_down_velocity = 0
        myDrone.yaw_velocity = 0
        error = 0
    if myDrone.send_rc_control:
        myDrone.send_rc_control(myDrone.left_right_velocity,
                                myDrone.for_back_velocity,
                                myDrone.up_down_velocity,
                                myDrone.yaw_velocity)
    return error