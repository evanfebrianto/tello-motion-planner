from lib.utils import *
import cv2
import time
 
w,h = 640,480
if w == 640:
    pid = [0.2,0.2,0.1]
    pidA = [0.006,0.008,0.01]
else:
    pid = [0.4,0.4,0.1]
    pidA = [0.012,0.015,0.01]
pErrorX, pErrorY, pErrorA = 0, 0, 0
last_errorX,last_errorY,last_errorA = 0,0,0
startCounter = 0  # for no Flight 1   - for flight 0

 
myDrone = initializeTello()

# Full screen mode
cv2.namedWindow('Image', cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty('Image', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
out = cv2.VideoWriter('output.mp4',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (w,h))

while True:
 
    ## Flight
    if startCounter == 0:
        myDrone.takeoff()
        startCounter = 1
 
    ## Step 1
    img = telloGetFrame(myDrone,w,h)
    ## Step 2
    img, info = findFace(img)
    ## Step 3
    pErrorX, pErrorY, pErrorA,last_errorX,last_errorY,last_errorA = trackFace(myDrone,info,w,h,pid,pidA,pErrorX,pErrorY,pErrorA,last_errorX,last_errorY,last_errorA)
    if startCounter == 1:
        print(info[1])
    cv2.resize(img, (640,480))
    cv2.imshow('Image',img)
    out.write(img)

    if info[2] < 40.0:
        time.sleep(1)
        myDrone.land()
        out.release()
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        myDrone.land()
        out.release()
        break