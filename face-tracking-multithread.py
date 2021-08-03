from djitellopy import Tello
from threading import Thread
import sys, os, time
from lib.detector import Detector
from lib.pid import PID

THRESH_HAND2FACE = 40
THRESH_FACEBB = 100
DEBUG = True

if __name__ == '__main__':
    try:
        # connect tello drone
        tello = Tello()
        tello.connect()

        batt = tello.get_battery()
        if batt > 30:
            # start streaming
            tello.streamon()

            # declare pid x,y,z
            pidX = PID(Kp=0.0, Ki=0, Kd=0)
            pidY = PID(Kp=0.0, Ki=0, Kd=0)
            pidZ = PID(Kp=0.0, Ki=0, Kd=0.0)
            pidYaw = PID(Kp=0.04, Ki=0.001, Kd=0.1)
            
            # start mission 
            tracker = Detector(tello_instance=tello,
                distance_thresh=THRESH_HAND2FACE,
                bb_thresh=THRESH_FACEBB,
                pid_x=pidX,
                pid_y=pidY,
                pid_z=pidZ,
                pid_yaw=pidYaw)

            # start detection, stream, and record video in different thread
            detector = Thread(target=tracker.faceHandDetector)
            recorder = Thread(target=tracker.videoRecorder)
            streamer = Thread(target=tracker.videoStreamer)
            controller = Thread(target=tracker.controller)
            detector.start()
            recorder.start()
            streamer.start()
            controller.start()

            if not DEBUG: tracker.takeoff()
            if not tracker.isFailed: tracker.isLandingCondition = False
            while not tracker.isLandingCondition:
                # keep program running until it detects hand
                if DEBUG:
                    print('face: {}'.format(tracker.face))
                    print('hand: {}'.format(tracker.thumb_tip))
                    print('distance: {}'.format(tracker.distance))
                    print('isLanding: {}\n'.format(tracker.isLandingCondition))
            
            tracker.land()

            # close all active threads
            tracker.isThreadActive = False
            detector.join()
            recorder.join()
            streamer.join()
            controller.join()
            tracker.end()
        else:
            print("Low battery! Only {}% left, please change!".format(batt))
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            # close all active threads
            tracker.isThreadActive = False
            detector.join()
            recorder.join()
            streamer.join()
            controller.join()
            tracker.end()
            sys.exit(0)
        except SystemExit:
            os._exit(0)
