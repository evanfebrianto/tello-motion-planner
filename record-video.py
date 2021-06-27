import time, cv2
from threading import Thread
from djitellopy import Tello

tello = Tello()

tello.connect()

keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))
    # video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video
recorder = Thread(target=videoRecorder)
recorder.start()

print('Tello battery percentage: {}'.format(tello.get_battery()))
tello.takeoff()
# tello.move_left(20)
# tello.rotate_counter_clockwise(360)
time.sleep(3)

start = time.time()
while time.time() - start < 10: #second
    tello.send_rc_control(left_right_velocity=0,
                        forward_backward_velocity=0,
                        up_down_velocity=0,
                        yaw_velocity=30)

    # tello.curve_xyz_speed(x1=100, y1=100, z1=100,
    #                       x2=200, y2=200, z2=200,
    #                       speed=10)

tello.land()

keepRecording = False
recorder.join()
