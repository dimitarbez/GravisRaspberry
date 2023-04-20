from time import sleep
from pynput import keyboard
import serial
import threading


import cv2 as cv
from face_tracker.face_tracker import FaceTracker

# if using the picamera, import those libraries as well
from picamera.array import PiRGBArray
from picamera import PiCamera

# point to the haar cascade file in the directory
cascPath = "./haarcascades/haarcascade_frontalface_default.xml"
faceCascade = cv.CascadeClassifier(cascPath)

# #start the camera and define settings
camera = PiCamera()
camera.resolution = (400, 300)  # a smaller resolution means faster processing
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(400, 300))

# set the distance between the edge of the screen
# and the borders that trigger robot rotation
side_borders_distance = 150

# face tracking area thresholds are used for forward/backward movement of the robot
# max square area threshold for face tracking
max_face_tracking_width = 150
# min square area threshold for face tracking
min_face_tracking_width = 120

tracked_face_color = (0, 255, 0)
side_border_color = (0, 0, 255)


ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)


def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
        if key.char == ('w'):
            print('sent')
            ser.write('motor:forward\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())
            print('sent')

        if key.char == ('a'):
            ser.write('motor:left\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        if key.char == ('s'):
            print('sent')
            ser.write('motor:backward\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        if key.char == ('d'):
            print('sent')
            ser.write('motor:right\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        elif key.char == ('1'):
            print('sent')
            ser.write('motor:speed:80\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        elif key.char == ('2'):
            print('sent')
            ser.write('motor:speed:120\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        elif key.char == ('3'):
            print('sent')
            ser.write('motor:speed:150\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        elif key.char == ('p'):
            print('sent')
            ser.write('servo:armheight:150\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

        elif key.char == ('o'):
            print('sent')
            ser.write('servo:armheight:90\n'.encode())
            data = ser.read(size=3).decode()
            print(data)
            print(data.decode())

    except AttributeError:
        print('special key {0} pressed'.format(
            key))


def on_release(key):

    ser.write('motor:stop\n'.encode())
    data = ser.read(size=3).decode()
    print(data)

    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    

def opencv_code():
    # give camera time to warm up
    sleep(0.1)

    face_tracker = FaceTracker()

    for still in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
      # take the frame as an array, convert it to black and white, and look for facial features
        image = still.array
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        face = face_tracker.get_face_from_image(gray, faceCascade)
        if len(face) == 0:
            continue

        face_x = face[0]
        face_y = face[1]
        face_width = face[2]
        face_height = face[3]

        # formula for object position
        object_in_right_area = face_tracker.is_face_in_right_region(
            face, image)
        object_in_left_area = face_tracker.is_face_in_left_region(face)

        cv.rectangle(image, (face_x, face_y), (face_x+face_width,
                     face_y+face_height), tracked_face_color, 2)

        # within the left region
        if object_in_left_area and not object_in_right_area:

            print('left')

        # within the right region
        elif object_in_right_area and not object_in_left_area:
            print('right')

        elif (object_in_left_area and object_in_right_area) or not (object_in_left_area and object_in_right_area):
            print('center')
            if face_width > max_face_tracking_width:
                print('move backward')

            elif face_width < min_face_tracking_width:
                print('move forward')

            else:
                print('stop')

        # draw side borders
        cv.line(image, (side_borders_distance, 0),
                (side_borders_distance, image.shape[0]), side_border_color, 5)
        cv.line(image, (image.shape[1] - side_borders_distance, 0),
                (image.shape[1] - side_borders_distance, image.shape[0]), side_border_color, 5)

        # display the resulting image
        cv.imshow("Display", image)

        # clear the stream capture
        rawCapture.truncate(0)

        # set "q" as the key to exit the program when pressed
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            break




if __name__ == "__main__":

    # Collect events until released
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()

    # ...or, in a non-blocking fashion:
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()


    thread_opencv = threading.Thread(target=opencv_code)
    thread_opencv.start()


    