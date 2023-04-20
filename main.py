from time import sleep
from pynput import keyboard
import serial
import threading
from pyrplidar import PyRPlidar
import math
import numpy as np


import cv2 as cv

# if using the picamera, import those libraries as well
from picamera.array import PiRGBArray
from picamera import PiCamera

# point to the haar cascade file in the directory
cascPath = "./haarcascades/haarcascade_frontalface_default.xml"
faceCascade = cv.CascadeClassifier(cascPath)

# #start the camera and define settings
camera = PiCamera()
camera.resolution = (640, 480)  # a smaller resolution means faster processing
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=camera.resolution)

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

is_light_on = False


def on_press(key):

    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
        if key.char == ('w'):
            print('sent')
            ser.write('motor:forward\n'.encode())

        if key.char == ('a'):
            ser.write('motor:left\n'.encode())

        if key.char == ('s'):
            print('sent')
            ser.write('motor:backward\n'.encode())

        if key.char == ('d'):
            print('sent')
            ser.write('motor:right\n'.encode())

        elif key.char == ('1'):
            print('sent')
            ser.write('motor:speed:50\n'.encode())

        elif key.char == ('2'):
            print('sent')
            ser.write('motor:speed:100\n'.encode())

        elif key.char == ('3'):
            print('sent')
            ser.write('motor:speed:150\n'.encode())

        elif key.char == ('4'):
            print('sent')
            ser.write('motor:speed:200\n'.encode())

        elif key.char == ('5'):
            print('sent')
            ser.write('motor:speed:255\n'.encode())

        elif key.char == ('p'):
            print('sent')
            ser.write('servo:armheight:150\n'.encode())

        elif key.char == ('o'):
            print('sent')
            ser.write('servo:armheight:90\n'.encode())

        elif key.char == ('l'):

            global is_light_on

            if is_light_on:
                ser.write('lights:front:0:0:0\n'.encode())
                ser.write('lights:back:0:0:0\n'.encode())
            if not is_light_on:
                ser.write('lights:front:255:255:255\n'.encode())
                ser.write('lights:back:255:0:0\n'.encode())

            is_light_on = not is_light_on

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

    for still in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True, resize=camera.resolution):
      # take the frame as an array, convert it to black and white, and look for facial features
        image = still.array

        # display the resulting image
        cv.imshow("Display", image)

        # clear the stream capture
        rawCapture.truncate(0)

        # set "q" as the key to exit the program when pressed
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            break


def robot_drive_code():
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


def lidar_code():

    lidar = PyRPlidar()

    lidar.connect(port="/dev/ttyUSB1", baudrate=115200, timeout=3)

    info = lidar.get_device_info()
    print(info)

    health = lidar.get_device_health()
    print(health)

    lidar.start_motor()

    lidar.start_scan()
    for count, scan in enumerate(lidar.iter_scans()):
        print(count, scan)
        if count == 20:
            break

    lidar.stop_scan()
    lidar.stop_motor()

    lidar.disconnect()


if __name__ == "__main__":

    thread_robot_control = threading.Thread(target=robot_drive_code)
    thread_opencv = threading.Thread(target=opencv_code)
    thread_lidar = threading.Thread(target=lidar_code)

    thread_lidar.start()
    thread_opencv.start()
    thread_robot_control.start()
