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

    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def read_serial_data():
    ser.flushInput()  # Clear the input buffer

    # Request and read battery voltage
    ser.write('read:battery\n'.encode())
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            if "Battery Voltage:" in line:
                battery_voltage = line.split(":")[1].strip()
                break

    # Request and read DHT data
    ser.write('read:dht\n'.encode())
    humidity, temperature = 'N/A', 'N/A'
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            if "humidity:" in line:
                try:
                    humidity_data, temperature_data = line.split('|')
                    humidity = humidity_data.split(':')[1].strip('%')
                    temperature = temperature_data.split(':')[1].strip('c')
                except ValueError:
                    print("Error parsing DHT data")
                break

    return battery_voltage, humidity, temperature


def opencv_code():
    # give camera time to warm up
    sleep(0.1)

    for still in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True, resize=camera.resolution):
        # take the frame as an array, convert it to black and white, and look for facial features
        image = still.array

        # Read telemetry data
        battery_voltage, humidity, temperature = read_serial_data()

        # Display HUD
        cv.putText(image, f'Battery: {battery_voltage}V', (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv.putText(image, f'Humidity: {humidity}%', (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv.putText(image, f'Temp: {temperature}C', (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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


    sleep(2)



    # Initialize the OpenCV window and frame
    cv.namedWindow("RPLidar", cv.WINDOW_NORMAL)
    cv.resizeWindow("RPLidar", 400, 400)
    frame = 255 * np.zeros((400, 400, 3), dtype=np.uint8)


    while True:
        lidar.connect(port="/dev/ttyUSB1", baudrate=115200, timeout=3)
        lidar.set_motor_pwm(500)

        print(lidar.get_health())
        print(lidar.get_info())
        print(lidar.get_samplerate())

        scan_generator = lidar.start_scan_express(4)
        sleep(0.5)
        for count, scan in enumerate(scan_generator()):
            print(scan)
            x = int(scan.distance * np.cos(np.radians(scan.angle)))
            y = int(scan.distance * np.sin(np.radians(scan.angle)))
            cv.circle(frame, (int(x/10) + 200, int(y/10) + 200),
                    2, (0, 255, 0), -1)

            # Display the frame
            if count % 10 == 0:
                cv.imshow("RPLidar", frame)
                cv.waitKey(1)


            if count > 4000:
                frame = 255 * np.zeros((400, 400, 3), dtype=np.uint8)
                break

        lidar.stop()
        lidar.disconnect()
        sleep(2)

    #cv.destroyAllWindows()


if __name__ == "__main__":

    thread_robot_control = threading.Thread(target=robot_drive_code)
    thread_opencv = threading.Thread(target=opencv_code)
    thread_lidar = threading.Thread(target=lidar_code)

    thread_lidar.start()
    thread_opencv.start()
    thread_robot_control.start()
