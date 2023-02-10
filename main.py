from time import sleep
from pynput import keyboard
import serial

ser = serial.Serial('COM10', baudrate=115200, timeout=1)

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
