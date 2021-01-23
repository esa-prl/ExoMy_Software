from picamera import PiCamera
from time import sleep
'''
This script helps to test the functionality of the Raspberry Pi camera.
The script must be run dicrectly on the Raspberry Pi (not in the Docker container)
'''
camera = PiCamera()

camera.start_preview()
sleep(5)
camera.stop_preview()
